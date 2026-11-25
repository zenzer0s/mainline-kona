// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP/Qualcomm NCI NFC I2C character device driver (/dev/nq-nci)
 * for Android NXP SNxxx / PN5xx NFC HALs.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define NQ_NCI_MAX_BUFFER_SIZE		554
#define NQ_NCI_GPIO_DELAY_MS		10

#define NFC_MAGIC			0xE9
#define NFC_SET_PWR			_IOW(NFC_MAGIC, 0x01, u32)
#define ESE_SET_PWR			_IOW(NFC_MAGIC, 0x02, u32)
#define ESE_GET_PWR			_IOR(NFC_MAGIC, 0x03, u32)
#define NFC_SET_RESET_READ_PENDING	_IOW(NFC_MAGIC, 0x04, u32)
#define NFC_GET_GPIO_STATUS		_IOR(NFC_MAGIC, 0x05, u32)

enum nq_nci_pwr_mode {
	MODE_POWER_OFF = 0,
	MODE_POWER_ON,
	MODE_FW_DWNLD_WITH_VEN,
	MODE_ISO_RST,
	MODE_FW_DWND_HIGH,
	MODE_POWER_RESET,
	MODE_FW_GPIO_LOW,
};

struct nq_nci_dev {
	struct i2c_client *client;
	struct miscdevice miscdev;
	struct gpio_desc *ven_gpiod;
	struct gpio_desc *fw_gpiod;
	struct mutex read_mutex;
	struct mutex write_mutex;
	spinlock_t irq_lock;
	wait_queue_head_t read_wq;
	bool irq_enabled;
	bool irq_pending;
	bool read_pending;
	u8 rx_buf[NQ_NCI_MAX_BUFFER_SIZE];
	u8 tx_buf[NQ_NCI_MAX_BUFFER_SIZE];
};

static void nq_nci_enable_irq(struct nq_nci_dev *nq)
{
	unsigned long flags;

	spin_lock_irqsave(&nq->irq_lock, flags);
	if (!nq->irq_enabled) {
		nq->irq_enabled = true;
		enable_irq(nq->client->irq);
	}
	spin_unlock_irqrestore(&nq->irq_lock, flags);
}

static void nq_nci_disable_irq(struct nq_nci_dev *nq)
{
	unsigned long flags;

	spin_lock_irqsave(&nq->irq_lock, flags);
	if (nq->irq_enabled) {
		disable_irq_nosync(nq->client->irq);
		nq->irq_enabled = false;
	}
	spin_unlock_irqrestore(&nq->irq_lock, flags);
}

static irqreturn_t nq_nci_irq_handler(int irq, void *dev_id)
{
	struct nq_nci_dev *nq = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&nq->irq_lock, flags);
	if (nq->irq_enabled) {
		disable_irq_nosync(irq);
		nq->irq_enabled = false;
	}
	nq->irq_pending = true;
	spin_unlock_irqrestore(&nq->irq_lock, flags);

	wake_up_interruptible(&nq->read_wq);
	return IRQ_HANDLED;
}

static int nq_nci_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct nq_nci_dev *nq = container_of(misc, struct nq_nci_dev, miscdev);

	filp->private_data = nq;
	return 0;
}

static int nq_nci_release(struct inode *inode, struct file *filp)
{
	struct nq_nci_dev *nq = filp->private_data;

	nq_nci_disable_irq(nq);
	return 0;
}

static __poll_t nq_nci_poll(struct file *filp, poll_table *wait)
{
	struct nq_nci_dev *nq = filp->private_data;
	unsigned long flags;
	bool ready;

	poll_wait(filp, &nq->read_wq, wait);

	spin_lock_irqsave(&nq->irq_lock, flags);
	ready = nq->irq_pending;
	if (!ready && !nq->irq_enabled) {
		nq->irq_enabled = true;
		enable_irq(nq->client->irq);
	}
	spin_unlock_irqrestore(&nq->irq_lock, flags);

	return ready ? (EPOLLIN | EPOLLRDNORM) : 0;
}

static ssize_t nq_nci_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *offset)
{
	struct nq_nci_dev *nq = filp->private_data;
	unsigned long flags;
	int ret;

	if (count == 0)
		return 0;
	if (count > NQ_NCI_MAX_BUFFER_SIZE)
		count = NQ_NCI_MAX_BUFFER_SIZE;

	mutex_lock(&nq->read_mutex);

	if (!(filp->f_flags & O_NONBLOCK)) {
		spin_lock_irqsave(&nq->irq_lock, flags);
		if (!nq->irq_pending && !nq->irq_enabled) {
			nq->irq_enabled = true;
			enable_irq(nq->client->irq);
		}
		spin_unlock_irqrestore(&nq->irq_lock, flags);

		ret = wait_event_interruptible(nq->read_wq, nq->irq_pending);
		if (ret) {
			mutex_unlock(&nq->read_mutex);
			return ret;
		}
	}

	ret = i2c_master_recv(nq->client, nq->rx_buf, count);
	if (ret < 0) {
		dev_err(&nq->client->dev, "i2c_master_recv failed: %d\n", ret);
		spin_lock_irqsave(&nq->irq_lock, flags);
		nq->irq_pending = false;
		spin_unlock_irqrestore(&nq->irq_lock, flags);
		mutex_unlock(&nq->read_mutex);
		return ret;
	}

	/*
	 * Clear irq_pending after reading payload or full frame; if the HAL
	 * issues a second read for the payload right after the 3-byte header,
	 * keep irq_pending set until the payload read completes or the next
	 * select/poll re-arms the level IRQ.
	 */
	spin_lock_irqsave(&nq->irq_lock, flags);
	if (count > 3)
		nq->irq_pending = false;
	spin_unlock_irqrestore(&nq->irq_lock, flags);

	if (copy_to_user(buf, nq->rx_buf, ret)) {
		mutex_unlock(&nq->read_mutex);
		return -EFAULT;
	}

	mutex_unlock(&nq->read_mutex);
	return ret;
}

static ssize_t nq_nci_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *offset)
{
	struct nq_nci_dev *nq = filp->private_data;
	int ret;

	if (count == 0)
		return 0;
	if (count > NQ_NCI_MAX_BUFFER_SIZE)
		count = NQ_NCI_MAX_BUFFER_SIZE;

	mutex_lock(&nq->write_mutex);

	if (copy_from_user(nq->tx_buf, buf, count)) {
		mutex_unlock(&nq->write_mutex);
		return -EFAULT;
	}

	ret = i2c_master_send(nq->client, nq->tx_buf, count);
	if (ret != (int)count) {
		dev_err(&nq->client->dev, "i2c_master_send failed: %d\n", ret);
		if (ret >= 0)
			ret = -EIO;
	}

	mutex_unlock(&nq->write_mutex);
	return ret;
}

static void nq_nci_set_pwr(struct nq_nci_dev *nq, unsigned long mode)
{
	unsigned long flags;

	switch (mode) {
	case MODE_POWER_OFF:
		nq_nci_disable_irq(nq);
		gpiod_set_value_cansleep(nq->fw_gpiod, 0);
		gpiod_set_value_cansleep(nq->ven_gpiod, 0);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		break;
	case MODE_POWER_ON:
		gpiod_set_value_cansleep(nq->fw_gpiod, 0);
		gpiod_set_value_cansleep(nq->ven_gpiod, 1);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		spin_lock_irqsave(&nq->irq_lock, flags);
		nq->irq_pending = false;
		spin_unlock_irqrestore(&nq->irq_lock, flags);
		nq_nci_enable_irq(nq);
		break;
	case MODE_FW_DWNLD_WITH_VEN:
		gpiod_set_value_cansleep(nq->ven_gpiod, 1);
		gpiod_set_value_cansleep(nq->fw_gpiod, 1);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		gpiod_set_value_cansleep(nq->ven_gpiod, 0);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		gpiod_set_value_cansleep(nq->ven_gpiod, 1);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		break;
	case MODE_ISO_RST:
	case MODE_POWER_RESET:
		gpiod_set_value_cansleep(nq->ven_gpiod, 0);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		gpiod_set_value_cansleep(nq->ven_gpiod, 1);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		break;
	case MODE_FW_DWND_HIGH:
		gpiod_set_value_cansleep(nq->fw_gpiod, 1);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		break;
	case MODE_FW_GPIO_LOW:
		gpiod_set_value_cansleep(nq->fw_gpiod, 0);
		msleep(NQ_NCI_GPIO_DELAY_MS);
		break;
	default:
		dev_warn(&nq->client->dev, "unknown NFC_SET_PWR mode: %lu\n", mode);
		break;
	}
}

static long nq_nci_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct nq_nci_dev *nq = filp->private_data;

	switch (cmd) {
	case NFC_SET_PWR:
		nq_nci_set_pwr(nq, arg);
		return 0;
	case ESE_SET_PWR:
	case ESE_GET_PWR:
		return 0;
	case NFC_SET_RESET_READ_PENDING:
		nq->read_pending = !!arg;
		return 0;
	case NFC_GET_GPIO_STATUS:
		return 0;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations nq_nci_fops = {
	.owner = THIS_MODULE,
	.open = nq_nci_open,
	.release = nq_nci_release,
	.poll = nq_nci_poll,
	.read = nq_nci_read,
	.write = nq_nci_write,
	.unlocked_ioctl = nq_nci_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int nq_nci_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct nq_nci_dev *nq;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	nq = devm_kzalloc(dev, sizeof(*nq), GFP_KERNEL);
	if (!nq)
		return -ENOMEM;

	nq->client = client;
	mutex_init(&nq->read_mutex);
	mutex_init(&nq->write_mutex);
	spin_lock_init(&nq->irq_lock);
	init_waitqueue_head(&nq->read_wq);

	nq->ven_gpiod = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(nq->ven_gpiod))
		return dev_err_probe(dev, PTR_ERR(nq->ven_gpiod),
				     "failed to get enable gpio\n");

	nq->fw_gpiod = devm_gpiod_get_optional(dev, "firmware", GPIOD_OUT_LOW);
	if (IS_ERR(nq->fw_gpiod))
		return dev_err_probe(dev, PTR_ERR(nq->fw_gpiod),
				     "failed to get firmware gpio\n");

	nq->miscdev.minor = MISC_DYNAMIC_MINOR;
	nq->miscdev.name = "nq-nci";
	nq->miscdev.fops = &nq_nci_fops;

	ret = misc_register(&nq->miscdev);
	if (ret) {
		dev_err(dev, "misc_register failed: %d\n", ret);
		return ret;
	}

	ret = devm_request_irq(dev, client->irq, nq_nci_irq_handler,
			       IRQF_TRIGGER_HIGH | IRQF_NO_AUTOEN,
			       "nq-nci", nq);
	if (ret) {
		misc_deregister(&nq->miscdev);
		return dev_err_probe(dev, ret, "failed to request irq %d\n",
				     client->irq);
	}

	i2c_set_clientdata(client, nq);
	dev_info(dev, "registered /dev/nq-nci (irq %d)\n", client->irq);
	return 0;
}

static void nq_nci_remove(struct i2c_client *client)
{
	struct nq_nci_dev *nq = i2c_get_clientdata(client);

	nq_nci_disable_irq(nq);
	misc_deregister(&nq->miscdev);
	gpiod_set_value_cansleep(nq->ven_gpiod, 0);
	gpiod_set_value_cansleep(nq->fw_gpiod, 0);
}

static const struct of_device_id nq_nci_of_match[] = {
	{ .compatible = "nxp,nxp-nci-i2c" },
	{ .compatible = "qcom,nq-nci" },
	{ }
};
MODULE_DEVICE_TABLE(of, nq_nci_of_match);

static const struct i2c_device_id nq_nci_i2c_id[] = {
	{ "nq-nci" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nq_nci_i2c_id);

static struct i2c_driver nq_nci_driver = {
	.driver = {
		.name = "nq-nci",
		.of_match_table = nq_nci_of_match,
	},
	.probe = nq_nci_probe,
	.remove = nq_nci_remove,
	.id_table = nq_nci_i2c_id,
};
module_i2c_driver(nq_nci_driver);

MODULE_DESCRIPTION("NXP/Qualcomm NCI NFC I2C character device driver");
MODULE_LICENSE("GPL");
