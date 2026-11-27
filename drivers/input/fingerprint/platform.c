/*
 * platform indepent driver interface
 *
 * Coypritht (c) 2017 Goodix
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

/* The DT names these "fp-gpio-*" without the -gpios suffix gpiod lookups need */
static int gf_of_get_gpio(struct device_node *np, const char *prop)
{
	struct of_phandle_args args;
	struct gpio_device *gdev;
	struct gpio_desc *desc;
	int gpio;

	if (of_parse_phandle_with_args(np, prop, "#gpio-cells", 0, &args))
		return -ENOENT;
	gdev = gpio_device_find_by_fwnode(of_fwnode_handle(args.np));
	of_node_put(args.np);
	if (!gdev)
		return -EPROBE_DEFER;
	desc = gpio_device_get_desc(gdev, args.args[0]);
	gpio = IS_ERR(desc) ? PTR_ERR(desc) : desc_to_gpio(desc);
	gpio_device_put(gdev);
	return gpio;
}

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	gf_dev->reset_gpio = gf_of_get_gpio(np, "fp-gpio-reset");
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);

	gf_dev->irq_gpio = gf_of_get_gpio(np, "fp-gpio-irq");
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	gf_dev->vdd =  regulator_get(dev, "vdd");
	if (IS_ERR(gf_dev->vdd)) {
		rc = PTR_ERR(gf_dev->vdd);
		pr_err("Regulator get failed vdd rc = %d\n", rc);
		goto err_pwr;
	}

	rc = of_property_read_u32(np, "vdd-max-uv", &gf_dev->vdd_max_uv);
	if(rc){
		pr_err("fail to get vdd_max_uv\n");
		goto err_reg;
	}

	rc = of_property_read_u32(np, "vdd-min-uv", &gf_dev->vdd_min_uv);
	if(rc){
		pr_err("fail to get vdd_min_uv\n");
		goto err_reg;
	}

	rc = regulator_set_voltage(gf_dev->vdd,gf_dev->vdd_min_uv, gf_dev->vdd_max_uv);
	if (rc) {
		pr_err("Regulator set voltage failed rc=%d\n", rc);
		goto err_reg;
	}

	gf_dev->fp_id = -1;
	gf_dev->id_gpio = gf_of_get_gpio(np, "fp-gpio-id");
	if (gf_dev->id_gpio >= 0) {
		if (!gpio_request(gf_dev->id_gpio, "goodix_fp_id")) {
			gpio_direction_input(gf_dev->id_gpio);
			gf_dev->fp_id = gpio_get_value(gf_dev->id_gpio);
			pr_info("goodix_fp: fp_id gpio=%d value=%d\n",
				gf_dev->id_gpio, gf_dev->fp_id);
		} else {
			gf_dev->id_gpio = -EINVAL;
		}
	}

	return rc;

err_reg:
	regulator_put(gf_dev->vdd);
err_pwr:
	gf_dev->vdd = NULL;
	gpio_free(gf_dev->irq_gpio);
err_irq:
	gpio_free(gf_dev->reset_gpio);
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	gf_dev->irq_gpio = -EINVAL;
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
	gf_dev->reset_gpio = -EINVAL;
	if (gpio_is_valid(gf_dev->id_gpio)) {
		gpio_free(gf_dev->id_gpio);
		gf_dev->id_gpio = -EINVAL;
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(gf_dev->vdd))
		return -ENODEV;
	if (!gf_dev->vdd_on) {
		regulator_set_load(gf_dev->vdd, 150000);
		rc = regulator_enable(gf_dev->vdd);
		if (rc) {
			pr_err("Regulator enable failed rc=%d\n", rc);
			goto err_reg;
		}
		gf_dev->vdd_on = true;
		msleep(30);
	}
	return rc;
err_reg:
	return rc;
}

int gf_set_vdd_lpm(struct gf_dev *gf_dev, bool lpm)
{
	if (IS_ERR_OR_NULL(gf_dev->vdd) || !gf_dev->vdd_on)
		return 0;

	return regulator_set_load(gf_dev->vdd, lpm ? 1000 : 150000);
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	/* Only drop the vote we took: the HAL powers off sensors it never powered on */
	if (!IS_ERR_OR_NULL(gf_dev->vdd) && gf_dev->vdd_on) {
		rc = regulator_disable(gf_dev->vdd);
		gf_dev->vdd_on = false;
		msleep(30);
	}

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_set_value(gf_dev->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpio_set_value(gf_dev->reset_gpio, 1);
	if (delay_ms)
		mdelay(delay_ms);
	return 0;
}

int gf_power_reset(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	}
	gpio_set_value(gf_dev->reset_gpio, 0);
	gf_power_off(gf_dev);
	msleep(50);
	gf_power_on(gf_dev);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(3);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -1;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}
