// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 The PenguinOS Project
 *
 * Userspace access to QSEE apps that QTEE does not expose as QSEECom compat
 * objects (keymaster64), through the legacy app lookup and send SCM calls, and
 * the legacy listeners (RPMB, secure storage) those apps call back into.
 */
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/qcom_tzmem.h>
#include <linux/dma-mapping.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>

#include "qcom_scm.h"

#define QSEE_BRIDGE_MAX_BUFS	4
#define QSEE_BRIDGE_MAX_LEN	SZ_4M

struct qsee_bridge_lookup {
	char name[64];
	__u32 app_id;
};

struct qsee_bridge_buf {
	__u64 ptr;
	__u32 len;
	__u32 offset;
};

struct qsee_bridge_send {
	__u32 app_id;
	__u32 n_bufs;
	__u64 req;
	__u64 rsp;
	__u32 req_len;
	__u32 rsp_len;
	__u32 addr64;
	__u32 pad;
	struct qsee_bridge_buf bufs[QSEE_BRIDGE_MAX_BUFS];
};

struct qsee_bridge_listener_reg {
	__u32 id;
	__u32 sb_len;
};

struct qsee_bridge_listener_buf {
	__u64 ptr;
	__u32 len;
	__u32 pad;
};

#define QSEE_BRIDGE_IOC_LOOKUP		_IOWR('Q', 0x40, struct qsee_bridge_lookup)
#define QSEE_BRIDGE_IOC_SEND		_IOW('Q', 0x41, struct qsee_bridge_send)
#define QSEE_BRIDGE_IOC_LISTENER_REG	_IOW('Q', 0x42, struct qsee_bridge_listener_reg)
#define QSEE_BRIDGE_IOC_LISTENER_RECV	_IOW('Q', 0x43, struct qsee_bridge_listener_buf)
#define QSEE_BRIDGE_IOC_LISTENER_RESP	_IOW('Q', 0x44, struct qsee_bridge_listener_buf)

#define QSEE_OWNER_TZ_APPS		48
#define QSEE_OWNER_QSEE_OS		50
#define QSEE_SVC_APP_ID_PLACEHOLDER	0
#define QSEE_SVC_LISTENER		2
#define QSEE_CMD_APP_SEND		1
#define QSEE_CMD_LISTENER_REG		1
#define QSEE_CMD_LISTENER_DEREG		2
#define QSEE_CMD_LISTENER_RESP		3
#define QSEE_CMD_LISTENER_REG_SMCINVOKE	6

#define QSEE_RESULT_SUCCESS		0
#define QSEE_RESULT_INCOMPLETE		1
#define QSEE_RESULT_BLOCKED_ON_LISTENER	2
#define QSEE_RESULT_FAILURE		0xffffffff

#define QSEE_LISTENER_TIMEOUT		(30 * HZ)

struct qsee_listener {
	struct list_head node;
	struct kref ref;
	u32 id;
	void *sb;
	dma_addr_t sb_phys;
	size_t sb_len;
	u64 sb_handle;
	wait_queue_head_t wq;
	bool req_pending;
	bool rsp_ready;
	bool dead;
};

/* One app command, with the listener round trips it causes, at a time */
static DEFINE_MUTEX(qsee_bridge_send_lock);
static LIST_HEAD(qsee_listeners);
static DEFINE_MUTEX(qsee_listeners_lock);

static struct miscdevice qsee_bridge_dev;

static void qsee_listener_free(struct kref *ref)
{
	struct qsee_listener *l = container_of(ref, struct qsee_listener, ref);

	if (l->sb) {
		if (l->sb_handle)
			qcom_tzmem_shm_bridge_delete(l->sb_handle);
		dma_unmap_single(qsee_bridge_dev.this_device, l->sb_phys, l->sb_len,
				 DMA_BIDIRECTIONAL);
		free_pages_exact(l->sb, l->sb_len);
	}
	kfree(l);
}

/* Hands the request TZ left in the listener's buffer to its process and waits for the reply */
static int qsee_listener_serve(u32 id)
{
	struct qsee_listener *l = NULL, *it;
	long left;
	int ret = 0;

	mutex_lock(&qsee_listeners_lock);
	list_for_each_entry(it, &qsee_listeners, node) {
		if (it->id == id && !it->dead) {
			l = it;
			kref_get(&l->ref);
			l->rsp_ready = false;
			l->req_pending = true;
			break;
		}
	}
	mutex_unlock(&qsee_listeners_lock);

	if (!l) {
		pr_warn_ratelimited("qseecom_bridge: no listener %#x\n", id);
		return -ENOENT;
	}

	wake_up_all(&l->wq);
	left = wait_event_killable_timeout(l->wq, l->rsp_ready || l->dead,
					   QSEE_LISTENER_TIMEOUT);
	if (left <= 0 || l->dead) {
		pr_warn("qseecom_bridge: listener %#x did not answer\n", id);
		ret = -ETIMEDOUT;
	}
	l->req_pending = false;
	kref_put(&l->ref, qsee_listener_free);
	return ret;
}

/*
 * A QTEE invocation returned a listener request (as stock's smcinvoke driver hands to
 * qseecom): serve it and resume the invocation. @res holds the invocation's response
 * type, result and listener id on entry, and the resumed invocation's on return.
 */
int qsee_bridge_qtee_listener(u64 res[3])
{
	u64 args[2];
	int ret;

	while (res[0] == QSEE_RESULT_INCOMPLETE) {
		args[0] = res[2];
		args[1] = qsee_listener_serve(res[2]) ? QSEE_RESULT_FAILURE : QSEE_RESULT_SUCCESS;
		ret = qcom_scm_qseecom_call_raw(QSEE_OWNER_QSEE_OS, QSEE_SVC_LISTENER,
						QSEE_CMD_LISTENER_RESP,
						QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_VAL),
						args, 2, res);
		if (ret)
			return ret;
	}
	if (res[0] == QSEE_RESULT_BLOCKED_ON_LISTENER) {
		pr_warn("qseecom_bridge: qtee blocked on listener %#llx\n", res[2]);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(qsee_bridge_qtee_listener);

static int qsee_bridge_app_call(u32 app_id, dma_addr_t req, size_t req_len, dma_addr_t rsp,
				size_t rsp_len)
{
	u64 args[5] = { app_id, req, req_len, rsp, rsp_len };
	u64 res[3];
	int ret;

	ret = qcom_scm_qseecom_call_raw(QSEE_OWNER_TZ_APPS, QSEE_SVC_APP_ID_PLACEHOLDER,
					QSEE_CMD_APP_SEND,
					QCOM_SCM_ARGS(5, QCOM_SCM_VAL, QCOM_SCM_RW, QCOM_SCM_VAL,
						      QCOM_SCM_RW, QCOM_SCM_VAL),
					args, 5, res);

	while (!ret && res[0] == QSEE_RESULT_INCOMPLETE) {
		u64 rsp_args[2];

		rsp_args[0] = res[2];
		rsp_args[1] = qsee_listener_serve(res[2]) ? QSEE_RESULT_FAILURE :
							    QSEE_RESULT_SUCCESS;
		ret = qcom_scm_qseecom_call_raw(QSEE_OWNER_QSEE_OS, QSEE_SVC_LISTENER,
						QSEE_CMD_LISTENER_RESP,
						QCOM_SCM_ARGS(2, QCOM_SCM_VAL, QCOM_SCM_VAL),
						rsp_args, 2, res);
	}
	if (ret)
		return ret;

	if (res[0] == QSEE_RESULT_BLOCKED_ON_LISTENER)
		pr_warn("qseecom_bridge: app %u blocked on listener %#llx\n", app_id, res[2]);
	return res[0] == QSEE_RESULT_SUCCESS ? 0 : -EIO;
}

static long qsee_bridge_listener_reg(struct file *file, void __user *argp)
{
	struct qsee_bridge_listener_reg r;
	struct qsee_listener *l, *it;
	u64 args[3], res[3];
	int ret;

	if (copy_from_user(&r, argp, sizeof(r)))
		return -EFAULT;
	if (!r.sb_len || r.sb_len > QSEE_BRIDGE_MAX_LEN)
		return -EINVAL;
	if (file->private_data)
		return -EBUSY;

	l = kzalloc_obj(*l);
	if (!l)
		return -ENOMEM;
	/*
	 * Cached, with the maintenance stock does around each request, rather than an
	 * uncached alias of memory TZ caches. Below 4G for the privileged listeners (RPMB).
	 */
	l->sb_len = PAGE_ALIGN(r.sb_len);
	l->sb = alloc_pages_exact(l->sb_len, GFP_KERNEL | GFP_DMA | __GFP_ZERO);
	if (!l->sb) {
		kfree(l);
		return -ENOMEM;
	}
	l->sb_phys = dma_map_single(qsee_bridge_dev.this_device, l->sb, l->sb_len,
				    DMA_BIDIRECTIONAL);
	if (dma_mapping_error(qsee_bridge_dev.this_device, l->sb_phys)) {
		free_pages_exact(l->sb, l->sb_len);
		kfree(l);
		return -ENOMEM;
	}
	ret = qcom_tzmem_shm_bridge_create(l->sb_phys, l->sb_len, &l->sb_handle);
	if (ret) {
		dma_unmap_single(qsee_bridge_dev.this_device, l->sb_phys, l->sb_len,
				 DMA_BIDIRECTIONAL);
		free_pages_exact(l->sb, l->sb_len);
		kfree(l);
		return ret;
	}
	l->id = r.id;
	kref_init(&l->ref);
	init_waitqueue_head(&l->wq);

	mutex_lock(&qsee_listeners_lock);
	list_for_each_entry(it, &qsee_listeners, node) {
		if (it->id == r.id && !it->dead) {
			ret = -EEXIST;
			goto err;
		}
	}

	args[0] = r.id;
	args[1] = l->sb_phys;
	args[2] = l->sb_len;
	/* The smcinvoke flavour, as stock: QTEE may then raise requests on QTEE invocations too */
	ret = qcom_scm_qseecom_call_raw(QSEE_OWNER_QSEE_OS, QSEE_SVC_LISTENER,
					QSEE_CMD_LISTENER_REG_SMCINVOKE,
					QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_RW, QCOM_SCM_VAL),
					args, 3, res);
	if (ret)
		ret = qcom_scm_qseecom_call_raw(QSEE_OWNER_QSEE_OS, QSEE_SVC_LISTENER,
						QSEE_CMD_LISTENER_REG,
						QCOM_SCM_ARGS(3, QCOM_SCM_VAL, QCOM_SCM_RW,
							      QCOM_SCM_VAL),
						args, 3, res);
	if (!ret && res[0] != QSEE_RESULT_SUCCESS) {
		pr_warn("qseecom_bridge: registering listener %#x: %#llx\n", r.id, res[0]);
		ret = -EIO;
	}
	if (ret)
		goto err;

	list_add(&l->node, &qsee_listeners);
	file->private_data = l;
	mutex_unlock(&qsee_listeners_lock);
	return 0;

err:
	mutex_unlock(&qsee_listeners_lock);
	kref_put(&l->ref, qsee_listener_free);
	return ret;
}

static long qsee_bridge_listener_recv(struct file *file, void __user *argp)
{
	struct qsee_listener *l = file->private_data;
	struct qsee_bridge_listener_buf b;
	int ret;

	if (!l)
		return -EINVAL;
	if (copy_from_user(&b, argp, sizeof(b)))
		return -EFAULT;

	ret = wait_event_interruptible(l->wq, l->req_pending);
	if (ret)
		return ret;

	dma_sync_single_for_cpu(qsee_bridge_dev.this_device, l->sb_phys, l->sb_len,
				DMA_BIDIRECTIONAL);

	if (copy_to_user(u64_to_user_ptr(b.ptr), l->sb, min_t(size_t, b.len, l->sb_len)))
		return -EFAULT;
	return 0;
}

static long qsee_bridge_listener_resp(struct file *file, void __user *argp)
{
	struct qsee_listener *l = file->private_data;
	struct qsee_bridge_listener_buf b;

	if (!l)
		return -EINVAL;
	if (copy_from_user(&b, argp, sizeof(b)))
		return -EFAULT;
	if (!l->req_pending)
		return -EINVAL;

	if (copy_from_user(l->sb, u64_to_user_ptr(b.ptr), min_t(size_t, b.len, l->sb_len)))
		return -EFAULT;
	dma_sync_single_for_device(qsee_bridge_dev.this_device, l->sb_phys, l->sb_len,
				   DMA_BIDIRECTIONAL);

	l->req_pending = false;
	l->rsp_ready = true;
	wake_up_all(&l->wq);
	return 0;
}

static long qsee_bridge_lookup(void __user *argp)
{
	struct qsee_bridge_lookup lk;
	int ret;

	if (copy_from_user(&lk, argp, sizeof(lk)))
		return -EFAULT;
	lk.name[sizeof(lk.name) - 1] = '\0';

	ret = qcom_scm_qseecom_app_get_id(lk.name, &lk.app_id);
	if (ret)
		return ret;

	return copy_to_user(argp, &lk, sizeof(lk)) ? -EFAULT : 0;
}

/* Cached buffer handed to TZ: cleaned to memory when mapped, invalidated when unmapped */
struct qsee_tz_buf {
	void *va;
	dma_addr_t pa;
	size_t len;
	size_t alloc_len;
	u64 handle;
};

static int qsee_tz_buf_get(struct qsee_tz_buf *b, size_t len, const void __user *src)
{
	struct device *dev = qsee_bridge_dev.this_device;
	int ret;

	b->len = len;
	b->alloc_len = PAGE_ALIGN(len);
	b->handle = 0;
	/* QSEE takes only buffers below 4G (ZONE_DMA here) */
	b->va = alloc_pages_exact(b->alloc_len, GFP_KERNEL | __GFP_ZERO | GFP_DMA);
	if (!b->va)
		return -ENOMEM;
	if (src && copy_from_user(b->va, src, len)) {
		free_pages_exact(b->va, b->alloc_len);
		b->va = NULL;
		return -EFAULT;
	}
	b->pa = dma_map_single(dev, b->va, b->alloc_len, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, b->pa)) {
		free_pages_exact(b->va, b->alloc_len);
		b->va = NULL;
		return -ENOMEM;
	}
	ret = qcom_tzmem_shm_bridge_create(b->pa, b->alloc_len, &b->handle);
	if (ret) {
		dma_unmap_single(dev, b->pa, b->alloc_len, DMA_BIDIRECTIONAL);
		free_pages_exact(b->va, b->alloc_len);
		b->va = NULL;
		return ret;
	}
	return 0;
}

static void qsee_tz_buf_put(struct qsee_tz_buf *b)
{
	if (!b->va)
		return;
	if (b->handle) {
		qcom_tzmem_shm_bridge_delete(b->handle);
		b->handle = 0;
	}
	dma_unmap_single(qsee_bridge_dev.this_device, b->pa, b->alloc_len, DMA_BIDIRECTIONAL);
	free_pages_exact(b->va, b->alloc_len);
	b->va = NULL;
}

static long qsee_bridge_send(void __user *argp)
{
	struct qsee_tz_buf req = {}, bufs[QSEE_BRIDGE_MAX_BUFS] = {};
	struct device *dev = qsee_bridge_dev.this_device;
	size_t rsp_off;
	struct qsee_bridge_send s;
	u32 addr_len;
	int ret = 0, i;

	if (copy_from_user(&s, argp, sizeof(s)))
		return -EFAULT;
	if (s.n_bufs > QSEE_BRIDGE_MAX_BUFS || !s.req_len || !s.rsp_len ||
	    s.req_len > QSEE_BRIDGE_MAX_LEN || s.rsp_len > QSEE_BRIDGE_MAX_LEN)
		return -EINVAL;
	addr_len = s.addr64 ? sizeof(u64) : sizeof(u32);

	for (i = 0; i < s.n_bufs; i++) {
		struct qsee_bridge_buf *b = &s.bufs[i];

		if (!b->len || b->len > QSEE_BRIDGE_MAX_LEN || b->offset > s.req_len - addr_len) {
			ret = -EINVAL;
			goto out;
		}
		ret = qsee_tz_buf_get(&bufs[i], b->len, u64_to_user_ptr(b->ptr));
		if (ret)
			goto out;
	}

	/* Request and response share one buffer, like the app shared buffer stock uses */
	rsp_off = PAGE_ALIGN(s.req_len);
	ret = qsee_tz_buf_get(&req, rsp_off + PAGE_ALIGN(s.rsp_len), NULL);
	if (ret)
		goto out;
	dma_sync_single_for_cpu(dev, req.pa, req.len, DMA_BIDIRECTIONAL);
	if (copy_from_user(req.va, u64_to_user_ptr(s.req), s.req_len)) {
		ret = -EFAULT;
		goto out;
	}
	for (i = 0; i < s.n_bufs; i++) {
		if (s.addr64)
			put_unaligned_le64(bufs[i].pa, req.va + s.bufs[i].offset);
		else
			put_unaligned_le32(bufs[i].pa, req.va + s.bufs[i].offset);
	}
	dma_sync_single_for_device(dev, req.pa, req.len, DMA_BIDIRECTIONAL);

	mutex_lock(&qsee_bridge_send_lock);
	qcom_scm_qtee_lock_acquire();
	qcom_scm_qsee_block();
	ret = qsee_bridge_app_call(s.app_id, req.pa, s.req_len, req.pa + rsp_off, s.rsp_len);
	qcom_scm_qsee_unblock();
	qcom_scm_qtee_lock_release();
	mutex_unlock(&qsee_bridge_send_lock);
	if (ret)
		goto out;

	dma_sync_single_for_cpu(dev, req.pa, req.len, DMA_BIDIRECTIONAL);
	if (copy_to_user(u64_to_user_ptr(s.rsp), req.va + rsp_off, s.rsp_len)) {
		ret = -EFAULT;
		goto out;
	}
	for (i = 0; i < s.n_bufs; i++) {
		dma_sync_single_for_cpu(dev, bufs[i].pa, bufs[i].len, DMA_BIDIRECTIONAL);
		if (copy_to_user(u64_to_user_ptr(s.bufs[i].ptr), bufs[i].va, s.bufs[i].len)) {
			ret = -EFAULT;
			goto out;
		}
	}

out:
	for (i = 0; i < s.n_bufs; i++)
		qsee_tz_buf_put(&bufs[i]);
	qsee_tz_buf_put(&req);
	return ret;
}

static long qsee_bridge_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* SCM may still be probing when the device is registered */
	if (!qcom_scm_is_available())
		return -EAGAIN;

	switch (cmd) {
	case QSEE_BRIDGE_IOC_LOOKUP:
		return qsee_bridge_lookup((void __user *)arg);
	case QSEE_BRIDGE_IOC_SEND:
		return qsee_bridge_send((void __user *)arg);
	case QSEE_BRIDGE_IOC_LISTENER_REG:
		return qsee_bridge_listener_reg(file, (void __user *)arg);
	case QSEE_BRIDGE_IOC_LISTENER_RECV:
		return qsee_bridge_listener_recv(file, (void __user *)arg);
	case QSEE_BRIDGE_IOC_LISTENER_RESP:
		return qsee_bridge_listener_resp(file, (void __user *)arg);
	default:
		return -ENOTTY;
	}
}

static int qsee_bridge_open(struct inode *inode, struct file *file)
{
	/* misc_open() leaves the miscdevice here; it holds this file's listener instead */
	file->private_data = NULL;
	return 0;
}

static int qsee_bridge_release(struct inode *inode, struct file *file)
{
	struct qsee_listener *l = file->private_data;
	u64 args[1], res[3];

	if (!l)
		return 0;

	args[0] = l->id;
	qcom_scm_qseecom_call_raw(QSEE_OWNER_QSEE_OS, QSEE_SVC_LISTENER,
				  QSEE_CMD_LISTENER_DEREG, QCOM_SCM_ARGS(1, QCOM_SCM_VAL),
				  args, 1, res);

	mutex_lock(&qsee_listeners_lock);
	l->dead = true;
	list_del(&l->node);
	mutex_unlock(&qsee_listeners_lock);
	wake_up_all(&l->wq);
	kref_put(&l->ref, qsee_listener_free);
	return 0;
}

static const struct file_operations qsee_bridge_fops = {
	.owner = THIS_MODULE,
	.open = qsee_bridge_open,
	.release = qsee_bridge_release,
	.unlocked_ioctl = qsee_bridge_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static struct miscdevice qsee_bridge_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qseecom_bridge",
	.fops = &qsee_bridge_fops,
};

static int __init qsee_bridge_init(void)
{
	int ret = misc_register(&qsee_bridge_dev);

	if (ret)
		return ret;
	return dma_coerce_mask_and_coherent(qsee_bridge_dev.this_device, DMA_BIT_MASK(64));
}
module_init(qsee_bridge_init);

static void __exit qsee_bridge_exit(void)
{
	misc_deregister(&qsee_bridge_dev);
}
module_exit(qsee_bridge_exit);

MODULE_DESCRIPTION("Userspace bridge to legacy QSEE apps");
MODULE_LICENSE("GPL");
