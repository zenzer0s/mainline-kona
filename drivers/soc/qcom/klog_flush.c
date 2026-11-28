// SPDX-License-Identifier: GPL-2.0
/*
 * Keep the kernel and bootloader logs on flash, so they survive a reset that
 * takes DRAM with it: a PON shutdown after a long power-button press, an XBL
 * cold reset, a hard power cycle.
 *
 * pstore/ramoops only holds the log until the next cold reset, which is the
 * one a clean install hits when the ROM hangs at boot and the user
 * power-cycles the phone. This writes the same text to a file under persist
 * every interval seconds, so a snapshot at most one interval old is already
 * on flash before anything dies, and exposes it as /proc/last_kmsg after the
 * next boot.
 *
 * Writes happen in a workqueue and never in the panic path: block IO after a
 * panic is unsafe, and the panic path is already covered by ramoops, which
 * survives panic and watchdog resets because those are warm.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/timekeeping.h>
#include <linux/utsname.h>
#include <linux/workqueue.h>

#include "uefi_kbootlog.h"

#define KLOG_DIR		"/mnt/vendor/persist/klog"
#define KLOG_NAME		"last_kmsg"
#define KLOG_MAGIC		0x4B4C4F47	/* "KLOG" */
#define KLOG_VERSION		1
#define KLOG_MAX_TEXT		(1024 * 1024)
#define KLOG_DEFAULT_INTERVAL	10

struct klog_header {
	u32 magic;
	u32 version;
	u32 text_len;
	u32 xbl_len;
	u32 crc;
	u32 reserved;
	u64 real_time;
	u64 mono_time;
	char kernel[64];
};

static char klog_name[64] = KLOG_NAME;
static int klog_interval = KLOG_DEFAULT_INTERVAL;
static bool is_recovery_mode;
module_param_string(file, klog_name, sizeof(klog_name), 0644);
MODULE_PARM_DESC(file, "file name under " KLOG_DIR);
module_param_named(interval, klog_interval, int, 0644);
MODULE_PARM_DESC(interval, "seconds between snapshots, 0 disables");

static char klog_path[192];

static struct kmsg_dumper klog_dumper;
static struct delayed_work klog_work;
static DEFINE_MUTEX(klog_lock);

/* Written by the dumper / collector, read by the workqueue. */
static char *klog_text;
static size_t klog_text_len;
static char *klog_xbl;
static size_t klog_xbl_len;
static char *klog_io_buf;
static struct klog_header klog_hdr;

/* Previous boot's snapshot loaded once from disk, served by /proc/last_kmsg. */
static char *klog_saved;
static size_t klog_saved_len;
static bool klog_prev_loaded;
static bool klog_meta_written;
static bool klog_xbl_tried;
static bool klog_has_flushed;
static u32 klog_last_crc;
static size_t klog_last_len;

static void klog_load_from_disk(void);

static void klog_capture_locked(void)
{
	struct kmsg_dump_iter iter;
	size_t len = 0;

	kmsg_dump_rewind(&iter);
	if (kmsg_dump_get_buffer(&iter, true, klog_text,
				 KLOG_MAX_TEXT - 1, &len)) {
		klog_text[len] = '\0';
		klog_text_len = len;
		memset(&klog_hdr, 0, sizeof(klog_hdr));
		klog_hdr.magic = KLOG_MAGIC;
		klog_hdr.version = KLOG_VERSION;
		klog_hdr.text_len = len;
		klog_hdr.xbl_len = klog_xbl_len;
		/* Saved for the workqueue, which hashes before it writes. */
		klog_hdr.crc = 0;
		klog_hdr.real_time = ktime_get_real_seconds();
		klog_hdr.mono_time = ktime_get_seconds();
		strscpy(klog_hdr.kernel, init_uts_ns.name.version,
			sizeof(klog_hdr.kernel));
	}
}

static void klog_dump(struct kmsg_dumper *dumper,
		      struct kmsg_dump_detail *detail)
{
	enum kmsg_dump_reason reason = detail ? detail->reason : KMSG_DUMP_UNDEF;

	if (reason == KMSG_DUMP_UNDEF)
		return;

	/*
	 * Runs in panic and watchdog context: no allocations, no block IO.
	 * mutex_trylock keeps a tick spinning on the logbuf from stalling a
	 * real dump.
	 */
	if (!mutex_trylock(&klog_lock))
		return;

	klog_capture_locked();
	mutex_unlock(&klog_lock);
}

/*
 * Capture the kernel log buffer directly without invoking global kmsg_dump(),
 * which would also trigger pstore/ramoops zlib compression with IRQs disabled
 * and fabricate spurious Oops#N files in /sys/fs/pstore on normal boots.
 */
static void klog_collect(void)
{
	mutex_lock(&klog_lock);
	klog_capture_locked();
	mutex_unlock(&klog_lock);
}

/*
 * The bootloader log is read once, after kbootlog has captured it, and kept
 * in memory: it never changes for this boot.
 */
static void klog_read_bootloader(void)
{
	ssize_t n;

	if (klog_xbl_len || klog_xbl_tried)
		return;

	n = qcom_kbootlog_copy(klog_xbl, KLOG_MAX_TEXT / 4 - 1);
	if (n <= 0)
		return;

	klog_xbl[n] = '\0';
	klog_xbl_len = n;
	klog_xbl_tried = true;
}

static ssize_t klog_write_all(struct file *f, loff_t pos,
			      const char *buf, size_t len)
{
	ssize_t done = 0;

	while (done < len) {
		ssize_t ret = kernel_write(f, buf + done, len - done, &pos);

		if (ret < 0)
			return ret;
		done += ret;
	}

	return done;
}

static bool klog_write_snapshot_file(const char *path,
				     const struct klog_header *hdr,
				     const char *text, size_t len,
				     const char *xbl, size_t xbl_len)
{
	struct file *f = filp_open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	int ret;

	if (IS_ERR(f))
		return false;

	ret = klog_write_all(f, 0, (const char *)hdr, sizeof(*hdr));
	if (ret == sizeof(*hdr))
		ret = klog_write_all(f, ret, text, len);
	if (ret == sizeof(*hdr) + len && xbl_len)
		klog_write_all(f, ret, xbl, xbl_len);
	vfs_fsync(f, 0);
	filp_close(f, NULL);
	return true;
}

static void klog_flush(struct work_struct *work)
{
	struct klog_header hdr;
	struct file *f;
	size_t len, xbl_len;
	int ret;
	bool written = false;

	if (!klog_interval)
		return;

	if (!klog_xbl_len && !klog_xbl_tried) {
		klog_read_bootloader();
		klog_xbl_tried = true;
	}

	/*
	 * Snapshot the previous boot's last_kmsg into memory BEFORE our first
	 * write truncates either persistent file, so /proc/last_kmsg always
	 * serves the previous boot rather than the current one.
	 */
	if (!klog_prev_loaded && !is_recovery_mode)
		klog_load_from_disk();

	klog_collect();

	mutex_lock(&klog_lock);
	memcpy(&hdr, &klog_hdr, sizeof(hdr));
	len = klog_text_len;
	if (len) {
		memcpy(klog_io_buf, klog_text, len);
		hdr.crc = crc32(0, klog_io_buf, len);
	}
	xbl_len = klog_xbl_len;
	mutex_unlock(&klog_lock);

	if (!len || hdr.magic != KLOG_MAGIC)
		goto requeue;

	if (klog_has_flushed && len == klog_last_len && hdr.crc == klog_last_crc)
		goto requeue;

	f = filp_open(klog_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
	if (!IS_ERR(f)) {
		ret = klog_write_all(f, 0, (const char *)&hdr, sizeof(hdr));
		if (ret == sizeof(hdr))
			ret = klog_write_all(f, ret, klog_io_buf, len);
		if (ret == sizeof(hdr) + len && xbl_len)
			klog_write_all(f, ret, klog_xbl, xbl_len);
		vfs_fsync(f, 0);
		filp_close(f, NULL);
		written = true;
		/* Persist is now mounted and overwritten; don't reload our own boot. */
		klog_prev_loaded = true;
	}

	/* Also write to /metadata/bringup/last_kmsg for guaranteed persistent capture */
	if (klog_write_snapshot_file("/metadata/bringup/last_kmsg", &hdr, klog_io_buf, len, klog_xbl, xbl_len))
		klog_meta_written = true;

	if (!written) {
		/* Persist partition might not be mounted yet; retry in 1s */
		schedule_delayed_work(&klog_work, 1 * HZ);
		return;
	}

	klog_last_crc = hdr.crc;
	klog_last_len = len;
	klog_has_flushed = true;
requeue:
	schedule_delayed_work(&klog_work, klog_interval * HZ);
}

static int klog_reboot_notifier(struct notifier_block *nb,
				unsigned long action, void *data)
{
	/*
	 * Last chance on a software restart or poweroff; the pstore path also
	 * dumps here, but this file is what survives a PON shutdown.
	 */
	if (action != SYS_RESTART && action != SYS_POWER_OFF &&
	    action != SYS_HALT)
		return NOTIFY_DONE;

	cancel_delayed_work_sync(&klog_work);
	klog_collect();
	klog_flush(&klog_work.work);
	cancel_delayed_work_sync(&klog_work);

	return NOTIFY_DONE;
}

static struct notifier_block klog_nb = {
	.notifier_call = klog_reboot_notifier,
};

static ssize_t klog_proc_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos,
				       klog_saved, klog_saved_len);
}

static void klog_load_from_disk(void)
{
	static const char * const paths[] = {
		KLOG_DIR "/last_kmsg",
		"/metadata/bringup/last_kmsg",
	};
	struct file *f = ERR_PTR(-ENOENT);
	loff_t pos = 0;
	ssize_t n;
	char *buf;
	int i;

	for (i = 0; i < ARRAY_SIZE(paths); i++) {
		if (i == 1 && klog_meta_written)
			continue;
		f = filp_open(paths[i], O_RDONLY, 0);
		if (!IS_ERR(f))
			break;
	}
	if (IS_ERR(f))
		return;

	buf = kvmalloc(KLOG_MAX_TEXT, GFP_KERNEL);
	if (!buf) {
		filp_close(f, NULL);
		return;
	}

	n = kernel_read(f, buf, KLOG_MAX_TEXT - 1, &pos);
	filp_close(f, NULL);

	if (n > sizeof(struct klog_header)) {
		struct klog_header *hdr = (struct klog_header *)buf;
		if (hdr->magic == KLOG_MAGIC) {
			size_t text_off = sizeof(struct klog_header);
			size_t text_sz = min_t(size_t, hdr->text_len, (size_t)(n - text_off));
			mutex_lock(&klog_lock);
			memcpy(klog_saved, buf + text_off, text_sz);
			klog_saved_len = text_sz;
			mutex_unlock(&klog_lock);
		} else {
			mutex_lock(&klog_lock);
			memcpy(klog_saved, buf, n);
			klog_saved_len = n;
			mutex_unlock(&klog_lock);
		}
		klog_prev_loaded = true;
	}
	kvfree(buf);
}

static int klog_proc_open(struct inode *inode, struct file *file)
{
	if (!klog_prev_loaded || is_recovery_mode)
		klog_load_from_disk();
	if (!klog_saved_len)
		return -ENODATA;

	return 0;
}

static const struct proc_ops klog_proc_ops = {
	.proc_open	= klog_proc_open,
	.proc_read	= klog_proc_read,
	.proc_lseek	= default_llseek,
};

static void klog_check_mode(void)
{
	if (saved_command_line &&
	    (strstr(saved_command_line, "androidboot.mode=recovery") ||
	     strstr(saved_command_line, "androidboot.bootmode=recovery") ||
	     strstr(saved_command_line, "androidboot.mode=fastboot") ||
	     strstr(saved_command_line, "oplus_ftm_mode=ftmrecovery"))) {
		is_recovery_mode = true;
		strscpy(klog_name, "recovery_kmsg", sizeof(klog_name));
	} else {
		is_recovery_mode = false;
		strscpy(klog_name, "last_kmsg", sizeof(klog_name));
	}
}

static int __init klog_flush_init(void)
{
	klog_text = kvzalloc(KLOG_MAX_TEXT, GFP_KERNEL);
	klog_saved = kvzalloc(KLOG_MAX_TEXT, GFP_KERNEL);
	klog_io_buf = kvzalloc(KLOG_MAX_TEXT, GFP_KERNEL);
	klog_xbl = kvzalloc(KLOG_MAX_TEXT / 4, GFP_KERNEL);
	if (!klog_text || !klog_saved || !klog_io_buf || !klog_xbl) {
		kvfree(klog_text);
		kvfree(klog_saved);
		kvfree(klog_io_buf);
		kvfree(klog_xbl);
		return -ENOMEM;
	}

	klog_check_mode();
	snprintf(klog_path, sizeof(klog_path), "%s/%s", KLOG_DIR, klog_name);

	klog_read_bootloader();

	klog_dumper.dump = klog_dump;
	klog_dumper.max_reason = KMSG_DUMP_EMERG;
	if (kmsg_dump_register(&klog_dumper)) {
		pr_err("klog: failed to register dumper\n");
		return -EBUSY;
	}

	INIT_DELAYED_WORK(&klog_work, klog_flush);
	register_reboot_notifier(&klog_nb);
	proc_create("last_kmsg", 0444, NULL, &klog_proc_ops);

	pr_info("klog: mode=%s, flushing logs to %s every %d seconds\n",
		is_recovery_mode ? "recovery" : "system", klog_path, klog_interval);

	/* First snapshot very early (2 seconds) to catch early init crashes */
	schedule_delayed_work(&klog_work, 2 * HZ);
	return 0;
}
late_initcall(klog_flush_init);
