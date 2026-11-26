// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 *
 * Core dual-Hall Alert Slider (tri-state key) state machine and /proc/tristatekey
 * interface for OnePlus 8T (kebab) / SM8250 devices.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "oplus_tri_key.h"

#define TRI_KEY_DEVICE_NAME	"oplus,hall_tri_state_key"
#define TRI_KEY_VENDOR_ID	0x22d9

#define TRI_KEY_TOL0		15
#define TRI_KEY_TOL2		40
#define TRI_KEY_THRESH_MARGIN	22

struct oplus_tri_key_dev {
	struct mutex lock;
	struct oplus_hall_sensor *sensors[DHALL_MAX];

	struct input_dev *input_dev;
	struct proc_dir_entry *proc_dir;
	struct wakeup_source *ws;

	struct work_struct register_work;
	struct work_struct dwork;
	struct work_struct timeout_work;
	struct hrtimer timer;

	bool initialized;
	bool calibrated;
	unsigned int debug_level;

	int position;
	int last_position;
	int interf;
	int last_interf;
	int interf_count;
	int timer_sec;

	/* Reported userspace state: 1 = TOP/UP, 2 = MID, 3 = BOTTOM/DOWN */
	short state;

	/* Latest raw readings: data0 = DOWN hall, data1 = UP hall */
	short dhall_data0;
	short dhall_data1;
	int last_d0;
	int last_d1;

	/* Raw factory calibration values */
	short dn_hall_up_v;
	short dn_hall_md_v;
	short dn_hall_dn_v;
	short up_hall_up_v;
	short up_hall_md_v;
	short up_hall_dn_v;

	/* Derived calibration sums, diffs, distances, and tolerances */
	short calib_up_sum;
	short calib_md_sum;
	short calib_dn_sum;
	short calib_up_min;
	short calib_md_min;
	short calib_dn_min;
	short calib_dn_um_dist;
	short calib_dn_md_dist;
	short calib_up_um_dist;
	short calib_up_md_dist;
	short calib_up_ud_dist;
	short calib_dn_ud_dist;

	short up_mid_tol;
	short up_tolerance;
	short mid_down_tol;
	short down_tolerance;
};

static struct oplus_tri_key_dev *g_tri_key;
static DEFINE_MUTEX(g_tri_key_reg_lock);

#define tri_key_dbg(chip, fmt, args...) \
	do { \
		if ((chip)->debug_level) \
			pr_info("oplus_tri_key: " fmt, ##args); \
	} while (0)

static int hall_clear_irq(struct oplus_tri_key_dev *chip, enum oplus_hall_id id)
{
	struct oplus_hall_sensor *s = chip->sensors[id];

	if (!s || !s->ops || !s->ops->clear_irq)
		return -ENODEV;

	return s->ops->clear_irq(s);
}

static int hall_get_data(struct oplus_tri_key_dev *chip, enum oplus_hall_id id)
{
	struct oplus_hall_sensor *s = chip->sensors[id];
	short *dst = (id == DHALL_DOWN) ? &chip->dhall_data0 : &chip->dhall_data1;

	if (!s || !s->ops || !s->ops->get_data)
		return -ENODEV;

	return s->ops->get_data(s, dst);
}

static int hall_read_both(struct oplus_tri_key_dev *chip)
{
	int ret;

	ret = hall_get_data(chip, DHALL_DOWN);
	if (ret < 0)
		return ret;

	return hall_get_data(chip, DHALL_UP);
}

static int hall_update_threshold(struct oplus_tri_key_dev *chip,
				 enum oplus_hall_id id, int position,
				 short lowthd, short highthd)
{
	struct oplus_hall_sensor *s = chip->sensors[id];

	if (!s || !s->ops || !s->ops->update_threshold)
		return -ENODEV;

	return s->ops->update_threshold(s, position, lowthd, highthd);
}

static int hall_set_detection_mode(struct oplus_tri_key_dev *chip,
				   enum oplus_hall_id id, u8 mode)
{
	struct oplus_hall_sensor *s = chip->sensors[id];

	if (!s || !s->ops || !s->ops->set_detection_mode)
		return -ENODEV;

	return s->ops->set_detection_mode(s, mode);
}

static void hall_unmask_irqs(struct oplus_tri_key_dev *chip)
{
	int id;

	hall_clear_irq(chip, DHALL_DOWN);
	hall_clear_irq(chip, DHALL_UP);

	for (id = 0; id < DHALL_MAX; id++) {
		struct oplus_hall_sensor *s = chip->sensors[id];

		if (s && s->ops && s->ops->enable_irq)
			s->ops->enable_irq(s, true);
	}
}

static void reboot_get_position(struct oplus_tri_key_dev *chip)
{
	short delta;

	if (chip->dhall_data1 < 0 || chip->dhall_data0 < 0)
		delta = (-chip->dhall_data1) - (-chip->dhall_data0);
	else
		delta = chip->dhall_data1 - chip->dhall_data0;

	if (delta > 30)
		chip->position = UP_STATE;
	else if (-delta > 30)
		chip->position = DOWN_STATE;
	else
		chip->position = MID_STATE;

	chip->last_position = chip->position;
}

static int interf_get_position(struct oplus_tri_key_dev *chip)
{
	short delta0 = chip->dhall_data0 - chip->last_d0;
	short delta1 = chip->dhall_data1 - chip->last_d1;

	tri_key_dbg(chip, "interf delta0=%d delta1=%d last_pos=%d\n",
		    delta0, delta1, chip->last_position);

	if (delta1 > chip->calib_up_um_dist - TRI_KEY_TOL0 &&
	    delta1 < chip->calib_up_um_dist + TRI_KEY_TOL0 &&
	    delta0 > chip->calib_dn_um_dist - TRI_KEY_TOL0 &&
	    delta0 < chip->calib_dn_um_dist + TRI_KEY_TOL0) {
		if (chip->last_position == MID_STATE)
			return UP_STATE;
	}

	if (delta1 > chip->calib_up_ud_dist - TRI_KEY_TOL0 &&
	    delta1 < chip->calib_up_ud_dist + TRI_KEY_TOL0 &&
	    delta0 > chip->calib_dn_ud_dist - TRI_KEY_TOL0 &&
	    delta0 < chip->calib_dn_ud_dist + TRI_KEY_TOL0)
		return UP_STATE;

	if (delta1 > -chip->calib_up_md_dist - TRI_KEY_TOL0 &&
	    delta1 < -chip->calib_up_md_dist + TRI_KEY_TOL0 &&
	    delta0 > -chip->calib_dn_md_dist - TRI_KEY_TOL0 &&
	    delta0 < -chip->calib_dn_md_dist + TRI_KEY_TOL0) {
		if (chip->last_position == MID_STATE)
			return DOWN_STATE;
	}

	if (delta1 > -chip->calib_up_ud_dist - TRI_KEY_TOL0 &&
	    delta1 < -chip->calib_up_ud_dist + TRI_KEY_TOL0 &&
	    delta0 > -chip->calib_dn_ud_dist - TRI_KEY_TOL0 &&
	    delta0 < -chip->calib_dn_ud_dist + TRI_KEY_TOL0)
		return DOWN_STATE;

	if (delta1 > -chip->calib_up_um_dist - TRI_KEY_TOL0 &&
	    delta1 < -chip->calib_up_um_dist + TRI_KEY_TOL0 &&
	    delta0 > -chip->calib_dn_um_dist - TRI_KEY_TOL0 &&
	    delta0 < -chip->calib_dn_um_dist + TRI_KEY_TOL0) {
		if (chip->last_position == UP_STATE)
			return MID_STATE;
	}

	if (delta1 > chip->calib_up_md_dist - TRI_KEY_TOL0 &&
	    delta1 < chip->calib_up_md_dist + TRI_KEY_TOL0 &&
	    delta0 > chip->calib_dn_md_dist - TRI_KEY_TOL0 &&
	    delta0 < chip->calib_dn_md_dist + TRI_KEY_TOL0) {
		if (chip->last_position == DOWN_STATE)
			return MID_STATE;
	}

	return -EINVAL;
}

static void get_position(struct oplus_tri_key_dev *chip)
{
	short diff = chip->dhall_data1 - chip->dhall_data0;

	if (chip->dhall_data0 > 0) {
		if (diff > chip->calib_up_min - chip->up_mid_tol &&
		    diff < chip->calib_up_min + chip->up_tolerance)
			chip->position = UP_STATE;
		if (diff > chip->calib_md_min - chip->mid_down_tol &&
		    diff < chip->calib_md_min + chip->up_mid_tol)
			chip->position = MID_STATE;
		if (diff > chip->calib_dn_min - chip->down_tolerance &&
		    diff < chip->calib_dn_min + chip->mid_down_tol)
			chip->position = DOWN_STATE;
	} else {
		if (diff > chip->calib_up_min - chip->up_tolerance &&
		    diff < chip->calib_up_min + chip->up_mid_tol)
			chip->position = UP_STATE;
		if (diff > chip->calib_md_min - chip->mid_down_tol &&
		    diff < chip->calib_md_min + chip->up_mid_tol)
			chip->position = MID_STATE;
		if (diff > chip->calib_dn_min - chip->mid_down_tol &&
		    diff < chip->calib_dn_min + chip->down_tolerance)
			chip->position = DOWN_STATE;
	}
}

static int judge_interference(struct oplus_tri_key_dev *chip)
{
	short delta = chip->dhall_data1 - chip->dhall_data0;
	short sum = chip->dhall_data0 + chip->dhall_data1;

	tri_key_dbg(chip, "delta=%d sum=%d\n", delta, sum);

	if (chip->dhall_data1 > 0) {
		if (delta > chip->calib_up_min - chip->up_mid_tol &&
		    delta < chip->calib_up_min + chip->up_tolerance) {
			chip->interf = (sum < chip->calib_up_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_up_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 1;
			return 0;
		}
		if (delta > chip->calib_md_min - chip->mid_down_tol &&
		    delta < chip->calib_md_min + chip->up_mid_tol) {
			chip->interf = (sum < chip->calib_md_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_md_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 2;
			return 0;
		}
		if (delta > chip->calib_dn_min - chip->down_tolerance &&
		    delta < chip->calib_dn_min + chip->mid_down_tol) {
			chip->interf = (sum < chip->calib_dn_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_dn_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 3;
			return 0;
		}
	} else {
		if (delta > chip->calib_up_min - chip->up_tolerance &&
		    delta < chip->calib_up_min + chip->up_mid_tol) {
			chip->interf = (sum < chip->calib_up_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_up_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 1;
			return 0;
		}
		if (delta > chip->calib_md_min - chip->mid_down_tol &&
		    delta < chip->calib_md_min + chip->up_mid_tol) {
			chip->interf = (sum < chip->calib_md_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_md_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 2;
			return 0;
		}
		if (delta > chip->calib_dn_min - chip->mid_down_tol &&
		    delta < chip->calib_dn_min + chip->down_tolerance) {
			chip->interf = (sum < chip->calib_dn_sum - TRI_KEY_TOL2 ||
					sum > chip->calib_dn_sum + TRI_KEY_TOL2) ? 1 : 0;
			chip->state = 3;
			return 0;
		}
	}

	chip->interf = 1;
	chip->state = 0;
	return -EINVAL;
}

static int reupdate_threshold(struct oplus_tri_key_dev *chip)
{
	int ret = 0;

	switch (chip->position) {
	case UP_STATE:
		ret = hall_update_threshold(chip, DHALL_UP, UP_STATE,
					    chip->dhall_data1 - TRI_KEY_THRESH_MARGIN,
					    chip->dhall_data1 + TRI_KEY_THRESH_MARGIN);
		if (ret < 0)
			goto out;
		ret = hall_update_threshold(chip, DHALL_DOWN, UP_STATE, -500, 500);
		if (ret < 0)
			goto out;
		break;

	case MID_STATE:
		ret = hall_update_threshold(chip, DHALL_UP, MID_STATE,
					    chip->dhall_data1 - TRI_KEY_THRESH_MARGIN,
					    chip->dhall_data1 + TRI_KEY_THRESH_MARGIN);
		if (ret < 0)
			goto out;
		ret = hall_update_threshold(chip, DHALL_DOWN, MID_STATE,
					    chip->dhall_data0 - TRI_KEY_THRESH_MARGIN,
					    chip->dhall_data0 + TRI_KEY_THRESH_MARGIN);
		if (ret < 0)
			goto out;
		break;

	case DOWN_STATE:
		ret = hall_update_threshold(chip, DHALL_DOWN, DOWN_STATE,
					    chip->dhall_data0 - TRI_KEY_THRESH_MARGIN,
					    chip->dhall_data0 + TRI_KEY_THRESH_MARGIN);
		if (ret < 0)
			goto out;
		ret = hall_update_threshold(chip, DHALL_UP, DOWN_STATE, -500, 500);
		if (ret < 0)
			goto out;
		break;

	default:
		ret = -EINVAL;
		break;
	}

out:
	chip->last_d0 = chip->dhall_data0;
	chip->last_d1 = chip->dhall_data1;
	chip->last_interf = chip->interf;
	hall_clear_irq(chip, DHALL_DOWN);
	hall_clear_irq(chip, DHALL_UP);

	if (ret < 0)
		pr_err("oplus_tri_key: update threshold failed (%d)\n", ret);
	else
		tri_key_dbg(chip, "thresholds updated pos=%d d0=%d d1=%d\n",
			    chip->position, chip->last_d0, chip->last_d1);

	return ret;
}

static void report_key_value(struct oplus_tri_key_dev *chip)
{
	switch (chip->position) {
	case UP_STATE:
		chip->state = 1;
		break;
	case MID_STATE:
		chip->state = 2;
		break;
	case DOWN_STATE:
		chip->state = 3;
		break;
	default:
		return;
	}

	input_report_key(chip->input_dev, KEY_F3, 1);
	input_sync(chip->input_dev);
	input_report_key(chip->input_dev, KEY_F3, 0);
	input_sync(chip->input_dev);

	pr_info("oplus_tri_key: reported position %d (state=%d)\n",
		chip->position, chip->state);
}

static int judge_calibration_data(struct oplus_tri_key_dev *chip)
{
	if (!chip->calibrated) {
		int prev_pos = chip->last_position;
		int ret = hall_read_both(chip);

		if (ret < 0)
			return ret;

		reboot_get_position(chip);
		if (chip->position != prev_pos)
			report_key_value(chip);
		reupdate_threshold(chip);
		return -EAGAIN;
	}

	return 0;
}

static void tri_key_dev_work(struct work_struct *work)
{
	struct oplus_tri_key_dev *chip =
		container_of(work, struct oplus_tri_key_dev, dwork);
	int ret, position;
	int diff0, diff1;
	int dhall0_sum = 0, dhall1_sum = 0;
	int aver0, aver1;
	int i;
	ktime_t start_time;

	mutex_lock(&chip->lock);
	if (!chip->initialized) {
		mutex_unlock(&chip->lock);
		return;
	}

	start_time = ktime_get();
	msleep(50);

	ret = judge_calibration_data(chip);
	if (ret < 0)
		goto out_unmask;

	ret = hall_read_both(chip);
	if (ret < 0) {
		pr_err("oplus_tri_key: failed to read hall data (%d)\n", ret);
		goto out_unmask;
	}
	tri_key_dbg(chip, "data1=%d data0=%d\n", chip->dhall_data1, chip->dhall_data0);

	judge_interference(chip);
	tri_key_dbg(chip, "interf=%d state=%d\n", chip->interf, chip->state);

	if (!chip->last_interf && chip->interf) {
		msleep(200);
		if (hall_read_both(chip) == 0)
			judge_interference(chip);
	}

	if (!chip->interf) {
		hrtimer_cancel(&chip->timer);
		chip->timer_sec = 1;
		if (!chip->last_interf) {
			chip->interf_count = 0;
			get_position(chip);
		} else {
			msleep(50);
			if (hall_read_both(chip) < 0)
				goto out_unmask;
			judge_interference(chip);
			if (chip->interf)
				goto out_unmask;
			get_position(chip);
		}
	} else {
		hrtimer_cancel(&chip->timer);
		hrtimer_start(&chip->timer, ktime_set(chip->timer_sec, 0),
			      HRTIMER_MODE_REL);

		for (i = 0; i < 4; i++) {
			msleep(35);
			hall_get_data(chip, DHALL_DOWN);
			hall_get_data(chip, DHALL_UP);
			dhall0_sum += chip->dhall_data0;
			dhall1_sum += chip->dhall_data1;
		}
		aver0 = dhall0_sum / 4;
		aver1 = dhall1_sum / 4;

		diff0 = aver0 - chip->dhall_data0;
		diff1 = aver1 - chip->dhall_data1;
		tri_key_dbg(chip, "interf diff0=%d diff1=%d\n", diff0, diff1);

		if (!chip->last_interf) {
			if (diff0 > -10 && diff0 < 10 &&
			    diff1 > -10 && diff1 < 10) {
				chip->position = chip->last_position;
				goto update_thresh;
			}
			chip->last_interf = chip->interf;
			goto out_unmask;
		}

		if ((diff0 < -10 || diff0 > 10) &&
		    (diff1 < -10 || diff1 > 10)) {
			chip->interf_count++;
			if (chip->interf_count >= 15) {
				msleep(5000);
				chip->interf_count = 0;
				goto out_unmask;
			}
			reupdate_threshold(chip);
			goto out_unmask;
		}

		chip->dhall_data0 = aver0;
		chip->dhall_data1 = aver1;
		position = interf_get_position(chip);
		if (position >= 0)
			chip->position = position;
		else
			tri_key_dbg(chip, "interf_get_position failed\n");
	}

update_thresh:
	ret = reupdate_threshold(chip);
	if (ret < 0)
		goto out_unmask;

	if (chip->position != chip->last_position) {
		u64 usecs = ktime_us_delta(ktime_get(), start_time);

		report_key_value(chip);
		chip->last_position = chip->position;
		tri_key_dbg(chip, "reported in %llu.%03llu ms\n",
			    usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
	}

out_unmask:
	hall_unmask_irqs(chip);
	mutex_unlock(&chip->lock);
}

static enum hrtimer_restart tri_key_status_timeout(struct hrtimer *timer)
{
	struct oplus_tri_key_dev *chip =
		container_of(timer, struct oplus_tri_key_dev, timer);

	schedule_work(&chip->timeout_work);
	return HRTIMER_NORESTART;
}

static void tri_key_timeout_work_func(struct work_struct *work)
{
	struct oplus_tri_key_dev *chip =
		container_of(work, struct oplus_tri_key_dev, timeout_work);

	mutex_lock(&chip->lock);
	if (!chip->initialized) {
		mutex_unlock(&chip->lock);
		return;
	}

	if (hall_read_both(chip) < 0) {
		mutex_unlock(&chip->lock);
		return;
	}

	judge_interference(chip);
	if (chip->interf) {
		chip->timer_sec = min(chip->timer_sec * 2, 2);
	} else {
		get_position(chip);
		if (chip->position != chip->last_position) {
			reupdate_threshold(chip);
			report_key_value(chip);
			chip->last_position = chip->position;
		}
		chip->timer_sec = 1;
	}

	mutex_unlock(&chip->lock);
}

static void init_calib_values(struct oplus_tri_key_dev *chip)
{
	chip->calib_up_sum = chip->dn_hall_up_v + chip->up_hall_up_v;
	chip->calib_md_sum = chip->dn_hall_md_v + chip->up_hall_md_v;
	chip->calib_dn_sum = chip->dn_hall_dn_v + chip->up_hall_dn_v;

	chip->calib_up_min = chip->up_hall_up_v - chip->dn_hall_up_v;
	chip->calib_md_min = chip->up_hall_md_v - chip->dn_hall_md_v;
	chip->calib_dn_min = chip->up_hall_dn_v - chip->dn_hall_dn_v;

	chip->calib_up_um_dist = chip->up_hall_up_v - chip->up_hall_md_v;
	chip->calib_up_md_dist = chip->up_hall_md_v - chip->up_hall_dn_v;
	chip->calib_dn_um_dist = chip->dn_hall_up_v - chip->dn_hall_md_v;
	chip->calib_dn_md_dist = chip->dn_hall_md_v - chip->dn_hall_dn_v;
	chip->calib_up_ud_dist = chip->up_hall_up_v - chip->up_hall_dn_v;
	chip->calib_dn_ud_dist = chip->dn_hall_up_v - chip->dn_hall_dn_v;

	chip->up_mid_tol = (short)(abs(chip->calib_up_min - chip->calib_md_min) * 4 / 10);
	chip->up_tolerance = (short)(abs(chip->calib_up_min - chip->calib_md_min) * 11 / 10);
	chip->mid_down_tol = (short)(abs(chip->calib_md_min - chip->calib_dn_min) * 4 / 10);
	chip->down_tolerance = (short)(abs(chip->calib_md_min - chip->calib_dn_min) * 11 / 10);

	pr_info("oplus_tri_key: calib UpMin=%d MdMin=%d DnMin=%d tol=(%d,%d)\n",
		chip->calib_up_min, chip->calib_md_min, chip->calib_dn_min,
		chip->up_mid_tol, chip->mid_down_tol);
}

static ssize_t proc_hall_data_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	char buf[32];
	int len;

	if (!chip) {
		len = scnprintf(buf, sizeof(buf), "-1\n");
	} else {
		mutex_lock(&chip->lock);
		hall_read_both(chip);
		len = scnprintf(buf, sizeof(buf), "%d, %d\n",
				chip->dhall_data0, chip->dhall_data1);
		mutex_unlock(&chip->lock);
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct proc_ops proc_hall_data_ops = {
	.proc_read = proc_hall_data_read,
	.proc_lseek = default_llseek,
};

static ssize_t proc_tri_state_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	char buf[16];
	int len;

	if (!chip) {
		len = scnprintf(buf, sizeof(buf), "-1\n");
	} else {
		mutex_lock(&chip->lock);
		len = scnprintf(buf, sizeof(buf), "%d\n", chip->state);
		mutex_unlock(&chip->lock);
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct proc_ops proc_tri_state_ops = {
	.proc_read = proc_tri_state_read,
	.proc_lseek = default_llseek,
};

static ssize_t proc_hall_data_calib_read(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	char buf[64];
	int len;

	if (!chip) {
		len = scnprintf(buf, sizeof(buf), "-1\n");
	} else {
		mutex_lock(&chip->lock);
		len = scnprintf(buf, sizeof(buf), "%d,%d,%d,%d,%d,%d,%d,%d\n",
				chip->dn_hall_up_v, chip->up_hall_up_v,
				chip->dn_hall_md_v, chip->up_hall_md_v,
				chip->dn_hall_dn_v, chip->up_hall_dn_v,
				chip->up_mid_tol, chip->mid_down_tol);
		mutex_unlock(&chip->lock);
	}

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t proc_hall_data_calib_write(struct file *file,
					  const char __user *buffer,
					  size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	char temp[64];
	int data[6];
	size_t len;

	if (!chip || count == 0)
		return count;

	len = min(count, sizeof(temp) - 1);
	if (copy_from_user(temp, buffer, len))
		return -EFAULT;
	temp[len] = '\0';

	if (sscanf(temp, "%d,%d,%d,%d,%d,%d",
		   &data[0], &data[1], &data[2],
		   &data[3], &data[4], &data[5]) != 6) {
		pr_err("oplus_tri_key: invalid calibration input: '%s'\n", temp);
		return -EINVAL;
	}

	mutex_lock(&chip->lock);
	chip->dn_hall_up_v = (short)data[0];
	chip->up_hall_up_v = (short)data[1];
	chip->dn_hall_md_v = (short)data[2];
	chip->up_hall_md_v = (short)data[3];
	chip->dn_hall_dn_v = (short)data[4];
	chip->up_hall_dn_v = (short)data[5];

	init_calib_values(chip);
	chip->calibrated = true;

	if (chip->initialized && hall_read_both(chip) == 0) {
		get_position(chip);
		reupdate_threshold(chip);
		if (chip->position != chip->last_position) {
			report_key_value(chip);
			chip->last_position = chip->position;
		}
	}
	mutex_unlock(&chip->lock);

	return count;
}

static const struct proc_ops proc_hall_data_calib_ops = {
	.proc_read = proc_hall_data_calib_read,
	.proc_write = proc_hall_data_calib_write,
	.proc_lseek = default_llseek,
};

static ssize_t proc_hall_debug_info_read(struct file *file,
					 char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	char buf[16];
	int len;

	len = scnprintf(buf, sizeof(buf), "%u\n", chip ? chip->debug_level : 0);
	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t proc_hall_debug_info_write(struct file *file,
					  const char __user *buffer,
					  size_t count, loff_t *ppos)
{
	struct oplus_tri_key_dev *chip = g_tri_key;
	unsigned int val;
	int ret;

	if (!chip)
		return -ENODEV;

	ret = kstrtouint_from_user(buffer, count, 0, &val);
	if (ret)
		return ret;

	chip->debug_level = val;
	return count;
}

static const struct proc_ops proc_hall_debug_info_ops = {
	.proc_read = proc_hall_debug_info_read,
	.proc_write = proc_hall_debug_info_write,
	.proc_lseek = default_llseek,
};

static int init_trikey_proc(struct oplus_tri_key_dev *chip)
{
	chip->proc_dir = proc_mkdir("tristatekey", NULL);
	if (!chip->proc_dir)
		return -ENOMEM;

	if (!proc_create("hall_data", 0444, chip->proc_dir, &proc_hall_data_ops) ||
	    !proc_create("tri_state", 0444, chip->proc_dir, &proc_tri_state_ops) ||
	    !proc_create("hall_data_calib", 0666, chip->proc_dir, &proc_hall_data_calib_ops) ||
	    !proc_create("hall_debug_info", 0644, chip->proc_dir, &proc_hall_debug_info_ops)) {
		remove_proc_subtree("tristatekey", NULL);
		chip->proc_dir = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void register_tri_key_dev_work(struct work_struct *work)
{
	struct oplus_tri_key_dev *chip =
		container_of(work, struct oplus_tri_key_dev, register_work);
	struct input_dev *idev;
	int ret;

	mutex_lock(&chip->lock);
	if (chip->initialized ||
	    !chip->sensors[DHALL_DOWN] || !chip->sensors[DHALL_UP]) {
		mutex_unlock(&chip->lock);
		return;
	}

	idev = input_allocate_device();
	if (!idev) {
		pr_err("oplus_tri_key: failed to allocate input device\n");
		mutex_unlock(&chip->lock);
		return;
	}

	idev->name = TRI_KEY_DEVICE_NAME;
	idev->id.vendor = TRI_KEY_VENDOR_ID;
	idev->id.bustype = BUS_I2C;
	input_set_capability(idev, EV_KEY, KEY_F3);

	ret = input_register_device(idev);
	if (ret) {
		pr_err("oplus_tri_key: failed to register input device (%d)\n", ret);
		input_free_device(idev);
		mutex_unlock(&chip->lock);
		return;
	}
	chip->input_dev = idev;

	ret = init_trikey_proc(chip);
	if (ret) {
		pr_err("oplus_tri_key: failed to create /proc/tristatekey (%d)\n", ret);
		goto err_input;
	}

	chip->ws = wakeup_source_register(NULL, "oplus_tri_state_key");
	if (!chip->ws) {
		ret = -ENOMEM;
		goto err_proc;
	}

	ret = hall_read_both(chip);
	if (ret < 0) {
		pr_err("oplus_tri_key: initial hall read failed (%d)\n", ret);
		goto err_ws;
	}
	pr_info("oplus_tri_key: initial hall readings down=%d up=%d\n",
		chip->dhall_data0, chip->dhall_data1);

	reboot_get_position(chip);
	ret = reupdate_threshold(chip);
	if (ret < 0)
		goto err_ws;

	report_key_value(chip);
	chip->last_position = chip->position;
	chip->initialized = true;

	ret = hall_set_detection_mode(chip, DHALL_DOWN, DETECTION_MODE_INTERRUPT);
	if (ret < 0) {
		pr_err("oplus_tri_key: failed to enable DOWN hall IRQ (%d)\n", ret);
		goto err_init;
	}

	ret = hall_set_detection_mode(chip, DHALL_UP, DETECTION_MODE_INTERRUPT);
	if (ret < 0) {
		pr_err("oplus_tri_key: failed to enable UP hall IRQ (%d)\n", ret);
		hall_set_detection_mode(chip, DHALL_DOWN, DETECTION_MODE_POLLING);
		goto err_init;
	}

	pr_info("oplus_tri_key: initialized (down=%s up=%s state=%d)\n",
		chip->sensors[DHALL_DOWN]->name,
		chip->sensors[DHALL_UP]->name,
		chip->state);
	mutex_unlock(&chip->lock);
	return;

err_init:
	chip->initialized = false;
err_ws:
	wakeup_source_unregister(chip->ws);
	chip->ws = NULL;
err_proc:
	remove_proc_subtree("tristatekey", NULL);
	chip->proc_dir = NULL;
err_input:
	input_unregister_device(chip->input_dev);
	chip->input_dev = NULL;
	mutex_unlock(&chip->lock);
}

void oplus_tri_key_irq_notify(struct oplus_hall_sensor *sensor)
{
	struct oplus_tri_key_dev *chip = g_tri_key;

	if (!chip || !chip->initialized)
		return;

	if (chip->ws)
		__pm_wakeup_event(chip->ws, 2000);

	schedule_work(&chip->dwork);
}

int oplus_tri_key_register_hall(struct oplus_hall_sensor *sensor)
{
	struct oplus_tri_key_dev *chip;

	if (!sensor || !sensor->ops || sensor->id >= DHALL_MAX)
		return -EINVAL;

	mutex_lock(&g_tri_key_reg_lock);

	if (!g_tri_key) {
		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			mutex_unlock(&g_tri_key_reg_lock);
			return -ENOMEM;
		}
		mutex_init(&chip->lock);
		INIT_WORK(&chip->register_work, register_tri_key_dev_work);
		INIT_WORK(&chip->dwork, tri_key_dev_work);
		INIT_WORK(&chip->timeout_work, tri_key_timeout_work_func);
		hrtimer_setup(&chip->timer, tri_key_status_timeout,
			      CLOCK_MONOTONIC, HRTIMER_MODE_REL);

		chip->last_position = -1;
		chip->last_interf = -1;
		chip->timer_sec = 1;
		chip->up_mid_tol = 15;
		chip->up_tolerance = 15;
		chip->mid_down_tol = 15;
		chip->down_tolerance = 15;

		g_tri_key = chip;
	} else {
		chip = g_tri_key;
	}

	mutex_lock(&chip->lock);
	if (chip->sensors[sensor->id]) {
		mutex_unlock(&chip->lock);
		mutex_unlock(&g_tri_key_reg_lock);
		return -EBUSY;
	}

	chip->sensors[sensor->id] = sensor;
	pr_info("oplus_tri_key: registered %s (slot %d)\n",
		sensor->name, sensor->id);

	if (chip->sensors[DHALL_DOWN] && chip->sensors[DHALL_UP])
		schedule_work(&chip->register_work);

	mutex_unlock(&chip->lock);
	mutex_unlock(&g_tri_key_reg_lock);
	return 0;
}

void oplus_tri_key_unregister_hall(struct oplus_hall_sensor *sensor)
{
	struct oplus_tri_key_dev *chip;
	bool teardown_core = false;

	if (!sensor || sensor->id >= DHALL_MAX)
		return;

	mutex_lock(&g_tri_key_reg_lock);
	chip = g_tri_key;
	if (!chip) {
		mutex_unlock(&g_tri_key_reg_lock);
		return;
	}

	mutex_lock(&chip->lock);
	if (chip->sensors[sensor->id] == sensor) {
		if (chip->initialized) {
			hall_set_detection_mode(chip, DHALL_DOWN, DETECTION_MODE_POLLING);
			hall_set_detection_mode(chip, DHALL_UP, DETECTION_MODE_POLLING);
			chip->initialized = false;
		}
		chip->sensors[sensor->id] = NULL;
	}
	teardown_core = (!chip->sensors[DHALL_DOWN] && !chip->sensors[DHALL_UP]);
	mutex_unlock(&chip->lock);

	cancel_work_sync(&chip->register_work);
	hrtimer_cancel(&chip->timer);
	cancel_work_sync(&chip->dwork);
	cancel_work_sync(&chip->timeout_work);

	mutex_lock(&chip->lock);
	if (chip->ws) {
		wakeup_source_unregister(chip->ws);
		chip->ws = NULL;
	}
	if (chip->proc_dir) {
		remove_proc_subtree("tristatekey", NULL);
		chip->proc_dir = NULL;
	}
	if (chip->input_dev) {
		input_unregister_device(chip->input_dev);
		chip->input_dev = NULL;
	}
	mutex_unlock(&chip->lock);

	if (teardown_core) {
		g_tri_key = NULL;
		kfree(chip);
	}
	mutex_unlock(&g_tri_key_reg_lock);
}

static int __init oplus_tri_key_init(void)
{
	int ret;

	ret = i2c_add_driver(&ist8801_hall_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&mxm1120_hall_driver);
	if (ret) {
		i2c_del_driver(&ist8801_hall_driver);
		return ret;
	}

	return 0;
}

static void __exit oplus_tri_key_exit(void)
{
	i2c_del_driver(&mxm1120_hall_driver);
	i2c_del_driver(&ist8801_hall_driver);
}

module_init(oplus_tri_key_init);
module_exit(oplus_tri_key_exit);

MODULE_DESCRIPTION("OPlus dual-Hall Alert Slider (tri-state key) driver");
MODULE_LICENSE("GPL");
