/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 *
 * Dual-Hall Alert Slider (tri-state key) driver for OnePlus SM8250 devices.
 */

#ifndef _OPLUS_TRI_KEY_H
#define _OPLUS_TRI_KEY_H

#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/types.h>

enum oplus_hall_id {
	DHALL_DOWN = 0,
	DHALL_UP = 1,
	DHALL_MAX,
};

enum oplus_hall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT = 1,
};

enum oplus_tri_key_position {
	UP_STATE = 0,
	DOWN_STATE = 1,
	MID_STATE = 2,
};

struct oplus_hall_sensor;

struct oplus_hall_ops {
	int (*get_data)(struct oplus_hall_sensor *sensor, short *data);
	int (*set_detection_mode)(struct oplus_hall_sensor *sensor, u8 mode);
	int (*enable_irq)(struct oplus_hall_sensor *sensor, bool enable);
	int (*clear_irq)(struct oplus_hall_sensor *sensor);
	int (*update_threshold)(struct oplus_hall_sensor *sensor,
				int position, short lowthd, short highthd);
	bool (*is_power_on)(struct oplus_hall_sensor *sensor);
};

struct oplus_hall_sensor {
	enum oplus_hall_id id;
	const char *name;
	struct device *dev;
	const struct oplus_hall_ops *ops;
	atomic_t irq_masked;
	int irq;
	bool irq_requested;
};

int oplus_tri_key_register_hall(struct oplus_hall_sensor *sensor);
void oplus_tri_key_unregister_hall(struct oplus_hall_sensor *sensor);
void oplus_tri_key_irq_notify(struct oplus_hall_sensor *sensor);

extern struct i2c_driver ist8801_hall_driver;
extern struct i2c_driver mxm1120_hall_driver;

#endif /* _OPLUS_TRI_KEY_H */
