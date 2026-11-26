/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_SYNAPTICS_TCM_NOTIFY_H
#define _LINUX_SYNAPTICS_TCM_NOTIFY_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_TOUCHSCREEN_SYNAPTICS_S3908)
void synaptics_tcm_notify_screen_state(bool on);
#else
static inline void synaptics_tcm_notify_screen_state(bool on) {}
#endif

#endif /* _LINUX_SYNAPTICS_TCM_NOTIFY_H */
