/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_GOODIX_FP_NOTIFY_H
#define _LINUX_GOODIX_FP_NOTIFY_H

#include <linux/types.h>

#ifdef CONFIG_FINGERPRINT_GOODIX_FP
void gf_notify_ui_ready(bool ready);
void gf_notify_screen_state(bool on);
#else
static inline void gf_notify_ui_ready(bool ready) {}
static inline void gf_notify_screen_state(bool on) {}
#endif

#endif /* _LINUX_GOODIX_FP_NOTIFY_H */
