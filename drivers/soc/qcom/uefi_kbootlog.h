/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __QCOM_UEFI_KBOOTLOG_H__
#define __QCOM_UEFI_KBOOTLOG_H__

#include <linux/types.h>

#ifdef CONFIG_QCOM_UEFI_KBOOTLOG
ssize_t qcom_kbootlog_copy(char *dst, size_t max);
#else
static inline ssize_t qcom_kbootlog_copy(char *dst, size_t max)
{
	return 0;
}
#endif

#endif /* __QCOM_UEFI_KBOOTLOG_H__ */
