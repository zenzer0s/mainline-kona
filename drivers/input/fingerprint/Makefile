# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_FINGERPRINT_GOODIX_FP) += goodix_fp.o
goodix_fp-objs := gf_spi.o platform.o netlink.o
ccflags-y += -DUSE_PLATFORM_BUS -DGF_NETLINK_ENABLE
