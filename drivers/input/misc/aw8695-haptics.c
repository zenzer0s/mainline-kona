// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 Luca Weiss <luca.weiss@fairphone.com>
 *
 * Partially based on vendor driver:
 *   Copyright (c) 2018 AWINIC Technology CO., LTD
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ktime.h>
#include <linux/leds.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/proc_fs.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define AW8624_CHIPID			0x24
#define AW8695_CHIPID			0x95
#define AW8697_CHIPID			0x97
#define AW8695_RESET			0xaa
/* Default of BASE_ADDR* registers */
#define AW8695_RAM_BASE_ADDR		0x800

#define AW8695_HIGH_MASK		GENMASK(15, 8)
#define AW8695_LOW_MASK			GENMASK(7, 0)

/* Chip ID */
#define AW8695_ID				0x00

/* System Status */
#define AW8695_SYSST				0x01
#define AW8695_SYSST_BSTERRS			BIT(7)
#define AW8695_SYSST_OVS			BIT(6)
#define AW8695_SYSST_UVLS			BIT(5)
#define AW8695_SYSST_FF_AES			BIT(4)
#define AW8695_SYSST_FF_AFS			BIT(3)
#define AW8695_SYSST_OCDS			BIT(2)
#define AW8695_SYSST_OTS			BIT(1)
#define AW8695_SYSST_DONES			BIT(0)

/* System Interrupt */
#define AW8695_SYSINT				0x02
#define AW8695_SYSINT_BSTERRI			BIT(7)
#define AW8695_SYSINT_OVI			BIT(6)
#define AW8695_SYSINT_UVLI			BIT(5)
#define AW8695_SYSINT_FF_AEI			BIT(4)
#define AW8695_SYSINT_FF_AFI			BIT(3)
#define AW8695_SYSINT_OCDI			BIT(2)
#define AW8695_SYSINT_OTI			BIT(1)
#define AW8695_SYSINT_DONEI			BIT(0)

/* System Interrupt Mask */
#define AW8695_SYSINTM				0x03
#define AW8695_SYSINTM_BSTERR_OFF		BIT(7)
#define AW8695_SYSINTM_OV_OFF			BIT(6)
#define AW8695_SYSINTM_UVLO_OFF			BIT(5)
#define AW8695_SYSINTM_FF_AE_OFF		BIT(4)
#define AW8695_SYSINTM_FF_AF_OFF		BIT(3)
#define AW8695_SYSINTM_OCD_OFF			BIT(2)
#define AW8695_SYSINTM_OT_OFF			BIT(1)
#define AW8695_SYSINTM_DONE_OFF			BIT(0)

/* System Control */
#define AW8695_SYSCTRL				0x04
#define AW8695_SYSCTRL_WAVDAT_MODE_MASK		GENMASK(7, 6)
#define AW8695_SYSCTRL_WAVDAT_MODE_4X		(3<<6)
#define AW8695_SYSCTRL_WAVDAT_MODE_2X		(0<<6)
#define AW8695_SYSCTRL_WAVDAT_MODE_1X		(1<<6)

#define AW8695_SYSCTRL_RAMINIT_EN		BIT(5)

#define AW8695_SYSCTRL_PLAY_MODE_MASK		GENMASK(3, 2)
#define AW8695_SYSCTRL_PLAY_MODE_CONT		(2<<2)
#define AW8695_SYSCTRL_PLAY_MODE_RTP		(1<<2)
#define AW8695_SYSCTRL_PLAY_MODE_RAM		(0<<2)

#define AW8695_SYSCTRL_BST_MODE_MASK		GENMASK(1, 1)
#define AW8695_SYSCTRL_BST_MODE_BOOST		(1<<1)
#define AW8695_SYSCTRL_BST_MODE_BYPASS		(0<<1)

#define AW8695_SYSCTRL_WORK_MODE_MASK		GENMASK(0, 0)
#define AW8695_SYSCTRL_STANDBY			(1<<0)
#define AW8695_SYSCTRL_ACTIVE			(0<<0)

/* Process Control */
#define AW8695_GO				0x05
#define AW8695_GO_ENABLE			BIT(0)

/* RTP Mode Data */
#define AW8695_RTP_DATA				0x06

/* Waveform #1 */
#define AW8695_WAVSEQ1				0x07
#define AW8695_WAVSEQ1_WAIT			BIT(7)
#define AW8695_WAVSEQ1_WAV_FRM_SEQ1_MASK	GENMASK(6, 0)

/* Waveform #2 */
#define AW8695_WAVSEQ2				0x08
#define AW8695_WAVSEQ2_WAIT			BIT(7)
#define AW8695_WAVSEQ2_WAV_FRM_SEQ2_MASK	GENMASK(6, 0)

/* Waveform #3 */
#define AW8695_WAVSEQ3				0x09
#define AW8695_WAVSEQ3_WAIT			BIT(7)
#define AW8695_WAVSEQ3_WAV_FRM_SEQ3_MASK	GENMASK(6, 0)

/* Waveform #4 */
#define AW8695_WAVSEQ4				0x0a
#define AW8695_WAVSEQ4_WAIT			BIT(7)
#define AW8695_WAVSEQ4_WAV_FRM_SEQ4_MASK	GENMASK(6, 0)

/* Waveform #5 */
#define AW8695_WAVSEQ5				0x0b
#define AW8695_WAVSEQ5_WAIT			BIT(7)
#define AW8695_WAVSEQ5_WAV_FRM_SEQ5_MASK	GENMASK(6, 0)

/* Waveform #6 */
#define AW8695_WAVSEQ6				0x0c
#define AW8695_WAVSEQ6_WAIT			BIT(7)
#define AW8695_WAVSEQ6_WAV_FRM_SEQ6_MASK	GENMASK(6, 0)

/* Waveform #7 */
#define AW8695_WAVSEQ7				0x0d
#define AW8695_WAVSEQ7_WAIT			BIT(7)
#define AW8695_WAVSEQ7_WAV_FRM_SEQ7_MASK	GENMASK(6, 0)

/* Waveform #8 */
#define AW8695_WAVSEQ8				0x0e
#define AW8695_WAVSEQ8_WAIT			BIT(7)
#define AW8695_WAVSEQ8_WAV_FRM_SEQ8_MASK	GENMASK(6, 0)

/* Waveform Loop #1 */
#define AW8695_WAVLOOP1				0x0f
#define AW8695_WAVLOOP1_SEQ1_MASK		GENMASK(7, 4)
#define AW8695_WAVLOOP1_SEQ2_MASK		GENMASK(3, 0)

/* Waveform Loop #2 */
#define AW8695_WAVLOOP2				0x10
#define AW8695_WAVLOOP2_SEQ3_MASK		GENMASK(7, 4)
#define AW8695_WAVLOOP2_SEQ4_MASK		GENMASK(3, 0)

/* Waveform Loop #3 */
#define AW8695_WAVLOOP3				0x11
#define AW8695_WAVLOOP3_SEQ5_MASK		GENMASK(7, 4)
#define AW8695_WAVLOOP3_SEQ6_MASK		GENMASK(3, 0)

/* Waveform Loop #4 */
#define AW8695_WAVLOOP4				0x12
#define AW8695_WAVLOOP4_SEQ7_MASK		GENMASK(7, 4)
#define AW8695_WAVLOOP4_SEQ8_MASK		GENMASK(3, 0)

/* Main Loop */
#define AW8695_MAIN_LOOP			0x13

/* TRIG1 Edge Waveform #1 */
#define AW8695_TRG1_WAV_P			0x14

/* TRIG2 Edge Waveform #1 */
#define AW8695_TRG2_WAV_P			0x15

/* TRIG3 Edge Waveform #1 */
#define AW8695_TRG3_WAV_P			0x16

/* TRIG1 Edge Waveform #2 */
#define AW8695_TRG1_WAV_N			0x17

/* TRIG2 Edge Waveform #2 */
#define AW8695_TRG2_WAV_N			0x18

/* TRIG3 Edge Waveform #2 */
#define AW8695_TRG3_WAV_N			0x19

#define AW8695_TRG_PRIO				0x1a
#define AW8695_PLAYPRIO_GO_MASK			GENMASK(7, 6)
#define AW8695_PLAYPRIO_TRIG3_MASK		GENMASK(5, 4)
#define AW8695_PLAYPRIO_TRIG2_MASK		GENMASK(3, 2)
#define AW8695_PLAYPRIO_TRIG1_MASK		GENMASK(1, 0)

/* Trig Pins Config */
#define AW8695_TRG_CFG1				0x1b
#define AW8695_TRGCFG1_TRG3_POLAR_NEG		BIT(5)
#define AW8695_TRGCFG1_TRG3_EDGE_POS		BIT(4)
#define AW8695_TRGCFG1_TRG2_POLAR_NEG		BIT(3)
#define AW8695_TRGCFG1_TRG2_EDGE_POS		BIT(2)
#define AW8695_TRGCFG1_TRG1_POLAR_NEG		BIT(1)
#define AW8695_TRGCFG1_TRG1_EDGE_POS		BIT(0)

/* Trig Pins Config */
#define AW8695_TRG_CFG2				0x1c
#define AW8695_TRGCFG2_TRG3_ENABLE		BIT(2)
#define AW8695_TRGCFG2_TRG2_ENABLE		BIT(1)
#define AW8695_TRGCFG2_TRG1_ENABLE		BIT(0)

/* Debug Control */
#define AW8695_DBGCTRL				0x20
#define AW8695_DBGCTRL_INT_EDGE_MODE_MASK	GENMASK(3, 3)
#define AW8695_DBGCTRL_INT_EDGE_MODE_POS	(1<<3)
#define AW8695_DBGCTRL_INT_EDGE_MODE_BOTH	(0<<3)
#define AW8695_DBGCTRL_INT_MODE_MASK		GENMASK(2, 2)
#define AW8695_DBGCTRL_INT_MODE_EDGE		(1<<2)
#define AW8695_DBGCTRL_INT_MODE_LEVEL		(0<<2)

/* High Five Bits of Wave SRAM */
#define AW8695_BASE_ADDRH			0x21

/* Low Eight Bits of Wave SRAM */
#define AW8695_BASE_ADDRL			0x22

/* High Four Bits of FIFO AE */
#define AW8695_FIFO_AEH				0x23

/* Low Eight Bits of FIFO AE */
#define AW8695_FIFO_AEL				0x24

/* High Four Bits of FIFO AF */
#define AW8695_FIFO_AFH				0x25

/* Low Eight Bits of FIFO AF */
#define AW8695_FIFO_AFL				0x26

#define AW8695_WAKE_DLY				0x27

#define AW8695_START_DLY			0x28

#define AW8695_END_DLY_H			0x29

#define AW8695_END_DLY_L			0x2a

/* Global Control Data */
#define AW8695_DATCTRL				0x2b
#define AW8695_DATCTRL_FC_MASK			GENMASK(6, 6)
#define AW8695_DATCTRL_FC_1000HZ		(3<<6)
#define AW8695_DATCTRL_FC_800HZ			(3<<6)
#define AW8695_DATCTRL_FC_600HZ			(1<<6)
#define AW8695_DATCTRL_FC_400HZ			(0<<6)

#define AW8695_DATCTRL_LPF_ENABLE_MASK		GENMASK(5, 5)
#define AW8695_DATCTRL_LPF_ENABLE		(1<<5)
#define AW8695_DATCTRL_LPF_DISABLE		(0<<5)

#define AW8695_DATCTRL_WAKEMODE_ENABLE_MASK	GENMASK(0, 0)
#define AW8695_DATCTRL_WAKEMODE_ENABLE		(1<<0)
#define AW8695_DATCTRL_WAKEMODE_DISABLE		(0<<0)

#define AW8695_PWMDEL				0x2c

/* PWM Output Protect Configuration */
#define AW8695_PWMPRC				0x2d
#define AW8695_PWMPRC_PRC_ENABLE		BIT(7)

/* PWM Debug */
#define AW8695_PWMDBG				0x2e
#define AW8695_PWMDBG_PWM_MODE_MASK		GENMASK(6, 5)
#define AW8695_PWMDBG_PWM_12K			(3<<5)
#define AW8695_PWMDBG_PWM_24K			(2<<5)
#define AW8695_PWMDBG_PWM_48K			(0<<5)

#define AW8695_LDOCTRL				0x2f

/* Debug Status */
#define AW8695_DBGSTAT				0x30
#define AW8695_DBGSTAT_FF_EMPTY			BIT(0)

/* Boost Debug #1 */
#define AW8695_BSTDBG1				0x31

/* Boost Debug #2 */
#define AW8695_BSTDBG2				0x32

/* Boost Debug #3 */
#define AW8695_BSTDBG3				0x33

/* Boost Config */
#define AW8695_BSTCFG				0x34
#define AW8695_BSTCFG_PEAKCUR_MASK		GENMASK(2, 0)
#define AW8695_BSTCFG_PEAKCUR_4A		(7<<0)
#define AW8695_BSTCFG_PEAKCUR_3P75A		(6<<0)
#define AW8695_BSTCFG_PEAKCUR_3P5A		(5<<0)
#define AW8695_BSTCFG_PEAKCUR_3P25A		(4<<0)
#define AW8695_BSTCFG_PEAKCUR_3A		(3<<0)
#define AW8695_BSTCFG_PEAKCUR_2P5A		(2<<0)
#define AW8695_BSTCFG_PEAKCUR_2A		(1<<0)
#define AW8695_BSTCFG_PEAKCUR_1P5A		(0<<0)

#define AW8695_ANADBG				0x35
#define AW8695_ANADBG_IOC_MASK			GENMASK(3, 2)
#define AW8695_ANADBG_IOC_4P65A			(3<<2)
#define AW8695_ANADBG_IOC_4P15A			(2<<2)
#define AW8695_ANADBG_IOC_3P65A			(1<<2)
#define AW8695_ANADBG_IOC_3P15A			(0<<2)

#define AW8695_ANACTRL				0x36
#define AW8695_ANACTRL_LRA_SRC_MASK		GENMASK(5, 5)
#define AW8695_ANACTRL_LRA_SRC_REG		(1<<5)
#define AW8695_ANACTRL_LRA_SRC_EFUSE		(0<<5)
#define AW8695_ANACTRL_HD_PD_MASK		GENMASK(3, 3)
#define AW8695_ANACTRL_HD_PD_EN			(1<<3)
#define AW8695_ANACTRL_HD_HZ_EN			(0<<3)

#define AW8695_CPDBG				0x37

#define AW8695_GLBDBG				0x38

/* Data Gain */
#define AW8695_DATDBG				0x39

/* Boost Debug #4 */
#define AW8695_BSTDBG4				0x3a
#define AW8695_BSTDBG4_BSTVOL_MASK		GENMASK(5, 1)

/* Boost Debug #5 */
#define AW8695_BSTDBG5				0x3b

/* Boost Debug #6 */
#define AW8695_BSTDBG6				0x3c

#define AW8695_HDRVDBG				0x3d

/* Waveform Protect Level */
#define AW8695_PRLVL				0x3e
#define AW8695_PRLVL_PR_ENABLE			BIT(7)
#define AW8695_PRLVL_PRLVL_MASK			GENMASK(6, 0)

/* Waveform Protect Period */
#define AW8695_PRTIME				0x3f

/* SRAM Address 0xhigh */
#define AW8695_RAMADDRH				0x40

/* SRAM Address 0xlow */
#define AW8695_RAMADDRL				0x41

/* SRAM Data */
#define AW8695_RAMDATA				0x42

#define AW8695_GLB_STATE			0x46

#define AW8695_BST_AUTO				0x47
#define AW8695_BST_AUTO_BST_AUTOSW_MASK		GENMASK(2, 2)
#define AW8695_BST_AUTO_BST_AUTOMATIC_BOOST	(1<<2)
#define AW8695_BST_AUTO_BST_MANUAL_BOOST	(0<<2)
#define AW8695_BST_AUTO_BST_RTP_ENABLE		BIT(1)
#define AW8695_BST_AUTO_BST_RAM_ENABLE		BIT(0)

/* CONT Mode Control */
#define AW8695_CONT_CTRL			0x48
#define AW8695_CONT_CTRL_ZC_DETEC_ENABLE	BIT(7)
#define AW8695_CONT_CTRL_WAIT_PERIOD_MASK	GENMASK(6, 5)
#define AW8695_CONT_CTRL_WAIT_8PERIOD		(3<<5)
#define AW8695_CONT_CTRL_WAIT_4PERIOD		(2<<5)
#define AW8695_CONT_CTRL_WAIT_2PERIOD		(1<<5)
#define AW8695_CONT_CTRL_WAIT_1PERIOD		(0<<5)
#define AW8695_CONT_CTRL_MODE_MASK		GENMASK(4, 4)
#define AW8695_CONT_CTRL_BY_DRV_TIME		(1<<4)
#define AW8695_CONT_CTRL_BY_GO_SIGNAL		(0<<4)
#define AW8695_CONT_CTRL_EN_CLOSE_MASK		GENMASK(3, 3)
#define AW8695_CONT_CTRL_CLOSE_PLAYBACK		(1<<3)
#define AW8695_CONT_CTRL_OPEN_PLAYBACK		(0<<3)
#define AW8695_CONT_CTRL_F0_DETECT_ENABLE	BIT(2)
#define AW8695_CONT_CTRL_O2C_ENABLE		BIT(1)
#define AW8695_CONT_CTRL_AUTO_BRK_ENABLE	BIT(0)

/* High 8 Bits Pre Setting f0 Value */
#define AW8695_F_PRE_H				0x49

/* Low 8 Bits Pre Setting f0 Value */
#define AW8695_F_PRE_L				0x4a

/* High 4 Bits of Delay Time Setting */
#define AW8695_TD_H				0x4b

/* Low 8 Bits of Delay Time Setting */
#define AW8695_TD_L				0x4c

#define AW8695_TSET				0x4d

#define AW8695_TRIM_LRA				0x5b

#define AW8695_R_SPARE				0x5d

#define AW8695_D2SCFG				0x5e
#define AW8695_D2SCFG_CLK_ADC_MASK		GENMASK(7, 5)
#define AW8695_D2SCFG_CLK_ASC_0P09375MHZ	(7<<5)
#define AW8695_D2SCFG_CLK_ASC_0P1875MHZ		(6<<5)
#define AW8695_D2SCFG_CLK_ASC_0P375MHZ		(5<<5)
#define AW8695_D2SCFG_CLK_ASC_0P75MHZ		(4<<5)
#define AW8695_D2SCFG_CLK_ASC_1P5MHZ		(3<<5)
#define AW8695_D2SCFG_CLK_ASC_3MHZ		(2<<5)
#define AW8695_D2SCFG_CLK_ASC_6MHZ		(1<<5)
#define AW8695_D2SCFG_CLK_ASC_12MHZ		(0<<5)

/* Detection Control */
#define AW8695_DETCTRL				0x5f
#define AW8695_DETCTRL_RL_OS_MASK		GENMASK(6, 6)
#define AW8695_DETCTRL_RL_DETECT		(1<<6)
#define AW8695_DETCTRL_OS_DETECT		(0<<6)
#define AW8695_DETCTRL_PROTECT_MASK		GENMASK(5, 5)
#define AW8695_DETCTRL_PROTECT_NO_ACTION	(1<<5)
#define AW8695_DETCTRL_PROTECT_SHUTDOWN		(0<<5)
#define AW8695_DETCTRL_ADO_SLOT_MODE_ENABLE	BIT(4)
#define AW8695_DETCTRL_VBAT_GO_ENABLE		BIT(1)
#define AW8695_DETCTRL_DIAG_GO_ENABLE		BIT(0)

/* Detected RL of LRA */
#define AW8695_RLDET				0x60

/* Detected Offset of LRA */
#define AW8695_OSDET				0x61

/* Detected VBAT */
#define AW8695_VBATDET				0x62

#define AW8695_TESTDET				0x63

#define AW8695_DETLO				0x64

#define AW8695_BEMFDBG				0x65

/* ADC Test */
#define AW8695_ADCTEST				0x66
#define AW8695_ADCTEST_VBAT_MODE_MASK		GENMASK(6, 6)
#define AW8695_ADCTEST_VBAT_HW_COMP		(1<<6)
#define AW8695_ADCTEST_VBAT_SW_COMP		(0<<6)

#define AW8695_BEMFTEST				0x67

/* High 8 Bits Detected f0 Value */
#define AW8695_F_LRA_F0_H			0x68

/* Low 8 Bits Detected f0 Value */
#define AW8695_F_LRA_F0_L			0x69

/* High 8 Bits CONT_ENG Gotten f0 Value */
#define AW8695_F_LRA_CONT_H			0x6a

/* Low 8 Bits CONT_ENG Gotten f0 Value */
#define AW8695_F_LRA_CONT_L			0x6b

#define AW8695_WAIT_VOL_MP			0x6d

#define AW8695_WAIT_VOL_MN			0x6f

#define AW8695_BEMF_VOL_H			0x70

#define AW8695_BEMF_VOL_L			0x71

/* Zero Cross Threshold High 8 Bits Configuration */
#define AW8695_ZC_THRSH_H			0x72

/* Zero Cross Threshold Low 8 Bits Configuration */
#define AW8695_ZC_THRSH_L			0x73

#define AW8695_BEMF_VTHH_H			0x74

#define AW8695_BEMF_VTHH_L			0x75

#define AW8695_BEMF_VTHL_H			0x76

#define AW8695_BEMF_VTHL_L			0x77

/* BEMF Detection Cycles Configuration */
#define AW8695_BEMF_NUM				0x78
#define AW8695_BEMF_NUM_BRK_MASK		GENMASK(3, 0)

/* Drive Time Setting */
#define AW8695_DRV_TIME				0x79

/* Non Zero Cross Time Setting */
#define AW8695_TIME_NZC				0x7a

/* Drive Level Setting */
#define AW8695_DRV_LVL				0x7b

/* Drive Level for Overdrive Setting */
#define AW8695_DRV_LVL_OV			0x7c

/* Number Configuration for F0 Trace #1 */
#define AW8695_NUM_F0_1				0x7d
#define AW8695_NUM_F0_1_PRE_MASK		GENMASK(7, 4)
#define AW8695_NUM_F0_1_WAIT_MASK		GENMASK(3, 0)

/* Number Configuration for F0 Trace #2 */
#define AW8695_NUM_F0_2				0x7e

/* Number Configuration for F0 Trace #3 */
#define AW8695_NUM_F0_3				0x7f

#define AW8695_MAX_REG				0x7f

/* AAC RichTap ioctl / mmap streaming ABI (compatible with libaacvibrator.so) */
#define RICHTAP_HAPTIC_HV			0x05
#define MMAP_BUF_DATA_VALID			0x55
#define MMAP_BUF_DATA_FINISHED			0xAA
#define MMAP_BUF_DATA_INVALID			0xFF

#define RICHTAP_IOCTL_GROUP			0x52
#define RICHTAP_GET_HWINFO			_IO(RICHTAP_IOCTL_GROUP, 0x03)
#define RICHTAP_SET_FREQ			_IO(RICHTAP_IOCTL_GROUP, 0x04)
#define RICHTAP_SETTING_GAIN			_IO(RICHTAP_IOCTL_GROUP, 0x05)
#define RICHTAP_OFF_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x06)
#define RICHTAP_TIMEOUT_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x07)
#define RICHTAP_RAM_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x08)
#define RICHTAP_RTP_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x09)
#define RICHTAP_STREAM_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x0A)
#define RICHTAP_UPDATE_RAM			_IO(RICHTAP_IOCTL_GROUP, 0x10)
#define RICHTAP_GET_F0				_IO(RICHTAP_IOCTL_GROUP, 0x11)
#define RICHTAP_STOP_MODE			_IO(RICHTAP_IOCTL_GROUP, 0x12)
/* Atomic F0-locked SRAM playback: arg = wav[6:0] | gain[15:8] | bst_vol[20:16] */
#define RICHTAP_PLAY_SRAM			_IO(RICHTAP_IOCTL_GROUP, 0x20)

#define RICHTAP_MMAP_BUF_SIZE			1000
#define RICHTAP_MMAP_PAGE_ORDER			2
#define RICHTAP_MMAP_BUF_SUM			16
#define RICHTAP_RTP_MAX_CHUNK			512
#define AW8695_PM_QOS_LATENCY_US		400
#define AW8695_OSC_CALI_FALLBACK_LEN		48000
#define AW8695_RAM_FW_DELAY_MS			5000
#define AW8695_VBAT_MIN_MV			3000
#define AW8695_VBAT_MAX_MV			4500
#define AW8695_HAPTIC_SEQ_SIZE			8
#define AW8695_HAPTIC_LOOP_SIZE			8
#define AW8695_HAPTIC_DEFAULT_GAIN		0x80
#define AW8695_HAPTIC_DEFAULT_VMAX		0x11
#define AW8695_HAPTIC_HIGH_VMAX_REG		0x16

enum aw8695_vibration_style {
	AW8695_VIBRATION_STYLE_CRISP = 0,
	AW8695_VIBRATION_STYLE_SOFT = 1,
};

#pragma pack(4)
struct mmap_buf_format {
	u8 status;
	u8 bit;
	s16 length;
	u32 reserve;
	struct mmap_buf_format *kernel_next;
	struct mmap_buf_format *user_next;
	u8 data[RICHTAP_MMAP_BUF_SIZE];
};
#pragma pack()

static_assert(sizeof(struct mmap_buf_format) == 1024);

enum aw8695_work_mode {
	AW8695_STANDBY_MODE,
	AW8695_RAM_MODE,
	AW8695_RAM_LOOP_MODE,
	AW8695_RTP_MODE,
	AW8695_TRIG_MODE,
	AW8695_CONT_MODE,
};

struct aw8695_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	u8 chipid;
	bool running;
	struct work_struct play_work;
	/* Parameters from devicetree */
	u32 f0_preset;
	u32 f0_coefficient;
	u32 f0_cali_percent;
	u32 drive_level;
	u32 f0_det_play;
	u32 f0_det_wait;
	u32 f0_det_repeat;
	u32 f0_det_trace;
	u8 boost_debug[3];
	u8 tset;
	u8 r_spare;
	u32 bemf_vthh;
	u32 bemf_vthl;
	/* Calibrated resonant frequency (Hz * 10, e.g. 1700 = 170.0 Hz) */
	u32 f0;
	u8 f0_cali_lra;
	/* 24 kHz oscillator drift calibration for RTP streaming */
	u8 osc_cali_lra;
	bool osc_calibrated;
	s64 osc_microsecond;
	/* SRAM waveform firmware cache */
	u8 *ram_fw_data;
	size_t ram_fw_len;
	u16 ram_base_addr;
	u16 ram_checksum;
	bool ram_fw_loaded;
	u8 vibration_style;
	struct delayed_work ram_work;
	/* Legacy /sys/class/leds/vibrator and /proc/vibrator/touch_style */
	struct led_classdev cdev;
	struct hrtimer vibrator_timer;
	struct work_struct vibrator_work;
	struct proc_dir_entry *proc_dir;
	struct proc_dir_entry *proc_touch_style;
	u8 seq[AW8695_HAPTIC_SEQ_SIZE];
	u8 loop[AW8695_HAPTIC_LOOP_SIZE];
	u8 gain;
	u8 vmax;
	u32 duration;
	u32 state;
	u32 vbat;
	u32 lra_resistance;
	/* CPU latency PM QoS for jitter-free RTP FIFO pacing */
	struct pm_qos_request pm_qos_req;
	struct mutex qos_lock;
	bool pm_qos_added;
	/* AAC RichTap /dev/awinic_haptic streaming state */
	struct mutex ioctl_lock;
	struct miscdevice miscdev;
	struct workqueue_struct *rtp_wq;
	struct work_struct haptic_rtp_work;
	struct mmap_buf_format *start_buf;
	u8 *rtp_ptr;
	u8 rtp_tx_buf[RICHTAP_RTP_MAX_CHUNK + 1];
	atomic_t done_flag;
	bool haptic_rtp_mode;
};

/*
 * Sine wave representing the magnitude of the drive to be used.
 * Data is encoded in two's complement.
 *   round(84 * sin(x / 16.25))
 */
static const u8 aw8695_sine_waveform[] = {
	0x00, 0x05, 0x0a, 0x0f, 0x14, 0x19, 0x1e, 0x23, 0x28, 0x2c, 0x30, 0x35,
	0x39, 0x3c, 0x40, 0x43, 0x46, 0x49, 0x4b, 0x4d, 0x4f, 0x51, 0x52, 0x53,
	0x54, 0x54, 0x54, 0x54, 0x53, 0x52, 0x51, 0x4f, 0x4d, 0x4b, 0x49, 0x46,
	0x43, 0x40, 0x3c, 0x39, 0x35, 0x31, 0x2c, 0x28, 0x23, 0x1f, 0x1a, 0x15,
	0x10, 0x0b, 0x05, 0x00, 0xfb, 0xf6, 0xf1, 0xec, 0xe7, 0xe2, 0xdd, 0xd9,
	0xd4, 0xd0, 0xcc, 0xc8, 0xc4, 0xc0, 0xbd, 0xba, 0xb7, 0xb5, 0xb3, 0xb1,
	0xaf, 0xae, 0xad, 0xac, 0xac, 0xac, 0xac, 0xad, 0xae, 0xaf, 0xb1, 0xb2,
	0xb5, 0xb7, 0xba, 0xbd, 0xc0, 0xc3, 0xc7, 0xcb, 0xcf, 0xd3, 0xd8, 0xdc,
	0xe1, 0xe6, 0xeb, 0xf0, 0xf5, 0xfa
};

/*
 * Header that gets written to AW8695 SRAM that describes the available
 * waveforms being transferred afterwards.
 *
 * @version: waveform library version
 * @start_address: start address of waveform in SRAM
 * @end_address: end address of waveform in SRAM
 */
struct aw8695_sram_waveform_header {
	u8 version;
	struct {
		__be16 start_address;
		__be16 end_address;
	} __packed waveform_address[1];
} __packed;

static const struct aw8695_sram_waveform_header sram_waveform_header = {
	.version = 0x01,
	.waveform_address = {
		/* Simple sine wave defined above */
		{
			.start_address = cpu_to_be16(AW8695_RAM_BASE_ADDR +
				sizeof(struct aw8695_sram_waveform_header)),
			.end_address = cpu_to_be16(AW8695_RAM_BASE_ADDR +
				sizeof(struct aw8695_sram_waveform_header) +
				ARRAY_SIZE(aw8695_sine_waveform) - 1),
		}
	}
};

static int aw8695_interrupt_clear(struct aw8695_data *haptics)
{
	unsigned int read_buf;

	/* Clear UVLI bit by reading register */
	return regmap_read(haptics->regmap, AW8695_SYSINT, &read_buf);
}

static int aw8695_haptic_set_active(struct aw8695_data *haptics)
{
	int err;

	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_WORK_MODE_MASK, AW8695_SYSCTRL_ACTIVE);
	if (err)
		return err;

	err = aw8695_interrupt_clear(haptics);
	if (err)
		return err;

	return regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_UVLO_OFF, 0);
}

static int aw8695_set_work_mode(struct aw8695_data *haptics,
				 enum aw8695_work_mode mode)
{
	struct device *dev = &haptics->client->dev;
	int err;

	switch (mode) {
	case AW8695_STANDBY_MODE:
		err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
			AW8695_SYSINTM_UVLO_OFF, AW8695_SYSINTM_UVLO_OFF);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_WORK_MODE_MASK, AW8695_SYSCTRL_STANDBY);
		if (err)
			return err;
		break;
	case AW8695_RAM_MODE:
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_PLAY_MODE_MASK, AW8695_SYSCTRL_PLAY_MODE_RAM);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BYPASS);
		if (err)
			return err;
		err = aw8695_haptic_set_active(haptics);
		if (err)
			return err;
		if (haptics->chipid == AW8697_CHIPID) {
			err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
				AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BOOST);
			if (err)
				return err;
			usleep_range(2000, 2500);
		}
		break;
	case AW8695_RAM_LOOP_MODE:
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_PLAY_MODE_MASK, AW8695_SYSCTRL_PLAY_MODE_RAM);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BYPASS);
		if (err)
			return err;
		err = aw8695_haptic_set_active(haptics);
		if (err)
			return err;
		break;
	case AW8695_RTP_MODE:
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_PLAY_MODE_MASK, AW8695_SYSCTRL_PLAY_MODE_RTP);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BYPASS);
		if (err)
			return err;
		err = aw8695_haptic_set_active(haptics);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BOOST);
		if (err)
			return err;
		usleep_range(2000, 2500);
		break;
	case AW8695_CONT_MODE:
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_PLAY_MODE_MASK, AW8695_SYSCTRL_PLAY_MODE_CONT);
		if (err)
			return err;
		err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			AW8695_SYSCTRL_BST_MODE_MASK, AW8695_SYSCTRL_BST_MODE_BYPASS);
		if (err)
			return err;
		err = aw8695_haptic_set_active(haptics);
		if (err)
			return err;
		break;
	default:
		dev_err(dev, "Unhandled mode: %d\n", mode);
		return -EINVAL;
	}

	return err;
}

static int aw8695_haptics_play(struct input_dev *dev, void *data,
			       struct ff_effect *effect)
{
	struct aw8695_data *haptics = input_get_drvdata(dev);
	int level;

	level = effect->u.rumble.strong_magnitude;
	if (!level)
		level = effect->u.rumble.weak_magnitude;

	haptics->running = level;
	schedule_work(&haptics->play_work);

	return 0;
}

static void aw8695_hw_reset(struct aw8695_data *haptics);
static int aw8695_ram_init(struct aw8695_data *haptics);

static int aw8695_restore_config(struct aw8695_data *haptics)
{
	int err;

	err = regmap_write(haptics->regmap, AW8695_ID, AW8695_RESET);
	if (err)
		return err;
	usleep_range(1000, 1500);

	err = aw8695_interrupt_clear(haptics);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_DBGCTRL,
		AW8695_DBGCTRL_INT_MODE_MASK, AW8695_DBGCTRL_INT_MODE_EDGE);
	if (err)
		return err;

	err = aw8695_set_work_mode(haptics, AW8695_STANDBY_MODE);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_PWMDBG,
		AW8695_PWMDBG_PWM_MODE_MASK, AW8695_PWMDBG_PWM_24K);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8695_BSTDBG1, haptics->boost_debug[0]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BSTDBG2, haptics->boost_debug[1]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BSTDBG3, haptics->boost_debug[2]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_TSET, haptics->tset);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_R_SPARE, haptics->r_spare);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_ANADBG,
		AW8695_ANADBG_IOC_MASK, AW8695_ANADBG_IOC_4P65A);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_BSTCFG,
		AW8695_BSTCFG_PEAKCUR_MASK,
		haptics->chipid == AW8697_CHIPID ?
			AW8695_BSTCFG_PEAKCUR_3P5A : AW8695_BSTCFG_PEAKCUR_2A);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
		AW8695_DETCTRL_PROTECT_MASK, AW8695_DETCTRL_PROTECT_NO_ACTION);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_PWMPRC,
		AW8695_PWMPRC_PRC_ENABLE, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_PRLVL,
		AW8695_PRLVL_PR_ENABLE, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_BST_AUTO,
		AW8695_BST_AUTO_BST_AUTOSW_MASK, AW8695_BST_AUTO_BST_MANUAL_BOOST);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_ADCTEST,
		AW8695_ADCTEST_VBAT_MODE_MASK, AW8695_ADCTEST_VBAT_HW_COMP);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_TRIM_LRA, haptics->f0_cali_lra);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHH_H,
			FIELD_GET(AW8695_HIGH_MASK, haptics->bemf_vthh));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHH_L,
			FIELD_GET(AW8695_LOW_MASK, haptics->bemf_vthh));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHL_H,
			FIELD_GET(AW8695_HIGH_MASK, haptics->bemf_vthl));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHL_L,
			FIELD_GET(AW8695_LOW_MASK, haptics->bemf_vthl));
	if (err)
		return err;

	return aw8695_ram_init(haptics);
}

static int aw8695_haptics_stop(struct aw8695_data *haptics)
{
	int err;
	unsigned int read_buf = 0;
	struct device *dev = &haptics->client->dev;

	err = regmap_read(haptics->regmap, AW8695_GLB_STATE, &read_buf);
	if (err)
		return err;

	if ((read_buf & 0x0f) != 0) {
		err = regmap_update_bits(haptics->regmap, AW8695_GO,
			AW8695_GO_ENABLE, 0);
		if (err)
			return err;

		err = regmap_read_poll_timeout(haptics->regmap, AW8695_GLB_STATE, read_buf,
				(read_buf & 0x0f) == 0, 500, 2000 * 100);
		if (err) {
			dev_err(dev, "Did not enter standby: %d, resetting controller\n", err);
			aw8695_hw_reset(haptics);
			return aw8695_restore_config(haptics);
		}
	}

	return aw8695_set_work_mode(haptics, AW8695_STANDBY_MODE);
}

static int aw8695_haptics_start(struct aw8695_data *haptics)
{
	int err;

	err = aw8695_haptics_stop(haptics);
	if (err)
		return err;

	/* Configure for waveform #1 to be played infinitely */
	err = regmap_write(haptics->regmap, AW8695_WAVSEQ1, 0x1);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_WAVLOOP1,
		AW8695_WAVLOOP1_SEQ1_MASK, 0xf0);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8695_WAVSEQ2, 0x0);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_WAVLOOP1,
		AW8695_WAVLOOP1_SEQ2_MASK, 0x0);
	if (err)
		return err;

	/* Configure for RAM loop mode */
	err = aw8695_set_work_mode(haptics, AW8695_RAM_LOOP_MODE);
	if (err)
		return err;

	/* Start vibration */
	return regmap_update_bits(haptics->regmap, AW8695_GO,
		AW8695_GO_ENABLE, AW8695_GO_ENABLE);
}

static void aw8695_close(struct input_dev *input)
{
	struct aw8695_data *haptics = input_get_drvdata(input);
	struct device *dev = &haptics->client->dev;
	int err = 0;

	cancel_work_sync(&haptics->play_work);
	mutex_lock(&haptics->lock);
	if (!haptics->haptic_rtp_mode)
		err = aw8695_haptics_stop(haptics);
	mutex_unlock(&haptics->lock);
	if (err)
		dev_err(dev, "Failed to stop haptics: %d\n", err);
}

static void aw8695_haptics_play_work(struct work_struct *work)
{
	struct aw8695_data *haptics =
		container_of(work, struct aw8695_data, play_work);
	struct device *dev = &haptics->client->dev;
	int err = 0;

	mutex_lock(&haptics->lock);
	if (!haptics->haptic_rtp_mode) {
		if (haptics->running)
			err = aw8695_haptics_start(haptics);
		else
			err = aw8695_haptics_stop(haptics);
	}
	mutex_unlock(&haptics->lock);

	if (err)
		dev_err(dev, "Failed to execute work command: %d\n", err);
}

static void aw8695_hw_reset(struct aw8695_data *haptics)
{
	/* Pull reset low */
	gpiod_set_value_cansleep(haptics->reset_gpio, 0);

	/* Wait ~1ms */
	usleep_range(1000, 2000);

	/* Pull reset high */
	gpiod_set_value_cansleep(haptics->reset_gpio, 1);

	/* Wait ~3.5ms until I2C is accessible */
	usleep_range(3500, 4000);
}

static int aw8695_haptic_offset_calibration(struct aw8695_data *haptics)
{
	unsigned int read_buf;
	int err;

	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, AW8695_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
		AW8695_DETCTRL_DIAG_GO_ENABLE, AW8695_DETCTRL_DIAG_GO_ENABLE);
	if (err)
		return err;

	err = regmap_read_poll_timeout(haptics->regmap, AW8695_DETCTRL, read_buf,
			(read_buf & AW8695_DETCTRL_DIAG_GO_ENABLE) == 0, 10000, 10000 * 50);
	if (err)
		return err;

	return regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, 0);
}

static int aw8695_haptic_read_f0(struct aw8695_data *haptics)
{
	struct device *dev = &haptics->client->dev;
	unsigned long f0;
	unsigned int f0_reg;
	unsigned int read_buf;
	int err;

	err = regmap_read(haptics->regmap, AW8695_F_LRA_F0_H, &read_buf);
	if (err)
		return err;
	f0_reg = FIELD_PREP(AW8695_HIGH_MASK, read_buf);

	err = regmap_read(haptics->regmap, AW8695_F_LRA_F0_L, &read_buf);
	if (err)
		return err;
	f0_reg |= FIELD_PREP(AW8695_LOW_MASK, read_buf);

	if (!f0_reg) {
		dev_err(dev, "Failed to read f0 value!\n");
		return -EINVAL;
	}

	f0 = 1000000000 / (f0_reg * haptics->f0_coefficient);
	dev_dbg(dev, "Read new f0: %d\n", (int)f0);

	return (int)f0;
}

static int aw8695_haptic_get_f0(struct aw8695_data *haptics)
{
	struct device *dev = &haptics->client->dev;
	unsigned int read_buf;
	unsigned int f0_trace_ms;
	unsigned int f0_reg;
	int f0;
	int err;

	err = aw8695_haptics_stop(haptics);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8695_TRIM_LRA, 0x00);
	if (err)
		return err;

	err = aw8695_set_work_mode(haptics, AW8695_CONT_MODE);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_CONT_CTRL,
		AW8695_CONT_CTRL_EN_CLOSE_MASK, AW8695_CONT_CTRL_OPEN_PLAYBACK);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_CONT_CTRL,
		AW8695_CONT_CTRL_F0_DETECT_ENABLE, AW8695_CONT_CTRL_F0_DETECT_ENABLE);
	if (err)
		return err;

	/* LPF */
	err = regmap_update_bits(haptics->regmap, AW8695_DATCTRL,
		AW8695_DATCTRL_FC_MASK, AW8695_DATCTRL_FC_1000HZ);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_DATCTRL,
		AW8695_DATCTRL_LPF_ENABLE_MASK, AW8695_DATCTRL_LPF_ENABLE);
	if (err)
		return err;

	/* LRA OSC Source */
	err = regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
		AW8695_ANACTRL_LRA_SRC_MASK, AW8695_ANACTRL_LRA_SRC_REG);
	if (err)
		return err;

	/* preset f0 */
	f0_reg = 1000000000 / (haptics->f0_preset * haptics->f0_coefficient);
	err = regmap_write(haptics->regmap, AW8695_F_PRE_H,
			FIELD_GET(AW8695_HIGH_MASK, f0_reg));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_F_PRE_L,
			FIELD_GET(AW8695_LOW_MASK, f0_reg));
	if (err)
		return err;

	/* f0 driver level */
	err = regmap_write(haptics->regmap, AW8695_DRV_LVL, haptics->drive_level);
	if (err)
		return err;

	/* f0 trace parameter */
	err = regmap_write(haptics->regmap, AW8695_NUM_F0_1,
			FIELD_PREP(AW8695_NUM_F0_1_PRE_MASK, haptics->f0_det_play) |
			FIELD_PREP(AW8695_NUM_F0_1_WAIT_MASK, haptics->f0_det_wait));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_NUM_F0_2, haptics->f0_det_repeat);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_NUM_F0_3, haptics->f0_det_trace);
	if (err)
		return err;

	err = aw8695_interrupt_clear(haptics);
	if (err)
		return err;

	/* play go and start f0 calibration */
	err = regmap_update_bits(haptics->regmap, AW8695_GO,
		AW8695_GO_ENABLE, AW8695_GO_ENABLE);
	if (err)
		return err;

	/* f0 trace time */
	f0_trace_ms =
		(1000 * 10 / haptics->f0_preset) * (haptics->f0_det_play + haptics->f0_det_wait +
		(haptics->f0_det_trace + haptics->f0_det_wait) * (haptics->f0_det_repeat - 1));
	usleep_range(f0_trace_ms * 1000, f0_trace_ms * 1000 + 500);

	err = regmap_read_poll_timeout(haptics->regmap, AW8695_GLB_STATE, read_buf,
			(read_buf & 0x0f) == 0, 10000, 10000 * 50);
	if (err) {
		dev_err(dev, "Did not enter standby: %d\n", err);
		return err;
	}

	f0 = aw8695_haptic_read_f0(haptics);

	/* restore default config */
	err = regmap_update_bits(haptics->regmap, AW8695_DATCTRL,
		AW8695_DATCTRL_LPF_ENABLE_MASK, AW8695_DATCTRL_LPF_DISABLE);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_CONT_CTRL,
		AW8695_CONT_CTRL_EN_CLOSE_MASK, AW8695_CONT_CTRL_CLOSE_PLAYBACK);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_CONT_CTRL,
		AW8695_CONT_CTRL_F0_DETECT_ENABLE, 0);
	if (err)
		return err;

	return f0;
}


static int aw8695_haptic_f0_calibration(struct aw8695_data *haptics)
{
	struct device *dev = &haptics->client->dev;
	unsigned int read_buf;
	int f0_cali_step, f0_limit, f0;
	char f0_cali_lra;
	int err;


	f0 = aw8695_haptic_get_f0(haptics);
	if (f0 < 0) {
		dev_err(dev, "Failed to read f0: %d\n", f0);
		return f0;
	}

	/* make sure the f0_limit is not more than f0_cali_percent % away from read f0 */
	if (f0 * 100 < haptics->f0_preset * (100 - haptics->f0_cali_percent) ||
	    f0 * 100 > haptics->f0_preset * (100 + haptics->f0_cali_percent)) {
		f0_limit = (int)haptics->f0_preset;
	} else {
		f0_limit = (int)f0;
	}

	/* calculate cali step */
	f0_cali_step = 100000 * (f0_limit - (int)haptics->f0_preset) /
		(f0_limit * 25);

	if (f0_cali_step >= 0) {
		if (f0_cali_step % 10 >= 5)
			f0_cali_step = f0_cali_step / 10 + 1 + 32;
		else
			f0_cali_step = f0_cali_step / 10 + 32;
	} else {
		if (f0_cali_step % 10 <= -5)
			f0_cali_step = 32 + (f0_cali_step / 10 - 1);
		else
			f0_cali_step = 32 + f0_cali_step / 10;
	}

	if (f0_cali_step > 31)
		f0_cali_lra = (char)f0_cali_step - 32;
	else
		f0_cali_lra = (char)f0_cali_step + 32;

	err = regmap_write(haptics->regmap, AW8695_TRIM_LRA, f0_cali_lra);
	if (err)
		return err;

	err = regmap_read(haptics->regmap, AW8695_TRIM_LRA, &read_buf);
	if (err)
		return err;

	haptics->f0 = f0_limit;
	haptics->f0_cali_lra = (u8)f0_cali_lra;
	dev_info(dev, "Calibrated f0=%u (%u.%u Hz), TRIM_LRA=0x%02x\n",
		 haptics->f0, haptics->f0 / 10, haptics->f0 % 10, read_buf);

	/* restore default work mode */
	err = aw8695_set_work_mode(haptics, AW8695_STANDBY_MODE);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_PLAY_MODE_MASK, AW8695_SYSCTRL_PLAY_MODE_RAM);
	if (err)
		return err;

	return aw8695_haptics_stop(haptics);
}

static int aw8695_init(struct aw8695_data *haptics)
{
	int err;
	unsigned int read_buf;
	struct device *dev = &haptics->client->dev;

	aw8695_hw_reset(haptics);

	err = regmap_read(haptics->regmap, AW8695_ID, &read_buf);
	if (err) {
		dev_err(dev, "Failed to read ID register: %d\n", err);
		return err;
	}

	switch (read_buf) {
	case AW8624_CHIPID:
	case AW8695_CHIPID:
	case AW8697_CHIPID:
		haptics->chipid = (u8)read_buf;
		break;
	default:
		dev_err(dev, "Chip ID mismatch: expected %x|%x|%x, got %x\n",
			AW8624_CHIPID, AW8695_CHIPID, AW8697_CHIPID, read_buf);
		return -ENODEV;
	}

	err = regmap_write(haptics->regmap, AW8695_ID, AW8695_RESET);
	if (err) {
		dev_err(dev, "Failed to reset: %d\n", err);
		return err;
	}

	/* Wait ~1ms after reset */
	usleep_range(1000, 1500);

	/* Clear UVLI bit by reading register */
	err = aw8695_interrupt_clear(haptics);
	if (err) {
		dev_err(dev, "Failed to clear interrupt: %d\n", err);
		return err;
	}

	/* Set interrupt mode to edge */
	err = regmap_update_bits(haptics->regmap, AW8695_DBGCTRL,
		AW8695_DBGCTRL_INT_MODE_MASK,
		AW8695_DBGCTRL_INT_MODE_EDGE);
	if (err) {
		dev_err(dev, "Failed to set interrupt mode: %d\n", err);
		return err;
	}

	/* Configure interrupts */
	err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_BSTERR_OFF, AW8695_SYSINTM_BSTERR_OFF);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_OV_OFF, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_UVLO_OFF, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_OCD_OFF, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
		AW8695_SYSINTM_OT_OFF, 0);
	if (err)
		return err;

	err = aw8695_set_work_mode(haptics, AW8695_STANDBY_MODE);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_PWMDBG,
		AW8695_PWMDBG_PWM_MODE_MASK, AW8695_PWMDBG_PWM_24K);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8695_BSTDBG1, haptics->boost_debug[0]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BSTDBG2, haptics->boost_debug[1]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BSTDBG3, haptics->boost_debug[2]);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_TSET, haptics->tset);
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_R_SPARE, haptics->r_spare);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_ANADBG,
		AW8695_ANADBG_IOC_MASK, AW8695_ANADBG_IOC_4P65A);
	if (err)
		return err;

	/* Set boost peak current (stock AW8697 uses 3.5A for sharp transient punch) */
	err = regmap_update_bits(haptics->regmap, AW8695_BSTCFG,
		AW8695_BSTCFG_PEAKCUR_MASK,
		haptics->chipid == AW8697_CHIPID ?
			AW8695_BSTCFG_PEAKCUR_3P5A : AW8695_BSTCFG_PEAKCUR_2A);
	if (err)
		return err;

	/* Adjust motorprotect config */
	err = regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
		AW8695_DETCTRL_PROTECT_MASK, AW8695_DETCTRL_PROTECT_NO_ACTION);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_PWMPRC,
		AW8695_PWMPRC_PRC_ENABLE, 0);
	if (err)
		return err;
	err = regmap_update_bits(haptics->regmap, AW8695_PRLVL,
		AW8695_PRLVL_PR_ENABLE, 0);
	if (err)
		return err;

	/* Adjust auto boost config */
	err = regmap_update_bits(haptics->regmap, AW8695_BST_AUTO,
		AW8695_BST_AUTO_BST_AUTOSW_MASK,
		AW8695_BST_AUTO_BST_MANUAL_BOOST);
	if (err)
		return err;

	err = aw8695_haptic_offset_calibration(haptics);
	if (err)
		return err;

	/* Set vbat compensation mode */
	err = regmap_update_bits(haptics->regmap, AW8695_ADCTEST,
		AW8695_ADCTEST_VBAT_MODE_MASK, AW8695_ADCTEST_VBAT_HW_COMP);
	if (err)
		return err;

	err = aw8695_haptic_f0_calibration(haptics);
	if (err)
		return err;

	/* beme config */
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHH_H,
			FIELD_GET(AW8695_HIGH_MASK, haptics->bemf_vthh));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHH_L,
			FIELD_GET(AW8695_LOW_MASK, haptics->bemf_vthh));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BEMF_VTHL_H,
			FIELD_GET(AW8695_HIGH_MASK, haptics->bemf_vthl));
	if (err)
		return err;
	return regmap_write(haptics->regmap, AW8695_BEMF_VTHL_L,
			FIELD_GET(AW8695_LOW_MASK, haptics->bemf_vthl));
}

static int aw8695_write_ram_data(struct aw8695_data *haptics,
				 const u8 *buf, size_t len)
{
	size_t off = 0;

	while (off < len) {
		size_t chunk = min_t(size_t, len - off, RICHTAP_RTP_MAX_CHUNK);
		int ret;

		haptics->rtp_tx_buf[0] = AW8695_RAMDATA;
		memcpy(&haptics->rtp_tx_buf[1], buf + off, chunk);
		ret = i2c_master_send(haptics->client, haptics->rtp_tx_buf, chunk + 1);
		if (ret < 0)
			return ret;
		off += chunk;
	}
	return 0;
}

static int aw8695_ram_load_container(struct aw8695_data *haptics,
				     const u8 *data, size_t len)
{
	struct device *dev = &haptics->client->dev;
	u16 expected_csum, calc_csum = 0;
	u16 base_addr, fifo_ae, fifo_af;
	size_t i;
	int err;

	if (len <= 4)
		return -EINVAL;

	expected_csum = ((u16)data[0] << 8) | data[1];
	for (i = 2; i < len; i++)
		calc_csum += data[i];
	if (calc_csum != expected_csum) {
		dev_err(dev, "SRAM firmware checksum mismatch: calc=0x%04x expected=0x%04x\n",
			calc_csum, expected_csum);
		return -EINVAL;
	}

	base_addr = ((u16)data[2] << 8) | data[3];
	if (base_addr < 0x400 || base_addr > 0x1000) {
		dev_err(dev, "Invalid SRAM firmware base_addr=0x%04x\n", base_addr);
		return -EINVAL;
	}

	fifo_ae = base_addr >> 1;
	fifo_af = base_addr - (base_addr >> 2);

	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, AW8695_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;

	err = regmap_write(haptics->regmap, AW8695_BASE_ADDRH,
		FIELD_GET(AW8695_HIGH_MASK, base_addr));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_BASE_ADDRL,
		FIELD_GET(AW8695_LOW_MASK, base_addr));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AEH,
		FIELD_GET(AW8695_HIGH_MASK, fifo_ae));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AEL,
		FIELD_GET(AW8695_LOW_MASK, fifo_ae));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AFH,
		FIELD_GET(AW8695_HIGH_MASK, fifo_af));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AFL,
		FIELD_GET(AW8695_LOW_MASK, fifo_af));
	if (err)
		goto out_disable;

	err = regmap_write(haptics->regmap, AW8695_RAMADDRH,
		FIELD_GET(AW8695_HIGH_MASK, base_addr));
	if (err)
		goto out_disable;
	err = regmap_write(haptics->regmap, AW8695_RAMADDRL,
		FIELD_GET(AW8695_LOW_MASK, base_addr));
	if (err)
		goto out_disable;

	err = aw8695_write_ram_data(haptics, data + 4, len - 4);
	if (err)
		goto out_disable;

	haptics->ram_base_addr = base_addr;
	haptics->ram_checksum = calc_csum;
	haptics->ram_fw_loaded = true;

out_disable:
	regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, 0);
	return err;
}

static int aw8695_ram_init(struct aw8695_data *haptics)
{
	unsigned char *ptr;
	int err;
	int i;

	if (haptics->ram_fw_data && haptics->ram_fw_len > 4) {
		err = aw8695_ram_load_container(haptics, haptics->ram_fw_data,
						haptics->ram_fw_len);
		if (!err)
			return 0;
	}

	/* Enable SRAM init */
	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, AW8695_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;

	/* Configure SRAM base address (2 KB RTP FIFO) and FIFO AE/AF thresholds */
	err = regmap_write(haptics->regmap, AW8695_BASE_ADDRH,
		FIELD_GET(AW8695_HIGH_MASK, AW8695_RAM_BASE_ADDR));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_BASE_ADDRL,
		FIELD_GET(AW8695_LOW_MASK, AW8695_RAM_BASE_ADDR));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AEH,
		FIELD_GET(AW8695_HIGH_MASK, AW8695_RAM_BASE_ADDR >> 1));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AEL,
		FIELD_GET(AW8695_LOW_MASK, AW8695_RAM_BASE_ADDR >> 1));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AFH,
		FIELD_GET(AW8695_HIGH_MASK, AW8695_RAM_BASE_ADDR - (AW8695_RAM_BASE_ADDR >> 2)));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_FIFO_AFL,
		FIELD_GET(AW8695_LOW_MASK, AW8695_RAM_BASE_ADDR - (AW8695_RAM_BASE_ADDR >> 2)));
	if (err)
		return err;

	/* Set RAMDATA write address */
	err = regmap_write(haptics->regmap, AW8695_RAMADDRH,
		FIELD_GET(AW8695_HIGH_MASK, AW8695_RAM_BASE_ADDR));
	if (err)
		return err;
	err = regmap_write(haptics->regmap, AW8695_RAMADDRL,
		FIELD_GET(AW8695_LOW_MASK, AW8695_RAM_BASE_ADDR));
	if (err)
		return err;

	/* Write waveform header */
	ptr = (unsigned char *) &sram_waveform_header;
	for (i = 0; i < sizeof(sram_waveform_header); i++) {
		err = regmap_write(haptics->regmap, AW8695_RAMDATA,
			ptr[i]);
		if (err)
			return err;
	}

	/* Write waveform data */
	for (i = 0; i < ARRAY_SIZE(aw8695_sine_waveform); i++) {
		err = regmap_write(haptics->regmap, AW8695_RAMDATA,
			aw8695_sine_waveform[i]);
		if (err)
			return err;
	}

	haptics->ram_base_addr = AW8695_RAM_BASE_ADDR;

	/* Disable SRAM init */
	return regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
		AW8695_SYSCTRL_RAMINIT_EN, 0);
}

static irqreturn_t aw8695_irq(int irq, void *data)
{
	struct aw8695_data *haptics = data;
	struct device *dev = &haptics->client->dev;
	unsigned int read_buf;
	int err;

	if (haptics->haptic_rtp_mode)
		return IRQ_HANDLED;

	err = regmap_read(haptics->regmap, AW8695_SYSINT, &read_buf);
	if (err) {
		dev_err(dev, "Failed to read SYSINT register: %d\n", err);
		return err;
	}
	dev_dbg(dev, "Interrupt: SYSINT=0x%x\n", read_buf);

	if (read_buf & AW8695_SYSINT_BSTERRI)
		dev_err(dev, "Received boost short circuit protection or over-voltage protection interrupt!\n");
	if (read_buf & AW8695_SYSINT_OVI)
		dev_err(dev, "Received wave data overflow or DPWM DC error interrupt!\n");
	if (read_buf & AW8695_SYSINT_UVLI)
		dev_err(dev, "Received under voltage lock out interrupt!\n");
	if (read_buf & AW8695_SYSINT_OCDI)
		dev_err(dev, "Received over current interrupt!\n");
	if (read_buf & AW8695_SYSINT_OTI)
		dev_err(dev, "Received over temperature interrupt!\n");

	if (read_buf & AW8695_SYSINT_DONEI)
		dev_dbg(dev, "Received playback done interrupt\n");
	/* FIFO mode is not (yet) implemented in this driver */
	if (read_buf & AW8695_SYSINT_FF_AEI)
		dev_dbg(dev, "Received FIFO almost empty interrupt\n");
	if (read_buf & AW8695_SYSINT_FF_AFI)
		dev_dbg(dev, "Received FIFO almost full interrupt\n");

	err = regmap_read(haptics->regmap, AW8695_DBGSTAT, &read_buf);
	if (err) {
		dev_err(dev, "Failed to read DBGSTAT register: %d\n", err);
		return err;
	}
	dev_dbg(dev, "Interrupt: DBGSTAT=0x%x\n", read_buf);

	err = regmap_read(haptics->regmap, AW8695_SYSST, &read_buf);
	if (err) {
		dev_err(dev, "Failed to read SYSST register: %d\n", err);
		return err;
	}
	dev_dbg(dev, "Interrupt: SYSST=0x%x\n", read_buf);

	return IRQ_HANDLED;
}

/* =========================================================================
 * AAC RichTap (/dev/awinic_haptic) RTP FIFO Streaming & IOCTL Interface
 * ========================================================================= */

static int aw8695_ram_update(struct aw8695_data *haptics);
static int aw8695_haptic_set_wav_seq(struct aw8695_data *haptics, u8 idx, u8 wav);
static int aw8695_haptic_set_wav_loop(struct aw8695_data *haptics, u8 idx, u8 loop);

static void aw8695_pm_qos_enable(struct aw8695_data *haptics, bool enable)
{
	mutex_lock(&haptics->qos_lock);
	if (haptics->pm_qos_added) {
		cpu_latency_qos_update_request(&haptics->pm_qos_req,
					       enable ? AW8695_PM_QOS_LATENCY_US
						      : PM_QOS_DEFAULT_VALUE);
	}
	mutex_unlock(&haptics->qos_lock);
}

static int aw8695_haptic_set_bst_vol(struct aw8695_data *haptics, u8 bst_vol)
{
	if (bst_vol & 0xe0)
		bst_vol = 0x1f;
	return regmap_update_bits(haptics->regmap, AW8695_BSTDBG4,
				  AW8695_BSTDBG4_BSTVOL_MASK, bst_vol << 1);
}

static int aw8695_haptic_set_gain(struct aw8695_data *haptics, u8 gain)
{
	return regmap_write(haptics->regmap, AW8695_DATDBG, gain);
}

static int aw8695_haptic_set_rtp_aei(struct aw8695_data *haptics, bool enable)
{
	return regmap_update_bits(haptics->regmap, AW8695_SYSINTM,
				  AW8695_SYSINTM_FF_AE_OFF,
				  enable ? 0 : AW8695_SYSINTM_FF_AE_OFF);
}

static int aw8695_richtap_set_osc_clock(struct aw8695_data *haptics, bool rtp_osc)
{
	/*
	 * aw8695_haptic_get_f0() switches ANACTRL.LRA_SRC to REG for f0-calibrated
	 * RAM playback. For 24 kHz RichTap RTP waveforms (already f0-resampled by
	 * libaacvibrator.so), use the OSC-drift calibrated TRIM_LRA if available,
	 * otherwise fall back to the factory eFuse 24 MHz oscillator trim so
	 * waveforms stay dead-on the LRA's high-Q resonant frequency.
	 */
	if (rtp_osc) {
		if (haptics->osc_calibrated) {
			regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
					   AW8695_ANACTRL_LRA_SRC_MASK,
					   AW8695_ANACTRL_LRA_SRC_REG);
			return regmap_write(haptics->regmap, AW8695_TRIM_LRA,
					    haptics->osc_cali_lra);
		}
		return regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
					  AW8695_ANACTRL_LRA_SRC_MASK,
					  AW8695_ANACTRL_LRA_SRC_EFUSE);
	}
	regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
			   AW8695_ANACTRL_LRA_SRC_MASK,
			   AW8695_ANACTRL_LRA_SRC_REG);
	return regmap_write(haptics->regmap, AW8695_TRIM_LRA, haptics->f0_cali_lra);
}

static int aw8695_rtp_write(struct aw8695_data *haptics, const u8 *buf, size_t len)
{
	size_t off = 0;

	while (off < len) {
		size_t chunk = min_t(size_t, len - off, RICHTAP_RTP_MAX_CHUNK);
		int ret;

		haptics->rtp_tx_buf[0] = AW8695_RTP_DATA;
		memcpy(&haptics->rtp_tx_buf[1], buf + off, chunk);
		ret = i2c_master_send(haptics->client, haptics->rtp_tx_buf, chunk + 1);
		if (ret < 0)
			return ret;
		off += chunk;
	}
	return 0;
}

static int aw8695_rtp_write_fifo(struct aw8695_data *haptics, const u8 *buf, size_t len)
{
	size_t base_addr = haptics->ram_base_addr ? haptics->ram_base_addr
						  : AW8695_RAM_BASE_ADDR;
	size_t off = 0;
	ktime_t t_start;

	/* Payloads <= FIFO size fit directly in the AW8697 hardware RTP FIFO */
	if (len <= base_addr)
		return aw8695_rtp_write(haptics, buf, len);

	/* Larger payloads: prefill 75% then pace on FIFO almost-full status */
	off = min_t(size_t, len, base_addr - (base_addr >> 2));
	if (aw8695_rtp_write(haptics, buf, off) < 0)
		return -EIO;

	t_start = ktime_get();
	while (off < len) {
		unsigned int sysst = 0;
		size_t chunk;
		int ret;

		if (ktime_us_delta(ktime_get(), t_start) > 800000)
			return -ETIMEDOUT;
		ret = regmap_read(haptics->regmap, AW8695_SYSST, &sysst);
		if (ret)
			return ret;
		if (sysst & AW8695_SYSST_DONES)
			break;
		if (sysst & AW8695_SYSST_FF_AFS) {
			usleep_range(500, 800);
			continue;
		}
		chunk = min_t(size_t, len - off, RICHTAP_RTP_MAX_CHUNK);
		ret = aw8695_rtp_write(haptics, buf + off, chunk);
		if (ret)
			return ret;
		off += chunk;
		t_start = ktime_get();
	}
	return 0;
}

static int aw8695_osc_trim_calc(struct aw8695_data *haptics, u64 theory_us,
				u64 real_us, u8 *trim_out)
{
	struct device *dev = &haptics->client->dev;
	u64 diff;
	u32 real_code, lra_trim;

	if (!theory_us || theory_us == real_us) {
		*trim_out = 0;
		return 1;
	}

	diff = (real_us > theory_us) ? (real_us - theory_us)
				     : (theory_us - real_us);
	if (diff > div_u64(theory_us, 50)) {
		dev_warn(dev, "OSC drift out of range (>2%%): real=%llu us theory=%llu us\n",
			 real_us, theory_us);
		return -ERANGE;
	}
	if (diff < div_u64(theory_us * 10, 10000)) {
		dev_info(dev, "OSC within 0.1%% (%llu vs %llu us), no trim needed\n",
			 real_us, theory_us);
		*trim_out = 0;
		return 1;
	}

	real_code = (u32)div64_u64(diff * 4000ULL, theory_us);
	real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
	if (real_us > theory_us)
		real_code = 32 + real_code;
	else
		real_code = 32 - real_code;

	if (real_code > 31)
		lra_trim = real_code - 32;
	else
		lra_trim = real_code + 32;

	*trim_out = (u8)(lra_trim & 0x3f);
	dev_info(dev, "OSC cali: real=%llu us theory=%llu us trim=0x%02x\n",
		 real_us, theory_us, *trim_out);
	return 0;
}

static int aw8695_rtp_osc_calibration(struct aw8695_data *haptics, bool compute_trim)
{
	struct device *dev = &haptics->client->dev;
	const struct firmware *fw = NULL;
	u8 *fallback_buf = NULL;
	const u8 *data;
	size_t data_len, rtp_cnt = 0;
	size_t chunk_max;
	unsigned int pwmdbg = 0, sysst = 0, dbgstat = 0;
	u32 pwm_hz;
	u64 theory_us;
	ktime_t t_start = 0, t_end, t_loop_start;
	bool started_ts = false;
	int ret = 0;

	/* Prefer stock aw8697_rtp.bin if present; otherwise stream silent 24 kHz buffer */
	if (firmware_request_nowarn(&fw, "aw8697_rtp.bin", dev) == 0 && fw && fw->size > 0) {
		data = fw->data;
		data_len = fw->size;
	} else {
		if (fw) {
			release_firmware(fw);
			fw = NULL;
		}
		fallback_buf = kvzalloc(AW8695_OSC_CALI_FALLBACK_LEN, GFP_KERNEL);
		if (!fallback_buf)
			return -ENOMEM;
		data = fallback_buf;
		data_len = AW8695_OSC_CALI_FALLBACK_LEN;
	}

	mutex_lock(&haptics->ioctl_lock);
	atomic_set(&haptics->done_flag, 1);
	if (haptics->start_buf) {
		int i;

		for (i = 0; i < RICHTAP_MMAP_BUF_SUM; i++)
			haptics->start_buf[i].status = MMAP_BUF_DATA_FINISHED;
		smp_mb();
	}
	if (haptics->rtp_wq)
		cancel_work_sync(&haptics->haptic_rtp_work);

	mutex_lock(&haptics->lock);
	aw8695_haptic_set_rtp_aei(haptics, false);
	aw8695_haptics_stop(haptics);

	regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
			   AW8695_ANACTRL_LRA_SRC_MASK,
			   AW8695_ANACTRL_LRA_SRC_REG);
	if (compute_trim)
		regmap_write(haptics->regmap, AW8695_TRIM_LRA, 0x00);
	else
		regmap_write(haptics->regmap, AW8695_TRIM_LRA, haptics->osc_cali_lra);

	aw8695_haptic_set_gain(haptics, haptics->gain);
	haptics->haptic_rtp_mode = true;
	aw8695_set_work_mode(haptics, AW8695_RTP_MODE);
	regmap_update_bits(haptics->regmap, AW8695_DBGCTRL,
			   AW8695_DBGCTRL_INT_MODE_MASK,
			   AW8695_DBGCTRL_INT_MODE_EDGE);
	aw8695_interrupt_clear(haptics);
	regmap_update_bits(haptics->regmap, AW8695_GO,
			   AW8695_GO_ENABLE, AW8695_GO_ENABLE);

	chunk_max = haptics->ram_base_addr ? (haptics->ram_base_addr >> 2)
					   : (AW8695_RAM_BASE_ADDR >> 2);
	if (chunk_max > RICHTAP_RTP_MAX_CHUNK)
		chunk_max = RICHTAP_RTP_MAX_CHUNK;

	aw8695_pm_qos_enable(haptics, true);
	t_loop_start = ktime_get();

	while (1) {
		if (rtp_cnt < data_len) {
			ret = regmap_read(haptics->regmap, AW8695_SYSST, &sysst);
			if (ret)
				break;
			if (!(sysst & AW8695_SYSST_FF_AFS)) {
				size_t buf_len = min_t(size_t, data_len - rtp_cnt, chunk_max);

				if (!started_ts) {
					t_start = ktime_get();
					started_ts = true;
				}
				ret = aw8695_rtp_write(haptics, data + rtp_cnt, buf_len);
				if (ret)
					break;
				rtp_cnt += buf_len;
			}
		}

		ret = regmap_read(haptics->regmap, AW8695_DBGSTAT, &dbgstat);
		if (ret)
			break;
		if (started_ts && rtp_cnt >= data_len &&
		    (dbgstat & AW8695_DBGSTAT_FF_EMPTY)) {
			t_end = ktime_get();
			haptics->osc_microsecond = ktime_us_delta(t_end, t_start);
			break;
		}

		t_end = ktime_get();
		if (started_ts) {
			haptics->osc_microsecond = ktime_us_delta(t_end, t_start);
			if (haptics->osc_microsecond > 6000000) {
				dev_warn(dev, "OSC cali timed out at %zu/%zu bytes\n",
					 rtp_cnt, data_len);
				ret = -ETIMEDOUT;
				break;
			}
		} else if (ktime_us_delta(t_end, t_loop_start) > 500000) {
			dev_warn(dev, "OSC cali FIFO never ready\n");
			ret = -ETIMEDOUT;
			break;
		}
	}

	aw8695_pm_qos_enable(haptics, false);
	aw8695_haptics_stop(haptics);
	haptics->haptic_rtp_mode = false;

	if (!ret && started_ts && compute_trim) {
		u8 trim = 0;
		int calc_ret;

		regmap_read(haptics->regmap, AW8695_PWMDBG, &pwmdbg);
		switch ((pwmdbg & AW8695_PWMDBG_PWM_MODE_MASK) >> 5) {
		case 3:
			pwm_hz = 12000;
			break;
		case 2:
			pwm_hz = 24000;
			break;
		default:
			pwm_hz = 48000;
			break;
		}
		theory_us = div_u64((u64)data_len * 1000000ULL, pwm_hz);
		calc_ret = aw8695_osc_trim_calc(haptics, theory_us,
						(u64)haptics->osc_microsecond, &trim);
		if (calc_ret >= 0) {
			haptics->osc_cali_lra = trim;
			haptics->osc_calibrated = (calc_ret == 0);
		} else {
			ret = calc_ret;
		}
	}

	aw8695_richtap_set_osc_clock(haptics, false);
	mutex_unlock(&haptics->lock);
	mutex_unlock(&haptics->ioctl_lock);

	kvfree(fallback_buf);
	if (fw)
		release_firmware(fw);
	return ret;
}

static void aw8695_richtap_clean_buf(struct aw8695_data *haptics, u8 status)
{
	int i;

	if (!haptics->start_buf)
		return;
	/* Never follow user-writable kernel_next pointers; index the ring directly */
	for (i = 0; i < RICHTAP_MMAP_BUF_SUM; i++)
		haptics->start_buf[i].status = status;
	smp_mb();
}

static void aw8695_richtap_rtp_work(struct work_struct *work)
{
	struct aw8695_data *haptics =
		container_of(work, struct aw8695_data, haptic_rtp_work);
	struct device *dev = &haptics->client->dev;
	unsigned int idx = 0;
	struct mmap_buf_format *opbuf = &haptics->start_buf[idx];
	unsigned int sysst = AW8695_SYSST_FF_AES;
	int wait_ms = 100;
	bool started = false;
	bool timed_out = false;
	ktime_t t_start;

	while (wait_ms-- > 0 && !atomic_read(&haptics->done_flag)) {
		smp_rmb();
		if (opbuf->status == MMAP_BUF_DATA_VALID) {
			aw8695_pm_qos_enable(haptics, true);
			mutex_lock(&haptics->lock);
			if (!atomic_read(&haptics->done_flag)) {
				/* Keep FF_AEI masked: this loop polls SYSST directly */
				aw8695_haptic_set_rtp_aei(haptics, false);
				aw8695_richtap_set_osc_clock(haptics, true);
				aw8695_set_work_mode(haptics, AW8695_RTP_MODE);
				aw8695_interrupt_clear(haptics);
				regmap_update_bits(haptics->regmap, AW8695_GO,
						   AW8695_GO_ENABLE, AW8695_GO_ENABLE);
				started = true;
			}
			mutex_unlock(&haptics->lock);
			break;
		}
		usleep_range(1000, 1200);
	}

	if (!started) {
		aw8695_pm_qos_enable(haptics, false);
		mutex_lock(&haptics->lock);
		haptics->haptic_rtp_mode = false;
		mutex_unlock(&haptics->lock);
		return;
	}

	t_start = ktime_get();
	while (!atomic_read(&haptics->done_flag)) {
		if (ktime_us_delta(ktime_get(), t_start) > 800000) {
			dev_warn(dev, "RichTap RTP stream timeout\n");
			timed_out = true;
			break;
		}
		smp_rmb();
		if ((sysst & AW8695_SYSST_DONES) ||
		    opbuf->status == MMAP_BUF_DATA_FINISHED ||
		    opbuf->status == MMAP_BUF_DATA_INVALID) {
			break;
		}
		if (opbuf->status == MMAP_BUF_DATA_VALID &&
		    (sysst & AW8695_SYSST_FF_AES)) {
			int len = opbuf->length;

			if (len > 0) {
				if (len > RICHTAP_MMAP_BUF_SIZE)
					len = RICHTAP_MMAP_BUF_SIZE;
				mutex_lock(&haptics->lock);
				aw8695_rtp_write(haptics, opbuf->data, len);
				mutex_unlock(&haptics->lock);
				memset(opbuf->data, 0, len);
			}
			smp_wmb();
			opbuf->status = MMAP_BUF_DATA_INVALID;
			smp_mb();
			idx = (idx + 1) % RICHTAP_MMAP_BUF_SUM;
			opbuf = &haptics->start_buf[idx];
			t_start = ktime_get();
		} else {
			usleep_range(800, 1200);
		}
		if (regmap_read(haptics->regmap, AW8695_SYSST, &sysst)) {
			timed_out = true;
			break;
		}
	}

	aw8695_pm_qos_enable(haptics, false);

	mutex_lock(&haptics->lock);
	aw8695_haptic_set_rtp_aei(haptics, false);
	if (timed_out || atomic_read(&haptics->done_flag)) {
		aw8695_haptics_stop(haptics);
		aw8695_richtap_set_osc_clock(haptics, false);
	}
	haptics->haptic_rtp_mode = false;
	mutex_unlock(&haptics->lock);
}

static int aw8695_richtap_open(struct inode *inode, struct file *file)
{
	struct aw8695_data *haptics =
		container_of(file->private_data, struct aw8695_data, miscdev);

	file->private_data = haptics;
	return 0;
}

static int aw8695_richtap_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long aw8695_richtap_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct aw8695_data *haptics = file->private_data;
	struct device *dev = &haptics->client->dev;
	u32 tmp;
	int ret = 0;

	switch (cmd) {
	case RICHTAP_GET_HWINFO:
		tmp = RICHTAP_HAPTIC_HV;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(tmp)))
			return -EFAULT;
		return 0;

	case RICHTAP_SET_FREQ:
		return -EOPNOTSUPP;

	case RICHTAP_GET_F0:
		tmp = haptics->f0 ? haptics->f0 : haptics->f0_preset;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(tmp)))
			return -EFAULT;
		return 0;

	case RICHTAP_OFF_MODE:
		return 0;

	case RICHTAP_SETTING_GAIN:
		if (arg > 0x80)
			arg = 0x80;
		mutex_lock(&haptics->lock);
		haptics->gain = (u8)arg;
		ret = regmap_write(haptics->regmap, AW8695_DATDBG, (u8)arg);
		mutex_unlock(&haptics->lock);
		return ret;

	case RICHTAP_UPDATE_RAM:
		return aw8695_ram_update(haptics);

	case RICHTAP_RTP_MODE:
		mutex_lock(&haptics->ioctl_lock);
		atomic_set(&haptics->done_flag, 1);
		aw8695_richtap_clean_buf(haptics, MMAP_BUF_DATA_FINISHED);
		cancel_work_sync(&haptics->haptic_rtp_work);

		/*
		 * Keep CPUs out of deep idle across the full ~6 ms I2C setup +
		 * RTP FIFO burst so cold-CPU C-state wakeups do not stretch
		 * single-byte I2C transactions from ~0.2 ms to ~1.0 ms.
		 */
		aw8695_pm_qos_enable(haptics, true);
		mutex_lock(&haptics->lock);
		aw8695_haptic_set_rtp_aei(haptics, false);
		aw8695_haptics_stop(haptics);
		if (copy_from_user(haptics->rtp_ptr, (void __user *)arg,
				   RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM)) {
			ret = -EFAULT;
			goto out_rtp;
		}
		tmp = *(u32 *)haptics->rtp_ptr;
		if (tmp > (RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM - 4)) {
			dev_err(dev, "RichTap RTP length invalid: %u\n", tmp);
			ret = -EINVAL;
			goto out_rtp;
		}
		haptics->haptic_rtp_mode = true;
		aw8695_haptic_set_bst_vol(haptics, 0x11);
		aw8695_richtap_set_osc_clock(haptics, true);
		aw8695_set_work_mode(haptics, AW8695_RTP_MODE);
		regmap_update_bits(haptics->regmap, AW8695_GO,
				   AW8695_GO_ENABLE, AW8695_GO_ENABLE);
		usleep_range(500, 800);
		ret = aw8695_rtp_write_fifo(haptics, &haptics->rtp_ptr[4], tmp);
		haptics->haptic_rtp_mode = false;
out_rtp:
		mutex_unlock(&haptics->lock);
		aw8695_pm_qos_enable(haptics, false);
		mutex_unlock(&haptics->ioctl_lock);
		return ret;

	case RICHTAP_STREAM_MODE:
		mutex_lock(&haptics->ioctl_lock);
		atomic_set(&haptics->done_flag, 1);
		aw8695_richtap_clean_buf(haptics, MMAP_BUF_DATA_FINISHED);
		cancel_work_sync(&haptics->haptic_rtp_work);

		mutex_lock(&haptics->lock);
		aw8695_richtap_clean_buf(haptics, MMAP_BUF_DATA_INVALID);
		aw8695_haptics_stop(haptics);
		atomic_set(&haptics->done_flag, 0);
		haptics->haptic_rtp_mode = true;
		aw8695_haptic_set_bst_vol(haptics, 0x11);
		queue_work(haptics->rtp_wq, &haptics->haptic_rtp_work);
		mutex_unlock(&haptics->lock);
		mutex_unlock(&haptics->ioctl_lock);
		return 0;

	case RICHTAP_STOP_MODE:
		mutex_lock(&haptics->ioctl_lock);
		atomic_set(&haptics->done_flag, 1);
		aw8695_richtap_clean_buf(haptics, MMAP_BUF_DATA_FINISHED);
		cancel_work_sync(&haptics->haptic_rtp_work);

		mutex_lock(&haptics->lock);
		aw8695_haptic_set_rtp_aei(haptics, false);
		aw8695_haptics_stop(haptics);
		aw8695_richtap_set_osc_clock(haptics, false);
		haptics->haptic_rtp_mode = false;
		mutex_unlock(&haptics->lock);
		mutex_unlock(&haptics->ioctl_lock);
		return 0;

	case RICHTAP_PLAY_SRAM: {
		u8 wav = arg & 0x7f;
		u8 gain = (arg >> 8) & 0xff;
		u8 bst_vol = (arg >> 16) & 0x1f;

		if (!wav)
			return -EINVAL;
		if (!gain || gain > AW8695_HAPTIC_DEFAULT_GAIN)
			gain = AW8695_HAPTIC_DEFAULT_GAIN;
		if (!bst_vol || bst_vol > AW8695_HAPTIC_HIGH_VMAX_REG)
			bst_vol = AW8695_HAPTIC_HIGH_VMAX_REG;

		mutex_lock(&haptics->ioctl_lock);
		atomic_set(&haptics->done_flag, 1);
		aw8695_richtap_clean_buf(haptics, MMAP_BUF_DATA_FINISHED);
		cancel_work_sync(&haptics->haptic_rtp_work);

		aw8695_pm_qos_enable(haptics, true);
		mutex_lock(&haptics->lock);
		haptics->haptic_rtp_mode = false;
		aw8695_haptic_set_rtp_aei(haptics, false);
		aw8695_haptics_stop(haptics);

		haptics->gain = gain;
		haptics->vmax = bst_vol;
		ret = aw8695_haptic_set_wav_seq(haptics, 0, wav);
		if (!ret)
			ret = aw8695_haptic_set_wav_seq(haptics, 1, 0);
		if (!ret)
			ret = aw8695_haptic_set_wav_loop(haptics, 0, 0);
		if (!ret)
			ret = aw8695_richtap_set_osc_clock(haptics, false);
		if (!ret)
			ret = aw8695_haptic_set_gain(haptics, gain);
		if (!ret)
			ret = aw8695_haptic_set_bst_vol(haptics, bst_vol);
		if (!ret)
			ret = aw8695_set_work_mode(haptics, AW8695_RAM_MODE);
		if (!ret)
			ret = regmap_update_bits(haptics->regmap, AW8695_GO,
						 AW8695_GO_ENABLE,
						 AW8695_GO_ENABLE);
		mutex_unlock(&haptics->lock);
		aw8695_pm_qos_enable(haptics, false);
		mutex_unlock(&haptics->ioctl_lock);
		return ret;
	}

	default:
		dev_dbg(dev, "RichTap unsupported ioctl: 0x%x\n", cmd);
		return -ENOTTY;
	}
}

static int aw8695_richtap_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct aw8695_data *haptics = filp->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long phys;

	if (vma->vm_pgoff != 0 || size != (PAGE_SIZE << RICHTAP_MMAP_PAGE_ORDER))
		return -EINVAL;
	if (!(vma->vm_flags & VM_SHARED))
		return -EPERM;

	vm_flags_set(vma, VM_DONTDUMP | VM_DONTEXPAND);
	phys = virt_to_phys(haptics->start_buf);
	return remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
			       size, vma->vm_page_prot);
}

static const struct file_operations aw8695_richtap_fops = {
	.owner = THIS_MODULE,
	.open = aw8695_richtap_open,
	.release = aw8695_richtap_release,
	.unlocked_ioctl = aw8695_richtap_ioctl,
	.compat_ioctl = aw8695_richtap_ioctl,
	.mmap = aw8695_richtap_mmap,
};

static void aw8695_richtap_cleanup(void *data)
{
	struct aw8695_data *haptics = data;
	int i;

	if (haptics->miscdev.minor != MISC_DYNAMIC_MINOR && haptics->miscdev.this_device)
		misc_deregister(&haptics->miscdev);
	atomic_set(&haptics->done_flag, 1);
	if (haptics->rtp_wq) {
		cancel_work_sync(&haptics->haptic_rtp_work);
		destroy_workqueue(haptics->rtp_wq);
	}
	mutex_lock(&haptics->qos_lock);
	if (haptics->pm_qos_added) {
		cpu_latency_qos_remove_request(&haptics->pm_qos_req);
		haptics->pm_qos_added = false;
	}
	mutex_unlock(&haptics->qos_lock);
	if (haptics->start_buf) {
		struct page *base = virt_to_page(haptics->start_buf);

		for (i = 0; i < (1 << RICHTAP_MMAP_PAGE_ORDER); i++)
			ClearPageReserved(base + i);
		free_pages((unsigned long)haptics->start_buf, RICHTAP_MMAP_PAGE_ORDER);
	}
	kfree(haptics->rtp_ptr);
}

static int aw8695_richtap_init(struct aw8695_data *haptics)
{
	struct device *dev = &haptics->client->dev;
	struct mmap_buf_format *temp;
	struct page *base;
	int i, err;

	haptics->rtp_ptr = kzalloc(RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM,
				   GFP_KERNEL);
	if (!haptics->rtp_ptr)
		return -ENOMEM;

	haptics->start_buf = (struct mmap_buf_format *)__get_free_pages(
		GFP_KERNEL | __GFP_ZERO, RICHTAP_MMAP_PAGE_ORDER);
	if (!haptics->start_buf) {
		kfree(haptics->rtp_ptr);
		haptics->rtp_ptr = NULL;
		return -ENOMEM;
	}

	base = virt_to_page(haptics->start_buf);
	for (i = 0; i < (1 << RICHTAP_MMAP_PAGE_ORDER); i++)
		SetPageReserved(base + i);

	temp = haptics->start_buf;
	for (i = 1; i < RICHTAP_MMAP_BUF_SUM; i++) {
		temp->kernel_next = haptics->start_buf + i;
		temp = temp->kernel_next;
	}
	temp->kernel_next = haptics->start_buf;

	haptics->rtp_wq = alloc_workqueue("aw8695_rtp",
					  WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!haptics->rtp_wq) {
		aw8695_richtap_cleanup(haptics);
		return -ENOMEM;
	}

	cpu_latency_qos_add_request(&haptics->pm_qos_req, PM_QOS_DEFAULT_VALUE);
	haptics->pm_qos_added = true;

	INIT_WORK(&haptics->haptic_rtp_work, aw8695_richtap_rtp_work);
	atomic_set(&haptics->done_flag, 1);
	haptics->haptic_rtp_mode = false;

	haptics->miscdev.minor = MISC_DYNAMIC_MINOR;
	haptics->miscdev.name = "awinic_haptic";
	haptics->miscdev.fops = &aw8695_richtap_fops;

	err = misc_register(&haptics->miscdev);
	if (err) {
		dev_err(dev, "Failed to register /dev/awinic_haptic: %d\n", err);
		aw8695_richtap_cleanup(haptics);
		return err;
	}

	err = devm_add_action_or_reset(dev, aw8695_richtap_cleanup, haptics);
	if (err)
		return err;

	dev_info(dev, "Registered RichTap /dev/awinic_haptic (f0=%u)\n",
		 haptics->f0);
	return 0;
}

/* =========================================================================
 * AW8697 SRAM Waveform Firmware Loader, Legacy /sys/class/leds/vibrator
 * and /proc/vibrator/touch_style
 * ========================================================================= */

static const char *aw8695_select_ram_fw_name(struct aw8695_data *haptics)
{
	u32 f0_hz = (haptics->f0 ? haptics->f0 : haptics->f0_preset) / 10;

	if (f0_hz < 167)
		return "aw8697_haptic_166.bin";
	if (f0_hz < 169)
		return "aw8697_haptic_168.bin";
	if (f0_hz < 171)
		return "aw8697_haptic_170.bin";
	if (f0_hz < 173)
		return "aw8697_haptic_172.bin";
	return "aw8697_haptic_174.bin";
}

static void aw8695_ram_loaded(const struct firmware *fw, void *context)
{
	struct aw8695_data *haptics = context;
	struct device *dev = &haptics->client->dev;
	size_t fw_len;
	u8 *copy;
	int err;

	if (!fw) {
		dev_dbg(dev, "No AW8697 SRAM firmware found, keeping built-in waveform\n");
		return;
	}

	fw_len = fw->size;
	if (fw_len <= 4) {
		dev_err(dev, "AW8697 SRAM firmware too small (%zu bytes)\n", fw_len);
		release_firmware(fw);
		return;
	}

	copy = kvmalloc(fw_len, GFP_KERNEL);
	if (!copy) {
		release_firmware(fw);
		return;
	}
	memcpy(copy, fw->data, fw_len);
	release_firmware(fw);

	mutex_lock(&haptics->lock);
	if (haptics->haptic_rtp_mode) {
		mutex_unlock(&haptics->lock);
		kvfree(copy);
		dev_info(dev, "Deferring SRAM firmware load while RTP is active\n");
		return;
	}

	aw8695_haptics_stop(haptics);
	err = aw8695_ram_load_container(haptics, copy, fw_len);
	if (!err) {
		kvfree(haptics->ram_fw_data);
		haptics->ram_fw_data = copy;
		haptics->ram_fw_len = fw_len;
		dev_info(dev, "Loaded AW8697 SRAM firmware (%zu bytes, base=0x%04x, csum=0x%04x)\n",
			 haptics->ram_fw_len, haptics->ram_base_addr,
			 haptics->ram_checksum);
	} else {
		kvfree(copy);
		dev_err(dev, "Failed to load AW8697 SRAM firmware: %d\n", err);
	}
	mutex_unlock(&haptics->lock);
}

static int aw8695_ram_update(struct aw8695_data *haptics)
{
	const char *fw_name = aw8695_select_ram_fw_name(haptics);
	struct device *dev = &haptics->client->dev;
	int err;

	err = request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT, fw_name,
				      dev, GFP_KERNEL, haptics,
				      aw8695_ram_loaded);
	if (err)
		dev_warn(dev, "request_firmware_nowait(%s) failed: %d\n",
			 fw_name, err);
	return err;
}

static void aw8695_ram_work_routine(struct work_struct *work)
{
	struct aw8695_data *haptics =
		container_of(work, struct aw8695_data, ram_work.work);

	aw8695_ram_update(haptics);
}

struct aw8695_vmax_map {
	u32 level;
	u8 vmax;
	u8 gain;
};

static const struct aw8695_vmax_map aw8695_vmax_table[] = {
	{ 800,  0x00, 0x30 }, { 900,  0x00, 0x36 }, { 1000, 0x00, 0x42 },
	{ 1100, 0x00, 0x48 }, { 1200, 0x00, 0x54 }, { 1300, 0x00, 0x60 },
	{ 1400, 0x00, 0x64 }, { 1500, 0x00, 0x70 }, { 1600, 0x00, 0x75 },
	{ 1700, 0x02, 0x75 }, { 1800, 0x04, 0x75 }, { 1900, 0x06, 0x75 },
	{ 2000, 0x08, 0x75 }, { 2100, 0x10, 0x75 }, { 2200, 0x12, 0x75 },
	{ 2300, 0x14, 0x75 }, { 2400, 0x16, 0x75 },
};

static void aw8695_convert_level_to_vmax(struct aw8695_data *haptics, u32 val)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(aw8695_vmax_table); i++) {
		if (val <= aw8695_vmax_table[i].level) {
			haptics->vmax = aw8695_vmax_table[i].vmax;
			haptics->gain = aw8695_vmax_table[i].gain;
			return;
		}
	}
	haptics->vmax = AW8695_HAPTIC_HIGH_VMAX_REG;
	haptics->gain = 0x75;
}

static int aw8695_haptic_set_wav_seq(struct aw8695_data *haptics, u8 idx, u8 wav)
{
	if (idx >= AW8695_HAPTIC_SEQ_SIZE)
		return -EINVAL;
	haptics->seq[idx] = wav;
	return regmap_write(haptics->regmap, AW8695_WAVSEQ1 + idx, wav);
}

static int aw8695_haptic_set_wav_loop(struct aw8695_data *haptics, u8 idx, u8 loop)
{
	unsigned int reg, mask, val;

	if (idx >= AW8695_HAPTIC_LOOP_SIZE)
		return -EINVAL;
	haptics->loop[idx] = loop & 0x0f;
	reg = AW8695_WAVLOOP1 + (idx / 2);
	if ((idx % 2) == 0) {
		mask = AW8695_WAVLOOP1_SEQ1_MASK;
		val = (loop & 0x0f) << 4;
	} else {
		mask = AW8695_WAVLOOP1_SEQ2_MASK;
		val = loop & 0x0f;
	}
	return regmap_update_bits(haptics->regmap, reg, mask, val);
}

static int aw8695_haptic_get_vbat(struct aw8695_data *haptics)
{
	unsigned int detctrl = 0, vbat_reg = 0;
	u32 vbat_mv;
	int err;

	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
				 AW8695_SYSCTRL_RAMINIT_EN,
				 AW8695_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;

	err = regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
				 AW8695_DETCTRL_VBAT_GO_ENABLE,
				 AW8695_DETCTRL_VBAT_GO_ENABLE);
	if (err)
		goto out_disable;

	regmap_read_poll_timeout(haptics->regmap, AW8695_DETCTRL, detctrl,
				 (detctrl & AW8695_DETCTRL_VBAT_GO_ENABLE) == 0,
				 500, 10000);

	err = regmap_read(haptics->regmap, AW8695_VBATDET, &vbat_reg);
	if (!err) {
		vbat_mv = (6100 * vbat_reg) / 256;
		haptics->vbat = clamp_t(u32, vbat_mv, AW8695_VBAT_MIN_MV,
					AW8695_VBAT_MAX_MV);
	}

out_disable:
	regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			   AW8695_SYSCTRL_RAMINIT_EN, 0);
	return err;
}

static int aw8695_haptic_get_lra_resistance(struct aw8695_data *haptics)
{
	unsigned int rldet = 0;
	int err;

	aw8695_haptics_stop(haptics);
	err = regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
				 AW8695_SYSCTRL_RAMINIT_EN,
				 AW8695_SYSCTRL_RAMINIT_EN);
	if (err)
		return err;
	regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			   AW8695_SYSCTRL_BST_MODE_MASK,
			   AW8695_SYSCTRL_BST_MODE_BYPASS);
	regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
			   AW8695_ANACTRL_HD_PD_MASK, AW8695_ANACTRL_HD_HZ_EN);
	regmap_update_bits(haptics->regmap, AW8695_D2SCFG,
			   AW8695_D2SCFG_CLK_ADC_MASK,
			   AW8695_D2SCFG_CLK_ASC_1P5MHZ);
	regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
			   AW8695_DETCTRL_RL_OS_MASK, AW8695_DETCTRL_RL_DETECT);
	regmap_update_bits(haptics->regmap, AW8695_DETCTRL,
			   AW8695_DETCTRL_DIAG_GO_ENABLE,
			   AW8695_DETCTRL_DIAG_GO_ENABLE);
	usleep_range(3000, 3500);

	err = regmap_read(haptics->regmap, AW8695_RLDET, &rldet);
	if (!err)
		haptics->lra_resistance = 298 * rldet;

	regmap_update_bits(haptics->regmap, AW8695_ANACTRL,
			   AW8695_ANACTRL_HD_PD_MASK, AW8695_ANACTRL_HD_PD_EN);
	regmap_update_bits(haptics->regmap, AW8695_D2SCFG,
			   AW8695_D2SCFG_CLK_ADC_MASK,
			   AW8695_D2SCFG_CLK_ASC_6MHZ);
	regmap_update_bits(haptics->regmap, AW8695_SYSCTRL,
			   AW8695_SYSCTRL_RAMINIT_EN, 0);
	return err;
}

static enum hrtimer_restart aw8695_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8695_data *haptics =
		container_of(timer, struct aw8695_data, vibrator_timer);

	haptics->state = 0;
	schedule_work(&haptics->vibrator_work);
	return HRTIMER_NORESTART;
}

static void aw8695_vibrator_work_routine(struct work_struct *work)
{
	struct aw8695_data *haptics =
		container_of(work, struct aw8695_data, vibrator_work);
	u8 wav;

	mutex_lock(&haptics->lock);
	if (haptics->haptic_rtp_mode) {
		mutex_unlock(&haptics->lock);
		return;
	}

	aw8695_haptics_stop(haptics);
	if (haptics->state && haptics->duration > 0) {
		wav = haptics->seq[0] ? haptics->seq[0] : 1;
		aw8695_richtap_set_osc_clock(haptics, false);
		aw8695_haptic_set_gain(haptics, haptics->gain);
		aw8695_haptic_set_bst_vol(haptics, haptics->vmax);
		aw8695_haptic_set_wav_seq(haptics, 0, wav);
		aw8695_haptic_set_wav_seq(haptics, 1, 0);
		aw8695_haptic_set_wav_loop(haptics, 0, 0x0f);
		aw8695_set_work_mode(haptics, AW8695_RAM_LOOP_MODE);
		regmap_update_bits(haptics->regmap, AW8695_GO,
				   AW8695_GO_ENABLE, AW8695_GO_ENABLE);
		hrtimer_start(&haptics->vibrator_timer,
			      ms_to_ktime(haptics->duration),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&haptics->lock);
}

static int aw8695_haptic_brightness_set(struct led_classdev *cdev,
					enum led_brightness level)
{
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	int err = 0;

	if (level > 0)
		aw8695_pm_qos_enable(haptics, true);

	mutex_lock(&haptics->lock);
	if (haptics->haptic_rtp_mode) {
		mutex_unlock(&haptics->lock);
		if (level > 0)
			aw8695_pm_qos_enable(haptics, false);
		return 0;
	}

	aw8695_haptics_stop(haptics);
	if (level > 0) {
		if (!haptics->seq[0])
			aw8695_haptic_set_wav_seq(haptics, 0, 1);
		aw8695_richtap_set_osc_clock(haptics, false);
		aw8695_haptic_set_gain(haptics, haptics->gain);
		aw8695_haptic_set_bst_vol(haptics, haptics->vmax);
		err = aw8695_set_work_mode(haptics, AW8695_RAM_MODE);
		if (!err)
			err = regmap_update_bits(haptics->regmap, AW8695_GO,
						 AW8695_GO_ENABLE,
						 AW8695_GO_ENABLE);
	}
	mutex_unlock(&haptics->lock);

	if (level > 0)
		aw8695_pm_qos_enable(haptics, false);
	return err;
}

static ssize_t oplus_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u\n", haptics->state);
}

static ssize_t oplus_state_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	haptics->state = val ? 1 : 0;
	return count;
}

static ssize_t oplus_duration_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	ktime_t remaining;

	if (hrtimer_active(&haptics->vibrator_timer)) {
		remaining = hrtimer_get_remaining(&haptics->vibrator_timer);
		return sysfs_emit(buf, "%lld\n", ktime_to_ms(remaining));
	}
	return sysfs_emit(buf, "%u\n", haptics->duration);
}

static ssize_t oplus_duration_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	haptics->duration = min_t(u32, val, 10000);
	return count;
}

static ssize_t oplus_activate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u\n", haptics->state);
}

static ssize_t oplus_activate_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;
	if (val > 1)
		return count;

	if (haptics->haptic_rtp_mode)
		return count;

	hrtimer_cancel(&haptics->vibrator_timer);
	if (val == 0 && !haptics->state)
		return count;

	haptics->state = val;
	cancel_work_sync(&haptics->vibrator_work);
	schedule_work(&haptics->vibrator_work);
	return count;
}

static ssize_t vmax_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "0x%02x\n", haptics->vmax);
}

static ssize_t vmax_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&haptics->lock);
	if (val <= 255) {
		haptics->gain = (u8)((val * AW8695_HAPTIC_DEFAULT_GAIN) / 255);
	} else if (val <= 2400) {
		aw8695_convert_level_to_vmax(haptics, val);
	} else {
		haptics->vmax = AW8695_HAPTIC_HIGH_VMAX_REG;
		haptics->gain = AW8695_HAPTIC_DEFAULT_GAIN;
	}
	if (!haptics->haptic_rtp_mode) {
		aw8695_haptic_set_gain(haptics, haptics->gain);
		aw8695_haptic_set_bst_vol(haptics, haptics->vmax);
	}
	mutex_unlock(&haptics->lock);
	return count;
}

static ssize_t gain_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "0x%02x\n", haptics->gain);
}

static ssize_t gain_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	if (val > AW8695_HAPTIC_DEFAULT_GAIN)
		val = AW8695_HAPTIC_DEFAULT_GAIN;

	mutex_lock(&haptics->lock);
	haptics->gain = (u8)val;
	if (!haptics->haptic_rtp_mode)
		aw8695_haptic_set_gain(haptics, haptics->gain);
	mutex_unlock(&haptics->lock);
	return count;
}

static ssize_t waveform_index_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u\n", haptics->seq[0]);
}

static ssize_t waveform_index_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&haptics->lock);
	if (!haptics->haptic_rtp_mode) {
		aw8695_haptic_set_wav_seq(haptics, 0, (u8)val);
		aw8695_haptic_set_wav_seq(haptics, 1, 0);
		aw8695_haptic_set_wav_loop(haptics, 0, 0);
	} else {
		haptics->seq[0] = (u8)val;
		haptics->seq[1] = 0;
		haptics->loop[0] = 0;
	}
	if ((val == 0x10 || val == 0x11 || val == 0x12) && haptics->gain > 0x75)
		haptics->gain = 0x75;
	mutex_unlock(&haptics->lock);
	return count;
}

static ssize_t seq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	ssize_t len = 0;
	int i;

	for (i = 0; i < AW8695_HAPTIC_SEQ_SIZE; i++)
		len += sysfs_emit_at(buf, len, "seq%d: 0x%02x\n", i + 1,
				     haptics->seq[i]);
	return len;
}

static ssize_t seq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	unsigned int idx, val;

	if (sscanf(buf, "%x %x", &idx, &val) == 2 &&
	    idx < AW8695_HAPTIC_SEQ_SIZE) {
		mutex_lock(&haptics->lock);
		if (!haptics->haptic_rtp_mode)
			aw8695_haptic_set_wav_seq(haptics, (u8)idx, (u8)val);
		else
			haptics->seq[idx] = (u8)val;
		mutex_unlock(&haptics->lock);
	}
	return count;
}

static ssize_t loop_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	ssize_t len = 0;
	int i;

	for (i = 0; i < AW8695_HAPTIC_LOOP_SIZE; i++)
		len += sysfs_emit_at(buf, len, "seq%d_loop: 0x%02x\n", i + 1,
				     haptics->loop[i]);
	return len;
}

static ssize_t loop_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	unsigned int idx, val;

	if (sscanf(buf, "%x %x", &idx, &val) == 2 &&
	    idx < AW8695_HAPTIC_LOOP_SIZE) {
		mutex_lock(&haptics->lock);
		if (!haptics->haptic_rtp_mode)
			aw8695_haptic_set_wav_loop(haptics, (u8)idx, (u8)val);
		else
			haptics->loop[idx] = (u8)(val & 0x0f);
		mutex_unlock(&haptics->lock);
	}
	return count;
}

static ssize_t f0_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u\n", haptics->f0);
}

static ssize_t cali_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u (trim=0x%02x)\n", haptics->f0,
			  haptics->f0_cali_lra);
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	if (val) {
		mutex_lock(&haptics->ioctl_lock);
		mutex_lock(&haptics->lock);
		if (!haptics->haptic_rtp_mode)
			aw8695_haptic_f0_calibration(haptics);
		mutex_unlock(&haptics->lock);
		mutex_unlock(&haptics->ioctl_lock);
	}
	return count;
}

static ssize_t ram_update_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "loaded=%d len=%zu base=0x%04x csum=0x%04x\n",
			  haptics->ram_fw_loaded, haptics->ram_fw_len,
			  haptics->ram_base_addr, haptics->ram_checksum);
}

static ssize_t ram_update_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	if (val)
		aw8695_ram_update(haptics);
	return count;
}

static ssize_t osc_cali_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%lld\n", haptics->osc_microsecond);
}

static ssize_t osc_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	if (val == 3)
		aw8695_rtp_osc_calibration(haptics, true);
	else if (val == 1)
		aw8695_rtp_osc_calibration(haptics, false);

	return count;
}

static ssize_t osc_data_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	return sysfs_emit(buf, "%u\n", haptics->osc_cali_lra);
}

static ssize_t osc_data_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);
	u32 val;
	int err;

	err = kstrtou32(buf, 0, &val);
	if (err)
		return err;

	mutex_lock(&haptics->lock);
	if (val == 0) {
		haptics->osc_cali_lra = 0;
		haptics->osc_calibrated = false;
	} else {
		haptics->osc_cali_lra = (u8)(val & 0x3f);
		haptics->osc_calibrated = true;
	}
	mutex_unlock(&haptics->lock);
	return count;
}

static ssize_t vbat_monitor_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	mutex_lock(&haptics->lock);
	if (!haptics->haptic_rtp_mode)
		aw8695_haptic_get_vbat(haptics);
	mutex_unlock(&haptics->lock);

	return sysfs_emit(buf, "%u\n", haptics->vbat);
}

static ssize_t lra_resistance_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8695_data *haptics = container_of(cdev, struct aw8695_data, cdev);

	mutex_lock(&haptics->lock);
	if (!haptics->haptic_rtp_mode)
		aw8695_haptic_get_lra_resistance(haptics);
	mutex_unlock(&haptics->lock);

	return sysfs_emit(buf, "%u\n", haptics->lra_resistance);
}

static DEVICE_ATTR_RW(oplus_state);
static DEVICE_ATTR_RW(oplus_duration);
static DEVICE_ATTR_RW(oplus_activate);
static DEVICE_ATTR_RW(vmax);
static DEVICE_ATTR_RW(gain);
static DEVICE_ATTR_RW(waveform_index);
static DEVICE_ATTR_RW(seq);
static DEVICE_ATTR_RW(loop);
static DEVICE_ATTR_RO(f0);
static DEVICE_ATTR_RW(cali);
static DEVICE_ATTR_RW(ram_update);
static DEVICE_ATTR_RW(osc_cali);
static DEVICE_ATTR_RW(osc_data);
static DEVICE_ATTR_RO(vbat_monitor);
static DEVICE_ATTR_RO(lra_resistance);

static struct attribute *aw8695_vibrator_attrs[] = {
	&dev_attr_oplus_state.attr,
	&dev_attr_oplus_duration.attr,
	&dev_attr_oplus_activate.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_waveform_index.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_osc_data.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	NULL,
};

ATTRIBUTE_GROUPS(aw8695_vibrator);

static ssize_t aw8695_touch_style_read(struct file *filp, char __user *buf,
				       size_t count, loff_t *ppos)
{
	struct aw8695_data *haptics = pde_data(file_inode(filp));
	char page[8];
	int len;

	len = scnprintf(page, sizeof(page), "%u\n", haptics->vibration_style);
	return simple_read_from_buffer(buf, count, ppos, page, len);
}

static ssize_t aw8695_touch_style_write(struct file *filp,
					const char __user *buf, size_t count,
					loff_t *ppos)
{
	struct aw8695_data *haptics = pde_data(file_inode(filp));
	char kbuf[8] = { 0 };
	int val, err;

	if (count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;

	err = kstrtoint(strim(kbuf), 0, &val);
	if (err)
		return err;

	if (val == AW8695_VIBRATION_STYLE_SOFT)
		haptics->vibration_style = AW8695_VIBRATION_STYLE_SOFT;
	else
		haptics->vibration_style = AW8695_VIBRATION_STYLE_CRISP;

	aw8695_ram_update(haptics);
	return count;
}

static const struct proc_ops aw8695_touch_style_ops = {
	.proc_read = aw8695_touch_style_read,
	.proc_write = aw8695_touch_style_write,
	.proc_lseek = default_llseek,
};

static void aw8695_vibrator_cleanup(void *data)
{
	struct aw8695_data *haptics = data;

	if (haptics->proc_touch_style)
		proc_remove(haptics->proc_touch_style);
	if (haptics->proc_dir)
		proc_remove(haptics->proc_dir);
	cancel_delayed_work_sync(&haptics->ram_work);
	hrtimer_cancel(&haptics->vibrator_timer);
	cancel_work_sync(&haptics->vibrator_work);
	kvfree(haptics->ram_fw_data);
}

static int aw8695_vibrator_init(struct aw8695_data *haptics)
{
	struct device *dev = &haptics->client->dev;
	int err;

	haptics->seq[0] = 1;
	haptics->gain = AW8695_HAPTIC_DEFAULT_GAIN;
	haptics->vmax = AW8695_HAPTIC_DEFAULT_VMAX;

	hrtimer_setup(&haptics->vibrator_timer, aw8695_vibrator_timer_func,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	INIT_WORK(&haptics->vibrator_work, aw8695_vibrator_work_routine);
	INIT_DELAYED_WORK(&haptics->ram_work, aw8695_ram_work_routine);

	err = devm_add_action_or_reset(dev, aw8695_vibrator_cleanup, haptics);
	if (err)
		return err;

	haptics->cdev.name = "vibrator";
	haptics->cdev.max_brightness = LED_FULL;
	haptics->cdev.brightness_set_blocking = aw8695_haptic_brightness_set;
	haptics->cdev.groups = aw8695_vibrator_groups;

	err = devm_led_classdev_register(dev, &haptics->cdev);
	if (err) {
		dev_warn(dev, "Failed to register /sys/class/leds/vibrator: %d\n", err);
		return err;
	}

	haptics->proc_dir = proc_mkdir("vibrator", NULL);
	if (haptics->proc_dir) {
		haptics->proc_touch_style = proc_create_data(
			"touch_style", 0664, haptics->proc_dir,
			&aw8695_touch_style_ops, haptics);
		if (!haptics->proc_touch_style)
			dev_warn(dev, "Failed to create /proc/vibrator/touch_style\n");
	} else {
		dev_warn(dev, "Failed to create /proc/vibrator\n");
	}

	schedule_delayed_work(&haptics->ram_work,
			      msecs_to_jiffies(AW8695_RAM_FW_DELAY_MS));
	return 0;
}

static const struct regmap_config aw8695_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AW8695_MAX_REG,
	.cache_type = REGCACHE_NONE,
};

static int aw8695_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw8695_data *haptics;
	int err;

	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	err = of_property_read_u32(dev->of_node, "awinic,f0-preset", &haptics->f0_preset);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-preset\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-coefficient",
				   &haptics->f0_coefficient);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-coefficient\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-calibration-percent",
				   &haptics->f0_cali_percent);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-coefficient\n");

	err = of_property_read_u32(dev->of_node, "awinic,drive-level", &haptics->drive_level);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,drive-level\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-detection-play-time",
				   &haptics->f0_det_play);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-detection-play-time\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-detection-wait-time",
				   &haptics->f0_det_wait);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-detection-wait-time\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-detection-repeat",
				   &haptics->f0_det_repeat);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-detection-repeat\n");

	err = of_property_read_u32(dev->of_node, "awinic,f0-detection-trace",
				   &haptics->f0_det_trace);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,f0-detection-trace\n");

	err = of_property_read_u8_array(dev->of_node, "awinic,boost-debug",
					haptics->boost_debug, ARRAY_SIZE(haptics->boost_debug));
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,boost-debug\n");

	err = of_property_read_u8(dev->of_node, "awinic,tset", &haptics->tset);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,tset\n");

	err = of_property_read_u8(dev->of_node, "awinic,r-spare", &haptics->r_spare);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,r-spare\n");

	err = of_property_read_u32(dev->of_node, "awinic,bemf-upper-threshold",
				   &haptics->bemf_vthh);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,bemf-upper-threshold\n");

	err = of_property_read_u32(dev->of_node, "awinic,bemf-lower-threshold",
				   &haptics->bemf_vthl);
	if (err)
		return dev_err_probe(dev, err, "Failed to read awinic,bemf-lower-threshold\n");

	haptics->client = client;
	i2c_set_clientdata(client, haptics);

	haptics->regmap = devm_regmap_init_i2c(client, &aw8695_regmap_config);
	if (IS_ERR(haptics->regmap))
		return dev_err_probe(dev, PTR_ERR(haptics->regmap),
				     "Failed to allocate register map\n");

	mutex_init(&haptics->lock);
	mutex_init(&haptics->ioctl_lock);
	mutex_init(&haptics->qos_lock);
	haptics->f0 = haptics->f0_preset;
	haptics->ram_base_addr = AW8695_RAM_BASE_ADDR;

	haptics->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(haptics->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(haptics->reset_gpio),
				     "Failed to get reset gpio\n");

	err = devm_request_threaded_irq(dev, client->irq, NULL, aw8695_irq,
		IRQF_ONESHOT, NULL, haptics);
	if (err)
		return dev_err_probe(dev, err, "Failed to request interrupt\n");

	err = aw8695_init(haptics);
	if (err)
		return dev_err_probe(dev, err, "Failed to init aw8695\n");

	err = aw8695_ram_init(haptics);
	if (err)
		return dev_err_probe(dev, err, "Failed to init aw8695 sram\n");

	err = aw8695_richtap_init(haptics);
	if (!err) {
		if (aw8695_vibrator_init(haptics))
			dev_warn(dev, "Legacy vibrator sysfs init failed, continuing with RichTap\n");
		return 0;
	}

	dev_warn(dev, "RichTap init failed (%d), falling back to input-ff\n", err);

	haptics->input_dev = devm_input_allocate_device(dev);
	if (!haptics->input_dev)
		return -ENOMEM;

	INIT_WORK(&haptics->play_work, aw8695_haptics_play_work);

	haptics->input_dev->name = "aw8695-haptics";
	haptics->input_dev->close = aw8695_close;

	input_set_drvdata(haptics->input_dev, haptics);
	input_set_capability(haptics->input_dev, EV_FF, FF_RUMBLE);

	err = input_ff_create_memless(haptics->input_dev, NULL,
				      aw8695_haptics_play);
	if (err)
		return dev_err_probe(dev, err, "Failed to create FF dev\n");

	err = input_register_device(haptics->input_dev);
	if (err)
		return dev_err_probe(dev, err, "Failed to register input device\n");

	return 0;
}

static const struct of_device_id aw8695_of_id[] = {
	{ .compatible = "awinic,aw8624", },
	{ .compatible = "awinic,aw8695", },
	{ .compatible = "awinic,aw8697", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, aw8695_of_id);

static struct i2c_driver aw8695_driver = {
	.driver = {
		.name = "aw8695-haptics",
		.of_match_table = aw8695_of_id,
	},
	.probe = aw8695_probe,
};

module_i2c_driver(aw8695_driver);

MODULE_AUTHOR("Luca Weiss <luca.weiss@fairphone.com>");
MODULE_DESCRIPTION("AW8695 LRA Haptic Driver");
MODULE_LICENSE("GPL");
