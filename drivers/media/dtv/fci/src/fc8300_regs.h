/*****************************************************************************
	Copyright(c) 2013 FCI Inc. All Rights Reserved

	File name : fc8300_regs.h

	Description : header of FC8300 register map

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#ifndef __FC8300_REGS_H__
#define __FC8300_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* #define BBM_I2C_SPI */
/* #define BBM_I2C_TSIF */
#define BBM_INT_LOW_ACTIVE
/* #define BBM_AUX_INT */
/* #define BBM_NULL_PID_FILTER */
/* #define BBM_FAIL_FRAME */
/* #define BBM_TS_204 */
/* #define BBM_2_DIVERSITY */
/* #define BBM_4_DIVERSITY */
/* #define BBM_DESCRAMBLER */
/* #define BBM_SPI_30M */
/* #define BBM_I2C_PARALLEL_TSIF */
/* #define BBM_BRAZIL_FREQ */

/* #define BBM_XTAL_FREQ               16000 */
/* #define BBM_XTAL_FREQ               16384 */
/* #define BBM_XTAL_FREQ               18000 */
/* #define BBM_XTAL_FREQ               19200 */
/* #define BBM_XTAL_FREQ               24000 */
/* #define BBM_XTAL_FREQ               24576 */
#define BBM_XTAL_FREQ               26000
/* #define BBM_XTAL_FREQ               27000 */
/* #define BBM_XTAL_FREQ               27120 */
/* #define BBM_XTAL_FREQ               32000 */
/* #define BBM_XTAL_FREQ               37200 */
/* #define BBM_XTAL_FREQ               37400 */
/* #define BBM_XTAL_FREQ               38400 */

#define BBM_BAND_WIDTH              6
/* #define BBM_BAND_WIDTH              7
#define BBM_BAND_WIDTH              8 */

/* #define BBM_TSIF_CLK_ARBITRARY */
#define BBM_TSIF_CLK                48000 /* Up to 48M */
/* #define BBM_TSIF_CLK                32000 */
/* #define BBM_TSIF_CLK                26000 */

#if (BBM_BAND_WIDTH == 6)
#define BBM_TARGET_CLK  97523
#elif (BBM_BAND_WIDTH == 7)
#define BBM_TARGET_CLK  113777
#else /* BBM_BAND_WIDTH == 8 */
#define BBM_TARGET_CLK  130031
#endif

	/* SYS_MD Interrupt */
#define SYS_MD_NO_OFDM_DETECT       0x01
#define SYS_MD_RESYNC_OCCUR         0x02
#define SYS_MD_TMCC_LOCK            0x04
#define SYS_MD_A_LAYER_BER_UPDATE   0x08
#define SYS_MD_B_LAYER_BER_UPDATE   0x10
#define SYS_MD_C_LAYER_BER_UPDATE   0x20
#define SYS_MD_BER_UPDATE           0x40

	/* MD_INT2 Interrupt */
/* TMCC EAB (Emergency Alarm Broadcast) Signal */
#define AUX_INT_TMCC_INT_SRC        0x01
/* The indicator to transmission switching parameter */
#define AUX_INT_TMCC_INDTPS_SRC     0x02
/* AC Dmd Diff Flag - Prev FRM */
#define AUX_INT_AC_PREFRM_SRC       0x04
/* AC EWI (Earthquake Warning Indicator) Start Flag */
#define AUX_INT_AC_EWISTAFLAG_SRC   0x08
#define AUX_INT_SYNC_RELATED_INT    0x10
#define AUX_INT_GPIO_INT_CLEAR      0x20
#define AUX_INT_FEC_RELATED_INT     0x40
/* Auto switch 12/1 seg */
#define AUX_INT_AUTO_SWITCH         0x80

	/* FEC INT */
#define FEC_INT_IRQ_A_TS_ERROR      0x01
#define FEC_INT_IRQ_B_TS_ERROR      0x02
#define FEC_INT_IRQ_C_TS_ERROR      0x04

	/* AUTO SWITCH INT */
#define AUTO_SWITCH_12_SEG          0x00
#define AUTO_SWITCH_1_SEG           0x01

	/* COMMON */
#define BBM_SW_RESET                0x0001
#define BBM_INT_STATUS              0x0002
#define BBM_INT_MASK                0x0003
#define BBM_INT_STS_EN              0x0006
#define BBM_TS0_DATA                0x0008
#define BBM_TS1_DATA                0x0009
#define BBM_TS2_DATA                0x000a
#define BBM_TS3_DATA                0x000b
#define BBM_AC_A_DATA               0x000c
#define BBM_AC_B_DATA               0x000d
#define BBM_AC_C_DATA               0x000e
#define BBM_AC_D_DATA               0x000f
#define BBM_TS_CLK_DIV              0x0010
#define BBM_TS_CTRL                 0x0011
#define BBM_MD_MISO                 0x0012
#define BBM_TS_SEL                  0x0013
#define BBM_TS_PAUSE                0x0014
#define BBM_RF_DEVID                0x0015
#define BBM_CLK_CTRL                0x0016
#define BBM_INT_AUTO_CLEAR          0x0017
#define BBM_INT_PERIOD              0x0018
#define BBM_NON_AUTO_INT_PERIOD     0x0019
#define BBM_STATUS_AUTO_CLEAR_EN    0x001a
#define BBM_TS_BER_CHK              0x001b
#define BBM_BUF_SPIOUT              0x001c
#define BBM_SPI2_CTRL               0x001d
#define BBM_INT_POLAR_SEL           0x0020
#define BBM_TS_PAT_MD               0x0021
#define BBM_INT_FORCE               0x0022
#define BBM_OUT_FORCE               0x0023
#define BBM_CHIP_ID                 0x0026
#define BBM_CHIP_VERSION            0x002a
#define BBM_RF_BYPASS               0x0037
#define BBM_PLL1_ENABLE             0x0050
#define BBM_PLL1_PD                 0x0051
#define BBM_PLL1_RESET              0x0052
#define BBM_PLL1_PRE_POST_SELECTION 0x0055
#define BBM_PLL1_DIVIDER            0x0056
#define BBM_PLL2_ENABLE             0x0060
#define BBM_PLL2_PD                 0x0061
#define BBM_PLL2_RESET              0x0062
#define BBM_PLL2_PRE_POST_SELECTION 0x0065
#define BBM_PLL2_DIVIDER            0x0066
#define BBM_DIVERSITY_DEVICE_SEL    0x0071
#define BBM_PLL_SEL                 0x0085
#define BBM_TS_PAT                  0x00a0
#define BBM_AC_PAT                  0x00a2
#define BBM_VERIFY_TEST             0x00a4
#define BBM_RF_POWER_SAVE           0x00ae
#define BBM_ADC_BIAS                0x00b3
#define BBM_ADC_PWRDN               0x00b4
#define BBM_ADC_RST                 0x00b5
#define BBM_RF_RST                  0x00b9
#define BBM_BB2RF_RFEN              0x00ba
#define BBM_XTAL_OUTBUF_EN          0x00bd
#define BBM_XTAL_OUTBUF_GAIN        0x00be
#define BBM_FUSELOAD                0x00c5
#define BBM_MEMORY_RWM0             0x00e0
#define BBM_MEMORY_RWM1             0x00e1
#define BBM_MEMORY_RWM2             0x00e2
#define BBM_MEMORY_RWM3             0x00e3
#define BBM_BUF_STATUS_CLEAR        0x8000
#define BBM_BUF_OVERRUN             0x8001
#define BBM_BUF_ENABLE              0x8002
#define BBM_BUF_INT_ENABLE          0x8003
#define BBM_FAIL_FRAME_TX           0x8004
#define BBM_SYS_MD_INT_CLR          0x8006
#define BBM_SYS_MD_INT_EN           0x8007
#define BBM_FEC_INT_CLR             0x8008
#define BBM_FEC_INT_EN              0x8009
#define BBM_HANGING_TS              0x800a
#define BBM_HANGING_AC              0x800b
#define BBM_HANGING_ENABLE          0x800c
#define BBM_AUX_INT_EN              0x800d
#define BBM_AUX_STATUS_CLEAR        0x800e
#define BBM_NULL_PID_FILTERING      0x800f
#define BBM_BUF_TS0_START           0x8010
#define BBM_BUF_TS1_START           0x8012
#define BBM_BUF_TS2_START           0x8014
#define BBM_BUF_TS3_START           0x8016
#define BBM_BUF_AC_A_START          0x8018
#define BBM_BUF_AC_B_START          0x801a
#define BBM_BUF_AC_C_START          0x801c
#define BBM_BUF_AC_D_START          0x801e
#define BBM_BUF_TS0_END             0x8020
#define BBM_BUF_TS1_END             0x8022
#define BBM_BUF_TS2_END             0x8024
#define BBM_BUF_TS3_END             0x8026
#define BBM_BUF_AC_A_END            0x8028
#define BBM_BUF_AC_B_END            0x802a
#define BBM_BUF_AC_C_END            0x802c
#define BBM_BUF_AC_D_END            0x802e
#define BBM_BUF_TS0_THR             0x8030
#define BBM_BUF_TS1_THR             0x8032
#define BBM_BUF_TS2_THR             0x8034
#define BBM_BUF_TS3_THR             0x8036
#define BBM_BUF_AC_A_THR            0x8038
#define BBM_BUF_AC_B_THR            0x803a
#define BBM_BUF_AC_C_THR            0x803c
#define BBM_BUF_AC_D_THR            0x803e
#define BBM_BID_FILTER_MODE         0x8040
#define BBM_LAYER_FILTER0           0x8041
#define BBM_LAYER_FILTER1           0x8042
#define BBM_LAYER_FILTER2           0x8043
#define BBM_BUF_READ_LENGTH         0x8064

	/* INTERNAL I2C */
#define BBM_I2C_PR                  0x0030
#define BBM_I2C_CTR                 0x0032
#define BBM_I2C_RXR                 0x0033
#define BBM_I2C_SR                  0x0034
#define BBM_I2C_TXR                 0x0035
#define BBM_I2C_CR                  0x0036

	/* FRONT */
#define BBM_ADC_CTRL                0x1000
#define BBM_REF_AMP                 0x1008
#define BBM_DC_EST_EN               0x1010
#define BBM_IQC_EN                  0x1020
#define BBM_LOW_IF_VALUE            0x1032
#define BBM_NCO_OFFSET              0x103c
#define BBM_ACIF_COEF_00            0x1040
#define BBM_ACIF_COEF_01            0x1041
#define BBM_ACIF_COEF_02            0x1042
#define BBM_ACIF_COEF_03            0x1043
#define BBM_ACIF_COEF_04            0x1044
#define BBM_ACIF_COEF_05            0x1045
#define BBM_ACIF_COEF_06            0x1046
#define BBM_ACIF_COEF_07            0x1047
#define BBM_ACIF_COEF_08            0x1048
#define BBM_ACIF_COEF_09            0x1049
#define BBM_ACIF_COEF_10            0x104a
#define BBM_ACIF_COEF_11            0x104b
#define BBM_ACIF_COEF_12            0x104c
#define BBM_ACIF_COEF_13            0x104d
#define BBM_ACIF_COEF_14            0x104e
#define BBM_ACIF_COEF_15            0x104f
#define BBM_SLPF_COEF_00            0x1050
#define BBM_SLPF_COEF_01            0x1051
#define BBM_SLPF_COEF_02            0x1052
#define BBM_SLPF_COEF_03            0x1053
#define BBM_PGA_GAIN_MAX            0x1064
#define BBM_PGA_GAIN_MIN            0x1065
#define BBM_CSF_GAIN_MAX            0x1069
#define BBM_PSAT_ON_REF_1SEG_QPSK   0x1083
#define BBM_PSAT_ON_REF_1SEG_16QAM  0x1084

	/* SYNC */
#define BBM_FREQ_COMPEN_VAL0        0x2008
#define BBM_FREQ_COMPEN_VAL1        0x2009
#define BBM_FREQ_COMPEN_VAL2        0x200a
#define BBM_FFS_ERR_MAX_EN          0x200b /* with BBM_FREQ_COMPEN_VAL0 */
#define BBM_SFS_FTS_ERR_MAX_1SEG    0x2014
#define BBM_SFS_FTS_ERR_MAX_3SEG    0x2015
#define BBM_SFS_MTH                 0x2016
#define BBM_IIFOECFG_EARLYSTOP_THM  0x2021

	/* FTS */
#define BBM_CFTSCFG_CIRGRMASKEXPSIZE_13SEG  0x2542
#define BBM_CFTSCFG_CIRGRMASKEXPSIZE2_13SEG 0x255c

	/* DEMOD */
#define BBM_SYSTEM_MODE             0x3010
#define BBM_CENTER_CH_NUM           0x3011
#define BBM_GMASK_AUTO              0x3022
#define BBM_RESYNC_ENABLE           0x3040
#define BBM_HOLD_RST_EN             0x3052
/* 1/12-SEG auto switch enable */
#define BBM_OSS_CFG_EN              0x30a0
/* 1/12-SEG auto switch's output status */
#define BBM_OSS_MNT                 0x30ac

	/* ECHO */
#define BBM_ECHOC_EN                0x3100
#define BBM_REF_DELAY_POST          0x3104
#define BBM_RESTART_BY_TS_EN        0x3113

	/* CE */
#define BBM_WSCN_MSQ                0x4063
#define BBM_MFD_MAN_VALUE           0x406f
#define BBM_AD_GAIN_PERIOD          0x4070
#define BBM_FAIP_MTD_SR_SHIFT_VALUE 0x417f
#define BBM_CIR_THR_23              0x41c7
#define BBM_MAN_PARTIAL_EN          0x41f1
#define BBM_MAN_LAYER_A_SEG_NUM     0x41f2
#define BBM_MAN_LAYER_B_SEG_NUM     0x41f3
#define BBM_MAN_LAYER_C_SEG_NUM     0x41f4
#define BBM_MAN_LAYER_A_MOD_TYPE    0x41f5
#define BBM_MAN_LAYER_B_MOD_TYPE    0x41f6
#define BBM_MAN_LAYER_C_MOD_TYPE    0x41f7
#define BBM_MAN_LAYER_A_CODE_RATE   0x41f8
#define BBM_MAN_LAYER_B_CODE_RATE   0x41f9
#define BBM_MAN_LAYER_C_CODE_RATE   0x41fa
#define BBM_MAN_LAYER_A_TI_LENGTH   0x41fb
#define BBM_MAN_LAYER_B_TI_LENGTH   0x41fc
#define BBM_MAN_LAYER_C_TI_LENGTH   0x41fd
#define BBM_FD_RD_LATENCY_1SEG      0x4200
#define BBM_FD_OUT_MODE             0x4208
#define BBM_MSNR_FREQ_S_POW_MAN_VALUE3 0x4247

	/* DIVERSITY */
#define BBM_DIVERSITY_EN            0x4300
#define BBM_DIVERSITY_MODE          0x4301
#define BBM_CN_WEIGHT_USE_EN        0x4304
#define BBM_D_SYNC_TIME_OUT_TH      0x4305
#define BBM_DIV_START_MODE          0x4307
#define BBM_COMB_OFF                0x4314
#define BBM_COMB_CN_OK_FD_EN        0x4333

	/* FEC */
#define BBM_BER_REQ                 0x5000
#define BBM_FEC_LAYER               0x5002
#define BBM_FEC_CTRL_A              0x5010
#define BBM_FEC_CTRL_B              0x5011
#define BBM_FEC_CTRL_C              0x5012
#define BBM_FEC_CTRL                0x5014
#define BBM_TDI_PRE_A               0x5019
#define BBM_TDI_PRE_B               0x501a
#define BBM_TDI_PRE_C               0x501b
#define BBM_BER_AUTO_UP             0x5022
#define BBM_VIT_A_BER_RXD_RSPS      0x5040
#define BBM_VIT_A_BER_ERR_RSPS      0x5042
#define BBM_VIT_A_BER_ERR_BITS      0x5044
#define BBM_VIT_B_BER_RXD_RSPS      0x5050
#define BBM_VIT_B_BER_ERR_RSPS      0x5052
#define BBM_VIT_B_BER_ERR_BITS      0x5054
#define BBM_VIT_C_BER_RXD_RSPS      0x5060
#define BBM_VIT_C_BER_ERR_RSPS      0x5062
#define BBM_VIT_C_BER_ERR_BITS      0x5064
#define BBM_BER_RXD_RSPS            0x5070
#define BBM_BER_ERR_RSPS            0x5072
#define BBM_BER_ERR_BITS            0x5074

	/* DEMAP */
#define BBM_DMP_A_BER_RXD_BITS      0x5080
#define BBM_DMP_A_BER_ERR_BITS      0x5084
#define BBM_DMP_B_BER_RXD_BITS      0x5090
#define BBM_DMP_B_BER_ERR_BITS      0x5094
#define BBM_DMP_C_BER_RXD_BITS      0x50A0
#define BBM_DMP_C_BER_ERR_BITS      0x50A4
#define BBM_DMP_BER_RXD_BITS        0x50B0
#define BBM_DMP_BER_ERR_BITS        0x50B4

	/* GPIO */
#define BBM_GPIO_DATA               0x9000
#define BBM_GPIO_DIR                0x9001
#define BBM_GPIO_SENSE              0x9002
#define BBM_GPIO_BOTH_EDGE          0x9003
#define BBM_GPIO_EVENT              0x9004
#define BBM_GPIO_IE                 0x9005
#define BBM_GPIO_STATUS             0x9006
#define BBM_GPIO_MASKED_STATUS      0x9007
#define BBM_GPIO_INTERRUPT_CLEAR    0x9008

	/* B-CAS DESCRAMBLER */
#define BBM_BCAS_ENABLE             0xa000
#define BBM_BCAS_ROUND              0xa001
#define BBM_BCAS_PID0_DKEY0         0xa020
#define BBM_BCAS_PID0_DKEY1         0xa024
#define BBM_BCAS_PID0_DKEY2         0xa028
#define BBM_BCAS_PID0_DKEY3         0xa02c
#define BBM_BCAS_CBC_INIT_L         0xa0a0
#define BBM_BCAS_CBC_INIT_R         0xa0a4

#define BBM_BCAS_SYS_KEY0           0xa0a8
#define BBM_BCAS_SYS_KEY1           0xa0ac
#define BBM_BCAS_SYS_KEY2           0xa0b0
#define BBM_BCAS_SYS_KEY3           0xa0b4
#define BBM_BCAS_SYS_KEY4           0xa0b8
#define BBM_BCAS_SYS_KEY5           0xa0bc
#define BBM_BCAS_SYS_KEY6           0xa0c0
#define BBM_BCAS_SYS_KEY7           0xa0c4

	/* DM */
#define BBM_DM_DATA                 0xf000

	/*  BUFFER CONFIGURATION */
#define TS0_BUF_START               (0x0000)
#define TS0_BUF_LENGTH              (188 * 321) /* 188 x 640 */
#define TS0_BUF_END                 (TS0_BUF_START + TS0_BUF_LENGTH - 1)
#define TS0_BUF_THR                 (TS0_BUF_LENGTH / 2 - 1)

#define TS1_BUF_START               (TS0_BUF_START + TS0_BUF_LENGTH)
#define TS1_BUF_LENGTH              (0)
#define TS1_BUF_END                 (TS1_BUF_START + TS1_BUF_LENGTH - 1)
#define TS1_BUF_THR                 (0)

#define TS2_BUF_START               (TS1_BUF_START + TS1_BUF_LENGTH)
#define TS2_BUF_LENGTH              (0)
#define TS2_BUF_END                 (TS2_BUF_START + TS2_BUF_LENGTH - 1)
#define TS2_BUF_THR                 (0)

#define TS3_BUF_START               (TS2_BUF_START + TS2_BUF_LENGTH)
#define TS3_BUF_LENGTH              (0)
#define TS3_BUF_END                 (TS3_BUF_START + TS3_BUF_LENGTH - 1)
#define TS3_BUF_THR                 (0)

#define AC_A_BUF_START              (TS3_BUF_START + TS3_BUF_LENGTH)
#define AC_A_BUF_LENGTH             (204) /* 204 x 2 */
#define AC_A_BUF_END                (AC_A_BUF_START + AC_A_BUF_LENGTH - 1)
#define AC_A_BUF_THR                (AC_A_BUF_LENGTH / 2 - 1)

#define AC_B_BUF_START              (AC_A_BUF_START + AC_A_BUF_LENGTH)
#define AC_B_BUF_LENGTH             (2040) /* 2040 x 2 */
#define AC_B_BUF_END                (AC_B_BUF_START + AC_B_BUF_LENGTH - 1)
#define AC_B_BUF_THR                (AC_B_BUF_LENGTH / 2 - 1)

#define AC_C_BUF_START              (AC_B_BUF_START + AC_B_BUF_LENGTH)
#define AC_C_BUF_LENGTH             (408) /* 408 x 2 */
#define AC_C_BUF_END                (AC_C_BUF_START + AC_C_BUF_LENGTH - 1)
#define AC_C_BUF_THR                (AC_C_BUF_LENGTH / 2 - 1)

#define AC_D_BUF_START              (AC_C_BUF_START + AC_C_BUF_LENGTH)
#define AC_D_BUF_LENGTH             (28) /* 28 x 2 */
#define AC_D_BUF_END                (AC_D_BUF_START + AC_D_BUF_LENGTH - 1)
#define AC_D_BUF_THR                (AC_D_BUF_LENGTH / 2 - 1)

#ifdef __cplusplus
}
#endif

#endif /* __FC8300_REGS_H__ */

