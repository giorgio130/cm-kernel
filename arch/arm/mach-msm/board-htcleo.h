/* arch/arm/mach-msm/board-htcleo.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H

#include <mach/board.h>

#define MSM_EBI1_BANK0_BASE	0x11800000
#define MSM_EBI1_BANK0_SIZE	0x18C00000 /* rounded down from 0x18DC0000 */

#define MSM_FB_BASE		0x2A5C0000
#define MSM_FB_SIZE		0x00600000

#define MSM_GPU_MEM_BASE	0x2ABC0000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_PMEM_MDP_BASE	0x2AEC0000
#define MSM_PMEM_MDP_SIZE	0x02000000

#define MSM_PMEM_ADSP_BASE	0x2CEC0000
#define MSM_PMEM_ADSP_SIZE	0x02900000

#define MSM_PMEM_CAMERA_BASE	0x2F7C0000
#define MSM_PMEM_CAMERA_SIZE	0x00800000

// MSM_RAM_CONSOLE uses the last 0x40000 of memory, defined in msm_iomap.h

#define HTCLEO_GPIO_PS_HOLD		25

#define HTCLEO_GPIO_UP_INT_N		35
#define HTCLEO_GPIO_UP_RESET_N	82
#define HTCLEO_GPIO_LS_EN_N		119

#define HTCLEO_GPIO_TP_INT_N		92
#define HTCLEO_GPIO_TP_LS_EN		93
#define HTCLEO_GPIO_TP_EN		160

#define HTCLEO_GPIO_POWER_KEY		94
#define HTCLEO_GPIO_SDMC_CD_REV0_N	153

#define HTCLEO_GPIO_WIFI_SHUTDOWN_N	127
#define HTCLEO_GPIO_WIFI_IRQ		152

#define HTCLEO_GPIO_BALL_UP		38
#define HTCLEO_GPIO_BALL_DOWN		37
#define HTCLEO_GPIO_BALL_LEFT		145
#define HTCLEO_GPIO_BALL_RIGHT	21

#define HTCLEO_GPIO_BT_UART1_RTS	43
#define HTCLEO_GPIO_BT_UART1_CTS	44
#define HTCLEO_GPIO_BT_UART1_RX	45
#define HTCLEO_GPIO_BT_UART1_TX	46
#define HTCLEO_GPIO_BT_RESET_N	146
#define HTCLEO_GPIO_BT_SHUTDOWN_N	128

#define HTCLEO_GPIO_BT_WAKE		57
#define HTCLEO_GPIO_BT_HOST_WAKE	86

#define HTCLEO_GPIO_PROXIMITY_INT_N	90
#define HTCLEO_GPIO_PROXIMITY_EN	120

#define HTCLEO_GPIO_DS2482_SLP_N	87
#define HTCLEO_GPIO_VIBRATOR_ON	89
/* Compass */
#define HTCLEO_REV0_GPIO_COMPASS_INT_N	36

#define HTCLEO_GPIO_COMPASS_INT_N	153
#define HTCLEO_GPIO_COMPASS_RST_N	107
#define HTCLEO_PROJECT_NAME          "htcleo"
#define HTCLEO_LAYOUTS { 			   \
	{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
	{ { 0, -1, 0}, { 1,  0,  0}, {0, 0,  1} }, \
	{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
}

/* Audio */
#define HTCLEO_AUD_JACKHP_EN		157
#define HTCLEO_AUD_2V5_EN		158
#define HTCLEO_AUD_MICPATH_SEL 	111
#define HTCLEO_AUD_A1026_INT		112
#define HTCLEO_AUD_A1026_WAKEUP 	113
#define HTCLEO_AUD_A1026_RESET 	129
#define HTCLEO_AUD_A1026_CLK		 -1

/* Bluetooth PCM */
#define HTCLEO_BT_PCM_OUT		68
#define HTCLEO_BT_PCM_IN		69
#define HTCLEO_BT_PCM_SYNC		70
#define HTCLEO_BT_PCM_CLK		71
/* flash light */
#define HTCLEO_GPIO_FLASHLIGHT_TORCH	58
#define HTCLEO_GPIO_FLASHLIGHT_FLASH	84

#define HTCLEO_GPIO_LED_3V3_EN	85
#define HTCLEO_GPIO_LCD_RST_N		29

/* 3.5mm remote control key interrupt shutdown signal */
#define HTCLEO_GPIO_35MM_KEY_INT_SHUTDOWN	19

#define HTCLEO_GPIO_DOCK		106

#endif /* __ARCH_ARM_MACH_MSM_BOARD_HTCLEO_H */
