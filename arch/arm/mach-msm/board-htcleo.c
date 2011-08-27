/* linux/arch/arm/mach-msm/board-htcleo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/a1026.h>
#include <linux/capella_cm3602.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>

#include "board-htcleo.h"
#include "devices.h"

static struct android_pmem_platform_data mdp_pmem_pdata = {
	.name		= "pmem",
	.start		= MSM_PMEM_MDP_BASE,
	.size		= MSM_PMEM_MDP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name		= "pmem_adsp",
	.start		= MSM_PMEM_ADSP_BASE,
	.size		= MSM_PMEM_ADSP_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name		= "pmem_camera",
	.start		= MSM_PMEM_CAMERA_BASE,
	.size		= MSM_PMEM_CAMERA_SIZE,
	.no_allocator	= 1,
	.cached		= 1,
};

static struct platform_device android_pmem_mdp_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &mdp_pmem_pdata
	},
};

static struct platform_device android_pmem_adsp_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_adsp_pdata,
	},
};

static struct platform_device android_pmem_camera_device = {
	.name		= "android_pmem",
	.id		= 2,
	.dev		= {
		.platform_data = &android_pmem_camera_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static struct i2c_board_info base_i2c_devices[] = {
};

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
	&msm_device_smd,
	&ram_console_device,
	&android_pmem_mdp_device,
	&android_pmem_adsp_device,
	&android_pmem_camera_device,
	&msm_device_i2c,
};

static struct msm_acpu_clock_platform_data htcleo_clock_data = {
	.acpu_switch_time_us	= 20,
	.max_speed_delta_khz	= 256000,
	.vdd_switch_time_us	= 62,
	.power_collapse_khz	= 245000,
	.wait_for_irq_khz	= 245000,
};

static void htcleo_reset(void)
{
	gpio_set_value(HTCLEO_GPIO_PS_HOLD, 0);
}

static void __init htcleo_init(void)
{
	printk("htcleo_init()\n");

	msm_hw_reset_hook = htcleo_reset;

	msm_acpu_clock_init(&htcleo_clock_data);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	i2c_register_board_info(0, base_i2c_devices,
		ARRAY_SIZE(base_i2c_devices));
	
	/* Blink the camera LED shortly to show that we're alive! */
	volatile unsigned *bank6_in = (unsigned int*)(MSM_GPIO1_BASE + 0x0864);
	volatile unsigned *bank6_out = (unsigned int*)(MSM_GPIO1_BASE + 0x0814);
	*bank6_out = *bank6_in ^ 0x200000;
	mdelay(50);
	*bank6_out = *bank6_in | 0x200000;
	mdelay(200);
}

static void __init htcleo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = MSM_EBI1_BANK0_BASE;
//	mi->bank[0].node = PHYS_TO_NID(MSM_EBI1_BANK0_BASE);
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
}

#if defined(CONFIG_VERY_EARLY_CONSOLE)
#if defined(CONFIG_HTC_FB_CONSOLE)
int __init htc_fb_console_init(void);
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
int __init ram_console_early_init(void);
#endif
#endif

static void __init htcleo_map_io(void)
{
	msm_map_qsd8x50_io();
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);

#if defined(CONFIG_VERY_EARLY_CONSOLE)
	// Init our consoles _really_ early
#if defined(CONFIG_HTC_FB_CONSOLE)
	htc_fb_console_init();
#endif
#if defined(CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT)
	ram_console_early_init();
#endif
#endif
}

extern struct sys_timer msm_timer;

MACHINE_START(HTCLEO, "htcleo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= 0x20000100,
	.fixup		= htcleo_fixup,
	.map_io		= htcleo_map_io,
	.init_irq	= msm_init_irq,
	.init_machine	= htcleo_init,
	.timer		= &msm_timer,
MACHINE_END
