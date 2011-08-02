/*
 * Copyright (C) 2007 HTC Incorporated
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
#ifndef _DS2746_BATTERY_H_
#define _DS2746_BATTERY_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>
//#include <linux/wrapper_types.h>
//#include <linux/ds2746_param.h>


/* battery charging state*/

enum {
	CHARGE_STATE_UNKNOWN,   		/* before anything is ready, we are in this state. shall default low current charge and show charging LED*/
	CHARGE_STATE_PREDICTION,		/* in normal case, we need to enter prediction for 10 seconds for 1st KADC*/
	CHARGE_STATE_DISCHARGE, 		/* cable out state*/
	CHARGE_STATE_CHARGING,  		/* charging state*/
	CHARGE_STATE_PENDING,   		/* charging state but no good*/
	CHARGE_STATE_FULL_WAIT_STABLE,  /* charging state but going full*/
	CHARGE_STATE_FULL_CHARGING, 	/* charging full, keep charging*/
	CHARGE_STATE_FULL_PENDING,  	/* charging full, stop charging*/
};      

enum {
	THERMAL_300,
	THERMAL_600,
	THERMAL_1000,
};



struct ds2746_platform_data {
	int (*func_get_thermal_id)(void);
};

/* battery behavior constant*/

#define BATTERY_PERCENTAGE_UNKNOWN  0xFF
#define BATTERY_LOW_PERCENTAGE	    10  	/* in 1%*/
#define BATTERY_CRITICAL_PERCENTAGE 5   	/* in 1%*/
#define BATTERY_EMPTY_PERCENTAGE    0   	/* in 1%*/

/* battery algorithm public functions*/

int get_state_check_interval_min_sec( void);
//BOOL do_power_alg( BOOL is_event_triggered);
//void power_alg_init( struct poweralg_config_type *debug_config);
void power_alg_preinit( void);
int ds2746_blocking_notify( unsigned long val, void *v);
void ds2746_charger_control( int type);
int ds2746_i2c_write_u8( u8 value, u8 reg);
int ds2746_i2c_read_u8( u8* value, u8 reg);
//void calibrate_id_ohm( struct battery_type *battery);
/* external function implemented by upper layer*/

/*extern void powerlog_to_file(struct poweralg_type* poweralg);*/
/*extern void update_os_batt_status(struct poweralg_type* poweralg);*/

#ifdef CONFIG_BATTERY_DS2746
extern int ds2746_register_notifier( struct notifier_block *nb);
extern int ds2746_unregister_notifier( struct notifier_block *nb);
extern int ds2746_get_battery_info( struct battery_info_reply *batt_info);
extern ssize_t htc_battery_show_attr( struct device_attribute *attr, char *buf);
#else
static int ds2746_register_notifier( struct notifier_block *nb) {

	return 0;
}
static int ds2746_unregister_notifier( struct notifier_block *nb) {

	return 0;
}
static int ds2746_get_battery_info( struct battery_info_reply *batt_info) {

	batt_info->level = 10;
	return 0;
}
extern ssize_t htc_battery_show_attr( struct device_attribute *attr, char *buf) {

	return 0;
}

#endif

#endif
