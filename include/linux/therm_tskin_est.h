/*
 * include/linux/therm_est.h
 *
 * Copyright (c) 2010-2019, NVIDIA CORPORATION.  All rights reserved.
 * Copyright (c) 2022, CTCaer.
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

#ifndef _LINUX_THERM_EST_H
#define _LINUX_THERM_EST_H

#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/pwm.h>

#define MODES_LEN (2)
#define COEFF_LEN (2)

#define MAX_ACTIVE_STATES	10
#define MAX_TIMER_TRIPS		10

#define MAX_SUBDEVICE_GROUP	2

struct therm_fan_est_subdevice {
	const char *dev_data;
	int (*get_temp)(const char *, int *);
	int group;
	/*
	 * as we read coeffs array from the device tree,
	 * specify type that has known width - 32 bits
	 *
	 * 'long' does not work, as its size if arch-dependent
	 */
	s32 coeffs_mode[2][2];
	int temp;
	int temp_adjusted;
};

struct therm_fan_est_data {
	long toffset;
	long polling_period;
	int ndevs;
	char *cdev_type;
	int active_trip_temps[MAX_ACTIVE_STATES];
	int active_hysteresis[MAX_ACTIVE_STATES];
	struct thermal_zone_params *tzp;
	struct therm_fan_est_subdevice devs[];
};

struct fan_dev_data {
	int next_state;
	int active_steps;
	int *fan_rpm;
	int *fan_pwm;
	int *fan_rru;
	int *fan_rrd;
	int *fan_state_cap_lookup;
	int num_profiles;
	int current_profile;
	const char **fan_profile_names;
	int **fan_profile_pwms;
	int *fan_profile_caps;
	struct workqueue_struct *workqueue;
	int fan_temp_control_flag;
	struct pwm_device *pwm_dev;
	bool pwm_legacy_api;
	int fan_cap_pwm;
	int fan_cur_pwm;
	int next_target_pwm;
	struct thermal_cooling_device *cdev;
	struct delayed_work fan_ramp_work;
	struct delayed_work fan_hyst_work;
	int step_time;
	long long precision_multiplier;
	struct mutex fan_state_lock;
	int pwm_period;
	int fan_pwm_max;
	struct device *dev;
	int tach_gpio;
	int tach_irq;
	atomic_t tach_enabled;
	int fan_state_cap;
	int pwm_gpio;
	int pwm_id;
	int fan_startup_pwm;
	int fan_startup_time;
	bool fan_kickstart; /*flag to check if fan is kickstart*/
	bool kickstart_en;  /*flag to check if kickstart feature is enabled*/
	enum pwm_polarity fan_pwm_polarity;
	int suspend_state;
	const char *name;
	struct regulator *fan_reg;
	bool is_fan_reg_enabled;
	struct dentry *debugfs_root;
	/* for tach feedback */
	atomic64_t rpm_measured;
	struct delayed_work fan_tach_work;
	struct workqueue_struct *tach_workqueue;
	int tach_period;
	spinlock_t irq_lock;
	int irq_count;
	u64 first_irq;
	u64 last_irq;
	u64 old_irq;

	bool   continuous_gov;
};

#define MULTIQP (100)
#define DEFERRED_RESUME_TIME 3000
#define THERMAL_GOV_PID "pid_thermal_gov"
#define THERMAL_CONTINUOUS_GOV "continuous_therm_gov"

struct therm_tskin_fan_est {
	long cur_temp;
	long pre_temp;
	long polling_period;
	struct workqueue_struct *workqueue;
	struct delayed_work therm_fan_est_work;
	long toffset;
	int ntemp;
	int ndevs;
	struct therm_fan_est_subdevice *devs;
	struct thermal_zone_device *thz;
	int current_trip_level;
	const char *cdev_type;
	rwlock_t state_lock;
	int num_profiles;
	int current_profile;
	const char **fan_profile_names;
	s32 **fan_profile_trip_temps;
	s32 **fan_profile_trip_hysteresis;
	s32 active_trip_temps[MAX_ACTIVE_STATES];
	s32 active_hysteresis[MAX_ACTIVE_STATES];
	struct thermal_zone_params *tzp;
	int num_resources;
	int trip_length;
	const char *name;
	bool is_pid_gov;
	bool reset_trip_index;
	/* allow cooling device to turn off at higher temperature if sleep */
	bool sleep_mode;
	int nonsleep_hyst;

	bool is_continuous_gov;
};

#if IS_ENABLED(CONFIG_THERMAL_GOV_CONTINUOUS)
int continuous_thermal_gov_update_params(struct thermal_zone_params *tzp,
		struct device_node *np);
#else
static inline int continuous_thermal_gov_update_params(
		struct thermal_zone_params *tzp, struct device_node *np)
{
	return 0;
}
#endif

#endif /* _LINUX_THERM_EST_H */
