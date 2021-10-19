/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 */

#ifndef _BMI160_DEV_H_
#define _BMI160_DEV_H_

#include <rtthread.h>
#include <math.h>

#include "sensor_bosch_bmx160.h"
#include "mpu6xxx.h"

#include <geometry_msgs/AccelStamped.h>



class Imu_Sensor
{
public:
	unsigned char log_type;
	int init();
	void measure_gyroscope();
	void measure_acceleration();
	void measure_magnetometer();
	geometry_msgs::Vector3 static_acc, static_gyro, static_magn;
	geometry_msgs::Vector3 raw_acceleration, raw_rotation, raw_magnetic_field;
};

void imu_static_compensation(geometry_msgs::Vector3 &static_gyro);

#endif
