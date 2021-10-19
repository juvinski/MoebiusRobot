/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <rtthread.h>
#include <geometry_msgs/AccelStamped.h>

class MPU6050
{
public:
	void init();
	void measure_gyroscope();
	void measure_acceleration();
	void measure_magnetometer();
	void static_compensation();
	void imu_serial_scope();

	geometry_msgs::Vector3 static_acc, static_gyro, static_magn;
	geometry_msgs::Vector3 raw_acceleration, raw_rotation, raw_magnetic_field;
};

#endif
