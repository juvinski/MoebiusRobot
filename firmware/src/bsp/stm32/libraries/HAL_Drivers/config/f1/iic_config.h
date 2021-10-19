/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-13     zylx         first version
 */

#ifndef __IIC_CONFIG_H__
#define __IIC_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_I2C2
#ifndef I2C2_BUS_CONFIG
#define I2C2_BUS_CONFIG                         \
    {                                           \
       .name                    = "i2c2",       \
    }
#endif /* I2C2_BUS_CONFIG */
#endif /* BSP_USING_I2C2 */

#ifdef __cplusplus
}
#endif

#endif /* __PWM_CONFIG_H__ */
