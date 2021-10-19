#ifndef SENSOR_BOS_BMI160_H__
#define SENSOR_BOS_BMI160_H__

#include <sensor.h>

#ifdef __cplusplus
extern "C"
{
#endif
	
int rt_hw_bmx160_init(const char *name, struct rt_sensor_config *cfg);
	
#ifdef __cplusplus
}
#endif

#endif
