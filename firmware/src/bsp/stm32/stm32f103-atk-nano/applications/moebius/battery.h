#ifndef _BATTERY_H_
#define _BATTERY_H_ 

#include "config.h"
#include <rtthread.h>
#include <rtdevice.h>

class Battery {
public:
	Battery(float _threshold, float _volt_min, float _volt_max);
	void init();
	float get_volt();
	float get_battery_notifier();
	bool get_battery_low();

private:
	rt_adc_device_t adc_dev;
	float threshold;
	float volt_min;
	float volt_max;
};

#endif // _BATTERY_H_
