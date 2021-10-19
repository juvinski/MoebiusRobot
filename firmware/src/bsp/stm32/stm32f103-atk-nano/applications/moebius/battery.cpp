#include "battery.h"


#define ADC_DEV_NAME        "adc1"      /* ADC �豸���� */
#define ADC_DEV_CHANNEL     5           /* ADC ͨ�� */
#define REFER_VOLTAGE       330         /* �ο���ѹ 3.3V,���ݾ��ȳ���100����2λС��*/
#define CONVERT_BITS        (1 << 12)   /* ת��λ��Ϊ12λ */

Battery::Battery(float _threshold, float _volt_min, float _volt_max)
{
	threshold = _threshold;
	volt_min = _volt_min;
	volt_max = _volt_max;
}

void Battery::init()
{

    /* �����豸 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
    }
	/* ʹ���豸 */
    rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
	
}

float Battery::get_volt()
{
	/* ��ȡ����ֵ */
    uint32_t value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
	return (((float)value * 3.3) / 4095.0) * 11.9;
}

float Battery::get_battery_notifier()
{
	float volt = get_volt();
	if(volt > volt_max)
		volt = volt_max;

	return ((volt- volt_min) / (volt_max - volt_min) * 100);
}

bool Battery::get_battery_low()
{
	if(get_battery_notifier() > threshold)
		return false;
	else
		return true;
}
