#include <rtthread.h>
#include <rtdevice.h>
#include "config.h"
#include "motors.h"
#include "car.h"
#define PERIOD 12500

int DC_Motor::counts_per_rev_ = COUNTS_PER_REV;

DC_Motor::DC_Motor(uint8_t port)
{
	this->pin_a = Motor_Port[port].Pin_A;
	this->pin_b = Motor_Port[port].Pin_B;
	this->pin_pwm_ch = Motor_Port[port].PWM_CH;
	rt_strncpy(this->name, Motor_Port[port].encode_name, rt_strlen(Motor_Port[port].encode_name));
	pulse_encoder_dev = rt_device_find(name);
	kp = K_P;
	ki = K_I;
	kd = K_D;
}

void DC_Motor::init(struct rt_device_pwm* dc_pwm, unsigned char dir)
{
	this->dc_pwm = dc_pwm;
	this->dir = dir;
	rt_pin_mode(pin_a, PIN_MODE_OUTPUT);
	rt_pin_mode(pin_b, PIN_MODE_OUTPUT);
	rt_pwm_enable(dc_pwm, pin_pwm_ch);
	
	/* 以只读方式打开设备 */
    rt_device_open(pulse_encoder_dev, RT_DEVICE_OFLAG_RDONLY);
}

void DC_Motor::runSpd(int spd)
{
	if(dir)	spd *= -1;
	spd *= 30;
	if(spd == 0)
	{
		rt_pin_write(pin_a, 0);
		rt_pin_write(pin_b, 0);
		rt_pwm_set(dc_pwm, pin_pwm_ch, PERIOD, 0);
		return ;
	}
	else if(spd > 0)
	{
		rt_pin_write(pin_a, 0);
		rt_pin_write(pin_b, 1);
	}
	else
	{
		rt_pin_write(pin_a, 1);
		rt_pin_write(pin_b, 0);
		spd *= -1;
	}
	rt_pwm_set(dc_pwm, pin_pwm_ch, PERIOD, spd);
}

void DC_Motor::updateSpd()
{
	//this function calculates the motor's RPM based on encoder ticks and delta time
	unsigned long current_time = rt_tick_get();
	unsigned long dt = current_time - prev_update_time_;
	
	//convert the time from milliseconds to minutes
	double dtm = (double)dt / 60000;
	rt_device_read(pulse_encoder_dev, 0, &encoder_ticks, 1);

	double delta_ticks = encoder_ticks - prev_encoder_ticks_;

	//calculate wheel's speed (RPM)
	now_spd = -(delta_ticks / counts_per_rev_) / dtm;   
	if(now_spd - last_spd > 500 || last_spd - now_spd > 500)	//屏蔽无效数据
	{
		rt_kprintf("Encoder err data = %d, last: %d\r\n", now_spd, last_spd);
		now_spd = last_spd;		//如果瞬时加速过高，则认为此次数据无效
	}
	else
	{
		if(dir)	now_spd *= -1;
		last_spd = now_spd;
	}
	prev_update_time_ = current_time;
	prev_encoder_ticks_ = encoder_ticks;
}

long DC_Motor::read_odom()
{
	rt_device_read(pulse_encoder_dev, 0, &odom, 1);
	if(odom - Last_odom > 30000 || Last_odom - odom > 30000)	//屏蔽无效数据
	{
		rt_kprintf("Odom err data = %d, last: %d\r\n", odom, Last_odom);
		odom = Last_odom;		
		rt_device_write(pulse_encoder_dev, 0, &odom, 1);
	}
	Last_odom = odom;
	if(!dir)	odom *= -1;
	return odom;
}

//PID控制器必须使用中断调用，10ms执行一次
void DC_Motor::Incremental_PID(int target)
{ 
	updateSpd();
	if(Last_tar > target)				//微分缓冲器
		Last_tar-=2;
	else if(Last_tar < target)
		Last_tar+=2;
	Bias = now_spd - Last_tar;          //计算增量偏差
	Pwm += kp * (Bias - Last_bias) + ki * Bias;   		//增量PI控制器
	if(Pwm > MOTOR_MAX_PWM)	 Pwm = MOTOR_MAX_PWM;		//输出限幅
	if(Pwm < -MOTOR_MAX_PWM) Pwm = -MOTOR_MAX_PWM;
	Last_bias=Bias;	                   					//保存上一次偏差 
	
	runSpd(Pwm);
}

void DC_Motor::Update_PID(float _kp, float _ki, float _kd)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;
}

