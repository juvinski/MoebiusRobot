#include <rtthread.h>
#include <rtdevice.h>
#include "car.h"
#include "mpu6050.h"		//调用.c文件接口访问IMU

#define PWM_DEV_NAME "pwm8"

void Mecanum_Car::Increment_PID()
{
	LF_Wheel->Incremental_PID(LF_Wheel_Spd);RF_Wheel->Incremental_PID(RF_Wheel_Spd);
	LR_Wheel->Incremental_PID(LR_Wheel_Spd);RR_Wheel->Incremental_PID(RR_Wheel_Spd);
}

void Mecanum_Car::Init()
{
	pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
    LF_Wheel->init(pwm_dev,1); RF_Wheel->init(pwm_dev,0); 
	LR_Wheel->init(pwm_dev,1); RR_Wheel->init(pwm_dev,0); //初始化电机PWM
}

void Mecanum_Car::RunSpd(int speed)
{
    LF_Wheel->runSpd(speed); RF_Wheel->runSpd(speed);
    LR_Wheel->runSpd(speed); RR_Wheel->runSpd(speed);
}

void Mecanum_Car::ROS_Move(float Line_vel,float turnBias)
{
    LF_Wheel->runSpd(+Line_vel + turnBias); RF_Wheel->runSpd(+Line_vel - turnBias);
    LR_Wheel->runSpd(+Line_vel + turnBias); RR_Wheel->runSpd(+Line_vel - turnBias);
}

void Mecanum_Car::PID_RunSpd(int speed)
{
	RR_Wheel_Spd = LF_Wheel_Spd = LR_Wheel_Spd = RF_Wheel_Spd = constrain(speed, -MAX_RPM, MAX_RPM);
}

void Mecanum_Car::PID_ROS_Move(float Line_vel,float turnBias)
{
    LF_Wheel->runSpd(+Line_vel + turnBias); RF_Wheel->runSpd(+Line_vel - turnBias);
    LR_Wheel->runSpd(+Line_vel + turnBias); RR_Wheel->runSpd(+Line_vel - turnBias);
}

void Mecanum_Car::Update_PID(float _kp, float _ki, float _kd)
{
	LF_Wheel->Update_PID(_kp, _ki, _kd); RF_Wheel->Update_PID(_kp, _ki, _kd);
    LR_Wheel->Update_PID(_kp, _ki, _kd); RR_Wheel->Update_PID(_kp, _ki, _kd);
}

