#include <rtthread.h>
#include "motors.h"
#include "config.h"

typedef DC_Motor Motor_Class;

class Mecanum_Car
{
public:
    int Now_spd;
    Mecanum_Car(Motor_Class *_LF_Wheel, Motor_Class *_RF_Wheel, Motor_Class *_LR_Wheel, Motor_Class *_RR_Wheel)
    {
        this->LF_Wheel = _LF_Wheel;
        this->RF_Wheel = _RF_Wheel;
        this->LR_Wheel = _LR_Wheel;
        this->RR_Wheel = _RR_Wheel;
    }
    void Init(void);
    void RunSpd(int Spd);
	void ROS_Move(float Line_vel,float Angle_vel);
	void PID_RunSpd(int Spd);
	void PID_ROS_Move(float Line_vel,float Angle_vel);
	void Increment_PID(void);
	void Update_PID(float _kp, float _ki, float _kd);
	
	int LR_Wheel_Spd;
	int RR_Wheel_Spd;
	int RF_Wheel_Spd;
	int LF_Wheel_Spd;
private:
	struct rt_device_pwm *pwm_dev;
    Motor_Class *LF_Wheel;
    Motor_Class *RF_Wheel;
    Motor_Class *LR_Wheel;
    Motor_Class *RR_Wheel;
};


//struct rt_device_pwm *pwm_dev;
