#ifndef _MOTORS_H_
#define _MOTORS_H_

#include <rtthread.h>

#define PORT_RR		0
#define PORT_LR		1
#define PORT_LF		2
#define PORT_RF		3

#define MOTOR_MAX_PWM		500

#define constrain(amt,low,high) \
	((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct
{
  uint8_t Pin_A;   
  uint8_t Pin_B;   
  uint8_t Pin_PWM; 
  uint8_t PWM_CH;	//Tim_ch
  const char *  encode_name;
} Motor_port_type;

const Motor_port_type Motor_Port[4] =
{
  //PIN_A   PIN_B   PWM     TIM    Encode name
  { 16,     17,     38,     1,     "pulse5"},	//Right rear
  { 37,     36,     39,     2,     "pulse4"},	//Left rear
  { 20,     21,     40,     3,     "pulse3"},	//Left front
  { 50,     44,     41,     4,     "pulse2"},	//Right front
//**************************************************************
};

class DC_Motor
{
public:
    int now_spd;
	int last_spd;
	static int counts_per_rev_;
    float kp;
    float ki;
    float kd;

    DC_Motor(uint8_t port);
	void init(struct rt_device_pwm* dc_pwm, unsigned char dir);
	void runSpd(int spd);
	void updateSpd(void);
	long read_odom(void);
	void Incremental_PID(int Target);
	int PWM_Tracker(int pwm, int dy);
	void Update_PID(float kp, float ki, float kd);

private:
    //The pins
	char name[10];
	int Bias, Pwm, Last_bias, Last_tar;
	rt_device_t pulse_encoder_dev;   /* Âö³å±àÂëÆ÷Éè±¸¾ä±ú */
	long prev_encoder_ticks_;
	long encoder_ticks;
	unsigned long prev_update_time_;
	struct rt_device_pwm* dc_pwm;
	unsigned char dir;
	int odom;
	int Last_odom;
	int pin_a;
	int pin_b;
	int pin_pwm_ch;
};

#endif

