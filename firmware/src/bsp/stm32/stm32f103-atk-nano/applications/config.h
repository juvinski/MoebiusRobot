// <<< Use Configuration Wizard in Context Menu >>>

//<h> rt_thread config

//	<s> vresion info
#define VERSION "V1.0.3"
/***********************************************************
//版本记录
//2020-07-13	V1.0.1	1.调试完成首个版本
//2020-08-17 	V1.0.2	1.修复陀螺仪上报的警告问题
						2.添加Melodic版本兼容
						3.降低里程计发布频率为1Hz,
						4.增加ROS连接识别、掉线重连功能
//2020-08-19	V1.0.3  1.PID控制器优化
						2.修复编码器偶尔出现的数据异常问题

******************************************************/

//<o> ROSVERSION
//<i> set rosversion
//<1=> Kintic
//<2=> Melodic
//<3=> Noetic
#define ROSVERSION 2

#define Kintic	1
#define Melodic	2
#define Noetic	3

/***************************Debug********************************/
//	<e> Debug enable
#define USING_DEBUG     1234
//	<q> ROS serial debug
#define ROS_DEBUG	1
//<o>Debug output interval
//<0-500>
//<i>0-500ms
#define DEBUG_RATE 1

#define LED			45
//#define LED			29

//	</e>
//</h>
/***************************Motor params********************************/
//<h> Motor params
//<i> Set motor working mode and param
#define PWM_DEV_NAME			"pwm8"
#define SPEED_LEVEL				30

/** Motor param **/
//<o>PWM Bits
//<8-12>
//<i>8-12ms
#define PWM_BITS 			8

//<o>Max revolutions per minute
//<0-255>
//<i>0-255rpm
#define MAX_RPM 			245				//motor's maximum RPM

//<o>Encoder counts per rev
//<0-7000>
//<i>0-7000
//#define COUNTS_PER_REV 		1560			//wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)
#define COUNTS_PER_REV 		1300			//wheel encoder's no of ticks per rev(gear_ratio * pulse_per_rev)

//<o>Wheel diameter
//<0-1>
//<i>0-1
#define WHEEL_DIAMETER 		0.080			//wheel's diameter in meters


//<o>base width
//<0-7000>
//<i>0-7000rpm
#define BASE_WIDTH 			0.26

//</h>

/***************************IMU config********************************/

/***************************IMU config********************************/
//<h> IMU_TYPE
//<i> set imu
#define IMU_DEV_MPU6050	1
#define IMU_DEV_BMX160	2
#define IMU_DEV_GY85	3

//<s>config imu bus
//<i>Set imu name "i2c1" or "i2c2"
#define IMU_DEVICE_NAME	"i2c2"

//<o>IMU_TYPE: set imu type
//<1=> mpuxxxx
//<2=> bmx160
//<3=> gy85
#define IMU_TYPE			1


//</h>

/***************************Filter param********************************/
//<h> Filter param
#define PI  	3.1415926

//<o>Imu publish rate
//<0-500>
//<i>0-500
#define IMU_PUBLISH_RATE 20 //hz
//<o>Velocity publish rate
//<0-500>
//<i>0-500
#define VEL_PUBLISH_RATE 1 //hz
//<o>Command rate
//<0-500>
//<i>0-500
#define COMMAND_RATE 10 //hz

/* PID Original
#define K_P    8.0 // P constant
#define K_I    1.0 // I constant
#define K_D    0.2 // D constant
 */
#define K_P    6.2 // P constant
#define K_I    0.8 // I constant
#define K_D    0.2 // D constant

//</h>


#define ACC_Gain 0.000061//由读数转换为加速度值，=1/16384


 
// <<< end of configuration section >>>
