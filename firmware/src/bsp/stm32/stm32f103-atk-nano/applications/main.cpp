#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "config.h"

#include "Kinematics.h"

#include <ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#if (ROSVERSION == Kintic)
#include <riki_msgs/PID.h>
#include <riki_msgs/Velocities.h>
#include <riki_msgs/RawImu.h>
#elif(ROSVERSION == Melodic)
#include <mbs_msgs/PID.h>
#include <mbs_msgs/Velocities.h>
#include <mbs_msgs/RawImu.h>
#endif

#include "motors.h"
#include "car.h"

#include "imu_dev.h"
#include "battery.h"

#if (ROSVERSION == Kintic)
#define mbs_msgs	riki_msgs
#elif(ROSVERSION == Melodic)

#endif


//打印浮点数
static void rt_fprintf(const char* str, float dat)
{
	int fh, fl;
	int x = dat * 100.0;
	fh = x / 100, fl = abs(x) % 100;
	rt_kprintf("%s: %d.%d\n", str, fh, fl);
}

DC_Motor 	LF_Wheel_Motor(PORT_LF), RF_Wheel_Motor(PORT_RF),LR_Wheel_Motor(PORT_LR), RR_Wheel_Motor(PORT_RR);
Mecanum_Car	robot(&LF_Wheel_Motor, &RF_Wheel_Motor, &LR_Wheel_Motor, &RR_Wheel_Motor);
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);

Imu_Sensor	Imu_dev;
Battery		power(10.0, 9.6, 12.6);

double required_angular_vel = 0;
double required_linear_vel = 0;
double required_excur_vel = 0;

uint32_t previous_command_time = 0;

bool is_first = true;

void pid_callback( const mbs_msgs::PID& pid)
{
	robot.Update_PID(pid.p, pid.i, pid.d);
}

void command_callback( const geometry_msgs::Twist& cmd_msg)
{
	required_linear_vel = cmd_msg.linear.x;
	required_excur_vel	= cmd_msg.linear.y;
	required_angular_vel = cmd_msg.angular.z;

	previous_command_time = rt_tick_get();
}

void move_base(DC_Motor *LF_Wheel, DC_Motor *RF_Wheel, DC_Motor *LR_Wheel, DC_Motor *RR_Wheel, Kinematics *kinematics)
{
	Kinematics::output req_rpm;

	//get the required rpm for each motor based on required velocities
	req_rpm = kinematics->getRPM(required_linear_vel, required_excur_vel, required_angular_vel);
	//the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
	//the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
	robot.LF_Wheel_Spd = constrain(req_rpm.LF_Wheel, -MAX_RPM, MAX_RPM);
	robot.RF_Wheel_Spd = constrain(req_rpm.RF_Wheel, -MAX_RPM, MAX_RPM);
	robot.LR_Wheel_Spd = constrain(req_rpm.LR_Wheel, -MAX_RPM, MAX_RPM);
	robot.RR_Wheel_Spd = constrain(req_rpm.RR_Wheel, -MAX_RPM, MAX_RPM);
}

void stop_base()
{
	required_linear_vel = 0;
	required_excur_vel = 0;
	required_angular_vel = 0;
}

void publish_linear_velocity(DC_Motor *motor1, DC_Motor *motor2,  DC_Motor *motor3, 
							DC_Motor *motor4, Kinematics *kinematics, 
							mbs_msgs::Velocities *raw_vel_msg, 
							ros::Publisher *raw_vel_pub)
{
	//update the current speed of each motor based on encoder's count
	Kinematics::velocities vel;
	
	vel = kinematics->getVelocities(motor1->now_spd, motor2->now_spd, motor3->now_spd, motor4->now_spd);

	//fill in the object
	raw_vel_msg->linear_x = vel.linear_x;
	raw_vel_msg->linear_y = vel.linear_y;
	raw_vel_msg->angular_z = vel.angular_z;

	//publish raw_vel_msg object to ROS
	raw_vel_pub->publish(raw_vel_msg);
}

void publish_imu(Imu_Sensor *_imu, mbs_msgs::Imu *raw_imu_msg, ros::NodeHandle *nh, ros::Publisher *raw_imu_pub)
{
	_imu->measure_acceleration();
	raw_imu_msg->raw_linear_acceleration = _imu->raw_acceleration;
	
	_imu->measure_gyroscope();
	raw_imu_msg->raw_angular_velocity = _imu->raw_rotation;
	
	_imu->measure_magnetometer();
	raw_imu_msg->raw_magnetic_field = _imu->raw_magnetic_field;

	raw_imu_pub->publish(raw_imu_msg);
}

void print_debug(ros::NodeHandle *nh)
{
	char buffer[50];

	rt_sprintf(buffer, "Encoder Left Front: %d, Right Front: %d", LF_Wheel_Motor.read_odom(), RF_Wheel_Motor.read_odom());
	nh->loginfo(buffer);
	rt_sprintf(buffer, "Encoder Left Rear: %d, Right Rear: %d", LR_Wheel_Motor.read_odom(), RR_Wheel_Motor.read_odom());
	nh->loginfo(buffer);
	
	int bat_value = power.get_battery_notifier() * 100;
	rt_sprintf(buffer, "BAT: %d%.", bat_value);
	nh->loginfo(buffer);
	if(bat_value < 30)
	{
		rt_sprintf(buffer, "The battery is low, please charge it in time: %d%.", bat_value);
		nh->loginfo(buffer);
	}
}

ros::NodeHandle  nh;

mbs_msgs::Imu raw_imu_msg;
mbs_msgs::Velocities raw_vel_msg;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", command_callback);
ros::Subscriber<mbs_msgs::PID> pid_sub("pid", pid_callback);
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

static void rosserial_thread_entry(void *parameter)
{
	uint32_t previous_control_time = 0;
	uint32_t previous_debug_time = 0;
	uint32_t previous_imu_time = 0;
	uint32_t publish_vel_time = 0;
	uint32_t publish_data_scope = 0;
	while (!nh.connected())
	{
		rt_kprintf("Speed: Left Front: %d, Right Front: %d,  ", LF_Wheel_Motor.now_spd, RF_Wheel_Motor.now_spd);
		rt_kprintf("last: Left Front: %d, Right Front: %d\r\n", LF_Wheel_Motor.last_spd, RF_Wheel_Motor.last_spd);
		rt_kprintf("Speed: Left Rear: %d, Right Rear: %d,  ", LR_Wheel_Motor.now_spd, RR_Wheel_Motor.now_spd);
		rt_kprintf("last: Left Rear: %d, Right Rear: %d\r\n", LR_Wheel_Motor.last_spd, RR_Wheel_Motor.last_spd);
		rt_thread_delay(200);
	}
	rt_kprintf("Ros serial is connect! \r\n");
	
    while (1)
    {
		if ((rt_tick_get() - previous_control_time) >= (1000 / COMMAND_RATE))
		{
			
			move_base(&LF_Wheel_Motor, &LR_Wheel_Motor, &RF_Wheel_Motor, &RR_Wheel_Motor, &kinematics);
			previous_control_time = rt_tick_get();
		}

		if ((rt_tick_get() - previous_command_time) >= 400)
		{
			stop_base();
		}
		
		if ((rt_tick_get() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
		{
			publish_linear_velocity(&LF_Wheel_Motor, &LR_Wheel_Motor, &RF_Wheel_Motor, &RR_Wheel_Motor, &kinematics, &raw_vel_msg, &raw_vel_pub);
			publish_vel_time = rt_tick_get();
		}

		if ((rt_tick_get() - previous_imu_time) >= (1000 / IMU_PUBLISH_RATE))
		{
			//sanity check if the IMU exits
			//publish the IMU data
			
			publish_imu(&Imu_dev, &raw_imu_msg, &nh, &raw_imu_pub);
			previous_imu_time = rt_tick_get();
		}

		if(ROS_DEBUG)
		{
			if ((rt_tick_get() - previous_debug_time) >= (1000 / DEBUG_RATE)) 
			{
				print_debug(&nh);
				previous_debug_time = rt_tick_get();
			}
		}
		
		if ((rt_tick_get() - publish_data_scope) >= (1000 / 20))
		{
//			imu.imu_serial_scope();
			publish_data_scope = rt_tick_get();
		}
		rt_thread_delay(10);
    }
}

static void PID_control(void *parameter)
{
	robot.Increment_PID();	//PID控制器
}

static rt_timer_t PID_Timer;

int main(void)
{
    // 启动一个线程用来和 ROS 通信
	uint8_t led_sta;
    rt_thread_t thread = rt_thread_create("rosserial",     rosserial_thread_entry, RT_NULL, 2048, 8, 10);
    if(thread != RT_NULL)
    {
        rt_thread_startup(thread);
        rt_kprintf("[rosserial] New thread rosserial\n");
    }
    else
    {
        rt_kprintf("[rosserial] Failed to create thread rosserial\n");
    }
	//创建PID控制器
    PID_Timer = rt_timer_create("PID_Timer", PID_control,
                             RT_NULL, 10,
                             RT_TIMER_FLAG_PERIODIC);

    if (PID_Timer != RT_NULL) rt_timer_start(PID_Timer);
	
    //Init motors, specif>y the respective motor pins
	robot.Init();
	int stype = Imu_dev.init();

	power.init();
	
    //Init node>
    nh.initNode();
	
	nh.subscribe(pid_sub);
	nh.subscribe(cmd_sub);
	nh.advertise(raw_vel_pub);
	nh.advertise(raw_imu_pub);

	rt_pin_mode(LED,PIN_MODE_OUTPUT);
	
	while(1)
	{
		if(led_sta)
		{
			rt_pin_write(LED, 0);
			led_sta = 0;
		}
		else
		{
			led_sta = 1;
			rt_pin_write(LED, 1);
		}
		nh.spinOnce();
	}
}

static void rostopic(uint8_t argc, char **argv)
{
	switch(argc)
	{
		case 0:
			break;
		case 1:
		{
			rt_kprintf("rostopic type   print topic type\r\n");
			rt_kprintf("rostopic echo   print messages to screen. \r\n");
			rt_kprintf("rostopic list   print information about active topics. \r\n");
		}
		break;
		case 2:
		{
			if(!strcmp("-h", argv[1]) || !strcmp("help", argv[1]))
			{
				rt_kprintf("rostopic type   print topic type\r\n");
				rt_kprintf("rostopic echo   print messages to screen. \r\n");
				rt_kprintf("rostopic list   print information about active topics. \r\n");
			}
			if(!strcmp("list", argv[1]))
			{

			}
		}
		break;
		
		case 3:
		{
			if(!strcmp("echo", argv[1]))
			{
			}
			if(!strcmp("type", argv[1]))
			{
			}
		}
		
		default:
			rt_kprintf("Param is too many please enter again!\r\n");
		break;
	}
}
FINSH_FUNCTION_EXPORT_ALIAS(rostopic, __cmd_rostopic, rostopic node );

static void robot_ctrl(uint8_t argc, char **argv)
{
	switch(argc)
	{
		case 0:
			break;
		case 1:
		{
			rt_kprintf("1. [read]: Read system parameters. \r\n");
			rt_kprintf("2. [set]: Set robot parameters. \r\n");
			rt_kprintf("3. [ctrl]: Ctrl robot. \r\n");
		}
		break;
		case 2:
		{
			if(!strcmp("read", argv[1]))
			{
				rt_kprintf("[-p] Read pid parameters. \r\n");
				rt_kprintf("[-e] Read odometer. \r\n");
				rt_kprintf("[-s] Read speed. \r\n");
				rt_kprintf("[-b] Read battery charge. \r\n");
				rt_kprintf("[-i] Read imu. \r\n");
			}
			if(!strcmp("set", argv[1]))
			{
				rt_kprintf("[-p] Set pid kp parameters. \r\n");
				rt_kprintf("[-i] Set pid ki parameters. \r\n");
				rt_kprintf("[-d] Set pid kd parameters. \r\n");
			}
			if(!strcmp("ctrl", argv[1]))
			{
				rt_kprintf("[-r][spd] Used PID control robot running. \r\n");
				rt_kprintf("[-s] Ctrl robot stop. \r\n");
			}
		}
		break;
		
		case 3:
		{
			if(!strcmp("read", argv[1]))
			{
				if(!strcmp("-p", argv[2]))	
				{ 
					rt_fprintf("kp = ", LF_Wheel_Motor.kp);
					rt_fprintf("ki = ", LF_Wheel_Motor.ki);
					rt_fprintf("kd = ", LF_Wheel_Motor.kd);
				}
				if(!strcmp("-e", argv[2]))	
				{
					rt_kprintf("Encoder Left Front: %d, Right Front: %d\r\n", LF_Wheel_Motor.read_odom(), RF_Wheel_Motor.read_odom());
					rt_kprintf("Encoder Left Rear: %d, Right Rear: %d\r\n", LR_Wheel_Motor.read_odom(), RR_Wheel_Motor.read_odom());
				}
				if(!strcmp("-s", argv[2]))
				{
					rt_kprintf("Speed: Left Front: %d, Right Front: %d\r\n", LF_Wheel_Motor.now_spd, RF_Wheel_Motor.now_spd);
					rt_kprintf("Speed: Left Rear: %d, Right Rear: %d\r\n", LR_Wheel_Motor.now_spd, RR_Wheel_Motor.now_spd);
				}
				if(!strcmp("-b", argv[2]))	{}
				if(!strcmp("-i", argv[2]))	
				{
					Imu_dev.log_type = 3;
				}
				
			}
			if(!strcmp("ctrl", argv[1]))
			{
				if(!strcmp("-s", argv[2]))	{}
			}
		}
		break;
		
		case 4:
		{
			if(!strcmp("set", argv[1]))
			{
				if(!strcmp("-p", argv[2]))	{}
				if(!strcmp("-i", argv[2]))	{}
				if(!strcmp("-d", argv[2]))	{}
			}
			if(!strcmp("ctrl", argv[1]))
			{
				if(!strcmp("-r", argv[2]))
				{
					int SetSpd = atoi(argv[3]);
					robot.PID_RunSpd(SetSpd);
				}
			}
		}break;
		
		default:
			rt_kprintf("Param is too many please enter again!\r\n");
		break;
	}
}
FINSH_FUNCTION_EXPORT_ALIAS(robot_ctrl, __cmd_robot_ctrl, robot ctrl!);


