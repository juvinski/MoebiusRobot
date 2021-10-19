#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float base_width, int pwm_bits)
{
  wheel_diameter_ = wheel_diameter;
  circumference_ = PI * wheel_diameter_;
  max_rpm_ = motor_max_rpm;
  base_width_ = base_width;
  pwm_res_ = pow(2.0, pwm_bits) - 1;
}

Kinematics::output Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  //convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * base_width_;

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;

  //calculate for the target motor RPM and direction
  //left front motor
  rpm.LF_Wheel = x_rpm_ - y_rpm_ + tan_rpm_;
  //left rear motor
  rpm.LR_Wheel = x_rpm_ + y_rpm_ + tan_rpm_;
  //right front motor
  rpm.RF_Wheel = x_rpm_ + y_rpm_ - tan_rpm_;
  //right rear motor
  rpm.RR_Wheel = x_rpm_ - y_rpm_ - tan_rpm_;

  return rpm;
}

Kinematics::output Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
	Kinematics::output rpm;
	Kinematics::output pwm;

	rpm = getRPM(linear_x, linear_y, angular_z);

	//convert from RPM to PWM

	//left front motor
	pwm.LF_Wheel = rpmToPWM(rpm.LF_Wheel);
	//left rear motor
	pwm.LR_Wheel = rpmToPWM(rpm.LR_Wheel);
	//right front motor
	pwm.RF_Wheel = rpmToPWM(rpm.RF_Wheel);
	//right rear motor
	pwm.RR_Wheel = rpmToPWM(rpm.RR_Wheel);
	
	return pwm;
}

Kinematics::velocities Kinematics::getVelocities(int LF_Wheel, int LR_Wheel, int RF_Wheel, int RR_Wheel)
{
  Kinematics::velocities vel;

  double average_rpm_x = (LF_Wheel + LR_Wheel + RF_Wheel + RR_Wheel) / 4;
  //convert revolutions per minute to revolutions per second
  double average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * (wheel_diameter_ * PI)); // m/s

  double average_rpm_y = (-LF_Wheel + LR_Wheel + RF_Wheel - RR_Wheel) / 4;
  //convert revolutions per minute in y axis to revolutions per second
  double average_rps_y = average_rpm_y / 60; // RPS
  vel.linear_y = (average_rps_y * (wheel_diameter_ * PI)); // m/s

  double average_rpm_a = (-LF_Wheel + LR_Wheel - RF_Wheel + RR_Wheel) / 4;
  //convert revolutions per minute to revolutions per second
  double average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * (wheel_diameter_ * PI)) / base_width_;

  return vel;
}

int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
 return (((double) rpm / (double) max_rpm_) * 255);
}
