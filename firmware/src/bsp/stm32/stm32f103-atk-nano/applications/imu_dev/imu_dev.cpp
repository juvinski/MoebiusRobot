#include "imu_dev.h"
#include "config.h"
#include "datascope.h"

#if (IMU_DEV_MPU6050 == IMU_TYPE)
	static struct mpu6xxx_device *dev;
#elif (IMU_DEV_BMX160 == IMU_TYPE)
	#define BMI160_ACCEL_DEV	"acce_bmi"
	#define BMI160_GYRO_DEV		"gyro_bmi"
	rt_device_t bmi160_accel, bmi160_gyro;
#elif (IMU_DEV_GY85 == IMU_TYPE)

#endif

//打印陀螺仪数据
static void imu_print(geometry_msgs::Vector3 accel, geometry_msgs::Vector3 gyro)
{
	int xh, xl, yh, yl, zh, zl;
	int x = accel.x * 100.0, y = accel.y * 100.0, z = accel.z * 100.0;
	
	xh = x / 100, xl = abs(x) % 100;
	yh = y / 100, yl = abs(y) % 100;
	zh = z / 100, zl = abs(z) % 100;
	
	rt_kprintf("accel: x:%d.%02d, y:%d.%02d, z:%d.%02d\n", xh, xl, yh, yl, zh, zl);
	
	x = gyro.x * 100.0, y = gyro.y * 100.0, z = gyro.z * 100.0;
	
	xh = x / 100, xl = abs(x) % 100;
	yh = y / 100, yl = abs(y) % 100;
	zh = z / 100, zl = abs(z) % 100;
	rt_kprintf("gyro: x:%d.%02d, y:%d.%02d, z:%d.%02d\n", xh, xl, yh, yl, zh, zl);
}

//发送陀螺仪数据到串口示波器
static void push_serial_scope(geometry_msgs::Vector3 accel, geometry_msgs::Vector3 gyro)
{
	extern void rt_kputs(const char *str);
	DataScope_Get_Channel_Data(accel.x , 1);
	DataScope_Get_Channel_Data(accel.y , 2);
	DataScope_Get_Channel_Data(accel.z , 3); 
	DataScope_Get_Channel_Data(gyro.x , 4);
	DataScope_Get_Channel_Data(gyro.y , 5);
	DataScope_Get_Channel_Data(gyro.z , 6);

	DataScope_Data_Generate(6);
	rt_kputs((const char *)&DataScope_OutPut_Buffer[0]);
}

//陀螺仪静态校准
void imu_static_compensation(geometry_msgs::Vector3 &static_gyro)
{
	double compensation_data_buf[3]={0};
	struct mpu6xxx_3axes gyro;
	int count;
	for(count = 0; count < 100; count ++)
	{
#if (IMU_DEV_MPU6050 == IMU_TYPE)
	mpu6xxx_get_gyro(dev, &gyro);
#elif (IMU_DEV_BMX160 == IMU_TYPE)

#elif (IMU_DEV_GY85 == IMU_TYPE)
		
#endif
		rt_thread_delay(10);

		compensation_data_buf[0] += gyro.x;
		compensation_data_buf[1] += gyro.y;
		compensation_data_buf[2] += gyro.z;
		
	}
	static_gyro.x = compensation_data_buf[0] / 100;
	static_gyro.y = compensation_data_buf[1] / 100;
	static_gyro.z = compensation_data_buf[2] / 100;
	rt_kprintf("Calibrate the gyroscope success ! static data :\r\n");
	imu_print(static_gyro, static_gyro);
}

int Imu_Sensor::init()
{

#if (IMU_DEV_MPU6050 == IMU_TYPE)
    dev = mpu6xxx_init(IMU_DEVICE_NAME, RT_NULL);
    if (dev == RT_NULL)
    {
        rt_kprintf("mpu6xxx init failed\n");
		return RT_ERROR;
    }
    rt_kprintf("mpu6xxx init succeed\n");
	return RT_EOK;
#elif (IMU_DEV_BMX160 == IMU_TYPE)
    struct rt_sensor_config cfg;
	rt_kprintf("Imu type : BMI160...\r\n");
    cfg.intf.dev_name = (char *)IMU_DEVICE_NAME;
    cfg.intf.user_data = (void *)0x69;
    if(!rt_hw_bmx160_init("bmi160", &cfg))
		rt_kprintf("Find bmx160 success!\r\n");
	else
	{
		rt_kprintf("sensor find failed!\r\n");
		return RT_ERROR;
	}
	bmi160_accel = rt_device_find(BMI160_ACCEL_DEV);
	bmi160_gyro = rt_device_find(BMI160_GYRO_DEV);

	if(bmi160_accel == RT_NULL || bmi160_gyro == RT_NULL)
	{
		rt_kprintf("No find bmi160 dev!\r\n");
		return RT_ERROR;
	}
	else
		rt_kprintf("Find sensor success!\r\n");
	rt_device_open(bmi160_accel, RT_DEVICE_FLAG_RDONLY);
	rt_device_open(bmi160_gyro, RT_DEVICE_FLAG_RDONLY);
	rt_kprintf("MPU Done...\r\n");
	return RT_EOK;
#elif (IMU_DEV_GY85 == IMU_TYPE)
	rt_kprintf("Imu type : Gy85...\r\n");
#endif

}

void Imu_Sensor::measure_acceleration()
{
#if (IMU_DEV_MPU6050 == IMU_TYPE)
	struct mpu6xxx_3axes accel;
	mpu6xxx_get_accel(dev, &accel);
	raw_acceleration.x = (float)((accel.x + static_acc.x) / 360.0);
	raw_acceleration.y = (float)((accel.y + static_acc.y) / 360.0);
	raw_acceleration.z = (float)((accel.z + static_acc.z) / 360.0);
#elif (IMU_DEV_BMX160 == IMU_TYPE)
	struct rt_sensor_data data;
    if (rt_device_read(bmi160_accel, 0, &data, 1) == 1)
    {
		raw_acceleration.x = (float)((data.data.acce.x + static_acc.x) / 360.0);
		raw_acceleration.y = (float)((data.data.acce.y + static_acc.y) / 360.0);
		raw_acceleration.z = (float)((data.data.acce.z + static_acc.z) / 360.0);
    }
#elif (IMU_DEV_GY85 == IMU_TYPE)

#endif
}

void Imu_Sensor::measure_gyroscope()
{
#if (IMU_DEV_MPU6050 == IMU_TYPE)
	struct mpu6xxx_3axes gyro;
	mpu6xxx_get_gyro(dev, &gyro);
	raw_rotation.x = (float)((gyro.x + static_gyro.x) / 360.0);
	raw_rotation.y = (float)((gyro.y + static_gyro.y) / 360.0);
	raw_rotation.z = (float)((gyro.z + static_gyro.z) / 360.0);
#elif (IMU_DEV_BMX160 == IMU_TYPE)
	struct rt_sensor_data data;
    rt_device_read(bmi160_gyro, 0, &data, 1);
	raw_rotation.x = (float)((data.data.gyro.x + static_gyro.x) / 360.0);
	raw_rotation.y = (float)((data.data.gyro.y + static_gyro.y) / 360.0);
	raw_rotation.z = (float)((data.data.gyro.z + static_gyro.z) / 360.0);
#elif (IMU_DEV_GY85 == IMU_TYPE)

#endif
	if(log_type == 1)
	{
		imu_print(raw_acceleration, raw_rotation);
	}
	else if(log_type == 2)
	{
		push_serial_scope(raw_acceleration, raw_rotation);
	}
	else if(log_type == 3)
	{
		log_type = 0;
		imu_print(raw_acceleration, raw_rotation);
	}
}

void Imu_Sensor::measure_magnetometer()
{
#if (IMU_DEV_MPU6050 == IMU_TYPE)
	raw_magnetic_field.x = (float)((360 + static_magn.x) / 360.0);
	raw_magnetic_field.y = (float)((360 + static_magn.y) / 360.0);
	raw_magnetic_field.z = (float)((360 + static_magn.z) / 360.0);
#elif (IMU_DEV_BMX160 == IMU_TYPE)
	raw_magnetic_field.x = (float)((360 + static_magn.x) / 360.0);
	raw_magnetic_field.y = (float)((360 + static_magn.y) / 360.0);
	raw_magnetic_field.z = (float)((360 + static_magn.z) / 360.0);
#elif (IMU_DEV_GY85 == IMU_TYPE)

#endif
}

