#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256
#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_SMALL_MEM
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart2"
#define RT_VER_NUM 0x40002
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_M3

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 2048
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */

#define RT_USING_CPLUSPLUS

/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_USING_MSH_ONLY
#define FINSH_ARG_MAX 10

/* Device virtual file system */


/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SERIAL
#define RT_SERIAL_USING_DMA
#define RT_SERIAL_RB_BUFSZ 1024
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN
#define RT_USING_ADC
#define RT_USING_PWM
#define RT_USING_SENSOR
#define RT_USING_SENSOR_CMD
#define RT_USING_PULSE_ENCODER

/* Using USB */


/* POSIX layer and C standard library */

#define RT_LIBC_USING_TIME

/* Network */

/* Socket abstraction layer */


/* Network interface device */


/* light weight TCP/IP stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */


/* RT-Thread online packages */
#define PKG_USING_SENSORS_DRIVERS
#define PKG_USING_BMI160_BMX160
#define BMI160_USING_ACCE
#define BMI160_USING_GYRO

#define PKG_USING_BMI160_BMX160_LATEST_VERSION

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */


/* multimedia packages */


/* tools packages */


/* system packages */


/* peripheral libraries and drivers */

#define PKG_USING_SENSORS_DRIVERS
#define PKG_USING_MPU6XXX
#define PKG_USING_MPU6XXX_LATEST_VERSION
#define PKG_USING_MPU6XXX_ACCE
#define PKG_USING_MPU6XXX_GYRO
#define PKG_USING_MPU6XXX_MAG

#define PKG_USING_ROSSERIAL
#define ROSSERIAL_USE_SERIAL
#define ROSSERIAL_UART_NAME "uart1"
#define ULOG_USING_ISR_LOG
/* roserial examples */


/* You may only choose one example at a time */

#define PKG_USING_ROSSERIAL_MLATEST_VERSION
#define PKG_ROSSERIAL_VER_NUM 0x29999

/* miscellaneous packages */


/* samples: kernel and components samples */

#define SOC_FAMILY_STM32
#define SOC_SERIES_STM32F1

/* Hardware Drivers Config */

#define SOC_STM32F103RC

/* Onboard Peripheral Drivers */

#define BSP_USING_USB_TO_USART

/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_UART
#define BSP_USING_UART1
#define BSP_USING_UART2
#define BSP_USING_I2C1
#define BSP_USING_I2C2
#define BSP_USING_PWM
#define BSP_USING_PWM8
#define BSP_USING_PWM8_CH1
#define BSP_USING_PWM8_CH2
#define BSP_USING_PWM8_CH3
#define BSP_USING_PWM8_CH4

#define BSP_I2C1_SCL_PIN	24
#define BSP_I2C1_SDA_PIN	25

#define BSP_I2C2_SCL_PIN	26
#define BSP_I2C2_SDA_PIN	27
#define BSP_USING_PULSE_ENCODER2
#define BSP_USING_PULSE_ENCODER3
#define BSP_USING_PULSE_ENCODER4
#define BSP_USING_PULSE_ENCODER5
#define BSP_USING_ADC
#define BSP_USING_ADC1

/* Board extended module Drivers */


#endif
