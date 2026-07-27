#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

// MPU6050 默认 I2C 地址 (ADO 接 GND 时 7-bit 地址为 0x68，STM32 HAL 要求左移一位 = 0xD0)
#define MPU6050_ADDR 0xD0

// MPU6050 内部寄存器地址
#define MPU6050_SMPLRT_DIV   0x19  // 采样率分频器
#define MPU6050_CONFIG       0x1A  // 配置寄存器（DLPF滤波）
#define MPU6050_GYRO_CONFIG  0x1B  // 陀螺仪配置
#define MPU6050_ACCEL_CONFIG 0x1C  // 加速度计配置
#define MPU6050_ACCEL_XOUT_H 0x3B  // 加速度计 X 轴高字节（数据起始地址）
#define MPU6050_TEMP_OUT_H   0x41  // 温度高字节
#define MPU6050_GYRO_XOUT_H  0x43  // 陀螺仪 X 轴高字节
#define MPU6050_PWR_MGMT_1   0x6B  // 电源管理 1
#define MPU6050_PWR_MGMT_2   0x6C  // 电源管理 2
#define MPU6050_WHO_AM_I     0x75  // 器件 ID 寄存器

// 用于存储解析后数据的结构体
typedef struct {
    // 原始 ADC 数据
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
		
		int16_t Temp_RAW;
	
    // 转换后的实际物理数值
    float Ax;          // 加速度 g
    float Ay;
    float Az;
    
    float Gx;          // 角速度 °/s
    float Gy;
    float Gz;

    float Temp; // 温度 ℃
} MPU6050_t;

// 函数声明
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct);


#endif
