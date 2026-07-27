#include "mpu6050.h"

/**
 * @brief  初始化 MPU6050
 * @param  hi2c: 指向 I2C 句柄的指针 (例如 &hi2c1)
 * @retval 0: 失败, 1: 成功
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check = 0;  // 必须初始化为 0，防止栈垃圾碰巧等于 0x68
    uint8_t data;

    // 1. 读取 WHO_AM_I 寄存器，检测器件是否存在
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 1000) != HAL_OK) {
        return 0; // I2C 通信失败
    }

    if (check == 0x68) { // 0x68 是 MPU6050 的默认器件 ID

        // 2. 唤醒传感器：PWR_MGMT_1 写入 0x00 清除 SLEEP 位
        data = 0x00;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 1000) != HAL_OK) {
            return 0;
        }
        HAL_Delay(10); // 等待内部时钟起振稳定

        // 3. 确保所有轴使能：PWR_MGMT_2 写入 0x00（全部退出待机模式）
        data = 0x00;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_2, 1, &data, 1, 1000) != HAL_OK) {
            return 0;
        }

        // 4. 设置采样率：SMPLRT_DIV 写入 0x00 (1kHz / (1+0) = 1000Hz 采样率)
        data = 0x00;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPLRT_DIV, 1, &data, 1, 1000) != HAL_OK) {
            return 0;
        }

        // 5. 配置加速度计范围：ACCEL_CONFIG 写入 0x00 (选择 ±2g 满量程)
        data = 0x00;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, 1000) != HAL_OK) {
            return 0;
        }

        // 6. 配置陀螺仪范围：GYRO_CONFIG 写入 0x00 (选择 ±250 °/s 满量程)
        data = 0x00;
        if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 1000) != HAL_OK) {
            return 0;
        }

        return 1; // 初始化成功
    }

    return 0; // 器件 ID 不匹配
}

/**
 * @brief  读取加速度、陀螺仪和温度的所有原始数据并转换为物理量
 * @param  hi2c: 指向 I2C 句柄的指针
 * @param  DataStruct: 指向存储读取数据的 MPU6050_t 结构体指针
 */
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[14];

    // 从 ACCEL_XOUT_H (0x3B) 开始，连续读取 14 字节
    // 布局: [AXH AXL AYH AYL AZH AZL TH TL GXH GXL GYH GYL GZH GZL]
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, Rec_Data, 14, 1000) != HAL_OK) {
        return; // I2C 读取失败，直接返回（保留上一次的数据）
    }

    /* 1. 解析原始 ADC 数据 (高字节在前，低字节在后) */
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Temp_RAW  = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    /* 2. 将原始值转换为物理量 */

    // 加速度计灵敏度：±2g 范围时为 16384 LSB/g
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0f;

    // 温度转换公式：Temp = (Raw / 340) + 36.53 (单位 ℃)
    DataStruct->Temp = (float)(DataStruct->Temp_RAW / 340.0f + 36.53f);

    // 陀螺仪灵敏度：±250 °/s 范围时为 131.0 LSB/(°/s)
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0f;
}
