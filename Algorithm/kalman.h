#ifndef __KALMAN_H
#define __KALMAN_H

#include <stdint.h>

/**
 * @brief 2 维卡尔曼滤波器输入变量
 * 融合陀螺仪（角速度）与加速度计（倾斜角），估计真实角度并补偿陀螺仪零偏。
 * 状态向量: [角度(°), 陀螺仪零偏(°/s)]
 * 单位规定:
 *   - 角度:     度 (°)
 *   - 角速度:   度/秒 (°/s)
 *   - 加速度:   g
 *   - 时间步长: 秒 (s)
 * 变量符号规定:
 *   - _prior:   表示先验估计值 
 *   - _post:    表示后验估计值
 */
typedef struct {
	float Q_angle;// 角度过程噪声协方差   (推荐初始值: 0.001)
	float Q_bias;// 零偏过程噪声协方差   (推荐初始值: 0.003)
	float R_measure;// 加速度计测量噪声协方差 (推荐初始值: 0.03)

	float angle_prior;// 先验估计角度 (°)
	float angle_post;// 后验估计角度，用于输出 (°)
	float bias_prior;// 先验估计陀螺仪零偏 (°/s)
	float bias_post;// 后验估计陀螺仪零偏，用于输出 (°/s)
	
	float rate;// 去偏后的角速度 (°/s)

	float P_prior[2][2];// 先验估计误差协方差矩阵
	float P_post[2][2];// 后验估计误差协方差矩阵
} Kalman_t;

/**
 * @brief  初始化卡尔曼滤波器
 * @param  kf:        滤波器句柄
 * @param  Q_angle:   角度过程噪声 — 越大越信任陀螺仪（估计），越小越信任加速度计（量测）
 * @param  Q_bias:    零偏过程噪声 — 越大零偏收敛越快，但稳态波动也越大
 * @param  R_measure: 加速度计量测噪声 — 越大越不相信加速度计
 */
void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure);

/**
 * @brief  由加速度计 g 值计算 Roll  角（绕 X 轴旋转）
 * @param  ax/ay/az: 加速度计三轴读数 (g)
 * @return Roll (°), 范围 [-180, 180]
 */
float Kalman_GetRoll(float ax, float ay, float az);

/**
 * @brief  由加速度计 g 值计算 Pitch 角（绕 Y 轴旋转）
 * @param  ax/ay/az: 加速度计三轴读数 (g)
 * @return Pitch (°), 范围 [-90, 90]
 */
float Kalman_GetPitch(float ax, float ay, float az);

/**
 * @brief  卡尔曼滤波器更新（单轴，每次读到传感器数据调用一次）
 * @param  kf:       滤波器句柄
 * @param  newAngle: 由加速度计算出的瞬时角度 (°)，例如 Kalman_GetRoll() / Kalman_GetPitch()
 * @param  newRate:  陀螺仪角速度 (°/s)，如 MPU6050 的 Gx / Gy / Gz
 * @param  dt:       距离上次调用的时间 (秒)，例如 125Hz 采样 → dt = 0.008
 * @return 滤波后的角度 (°)
 *
 * 调用频率: 与 MPU6050 采样率相同（核心在 main 循环中每次读取传感器后调用）
 * dt: 必须准确，否则滤波发散
 */
float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt);


#endif
