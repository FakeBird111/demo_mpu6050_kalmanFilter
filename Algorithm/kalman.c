#include "kalman.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513f // 180 / PI

void Kalman_Init(Kalman_t *kf, float Q_angle, float Q_bias, float R_measure){
	// 配置噪声协方差
	kf->Q_angle   = Q_angle;
	kf->Q_bias    = Q_bias;
	kf->R_measure = R_measure;

	// 先验、后验估计状态量，赋予初值
	kf->angle_prior = 0.0f;
	kf->angle_post  = 0.0f;
	kf->bias_prior  = 0.0f;
	kf->bias_post   = 0.0f;
	kf->rate        = 0.0f;

	// 误差协方差矩阵，赋予初值
	kf->P_prior[0][0] = 0.0f;
	kf->P_prior[0][1] = 0.0f;
	kf->P_prior[1][0] = 0.0f;
	kf->P_prior[1][1] = 0.0f;

	kf->P_post[0][0] = 0.0f;
	kf->P_post[0][1] = 0.0f;
	kf->P_post[1][0] = 0.0f;
	kf->P_post[1][1] = 0.0f;
}

float Kalman_GetRoll(float ax, float ay, float az)
{
    (void)ax; // Roll 轴解算不依赖 ax，此行消除编译器 warning
    return atan2f(ay, az) * RAD_TO_DEG;
}

float Kalman_GetPitch(float ax, float ay, float az)
{
    return atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

float Kalman_Update(Kalman_t *kf, float newAngle, float newRate, float dt)
{
    if (kf == 0) return 0.0f;

    /* 1.时间更新 (Predict / Time Update) */
	
    // 1.1.计算去偏后的角速度 rate
    kf->rate = newRate - kf->bias_post;

    // 1.2.计算先验估计角度 angle_prior 和 零偏 bias_prior
    kf->angle_prior = kf->angle_post + kf->rate * dt;
    kf->bias_prior  = kf->bias_post;

    // 1.3.更新先验误差协方差矩阵 P_prior = F * P_post * F^T + Q
    kf->P_prior[0][0] = kf->P_post[0][0] - dt * (kf->P_post[1][0] + kf->P_post[0][1]) + dt * dt * kf->P_post[1][1] + kf->Q_angle;
    kf->P_prior[0][1] = kf->P_post[0][1] - dt * kf->P_post[1][1];
    kf->P_prior[1][0] = kf->P_post[1][0] - dt * kf->P_post[1][1];
    kf->P_prior[1][1] = kf->P_post[1][1] + kf->Q_bias;

    /* 2.量测更新 (Update / Measurement Update) */
	
    // 2.1.计算测量残差 y (Measurement Innovation)
    float y = newAngle - kf->angle_prior;

    // 2.2.计算残差总不确定度 S (Innovation Covariance)
    float S = kf->P_prior[0][0] + kf->R_measure;

    // 2.3.计算卡尔曼增益 K (Kalman Gain)
    float K[2];
    K[0] = kf->P_prior[0][0] / S; // 用于修正角度
    K[1] = kf->P_prior[1][0] / S; // 用于修正零偏

    // 2.4.计算后验状态估计 (更新真实的 angle_post 和 bias_post)
    kf->angle_post = kf->angle_prior + K[0] * y;
    kf->bias_post  = kf->bias_prior  + K[1] * y;

    // 2.5.更新后验误差协方差矩阵 P_post = (I - K * H) * P_prior
    kf->P_post[0][0] = kf->P_prior[0][0] - K[0] * kf->P_prior[0][0];
    kf->P_post[0][1] = kf->P_prior[0][1] - K[0] * kf->P_prior[0][1];
    kf->P_post[1][0] = kf->P_prior[1][0] - K[1] * kf->P_prior[0][0];
    kf->P_post[1][1] = kf->P_prior[1][1] - K[1] * kf->P_prior[0][1];

    /* 返回解算后的最优后验角度 */
    return kf->angle_post;
}
