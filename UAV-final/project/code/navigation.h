#ifndef CODE_NAVIGATION_H_
#define CODE_NAVIGATION_H_

#include "zf_common_headfile.h"

// 单轴卡尔曼滤波结构体（基础单元）
typedef struct {
    float x[2];          // 状态量: [0]速度, [1]加速度偏置
    float P[2][2];       // 协方差矩阵
    float Q[2][2];       // 过程噪声矩阵
    float R;             // 观测噪声
    float dt;            // 采样时间 (s)
} KF_Axis_t;

// 三轴卡尔曼滤波总结构体
typedef struct {
    KF_Axis_t x;         // X轴滤波
    KF_Axis_t y;         // Y轴滤波
    KF_Axis_t z;         // Z轴滤波
} KF_3Axis_Velocity_t;


extern float acc_n[3];
extern float acc_n_filtered[3];
extern float vel_imu[3];
extern float acc_n_b[3];
extern int nag_x_sum;
extern int nag_y_sum;

void acc_init(void);
void transform_accel_to_nav(void);
void calculate_imu_velocity(float dt);
void reset_imu_velocity(void);
void KF_3Axis_Init(KF_3Axis_Velocity_t *kf, float dt);
void KF_3Axis_Update(KF_3Axis_Velocity_t *kf, float imu_acc[3], float last_vel[3], float out_vel[3]);
void Nag_init(void);
void Nag_Read(uint8 image_x,uint8 image_y);
#endif /* CODE_NAVIGATION_H_ */
