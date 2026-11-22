#ifndef CODE_PROCESS_RX_H_
#define CODE_PROCESS_RX_H_

#include "zf_common_headfile.h"

#define DEG_TO_RAD_R 0.017453292519943295769236907684886   // 度转弧度系数
#define RAD_TO_DEG_R 57.295779513082320876798154814105     // 弧度转度系数

typedef struct {
    float pitch;    // 俯仰角 (度)
    float roll;     // 横滚角 (度)
    float yaw;      // 偏航角 (本滤波器不更新，仅传递输入值)
} Attitude_2D_t;


// 2状态卡尔曼滤波器 (仅 pitch, roll)
typedef struct {
    float dt;                   // 采样周期 (秒)
    float Q_pitch;              // pitch 过程噪声
    float Q_roll;               // roll 过程噪声
    float R_pitch;              // pitch 测量噪声
    float R_roll;               // roll 测量噪声

    // 状态
    float pitch;
    float roll;

    // 协方差矩阵 (2x2)
    float P00;  // pitch 方差
    float P01;  // pitch-roll 协方差
    float P11;  // roll 方差
} Kalman2D;


//滤波后加速度和角速度
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t_rx;

//四元数结构体
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t_rx;

//欧拉角结构体
typedef struct {
    float pitch;
    float roll;
    float yaw;
    float last_yaw;
    float last_roll;
    uint8 Dirchange;
} euler_param_t_rx;

extern float acc_data[3];
extern float gyro_data[3];
extern quater_param_t_rx Q_info;
extern euler_param_t_rx eulerAngle;
extern icm_param_t_rx icm_data;

void imu_update_acc(float acc_x, float acc_y, float acc_z,float gyro_x, float gyro_y, float gyro_z);
void imu_GyroOffset(void);
void imu_rx_init(void);
void ICM_getEulerianAngles(void);

#endif /* CODE_PROCESS_RX_H_ */
