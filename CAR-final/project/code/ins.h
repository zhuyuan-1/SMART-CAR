#ifndef CODE_INS_H_
#define CODE_INS_H_

#include "zf_common_headfile.h"

/********************************************************************************************************************
 * 卡尔曼滤波结构体定义
 ********************************************************************************************************************/
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

//全局变量
extern float pos_x_encoder;          // 编码器解算X坐标
extern float pos_y_encoder;          // 编码器解算Y坐标
extern float pos_x_final;            // 最终融合X坐标
extern float pos_y_final;            // 最终融合Y坐标

//惯导初始化
void navigation_init(void);
//惯导位置解算（数据获取中断中）
void navigation(float DT); //X前后 Y左右，DT为s

#endif

