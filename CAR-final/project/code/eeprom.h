/*********************************************************************************************************************
* CYT4BB7_CAR1 — EEPROM（Work Flash）参数存储模块
* 文件: eeprom.h
* 说明: 基于 Flash 的参数持久化存储，包括 PID、速度、坐标和信标数据
********************************************************************************************************************/

#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "zf_common_headfile.h"

//==================================================== 参数存储位置枚举 ====================================================
// 每个参数对应 Flash 页面中的一个槽位偏移
// 支持多组参数通过 speed_para_set 切换，每组占用 para_num 个位置
//==================================================== 参数存储位置枚举 ====================================================
typedef enum
{
    // 电机L PID 参数
    motor_l_p, motor_l_i, motor_l_d,
    // 电机R PID 参数
    motor_r_p, motor_r_i, motor_r_d,
    // 舵机 PID 参数
    dir_p, dir_i, dir_d,
    // 目标速度 PID 参数
    target_speed_p, target_speed_i, target_speed_d,
    // 其他参数
    para_target_distance_coef,          // 目标距离系数
    para_dynamic_p_coef,                // 动态P系数
    para_target_speed_offset_coef,      // 目标速度偏移系数
    para_point_distance,                // 目标点距离（uint8 类型）

    // 角度环 PID
    angle_p, angle_i, angle_d,
    // 速度环 PID
    speed_p, speed_i, speed_d,
    // 位置环 PID
    pos_p, pos_i, pos_d,
    // 基础速度 / 冲刺速度
    base_speed,
    sprint_speed,
    ins_count,                          // 坐标数量
    pixel_charge_threshold,             // 像素冲刺阈值（uint32，通过 EEPROM_Write_Data_uint32 存取）
    beacon_count,                       // 信标个数（uint8）
    target_point_num,                   // 序列长度（uint8）

    para_num                            // 每组参数的个数（用于计算偏移）
} parameter;


//==================================================== Flash 存储地址定义 ====================================================
// Work Flash: 96 页 × 2KB = 192KB（页号 0 ~ 95）
#define FLASH_SECTION_INDEX       (0)     // 扇区编号（CYT4BB 上未实际使用，仅用于保持接口兼容）
#define FLASH_PAGE_INDEX          (11)    // PID 参数存储页
#define COORD_PAGE_INDEX          (10)    // 惯导坐标存储页（独立页面，避免与 PID 参数冲突）
#define COORD_MAX_COUNT           (30)    // 最多存储的坐标点数量（每点 2 个 float = 8 字节）
#define BEACON_DATA_PAGE_INDEX    (9)     // 信标数据存储页（信标序列 + 信标位置）


//==================================================== 惯导坐标存储结构体 ====================================================
typedef struct
{
    float x;    // X 坐标
    float y;    // Y 坐标
} coord_point_t;


//==================================================== 函数声明 ====================================================
void    Flash_init(void);                                            // Flash 初始化
void    EEPROM_Update_All_Para(void);                                 // 从 Flash 更新所有参数到全局变量
float   EEPROM_Read_Data(parameter para);                             // 从 Flash 读取单个 float 参数
void    EEPROM_Write_Data(parameter para, float data);                // 写入单个 float 参数到 Flash
void    EEPROM_Write_Data_uint16(parameter para, uint16 data);        // 写入单个 uint16 参数到 Flash
void    EEPROM_Write_Data_uint32(parameter para, uint32 data);        // 写入单个 uint32 参数到 Flash
void    Para_Init(void);                                              // 参数初始化（写入默认值到 Flash，仅首次上电调用）

// 惯导坐标批量存储（使用独立 Flash 页，直接批量读写）
void    coord_read_all(coord_point_t *dst, uint32 count);            // 从 Flash 批量读取坐标点
void    coord_write_all(const coord_point_t *src, uint32 count);     // 批量写入坐标点到 Flash（先擦除整页再写入）
void    coord_erase_all(void);                                       // 擦除坐标存储页

// 信标数据批量存储（使用独立 Flash 页 BEACON_DATA_PAGE_INDEX）
void    beacon_seq_read_all(uint8 *dst, uint32 count);               // 从 Flash 批量读取信标序列（uint8 数组）
void    beacon_seq_write_all(const uint8 *src, uint32 count);        // 批量写入信标序列到 Flash
void    beacon_pos_read_all(float (*dst)[2], uint32 count);          // 从 Flash 批量读取信标位置（float[count][2] 数组）
void    beacon_pos_write_all(const float (*src)[2], uint32 count);   // 批量写入信标位置到 Flash
void    beacon_data_erase_all(void);                                 // 擦除信标数据存储页

//==================================================== 外部变量引用 ====================================================
extern uint8    para_update_flag;            // 参数更新标志
extern uint8    para_update_display_flag;    // 参数更新显示标志
extern uint8    speed_para_set;              // 当前参数组编号（支持多组参数存储）
extern float    test;

// PID 批量读写
void    EEPROM_Save_PID(parameter p_enum, float kp, float ki, float kd);   // 批量保存 PID 组到 Flash
void    EEPROM_Load_PID(parameter p_enum, float *kp, float *ki, float *kd); // 批量从 Flash 读取 PID 组

#endif
