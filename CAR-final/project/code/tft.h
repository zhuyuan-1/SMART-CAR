/*********************************************************************************************************************
* CYT4BB7_CAR1 — TFT 屏幕显示与按键输入模块
* 文件: tft.h
* 说明: 1.8寸 TFT 显示屏 + 3 按键菜单系统
*       KEY1=下/减  KEY2=上/加  KEY3=确认/选择
********************************************************************************************************************/

#ifndef __TFT_H__
#define __TFT_H__

#include "zf_common_headfile.h"
#include "eeprom.h"
#include "ipc.h"

// 按键引脚定义
#define KEY1                    (P20_0)     // 下 / 减
#define KEY2                    (P20_1)     // 上 / 加
#define KEY3                    (P20_2)     // 确认 / 选择

// 外部变量声明
extern float now_start;                                                  // 开始运行标志位
extern float V_a;                                                        // 速度赋值
extern float voltage_power_supply_tft;                                   // 电源电压
extern float stop_m2;                                                    // 停止标志位
extern float serial_data[4];                                             // 串口/UART 输入数据（预留）

extern float angle_pid[3];                                               // 角度环 Kp/Ki/Kd
extern float speed_pid[3];                                               // 速度环 Kp/Ki/Kd
extern float pos_pid[3];                                                 // 位置环 Kp/Ki/Kd
extern float base_speed_val;                                             // 基础速度值
extern float sprint_speed_val;                                           // 冲刺速度值
extern uint32 ins_coord_count;                                           // 惯导坐标数量
extern coord_point_t coord_buffer[COORD_MAX_COUNT];                      // 惯导坐标缓冲区

// 新增：像素冲刺阈值 + 信标数据 RAM 缓冲区（可在屏幕上修改）
extern uint32 pixel_threshold_val;                                       // 像素冲刺阈值
extern uint8  beacon_seq_ram[40];                                        // 信标序列 RAM 副本（40 个 uint8）
extern float  beacon_pos_ram[11][2];                                     // 信标位置 RAM 副本（11 个位置 × x,y）
extern uint8  g_beacon_count_tft;                                        // 信标个数（屏幕端副本）
extern uint8  g_target_point_num_tft;                                    // 序列长度（屏幕端副本）

// 函数声明
void    TFT_init(void);                                                  // TFT 屏幕初始化
void    KEY_init(void);                                                  // 按键 GPIO 初始化
void    TFT_KEYput(uint8 get_key1, uint8 get_key2, uint8 get_key3);     // 按键菜单状态机
void    Load_All_From_Flash(void);                                       // 从 Flash 加载所有参数到 RAM
void    ipc_push_params(uint8 cmd_mask);                                 // 将修改的参数推送到共享内存，供 CM7_0 读取

void    KEY_Debounce(void);                                              // 按键消抖（每个主循环调用一次）
uint8   KEY_WasPressed(uint8 idx);                                       // 检测按键按下沿（0=KEY1, 1=KEY2, 2=KEY3）
void    KEY_DebugPrint(uint8 idx);                                       // 调试：打印按键内部状态

#endif /* __TFT_H__ */
