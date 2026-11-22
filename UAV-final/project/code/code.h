//本文件是.h设置文件，总结工程.h
#ifndef CODE_CODE_H_
#define CODE_CODE_H_

//基础配置文件
#include "small_driver_uart_control.h"          // 四路电调串口配置
#include "optical_flow.h"                       // 光流串口配置
#include "led_judge.h"                          // led指示灯配置
#include "wifi.h"                               // wifi配置代码
#include "process_rx.h"                         // IMU配置代码
//机体保护文件
#include "safety_protection.h"                  // 倾角保护
//调试配置文件
#include "wifi_debug.h"                         // wifi_debug调试
// 数据处理算法
#include "date_processing.h"                    // 光流数据与IMU数据处理
#include "navigation.h"
//飞控算法
#include "pid.h"                                // pid算法
#include "control.h"                            // 飞控算法

#endif /* CODE_CODE_H_ */
