#ifndef __SAFETY_PROTECTION_H
#define __SAFETY_PROTECTION_H

#include "zf_common_headfile.h"

#define ADC_power_supply              (ADC0_CH21_P07_5)    //电源检测ADC引脚
#define KEY4                         (P20_3)             //开关按键

extern float voltage_power_supply;                      //电源电压
extern uint8 IMU_safe_count;

void power_supply_init(void);
void KEY2_init(void);
void voltage_adc(void);
void IMU_safe(float front_gyro,float rollove_gyro);
float my_abs(float value);

#endif
