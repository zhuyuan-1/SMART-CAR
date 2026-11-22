#ifndef CODE_TFT_H_
#define CODE_TFT_H_

#include "zf_common_headfile.h"

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)


extern float now_start;              //开始运行标志位
extern float V_a;                    //速度赋值
extern float voltage_power_supply;   //电源电压
extern float stop_m2;                //停止标志位

//初始化
void TFT_init(void);
void KEY_init(void);

//按键菜单
void TFT_KEYput(uint8 get_key1,uint8 get_key2,uint8 get_key3);


#endif /* CODE_ADC_H_ */
