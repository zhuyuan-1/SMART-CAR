/*
 * init.c
 *
 *  Created on: 2026年1月9日
 *  此库是用于各种外社的初始化
 */
#include"init.h"


//函数名称    tft_init()
//函数功能    tft初始化
//参数说明    void
//备注
void tft_init()
{
    tft180_set_dir(TFT180_PORTAIT);                                           // 需要先横屏 不然显示不下
    tft180_set_color(RGB565_RED,RGB565_WHITE);
    tft180_init();
    tft180_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);                                                  // 闪灯表示异常
    }
    tft180_show_string(0, 16, "init success.");
}




