#include "code.h"

uint32 led_time = 0;     // 由外部中断每4ms递增

// 飞控指示灯初始化
void led_judge_init(void)
{
  pwm_init(LED_red1 , 1000, 0); 
  pwm_init(LED_red2 , 1000, 0); 
  pwm_init(LED_blue1, 1000, 0); 
  pwm_init(LED_blue2, 1000, 0); 
}

// 飞控指示灯常亮
void led_judge_light(uint32_t duty)
{
  pwm_set_duty(LED_red1, duty);
  pwm_set_duty(LED_red2, duty);
  pwm_set_duty(LED_blue1, duty);
  pwm_set_duty(LED_blue2, duty);
}

// 飞控指示（呼吸灯）
void led_judge_use(void)
{
  if (stop == 1)
  {
    led_judge_light(LED_PWM_MAX);   // 全亮
    led_time = 0;                   // 停止时复位
  }
  else
  {
    // ① 关键修改：确保 led_time 不会无限累加，达到一个周期后清零
    if (led_time >= PERIOD_TICKS) {
        led_time = 0;
    }

    // ② 此时 led_time 保证在 0 ~ PERIOD_TICKS-1 范围内
    float phase = (float)led_time / PERIOD_TICKS * 2.0f * PI;
    float sin_val = (sinf(phase) + 1.0f) / 2.0f;   // 使用 sinf 单精度更快
    uint32_t duty = (uint32_t)(sin_val * LED_PWM_MAX);
    
    led_judge_light(duty);
  }
}