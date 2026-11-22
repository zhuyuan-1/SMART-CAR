#ifndef CODE_LED_JUDGE_H_
#define CODE_LED_JUDGE_H_

#include "zf_common_headfile.h"

#define LED_red1                    (TCPWM_CH13_P00_3)
#define LED_red2                    (TCPWM_CH19_P08_0)
#define LED_blue1                   (TCPWM_CH36_P21_6)
#define LED_blue2                   (TCPWM_CH40_P12_4)

#define LED_PWM_MAX   10000
#define BREATH_PERIOD 4000   // 呼吸周期（毫秒）
#define TIMER_STEP    2      // 中断间隔（毫秒）
#define PERIOD_TICKS  (BREATH_PERIOD / TIMER_STEP)   // 1000次中断

extern uint32 led_time;

void led_judge_init(void);
void led_judge_light(uint32 duty);
void led_judge_use(void);

#endif /* CODE_LED_JUDGE_H_ */
