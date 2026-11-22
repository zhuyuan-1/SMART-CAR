#include "adc.h"

float voltage_power_supply = 0.0f;

//ADC初始化
void power_supply_init(void)
{
    adc_init(ADC_power_supply, ADC_12BIT);
}

// 蜂鸣器初始化
void buzzer_init(void)
{
    // 初始化 P19_4 为推挽输出，默认低电平（不响）
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

//电源电压检测
void buzzer_on(void)
{
  voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply, 60)/1200 *11 ;//目标3080~3106
  //printf ("%f\r\n",voltage_power_supply);
  if (voltage_power_supply >= 12.6f)
  {
    gpio_set_level(P19_4, 1); // 输出高电平
  }
  if (voltage_power_supply <= 11.3f)
  {
    gpio_set_level(P19_4, 1); // 输出高电平
    pit_all_close();
  }
}

