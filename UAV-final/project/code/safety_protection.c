#include "code.h"

#define POWER_VOLTAGE_COEFF (222.0f/(1232.75f*22.0f))
uint8 IMU_safe_count = 0;
float voltage_power_supply = 12.0f;         //电源总电压
//无人机45度保护
void IMU_safe(float front_gyro,float rollove_gyro)
{
    if(fabs(front_gyro) > 55 || fabs(rollove_gyro) > 55)
    {
      if(IMU_safe_count>150)
      {
        IMU_safe_count = 0;
        stop=2;
      }
      else
        IMU_safe_count++;
    }
        else
    {
        IMU_safe_count=0;
    }
}
//按键初始化
void KEY2_init(void)
{
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);           // 初始化 KEY3 输入 默认高电平 上拉输入
}

//电源检测adc初始化
void power_supply_init(void)
{
    adc_init(ADC_power_supply, ADC_12BIT);
}

//获取电源电压
void voltage_adc(void)
{
    voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply,30) * POWER_VOLTAGE_COEFF;;
}
