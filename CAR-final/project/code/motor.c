#include "motor.h"

uint32 auu_motor(int32 c)
 {
     if(c > 5000)
             return 5000;
     else if(c < 0)
             return 0;
     else
         return c;
 }

//电机初始化
void motor_init(void)
{
    pwm_init(PWM_L1, FREQ_MOTOR, 0);
    pwm_init(PWM_R1, FREQ_MOTOR, 0);
    pwm_init(PWM_L2, FREQ_MOTOR, 0);
    pwm_init(PWM_R2, FREQ_MOTOR, 0);
    gpio_init(DIR_L1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_R1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_L2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_R2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

//函数名称          motor_out
//函数功能          电机驱动函数
//参数说明          right1_pwm：右前电机pwm占比
//参数说明          left1_pwm：左前电机pwm占比
//参数说明          right2_pwm：右后电机pwm占比
//参数说明          left2_pwm：左后电机pwm占比
//返回类型          void
void motor_out(float right1_pwm,float left1_pwm,float right2_pwm,float left2_pwm)
{
    if(right1_pwm >= 0)
    {
        gpio_set_level(DIR_R1,GPIO_HIGH);
        pwm_set_duty(PWM_R1, auu_motor((int32)right1_pwm));
    }
    else
    {
        gpio_set_level(DIR_R1,GPIO_LOW);
        pwm_set_duty(PWM_R1, auu_motor((int32)(-right1_pwm)));
    }
    if(right2_pwm >= 0)
    {
        gpio_set_level(DIR_R2, GPIO_HIGH);
        pwm_set_duty(PWM_R2, auu_motor((int32)right2_pwm));
    }
    else
    {
        gpio_set_level(DIR_R2, GPIO_LOW);
        pwm_set_duty(PWM_R2, auu_motor((int32)(-right2_pwm)));
    }
    if(left1_pwm >= 0)
    {
        gpio_set_level(DIR_L1, GPIO_LOW);
        pwm_set_duty(PWM_L1, auu_motor((int32)left1_pwm));
    }
    else
    {
        gpio_set_level(DIR_L1, GPIO_HIGH);
        pwm_set_duty(PWM_L1, auu_motor((int32)(-left1_pwm)));
    }
     if(left2_pwm >= 0)
    {
        gpio_set_level(DIR_L2, GPIO_LOW);
        pwm_set_duty(PWM_L2, auu_motor((int32)left2_pwm));
    }
    else
    {
        gpio_set_level(DIR_L2, GPIO_HIGH );
        pwm_set_duty(PWM_L2, auu_motor((int32)(-left2_pwm)));
    }
}