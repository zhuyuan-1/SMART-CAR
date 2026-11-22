#include "code.h"

#define COMPENSATION_X          2.08    // X坐标径向补偿
#define COMPENSATION_Y          1.68    // Y坐标径向补偿
#define PIX_PER_DEG_X           3.28f   // pitch → 图像X轴偏移(像素/度)
#define PIX_PER_DEG_Y           1.88f   // roll  → 图像Y轴偏移(像素/度)

//降落参数
#define LAND_SPEED_DESCENT      100.0f   // 快速下降速度 mm/s
#define LAND_SPEED_FLARE        50.0f   // 近地缓降速度 mm/s
#define LAND_ALT_FLARE          800.0f   // 切换缓降的高度阈值 mm
#define TOUCHDOWN_ALT_THRESH    200.0f    // 触地判定高度 mm (TOF盲区50mm以上)
#define TOUCHDOWN_SPEED_THRESH  20.0f    // 触地判定速度 mm/s
#define TOUCHDOWN_CONFIRM_CNT   30       // 触地确认周期数 (7ms*30≈210ms)
#define TOUCHDOWN_DISARM_DELAY  50       // 触地后等待周期数 (7ms*50≈350ms)
#define LAND_TIMEOUT_CYCLES     2000     // 降落超时周期数 (~14秒)

uint8  land_state = LAND_IDLE;
uint16 land_touchdown_cnt = 0;
uint16 land_timeout_cnt = 0;
// ============================
//全局变量
//目标值
float target_high = 1000.0f;              //高度（mm），之后根据TOF来设计，单位：mm
float target_x = 45.0f;                   //x轴方向的目标位移,对应前进位移
float target_y = 94.0f;                   //y轴方向的目标位移,对应左右位移
float target_x_speed = 0.0f;              //x轴方向的目标速度,对应前进速度
float target_y_speed = 0.0f;              //y轴方向的目标速度,对应左右速度
float target_z_speed = 0.0f;              //z轴方向的目标速度,对应上下速度
float target_motor_front = 1.5f;          //前后目标角度roll
float target_motor_rollove = -0.4f;       //左右侧翻目标角度pitch
float target_motor_turn = 0.0f;           //左右侧翻目标角度yaw
float target_gyro_front = 0.0f;           //前后目标角速度
float target_gyro_rollove = 0.0f;         //左右侧翻目标角速度
float target_gyro_turn = 0.0f;            //方向目标角速度
//全局系数
float comp_pixel_x = 0;
float comp_pixel_y = 0;
uint8 i=0;                                 // 计数器计时
uint8 first_time = 1;                     // 电池改油门判断位
uint16 vol_time = 0;                      // 电池计数
int16 car_find = 0;                       // 定高稳定后在开始追车
float open_uart_judge = 0.0f;             // 车启动标志位
uint8 stop = 0;
uint8 flow_change = 0;
float stop_m2 = 0;
float gyro_x_f = 0, gyro_y_f = 0, gyro_z_f = 0;
float roll,pitch,yaw;
float X_speed_now,Y_speed_now,High_speed_now;
float X_distance_now,Y_distance_now;
float high_maintain = 0.0f;
uint8 high_judge = 0;
float y_distance = 0.0f;                   //光流y轴位移
float x_distance = 0.0f;                   //光流x轴位移
float x_beacon = 0.0f;                    //信标X坐标
float y_beacon = 0.0f;                    //信标Y坐标
float x_car = 0.0f;                       //小车X坐标
float y_car = 0.0f;                       //小车Y坐标
float car_image_flag = 0.0f;              //小车方向
signed short int PWM_out1, PWM_out2, PWM_out3, PWM_out4; // 四个电机PWM输出
float Balance_Pwm_front = 0.0f, Balance_Pwm_rollove = 0.0f, Balance_Pwm_turn = 0.0f;
float Gyro_Pwm_front = 0.0f, Gyro_Pwm_rollove = 0.0f, Gyro_Pwm_turn = 0.0f; // 平衡PWM值
float Speed_Pwm_front = 0.0f,Speed_Pwm_rollove = 0.0f,Speed_Pwm_high = 0.0f;
float high_pwm = 0.0f,x_pwm = 0.0f,y_pwm = 0.0f;
// PID参数
pid_param_t motor_high;          //高度PID参数
pid_param_t motor_x;             //前后位移PID参数
pid_param_t motor_y;             //左右位移PID参数
pid_param_t motor_x_speed;       //前后速度PID参数
pid_param_t motor_y_speed;       //左右速度PID参数
pid_param_t motor_z_speed;       //上下速度PID参数
pid_param_t motor_front;         //前后角度PID参数
pid_param_t motor_rollove;       //左右侧翻角度PID参数
pid_param_t motor_gyro_front;    //前后陀螺仪PID参数
pid_param_t motor_gyro_rollove;  //左右侧翻陀螺仪PID参数
pid_param_t motor_turn;          //方向角度PID参数
pid_param_t motor_gyro_turn;     //方向陀螺仪PID参数

// 初始化PID参数
void pid_init(void)
{
 // 初始化前后位移PID
    PidInit(&motor_x);
    PidChange(&motor_x, 0.2f, 0.0f, 0.2f);
    PidChange_imax(&motor_x, 1.0f);//改变限幅

    // 初始化左右位移PID
    PidInit(&motor_y);
    PidChange(&motor_y, 0.128f, 0.0f, 0.15f);
    PidChange_imax(&motor_y, 1.0f);//改变限幅

    // 初始化高度PID
    PidInit(&motor_high);
    PidChange(&motor_high, 1.65f, 0.0f, 0.7f);
    PidChange_imax(&motor_high, 2000.0f);//改变限幅

    // 初始化前后速度PID
    PidInit(&motor_x_speed);
    PidChange(&motor_x_speed, 1.2f, 0.01f, 0.8f);
    PidChange_imax(&motor_x_speed, 500.0f);//改变限幅

    // 初始化左右速度PID
    PidInit(&motor_y_speed);
    PidChange(&motor_y_speed, 1.0f, 0.01f, 0.58f);
    PidChange_imax(&motor_y_speed, 500.0f);//改变限幅

    // 初始化上下速度PID
    PidInit(&motor_z_speed);
    PidChange(&motor_z_speed, 3.0f, 0.0f, 1.8f);
    PidChange_imax(&motor_z_speed, 2000.0f);//改变限幅

    // 初始化前后角度PID
    PidInit(&motor_front);
    PidChange(&motor_front, 1.0f, 0.05f, 0.8f);
    PidChange_imax(&motor_front, 20.0f);//改变限幅

    // 初始化左右侧翻角度PID
    PidInit(&motor_rollove);
    PidChange(&motor_rollove, 0.7f, 0.03f, 0.55f);
    PidChange_imax(&motor_rollove, 30.0f);//改变限幅

    // 初始化方向角度PID
    PidInit(&motor_turn);
    PidChange(&motor_turn, 1.5f, 0.02f, 0.72f);
    PidChange_imax(&motor_turn, 20.0f);//改变限幅

    // 初始化前后角速度PID
    PidInit(&motor_gyro_front);
    PidChange(&motor_gyro_front, 5.0f, 0.2f, 3.0f);
    PidChange_imax(&motor_gyro_front, 3000.0f);//改变限幅

     // 初始化左右侧翻角速度PID
     PidInit(&motor_gyro_rollove);
     PidChange(&motor_gyro_rollove, 3.0f, 0.18f, 2.0f);
     PidChange_imax(&motor_gyro_rollove, 3000.0f);//改变限幅
     
     // 初始化方向角速度PID
     PidInit(&motor_gyro_turn);
     PidChange(&motor_gyro_turn, 15.0f, 0.5f, 9.0f);
     PidChange_imax(&motor_gyro_turn, 3000.0f);//改变限幅
 }
//电机占空比限制函数
int16 auu(int16 c)
 {
     if(c > 10000)
             return 10000;
     else if(c < 0)
             return 0;
     else
         return c;
 }
//速度控制计算PID
float SpeedControl(pid_param_t * pid, float target_speed, float now_speed)
{
    float error = target_speed - now_speed;
    /* 累积误差 */
    pid->integrator += error;
    /* 误差限幅 */
    pid->integrator = constrain_float(pid->integrator, -(pid->imax), pid->imax);
    
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_i = constrain_float(pid->out_i, -5, 5);
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
        
    return pid->out; // 返回控制输出
}

// 平衡控制计算PID
float Balance(pid_param_t * pid ,float Angle, float target_angle)
{
    float error = target_angle - Angle;
    /* 累积误差 */
    pid->integrator += error;
    /* 误差限幅 */
    pid->integrator = constrain_float(pid->integrator, -(pid->imax), pid->imax);
    
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_i = constrain_float(pid->out_i, -10, 10);
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
        
    return pid->out; // 返回控制输出
}
// 陀螺仪控制计算PID
float GyroControl(pid_param_t * pid, float target_gyro, float now_gyro)
{
    float error = target_gyro - now_gyro;
    /* 累积误差 */
    pid->integrator += error;
    /* 误差限幅 */
    pid->integrator = constrain_float(pid->integrator, -(pid->imax), pid->imax);
    
    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_i = constrain_float(pid->out_i, -500, 500);
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
        
    return pid->out; // 返回控制输出
}

// 高度位置控制计算PID
float highControl(pid_param_t * pid, float target_high, float now_high)
{
    float high_pwm1 = 0;
    if(my_abs(target_high - now_high) > 50)//50mm死区截断
       high_pwm1 = PidLocCtrl(pid, target_high - now_high);
    else
       high_pwm1 = PidLocCtrl(pid, 0);
    return high_pwm1; // 返回控制输出
}

// 信标位置控制计算PID
float distanceControl(pid_param_t * pid, float target_distance, float now_distance,float distance_down)
{
    float distance_pwm = 0;
    if(my_abs(target_distance - now_distance) > distance_down)//distance_down个像素点死区截断
       distance_pwm = PidLocCtrl(pid, target_distance - now_distance);
    else
       distance_pwm = PidLocCtrl(pid, 0);
    return distance_pwm; // 返回控制输出
}
// 计算电压自适应悬停油门，100ms一次
float Get_Auto_Hover_Throttle(void)
{
    float bat_vol = voltage_power_supply;//获取电池电量
    
    // 电压限幅，防止极值溢出
    if(bat_vol > VOL_MAX) bat_vol = VOL_MAX;
    if(bat_vol < VOL_MIN) bat_vol = VOL_MIN;

    // 线性补偿：电压越低，油门越大
    // 最终自适应悬停油门
    float hover_thr = VOL_BASE * THROTTLE_HOVER / bat_vol;
    
    return hover_thr;
}
//根据小车x,y坐标锁死高度
void high_distancechange(float x,float y)
{
//  if(x>64 && x<124 && y>55 && y<95)
//    tof_judge = 1;
//  else
//    tof_judge = 0;
}

// 降落状态机 —— 每个控制周期(7ms)调用一次
void control_Landing(void)
{
    // 紧急停止优先
    if (stop == 2)
    {
        land_state = LAND_DISARMED;
        return;
    }

    // 超时保护：防止TOF故障导致无限降落
    land_timeout_cnt++;
    if (land_timeout_cnt >= LAND_TIMEOUT_CYCLES)
    {
        land_state = LAND_DISARMED;
        return;
    }

    switch (land_state)
    {
        case LAND_DESCENT:
            // 根据当前高度选择下降速度
            if (tof_now_mm <= LAND_ALT_FLARE)
            {
                land_state = LAND_FLARE;
                target_z_speed = LAND_SPEED_FLARE;
                land_touchdown_cnt = 0;
            }
            else
            {
                target_z_speed = LAND_SPEED_DESCENT;
            }
            target_high = 0.0f;  // 旁路高度位置环
            break;

        case LAND_FLARE:
            // 触地检测：高度低于阈值 + 速度接近零
            if (tof_now_mm < TOUCHDOWN_ALT_THRESH )
            {
                land_touchdown_cnt++;
                if (land_touchdown_cnt >= TOUCHDOWN_CONFIRM_CNT)
                {
                    land_state = LAND_TOUCHDOWN;
                    land_touchdown_cnt = 0;  // 复用作解鎖倒计时
                }
                land_state = LAND_TOUCHDOWN;
            }
            else
            {
                land_touchdown_cnt = 0;
            }

            // 如果弹跳回升超过缓降高度，回到快速下降
            if (tof_now_mm > LAND_ALT_FLARE)
            {
                land_state = LAND_DESCENT;
                target_z_speed = LAND_SPEED_DESCENT;
            }
            else
            {
                target_z_speed = LAND_SPEED_FLARE;
            }
            target_high = 0.0f;
            break;

        case LAND_TOUCHDOWN:
            land_touchdown_cnt++;
            if (land_touchdown_cnt >= TOUCHDOWN_DISARM_DELAY)
            {
                land_state = LAND_DISARMED;
                stop = 2;
            }
            else
            {
                stop = 2;
                // 短暂维持微小下推力，防止弹跳
                target_z_speed = LAND_SPEED_FLARE * 0.5f;
                target_high = 0.0f;
            }
            break;

        case LAND_DISARMED:
        default:
            target_z_speed = 0.0f;
            break;
    }
}

//飞控初始化
void control_init(void)
{
    pid_init();                         // 初始化PID
    optical_flow_init();                // 初始化光流通讯功能
    power_supply_init();                // adc检测电源电压初始化
    imu_rx_init();                      // 姿态传感器初始化 
    acc_init();
    KF_3Axis_Init(&speed_kf, 0.025);
    small_driver_uart_init();	        // 初始化四路电调驱动通讯功能
}

//飞控平衡算法
void control_Balance(void)
{
    // 更新变化值
    i = (i + 1) % 12;
    vol_time++;
    if(car_find >= 0)
      car_find++;
    
        // ====== 降落状态机 ======
    if (land_state != LAND_IDLE)
    {
        control_Landing();
        if (land_state == LAND_DISARMED)
        {
            small_driver_set_duty(0, 0, 0, 0);
            return;
        }
    }
    // ========================
    // 更新传入数据
    roll           = eulerAngle.roll;
    pitch          = eulerAngle.pitch;
    yaw            = eulerAngle.yaw;
    X_speed_now    = speed_now[0]*10;
    Y_speed_now    = speed_now[1]*10;
    High_speed_now = speed_now[2]*1000;//将m/s变为mm/s
    //旋转补偿
    static float pitch_hist[3] = {0}, roll_hist[3] = {0};
    pitch_hist[2] = pitch_hist[1];
    pitch_hist[1] = pitch_hist[0];
    pitch_hist[0] = pitch;
    roll_hist[2]  = roll_hist[1];
    roll_hist[1]  = roll_hist[0];
    roll_hist[0]  = roll;

    comp_pixel_x = pitch_hist[2] * PIX_PER_DEG_X; // pitch→图像X偏移
    comp_pixel_y = roll_hist[2]  * PIX_PER_DEG_Y; // roll→图像Y偏移
//    if((roll_hist[2]>20.8&&roll_hist[1]>20.8)||(roll_hist[2]<-20.8&&roll_hist[1]<-20.8))
//    X_distance_now = y_car + comp_pixel_y;
//    else
//      X_distance_now = y_car;
//    if((pitch_hist[2]>20.8&&pitch_hist[1]>20.8)||(pitch_hist[2]<-20.8&&pitch_hist[1]<-20.8))
//    Y_distance_now = x_car - comp_pixel_x;
//    else
//      Y_distance_now = x_car;
    if(roll_hist[2]>12.0||roll_hist[2]<-12.0)
    X_distance_now = y_car + comp_pixel_y;
    else
      X_distance_now = y_car;
    if(pitch_hist[2]>12.8||pitch_hist[2]<-12.8)
    Y_distance_now = x_car - comp_pixel_x;
    else
      Y_distance_now = x_car;
    
    //限定高度
    high_distancechange(X_distance_now,Y_distance_now);
    //根据电池电量改变悬停中值
    if(first_time == 1)
    {
        high_maintain = THROTTLE_HOVER;
        if(vol_time >= 300)//2.1s等待电池突变完成
        {
          vol_time =  0;
          first_time = 0;
        }
    }
    else if(vol_time >= 15 && first_time == 0)//105ms计算一次
    {
      high_maintain = Get_Auto_Hover_Throttle();
      vol_time =  0;
    }  
    //定高后再追车
    if(car_find >= 600)//4.2s
    {
      car_find = -1;
      open_uart_judge = 2.5f;
    }
    //定高后再追车
    if(car_find >= 500)//3.5s
      flow_change = 1;
    
    //计算速度pwm值
    // 低速计算：位置PID（每6次调用计算一次）
    if(i % 6 == 0)
    {
//           // --- TOF跳变保护：防抖恢复机制 ---
//        //  单次跳变(>150mm)冻结，需连续3帧正常才恢复跟踪
//        //  避免飞到车顶边沿时 TOF 在 地面?车顶 间来回抖
//        static float tof_safe = 1000.0f;
//        static bool  tof_safe_init = false;
//        static uint8 tof_jump_cnt = 0;
//
//        if (!tof_safe_init) {
//            tof_safe = tof_now_mm;
//            tof_safe_init = true;
//        }
//
//        if (fabsf(tof_now_mm - tof_safe) <= 150.0f) {
//            if (tof_jump_cnt == 0) {
//                tof_safe = tof_now_mm;            // 正常跟踪
//            } else {
//                tof_jump_cnt--;                   // 稳定帧计数
//                if (tof_jump_cnt == 0)
//                    tof_safe = tof_now_mm;        // 连续稳定后恢复
//            }
//        } else {
//            tof_jump_cnt = 3;                     // 检测到跳变，冻结并等待恢复
//        }
//        
//        // --- 车在正上方时切掉高度环，纯悬停避免 TOF 误读车顶 ---
//        float center_dx = fabsf(X_distance_now - 60.0f);
//        float center_dy = fabsf(Y_distance_now - 94.0f);
//        if (center_dx < 15.0f && center_dy < 15.0f) {
//            high_pwm = 0.0f;    // 车在正下方，关高度PID，靠自适应悬停油门
//        } else {
//            high_pwm = highControl(&motor_high, target_high, tof_safe);
//        }
      high_pwm = highControl(&motor_high, target_high, tof_now_mm);
      if(flow_change == 1)
      {
        //径向补偿
        float r_norm_x = fabsf(X_distance_now - 60.0f) / 60.0f;
        if (r_norm_x > 1.0f) r_norm_x = 1.0f;
        float scale_x = 1.0f + COMPENSATION_X * (1.0f - r_norm_x * r_norm_x);

        // Y轴：距中心的归一化偏移（0=中心, 1=边缘）
        float r_norm_y = fabsf(Y_distance_now - 94.0f) / 94.0f;
        if (r_norm_y > 1.0f) r_norm_y = 1.0f;
        float scale_y = 1.0f + COMPENSATION_Y * (1.0f - r_norm_y * r_norm_y);

        float dx = target_x - X_distance_now;
        float dy = target_y - Y_distance_now;
        float X_dist_comp = target_x - dx / scale_x;
        float Y_dist_comp = target_y - dy / scale_y;
        //输出
        x_pwm = distanceControl(&motor_x, target_x, X_dist_comp, 2.0f);
        y_pwm = distanceControl(&motor_y, target_y, Y_dist_comp, 3.0f);
      }
    }
    // 中低速计算：速度PID（每3次调用计算一次）
    if(i % 4 == 0)
    {       

      //光流速度
      if(flow_change == 1)
      {
        Speed_Pwm_front   = SpeedControl(&motor_x_speed, target_x_speed - x_pwm, X_speed_now);
        Speed_Pwm_rollove = SpeedControl(&motor_y_speed, target_y_speed - y_pwm, Y_speed_now);
      }
      Speed_Pwm_high = SpeedControl(&motor_z_speed, target_z_speed - high_pwm, High_speed_now);
    }
    // 计算平衡PWM值和高度pwm值
    // 中速计算：角度PID + 高度PID（每2次调用计算一次）
    if(i % 2 == 0)
    {
      Balance_Pwm_front   = Balance(&motor_front,   roll,  target_motor_front   - Speed_Pwm_front);
      Balance_Pwm_rollove = Balance(&motor_rollove, pitch, target_motor_rollove + Speed_Pwm_rollove);
      Balance_Pwm_turn    = Balance(&motor_turn,    yaw,   target_motor_turn);
    }
    // 高速计算：角速度PID（每次调用都计算）
    // 计算陀螺仪控制PWM值
    Gyro_Pwm_front   = GyroControl(&motor_gyro_front  ,target_gyro_front   - Balance_Pwm_front,   gyro_x_f);
    Gyro_Pwm_rollove = GyroControl(&motor_gyro_rollove,target_gyro_rollove - Balance_Pwm_rollove, gyro_y_f);
    Gyro_Pwm_turn    = GyroControl(&motor_gyro_turn   ,target_gyro_turn    + Balance_Pwm_turn,    gyro_z_f);

    // ====================================================================
    // 第五节：PWM 混控 — X 型四旋翼分配
    //
    //  电机布局（俯视图）：        混控矩阵（符号来源于控制律推导）：
    //    M2(前左)  M3(前右)        M1 = 悬停 -高度 - roll -pitch -yaw (CCW)
    //        \      /              M2 = 悬停 -高度 + roll -pitch +yaw (CW)
    //         中心                 M3 = 悬停 -高度 +roll +pitch -yaw (CCW)
    //        /      \              M4 = 悬停 -高度 -roll +pitch +yaw (CW)
    //    M1(后左)  M4(后右)
    //
    //  CCW/CW 为电机转向，yaw 项符号与转向相反以产生反扭矩
    // ====================================================================
    PWM_out1 = (int16)(high_maintain - Speed_Pwm_high - Gyro_Pwm_front - Gyro_Pwm_rollove + Gyro_Pwm_turn);
    PWM_out2 = (int16)(high_maintain - Speed_Pwm_high + Gyro_Pwm_front - Gyro_Pwm_rollove - Gyro_Pwm_turn);
    PWM_out3 = (int16)(high_maintain - Speed_Pwm_high + Gyro_Pwm_front + Gyro_Pwm_rollove + Gyro_Pwm_turn);
    PWM_out4 = (int16)(high_maintain - Speed_Pwm_high - Gyro_Pwm_front + Gyro_Pwm_rollove - Gyro_Pwm_turn);
   
    //pwm输出
    small_driver_set_duty(-auu(PWM_out1),auu(PWM_out2), -auu(PWM_out3), -auu(PWM_out4));
}
