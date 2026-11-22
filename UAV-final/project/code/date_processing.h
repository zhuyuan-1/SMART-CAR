#ifndef _DATE_PROCESSING_H
#define _DATE_PROCESSING_H

#include "zf_common_headfile.h"
#include "navigation.h"

extern float tof_now_mm;//实时高度
extern float tof_high_speed;//Tof高度实时速度
extern float optical_x_speed;
extern float optical_y_speed;
extern KF_3Axis_Velocity_t speed_kf;
extern float speed_now[3];

float my_abs(float value);
float gyro_roll_init(float roll_gyro);
float gyro_pitch_init(float pitch_gyro);
void tof_get(float pitch_deg, float roll_deg);
float Turn_target(float target_angle);
float Turn_gyro_fixed(float target_angle, float gyro);
void optical_flow_filter_process(int16 dx_raw, int16 dy_raw,float alpha, int16 threshold,int16 *p_dx_out, int16 *p_dy_out);
void optical_flow_contorl(uint8 flow_change);
void Gyro_Filter_2ms(float raw_x, float raw_y, float raw_z,float *filt_x, float *filt_y, float *filt_z);
#endif /*_DATE_PROCESSING_H*/

