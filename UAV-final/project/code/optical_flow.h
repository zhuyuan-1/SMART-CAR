#ifndef CODE_OPTICAL_FLOW_H
#define CODE_OPTICAL_FLOW_H_

#include "zf_common_headfile.h"

#define UART_INDEX              (UART_2)                                         // 默认 UART_2
#define UART_BAUDRATE           (460800)                                         // 默认 460800
#define UART_TX_PIN             (UART2_TX_P10_1)                                 // 默认 UART2_TX_P10_1
#define UART_RX_PIN             (UART2_RX_P10_0)                                 // 默认 UART2_RX_P10_0

typedef struct
{
    uint16                optical_TOF1_mm;        //tof1高度信息（单位：mm）
    uint16                optical_TOF2_mm;        //tof2高度信息（单位：mm）
    int16                 optical_x_drift;        //一次读取x的位移
    int16                 optical_y_drift;        //一次读取y的位移
    int32                 optical_x_drift_i;      //读取x的持续位移
    int32                 optical_y_drift_i;      //读取y的持续位移
    uint16                optical_timespan;       //上一次发送光流数据到这一次发送累计时间（us）
}optical_flow;

extern optical_flow flow_get;                   // 光流结构体
extern float observe_speed[3];

void optical_flow_init(void);
void flow_uart_interrupt_handler(void);
void opyical_flow_get(void);

#endif /* CODE_OPTICAL_FLOW_H */
