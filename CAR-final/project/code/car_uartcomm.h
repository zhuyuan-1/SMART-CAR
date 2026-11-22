#ifndef CODE_CAR_UARTCOMM_H_
#define CODE_CAR_UARTCOMM_H_

#include "zf_common_headfile.h"

#define UART_INDEX              (UART_4)                                         // 默认 UART_2
#define UART_BAUDRATE           (460800)                                         // 默认 460800
#define UART_TX_PIN             (UART4_TX_P14_1)                                 // 默认UART4_TX_P14_1
#define UART_RX_PIN             (UART4_RX_P14_0)                                 // 默认UART4_RX_P14_0

typedef struct
{
    uint8                 car_x;          //车体x坐标
    uint8                 car_y;          //车体y坐标
    uint8                 car_direction;  //车体方向
    uint8                 signs_x;        //信标x坐标
    uint8                 signs_y;        //信标y坐标
    uint8                 start;          //标志位
    uint16                coom_timespan;  //上一次发送数据到这一次发送累计时间（us）
}uart_comm;

typedef struct
{
    uint16  car_x_v;
    uint16  car_y_v;
    uint8   uav_spot;
    uint8   car_a;
    uint8   car_b;
    uint8   car_c;
}uart_comm_send;                              //发送用结构体类型

extern uart_comm uart_get;                   // 坐标结构体
extern uart_comm_send uart_send;             // 发送结构体结构体

extern uint8 out_judge;                     //建立后输出1
extern uint8 duta_judge;               //发送标置位

void car_uart_init(void);
void car_uart_interrupt_handler(void);
void car_uart_get(void);

#endif /* CODE_CAR_UARTCOMM_H_ */