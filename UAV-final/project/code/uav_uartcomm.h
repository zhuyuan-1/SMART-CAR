#ifndef CODE_UAV_UARTCOMM_H_
#define CODE_UAV_UARTCOMM_H_

#include "zf_common_headfile.h"

#define UART_INDEX              (UART_6)                                         // 默认 UART_2
#define UART_BAUDRATE           (460800)                                         // 默认 460800
#define UART_TX_PIN             (UART6_TX_P03_1)                                 // 默认 UART0_TX_P07_1
#define UART_RX_PIN             (UART6_RX_P03_0)                                 // 默认 UART0_RX_P07_0

typedef struct
{
    uint8  car_x;
    uint8  car_y;
    uint8  car_direction;
    uint8  signs_x;
    uint8  signs_y;
    uint16 coom_timespan;
}uart_comm;//发送用结构体类型

typedef struct
{
    uint16  car_x_v;
    uint16  car_y_v;
    uint8   uav_spot;
    uint8   car_a;
    uint8   car_b;
    uint8   car_c;
}uart_comm_get;//发送用结构体类型

extern uart_comm uart_send;              // 发送用的结构体
extern uart_comm_get uart_get;           // 定义接收结构体

extern uint8 out_judge;//建立后输出1
extern uint8 duta_judge;//发送标置位
extern float open_uart_judge;//车启动标志位
extern uint8 open_judge;//车启动标志位

void uav_uart_init(void);

void uav_uart_interrupt_handler (void);

void uav_uart_text(void);
void uav_uart_slave(void);

#endif /* CODE_UART_SLAVE_H_ */