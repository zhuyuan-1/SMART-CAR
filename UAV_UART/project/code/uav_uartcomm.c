#include "uav_uartcomm.h"
uart_comm uart_send;              // 定义发送结构体

uint8 uart_get_data[128];                                                       // 串口接收数据缓冲区
uint8 fifo_get_data[128];                                                       // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
uint8 communication_judge = 0;                                                  // 串口通信状态机标志位
uint8 out_judge = 0;                                                            // 输出标志位
uint8 duta_judge = 1;                                                           // 串口发送标志位
uint8 duta[9]={0};                                                             // 数据包
uint16 time_uart = 25000;                                                       // 25ms,以us形式发送
fifo_struct uart_data_fifo;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信从机状态机函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_slave();
//-------------------------------------------------------------------------------------------------------------------
void uav_uart_slave(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo);
    
    switch(communication_judge)
    {
        case 0: {
            if(fifo_data_count >= 2)            // 至少2字节才检查握手包
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
                
                // 遍历查找握手包（修正数组越界问题）
                for(uint32 i = 1; i < fifo_data_count; i++)  // i从1开始，避免i-1越界
                {
                    if(fifo_get_data[i-1] == 0xCC && fifo_get_data[i] == 0xDD)
                    {
                        communication_judge = 1;
                        // 发送握手应答
                        uint8 a[2] = {0xCC, 0xDD};
                        uart_write_buffer(UART_INDEX, a, 2);
                        out_judge = 1;
                        break;  // 找到后退出循环，避免重复处理
                    }
                }
            }
        } break;
        
        case 1: {
            // 只在需要时发送一次数据包（避免持续发送）
            if(duta_judge)
            {
                duta[0]  = 0x2E;                            // 帧头
                
//                duta[1]  = 0x04 & 0xFF;                     // 车体x坐标
//                duta[2]  = 0x06 & 0xFF;                     // 车体y坐标
//                duta[3]  = 0x02 & 0xFF;                     // 车体方向
//                duta[4]  = 0x0A & 0xFF;                     // 信标x坐标
//                duta[5]  = 0x0C & 0xFF;                     // 信标y坐标
                
                duta[1] = uart_send.car_x & 0xFF;
                duta[2] = uart_send.car_y & 0xFF;
                duta[3] = uart_send.car_direction & 0xFF;
                duta[4] = uart_send.signs_x & 0xFF;
                duta[5] = uart_send.signs_y & 0xFF;
                duta[6]  = (time_uart >> 8) & 0xFF;         // 时间间隔高八位
                duta[7]  = time_uart & 0xFF;                // 时间间隔低八位
                duta[8] = 0x1F;                             // 帧尾
                
                uart_write_buffer(UART_INDEX, duta, 9);
                duta_judge = 0;  // 发送完成后重置标志，避免重复发送
            }
        } break;
        
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信上位机测试函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_text();
//-------------------------------------------------------------------------------------------------------------------

void uav_uart_text(void)
{
  fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
  uint32 a = 2; 
  if(fifo_data_count != 0)                                                // 读取到数据了
  {
    fifo_read_buffer(&uart_data_fifo, fifo_get_data, &a, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
    //uart_write_string(UART_INDEX, "\r\nUART get data:");                // 输出测试信息
    uart_write_buffer(UART_INDEX, fifo_get_data, 1);                  // 将读取到的数据发送出去
  }

}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信初始化函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_init();
//-------------------------------------------------------------------------------------------------------------------
void uav_uart_init(void)
 {
   fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 128);              // 初始化 fifo 挂载缓冲区
   uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
   uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
   fifo_clear (&uart_data_fifo);
 //  uart_write_string(UART_INDEX, "UART INIT SUCCESS.");                        // 输出测试信息
 //  uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
//   uart_write_byte(UART_INDEX, '\n');                                          // 输出换行

 }
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用
// 参数说明       void
// 返回参数       void
// 使用示例       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uav_uart_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    if(uart_query_byte(UART_INDEX, &get_data))                                  // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        fifo_write_element(&uart_data_fifo, get_data);                   // 将数据写入 fifo
    }
}
