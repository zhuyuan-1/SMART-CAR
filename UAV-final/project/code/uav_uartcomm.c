#include "uav_uartcomm.h"

uart_comm uart_send;              // 定义发送结构体
uart_comm_get uart_get;           // 定义接收结构体

uint8 uart_get_data[128];                                                       // 串口接收数据缓冲区
uint8 fifo_get_data[128];                                                       // fifo 输出读出缓冲区
float open_uart_judge = 0.0f;                                                      // 车启动标志位
uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数
uint8 communication_judge = 0;                                                  // 串口通信状态机标志位
uint8 out_judge = 0;                                                            // 输出标志位
uint8 duta_judge = 1;                                                           // 串口发送标志位
uint8 duta[10]={0};                                                             // 数据包
uint16 time_uart = 20000;                                                       // 25ms,以us形式发送
uint8 open_judge= 0;//车启动标志位
fifo_struct uart_data_fifo;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信从机状态机函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_slave();
//-------------------------------------------------------------------------------------------------------------------
void uav_uart_slave(void)
{
    uint32 len = 10;                           // 数据包长度标志位

    fifo_data_count = fifo_used(&uart_data_fifo);

    switch(communication_judge)
    {
        case 0: {
            if(fifo_data_count >= 2)            // 至少2字节才检查握手包
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);

                // 搜索握手包（添加边界检查）
                for(uint32 i = 1; i < fifo_data_count; i++)  // i从1开始，避免i-1越界
                {
                    if(fifo_get_data[i-1] == 0xCC && fifo_get_data[i] == 0xDD)
                    {
                        communication_judge = 1;
                        // 回复握手应答
                        uint8 a[2] = {0xCC, 0xDD};
                        uart_write_buffer(UART_INDEX, a, 2);
                        out_judge = 1;
                        // 清空FIFO中残留的多余握手包字节，防止后续数据偏移
                        fifo_clear(&uart_data_fifo);
                        break;  // 找到就退出循环，避免重复发送
                    }
                }
            }
        } break;

        case 1: {
            // 首次进入case 1时清空FIFO残留，防止热重启导致的数据偏移
            static uint8 first_entry = 1;
            if(first_entry) {
                fifo_clear(&uart_data_fifo);
                first_entry = 0;
            }

            // 超时计数：长时间未收到有效帧则退回握手阶段
            static uint32 comm_timeout = 0;
            comm_timeout++;

            // **************************** 发送数据处理 ****************************
            if(duta_judge)
            {
                duta[0]  = 0x2E;                            // 帧头
                duta[1] = uart_send.car_x & 0xFF;
                duta[2] = uart_send.car_y & 0xFF;
                duta[3] = uart_send.car_direction & 0xFF;
                duta[4] = uart_send.signs_x & 0xFF;
                duta[5] = uart_send.signs_y & 0xFF;
                duta[6] = (time_uart >> 8) & 0xFF;         // 时间戳高八位
                duta[7] = time_uart & 0xFF;                // 时间戳低八位
                duta[8] = open_judge & 0xFF;              // 控制量
                duta[9] = 0x1F;                            // 帧尾
                uart_write_buffer(UART_INDEX, duta, 10);
                duta_judge = 0;  // 发送完成后清除标志，避免重复发送
            }
            // **************************** 接收数据处理 ****************************
            fifo_data_count = fifo_used(&uart_data_fifo);  // 查看 fifo 是否有数据
            if(fifo_data_count >= len)          // 确保数据长度足够
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &len, FIFO_READ_AND_CLEAN);

                // 从缓冲区尾部向前搜索帧头(0xAA)帧尾(0xCC)，j>=9确保j-9不越界
                for(uint32 j = fifo_data_count-1; j >= 9; j--)
                {
                  if(fifo_get_data[j-9] == 0xAA && fifo_get_data[j] == 0xCC)
                  {
                    uart_get.car_x_v  = (uint16)((fifo_get_data[j-8] << 8) | fifo_get_data[j-7]);
                    uart_get.car_y_v  = (uint16)((fifo_get_data[j-6] << 8) | fifo_get_data[j-5]);
                    uart_get.uav_spot = fifo_get_data[j-4];
                    uart_get.car_a    = fifo_get_data[j-3];
                    uart_get.car_b    = fifo_get_data[j-2];
                    uart_get.car_c    = fifo_get_data[j-1];
                    //printf("%d,%d\r\n",uart_get.car_x_v,uart_get.car_y_v);
                    comm_timeout = 0;  // 收到有效帧，重置超时计数
                    break;  // 找到完整帧后退出
                  }
                }
            }

            // 超时回退：长时间(约20秒)未收到有效帧，退回握手阶段重新同步
            if(comm_timeout > 1000) {
                communication_judge = 0;
                comm_timeout = 0;
                first_entry = 1;   // 下次进入case 1时再次清空FIFO
                fifo_clear(&uart_data_fifo);
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
   uart_get.car_x_v = 0;
   uart_get.car_y_v = 0;
   uart_get.uav_spot = 0;
   uart_get.car_a = 0;
   uart_get.car_b = 0;
   uart_get.car_c = 0;
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
