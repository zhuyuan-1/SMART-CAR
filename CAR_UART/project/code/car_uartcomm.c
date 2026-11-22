#include "car_uartcomm.h"
uart_comm uart_get;                                                             // 位置数据读取结构体
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

// 超时计数需要设为静态，避免每次调用重置
static uint32 comm_timeout_counter = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信主机状态机函数
// 参数说明       void
// 返回参数       void
// 使用示例       car_uart_get();
//-------------------------------------------------------------------------------------------------------------------

void car_uart_get(void)
{
    uint8 date_judge = 1;
    uint32 len = 9;                           // 数据包长度标志位
    
    // 超时逻辑修正：静态变量持续计数，达到阈值重置
    comm_timeout_counter++;
    if(comm_timeout_counter > 1000)            // 超时计数（需根据主循环频率调整）
    {
        comm_timeout_counter = 0;
        date_judge = 1;
        communication_judge = 0;               // 超时后重置状态机
    }

    fifo_data_count = fifo_used(&uart_data_fifo);  // 查看 fifo 是否有数据
    
    switch(communication_judge)
    {
        case 0: {    // 建立单片机与单片机之间的串口通信
            if(date_judge)  // 只在需要时发送握手包
            {
                uint8 a[2] = {0xCC, 0xDD};//双机通信通用串口包
                uart_write_buffer(UART_INDEX, a, 2);
                date_judge = 0;
            }
            
            if(fifo_data_count != 0)            // 读取到数据了
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);
                
                // 检查握手应答（增加边界检查）
                if(fifo_data_count >= 2 && fifo_get_data[0] == 0xCC && fifo_get_data[1] == 0xDD)
                {
                    communication_judge = 1;
                    comm_timeout_counter = 0;   // 收到应答重置超时计数
                }
            }
        } break;
        
        case 1: {
            fifo_data_count = fifo_used(&uart_data_fifo);
            
            if(fifo_data_count >= len)          // 确保数据长度足够
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &len, FIFO_READ_AND_CLEAN);
                
                // 检查帧头帧尾
                if(fifo_get_data[0] == 0x2E && fifo_get_data[8] == 0x1F)
                {
                     uart_get.car_direction = fifo_get_data[3];
                     uart_get.car_x = fifo_get_data[1];
                     uart_get.car_y = fifo_get_data[2];
                     uart_get.coom_timespan = (uint16)((fifo_get_data[6] << 8) | fifo_get_data[7]);
                     uart_get.signs_x = fifo_get_data[4];
                     uart_get.signs_y = fifo_get_data[5];
                     printf("%d,%d\r\n",uart_get.car_x,uart_get.car_y);
                     comm_timeout_counter = 0;   // 收到数据重置超时计数
                }
            }
        } break;
        
        default: break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口通信初始化函数
// 参数说明       void
// 返回参数       void
// 使用示例       communication_init();
//-------------------------------------------------------------------------------------------------------------------
void car_uart_init(void)
 {
   fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 128);              // 初始化 fifo 挂载缓冲区
   uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
   uart_rx_interrupt(UART_INDEX, 1);                                           // 开启 UART_INDEX 的接收中断
   fifo_clear (&uart_data_fifo);
   uart_get.car_direction = 0;
   uart_get.car_x = 0;
   uart_get.car_y = 0;
   uart_get.coom_timespan = 0;
   uart_get.signs_x = 0;
   uart_get.signs_y = 0;
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
void car_uart_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    if(uart_query_byte(UART_INDEX, &get_data))                                  // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    {
        fifo_write_element(&uart_data_fifo, get_data);                   // 将数据写入 fifo
    }
}
