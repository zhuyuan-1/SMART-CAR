#include "wifi.h"

//WiFi初始化
void wifi_init(void)
{
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

}
//TCP连接
void TCP_init(void)
{
    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(0 == WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
     //手动连接,自动去底层改
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            WIFI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
            WIFI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                               // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }
}

// 逐飞助手初始化
//1.串口
//2.高速spi_wifi
void seekfree_init(int judge)
{
    if(judge == 1)// 设置逐飞助手使用DEBUG串口进行收发
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    else if(judge == 2)
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

   // 如果要发送图像信息，则务必调用seekfree_assistant_camera_information_config函数进行必要的参数设置
   // 如果需要发送边线则还需调用seekfree_assistant_camera_boundary_config函数设置边线的信息
}
//逐飞助手调参设置
//将8个通道值初始化
void assistant_init(float a[])
{
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        seekfree_assistant_parameter[i] = a[i];
    }
}
//wifi串口发送字符
void wifi_send(char a[])
{
    wifi_spi_send_string((const char *)a);
}
//wifi串口接收
void wifi_get(uint8 wifi_spi_get_data_buffer[])
{
    wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
}
//wifi串口接受字符并返回输出
void wifi_sendreturn(uint8 wifi_spi_get_data_buffer[])
{
    uint32 data_length;
    data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
    if(data_length)                                                     // 如果接收到数据 则进行数据类型判断
    {
        if(!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
        {
            memset(wifi_spi_get_data_buffer, 0, data_length);           // 数据发送完成 清空数据
        }
        else
        {
            printf("\r\n %ld bytes data send failed.", data_length);
        }
    }
}

//逐飞助手波形设置
void seekfree_oscilloscope_send(int seekfree_data, float a[])
{
    // 写入需要发送的数据，有几个通道就写多少个数据
    // 这里仅写入4个通道数据
if(seekfree_data == 1)
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
else if(2 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
}
else if(3 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
}
else if(4 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
    seekfree_assistant_oscilloscope_data.data[3] = a[3];
}
else if(5 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
    seekfree_assistant_oscilloscope_data.data[3] = a[3];
    seekfree_assistant_oscilloscope_data.data[4] = a[4];
}
else if(6 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
    seekfree_assistant_oscilloscope_data.data[3] = a[3];
    seekfree_assistant_oscilloscope_data.data[4] = a[4];
    seekfree_assistant_oscilloscope_data.data[5] = a[5];
}
else if(7 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
    seekfree_assistant_oscilloscope_data.data[3] = a[3];
    seekfree_assistant_oscilloscope_data.data[4] = a[4];
    seekfree_assistant_oscilloscope_data.data[5] = a[5];
    seekfree_assistant_oscilloscope_data.data[6] = a[6];
}
else if(8 == seekfree_data)
{
    seekfree_assistant_oscilloscope_data.data[0] = a[0];
    seekfree_assistant_oscilloscope_data.data[1] = a[1];
    seekfree_assistant_oscilloscope_data.data[2] = a[2];
    seekfree_assistant_oscilloscope_data.data[3] = a[3];
    seekfree_assistant_oscilloscope_data.data[4] = a[4];
    seekfree_assistant_oscilloscope_data.data[5] = a[5];
    seekfree_assistant_oscilloscope_data.data[6] = a[6];
    seekfree_assistant_oscilloscope_data.data[7] = a[7];
}
    // 设置本次需要发送几个通道的数据
    seekfree_assistant_oscilloscope_data.channel_num = (uint8)seekfree_data;

    // 这里进发送了4个通道的数据，最大支持8通道
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);

    system_delay_ms(20);
    // 有可能会在逐飞助手软件上看到波形更新不够连续，这是因为使用WIFI有不确定的延迟导致的

    // 解析上位机发送过来的参数，解析后数据会存放在seekfree_assistant_oscilloscope_data数组中，可以通过在线调试的方式查看数据
    // 例程为了方便因此写在了主循环，实际使用中推荐放到周期中断等位置，需要确保函数能够及时的被调用，调用周期不超过20ms
    seekfree_assistant_data_analysis();

}
//逐飞助手调参设置
void seekfree_assistant_change(int seekfree_data, float a[])
{
    // 滴答客解析接收到的数据
    seekfree_assistant_data_analysis();

    // 遍历,八个通道测算读取
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        // 更新标志位
        if(seekfree_assistant_parameter_update_flag[i])
        {
            seekfree_assistant_parameter_update_flag[i] = 0;
            seekfree_data = (int)i;
            a[i]=seekfree_assistant_parameter[i];
            // 通过DEBBUG串口发送信息
//            printf("receive data channel : %d ", seekfree_data);//输出第几个通道
//            printf("data : %f ", a[i]);//输出通道改变数值
//            printf("\r\n");
        }
    }


}

//逐飞助手摄像头发送图像
void seekfree_camera_send(void)
{
    // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
    //memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    //上面一部要有
    // 发送图像
    seekfree_assistant_camera_send();
    // 如果使用UDP协议传输数据则推荐在数据全部发送到模块之后立即调用wifi_spi_udp_send_now()函数，以告知模块立即将收到的数据发送到网络上
    // 如果没有立即调用则模块会在持续2毫秒未收到数据后，将数据发送到网络上
    // 调用wifi_spi_udp_send_now()前传输给模块的数据数量建议不要超过40960字节
    // wifi_spi_udp_send_now();
}
