/*
 * wifi_lineless.c
 *
 *  Created on: 2025年4月22日
 *      Author: lenovo
 */

#include "wifi_lineless.h"
#include "headfile.h"

uint8 image_copy[MT9V03X_H][MT9V03X_W];
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

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

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, MT9V03X_H, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);
}

void wifi_disp(void)
{
    for(int32_t i = 0; i < BOUNDARY_NUM; i++)
    {
        xy_x1_boundary[i] = 0;
        xy_x2_boundary[i] = 0;
        xy_x3_boundary[i] = 0;
    }
    for(int32_t i = 0; i < BOUNDARY_NUM; i++)
    {
        xy_y1_boundary[i] = 0;
        xy_y2_boundary[i] = 0;
        xy_y3_boundary[i] = 0;
    }


    if (cross_type == CROSS_IN)
    {
        for(int32_t i = 0; i < pts_far_right_count; i++)
        {
            if(pts_far_right[i][1]>=188)pts_far_right[i][1]=188;
            if(pts_far_right[i+1][1]>=188)pts_far_right[i+1][1]=188;
            if(pts_far_right[i][1]<=0)pts_far_right[i][1]=0;
            if(pts_far_right[i+1][1]<=0)pts_far_right[i+1][1]=0;
            xy_x1_boundary[i] = (uint16)pts_far_right[i][1];
            xy_y1_boundary[i] = (uint16)pts_far_right[i][0];
        }

        for(int32_t i = 0; i < pts_far_left_count; i++)
        {
            xy_x2_boundary[i] = (uint16)pts_far_left[i][1];
            xy_y2_boundary[i] = (uint16)pts_far_left[i][0];
        }
    }
    else{
        for(int32_t i = 0; i < pts_right_count; i++)
        {
            xy_x1_boundary[i] = (uint16)pts_right[i][1];
            xy_y1_boundary[i] = (uint16)pts_right[i][0];
        }

        for(int32_t i = 0; i < pts_left_count; i++)
        {
            xy_x2_boundary[i] = (uint16)pts_left[i][1];
            xy_y2_boundary[i] = (uint16)pts_left[i][0];
        }
    }

    if (track_type == TRACK_LEFT)
    {
        for(int32_t i = 0; i < mid_left_count; i++)
        {
            xy_x3_boundary[i] = (uint16)rptsn[i][1];
            xy_y3_boundary[i] = (uint16)rptsn[i][0];
        }
    }
    else
    {
        for(int32_t i = 0; i < mid_right_count; i++)
        {
            xy_x3_boundary[i] = (uint16)rptsn[i][1];
            xy_y3_boundary[i] = (uint16)rptsn[i][0];
        }
    }
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    seekfree_assistant_camera_send();
}


