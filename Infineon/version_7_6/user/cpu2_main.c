/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu2_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu2_dsram"
#include "guandao.h"
#include "zf_device_lora3a22.h"
extern int rtk_flagpoint;
//int mode2=0;
extern int mode;
extern int flagpoint;
//extern float angle5;
extern float error_of_CameraOrBalance;
extern float speed_alone;
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
void core2_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {


        if (lora3a22_state_flag == 1)
        {
            if (lora3a22_finsh_flag == 1)
            {

//                  printf ("head = %d\r\n",lora3a22_uart_transfer.head);
                //lora3a22 帧头

//                  printf ("sum_check = %d\r\n",lora3a22_uart_transfer.sum_check);
                //lora3a22 和校验

//                  printf ("key0 = %d\r\n",lora3a22_uart_transfer.key[0]);
                //左边摇杆按键
                if(lora3a22_uart_transfer.key[0]==0)
                {
                mode=3;//左按键按下停车
                }

//                  printf ("key1 = %d\r\n",lora3a22_uart_transfer.key[1]);
                //右边摇杆按键
                if(!lora3a22_uart_transfer.key[1])
                {
                    flagpoint=1;
                    rtk_flagpoint=1;

                    system_delay_ms(250);

//                    while(!lora3a22_uart_transfer.key[1]);
                }
//                  printf ("joystick[0] = %d\r\n",lora3a22_uart_transfer.joystick[0]);
                //左边摇杆左右值
//                  printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);
                //左边摇杆上下值
                if(lora3a22_uart_transfer.joystick[1])
                {
                speed_alone= lora3a22_uart_transfer.joystick[1]/2000.0*250;
                }
                else
                {
                    speed_alone=180;//速度设置位
                }
//                  printf ("joystick[2] = %d\r\n",lora3a22_uart_transfer.joystick[2]);
                //右边摇杆左右值

                error_of_CameraOrBalance=lora3a22_uart_transfer.joystick[2]/2000.0*65;

//                  printf ("joystick[3] = %d\r\n",lora3a22_uart_transfer.joystick[3]);
                //右边摇杆上下值

               lora3a22_finsh_flag = 0;
            }

        }

    }
}

        // 此处编写需要循环执行的代码



#pragma section all restore
