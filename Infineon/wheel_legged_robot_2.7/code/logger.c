/*
 * logger.c
 *
 *  Created on: 2025年4月26日
 *      Author: 17104
 */

#include"logger.h"

int8 logger_send_flag = 1;
int8 logger_save_flag = 0;
uint32 time_stamp = 0;

void logger_init(void)
{
    gpio_init(LOGGER_UART_RTS_PIN, GPO, 1, GPO_PUSH_PULL);      // 初始化流控引脚
    uart_init (LOGGER_UART_INDEX, LOGGER_UART_BUAD_RATE, LOGGER_UART_RX_PIN, LOGGER_UART_TX_PIN);   // 初始化串口
    system_delay_ms(100);
    printf("time_stamp,batter_voltage,actual_max_speed,actual_target_speed,Instantaneous_speed,"
            "est_velocity,is_slip,slip_ratio,roll,pitch,yaw,ax_linear,ay_linear,az_linear,gx,gy,gz,begin_yaw,resultant_acceleration,"
            "standardized_curvature_ave,lateral_deviation,preview_distance,is_supplement_line,rptsn_num,pure_angle,transection_distance,slope_status,horizontal_E,flag,Uq[0],Uq[1],alpha\n");

}//---show---?

void logger_send(void)
{
    if(logger_send_flag)
    {
//        gpio_set_level(LOGGER_UART_RTS_PIN, 1);
//        printf();

        printf("%lu,%f,%f,%f,%f, %f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f,%f,%f,%d,%d,%f,%f,%d, %f,%d,%f,%f,%f\n",
                time_stamp, input.batter_voltage, actual_max_speed, input.actual_target_speed, Instantaneous_speed,
                vel_kf.est_velocity,vel_kf.is_slip,vel_kf.slip_ratio,imu.roll, imu.pitch, imu.yaw,imu.ax_linear,imu.ay_linear,imu.az_linear,now_gx,now_gy,now_gz,begin_yaw,imu.resultant_acceleration,
                standardized_curvature_ave, lateral_deviation, preview_distance,is_supplement_line,rptsn_num,pure_angle,transection_distance,slope_status,
                input.horizontal_E,input.flag, Uq[0], Uq[1], alpha);
        printf("666\n");
        logger_send_flag = 0;
    }
}

void logger_save(void)
{
    if(logger_save_flag)
    {
        printf("!!!saved!!!\n");
        system_delay_ms(200);
        gpio_set_level(LOGGER_UART_RTS_PIN, 0);
        system_delay_ms(200);
        gpio_set_level(LOGGER_UART_RTS_PIN, 1);
        system_delay_ms(200);
        printf("time_stamp,batter_voltage,actual_max_speed,actual_target_speed,Instantaneous_speed,"
                    "est_velocity,is_slip,slip_ratio,roll,pitch,yaw,ax_linear,ay_linear,az_linear,gx,gy,gz,begin_yaw,resultant_acceleration,"
                    "standardized_curvature_ave,lateral_deviation,preview_distance,is_supplement_line,rptsn_num,pure_angle,transection_distance,slope_status,horizontal_E,flag,Uq[0],Uq[1],alpha\n");
        logger_save_flag = 0;
    }
}


