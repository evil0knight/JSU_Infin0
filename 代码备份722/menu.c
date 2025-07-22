/*
 * menu.c
 *
 *  Created on: 2024年8月18日
 *      Author: shuyu
 */
#include "zf_common_headfile.h"

IfxCpu_mutexLock tu;

int place_index = 0 ;
int value_index = 0;
int last_place_index = 0;
int allow_value_show = 0;
int allow_image_show = 0;
int key_pit_flag = 1;
extern delayt;

/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////
/////////////////////////Menu/////////////////////

void cursor_selected(int max_index) {
    if (place_index >= max_index) {
        place_index = 0;
    } else if (place_index <= -1) {
        place_index = max_index - 1;
    }

    for (int index = 0; index < 20; index++) {
        if (index != place_index + 1) {
            // 假设"  "表示两个空格
            ips200_show_string(0, index * Font_size_H, "  ");
        }
    }
    // 显示箭头"->"，这里假设箭头字符在字体中已定义，或需要替换为实际字符代码
    ips200_show_string(0, (place_index + 1) * Font_size_H, "->");
    last_place_index = place_index;
}
void cursor_selected_image(int max_index) {
    if (place_index >= max_index) {
        place_index = 8;
    } else if (place_index <= 7) {
        place_index = max_index - 1;
    }

    for (int index = 8; index < 20; index++) {
        if (index != place_index + 1) {
            // 假设"  "表示两个空格
            ips200_show_string(0, index * Font_size_H, "  ");
        }
    }
    // 显示箭头"->"，这里假设箭头字符在字体中已定义，或需要替换为实际字符代码
    ips200_show_string(0, (place_index + 1) * Font_size_H, "->");
    last_place_index = place_index;
}


void show_string_value(uint16 base_y, uint32 value, uint8 num_digits, const char* str) {
    ips200_show_string(20, base_y*16, str);
    ips200_show_int(180,base_y*16, value, num_digits); // 假设x坐标为0，你可以根据需要调整
}

void show_string_value_float(uint16 base_y, float value,uint8 num, uint8 pointnum, const char* str) {
    ips200_show_string(20, base_y*16, str);
    ips200_show_float(150,base_y*16, value, num, pointnum); // 假设x坐标为0，你可以根据需要调整
}

void adjust_menu(void)
{
    //Read_FLASH();
    pit_ms_init(CCU61_CH1, 115);
    int value_number = 4;
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 3){
        value_index = 0;
        control_menu();
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 3){
            value_index = 0;
            image_menu();
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 3){
            value_index = 0;
            function_menu();

        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 3){
        //key_pit_flag = 0;
        ips200_clear();
        value_index = 0;
        place_index = 0;
        allow_value_show = 1;
        allow_image_show = 1;
        //Write_FLASH();
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3){
        // value_index = 0;
        // place_index = 0;
        // system_delay_ms(700);
        // mode_switch(walk);
        // //ServoPID.highleft=7.6;
        // //ServoPID.highright=7.6;//3.3
        // system_delay_ms(700);
        // annulus_L_memory=0;
        // annulus_R_memory=0;
        key_pit_flag = 0;
        ips200_full(RGB565_BLACK);
        value_index = 0;
        place_index = 0;
        allow_value_show = 0;
        allow_image_show = 0;
        system_delay_ms(700);
        mode_switch(walk);
        system_delay_ms(700);
        annulus_L_memory=0;
        annulus_R_memory=0;
        //Write_FLASH();
        break;
        }
    }

    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Menu  =- ");
    ips200_show_string(20, 1*Font_size_H, "Control");
    ips200_show_string(20, 2*Font_size_H, "image");
    ips200_show_string(20, 3*Font_size_H, "Function");
    ips200_show_string(20, 4*Font_size_H, "observer");
    ips200_show_string(20, (value_number+1)*Font_size_H, "go");
    ips200_show_string(20,121+16*9,"IP:");
    ips200_show_string(40,121+16*9,wifi_spi_ip_addr_port);
    }
}


void control_menu_2(void)
{
    int value_number = 14;
    ips200_clear();

    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        pid1_walk.run_speed+=10;
        //SpeedLoop.TempKP+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.run_speed-=10;
        //SpeedLoop.TempKP-=1;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1){
            value_index = 0;
            pid1_walk.sudu_left.maxIntegral+=1;
            pid1_walk.sudu_right.maxIntegral+=1;
        }
        if(value_index <= -1){
            value_index = 0;
            pid1_walk.sudu_left.maxIntegral-=1;
            pid1_walk.sudu_right.maxIntegral-=1;
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 1){
            value_index = 0;
            pid1_walk.jiaodu_left.ki+=0.01;
            pid1_walk.jiaodu_right.ki+=0.01;
            PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
            PID_Clear(&pid1_walk.jiaodu_right);//清空角度环pid的历史数据
        }
        if(value_index <= -1){
            pid1_walk.jiaodu_left.ki-=0.01;
            pid1_walk.jiaodu_right.ki-=0.01;
            value_index = 0;
            PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
            PID_Clear(&pid1_walk.jiaodu_right);//清空角度环pid的历史数据
        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 1){
        pid1_walk.run_speed+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        pid1_walk.run_speed-=10;
        value_index = 0;
        }
    }
    else if (place_index == 4 ) {
        if(value_index >= 1){
        pid1_walk.jiaodu_left.kd+=10;
        pid1_walk.jiaodu_right.kd+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.jiaodu_left.kd-=10;
        pid1_walk.jiaodu_right.kd-=10;
        }
    }
    else if (place_index == 5 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.kp+=0.01;
        pid1_walk.sudu_right.kp+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.kp-=0.01;
        pid1_walk.sudu_right.kp-=0.01;
        }
    }
    else if (place_index == 6 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.ki+=0.0001;
        pid1_walk.sudu_right.ki+=0.0001;
        PID_Clear(&pid1_walk.sudu_left);//清空速度环pid的历史数据
        PID_Clear(&pid1_walk.sudu_right);//清空速度环pid的历史数据
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.ki-=0.001;
        pid1_walk.sudu_right.ki-=0.001;
        PID_Clear(&pid1_walk.sudu_left);//清空速度环pid的历史数据
        PID_Clear(&pid1_walk.sudu_right);//清空速度环pid的历史数据
        }
    }
    else if (place_index == 7 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.kd+=0.01;
        pid1_walk.sudu_right.kd+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.kd-=0.01;
        pid1_walk.sudu_right.kd-=0.01;
        }
    }
    else if (place_index == 8 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.maxOutput=1500;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.maxOutput=0;
        }
    }
    else if (place_index == 9 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.kp+=1;
        pid1_walk.zhuanxiang.kp+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.kp-=1;
        pid1_walk.zhuanxiang.kp-=1;
        }
    }
    else if (place_index == 10 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.ki+=0.001;
        pid1_walk.zhuanxiang.ki+=0.001;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.ki-=0.001;
        pid1_walk.zhuanxiang.ki-=0.001;
        }
    }
    else if (place_index == 11 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.kd+=0.1;
        pid1_walk.zhuanxiang.kd+=0.1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.kd-=0.1;
        pid1_walk.zhuanxiang.kd-=0.1;
        }
    }
    else if (place_index == 12 ) {
        if(value_index >= 1){
        pid2_flexible.mechanical_neutral_point+=0.5;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.mechanical_neutral_point-=0.5;
        }
    }
    else if (place_index == 13 ) {
        if(value_index >= 1){
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }
    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Control  =- ");
    show_string_value_float(1,-imu660ra_gyro_z,3,3,"-imu660ra_gyro_z");
    show_string_value_float(2,pid1_walk.sudu_left.maxIntegral,3,3,"sudu_MaxI");
    //show_string_value_float(3,pid1_walk.sudu_right.v,3,3,"sudu_left");
    show_string_value_float(4,pid1_walk.sudu_left.integral,3,3,"sudu.integral");
    show_string_value_float(5,pid1_walk.sudu_left.output,3,3,"sudu.output");
    show_string_value_float(6,pid1_walk.jiaodu_left.integral,3,3,"jiaodu.integral");
    show_string_value_float(7,pid1_walk.jiaodu_left.output,3,5,"jiaodu.output");
    show_string_value_float(8,pid1_walk.sudu_left.kd,3,3,"sudu.kd");
    show_string_value_float(9,pid1_walk.zhuanxiang.maxOutput,3,3,"zhuanxiang.max");
    show_string_value_float(10,pid1_walk.zhuanxiang.kp,3,3,"zhuanxiang.kp");
    show_string_value_float(11,pid1_walk.zhuanxiang.ki,3,3,"zhuanxiang.ki");
    show_string_value_float(12,pid1_walk.zhuanxiang.kd,3,3,"zhuanxiang.kd");
    show_string_value_float(13,pid2_flexible.mechanical_neutral_point,3,3,"neutral_point");
    show_string_value_float(14,pid1_walk.sudu_left.output,3,3,"pid1_walk.sudu_left.output");


    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    }
}
void control_menu(void)
{
    int value_number = 14;
    ips200_clear();

    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        pid1_walk.run_speed+=10;
        //SpeedLoop.TempKP+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.run_speed-=10;
        //SpeedLoop.TempKP-=1;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1){
            value_index = 0;
            pid1_walk.jiaodu_left.kp+=1;
            pid1_walk.jiaodu_right.kp+=1;
        }
        if(value_index <= -1){
            value_index = 0;
            pid1_walk.jiaodu_left.kp-=1;
            pid1_walk.jiaodu_right.kp-=1;
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 1){
            value_index = 0;
            pid1_walk.jiaodu_left.ki+=0.01;
            pid1_walk.jiaodu_right.ki+=0.01;
            PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
            PID_Clear(&pid1_walk.jiaodu_right);//清空角度环pid的历史数据
        }
        if(value_index <= -1){
            pid1_walk.jiaodu_left.ki-=0.01;
            pid1_walk.jiaodu_right.ki-=0.01;
            value_index = 0;
            PID_Clear(&pid1_walk.jiaodu_left);//清空角度环pid的历史数据
            PID_Clear(&pid1_walk.jiaodu_right);//清空角度环pid的历史数据
        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 1){
        pid1_walk.run_speed+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        pid1_walk.run_speed-=10;
        value_index = 0;
        }
    }
    else if (place_index == 4 ) {
        if(value_index >= 1){
        pid1_walk.jiaodu_left.kd+=10;
        pid1_walk.jiaodu_right.kd+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.jiaodu_left.kd-=10;
        pid1_walk.jiaodu_right.kd-=10;
        }
    }
    else if (place_index == 5 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.kp+=0.01;
        pid1_walk.sudu_right.kp+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.kp-=0.01;
        pid1_walk.sudu_right.kp-=0.01;
        }
    }
    else if (place_index == 6 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.ki+=0.0001;
        pid1_walk.sudu_right.ki+=0.0001;
        PID_Clear(&pid1_walk.sudu_left);//清空速度环pid的历史数据
        PID_Clear(&pid1_walk.sudu_right);//清空速度环pid的历史数据
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.ki-=0.001;
        pid1_walk.sudu_right.ki-=0.001;
        PID_Clear(&pid1_walk.sudu_left);//清空速度环pid的历史数据
        PID_Clear(&pid1_walk.sudu_right);//清空速度环pid的历史数据
        }
    }
    else if (place_index == 7 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.kd+=0.01;
        pid1_walk.sudu_right.kd+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.kd-=0.01;
        pid1_walk.sudu_right.kd-=0.01;
        }
    }
    else if (place_index == 8 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.maxOutput=1500;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.maxOutput=0;
        }
    }
    else if (place_index == 9 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.kp+=1;
        pid1_walk.zhuanxiang.kp+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.kp-=1;
        pid1_walk.zhuanxiang.kp-=1;
        }
    }
    else if (place_index == 10 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.ki+=0.001;
        pid1_walk.zhuanxiang.ki+=0.001;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.ki-=0.001;
        pid1_walk.zhuanxiang.ki-=0.001;
        }
    }
    else if (place_index == 11 ) {
        if(value_index >= 1){
        pid1_walk.zhuanxiang.kd+=0.1;
        pid1_walk.zhuanxiang.kd+=0.1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.zhuanxiang.kd-=0.1;
        pid1_walk.zhuanxiang.kd-=0.1;
        }
    }
    else if (place_index == 12 ) {
        if(value_index >= 1){
        pid1_walk.sudu_left.maxIntegral+=100;
        pid1_walk.sudu_left.maxIntegral+=100;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid1_walk.sudu_left.maxIntegral-=100;
        pid1_walk.sudu_left.maxIntegral-=100;
        }
    }
    else if (place_index == 13 ) {
        if(value_index >= 1){
        value_index = 0;
        control_menu_2();
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }
    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Control  =- ");
    show_string_value_float(1,angle_roll,3,3,"angle_roll");
    show_string_value_float(2,pid1_walk.jiaodu_left.kp,3,3,"jiaodu.kp");
    show_string_value_float(3,pid1_walk.jiaodu_left.ki,3,3,"jiaodu.ki");
    show_string_value(4,pid1_walk.run_speed,3,"pid1_walk.run_speed");
    show_string_value_float(5,pid1_walk.jiaodu_left.kd,3,3,"jiaodu.kd");
    show_string_value_float(6,pid1_walk.sudu_left.kp,3,3,"sudu.kp");
    show_string_value_float(7,pid1_walk.sudu_left.ki,3,5,"sudu.ki");
    show_string_value_float(8,pid1_walk.sudu_left.kd,3,3,"sudu.kd");
    show_string_value_float(9,pid1_walk.sudu_left.maxOutput,3,3,"sudu.max");
    show_string_value_float(10,pid1_walk.zhuanxiang.kp,3,3,"zhuanxiang.kp");
    show_string_value_float(11,pid1_walk.zhuanxiang.ki,3,3,"zhuanxiang.ki");
    show_string_value_float(12,pid1_walk.zhuanxiang.kd,3,3,"zhuanxiang.kd");
    show_string_value_float(13,pid1_walk.sudu_right.output,3,3,"pid1_walk.sudu_right.output");
    show_string_value_float(14,pid1_walk.sudu_left.output,3,3,"pid1_walk.sudu_left.output");


    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    }


}
void image_menu(void)
{
    int value_number = 17;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number-1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        image_menu_2();
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image  =- ");
    show_string_value(9 ,zebra_crossing_flag,3,"zebra_crossing_flag");
    show_string_value(10,jump_position_flag,3,"jump_position_flag");
    show_string_value(11,Left_straight_flag,3,"Left_straight_flag");
    show_string_value(12,Right_straight_flag,3,"Right_straight_flag");
    show_string_value(13,Endline,3,"Endline");
    show_string_value(14,blake_line,3,"blake_line");
    show_string_value(15,(uint32)Crossroad_memory,3,"Crossroad_memory");
    show_string_value(16,(uint32)Crossroad_Flag,3," Crossroad_Flag");
    show_string_value(17,0,3,"nextpage");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    
    int i;
    //image_process();
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
    for (i = Endline; i < image_h-1; i++)
    {
        //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        ips200_draw_point(middle[i], i,  RGB565_GREEN);
        ips200_draw_point(left[i], i, RGB565_RED);
        ips200_draw_point(right[i], i, RGB565_BLUE);

     }
     if(Endline<115){
         for(int i=119;i>Endline;i--)
         {
             //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
             //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
             //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
             if(Endline>1&&Endline<120)
             {
                 ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
             }
          }
     }
    }


}
void image_menu_2(void)
{
    int value_number = 17;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number-7 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        threshold_adjust += 1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        threshold_adjust -= 1;
        }
    }
    else if (place_index == value_number-1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        image_menu_3();
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image2 =- ");
    show_string_value(9 ,annulus_L_memory,3,"annulus_L_memory");
    show_string_value(10,bridge_in_flag,3,"bridge_in_flag");
    show_string_value(11,threshold_adjust,3,"threshold_adjust");
    show_string_value(12,annulus_R_memory,3,"annulus_R_memory");
    show_string_value(13,bridge_number,3,"bridge_number");
    show_string_value(14,left_pattern_found,3,"left_found");
    show_string_value(15,right_pattern_found,3,"right_found");
    show_string_value(16,left[71]-left[70],3,"l[71]-l[70]");
    show_string_value(17,left[100]-left[99],3,"l[100]-l[99]");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    //image_process();
    int i;
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
    for (i = Endline; i < image_h-1; i++)
    {
        //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        ips200_draw_point(middle[i], i,  RGB565_GREEN);
        ips200_draw_point(left[i], i, RGB565_RED);
        ips200_draw_point(right[i], i, RGB565_BLUE);

     }
     if(Endline<115){
         for(int i=119;i>Endline;i--)
         {
             //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
             //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
             //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
             if(Endline>1&&Endline<120)
             {
                 ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
             }
          }
     }

    }


}

void image_menu_3(void)
{
    int value_number = 17;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number-1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        image_menu_4();
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }
    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image3 =- ");
    show_string_value(9 ,sum_island,3,"sum_island");
    show_string_value(10,island,3,"island");
    show_string_value(11,Crossroad_Flag,3,"Crossroad_Flag");
    show_string_value(12,roundabout_X,3,"roundabout_X");
    show_string_value(13,roundabout_Y,3,"roundabout_Y");
    show_string_value(14,Lower_left_inflection_X,3,"Lower_leftX");
    show_string_value(15,Lower_left_inflection_Y,3,"Lower_lefTY");
    show_string_value(16,0,3,"XXXXXXXX");
    show_string_value(17,0,3,"nextpage");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    //image_process();
    int i;
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
    for (i = Endline; i < image_h-1; i++)
    {
        //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        ips200_draw_point(middle[i], i,  RGB565_GREEN);
        ips200_draw_point(left[i], i, RGB565_RED);
        ips200_draw_point(right[i], i, RGB565_BLUE);

     }
     if(Endline<115){
         for(int i=119;i>Endline;i--)
         {
             //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
             //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
             //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
             if(Endline>1&&Endline<120)
             {
                 ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
             }
          }
     }

    }


}
void image_menu_4(void)
{
    int value_number = 17;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number-1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        image_menu_5();
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }
    //lost_left();
    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image4 =- ");
    show_string_value(9 ,Upper_left_inflection_Flag,3,"Upper_left");
    show_string_value(10,Endline,3,"Endline");
    show_string_value(11,inc,3,"inc");
    show_string_value(12,dec,3,"dec");
    show_string_value_float(13,Lost_right_Flag,3,3,"Lost_right_Flag");
    show_string_value_float(14,inc_y,3,3,"inc_y");
    show_string_value_float(15,Lost_left_Flag,3,3,"Lost_left");
    show_string_value_float(16,jump_state,3,3,"jump_state");
    show_string_value(17,0,3,"nextpage");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    int i;
    //RoundaboutGetArc(1, 0, 0);
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
    for (i = Endline; i < image_h-1; i++)
    {
        //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        ips200_draw_point(middle[i], i,  RGB565_GREEN);
        ips200_draw_point(left[i], i, RGB565_RED);
        ips200_draw_point(right[i], i, RGB565_BLUE);

     }
     if(Endline<115){
         for(int i=119;i>Endline;i--)
         {
             //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
             //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
             //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
             if(Endline>1&&Endline<120)
             {
                 ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
             }
          }
     }
    }


}
void image_menu_5(void)
{
    int value_number = 17;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        }
    }
    else if (place_index == value_number-1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        break;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image5 =- ");
    show_string_value_float(9 ,yaw_cicle,3,3,"yaw_cicle");
    show_string_value_float(10,euler_angle.yaw,3,3,"yaw");
    show_string_value_float(11,last_boundary_change_x,3,3,"last_boundaryx");
    show_string_value_float(12,last_boundary_change_y,3,3,"last_boundaryy");
    show_string_value_float(13,first_boundary_change_x,3,3,"first_boundaryx");
    show_string_value_float(14,first_boundary_change_y,3,3,"first_boundaryy");
    show_string_value_float(15,k33,3,3,"k3");
    show_string_value_float(16,k44,3,3,"k4");
    show_string_value(17,0,3,"nextpage");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");

    int i;
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
    for (i = Endline; i < image_h-1; i++)
    {
        //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        ips200_draw_point(middle[i], i,  RGB565_GREEN);
        ips200_draw_point(left[i], i, RGB565_RED);
        ips200_draw_point(right[i], i, RGB565_BLUE);

     }
     if(Endline<115){
         for(int i=119;i>Endline;i--)
         {
             //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //中线
             //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //左控制线
             //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //右控制线
             if(Endline>1&&Endline<120)
             {
                 ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //截止行
             }
          }
     }
    }


}
void function_menu(void)
{
    int value_number = 2;
    ips200_clear();
    while(1)
    {
    if (place_index == 0 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        if(++mode==4)mode=0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        mode=stopworking;
        small_driver_set_duty(0,0);//站立串级pid设置电机转速
        }
    }
    else if (place_index == 1 ) {
        if(value_index >= 1)
        {
        value_index = 0;
        //run_time+=1;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        //run_time-=1;
        }
    }
    else if (place_index == value_number ) {
        if(value_index >= 3)
        {
            ips200_clear();
            value_index = 0;
            place_index = 0;
            break;
        }
    }

    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Function  =- ");
    if(mode==walk){
        ips200_show_string(20, 16, "mode");
        ips200_show_string(160,16, "  walk  ");
    }
    if(mode==stopworking){
        ips200_show_string(20, 16, "mode");
        ips200_show_string(160,16, "stopwork");
    }
    if(mode==stand_c){
        ips200_show_string(20, 16, "mode");
        ips200_show_string(160,16, "stand_c");
    }
    if(mode==flexible){
        ips200_show_string(20, 16, "mode");
        ips200_show_string(160,16, "flexible");
    }
    //show_string_value(1,mode,3,"mode");
    //show_string_value(2,run_time,3,"run_time");
    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
    }


}



