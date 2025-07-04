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
    ips200_show_uint(180,base_y*16, value, num_digits); // 假设x坐标为0，你可以根据需要调整
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
        key_pit_flag = 0;
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
        key_pit_flag = 0;
        ips200_full(RGB565_BLACK);
        value_index = 0;
        place_index = 0;
        allow_value_show = 0;
        allow_image_show = 0;
        system_delay_ms(700);
        mode=walk;
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
    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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
            pid2_flexible.jiaodu_left.kp+=1;
            pid2_flexible.jiaodu_right.kp+=1;
        }
        if(value_index <= -1){
            value_index = 0;
            pid2_flexible.jiaodu_left.kp-=1;
            pid2_flexible.jiaodu_right.kp-=1;
        }
    }
    else if (place_index == 2 ) {
        if(value_index >= 1){
            value_index = 0;
            pid2_flexible.jiaodu_left.ki+=1;
            pid2_flexible.jiaodu_right.ki+=1;
        }
        if(value_index <= -1){
            pid2_flexible.jiaodu_left.ki-=1;
            pid2_flexible.jiaodu_right.ki-=1;
            value_index = 0;
        }
    }
    else if (place_index == 3 ) {
        if(value_index >= 1){
        pid2_flexible.run_speed+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        pid2_flexible.run_speed-=10;
        value_index = 0;
        }
    }
    else if (place_index == 4 ) {
        if(value_index >= 1){
        pid2_flexible.jiaodu_left.kd+=10;
        pid2_flexible.jiaodu_right.kd+=10;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.jiaodu_left.kd-=10;
        pid2_flexible.jiaodu_right.kd-=10;
        }
    }
    else if (place_index == 5 ) {
        if(value_index >= 1){
        pid2_flexible.sudu_left.kp+=0.01;
        pid2_flexible.sudu_right.kp+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.sudu_left.kp-=0.01;
        pid2_flexible.sudu_right.kp-=0.01;
        }
    }
    else if (place_index == 6 ) {
        if(value_index >= 1){
        pid2_flexible.sudu_left.ki+=0.001;
        pid2_flexible.sudu_right.ki+=0.001;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.sudu_left.ki-=0.001;
        pid2_flexible.sudu_right.ki-=0.001;
        }
    }
    else if (place_index == 7 ) {
        if(value_index >= 1){
        pid2_flexible.sudu_left.kd+=0.01;
        pid2_flexible.sudu_right.kd+=0.01;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.sudu_left.kd-=0.01;
        pid2_flexible.sudu_right.kd-=0.01;
        }
    }
    else if (place_index == 8 ) {
        if(value_index >= 1){
        pid2_flexible.zhuanxiang.maxOutput=1500;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.zhuanxiang.maxOutput=0;
        }
    }
    else if (place_index == 9 ) {
        if(value_index >= 1){
        pid2_flexible.zhuanxiang.kp+=1;
        pid2_flexible.zhuanxiang.kp+=1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.zhuanxiang.kp-=1;
        pid2_flexible.zhuanxiang.kp-=1;
        }
    }
    else if (place_index == 10 ) {
        if(value_index >= 1){
        pid2_flexible.zhuanxiang.ki+=0.001;
        pid2_flexible.zhuanxiang.ki+=0.001;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.zhuanxiang.ki-=0.001;
        pid2_flexible.zhuanxiang.ki-=0.001;
        }
    }
    else if (place_index == 11 ) {
        if(value_index >= 1){
        pid2_flexible.zhuanxiang.kd+=0.1;
        pid2_flexible.zhuanxiang.kd+=0.1;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.zhuanxiang.kd-=0.1;
        pid2_flexible.zhuanxiang.kd-=0.1;
        }
    }
    else if (place_index == 12 ) {
        if(value_index >= 1){
        pid2_flexible.zhuanxiang.maxIntegral+=100;
        pid2_flexible.zhuanxiang.maxIntegral+=100;
        value_index = 0;
        }
        if(value_index <= -1)
        {
        value_index = 0;
        pid2_flexible.zhuanxiang.maxIntegral-=100;
        pid2_flexible.zhuanxiang.maxIntegral-=100;
        }
    }
    else if (place_index == 13 ) {
        if(value_index >= 1){
        value_index = 0;
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
    cursor_selected(value_number+1);
    ips200_show_string(0, 0*Font_size_H, " -=  Control  =- ");
    show_string_value_float(1,-imu660ra_gyro_z,3,3,"-imu660ra_gyro_z");
    show_string_value_float(2,pid2_flexible.jiaodu_left.kp,3,3,"jiaodu.kp");
    show_string_value_float(3,pid2_flexible.jiaodu_left.ki,3,3,"jiaodu.ki");
    show_string_value(4,pid2_flexible.run_speed,3,"pid2_flexible.run_speed");
    show_string_value_float(5,pid2_flexible.jiaodu_left.kd,3,3,"jiaodu.kd");
    show_string_value_float(6,pid2_flexible.sudu_left.kp,3,3,"sudu.kp");
    show_string_value_float(7,pid2_flexible.sudu_left.ki,3,3,"sudu.ki");
    show_string_value_float(8,pid2_flexible.sudu_left.kd,3,3,"sudu.kd");
    show_string_value_float(9,pid2_flexible.zhuanxiang.maxOutput,3,3,"zhuanxiang.max");
    show_string_value_float(10,pid2_flexible.zhuanxiang.kp,3,3,"zhuanxiang.kp");
    show_string_value_float(11,pid2_flexible.zhuanxiang.ki,3,3,"zhuanxiang.ki");
    show_string_value_float(12,pid2_flexible.zhuanxiang.kd,3,3,"zhuanxiang.kd");
    show_string_value_float(13,pid2_flexible.zhuanxiang.maxIntegral,3,3,"zhuanxiang.maxIn");
    show_string_value_float(14,pid2_flexible.zhuanxiang.integral,3,3,"zhuanxiang.integral");


    ips200_show_string(20,(value_number+1)*Font_size_H, "EXIT");
    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
           IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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

    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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
    show_string_value(10,BridgeState,3,"BridgeState");
    show_string_value(11,loss_1,3,"loss_1");
    show_string_value(12,loss_2,3,"loss_2");
    show_string_value(13,bridge_number,3,"bridge_number");
    show_string_value(14,right[30]-left[30],3,"right[30]-left[30]");
    show_string_value(15,right[50]-left[50],3,"right[50]-left[50]");
    show_string_value(16,right[70]-left[70],3,"right[70]-left[70]");
    show_string_value(17,right[100]-left[100],3,"right[100]-left[100]");
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

    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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
    show_string_value(9 ,0,3,"XXXXXXXX");
    show_string_value(10,0,3,"XXXXXXXX");
    show_string_value(11,0,3,"XXXXXXXX");
    show_string_value(12,0,3,"XXXXXXXX");
    show_string_value(13,0,3,"XXXXXXXX");
    show_string_value(14,0,3,"XXXXXXXX");
    show_string_value(15,0,3,"XXXXXXXX");
    show_string_value(16,0,3,"XXXXXXXX");
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

    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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

    cursor_selected_image(value_number+1);
    ips200_show_string(0, 8*Font_size_H, " -=  Image4 =- ");
    show_string_value(9 ,0,3,"XXXXXXXX");
    show_string_value(10,0,3,"XXXXXXXX");
    show_string_value(11,0,3,"XXXXXXXX");
    show_string_value(12,0,3,"XXXXXXXX");
    show_string_value(13,0,3,"XXXXXXXX");
    show_string_value(14,0,3,"XXXXXXXX");
    show_string_value(15,0,3,"XXXXXXXX");
    show_string_value(16,0,3,"XXXXXXXX");
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

    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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
    show_string_value(9 ,0,3,"XXXXXXXX");
    show_string_value(10,0,3,"XXXXXXXX");
    show_string_value(11,0,3,"XXXXXXXX");
    show_string_value(12,0,3,"XXXXXXXX");
    show_string_value(13,0,3,"XXXXXXXX");
    show_string_value(14,0,3,"XXXXXXXX");
    show_string_value(15,0,3,"XXXXXXXX");
    show_string_value(16,0,3,"XXXXXXXX");
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

    if(IfxCpu_acquireMutex(&tu))//双核传输
    {
            IfxCpu_releaseMutex(&tu);
    }
    system_delay_ms(5);
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



