#include "zf_common_headfile.h"
void show_pid0_stand_b(void){
    ips200_show_string              (0, 0, "jiaodu");
    ips200_show_float               (120, 0, pid0_stand_b.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "sudu");
    ips200_show_float               (120, 40, pid0_stand_b.sudu.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
}
void show_pid0_stand_c(void){
    ips200_show_string              (0, 0, "jiaodu_output");
    ips200_show_float               (120, 0, pid0_stand_c.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "sudu_output");
    ips200_show_float               (120, 40, pid0_stand_c.sudu.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    ips200_show_string              (0, 80, "jiaosudu_output");
    ips200_show_float               (120, 80, pid0_stand_c.jiaosudu_left.output, 4, 4);
    ips200_show_string              (0, 100, "jiaosudu");
    ips200_show_int                 (120, 100, imu660ra_gyro_x, 4);
}
void show_pid1_walk(void){
    ips200_show_string              (0, 0, "jiaodu");
    ips200_show_float               (120, 0, pid1_walk.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "sudu");
    ips200_show_float               (120, 40, pid1_walk.sudu.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    ips200_show_string              (0, 80, "jiaosuduoutl");
    ips200_show_float               (120, 80, pid1_walk.jiaosudu_left.output, 4, 4);
    ips200_show_string              (0, 100, "jiaosudu");
    ips200_show_int                 (120, 100, imu660ra_gyro_x, 4);
    ips200_show_string              (0, 120, "imu660ra_gyro_z");
    ips200_show_int                 (120, 120, imu660ra_gyro_z, 4);
    ips200_show_string              (0, 140, "jiaosuduoutr");
    ips200_show_float               (120, 140, pid1_walk.jiaosudu_right.output, 4, 4);
}
void show_stopworking(void){
    ips200_show_string              (0, 0, "ServoPID.high");
    ips200_show_float               (120, 0, ServoPID.high, 4, 4);
    ips200_show_string              (0, 20, "ServoPID.angle");
    ips200_show_int                 (120, 20, ServoPID.angle, 4);
    //ips200_show_string              (0, 40, "sudu");
    //ips200_show_float               (120, 40, pid1_walk.sudu.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    //ips200_show_string              (0, 80, "jiaosuduoutl");
    //ips200_show_float               (120, 80, pid1_walk.jiaosudu_left.output, 4, 4);
    //ips200_show_string              (0, 100, "jiaosudu");
    //ips200_show_int                 (120, 100, imu660ra_gyro_x, 4);
    //ips200_show_string              (0, 120, "imu660ra_gyro_z");
    //ips200_show_int                 (120, 120, imu660ra_gyro_z, 4);
    //ips200_show_string              (0, 140, "jiaosuduoutr");
    //ips200_show_float               (120, 140, pid1_walk.jiaosudu_right.output, 4, 4);
}

void show_pid_information(void){
    switch(mode){
        case stand_b:show_pid0_stand_b();break;
        case stand_c:show_pid0_stand_c();break;
        case walk:show_pid1_walk();break;
        case stopworking:show_stopworking();break;
    }
}
