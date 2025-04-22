#include "zf_common_headfile.h"
void show_pid2_flexible(void){
    ips200_show_string              (0, 0, "jiaodu");
    ips200_show_float               (120, 0, pid2_flexible.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "suduleft");
    ips200_show_float               (120, 40, pid2_flexible.sudu_left.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    ips200_show_string              (0, 80, "jiaosuduoutl");
    ips200_show_float               (120, 80, pid2_flexible.jiaosudu_left.output, 4, 4);
    ips200_show_string              (0, 100, "jiaosudu");
    ips200_show_int                 (120, 100, imu660ra_gyro_x, 4);
    ips200_show_string              (0, 120, "imu660ra_gyro_z");
    ips200_show_int                 (120, 120, imu660ra_gyro_z, 4);
    ips200_show_string              (0, 140, "jiaosuduoutr");
    ips200_show_float               (120, 140, pid2_flexible.jiaosudu_right.output, 4, 4);
}
void show_pid3_jump(void){
    ips200_show_string              (0, 0, "jiaodu");
    ips200_show_float               (120, 0, pid3_jump.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "suduleft");
    ips200_show_float               (120, 40, pid3_jump.sudu_left.output, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    ips200_show_string              (0, 80, "jiaosuduoutl");
    ips200_show_float               (120, 80, pid3_jump.jiaosudu_left.output, 4, 4);
    ips200_show_string              (0, 100, "jiaosudu");
    ips200_show_int                 (120, 100, imu660ra_gyro_x, 4);
    ips200_show_string              (0, 120, "imu660ra_gyro_z");
    ips200_show_int                 (120, 120, imu660ra_gyro_z, 4);
    ips200_show_string              (0, 140, "jiaosuduoutr");
    ips200_show_float               (120, 140, pid3_jump.jiaosudu_right.output, 4, 4);
}
void show_pid0_stand_c(void){
    ips200_show_string              (0, 0, "jiaodu_output");
    ips200_show_float               (120, 0, pid0_stand_c.jiaodu_left.output, 4, 4);
    ips200_show_string              (0, 20, "left_speed");
    ips200_show_int                 (120, 20, motor_value.receive_left_speed_data, 4);
    ips200_show_string              (0, 40, "suduleft_output");
    ips200_show_float               (120, 40, pid0_stand_c.sudu_left.output, 4, 4);
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
    ips200_show_string              (0, 40, "sudu_left");
    ips200_show_float               (120, 40, pid1_walk.sudu_left.output, 4, 4);
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
    ips200_show_string              (0, 0, "ServoPID.highright");
    ips200_show_float               (120, 0, ServoPID.highright, 4, 4);
    ips200_show_string              (0, 20, "ServoPID.angle_roll");
    ips200_show_float                 (120, 20, ServoPID.angle_roll, 4,4);
    ips200_show_string              (0, 40, "ServoPID.pwm_ph4");
    ips200_show_float               (120, 40, ServoPID.pwm_ph4, 4, 4);
    ips200_show_string              (0, 60, "angle_roll");
    ips200_show_float               (120, 60, angle_roll, 4, 4);
    ips200_show_string              (0, 80, "ServoPID.pwm_ph1");
    ips200_show_float               (120, 80,ServoPID.pwm_ph1, 4, 4);
    ips200_show_string              (0, 100, "ServoPID.highleft");
    ips200_show_int                 (120, 100, ServoPID.highleft, 4);
    ips200_show_string              (0, 120, "ServoPID._pitch");
    ips200_show_float                 (120, 120, ServoPID.angle_pitch, 4,4);
    ips200_show_string              (0, 140, "angle_pitch");
    ips200_show_float               (120, 140, ServoPID.angle_pitch, 4, 4);
}

void show_pid_information(void){
    switch(mode){
        case flexible:show_pid2_flexible();break;
        case stand_c:show_pid0_stand_c();break;
        case walk:show_pid1_walk();break;
        case stopworking:show_stopworking();break;
        case jump:show_pid3_jump();break;
    }
}
