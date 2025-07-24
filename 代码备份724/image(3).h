
#include "zf_common_headfile.h"

#define _IMAGE_H_2
#if defined(_IMAGE_H_1)
#define image_h 120// 图像高度（像素）
#define image_w 188// 图像宽度（像素）


#define white_pixel 255// 二值图白色像素值
#define black_pixel 0// 二值图黑色像素值

#define bin_jump_num    1// 二值化跳变阈值（像素）
#define border_max  image_w-2 // 边界最大值（防止越界）
#define border_min  1   // 边界最小值（防止越界）

typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // 未检测到单桥状态
    SINGLE_BRIDGE_ACTIVE       // 检测到单桥状态
} SingleBridgeState;

extern uint8 original_image[image_h][image_w];// 原始图像缓冲区
extern uint8 bin_image[image_h][image_w];// 二值化图像缓冲区

// 边界数组（x1=左边界，x2=中心线，x3=右边界）
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

extern float border;// 边界偏移量（用于转向控制）
extern SingleBridgeState BridgeState;// 桥梁状态标记
extern int jump_position;// 跳变点位置标记
extern int stop_position;// 停车点位置标记
extern int buzzer;// 蜂鸣器状态标记
extern int string_not;// 无有效赛道标记
extern int island;
extern int sum_island;
extern int cross_sum;

/*
 * 环岛代码2用的变量
 *
 * */

extern void image_process(void); // 图像主处理函数

#elif defined(_IMAGE_H_2)
#define image_h 120//图像高度
#define image_w 188//图像宽度

#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值
extern float border ;        // 边界阈值,图像偏差
extern float border_last ;        // 边界阈值,上一次图像偏差
void Get_image(uint8(*mt9v03x_image)[image_w]);
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 Finish_Flag; //处理完成标识位
extern uint32 image_process_time;   //图像处理时间
extern uint8 Lost_point_L_scan_line;
extern uint8 Lost_point_R_scan_line;
extern uint8 zebra_crossing_flag;
extern uint8 jump_position_flag;
extern uint8 Crossroad_Flag;
extern int sum_island ;       // 岛屿总数
extern int island ;       
extern int cross_sum ;        // 交叉点计数
extern uint8 Right_straight_flag; //右直线
extern uint8 Left_straight_flag; //左直线
extern uint8 Lost_left_Flag;
extern uint8 Upper_left_inflection_Flag;
extern float yaw_cicle;
extern uint16 data_stastics_r ;//统计右边找到点的个数
extern uint16 data_stastics_l;//统计左边找到点的个数
extern int middle[120];
extern int left[120];
extern int right[120];
extern int Endline;
extern int blake_line;
extern int delayt;
extern int delayt_flag;
extern uint8_t inc, dec, n, inc_y;
extern uint8 roundabout_X;
extern uint8 roundabout_Y;
extern uint8 roundabout_Flag;     //圆环检测标志（0=未检测到，1=检测到）
extern uint8 Exit_loop_X;
extern uint8 Exit_loop_Y;
extern uint8 Exit_loop_Flag;     //出环检测标志（0=未出环，1=已出环）
extern uint8 Lower_left_inflection_X ;
extern uint8 Lower_left_inflection_Y ;
extern uint8 Lower_left_inflection_Flag;
extern int last_boundary_change_x,last_boundary_change_y,first_boundary_change_x,first_boundary_change_y;
typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // 未检测到单桥状态
    SINGLE_BRIDGE_ACTIVE       // 检测到单桥状态
} SingleBridgeState;
extern SingleBridgeState BridgeState;// 桥梁状态标记
extern int bridge_out_flag;
extern int bridge_in_flag;
extern int bridge_in_flag_num;
extern int bridge_out_flag_num;
extern int bridge_entry_cooldown;
extern int white_line1;  // 第75行白色像素计数
extern int white_line2;  // 第70行白色像素计数
extern int left_pattern_found;  // 左侧白-黑-白模式
extern int right_pattern_found; // 右侧白-黑-白-黑-白模式
extern float k11,k22,k33,k44,k55,k66,k77,k88;
extern uint8 Lost_right_Flag;
extern int threshold_adjust;

extern void image_process(void);
void image_draw_rectan(uint8(*image)[image_w]);
void image_filter(uint8(*imag)[image_w]);//形态学滤波，膨胀和腐蚀的思想    用于表示图像数据
void binaryzation(void);
void IPS_show(void);
float absolute(float z);
void right_straight(void);
int l_loss_judge(uint8 *l_border,uint8 start ,uint8 end);
int r_loss_judge(uint8 *r_border,uint8 start ,uint8 end);
void Addingline1( uint8 choice, uint8 startX, uint8 startY);
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY);
void Addingline2( uint8 choice, uint8 startX, uint8 startY);
void Element_recognition(void);
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);
int Judgment_symbol(float x, float y);
void Lower_left(void);
void Lower_right(void);
void Upper_left(void);
void Upper_right(void);
void zebra_crossing(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border);
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r);
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line, uint8* hightest);
void leg_height_balance_control_wheel_speed(void);
// void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
//                  uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
//                  uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r);
//void wheel_obstacle_height_control(void);
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept);
int check_border_validity(uint8 *l_border, uint8 *r_border);
uint8_t RoundaboutGetArc(uint8_t status, uint8_t num);
void Exit_loop_L_inflection(void);
int check_border_white_excess(void);
extern int  middle[120];
extern uint8 imag[120][188];
extern uint8 imag_copy[120][188];
extern uint8_t imageOut[2][image_w];
extern uint8 threshold_value;
extern int annulus_L_memory_flag;
extern uint8 annulus_L_memory;
extern uint8 annulus_R_memory;     //右圆环计步        用于存储右环形区域的记忆状态
extern int blake_line;
extern uint8 Crossroad_memory;
extern int loss_1;       // 边界宽度变化计数器
extern int loss_2;       // 边界位置突变计数器
extern int bridge_number;

#endif /*_IMAGE_H*/
extern float turn_value;
void get_turn_value(float kp,float kp2,float kd,float gkd);// 计算转角值