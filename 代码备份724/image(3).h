
#include "zf_common_headfile.h"

#define _IMAGE_H_2
#if defined(_IMAGE_H_1)
#define image_h 120// ͼ��߶ȣ����أ�
#define image_w 188// ͼ���ȣ����أ�


#define white_pixel 255// ��ֵͼ��ɫ����ֵ
#define black_pixel 0// ��ֵͼ��ɫ����ֵ

#define bin_jump_num    1// ��ֵ��������ֵ�����أ�
#define border_max  image_w-2 // �߽����ֵ����ֹԽ�磩
#define border_min  1   // �߽���Сֵ����ֹԽ�磩

typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // δ��⵽����״̬
    SINGLE_BRIDGE_ACTIVE       // ��⵽����״̬
} SingleBridgeState;

extern uint8 original_image[image_h][image_w];// ԭʼͼ�񻺳���
extern uint8 bin_image[image_h][image_w];// ��ֵ��ͼ�񻺳���

// �߽����飨x1=��߽磬x2=�����ߣ�x3=�ұ߽磩
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

extern float border;// �߽�ƫ����������ת����ƣ�
extern SingleBridgeState BridgeState;// ����״̬���
extern int jump_position;// �����λ�ñ��
extern int stop_position;// ͣ����λ�ñ��
extern int buzzer;// ������״̬���
extern int string_not;// ����Ч�������
extern int island;
extern int sum_island;
extern int cross_sum;

/*
 * ��������2�õı���
 *
 * */

extern void image_process(void); // ͼ����������

#elif defined(_IMAGE_H_2)
#define image_h 120//ͼ��߶�
#define image_w 188//ͼ����

#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//�����ĵ���
#define border_max  image_w-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ
extern float border ;        // �߽���ֵ,ͼ��ƫ��
extern float border_last ;        // �߽���ֵ,��һ��ͼ��ƫ��
void Get_image(uint8(*mt9v03x_image)[image_w]);
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//ͼ������
extern uint8 Finish_Flag; //������ɱ�ʶλ
extern uint32 image_process_time;   //ͼ����ʱ��
extern uint8 Lost_point_L_scan_line;
extern uint8 Lost_point_R_scan_line;
extern uint8 zebra_crossing_flag;
extern uint8 jump_position_flag;
extern uint8 Crossroad_Flag;
extern int sum_island ;       // ��������
extern int island ;       
extern int cross_sum ;        // ��������
extern uint8 Right_straight_flag; //��ֱ��
extern uint8 Left_straight_flag; //��ֱ��
extern uint8 Lost_left_Flag;
extern uint8 Upper_left_inflection_Flag;
extern float yaw_cicle;
extern uint16 data_stastics_r ;//ͳ���ұ��ҵ���ĸ���
extern uint16 data_stastics_l;//ͳ������ҵ���ĸ���
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
extern uint8 roundabout_Flag;     //Բ������־��0=δ��⵽��1=��⵽��
extern uint8 Exit_loop_X;
extern uint8 Exit_loop_Y;
extern uint8 Exit_loop_Flag;     //��������־��0=δ������1=�ѳ�����
extern uint8 Lower_left_inflection_X ;
extern uint8 Lower_left_inflection_Y ;
extern uint8 Lower_left_inflection_Flag;
extern int last_boundary_change_x,last_boundary_change_y,first_boundary_change_x,first_boundary_change_y;
typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // δ��⵽����״̬
    SINGLE_BRIDGE_ACTIVE       // ��⵽����״̬
} SingleBridgeState;
extern SingleBridgeState BridgeState;// ����״̬���
extern int bridge_out_flag;
extern int bridge_in_flag;
extern int bridge_in_flag_num;
extern int bridge_out_flag_num;
extern int bridge_entry_cooldown;
extern int white_line1;  // ��75�а�ɫ���ؼ���
extern int white_line2;  // ��70�а�ɫ���ؼ���
extern int left_pattern_found;  // ����-��-��ģʽ
extern int right_pattern_found; // �Ҳ��-��-��-��-��ģʽ
extern float k11,k22,k33,k44,k55,k66,k77,k88;
extern uint8 Lost_right_Flag;
extern int threshold_adjust;

extern void image_process(void);
void image_draw_rectan(uint8(*image)[image_w]);
void image_filter(uint8(*imag)[image_w]);//��̬ѧ�˲������ͺ͸�ʴ��˼��    ���ڱ�ʾͼ������
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
extern uint8 annulus_R_memory;     //��Բ���Ʋ�        ���ڴ洢�һ�������ļ���״̬
extern int blake_line;
extern uint8 Crossroad_memory;
extern int loss_1;       // �߽��ȱ仯������
extern int loss_2;       // �߽�λ��ͻ�������
extern int bridge_number;

#endif /*_IMAGE_H*/
extern float turn_value;
void get_turn_value(float kp,float kp2,float kd,float gkd);// ����ת��ֵ