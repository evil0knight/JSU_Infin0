#include "zf_common_headfile.h"


//ISLAND_H_1�����ȿ�Դ�е�Բ������,ISLAND_H_2��Bվ�Ĵ��룬ISLAND_H_3��100Ԫ����
//#define ISLAND_H_3



#if defined(ISLAND_H_1)

island_mode_choice circle_type;
track_type_e track_type;
circle_type = CIRCLE_NONE;
int beel = 0;

int32_t Left_Border_None_Circle  = 0;
int32_t Right_Border_None_Circle = 0;

int32_t Left_Border_Have_Circle  = 0;
int32_t Right_Border_Have_Circle = 0;

int32_t Left_Border_ToLeft_Circle  = 0;
int32_t Right_Border_ToLeft_Circle = 0;

int32_t Left_Border_ToRight_Circle  = 0;
int32_t Right_Border_ToRight_Circle = 0;
/**
* @brief ����������亯��
* @param uint8(*image)[image_w]     �Ҷ�ͼ������
* @param uint8 *l_border            ���Ե����
* @param uint8 *r_border            �ұ�Ե����
* @param uint16 total_num_l         ���Ե������
* @param uint16 total_num_r         �ұ�Ե������
* @param uint16 *dir_l              ���Ե��������
* @param uint16 *dir_r              �ұ�Ե��������
* @param uint16(*points_l)[2]       ���Ե����������
* @param uint16(*points_r)[2]       �ұ�Ե����������
* @param uint8* hightest            �����Ч��ָ��
* @param uint16 *l_index            ���Ե��������
* @param uint16 *r_index            �ұ�Ե��������
* @param int monotonicity_l         ���Ե������
* @param int monotonicity_r         �ұ�Ե������
*/
void CheckCircle(uint8 *l_border, uint8 *r_border,
                 uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r,
                 uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,
                 uint16 *l_index, uint16 *r_index,
                 int monotonicity_l, int monotonicity_r)
{
    // ��ԭ�����еı����滻Ϊ����
    // ԭ����1: circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1
    if (circle_type == CIRCLE_NONE &&
        total_num_l > 0 &&           // Lpt0_found: ���Ե����
        total_num_r == 0 &&          // !Lpt1_found: �ұ�Ե������
        monotonicity_r == 1)         // is_straight1: �ұ�Ե������Ϊ1 (ֱ��)
    {
        circle_type = CIRCLE_LEFT_BEGIN;
        //beel = 500;
    }

    // ԭ����2: circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0
    if (circle_type == CIRCLE_NONE &&
        total_num_l == 0 &&          // !Lpt0_found: ���Ե������
        total_num_r > 0 &&           // Lpt1_found: �ұ�Ե����
        monotonicity_l == 1)         // is_straight0: ���Ե������Ϊ1 (ֱ��)
    {
        circle_type = CIRCLE_RIGHT_BEGIN;
       // beel = 500;
    }
}
void RunCircle(uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
               uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
               uint8* highest, uint16 *l_index, uint16 *r_index,
               int monotonicity_l, int monotonicity_r)
{
    if (circle_type == CIRCLE_LEFT_BEGIN)
    {
        track_type = TRACK_RIGHT;

        // �滻��pts_resample_left_count -> total_num_l
        if (total_num_l < 0.06 / RESAMPLEDIST) {
            Left_Border_None_Circle++;
        }
        if (total_num_l > 0.2 / RESAMPLEDIST && Left_Border_None_Circle > FRAMENONE) {
            Left_Border_Have_Circle++;
            if (Left_Border_Have_Circle > FRAMENONE) {
                circle_type             = CIRCLE_LEFT_IN;
                Left_Border_None_Circle = 0;
                Left_Border_Have_Circle = 0;
            }
        }
    }
    else if (circle_type == CIRCLE_LEFT_IN)
    {
        track_type = TRACK_LEFT;

        // �滻��pts_resample_right -> points_r
        if (points_r[(int32_t)(0.2 / RESAMPLEDIST)][1] - points_r[0][1] < -2) {
            Right_Border_ToLeft_Circle++;
        }
        if (Right_Border_ToLeft_Circle > FRAMETOLEFT) {
            circle_type                = CIRCLE_LEFT_RUNNING;
            Right_Border_ToLeft_Circle = 0;
        }
    }
    else if (circle_type == CIRCLE_LEFT_RUNNING)
    {
        track_type = TRACK_RIGHT;

        // �滻��Lpt1_found -> (r_index �������߼�)
        if (/* r_index ���� */) {
            // �滻��Lpt1_rpts1s_id -> r_index ��Ӧֵ
            total_num_r = mid_right_count = /* r_index ֵ */;
        }
        if (/* r_index ���� */ && /* r_index ֵ */ < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    else if (circle_type == CIRCLE_LEFT_OUT)
    {
        track_type = TRACK_LEFT;

        // �滻��is_straight1 -> monotonicity_r �ж�
        if (monotonicity_r == STRAIGHT) {
            circle_type = CIRCLE_LEFT_END;
        }
    }
    else if (circle_type == CIRCLE_LEFT_END)
    {
        track_type = TRACK_RIGHT;

        // �滻��pts_resample_left_count -> total_num_l
        if (total_num_l < 0.1 / RESAMPLEDIST) {
            Left_Border_None_Circle++;
        }
        if (total_num_l > 0.5 / RESAMPLEDIST && Left_Border_None_Circle > FRAMENONE) {
            circle_type                 = CIRCLE_NONE;
            Left_Border_None_Circle     = 0;
            Left_Border_Have_Circle     = 0;
            Right_Border_ToLeft_Circle  = 0;
            Right_Border_ToRight_Circle = 0;
            //beel = 500;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_BEGIN)
    {
        track_type = TRACK_LEFT;

        // �滻��pts_resample_right_count -> total_num_r
        if (total_num_r < 0.06 / RESAMPLEDIST) {
            Right_Border_None_Circle++;
        }
        if (total_num_r > 0.2 / RESAMPLEDIST && Right_Border_None_Circle > FRAMENONE) {
            Right_Border_Have_Circle++;
            if (Right_Border_Have_Circle > FRAMENONE) {
                circle_type              = CIRCLE_RIGHT_IN;
                Right_Border_None_Circle = 0;
                Right_Border_Have_Circle = 0;
            }
        }
    }
    else if (circle_type == CIRCLE_RIGHT_IN)
    {
        track_type = TRACK_RIGHT;

        // �滻��pts_resample_left -> points_l
        if (points_l[(int32_t)(0.2 / RESAMPLEDIST)][1] - points_l[0][1] > 2) {
            Left_Border_ToRight_Circle++;
        }
        if (Left_Border_ToRight_Circle > FRAMETORIGHT) {
            circle_type                = CIRCLE_RIGHT_RUNNING;
            Left_Border_ToRight_Circle = 0;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_RUNNING)
    {
        track_type = TRACK_RIGHT;

        // �滻��Lpt0_found -> (l_index �������߼�)
        if (/* l_index ���� */) {
            // �滻��Lpt0_rpts0s_id -> l_index ��Ӧֵ
            total_num_l = mid_left_count = /* l_index ֵ */;
        }
        if (/* l_index ���� */ && /* l_index ֵ */ < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_OUT)
    {
        track_type = TRACK_RIGHT;

        // �滻��is_straight0 -> monotonicity_l �ж�
        if (monotonicity_l == STRAIGHT) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_END)
    {
        track_type = TRACK_LEFT;

        // �滻��pts_resample_right_count -> total_num_r
        if (total_num_r < 0.1 / RESAMPLEDIST) {
            Right_Border_None_Circle++;
        }
        if (total_num_r > (0.5 / RESAMPLEDIST) && Right_Border_None_Circle > FRAMENONE) {
            circle_type                = CIRCLE_NONE;
            Right_Border_None_Circle   = 0;
            Right_Border_Have_Circle   = 0;
            Left_Border_ToLeft_Circle  = 0;
            Left_Border_ToRight_Circle = 0;
            //beel = 500;
        }
    }
}
#elif defined(ISLAND_H_2)
/**
* @brief ����������亯��
* @param uint8(*image)[image_w]     �Ҷ�ͼ������
* @param uint8 *l_border            ���Ե����
* @param uint8 *r_border            �ұ�Ե����
* @param uint16 total_num_l         ���Ե������
* @param uint16 total_num_r         �ұ�Ե������
* @param uint16 *dir_l              ���Ե��������
* @param uint16 *dir_r              �ұ�Ե��������
* @param uint16(*points_l)[2]       ���Ե����������
* @param uint16(*points_r)[2]       �ұ�Ե����������
* @param uint8* hightest            �����Ч��ָ��
* @param uint16 *l_index            ���Ե��������
* @param uint16 *r_index            �ұ�Ե��������
* @param int monotonicity_l         ���Ե������
* @param int monotonicity_r         �ұ�Ե�����ԣ�������1����������0
* @return �޷���ֵ������洢��ȫ�ֱ�����
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void around_fill_1(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
    if(/*island==2&&*/monotonicity_r==0&&monotonicity_l==1&&*hightest<15&&(circle_type==CIRCLE_NONE||circle_type==CIRCLE_RIGHT_BEGIN))//��⵽���һ������ұ߳��ֶ���,�ұ��ѵ��ڿ���,��ߵ���
    {
    /*��*/circle_type=CIRCLE_RIGHT_BEGIN;//��ʼԲ��

    /*��*/
    }



}
#elif defined(ISLAND_H_3)
void circle_find(void)
{
//       if(circle_flag)
//         k= -0.16f;
       circle_width_left = circle_width_right = 0;
       // ��Բ���������
       if(L_corner_flag==1&&L_corner_row>5&&L_corner_row<MT9V03X_H-5&&// ��ս���Ч��λ��ͼ���м�����
          abs(zx[L_corner_row]-zx[L_corner_row+2])<=3&&// ���յ㸽�������ԣ���ֵ��3��
          abs(zx[L_corner_row+1]-zx[L_corner_row+3])<=3&&
           abs(zx[L_corner_row+2]-zx[L_corner_row+4])<=3&&
              (zx[L_corner_row]-zx[L_corner_row-2])>=5&&// ���յ�ͻ���ԣ���ֵ��5��
              (zx[L_corner_row]-zx[L_corner_row-3])>=5&&//  -3ƫ�Ƶı�Ե�仯��5
              (zx[L_corner_row]-zx[L_corner_row-4])>=7
              && (yx[R_corner_row]-yx[R_corner_row-2])<=3&&// �Ҳ�յ���ͻ�䣨��ֵ��3��
              (yx[R_corner_row]-yx[R_corner_row-3])<=3&&
              (yx[R_corner_row]-yx[R_corner_row-4])<=3)
       {
           circle_width_left = yx[L_corner_row-10]-zx[L_corner_row-10];// ������Բ�����
           L_circle_corner_row = L_corner_row;// ��¼�յ��к�
           L_circle_corner_flag = 1;;// �����Բ�����ɹ�
       }

       if(R_corner_flag==1&&R_corner_row>5&&R_corner_row<MT9V03X_H-5&&// ��Բ�����
            abs(yx[R_corner_row]-yx[R_corner_row+2])<=2&&//  �ұ�Ե�У�R_corner_row��+2ƫ�Ƶı�Ե�仯��2���ж�ƽ��
            abs(yx[R_corner_row+1]-yx[R_corner_row+3])<=2&&
            abs(yx[R_corner_row+2]-yx[R_corner_row+4])<=2&&
             (yx[R_corner_row]-yx[R_corner_row-2])<=-5&&// �Ҳ�յ�ͻ�䷽���෴����ֵ��
             (yx[R_corner_row]-yx[R_corner_row-3])<=-5&&
             (yx[R_corner_row]-yx[R_corner_row-4])<=-7
            && (zx[L_corner_row]-zx[L_corner_row-2])<=3&&// ���յ���ͻ��
            (zx[L_corner_row]-zx[L_corner_row-3])<=3&&
            (zx[L_corner_row]-zx[L_corner_row-4])<=3)
        {
            circle_width_right = yx[R_corner_row-10]-zx[R_corner_row-10];// ������Բ�����
            R_circle_corner_row = R_corner_row;//  ��¼��Բ�������к�
            R_circle_corner_flag = 1;//  ����Բ������־Ϊ1����⵽������Բ��
        }
        base_width = yx[MT9V03X_H-2]-zx[MT9V03X_H-2];//  ����ͼ��ײ���MT9V03X_H-2�У������ұ�Ե���Ϊ�������
       //������Բ��ģʽ
        if(circle_mode==0&&L_circle_corner_flag==1&&
            Continuity_Change_Right(MT9V03X_H-10,30)==0&&// �Ҳ��������ޱ仯
            Right_TotalLost(MT9V03X_H-30,Finnalline)<2&&// �Ҳඪʧ�߽�����2��
            L_corner_row>MT9V03X_H-80&&// �յ㿿��ͼ��ײ�
            Left_Lost_Time>15)// ��ඪʧʱ��ϳ�
            //&&Monotonicity_Change_Right()==0
            {
                left_circle_cnt++;// �ۼ���Բ����������
            }
        else if(left_circle_cnt!=0)
        {
            left_circle_cnt--;// δ��������ʱ�ݼ����������󴥣�
        }
        if(left_circle_cnt>=2)// ������⵽2��
        {
            left_circle_cnt=0;
            left_circle_flag_cnt++;
            circle_flag = 1;
            circle_mode = 1;//����׶�1����ʼת�䣩
            left_circle_flag = 1;
            buzzer();// ��������ʾ
        }

        //��Բ����������
        if(T_2s_flag==1&&//2�붨ʱ��־���������ڷ����������Լ�⣩
            circle_mode==0&&//��ǰδ�����κ�Բ������׶�
            L_circle_corner_flag==1&&//��Բ���ս��Ѽ�⵽
            Continuity_Change_Right(MT9V03X_H-10,30)==0&&//ͼ��ײ�30�����Ҳ�߽�������ͻ��
            Right_TotalLost(MT9V03X_H-50,Finnalline)<2&&//�Ҳ��ڵײ�50���ڶ�ʧ�߽�����2��
            L_corner_row>MT9V03X_H-80&&//��ս�λ��ͼ��ײ�80���ڣ���������
            circle_width_left>135)//��Բ����ȳ���135���أ�������ֵ��
            //&&Monotonicity_Change_Right()==0
        {
                circle_flag = 1;     // ȫ��Բ����־��λ
                circle_mode = 1;     // ����׶�1����ʼת�䣩
                left_circle_flag = 1; // ��Բ����־��λ
                buzzer();           // ���������죨��ʾ��ʻԱ�����
        }

        //�׶�1����ʼת��
        if(left_circle_flag==1)//  ?
        {
            if(circle_mode==1)
            {
                gpio_toggle_level(BUZZER_PIN);// �л�������״̬����˸��ʾ��

//            buzzer();
                yaw_flag=1; // ���������ǿ���
//            Circle_in_Left();
            if(T1_ms_cricle1_flag==1&&Boundry_Start_Left==MT9V03X_H-2&&Left_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-40)==0&&Right_TotalLost(MT9V03X_H-40,Finnalline)<5)//
            {
                t2_ms_cricle1 = 0;       // �����ʱ��
                T1_ms_cricle1_flag = 0;  // ����׶�1��־
                circle_mode = 2;         // ����׶�2
                now_yaw=0;               // ���������ǽǶ�
            }
        }

        if(circle_mode==2)
        {
            gpio_set_level(BUZZER_PIN, GPIO_LOW);// �رշ�����

            yaw_flag=1;
//            gpio_init(BUZZER_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//            Circle_in_Left();
            if(abs(now_yaw)>=16200)//?   ?   50  //Lin_circleup_row>MT9V03X_H-50&&Right_Lost_Time<=2
            {
//                gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
                t2_ms_cricle2 = 0;
                circle_mode=3;//����׶�3
                now_yaw=0;

            }
        }
        if(circle_mode==3)
        {

            yaw_flag=1;
//            buzzer();
//            buzzer();
//            buzzer();
//            Circle_out_Left();
            // �ǶȽ�һ������
            if(abs(now_yaw)>=22800)//&&Rout_circleup_row>MT9V03X_H-80)//?   ?   200
            {
                t2_ms_cricle3 = 0;
                circle_mode=4;
                now_yaw=0;

            }
        }
        if(circle_mode==4)
        {

            get_upturningleft_point();// ��ȡ��ת�����Ĳο��㣨δʵ�֣�
//            Circle_out_Left();
            yaw_flag=1;
            // �ǶȽӽ����
            if(abs(now_yaw)>=24800)//?   ?   10
            {
                if(Finnalline<=20)// �ǶȽӽ����
                {
                    circle_mode = 5;// ���ձ߽��߽϶̣��ӽ�ֱ����
                    now_yaw=0;

                }
            }
        }
        if(circle_mode==5)
        {
            gpio_toggle_level(BUZZER_PIN);// ��������˸
//            get_upturningleft_point();
//            Circle_out_Left();
//            buzzer();
//            buzzer();
//            buzzer();
//            Circle_in_Left();
            if(T1_ms_cricle5_flag==1&&Left_sideIsNoLost(MT9V03X_H-35,MT9V03X_H-80)==0)//Left_Lost_Time<15&&Lin_circleup_flag!=1&&Lin_circleup_row>MT9V03X_H-40
            {
                gpio_set_level(BUZZER_PIN, GPIO_LOW);// �رշ�����
//                buzzer();
                yaw_flag=0;// ���������ǿ���
                now_yaw=0;
                circle_cnt++; // Բ����������
                T1_ms_cricle5_flag = 0;
                circle_flag=0;// ������б�־
                circle_mode=0;
                left_circle_flag=0;
                buzzer();// �����ʾ��
            }
        }
      }


        if(circle_mode==0&&//ϵͳδ��������Բ��״̬
            R_circle_corner_flag==1&&//��Բ���ս���ͨ��ͼ��ʶ��ȷ��
            Continuity_Change_Left(MT9V03X_H-10,30)==0&&//���߽��ڵײ�30���ڱ仯ƽ������ͻ�䣩
            Left_TotalLost(MT9V03X_H-30,Finnalline)<2&&//���߽綪ʧ��������2�Σ���֤���������ԣ�
            R_corner_row>MT9V03X_H-80&&//�յ�λ��ͼ��ײ�80���ڣ���������
            Right_Lost_Time>15)//&&Monotonicity_Change_Left()==0
        {
            right_circle_cnt++;// ��������ʱ���Ӽ���
        }
        else if(right_circle_cnt!=0)
        {
            right_circle_cnt--;// ����������ʱ�ݼ�����
        }
        if(right_circle_cnt>=3)// �ۼ�3��ȷ��
        {
            right_circle_cnt=0;
            right_circle_flag_cnt++;
            circle_flag = 1;     // ȫ��Բ����־
            circle_mode = 1;     // ����׶�1����ת���룩
            right_circle_flag = 1;// ��Բ��ר����־
//            buzzer();
//            buzzer();
//            buzzer();
        }
        //
        if(T_2s_flag==1&&circle_mode==0&&R_circle_corner_flag==1&&Continuity_Change_Left(MT9V03X_H-10,30)==0
                &&Left_TotalLost(MT9V03X_H-5,Finnalline)<2&&R_corner_row>MT9V03X_H-80&&circle_width_right>135)//&&Monotonicity_Change_Left()==0
        {
            // ǿ�ƽ���Բ��ģʽ������������
            circle_flag = 1;
            circle_mode = 1;//???
            right_circle_flag = 1;
//            buzzer();
//            buzzer();
//            buzzer();
        }
        if(right_circle_flag==1)//  ?
        {
            //�׶�1����ʼ��ת����
            if(circle_mode==1)
            {
                gpio_toggle_level(BUZZER_PIN);// ������״̬��ת
                yaw_flag=1;// ���������ǿ���

    //              Circle_in_Right();
                if(T1_ms_cricle1_flag==1&&
                    Boundry_Start_Right==MT9V03X_H-2&&// �ұ߽��ͼ��ײ���ʼ
                    Right_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-30)==0&&Left_TotalLost(MT9V03X_H-40,Finnalline)<5)//
                {
                    t2_ms_cricle1 = 0;
                    T1_ms_cricle1_flag = 0;
                    circle_mode = 2;//
                    now_yaw=0;
                }
            }
            //�׶�2���ȶ���ת
            if(circle_mode==2)
            {
                yaw_flag=1;

    //              Circle_in_Right();
                if(abs(now_yaw)>=16200)//?   ?   50  //Rin_circleup_row>MT9V03X_H-50&&Left_Lost_Time<=2
                {
                    t2_ms_cricle2 = 0;
                    circle_mode=3;//
                    now_yaw=0;
                }
            }
            //�׶�3������׼��
            if(circle_mode==3)
            {
                yaw_flag=1;

    //              Circle_out_Right();
                if(abs(now_yaw)>=22800)//&&Rout_circleup_row>MT9V03X_H-80)//?   ?   200
                {//speed = aim_speed + (200-aimspeed)exp(-0.02*abs(dev))
                    t2_ms_cricle3 = 0;
                    circle_mode=4;//
                    now_yaw=0;
                }
            }
            //�׶�4������ִ��
            if(circle_mode==4)
            {
    //              get_upturningright_point();
    //              Circle_out_Right();

                yaw_flag=1;

                if(abs(now_yaw)>=24800)// �ӽ���ɽǶ�
                {
                    if(Finnalline<=20)// ���ձ߽����㹻�̣��ӽ�ֱ����
                    {
                        circle_mode = 5;// ������β�׶�
                        now_yaw=0;
                    }
                }
            }
            //�׶�5���ָ�ֱ��
            if(circle_mode==5)
            {
                gpio_toggle_level(BUZZER_PIN);// ��������˸
    //              Circle_out_Right();

    //              get_upturningleft_point();//     ??
    //              get_upturningright_point();
    //              Lengthen_Left_Boundry(L_upcorner_row-1,MT9V03X_H-1);//     ??
    //              Circle_in_Right();
                if(T1_ms_cricle5_flag==1&&Right_sideIsNoLost(MT9V03X_H-30,MT9V03X_H-80)==0)//Right_Lost_Time<15&&Rin_circleup_row>MT9V03X_H-40&&Rin_circleup_flag!=1
                {
                    gpio_set_level(BUZZER_PIN, GPIO_LOW);
    //                  buzzer();
                    yaw_flag=0;
                    now_yaw=0;
                    circle_cnt++;
                    T1_ms_cricle5_flag = 0;
                    circle_flag=0;
                    circle_mode=0;
                    right_circle_flag=0;
    //                  buzzer();
                }
            }
            }
}


#endif
