#include  "zf_common_headfile.h"

#define White 255   //�����ɫ���ص�ֵΪ255
#define Black 0     //�����ɫ���ص�ֵΪ0

#define Left 1     // ��������ʶΪ1
#define Right 2    // �����Ҳ��ʶΪ2
#define up 0       // �����ϱ߽��ʶΪ0
#define down 1     // �����±߽��ʶΪ1
#define START_H 110 // ������ʼɨ����
#define END_H 10   // �������ɨ����
int x,y;
float parameterB,parameterA;   //y=parameterB*x+parameterA
extern int flexible_leg_high;
float k1,k2,k3,k4,k5,k6;
int left[120]={2};             //������СΪ120�����߽����飬��ʼֵΪ2
int right[120]={185};          //������СΪ120���Ҳ�߽����飬��ʼֵΪ185
int left_cicle[120]={2};             //������СΪ120�����߽����飬��ʼֵΪ2
int right_cicle[120]={185};          //������СΪ120���Ҳ�߽����飬��ʼֵΪ185
int middle[120]={93};          //������СΪ120���м������飬��ʼֵΪ93
int Endline=1;                 //�����յ��߱�־����ʼΪ1
int WhiteNum=0;                //��ɫ���ؼ���������ʼΪ0
uint8_t imageOut[2][image_w]; //�������±߽����飬[0]Ϊ�ϱ߽磬[1]Ϊ�±߽�
uint8 right_lost_num=0;        //ͳ���ұ߽綪ʧ�Ĵ���
uint8 left_lost_num=0;         //ͳ����߽綪ʧ�Ĵ���
uint8 imag[120][188];          //����120��188�е�ͼ������      ���ڴ洢ͼ������
uint8 imag_copy[120][188];
uint8 threshold_value=175;     //��ֵ����ֵ����ʼ175
uint32 image_process_time=0;   //ͼ����ʱ��
float turn_value=0;       // ת��ֵ����border���ж��м���ó�
float border = 96;        // �߽���ֵ,ͼ��ƫ��
float border_last = 96;        // �߽���ֵ,��һ��ͼ��ƫ��
int annulus_L_memory_flag=0;

uint8 Right_straight_flag=0; //��ֱ��
uint8 Left_straight_flag=0; //��ֱ��

uint8 annulus_L_Flag=0;       //��Բ��
uint8 annulus_R_Flag=0;       //��Բ��
uint8 annulus_L_memory=0;     //��Բ���Ʋ�        ���ڴ洢��������ļ���״̬
uint8 annulus_R_memory=0;     //��Բ���Ʋ�        ���ڴ洢�һ�������ļ���״̬
uint8 zebra_crossing_flag=0;//������

// RoundaboutGetArc�˲�����
uint8 roundabout_arc_filter_left = 0;   // ��Բ�����߼���˲�������
uint8 roundabout_arc_filter_right = 0;  // ��Բ�����߼���˲�������
#define ROUNDABOUT_ARC_FILTER_THRESHOLD 2  // �˲���ֵ��������⵽3�β�ȷ��

// roundabout_L/R�˲�����
uint8 roundabout_L_filter = 0;   // ��Բ���Ϲյ����˲�������
uint8 roundabout_R_filter = 0;   // ��Բ���Ϲյ����˲�������
#define ROUNDABOUT_FILTER_THRESHOLD 3  // roundabout�˲���ֵ��������⵽2�β�ȷ��


//Բ��͹���
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;     //Բ������־��0=δ��⵽��1=��⵽��

//����ʶ���
uint8 Exit_loop_X=0;
uint8 Exit_loop_Y=0;
uint8 Exit_loop_Flag=0;     //��������־��0=δ������1=�ѳ�����

//ʮ��
uint8 Crossroad_Flag=0;      //ʮ��(0=��ʮ�֣�1=��⵽ʮ�֣�
uint8 Crossroad_memory=0;     //ʮ�ּƲ�
uint8 Finish_Flag=1; //������ɱ�ʶλ

//����
uint8 Lost_left_Flag = 0;          //��ඪ�߱�־��0=δ���ߣ�1=��ඪ�ߣ�
uint8 Lost_right_Flag = 0;         //�Ҳඪ�߱�־��0=δ���ߣ�1=�Ҳඪ�ߣ�
uint8 Lost_point_L_scan_line = 0;  //��ඪ�ߵ�ɨ���У�Y���꣩
uint8 Lost_point_R_scan_line = 0;  //�Ҳඪ�ߵ�ɨ���У�Y���꣩

//���¹յ�
uint8 Lower_left_inflection_X =0;
uint8 Lower_left_inflection_Y =0;
uint8 Lower_left_inflection_Flag=0;

//���¹յ�
uint8 Lower_right_inflection_X =0;
uint8 Lower_right_inflection_Y =0;
uint8 Lower_right_inflection_Flag=0;

//���Ϲյ�
uint8 Upper_left_inflection_X =0;
uint8 Upper_left_inflection_Y =0;
uint8 Upper_left_inflection_Flag=0;

//���Ϲյ�
uint8 Upper_right_inflection_X =0;
uint8 Upper_right_inflection_Y =0;
uint8 Upper_right_inflection_Flag=0;

int sum_island = 0;       // ��������
int island = 0;       
int cross_sum = 0;        // ��������

SingleBridgeState BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;

//����һ���������ľ���ֵ
float absolute(float z)
{
    z = z< 0 ? (-z) : z;
uint8 Upper_left_inflection_Flag=0;
    return z;
}
/*
�������ƣ�int my_abs(int value)
����˵�����������ֵ
����˵����
����ֵ������ֵ
�޸�ʱ�䣺2025��1��5��
��ע��
ʾ����my_abs(x)
*/
int  my_abs(int value)
{
    if(value>=0) return value;
    else return -value;
}
//��x������a��b֮��
int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

//x������-y��y֮��
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}
/*
�������ƣ�int my_limit(int num,int value)
����˵������ֵ���ƺ���
����˵����
����ֵ��a <= c <= b
�޸�ʱ�䣺2025��3��14��
��ע��
ʾ����my_auu(c,b,a)
*/
int my_auu(int c,int b,int a)
 {
     if(c > b)
             return b;
     else if(c < a)
             return a;
     else
         return c;
 }

//ͼ��ɼ�����
//�� mt9v03x_image �л�ȡͼ�����ݣ����洢�� original_image ��
uint8 original_image[image_h][image_w];
void Get_image(uint8(*mt9v03x_image)[image_w])
{
    //use_num�����Ƿ�ѹ��ͼ��(1��ѹ����2���в���ѹ��)
#define use_num     1   //����ͼ�����ݵĲ������
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//����Ĳ�����д�������ͷ�ɼ�����ͼ��
            line++;
        }
        line = 0;
        row++;
    }
}

//���(Otsu)��ֵ��    ʹ�� Otsu �㷨����ͼ��������ֵ
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{

#define GrayScale 256               //����Ҷȼ�Ϊ 256
    uint16 Image_Width  = col;      //����ͼ��Ŀ��
    uint16 Image_Height = row;      //����ͼ��ĸ߶�
    int X; uint16 Y;
    uint8* data = image;            //�������ͼ������ָ�븳ֵ�� data
    int HistGram[GrayScale] = {0};  //�洢ÿ���Ҷȼ�������������

    uint32 Amount = 0;                  // ��������
    uint32 PixelBack = 0;               // ����������
    uint32 PixelIntegralBack = 0;       // �������ػҶ��ܺ�
    uint32 PixelIntegral = 0;           // �������ػҶ��ܺ�
    int32 PixelIntegralFore = 0;        // ǰ�����ػҶ��ܺ�
    int32 PixelFore = 0;                // ǰ��������
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // ��䷽��
    uint8 MinValue=0, MaxValue=0;       // ͼ���е���С�����Ҷ�ֵ
    uint8 Threshold = 0;                // ������ֵ

    //ͳ��ÿ���Ҷ�ֵ����������
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height��ΪY =Image_Height���Ա���� �ж�ֵ��
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //ͳ��ÿ���Ҷ�ֵ�ĸ�����Ϣ
        }
    }



//ȷ��ͼ����ʵ�ʴ��ڵ���С�����Ҷ�ֵ�������ں��������п��ǲ����ڵĻҶȼ������Ч�ʡ�
    // ��ȡ��С�Ҷ�ֵ
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
    // ��ȡ���Ҷ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--) ;
    // �������������ͼ����ֻ��һ�ֻ����ֻҶ�
    if (MaxValue == MinValue)
    {
        return MaxValue;          // ͼ����ֻ��һ����ɫ  ���ظ���ɫ�ĻҶ�ֵ��
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;          // ͼ����ֻ�ж�����ɫ   ���ؽ�С�ĻҶ�ֵ��
    }



    // �������лҶȼ�����������������
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  ��������
    }
    // �����������صĻҶ��ܺ�
    PixelIntegral = 0;     //��ʼ�����ػ���ֵΪ 0
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//�Ҷ�ֵ����
    }

    //Ѱ�������ֵ
    SigmaB = -1 ;                        //��ʼ����䷽��Ϊ -1
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //ǰ�����ص���
          PixelFore = Amount - PixelBack;         //�������ص���
          OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
          OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
          PixelIntegralBack += HistGram[Y] * Y;  //ǰ���Ҷ�ֵ
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
          MicroBack = (double)PixelIntegralBack / PixelBack;//ǰ���ҶȰٷֱ�
          MicroFore = (double)PixelIntegralFore / PixelFore;//�����ҶȰٷֱ�
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//����������䷽���Ӧ����ֵ
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }

   return Threshold;//���������ֵ
}

int threshold_adjust=0;
//��ֵ������   ��ͼ��ת��Ϊ�ڰ׶�ֵͼ��
void binaryzation(void)
{
  uint8 i,j;
//���� OtsuThreshold �������������ֵ������ 5��
threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+threshold_adjust;
  // ����ͼ�����
  for(i = 0;i<image_h;i++)
  {
       //����ͼ�����
      for(j = 0;j<image_w;j++)
      {
          //�����ǰ����ֵ������ֵ�����丳ֵΪ��ɫ����
          if(original_image[i][j]>threshold_value)imag_copy[i][j] = white_pixel;
          //�����丳ֵΪ��ɫ����
          else imag_copy[i][j] = black_pixel;
      }
  }
  
  // ������½�����(����ЧӦ����)�������д�򷨶�ֵ��
  // ����Χ: ��� image_w*3/4 �� image_w, �߶� image_h*2/3 �� image_h
  uint8 corner_start_x = image_w * 3 / 4;  // ���½�������ʼx����
  uint8 corner_end_x = image_w;            // ���½��������x����  
  uint8 corner_start_y = image_h * 2 / 3;  // ���½�������ʼy����
  uint8 corner_end_y = image_h;            // ���½��������y����
  
  // ������ʱ����洢���½��������������
  uint8 corner_width = corner_end_x - corner_start_x;
  uint8 corner_height = corner_end_y - corner_start_y;
  uint8 corner_image[corner_height * corner_width];
  
  // ��ȡ���½��������������
  uint16 corner_index = 0;
  for(i = corner_start_y; i < corner_end_y; i++)
  {
      for(j = corner_start_x; j < corner_end_x; j++)
      {
          corner_image[corner_index++] = original_image[i][j];
      }
  }
  
  // �����½����򵥶���������ֵ
  uint8 corner_threshold = OtsuThreshold(corner_image, corner_width, corner_height) + threshold_adjust;
  
  // ������ֵ���¶����½�������ж�ֵ��
  for(i = corner_start_y; i < corner_end_y; i++)
  {
      for(j = corner_start_x; j < corner_end_x; j++)
      {
          //�����ǰ����ֵ�������½�������ֵ�����丳ֵΪ��ɫ����
          if(original_image[i][j] > corner_threshold) imag_copy[i][j] = white_pixel;
          //�����丳ֵΪ��ɫ����
          else imag_copy[i][j] = black_pixel;
      }
  }
}

//�����߽������
uint8 start_point_l[2] = { 0 };//�������x��yֵ
uint8 start_point_r[2] = { 0 };//�ұ�����x��yֵ
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //����
    start_point_l[0] = 0;//��ʼ������ʼ��� x ����Ϊ 0
    start_point_l[1] = 0;//��ʼ������ʼ��� y ����Ϊ 0

    start_point_r[0] = 0;//��ʼ������ʼ��� x ����Ϊ 0
    start_point_r[1] = 0;//��ʼ������ʼ��� y ����Ϊ 0

    // ���ݵ�����״̬����������Χ
    if (BridgeState != SINGLE_BRIDGE_ACTIVE)
    {
        //���м�����ߣ��������
        for (i = image_w / 2; i > border_min; i--)
        {
            start_point_l[0] = i;//��������ʼ��� y ����
            start_point_l[1] = start_row;//��������ʼ��� y ����
            //�����ǰ����Ϊ��ɫ����ǰһ������Ϊ��ɫ��˵���ҵ�������ʼ��
            if (imag[start_row][i] == 255 && imag[start_row][i - 1] == 0)
            {
                //printf("�ҵ�������image[%d][%d]\n", start_row,i);
                l_found = 1;//��������ʼ���ҵ��ı�־λ
                break;
            }
        }

        //��ͼ���м����ұ�������������ʼ��
        for (i = image_w / 2; i < border_max; i++)
        {
            start_point_r[0] = i;//��������ʼ��� x ����
            start_point_r[1] = start_row;//��������ʼ��� y ����
            //�����ǰ����Ϊ��ɫ���Һ�һ������Ϊ��ɫ��˵���ҵ�������ʼ��
            if (imag[start_row][i] == 255 && imag[start_row][i + 1] == 0)
            {
                //printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
                r_found = 1;//��������ʼ���ҵ��ı�־λ
                break;
            }
        }
    }
    else
    {
        // ������״̬���ſ�������Χ���Ӹ���Χ�������
        //���м�����ߣ�������� - ����������Χ
        for (i = image_w * 2 / 3; i > border_min; i--)
        {
            start_point_l[0] = i;//��������ʼ��� y ����
            start_point_l[1] = start_row;//��������ʼ��� y ����
            //�����ǰ����Ϊ��ɫ����ǰһ������Ϊ��ɫ��˵���ҵ�������ʼ��
            if (imag[start_row][i] == 255 && imag[start_row][i - 1] == 0)
            {
                //printf("�ҵ�������image[%d][%d]\n", start_row,i);
                l_found = 1;//��������ʼ���ҵ��ı�־λ
                break;
            }
        }

        //��ͼ���м����ұ�������������ʼ�� - ����������Χ
        for (i = image_w / 3; i < border_max; i++)
        {
            start_point_r[0] = i;//��������ʼ��� x ����
            start_point_r[1] = start_row;//��������ʼ��� y ����
            //�����ǰ����Ϊ��ɫ���Һ�һ������Ϊ��ɫ��˵���ҵ�������ʼ��
            if (imag[start_row][i] == 255 && imag[start_row][i + 1] == 0)
            {
                //printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
                r_found = 1;//��������ʼ���ҵ��ı�־λ
                break;
            }
        }
    }

    //���������ʼ�㶼�ҵ��ˣ�����1
    if(l_found&&r_found)return 1;
    else {
        //printf("δ�ҵ����\n");
        return 0;
    }
}

//�����߽����
#define USE_num image_h*3   //����ָ���洢��������С
 //��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//�洢��߽�ĵ�
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//�洢�ұ߽�ĵ�
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ߽��ķ���
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢��߽��ķ���
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���
//���ܶ��������������ֹ��־��ͼ�����ݡ����ұ߽������ָ�롢������ʼ������ͽ�����ָ��
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*Endline)
{

    uint8 i = 0, j = 0;
    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };//�洢��߽����������
    uint8 index_l = 0;                     //�洢��߽��������������
    uint8 temp_l[8][2] = { {  0 } };      //��ʱ�洢��߽�ĵ�
    uint8 center_point_l[2] = {  0 };     //�洢��߽�����ĵ�
    uint16 l_data_statics;                //ͳ����߽��ҵ��ĵ������
    //����˸����򣬱�ʾ��߽����������
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�����˳ʱ��

    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };//�洢�ұ߽����������
    uint8 center_point_r[2] = { 0 };        //�洢�ұ߽�����ĵ�
    uint8 index_r = 0;                      //�洢�ұ߽��������������
    uint8 temp_r[8][2] = { {  0 } };        //��ʱ�洢�ұ߽�ĵ�
    uint16 r_data_statics;                  //ͳ���ұ߽��ҵ��ĵ������
    //����˸�����    ��ʾ�ұ߽����������
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�������ʱ��

    l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

    //��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x;//��ʼ����߽�����ĵ�� x ����Ϊ����ʼ��� x ����
    center_point_l[1] = l_start_y;//��ʼ����߽�����ĵ�� y ����Ϊ����ʼ��� y ����
    center_point_r[0] = r_start_x;//��ʼ���ұ߽�����ĵ�� x ����Ϊ����ʼ��� x ����
    center_point_r[1] = r_start_y;//��ʼ���ұ߽�����ĵ�� y ����Ϊ����ʼ��� y ����

    //��������ѭ��
    while (break_flag--)
    {

        //������߽����������
        for (i = 0; i < 8; i++)//����8F����
        {   //������߽���������� x ����
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            //������߽���������� y ����
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0];//����߽�����ĵ�� x ����洢�� points_l ��
        points_l[l_data_statics][1] = center_point_l[1];//����߽�����ĵ�� y ����洢�� points_l ��
        l_data_statics++;//������һ

        //�����ұ߽����������
        for (i = 0; i < 8; i++)//����8F����
        {
            //�����ұ߽���������� x ����
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            //�����ұ߽���������� y ����
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//�����㣬��ʹ��
        //������߽����������
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//�����㣬��ʹ��    ��ʼ����ʱ�洢��߽�������
            temp_l[i][1] = 0;//�����㣬��ʹ��    ��ʼ����ʱ�洢��߽�������
        }

        //����ж�
        for (i = 0; i < 8; i++)
        {   //�����ǰ��������Ϊ��ɫ������һ����������Ϊ��ɫ��˵���ҵ�����߽�ĵ�
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                //����߽�ĵ�� x ����洢�� temp_l ��
                temp_l[index_l][0] = search_filds_l[(i)][0];
                //����߽�ĵ�� y ����洢�� temp_l ��
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//��¼��߽��ķ���
            }

            //����ҵ�����߽�ĵ�
            if (index_l)
            {
                //���������
                center_point_l[0] = temp_l[0][0];//������߽�����ĵ�� x ����
                center_point_l[1] = temp_l[0][1];//������߽�����ĵ�� y ����
                //������߽�ĵ�
                for (j = 0; j < index_l; j++)
                {
                    //�����ǰ��� y ����С�����ĵ�� y ���꣬�������ĵ������
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        //��������������������ͬ������ѭ�� - �޸ģ�ֻ��ͼ���ϲ�����������ͬ���˳�
        if (((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
            && (points_r[r_data_statics][1] < 30 || points_l[l_data_statics-1][1] < 30)) // ֻ����ͼ���ϲ�����������ͬ���˳�
        {
            //printf("���ν���ͬһ���㣬�˳�\n");
            break;
        }
        //������ұ߽�ĵ�ľ���С�� 2������ѭ���������½����ߵ�λ�� - �޸ģ�����λ�ú������������
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2
            ) 
        {
            //printf("\n���������˳�\n");
            *Endline   = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
            //printf("\n��y=%d���˳�\n",*Endline);
            break;
        }
        //����ұ߽�ĵ�� y ����С����߽�ĵ�� y ���꣬��߽�ȴ��ұ߽硣
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
           // printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
            continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
        }
        //�����߽�ĵ�ķ���Ϊ 7�����ұ߽�ĵ�� y ���������߽�ĵ�� y ���꣬��߽����һ����
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
        {
            //printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//�ұ߽�������� 1

        index_r = 0;//�����㣬��ʹ��
        //�����ұ߽����������
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//�����㣬��ʹ��
            temp_r[i][1] = 0;//�����㣬��ʹ��
        }

        //�ұ��ж�
        for (i = 0; i < 8; i++)
        {
            //�����ǰ��������Ϊ��ɫ������һ����������Ϊ��ɫ��˵���ҵ����ұ߽�ĵ㡣
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                //���ұ߽�ĵ�� x ����洢�� temp_r ��
                temp_r[index_r][0] = search_filds_r[(i)][0];
                //���ұ߽�ĵ�� y ����洢�� temp_r ��
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//������һ
                dir_r[r_data_statics - 1] = (i);//��¼�ұ߽��ķ���
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            //����ҵ����ұ߽�ĵ�
            if (index_r)
            {

                //���������
                center_point_r[0] = temp_r[0][0];//�����ұ߽�����ĵ�� x ����
                center_point_r[1] = temp_r[0][1];//�����ұ߽�����ĵ�� y ����
                //�����ұ߽�ĵ�
                for (j = 0; j < index_r; j++)
                {
                    //�����ǰ��� y ����С�����ĵ�� y ���꣬�������ĵ������
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        //�����ұ߽�����ĵ������
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }
            }
        }
    }

    //ȡ��ѭ������
    *l_stastic = l_data_statics;//������߽������ָ���ֵ
    *r_stastic = r_data_statics;

}
uint16 l_index[120] = {0};  // ÿ�ж�Ӧ��points_l�е�����
uint16 r_index[120] = {0};  // ÿ�ж�Ӧ��points_r�е�����

//��ȡ��߽������
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    //��ʼ��
    for (i = 0;i<image_h;i++)
    {
        left[i] = border_min;
        l_index[i] = 0;
    }
    h = image_h - 2;//��ʼ���к�Ϊͼ��߶ȼ� 2
    //������߽�ĵ�
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        //�����ǰ��� y ��������к�
        if (points_l[j][1] == h)
        {
            //����߽�����ĵ�ǰ�е�Ԫ�ظ���Ϊ��ǰ��� x ����� 1
            left[h] = points_l[j][0]+1;
            l_index[h] = j;                    // ��¼����ԭʼ�����е�����
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
        {
            break;//�����һ���˳�
        }
    }
}
void get_left_cicle(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //��ʼ��
    for (i = 0;i<image_h;i++)
    {
        left_cicle[i] = border_min;
    }
    h = image_h - 2;//��ʼ���к�Ϊͼ��߶ȼ� 2
    //������߽�ĵ�
    for (j = 0; j < total_L; j++)
    {
        if (points_l[j][1] == h)
        {
            left_cicle[h] = points_l[j][0]+1;
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
        {
            break;//�����һ���˳�
        }
    }
}

//�ж���߽��Ƿ�ʧ
void lost_left(void){
    uint8 i=0;
    left_lost_num=0;
    Lost_left_Flag=0;
    //�ӵ� 110 �е��� 10 �б���ͼ��
    for(i=Endline+10;i<110;i++){
        //�����ǰ�еĵ� 2 �е�����Ϊ��ɫ
        if(imag[i][3]==White){
            left_lost_num++;//��߽綪ʧ������ 1
            Lost_point_L_scan_line=i+4;//��¼��߽綪ʧ���ɨ����λ��
        }
        //�����߽綪ʧ�������� 15
        if(left_lost_num>(100-Endline)/9){
            Lost_left_Flag=1; //�ж�����·��Ƿ���
            return;
        }
    }
}

//�ú������ڻ�ȡ�ұ߽������
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //����ͼ�����
    for (i = 0; i < image_h; i++)
    {
        right[i] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
        r_index[i] = border_max; // ��ʼ��Ϊ���߽�ֵ
    }
    h = image_h - 2;
    //�����ұ߽�ĵ�
    for (j = 0; j < total_R; j++)
    {
        //�����ǰ��� y ��������к�
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;
            r_index[h] = j;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}
void get_right_cicle(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //����ͼ�����
    for (i = 0; i < image_h; i++)
    {
        right_cicle[i] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    h = image_h - 2;
    //�����ұ߽�ĵ�
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right_cicle[h] = points_r[j][0] - 1;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}
//�ж��ұ߽��Ƿ�ʧ
void lost_right(void){
    uint8 i=0;
    right_lost_num=0;
    Lost_right_Flag=0;
    //�ӵ� 110 �е��� 10 �б���ͼ��
    for(i=Endline+10;i<110;i++){
        //�����ǰ�еĵ� 185 �е�����Ϊ��ɫ
        if(imag[i][185]==White){
            right_lost_num++;//�ұ߽綪ʧ������ 1
            Lost_point_R_scan_line=i+4;//��¼�ұ߽綪ʧ���ɨ����λ��
        }
        //����ұ߽綪ʧ����������ֵ
        if(right_lost_num>(100-Endline)/9){
            Lost_right_Flag=1;  //�ж��ұ��·��Ƿ���
            return;
        }
    }
}

//Ѱ������
void middle_line(void){
    for(y=119;y>Endline;y--){
        //�����м��ߵ����꣬�����ұ߽������ƽ��ֵ
        middle[y]=(right[y]+left[y])/2;
    }
}


//�������ͺ͸�ʴ����ֵ����
#define threshold_max   255*5
#define threshold_min   255*2
void image_filter(uint8(*imag)[image_w])//��̬ѧ�˲������ͺ͸�ʴ��˼��    ���ڱ�ʾͼ������
{
    uint16 i, j;
    uint32 num = 0;
    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //ͳ�ư˸����������ֵ
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];

            //�����Χ���ص��ܺʹ��ڵ�����ֵ�����ֵ���ҵ�ǰ����Ϊ��ɫ
            if (num >= threshold_max && imag[i][j] == 0)
            {
                imag[i][j] = 255;//��  ���Ը�ɺ궨�壬�������
            }
            //�����Χ���ص��ܺ�С�ڵ�����ֵ����Сֵ���ҵ�ǰ����Ϊ��ɫ
            if (num <= threshold_min && imag[i][j] == 255)
            {
                imag[i][j] = 0;//��
            }
        }
    }
}


//������ͼ��ı�Ե���ƾ���
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;            //��ͼ��ĵ�һ�е���������Ϊ��ɫ
        image[i][1] = 0;            //��ͼ��ĵڶ��е���������Ϊ��ɫ
        image[i][image_w - 1] = 0;  //��ͼ������һ�е���������Ϊ��ɫ
        image[i][image_w - 2] = 0;  //��ͼ��ĵ����ڶ��е���������Ϊ��ɫ
    }

    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;        //��ͼ��ĵ�һ�е���������Ϊ��ɫ
        image[1][i] = 0;        //��ͼ��ĵڶ��е���������Ϊ��ɫ
        //image[image_h-1][i] = 0;

    }
}

//==========================================================���������ж�=================================================
/**
 * @brief ���Ե��ʧ�ж�
 * @param uint8 *l_border     ���Ե����ĵ�ַ
 * @param uint8 start         �����ʼ��
 * @param uint8 end           �����ֹ��
 * @see CTest       l_loss_judge(l_border,20 ,0)
 * @return ����ֵ˵��
 * -1 ��Ч
 * 1  ��ʧ
 * 0  ����
 */
int l_loss_judge(uint8 *l_border,uint8 start ,uint8 end)
{
    uint16 i;         // ѭ��������
    uint16 sum = 0;   // ��¼����"��Ե��ʧ"����������
    
    // ����start��end����Ч��Χ�ڣ�2��image_h-2��
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
     // ��������Ч�ԣ������ʼ�д��ڵ�����ֹ�У�������Ч
    if(start >= end)
        return -1;
    // ����[start, end]��Χ�ڵ�ÿһ��
    for(i = start;i <= end; i++)
    {
        // �����ǰ�е����Եλ��С�ڵ���border_min+2
       // ˵�����Ե���ڿ���ͼ����࣬���ܶ�ʧ
       if(l_border[i] <= border_min+2 )
           sum++;
    }
    // �������"��Ե��ʧ"���������������ܼ��������80%
    // ����Ϊ������������Ե��ʧ
    if(sum >= (my_abs(start - end)/5*4))
        return 1;  // ���Ե��ʧ
    else
        return 0;  // ���Ե����
}

/**
 * @brief �ұ�Ե��ʧ�ж�
 * @param uint8 *r_border     �ұ�Ե����ĵ�ַ
 * @param uint8 start         �����ʼ��
 * @param uint8 end           �����ֹ��
 * @see CTest       r_loss_judge(l_border,20 ,0)
 * @return ����ֵ˵��
 * -1 ��Ч
 * 1  ��ʧ
 * 0  ����
 */
int r_loss_judge(uint8 *r_border,uint8 start ,uint8 end)
{
    uint16 i;         // ѭ��������
    uint16 sum = 0;   // ��¼����"��Ե��ʧ"����������
    
    // ����start��end����Ч��Χ�ڣ�2��image_h-2��
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    // ��������Ч�ԣ������ʼ�д��ڵ�����ֹ�У�������Ч
    if(start >= end)
        return -1;
    for(i = start;i <= end; i++)
    {
       if(r_border[i] >= border_max-2 )
           sum++;
    }
    if(sum >= (my_abs(start - end)/5*4))
        return 1;
    else
        return 0;
}


//==========================================================���������ж�=================================================

//==========================================================�յ�ʶ��====================================================

//---------------------------------���¹յ�--------------------------------
//-----------------------------�ڶ��棺�öϵ��ж�------------------------
void Lower_left(void){
    // ��ʼ�����¹յ��־������
    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;
    // ��ͼ��ײ�����ɨ��Ѱ�ҹյ�
    for(y=image_h-3;y>Endline+10;y--){
        // ȷ��ɨ������Ч
        if(y>30){
            // �յ��ж���������ǰ�����Ϸ��еĲ�ֵ>5���Ϸ��нӽ��߽�(2)
            // ͬʱ��ǰ�����·��в�ֵ<5�ҵ�ǰ��λ�ú���(>10)
            if((left[y]-left[y-4])>10&&left[y-4]<=4&&(left[y]-left[y+2])<5&&left[y]>10){
                // �����������Ƿ������¹յ㲢��¼����
                Lower_left_inflection_Flag=1;
                Lower_left_inflection_X =left[y];
                Lower_left_inflection_Y =y;
                return;
            }
        }
    }
}

//-------------------------------------���¹յ�---------------------------------
//-----------------------------�ڶ��棺�öϵ��ж�------------------------
void Lower_right(void){
    // ��ʼ�����¹յ��־������
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
    // ��ͼ��ײ�����ɨ��Ѱ�ҹյ�
    for(y=image_h-3;y>Endline+10;y--){
        // ȷ��ɨ������Ч
        if(y>30){
            // �յ��ж���������ǰ�����Ϸ��еĲ�ֵ>5���Ϸ��нӽ��߽�(185)
            // ͬʱ��ǰ�����·��в�ֵ<5�ҵ�ǰ��λ�ú���(<170)
            if((right[y-4]-right[y])>10&&right[y-4]>=185&&(right[y+2]-right[y])<5&&right[y]<170){
                // �����������Ƿ������¹յ㲢��¼����
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                return;
            }
        }
    }
}

//-------------------------------------���Ϲյ�------------------------------------
//-----------------------------�ڶ��棺�öϵ��ж�------------------------
void Upper_left(void){
    uint8 h=image_h-3;
    uint8 i;

    // ��ʼ�����Ϲյ��־������
    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X =0;
    Upper_left_inflection_Y =0;

    // ����ඪʧ��ʱִ�����⴦��
    if(Lost_left_Flag==1){
        //���Բ��д���ҵ㷽ʽ
          if(annulus_L_Flag==1){
              // �Ӷ�ʧ���·���ʼɨ��
              for(h=Lost_point_L_scan_line+25;h>(Endline+25);h--){//�Ķ�
                // ��������㣺��ǰ�����·��в�ֵ>3���·��нӽ��߽�

                  if((left[h]-left[h+2])>40&&left[h+4]<=10&&left[h]!=2&&((left[h-4]-left[h])<4||(left[h-4]-left[h])>1000)&&left[h]>20){
                   if(left[h]>93) {            //���Բ��״̬4���߳��ֵĶϲ�������еĳ����Ż�
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       //-----------��һ�棺��ɨ�߷����ж�-----------
                       uint8 Find_Flag=0;
                       for(y=h;y<110;y++){
                           Find_Flag=0;
                           for(x=left[h]+10;x>70;x--){
                                // Ѱ�ұ߽�㣺�����Ҳ��
                                if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
                                    Find_Flag=1;
                                    break;
                                 }

                               }
                            // ����Ҳ����߽�㣬˵���ѹ��յ�
                            if(Find_Flag==0){
                               Upper_left_inflection_Y =y-1;
                               return;
                               }

                           }

                    }
                    else{
                         // ��ͨ����ֱ�Ӽ�¼��
                        Upper_left_inflection_Flag=1;
                        Upper_left_inflection_X =left[h];
                        Upper_left_inflection_Y =h;
                        return;
                    }
                }
            }
        }
        // ��Բ�������Ĵ���
        else{
            for(h=Lost_point_L_scan_line+3;h>(Endline+10);h--){
                // ��������㣺��ǰ�����·��в�ֵ>3���·��нӽ��߽�
                if((left[h]-left[h+4])>3&&left[h+10]==2&&left[h]!=2&&(left[h-3]-left[h])<3){

                    // ��ǲ���¼�յ�
                    Upper_left_inflection_Flag=1;
                    Upper_left_inflection_X =left[h];
                    Upper_left_inflection_Y =h;
                    return;

                }
             }
          }

    }
    //----------------------------��һ�棺��ɨ�߷����ж�--------------------------------
    //        for(y=Endline+15;y<110;y++){
    //            for(x=left[y-1];x<186&&x>1;x--){
    //                  if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
    //                     Upper_left_inflection_X =x;
    //                     Upper_left_inflection_Y =y;
    //                     //ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y , RGB565_BLUE );
    //                     break;
    //                  }
    //                  if(x<(left[y-1]-3)){
    //                      Upper_left_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //
    //
    //            }

}

//-----------------------------------���Ϲյ�-----------------------------------
//-----------------------------�ڶ��棺�öϵ��ж�------------------------
void Upper_right(void){
    uint8 h=image_h-3;
    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;
    if(Lost_right_Flag==1){
        //���Բ��д���ҵ㷽ʽ
          if(annulus_R_Flag==1){
              for(h=Lost_point_R_scan_line+5;h>(Endline+10);h--){
                if((right[h+8]-right[h])>3&&right[h+8]==185&&right[h]!=185&&(right[h]-right[h-4])<5&&h<60){
                   if(right[h]>93) {            //���Բ��״̬4���߳��ֵĶϲ�������еĳ����Ż�
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       //-----------��һ�棺��ɨ�߷����ж�-----------
                       uint8 Find_Flag=0;
                       for(y=h;y<110;y++){
                           Find_Flag=0;
                           for(x=right[h]+10;x>70;x--){
                                 if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
                                     Find_Flag=1;
                                    break;
                                 }

                               }
                           if(Find_Flag==0){
                               Upper_right_inflection_Y =y-1;
                               return;
                               }

                           }

                   }
                   else{
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       return;
                   }
                }
             }
          }
          else{
              for(h=Lost_point_R_scan_line+5;h>(Endline+10);h--){

                      if((right[h+5]-right[h])>15&&right[h+10]==185&&right[h]!=185&&(right[h]-right[h-1])<3&&right[h]<178){
                          Upper_right_inflection_Flag=1;
                          Upper_right_inflection_X =right[h];
                          Upper_right_inflection_Y =h;
                          return;

                  }

              }
          }

    }
    //----------------------------��һ�棺��ɨ�߷����ж�--------------------------------
    //        for(y=Endline+15;y<110;y++){
    //            for(x=right[y-1];x<186&&x>1;x++){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Upper_right_inflection_X =x;
    //                     Upper_right_inflection_Y =y;
    //                    // ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y , RGB565_BLUE );
    //                     break;
    //                  }
    //                  if(x>(right[y-1]+3)){
    //                      Upper_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //            }
}

//--------�յ����ж�-----------
void inflection_point(void){

    Upper_left();
    Upper_right();
    Lower_left();
    Lower_right();
}
//==================================================��ֱ��ʶ��===========================================================
float k11,k22,k33,k44,k55,k66,k77,k88;
void right_straight(void){
    float k1,k2,k3,k4,k5,k6,k7,k8;
    Right_straight_flag=0;   // ��ʼ����ֱ�߱�־
    if(Endline>17)return;
    // ���㲻ͬ�����б�������ж�ֱ�߶�
    k1=((float)right[Endline+70]-(float)right[Endline+40])/30;  // �ײ�����б��
    k2=((float)right[Endline+60]-(float)right[Endline+50])/10;  // �в�����б��
    k3=((float)right[Endline+50]-(float)right[Endline+10])/40;   // �������ײ�б��
    k4=((float)right[Endline+30]-(float)right[Endline+10])/20;   // �������в�б��
    k5=((float)right[Endline+70]-(float)right[Endline+60])/10;   // ��ײ�����б��
    k6=((float)right[Endline+40]-(float)right[Endline+30])/10;   // ���²�����б��
    k7=((float)right[Endline+40]-(float)right[Endline+15])/25;   // ���ϲ�����б��
    k8=((float)right[Endline+20]-(float)right[Endline+10])/10;   // �ϲ�ϸ������б��
    // �ж��Ƿ�Ϊֱ�� - ʹ�ø���б�ʵ���о�ȷ�ж�
    if(absolute(k1-k2)<0.3&&absolute(k2-k3)<0.3&&absolute(k3-k4)<0.3&&
       absolute(k4-k5)<0.3&&absolute(k5-k6)<0.3&&absolute(k6-k7)<0.3&&
       absolute(k7-k8)<0.3&&absolute(k1-k8)<0.5&&
       k1!=0&&k2!=0&&k3!=0&&k4!=0&&k5!=0&&k6!=0&&k7!=0&&k8!=0){
            Right_straight_flag=1;
    }
}
//==================================================��ֱ��ʶ��===========================================================
void left_straight(void){
    float k1, k2, k3, k4, k5, k6, k7, k8;
    Left_straight_flag = 0;   // ��ʼ����ֱ�߱�־
    if(Endline>17)return;
    // ���㲻ͬ�����б�������ж�ֱ�߶�
    k1=((float)left[Endline+80]-(float)left[Endline+50])/30;  // �ײ�����б��
    k2=((float)left[Endline+70]-(float)left[Endline+60])/10;  // �в�����б��
    k3=((float)left[Endline+60]-(float)left[Endline+10])/50;   // �������ײ�б��
    k4=((float)left[Endline+30]-(float)left[Endline+10])/20;   // �������в�б��
    k5=((float)left[Endline+80]-(float)left[Endline+70])/10;   // ��ײ�����б��
    k6=((float)left[Endline+50]-(float)left[Endline+30])/20;   // ���²�����б��
    k7=((float)left[Endline+40]-(float)left[Endline+15])/25;   // ���ϲ�����б��
    k8=((float)left[Endline+20]-(float)left[Endline+10])/10;   // �ϲ�ϸ������б��
    // �ж��Ƿ�Ϊֱ�� - ʹ�ø���б�ʵ���о�ȷ�ж�
    if(absolute(k1-k2)<0.3&&absolute(k2-k3)<0.3&&absolute(k3-k4)<0.3&&
       absolute(k4-k5)<0.3&&absolute(k5-k6)<0.3&&absolute(k6-k7)<0.3&&
       absolute(k7-k8)<0.3&&absolute(k1-k8)<0.5&&
       k1!=0&&k2!=0&&k3!=0&&k4!=0&&k5!=0&&k6!=0&&k7!=0&&k8!=0){
            Left_straight_flag=1;
    }
}
//=================================================ʮ��ʶ��=======================================================
void crossroad(void){

    //��ǰʮ�ִ���
    if(annulus_R_Flag==0&&annulus_L_Flag==0&&Lost_left_Flag==1&&Lost_point_L_scan_line>60&&Lost_point_R_scan_line>60&&Upper_right_inflection_Flag==1&&Upper_left_inflection_Flag==1&&Crossroad_Flag==0&&Crossroad_memory==0&&Endline<5){
        Crossroad_Flag=1;
        Crossroad_memory=1;
    }
    if(Crossroad_Flag==1){

        //״̬1 ����ʮ�֣����������������յ�
        if(Crossroad_memory==1){

            //����������б�ʲ���
            // ���ݹյ�������в���
            if(Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1){
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X , Lower_left_inflection_Y );
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,  Lower_right_inflection_X, Lower_right_inflection_Y);
            }
            else{
                 // Ĭ�ϲ���λ��
                Addingline( 1, 77,20, 37,83);
                Addingline( 2, 128,29, 160, 99);
            }

            // �������йյ�����
            if ( Lower_right_inflection_Flag==1&&Lower_left_inflection_Flag==0)
            {
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 2 , 118 );
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,  Lower_right_inflection_X, Lower_right_inflection_Y);
            }
            if ( Lower_right_inflection_Flag==0&&Lower_left_inflection_Flag==1)
            {
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X , Lower_left_inflection_Y );
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,  187, 118);
            }
            // û�йյ�ʱ�л�״̬
            if ( Lower_right_inflection_Flag==0&&Lower_left_inflection_Flag==0)
            {
                Crossroad_memory=2;
            }
        }

        //״̬2 �������Ϲյ�����Ϲյ㣬��������
        if(Crossroad_memory==2){
            if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1){
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 23, 118);
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, 173,118);
//              Addingline2(1,Upper_left_inflection_X,Upper_left_inflection_Y);
//              Addingline2(2,Upper_right_inflection_X,Upper_right_inflection_Y);
           }
//           else{
//               Addingline(1,44,67,23,118);
//               Addingline(2,140,58,173,118);
//           }
            // �����йյ�Ĵ���
            if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==1){
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, 187,118);
            }
            if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==0){
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 2, 118);
            }
//           if(Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_left_inflection_Flag==1){
//               Crossroad_memory=3;
//           }
            // û�йյ�ʱ�˳�ʮ��״̬
            if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
                Crossroad_Flag=0;
                Crossroad_memory=0;
                return;
            }
            return;
       }

       //״̬3 ��ʮ������ʻ,�������յ㲹��
//       if(Crossroad_memory==3){
//           if(Upper_right_inflection_Flag==0&&Upper_left_inflection_Flag==0){
//               Addingline1(1,Lower_left_inflection_X,Lower_left_inflection_Y);
//               Addingline1(1,Lower_right_inflection_X,Lower_right_inflection_Y);
//           }
//           else{
//                          Addingline1( 1, 57,45);
//                          Addingline1( 2, 139,52);
//                      }
//           if(Upper_right_inflection_Flag==1&&Upper_left_inflection_Flag==1){
//               Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X , Lower_left_inflection_Y );
//               Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,  Lower_right_inflection_X, Lower_right_inflection_Y);
//                          }
//           else{
//               Addingline( 1, 77,20, 37,83);
//               Addingline( 2, 128,29, 160, 99);
//           if(Lower_right_inflection_Flag==0&&Lower_left_inflection_Flag==0){
//                          Crossroad_memory=4;
//                      }
//       }
//       }
       //״̬4 ���߳�ʮ��
//       if(Crossroad_memory==4){
//           if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==1){
//           //               Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 5, 118);
//           //               Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, 184,118);
//                          Addingline2(1,Upper_left_inflection_X,Upper_left_inflection_Y);
//                          Addingline2(2,Upper_right_inflection_X,Upper_right_inflection_Y);
//                      }
//           else{
//                          Addingline2(1,59,51);
//                          Addingline2(2,140,58);
//                      }
//           }
           //�˳�
//           if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
//               Crossroad_Flag=0;
//               Crossroad_memory=0;
//               return;
//           }

//
  }
}
/**
 * @brief ʮ�ֽ���������亯��
 * @param uint8(*image)[image_w]     �Ҷ�ͼ������
 * @param uint8 *l_border            ���Ե����
 * @param uint8 *r_border            �ұ�Ե����
 * @param uint16 total_num_l         ���Ե������
 * @param uint16 total_num_r         �ұ�Ե������
 * @param uint16 *dir_l              ���Ե��������
 * @param uint16 *dir_r              �ұ�Ե��������
 * @param uint16(*points_l)[2]       ���Ե����������
 * @param uint16(*points_r)[2]       �ұ�Ե����������
 * @return �޷���ֵ������洢��ȫ�ֱ�����
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                                         uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint16 i;
    uint8 break_num_l = 0;  // ���Եͻ�����к�
    uint8 break_num_r = 0;  // �ұ�Եͻ�����к�
    uint8 end_num_l = 0;    // ���Ե�ָ�����к�
    uint8 end_num_r = 0;    // �ұ�Ե�ָ�����к�
    uint8 start, end;       // ����������ֹ�к�
    float slope_l_rate = 0, intercept_l = 0;  // ֱ����ϵ�б�ʺͽؾ�
    // ��δ��⵽ʮ�ֽ���ʱ�����н���·���ж�
    if(cross_sum == 0)
    {
         // ������������Ե��ʧ����������ض����ʱ�ж�Ϊʮ�ֽ���
        if(((l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 60 ,80) == 1)
                || (l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 60 ,80) == 1)
                || (l_loss_judge(l_border,90 ,110) == 1 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 60 ,80) == 1))
                && sum_island == 0 // ȷ��û�йµ������
                )
        {
            cross_sum = 1; // ���Ϊʮ�ֽ�����״̬
        }
    }
     // ������ʮ�ֽ�����״̬ʱ
    if(cross_sum == 1)
    {
        // ������Ե�ķ���ͻ���ͻָ���
        for (i = 1; i < total_num_l; i++)
        {   
            // ���1�������4��Ϊ��4����Ե����ͻ�䣩
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                  /* ��һ�׶��жϱ�Ե�����Ƿ��4��Ϊ��4 */
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /* �ڶ��׶�ȷ���Ƿ�Ϊ���Ե������ͼ����ࣩ */
            {
                break_num_l = (uint8)points_l[i][1];  // ��¼ͻ�����к�
                break;
            }
            // ���2������ӷ�4��Ϊ4����ʾ��Ե����ָ�
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /* ��һ�׶��жϱ�Ե�����Ƿ�ӷ�4��Ϊ4 */
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
               /* �ڶ��׶�ȷ���Ƿ�Ϊ���Ե */
            {
                end_num_l = (uint8)points_l[i][1];// ��¼�ָ�����к�
            }
        }
        // ����ұ�Ե�ķ���ͻ���ͻָ��㣨�߼������Ե��ͬ��
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                   /* ��һ�׶��жϱ�Ե�����Ƿ�ͻ�� */
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /* �ڶ��׶�ȷ���Ƿ�Ϊ�ұ�Ե������ͼ���Ҳࣩ */
            {
                break_num_r = (uint8)points_r[i][1];// ��¼ͻ�����к�
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                      /* ��һ�׶��жϱ�Ե�����Ƿ�ָ� */
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                        /* �ڶ��׶�ȷ���Ƿ�Ϊ�ұ�Ե */
            {
                end_num_r = (uint8)points_r[i][1];// ��¼�ָ�����к�
            }
        }
        // ����ͻ���ͻָ��������������Ƿ�ת������һ״̬
        if ((end_num_l == 0 && end_num_r == 0)||(end_num_l >= 110 && end_num_r >= 110)
                ||(break_num_l && break_num_r == 0 && end_num_l == 0 && end_num_r)
                ||(break_num_r && break_num_l == 0 && end_num_r == 0 && end_num_l)
                )
        {
            cross_sum = 2;// ת����ʮ�ֽ������״̬
        }
        // �������Ե��ͻ��ͻָ���ʱ���޸����Ե
        if (break_num_l && break_num_r == 0 && end_num_l && end_num_r == 0)
        {
             // �������ֱ�ߵ�б�ʣ�k = ��y/��x��
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//��??k=y/x
             // �������ֱ�ߵĽؾࣨb = y - kx��
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//???b=y-kx
            // ��ֱ�߷����޸���Ե��x = (y - b)/k��
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
        // �����ұ�Ե��ͻ��ͻָ���ʱ���޸��ұ�Ե���߼������Ե��ͬ��
        if (break_num_r && break_num_l == 0 && end_num_r && end_num_l == 0)
         {
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//��??k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//???b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
         }
        // ��˫���Ե����ͻ��ͻָ���ʱ��ͬʱ�޸������Ե
        if (break_num_l && break_num_r && end_num_l && end_num_r)
        {
             // �޸����Ե
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//��??k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//???b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            // �޸��ұ�Ե
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//��??k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//???b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }

    }
    // ������ʮ�ֽ������״̬ʱ
    if(cross_sum == 2)
    {
        // ���¼�����Ե�ķ���ͻ��㣨����������ʼλ��)
        for (i = 10; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                  /* �жϱ�Ե�����Ƿ�ͻ�� */
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                 /* ȷ���Ƿ�Ϊ���Ե */
            {
                break_num_l = (uint8)points_l[i][1]-5;// ��¼ͻ����кŲ�΢��
                break;
            }
        }
        // ���¼���ұ�Ե�ķ���ͻ��㣨�߼������Ե��ͬ��
        for (i = 10; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                   /* �жϱ�Ե�����Ƿ�ͻ�� */
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /* ȷ���Ƿ�Ϊ�ұ�Ե */
            {
                break_num_r = (uint8)points_r[i][1]-5;// ��¼ͻ����кŲ�΢��
                break;
            }
        }
         // �ж��Ƿ���Ҫ�˳�ʮ�ֽ���״̬
        if((break_num_l == 0 && break_num_r == 0)||(break_num_l >= 115 && break_num_r >= 115)
                ||(l_loss_judge(l_border,90 ,110) == 0 && r_loss_judge(r_border,90 ,110) == 0))
        {
            cross_sum = 0;// �˳�ʮ�ֽ���״̬
        }
         // ʹ����С���˷����ֱ�ߣ������޸����Ե
        start = break_num_l - 10;  // �����ʼ��
        start = (uint8)limit_a_b(start, 0, image_h);  // ȷ����ʼ����Ч
        end = break_num_l - 5;     // �����ֹ��
        calculate_s_i(start, end, left, &slope_l_rate, &intercept_l);  // ��С���˷����
         // �����ֱ�������޸���Ե��ͼ��ײ�
        for (i = break_num_l - 5; i < image_h - 1 + 50; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//???
        }

        // ʹ����С���˷����ֱ�ߣ������޸��ұ�Ե���߼������Ե��ͬ��
        start = break_num_r - 10;  // �����ʼ��
        start = (uint8)limit_a_b(start, 0, image_h);  // ȷ����ʼ����Ч
        end = break_num_r - 5;     // �����ֹ��
        calculate_s_i(start, end, right, &slope_l_rate, &intercept_l);  // ��С���˷����
         // �����ֱ�������޸���Ե��ͼ��ײ�
        for (i = break_num_r - 5; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }

}
//=============================================Բ��===================================================
//================����ʶ��=====================

void roundabout_L(void){//OK,�Ϲյ�ʶ��
    uint8 detected = 0;  // ���μ����
    uint8 temp_x = 0, temp_y = 0; // ��ʱ����
    
    for(y=50;y>10;y--){
        if(my_abs(left[y]-left[y-1])<3&&my_abs(left[y]-left[y+1])>10&&left[y]>10&&Lost_left_Flag==1){
            //y+=4;
            detected = 1;
            temp_x = left[y];
            temp_y = y;
            break;
        }
    }
    
    // �˲�����
    if(detected) {
        roundabout_L_filter++;
        if(roundabout_L_filter >= ROUNDABOUT_FILTER_THRESHOLD) {
            roundabout_Flag = 1;
            roundabout_X = temp_x;
            roundabout_Y = temp_y;
            roundabout_L_filter = ROUNDABOUT_FILTER_THRESHOLD; // ��ֹ���
        } else {
            roundabout_Flag = 0;
        }
    } else {
        roundabout_L_filter = 0;
        roundabout_Flag = 0;
        roundabout_X = 0;
        roundabout_Y = 0;
    }
}

void roundabout_R(void){//OK,�Ϲյ�ʶ��
    uint8 detected = 0;  // ���μ����
    uint8 temp_x = 0, temp_y = 0; // ��ʱ����
    
    for(y=50;y>10;y--){
        if(my_abs(right[y]-right[y-1])<3&&my_abs(right[y]-right[y+1])>10&&right[y]<178&&Lost_right_Flag==1){
            //y+=4;
            detected = 1;
            temp_x = right[y];
            temp_y = y;
            break;
        }
    }
    
    // �˲�����
    if(detected) {
        roundabout_R_filter++;
        if(roundabout_R_filter >= ROUNDABOUT_FILTER_THRESHOLD) {
            roundabout_Flag = 1;
            roundabout_X = temp_x;
            roundabout_Y = temp_y;
            roundabout_R_filter = ROUNDABOUT_FILTER_THRESHOLD; // ��ֹ���
        } else {
            roundabout_Flag = 0;
        }
    } else {
        roundabout_R_filter = 0;
        roundabout_Flag = 0;
        roundabout_X = 0;
        roundabout_Y = 0;
    }
}

/*!
  * @brief    ����roundabout_L/R�˲���
  * @param    ��
  * @return   ��
  * @note     ��Բ��״̬�ı�ʱ���ã������˲�������
  */
void Roundabout_ResetFilter(void)
{
    roundabout_L_filter = 0;
    roundabout_R_filter = 0;
}

//================�����յ�ʶ��=====================
void Exit_loop_L_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //��Բ����ʶ���ұ߹յ�
    for(i=110;i>Endline+10;i--){
        if(right[i+4]-right[i]>0&&right[i-4]-right[i]>4&&right[i]<right[i+3]&&right[i]<right[i-3]){
            Exit_loop_Flag=1;
            Exit_loop_X=right[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

void Exit_loop_R_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //��Բ����ʶ����߹յ�
    for(i=110;i>Endline+10;i--){
        if(left[i]-left[i+4]>0&&left[i]-left[i-4]>4&&left[i]>left[i+3]&&left[i]>left[i-3]){
            Exit_loop_Flag=1;
            Exit_loop_X=left[i];
            Exit_loop_Y=i;
            return;
        }
    }
}
/**
 * @brief �ж����ұ߽��ɫ�����Ƿ����
 * @param void
 * @return ����� (1:��ɫ��������, 0:����)
 * @note ��Endline+5��ʼ��⣬��������ֵ��10
 */
int check_border_white_excess(void)
{
    int left_white_count = 0;   // ��߽��ɫ���ؼ���
    int right_white_count = 0;  // �ұ߽��ɫ���ؼ���
    int total_check_rows = 0;   // ����������
    
    // ��Endline+5��ʼ����ɨ�赽ͼ��ײ�
    for(int i = Endline + 5; i < image_h - 5; i++)
    {
        total_check_rows++;
        
        // �����߽�λ�õ����أ���3�У�
        if(imag[i][3] == White)
        {
            left_white_count++;
        }
        
        // ����ұ߽�λ�õ����أ���185�У�
        if(imag[i][185] == White)
        {
            right_white_count++;
        }
    }
    
    // ��̬������ֵ����5��ʼ�����ӵ�10
    int threshold_base = 15;                    // ������ֵ
    int threshold_increment = 5;               // ����
    int max_threshold = 60;                    // �����ֵ
    
    // ���ݼ���������㵱ǰ��ֵ
    int current_threshold = threshold_base;
    if(total_check_rows > 20)
    {
        current_threshold = threshold_base + ((total_check_rows - 20)*3 / 4);
        if(current_threshold > max_threshold)
        {
            current_threshold = max_threshold;
        }
    }
    
    // �жϰ�ɫ�����Ƿ񳬹���ֵ
    if(left_white_count >= current_threshold && right_white_count >= current_threshold)
    {
        return 1;  // ��ɫ��������
    }
    
    return 0;  // ����
}
/**
* @brief ��С���˷�
* @param uint8 begin                ��ʼ��
* @param uint8 end                  ������
* @param uint8 *border              ָ����Ҫ����б�ʵı߽��׵�ַ
* @see CTest       Slope_Calculate(start, end, border);//б��
* @return ����ֵ˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast; // ��̬����������һ�μ�����

    // ��������ۼӺ�
    for (i = begin; i < end; i++)
    {
        xsum += i;               // xֵ�ۼ�
        ysum += border[i];       // yֵ�ۼ�
        xysum += i * border[i];  // x*y�ۼ�
        x2sum += i * i;          // xƽ���ۼ�

    }
    // ����б��(��С���˷���ʽ)
    if ((end - begin)*x2sum - xsum * xsum) // �жϷ�ĸ�Ƿ�Ϊ0
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;// ������Ч���
    }
    else
    {
        result = resultlast;// ʹ����һ�εĽ��
    }
    return result;
}

/**
* @brief ����б�ʽؾ�
* @param uint8 start                ��ʼ��
* @param uint8 end                  ������
* @param uint8 *border              ָ����Ҫ����б�ʵı߽�
* @param float *slope_rate          ����б�ʵĵ�ַ
* @param float *intercept           ���ؽؾ�ĵ�ַ
* @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return ����ֵ˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;        // i��ѭ����������num����Ч������
    uint16 xsum = 0, ysum = 0; // xsum/ysum��x��y������ۼӺ�
    float y_average, x_average; // x_average/y_average��x��y�����ƽ��ֵ
    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;        // x�����ۼӣ�i ��Ϊ x ֵ��
        ysum += border[i];// y�����ۼӣ�border[i] ��Ϊ y ֵ��
        num++;            // ��Ч����������
    }

    // ����ƽ��ֵ
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);

    }

     // ����б�ʺͽؾ�
    *slope_rate = Slope_Calculate(start, end, border);  // ����б�ʼ��㺯��
    *intercept = y_average - (*slope_rate) * x_average;  // ����ؾ�
}
uint8 broken_line_x = 0;
uint8 broken_line_y = 0;
/**
 * @brief �ϵ��⺯��
 * @param uint8 dir        ��������0�����ϵ���������1�����µ���������
 * @param uint8 start      ������ʼ�к�
 * @param uint8 *border    ��Եλ�������ָ�루����Ϊ�кţ�ֵΪ�кţ�
 * @see CTest       broken_line_judge(1,90,120,l_border);
 * @return �޷���ֵ������洢��ȫ�ֱ�����
 */
void broken_line_judge(int dir, uint8 start, uint8 end, uint8 *border)
{
     // ��start�п�ʼ����������������end��
     if(dir==0){
    for (uint16 i = (uint16)start; i <= (uint16)end; i++)
    {
        // ���㵱ǰ������һ�еı�Եλ�ò�ֵ���ж��Ƿ񳬹���ֵ��4���أ�
        for(uint16 i = (uint16)start ;i <= (uint16)end; i++)
        {
            if(my_abs(border[i] - border[i-1]) >= 4)
            {
                // ��¼��һ�еı�Եλ����Ϊ�ϵ����꣨��Ϊͻ�䷢����i��i-1��֮�䣩
                broken_line_x = border[i-1];  // �ϵ��x���꣨�кţ�
                broken_line_y = (uint8)(i-1); // �ϵ��y���꣨�кţ�
                break;  // �ҵ���һ���ϵ�������˳�ѭ��
            }

             // �����������������У�end-2��ʱ����δ�ҵ��ϵ�����Ϊ��Ч
            if(i == (uint16)(end-2))
            {
                broken_line_x = -1;  // ��Чx����
                broken_line_y = -1;  // ��Чy����
                break;
            }
        }
    }
}
    if(dir == 1)
    {
        // ��end�п�ʼ����������������start��
        for(uint16 i = (uint16)end;i >= (uint16)start;i--)
        {
             // ���㵱ǰ������һ�еı�Եλ�ò�ֵ���ж��Ƿ񳬹���ֵ��4���أ�
            if(my_abs(border[i] - border[i+1]) >= 4)
            {
                // ��¼��һ�еı�Եλ����Ϊ�ϵ����꣨��Ϊͻ�䷢����i��i+1��֮�䣩
                broken_line_x = border[i+1];  // �ϵ��x���꣨�кţ�
                broken_line_y = (uint8)(i+1); // �ϵ��y���꣨�кţ�
                break;  // �ҵ���һ���ϵ�������˳�ѭ��
            }
            // �����������������У�start+2��ʱ����δ�ҵ��ϵ�����Ϊ��Ч
            if(i == (int)(start+2))
            {
                broken_line_x = -1;
                broken_line_y = -1;
                break;
            }
        }
    }
}
/**
 * @brief �������жϺ���
 * @param uint8* hightest            �����Ч��ָ��
 * @param uint8 *l_border            ���Ե����ĵ�ַ
 * @param uint8 *r_border            �ұ�Ե����ĵ�ַ
 * @param uint16 total_num_l         ���Ե�ܵ���
 * @param uint16 total_num_r         �ұ�Ե�ܵ���
 * @param uint16 *dir_l              ���Ե��������ĵ�ַ
 * @param uint16 *dir_r              �ұ�Ե��������ĵ�ַ
 * @param uint16(*points_l)[2]       ���Ե����������ĵ�ַ
 * @param uint16(*points_r)[2]       �ұ�Ե����������ĵ�ַ
 * @param uint16 *l_index            ���Ե��������ĵ�ַ
 * @param uint16 *r_index            �ұ�Ե��������ĵ�ַ
 * @see CTest      monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
 * @return ����ֵ˵��
 * 1 ����
 * 0 �ǵ���
 */
int monotonicity_l = -1;  // ���Ե�����Խ����ȫ�ֱ�����
int monotonicity_r = -1;  // �ұ�Ե�����Խ����ȫ�ֱ�����
void monotonicity_line(uint8* hightest, uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
        uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],uint16 *l_index,uint16 *r_index)
{
    uint16 i;
    uint16 total_l = 0;
    uint16 total_r = 0;      // ���ұ�Ե���ܵ���
    uint16 arr_l = 0;
    uint16 arr_r = 0;      // ���ұ�Ե�����ض����������ĵ���
    uint16 abb_l = 0;
    uint16 abb_r = 0;      // ���ұ�Ե���ڵ�仯������ֵ�Ĵ���
    uint16 acc_l = 0;
    uint16 acc_r = 0;      // ���ұ�Ե�����仯������ֵ�Ĵ���
    // ���Ե�Ĵ���
    for (i = 1; i < total_num_l-20; i++)
    {
        // �ҵ���Ե����߽��غ����к�С�ڵ���20�ĵ㣬����ѭ��
        if(points_l[i][0] == (uint16)l_border[points_l[i][1]] && points_l[i][1] <= 20)
            break;
            
        // ͳ�Ʒ���Ϊ4��5��6�ĵ㣨��Щ������ܴ����Ե�������ԽϺã�
        if ((dir_l[i] == 4) || (dir_l[i] == 5) || (dir_l[i] == 6))
        {
            arr_l++;
        }
        total_l++;
    }
  // ����󲿷ֵ�ķ������������arr_l >= total_l-5��
    if(arr_l>=(total_l-5))
    {
        // ��������Ч�д��ڵ���20���������Ч�п�ʼ���
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                // ͳ�������仯����5�Ĵ��������ܱ�ʾ��Ե��������
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;  // �ǵ���
                    break;
                }
 // ͳ�Ʊ�Եλ�ñ仯����2�Ĵ��������ܱ�ʾ��Ե������
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;  // �ǵ���
                    break;
                }
                
                // �����鵽��105�л�û�з������⣬��Ϊ�ǵ�����
                if(i == 105)
                {
                    monotonicity_l = 1;  // ����
                    break;
                }
            }
        }
        else// ��������Ч��С��20���ӵ�20�п�ʼ���
            for (int i = 20; i < image_h-2; i++)
            {
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                }
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                }
                if(i == 105)
                {
                    monotonicity_l = 1;
                    break;
                }
            }
    }
    else monotonicity_l = 0;// �������ͳ�Ʋ�����������ֱ���ж�Ϊ�ǵ���
    // �ұ�Ե�Ĵ����߼������Ե��ͬ��
    for (i = 1; i < total_num_r-20; i++)
    {
        if(points_r[i][0] == (uint16)r_border[points_r[i][1]] && points_r[i][1] <= 20)
            break;
        if ((dir_r[i] == 4) || (dir_r[i] == 5) || (dir_r[i] == 6))
        {
            arr_r++;
        }
        total_r++;
    }
       // �ұ�Ե�������жϣ��߼������Ե��ͬ��
    if(arr_r>=(total_r-5))
    {
          // �������Ч�п�ʼ���¼���Ե������
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                // ��������仯�Ƿ񳬹���ֵ�����ܷ�ӳ��Ե��Ծ��
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /* �߼��������������仯��5�Ĵ�����5�Σ���Ϊ��Ե���������� */
                }
                // ����Եλ�ñ仯�Ƿ񳬹���ֵ�����ܷ�ӳ��Ե������
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /* �߼��������б�Եλ�ñ仯��2�Ĵ�����5�Σ���Ϊ��Եλ�ò����� */
                }
                // ǿ����ֹ��������鵽��105��ʱ�ж�Ϊ����
                if(i == 105)
                   /* �߼�����ǰ105��δ�����쳣����Ϊ��Ե���嵥�� */
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
        else
        {
            // �������Ч��<20ʱ���ӵ�20�п�ʼ��飨���Զ������ܵ���������
            for (int i = 20; i < image_h-2; i++)
            {
                // ����һ��֧��ͬ�������仯���
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    
                }
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                }
                if(i == 105)
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
    }
    else monotonicity_r = 0;
}
//================��Բ��ʶ��=====================

uint8_t UpdownSideGet(void)
{
    uint8_t i = 0, j = 0;
    uint8_t last = image_h/2;

    imageOut[0][image_w-1] = 0;
    imageOut[1][image_w-1] = image_h-1;
     //��ͼ���м���    ���е���     ���е���      ɨ��

    //�����м䵥����һ�е����±���
    for(i = last; i >= 5; i--)
    {
        if(!imag[i][image_w/2])
        {
            imageOut[up][image_w/2] = i;
            break;
        }
    }

    for(i = last; i < image_h-5; i++)
    {
        if(!imag[i][image_w/2])
        {
            imageOut[down][image_w/2] = i;
            break;
        }
    }
    //�����е����±���
    //���е���
    for(i = image_w/2-1; i > 2; i--)//����ÿһ��
    {
        imageOut[up][i] = 0;
        imageOut[down][i] = image_h-1;

        for(j = imageOut[0][i+1] + 30; j > 5; j--)//һ���е�ɨ��ÿ��  �����е�����+10��ʼ����ɨ��
        {
            if(!imag[j][i])
            {
                imageOut[up][i] = j;
                break;
            }
        }
        for(j = imageOut[1][i+1] - 30; j < image_h-5; j++)
        {
            if(!imag[j][i])
            {
                imageOut[down][i] = j;
                break;
            }
        }
    }

    //���е���
    for(i = image_w/2+1; i < image_w-2; i++)
        {
            imageOut[up][i] = 0;
            imageOut[down][i] = image_h-1;

            for(j = imageOut[0][i-1] + 30; j > 5; j--)
            {
                if(!imag[j][i])
                {
                    imageOut[up][i] = j;
                    break;
                }
            }
            for(j = imageOut[1][i-1] - 30; j < image_h-5; j++)
            {
                if(!imag[j][i])
                {
                    imageOut[down][i] = j;
                    break;
                }
            }
        }
    return 0;
}
/*!
  * @brief    �ж����ұ����Ƿ���ڻ���
  * ����� index Բ���Ķ���λ��
  * @param    imageInput �� ��ֵͼ����Ϣ
  * @param    imageOut   �� ��������
  * @param    status     �� 1�������  2���ұ���
  * @param    num        �� Բ���Ĵ�С �õ�����ʾ  ������N����  ����N������
  * @return   1 �л���    0  û����

  */
 uint8_t inc = 0, dec = 0, n = 0,inc_y=0;
uint8_t RoundaboutGetArc(uint8_t status, uint8_t num)
{
    int i = 0;
    uint8_t arc_detected = 0;  // ���μ����
    inc = 0; dec = 0; n = 0;inc_y=0;
    
    switch(status)
    {
      case 1:  // ����߼��
        for(i = Endline+10; i < 100; i++)
        {
        	//û�ж���
            if(imag[i][2]!=White && imag[i+1][2]!=White)
            {
                if(left_cicle[i]- left_cicle[i+1]< left_cicle[i+1]-left_cicle[i+2]&&left_cicle[i]>=left_cicle[i+1]&&left_cicle[i]>left_cicle[110])
                {
                    //inc_y=i;
                    inc++;
                    //inc+=n;
                    n=0;
                }
                if(left_cicle[i] < left_cicle[i+1])
                {
                    dec++;
                    //dec+=n;
                    n=0;
                }
                if(inc>9&&dec>1)
                {
                    arc_detected = 1;
                    break;
                }
                /* �л��� */
            }
        }
        
        // ������˲�����
        if(arc_detected)
        {
            roundabout_arc_filter_left++;
            if(roundabout_arc_filter_left >= ROUNDABOUT_ARC_FILTER_THRESHOLD)
            {
                roundabout_arc_filter_left = ROUNDABOUT_ARC_FILTER_THRESHOLD; // ��ֹ���
                return 1;
            }
        }
        else
        {
            roundabout_arc_filter_left = 0; // δ��⵽������
        }
        break;

      case 2:  // �ұ��߼��
        for(i = Endline+10; i < 100; i++)
        {
            //û�ж���
            if(imag[i][185]!=White && imag[i+1][185]!=White)
            {
                if(right_cicle[i+1]- right_cicle[i]< right_cicle[i+2]-right_cicle[i+1]&&right_cicle[i]<=right_cicle[i+1]&&right_cicle[i]<right_cicle[110])
                {
                    inc++;
                    inc_y=i;
                    //inc+=n;
                    n=0;
                }
                if(right_cicle[i] > right_cicle[i+1])
                {
                    dec++;
                    //dec+=n;
                    n=0;
                }

                /* �л��� */
                if(inc>9&&dec>1)
                {
                    arc_detected = 1;
                    break;
                }
            }
        }
        
        // �ұ����˲�����
        if(arc_detected)
        {
            roundabout_arc_filter_right++;
            if(roundabout_arc_filter_right >= ROUNDABOUT_ARC_FILTER_THRESHOLD)
            {
                roundabout_arc_filter_right = ROUNDABOUT_ARC_FILTER_THRESHOLD; // ��ֹ���
                return 1;
            }
        }
        else
        {
            roundabout_arc_filter_right = 0; // δ��⵽������
        }
        break;
    }

    return 0;
}

/*!
  * @brief    ����RoundaboutGetArc�˲���
  * @param    ��
  * @return   ��
  * @note     �ڻ���״̬�ı�ʱ���ã������˲�������
  */
void RoundaboutGetArc_ResetFilter(void)
{
    roundabout_arc_filter_left = 0;
    roundabout_arc_filter_right = 0;
}

/*!
  * @brief    �ж��ϱ����Ƿ񵥵�
  * @return   0��������or���� 1������������ 2�������ݼ�
  * @note
  * @see
  * @date     2021/11/30 ���ڶ�
  */
uint8_t RoadUpSide_Mono(void)
{
    UpdownSideGet();
    uint8_t i = 0, num = 0;

    for(i = 5; i < 183; i++)
    {
        if(imageOut[0][i] >= imageOut[0][i+1])
            num++;
        else
            num = 0;
        //if(my_abs(imageOut[0][i] - imageOut[0][i+1])>4)return 0;
        if (num >= (178)*6/7)
            return 1;
    }
    for(i = 5; i < 183; i++)
    {
        if(imageOut[0][i] <= imageOut[0][i+1])
            num++;
        else
            num = 0;
        //if(my_abs(imageOut[0][i] - imageOut[0][i+1])>4)return 0;
        if (num >= (178)*6/7)
            return 2;
    }
    return 0;
}
int count=0;
int round_size=35;
int yaw_cicle_flag=0;
float yaw_cicle=0;
void annulus_L(void){
    if(jump_position_flag != 0||
        Crossroad_memory!=0||
        Crossroad_Flag!=0||
        (annulus_R_memory!=0&&annulus_R_memory!=7)||
        BridgeState != SINGLE_BRIDGE_NOT_ACTIVE||
        jump_active
        )return;
    if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1&&Left_straight_flag==0&&annulus_L_memory==0
    ){
        get_left_cicle(data_stastics_l);
        if(RoundaboutGetArc(1, 10)){
            annulus_L_Flag=1;
            annulus_L_memory =2;
            if(mode!=stopworking){
                ips200_full(RGB565_PURPLE);
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,inc,3,"inc");
                show_string_value(12,dec,3,"dec");
                show_string_value_float(13,inc_y,3,3,"inc_y");
                show_string_value(14,0,3,"Recognize the cicle");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
        }
        else Addingline1( 1, Lower_left_inflection_X, Lower_left_inflection_Y);
    }
    if (annulus_L_memory == 2 )
    {
        roundabout_L();          //����
        get_left_cicle(data_stastics_l);
        // if(roundabout_Flag==1){
        //     Addingline( 1, roundabout_X, roundabout_Y,24 , 118 );
        // }
        if(roundabout_Flag==1&&!RoundaboutGetArc(1, 10)){
            annulus_L_memory = 3;
            if(mode!=stopworking){
                ips200_full(RGB565_BLUE);
                show_string_value(14,0,3,"start turn into the cicle");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,inc,3,"inc");
                show_string_value(12,dec,3,"dec");
                show_string_value_float(13,inc_y,3,3,"inc_y");

                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
        }
        else {
            //Addingline( 1, 70, 5,24 , 118 );
            return;
        }
    }
    if(annulus_L_memory == 3)
    {
        roundabout_L();          //����
        get_right_cicle(data_stastics_r);
        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
        if(roundabout_Flag==1&&roundabout_Y<25){
             Addingline( 2, roundabout_X,roundabout_Y,186 , 118);
         }
        else{
            Addingline( 2, 2,60,186 , 108);
        }
        if((roundabout_Flag==0&&Endline>20&&RoadUpSide_Mono()==2&&absolute(yaw_cicle-euler_angle.yaw)>20)||absolute(yaw_cicle-euler_angle.yaw)>60){
            annulus_L_memory = 4;
            if(mode!=stopworking){
                ips200_full(RGB565_BROWN);
                show_string_value(14,0,3,"in the cicle");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,roundabout_X,3,"roundabout_X");
                show_string_value(12,roundabout_Y,3,"roundabout_Y");
                show_string_value_float(13,Endline,3,3,"Endline");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
    if(annulus_L_memory == 4)
    {
        Exit_loop_L_inflection();

        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
        if((check_border_white_excess())&&absolute(yaw_cicle-euler_angle.yaw)>100){
            annulus_L_memory = 5;
            if(mode!=stopworking){
                ips200_full(RGB565_GRAY);
                show_string_value(14,0,3,"find the corner");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,euler_angle.yaw,3,"euler_angle.yaw");
                show_string_value(12,yaw_cicle,3,"yaw_cicle");
                show_string_value_float(13,inc_y,3,3,"inc_y");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
    if (annulus_L_memory == 5 )
    {
        get_right_cicle(data_stastics_r);
        get_left_cicle(data_stastics_l);
        Exit_loop_L_inflection();
        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
        if(Exit_loop_Flag==1){
        Addingline( 2, 3, Endline+3, Exit_loop_X, Exit_loop_Y);
        }
        else Addingline( 2, 3, Endline+3, 180, 110);
        if(absolute(yaw_cicle-euler_angle.yaw)>20){
            annulus_L_memory = 7;
            annulus_R_memory = 7;
            RoundaboutGetArc_ResetFilter(); // ����Բ�����߼���˲���
            Roundabout_ResetFilter(); // ����Բ���Ϲյ����˲���
            if(mode!=stopworking)ips200_full(RGB565_BLACK);
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
}

//================��Բ��ʶ��=====================
void annulus_R(void){
    if(jump_position_flag != 0||
        Crossroad_memory!=0||
        Crossroad_Flag!=0||
         (annulus_L_memory!=0&&annulus_L_memory!=7)||
        BridgeState != SINGLE_BRIDGE_NOT_ACTIVE||
        jump_active
        )return;
     if(Lost_right_Flag==1&&Lost_left_Flag==0&&Left_straight_flag==1&&Right_straight_flag==0&&annulus_R_memory==0
    ){
        get_right_cicle(data_stastics_r);
        if(RoundaboutGetArc(2, 10)){
            annulus_R_Flag=1;
            annulus_R_memory =2;
            if(mode!=stopworking){
                ips200_full(RGB565_RED);
                show_string_value(14,0,3,"Recognize the cicle");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,inc,3,"inc");
                show_string_value(12,dec,3,"dec");
                show_string_value_float(13,inc_y,3,3,"inc_y");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(right_cicle[i], i, RGB565_BLUE);
                }
            }
        }
         else if(Lower_right_inflection_Flag==1)
             Addingline1( 2, Lower_right_inflection_X, Lower_right_inflection_Y);
        //else Addingline( 2, 118, 5,164 , 118 );
    }
    if (annulus_R_memory == 2 )
    {
        roundabout_R();          //����
        get_right_cicle(data_stastics_r);
        // if(roundabout_Flag==1){
        //     Addingline( 2, roundabout_X, roundabout_Y,178 , 118 );
        // }
        if(roundabout_Flag==1&&!RoundaboutGetArc(2, 10)){
            if(mode!=stopworking){
                ips200_full(RGB565_GREEN);
                show_string_value(14,0,3,"start turn into the cicle");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,inc,3,"inc");
                show_string_value(12,dec,3,"dec");
                show_string_value_float(13,inc_y,3,3,"inc_y");

                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(right_cicle[i], i, RGB565_RED);
                }
            }
            annulus_R_memory = 3;
        }
        else 
        {
            //Addingline( 2, 118, 5,164 , 118 );
            return;
        }
    }
    if(annulus_R_memory == 3)
    {
        roundabout_R();          //����
        get_left_cicle(data_stastics_l);
        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
         if(roundabout_Flag==1&&roundabout_Y<20){
             Addingline( 1, roundabout_X,roundabout_Y,2 , 118);
         }
        else{
            Addingline( 1, 186,50,2 , 118);
        }
        if((roundabout_Flag==0&&Endline>20&&RoadUpSide_Mono()==1&&absolute(yaw_cicle-euler_angle.yaw)>20)||absolute(yaw_cicle-euler_angle.yaw)>60){
            annulus_R_memory = 4;
            if(mode!=stopworking){
                ips200_full(RGB565_BLUE);
                show_string_value(14,0,3,"in the cicle");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,roundabout_X,3,"roundabout_X");
                show_string_value(12,roundabout_Y,3,"roundabout_Y");
                show_string_value_float(13,Endline,3,3,"Endline");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(right_cicle[i], i, RGB565_RED);
                }
            }
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
    if(annulus_R_memory == 4)
    {
        Exit_loop_R_inflection();
        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
        if((check_border_white_excess())&&absolute(yaw_cicle-euler_angle.yaw)>100){
            annulus_R_memory = 5;
            if(mode!=stopworking)
            {
                ips200_full(RGB565_WHITE);
                show_string_value(14,0,3,"find the corner");
                show_string_value(9,Right_straight_flag,3,"Right_straight_flag");
                show_string_value(10,Left_straight_flag,3,"Left_straight_flag");
                show_string_value(11,euler_angle.yaw,3,"euler_angle.yaw");
                show_string_value(12,yaw_cicle,3,"yaw_cicle");
                show_string_value_float(13,inc_y,3,3,"inc_y");
                ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                for (int i = Endline; i < image_h-1; i++)
                {
                    //  middle[i] = (left[i] + right[i]) >> 1;//������
                    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
    if (annulus_R_memory == 5 )
    {
        get_left_cicle(data_stastics_l);
        get_right_cicle(data_stastics_r);
        Exit_loop_R_inflection();
        if(yaw_cicle_flag==0){
            yaw_cicle=euler_angle.yaw;
            yaw_cicle_flag=1;
        }
        if(Exit_loop_Flag==1){
        Addingline( 1, 185, Endline+3, Exit_loop_X, Exit_loop_Y);
        }
        else Addingline( 1, 185, Endline+3, 8, 110);
        if(absolute(yaw_cicle-euler_angle.yaw)>20){
            annulus_L_memory = 7;
            annulus_R_memory = 7;
            RoundaboutGetArc_ResetFilter(); // ����Բ�����߼���˲���
            Roundabout_ResetFilter(); // ����Բ���Ϲյ����˲���
            if(mode!=stopworking)ips200_full(RGB565_BLACK);
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
}



//---------------------------------------�������---------------------------------
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2)
 {
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i <=endline1; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        for (i = startline2; i <=endline2; i++)
        {
            sumX += i;
            sumY += middle[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x��ƽ��ֵ
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y��ƽ��ֵ
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (middle[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += left[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x��ƽ��ֵ
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y��ƽ��ֵ
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (left[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //�������
    {
        /**����sumX sumY**/
        for (i = startline1; i <= endline1; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumX += i;
            sumY += right[i];
        }
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x��ƽ��ֵ
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y��ƽ��ֵ
        for (i = startline1; i <= endline1; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i <= endline2; i++)
        {
            sumUp += (right[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
//------------------------------�ж�ֱ��б���Ƿ���ͬ------------------------------
int Judgment_symbol(float x, float y)
{
    int a;
    a = 0;
    if (x < 0 && y < 0) a = 1;
    if (x >= 0 && y >= 0) a = 1;
    return a;
}
//-----------------------------------------����-----------------------------------

//�������յ�Ĳ��ߺ���
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;

    // ֱ�� x = ky + b
    float k = 0;
    float b = 0;
    switch(choice)
    {
      case 1://����
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                left[y] = (uint8)(k * y + b);
            }
            break;
        }

      case 2://�Ҳ���
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                right[y]= (uint8)(k * y + b);

            }
            break;
        }

      case 3://�в���
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = Endline+10; y < endY; y++)
            {
                middle[y] = (uint8)(k * y + b);
            }
            break;
        }

    }
}


void Addingline1( uint8 choice, uint8 startX, uint8 startY)    //�����յ���б�������ӳ�
{

    // ֱ�� x = ky + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://����
        {

            k = (float)(((float)left[Lower_left_inflection_Y+1] - (float)left[Lower_left_inflection_Y+5]) /(-4));
            b = (float)((float)left[Lower_left_inflection_Y+5]- (float)(Lower_left_inflection_Y+5) * k);

            for(y = startY; y >(Endline+20); y--)
            {

             temp = (int)(k* y + b);
             if(temp<180&&temp>10){
                 left[y]=temp;
             }
            }
            break;
        }

      case 2://�Ҳ���  ������
      {

           k = (float)(((float)right[Lower_right_inflection_Y+1] - (float)right[Lower_right_inflection_Y+5]) /(-4));
           b = (float)((float)right[Lower_right_inflection_Y+5]- (float)(Lower_right_inflection_Y+5) * k);

           for(y = startY; y >(Endline+20); y--)
           {

            temp = (int)(k* y + b);
            if(temp<180&&temp>10){
                right[y]=temp;
            }
           }
           break;
       }

      case 3://�в���
      {
           // ʹ�����ұ߽��ƽ��ֵ���������ߵ�б��
           float left_k = (float)(((float)left[Lower_left_inflection_Y+1] - (float)left[Lower_left_inflection_Y+5]) /(-4));
           float right_k = (float)(((float)right[Lower_right_inflection_Y+1] - (float)right[Lower_right_inflection_Y+5]) /(-4));
           
           // ȡ����б�ʵ�ƽ��ֵ��Ϊ����б��
           k = (left_k + right_k) / 2;
           
           // ʹ����ʼ�����ؾ�
           b = (float)startX - (float)startY * k;

           for(y = startY; y >(Endline+20); y--)
           {
            temp = (int)(k* y + b);
            if(temp<180&&temp>10){
                middle[y]=temp;
            }
           }
           break;
       }
    }
}

void Addingline2( uint8 choice, uint8 startX, uint8 startY)   //�ҵ��Ϲյ���б����������
{

    // ֱ�� x = k*y + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://����
        {

            k = (float)(((float)left[Upper_left_inflection_Y-2] - (float)left[Upper_left_inflection_Y-5]) /3);
            b = (float)((float)left[Upper_left_inflection_Y-5]- (float)(Upper_left_inflection_Y-5) * k);

            for(y = startY; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<185&&temp>2){
                 left[y]=temp;
             }

            }
            break;
        }

     case 2://�Ҳ���  ������
        {

            k = (float)(((float)right[Upper_right_inflection_Y-2] - (float)right[Upper_right_inflection_Y-5]) /3);
            b = (float)((float)right[Upper_right_inflection_Y-5]- (float)(Upper_right_inflection_Y-5) * k);
            for(y = startY; y<(image_h-3); y++)
            {

             temp = (int)(k* y + b);
             if(temp<185&&temp>2){
                 right[y]=temp;
             }

            }
            break;
        }

    }
}
/**
* @brief ������״̬��⺯��
* @param uint8(*image)[image_w]     ��ֵ��ͼ��
* @param uint8 *l_border            ��߽�����ָ��
* @param uint8 *r_border            �ұ߽�����ָ��
*  @see CTest       cross_stop(image,l_border,r_border);
* @return ����˵��
*     -<em>false</em> ���ʧ��
*     -<em>true</em> ���ɹ�
 */
void zebra_crossing(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border)
{
    if(jump_position_flag != 0||
        Crossroad_memory!=0||
        Crossroad_Flag!=0||
        (annulus_R_memory!=0&&annulus_R_memory!=7)||
        (annulus_L_memory!=0&&annulus_L_memory!=7)||
        BridgeState != SINGLE_BRIDGE_NOT_ACTIVE||
        Endline>5||
        Lost_left_Flag==1||
        Lost_right_Flag==1
        )return;
    uint8 start_point  = 85;  // �����ʼ�У���ֱλ�ã�
    uint8 end_point = 105;    // �������У���ֱλ�ã�
    // ��ʼ��������
    int banmaxian_kuandu = 0;  // �����߿�ȣ�������ɫ������
    int banmaxian_hangshu = 0; // ��Ч����������
    int banmaxian_geshu = 0;   // ��ǰ�м�⵽�ĺ�ɫ��������
    zebra_crossing_flag=0;

     // ������������ɨ�裨end_point �� start_point��
    for (uint16 y = end_point; y >= start_point; y--)
    {
        // �����ұ߽�֮�����ˮƽɨ��
        for (int x = (int)l_border[y]; x <= (int)r_border[y]; x++)
        {
            int baidian_heng=0;  // �洢��ɫ���ˮƽ���꣨���ڼ���������ȣ�

            // ���Ӱ�ɫ����ɫ������㣨�����أ�
            // image[y][x]�ǵ�ǰ���أ�image[y][x-1]��ǰһ������
            if (image[y][x] == 0 && image[y][x-1] == 255)
            {
                // ������㿪ʼ����ɨ�裬Ѱ����һ���Ӻ�ɫ����ɫ������㣨�½��أ�
                for(int a=x+1;a<x+15;a++)
                {
                    if(image[y][a-1] == 0 && image[y][a] == 255)
                    {
                        baidian_heng = a;  // ��¼�½��ص�x����
                        break;  // �ҵ��������˳��ڲ�ѭ��
                    }
                }
                // �����ɫ�����Ŀ�ȣ����������֮��ľ��룩
                banmaxian_kuandu = baidian_heng - x;
                // �ж��Ƿ�Ϊ��Ч�����ߣ������4-8����֮�䣩
                if (banmaxian_kuandu >= 4 && banmaxian_kuandu <= 8)
                {
                    banmaxian_geshu++;  // ��Ч��������+1
                    banmaxian_kuandu = 0;  // ���ÿ�ȼ�������׼����һ���������
                }
                //�����ߵĿ�Ȳ���4~8֮��������Ч������
                else
                {
                   // ��Ч��ȣ����ü�����
                    banmaxian_kuandu = 0;
                }
            }
        }

        // �жϵ�ǰ���Ƿ�Ϊ��Ч�������У�4-9����ɫ������
        if (banmaxian_geshu >= 4 && banmaxian_geshu <= 9)
        {
            banmaxian_hangshu++;// ��Ч����+1
        }
    }
    // �ж��Ƿ�����ʮ���߼������
     if(banmaxian_hangshu >= 4  // ������4�м�⵽��Ч������
        )  // û��ʮ�����ۻ����
    {
        // ����ͣ��λ�ñ�ǣ�1��ʾ��⵽ʮ���ߣ�
        pid1_walk.run_speed=0;
        zebra_crossing_flag=1;
    }
}

/**
* @brief �����ٶȼ�����ʱ����
* @param int16  speed       �ٶ�
* @param int    kp          �ٶ�ת��Ϊʱ��ı��������ڵ���
* @return ��ʱʱ��
 */
int jump_delay(int16 speed,int kp)
{
    int time=speed*kp;
    //�����һ���޷����ж�
    //if(time>max)return max;
    //else if(time<min)return min;
    //else
    return time;

}

// б�ʼ��������ֵ
#define SLOPE_CALC_HEIGHT_LIMIT   110      // ����߶�����
#define SLOPE_CALC_WIDTH_LIMIT    170      // ����������
#define SLOPE_DIFF_THRESHOLD      1.5f     // б�ʲ�����ֵ
#define CENTER_ZONE_MIN           85       // ����������Сֵ
#define CENTER_ZONE_MAX           103      // �����������ֵ
#define MIN_VALID_POINTS          3        // ������Ч����
#define SAMPLE_INTERVAL           3        // �������
#define MAX_COLLECT_POINTS        8        // ����ռ�����

/**
 * @brief �������ұ߽���ȶ�б�ʣ��ų�ͻ��㣩
 * @param l_border ��߽�����
 * @param r_border �ұ߽�����
 * @param highest ��ߵ�λ��
 * @param left_slope ���ص���߽�б��ָ��
 * @param right_slope ���ص��ұ߽�б��ָ��
 * @return ��Ч�Ա�־ (1:���߶���Ч, 0:��Ч)
 * @note �����߽�����x������3�ĵ��������һ���ߣ���ֵ����94������ô�������б��
 *       �Ҳ�ͬ���������������������ͻ�������������ͻ�����������б��
 */
int calculate_border_slopes(uint8 *l_border, uint8 *r_border, uint8 highest, float *left_slope, float *right_slope)
{
    *left_slope = 1.0f;   // ��Чʱ����Ϊ1
    *right_slope = 1.0f;  // ��Чʱ����Ϊ1
    
    int left_valid = 0, right_valid = 0;
    
    // ��߽�б�ʼ���
    int left_valid_points = 0;
    float left_sum_x = 0, left_sum_y = 0, left_sum_xy = 0, left_sum_x2 = 0;
    
    // �ӵײ�����ɨ�裬Ѱ����������Ч�㣬������110�߶���
    for(int y = SLOPE_CALC_HEIGHT_LIMIT; y >= highest + 10 && y >= SAMPLE_INTERVAL * 2; y -= SAMPLE_INTERVAL)
    {
        if(y >= SAMPLE_INTERVAL * 2 && y <= SLOPE_CALC_HEIGHT_LIMIT - SAMPLE_INTERVAL)
        {
            int x1 = l_border[y + SAMPLE_INTERVAL * 2];  // ��ǰ���·�6��
            int x2 = l_border[y + SAMPLE_INTERVAL];      // ��ǰ���·�3��
            int x3 = l_border[y];                        // ��ǰ��
            
            // �ų��������򸽽��ĵ�
            if(x1 >= CENTER_ZONE_MIN && x1 <= CENTER_ZONE_MAX) continue;
            if(x2 >= CENTER_ZONE_MIN && x2 <= CENTER_ZONE_MAX) continue;
            if(x3 >= CENTER_ZONE_MIN && x3 <= CENTER_ZONE_MAX) continue;
            
            // �ų����Եı߽綪ʧ�㣬������170�����
            if(x1 <= 5 || x2 <= 5 || x3 <= 5) continue;
            if(x1 >= SLOPE_CALC_WIDTH_LIMIT || x2 >= SLOPE_CALC_WIDTH_LIMIT || x3 >= SLOPE_CALC_WIDTH_LIMIT) continue;
            
            // ��������Ƿ�������ߣ�б�ʱ仯����
            float slope1 = (float)(x2 - x1) / (float)SAMPLE_INTERVAL;
            float slope2 = (float)(x3 - x2) / (float)SAMPLE_INTERVAL;
            
            // �������б��������Ϊ��ֱ�߶�
            if(fabs(slope1 - slope2) <= SLOPE_DIFF_THRESHOLD)
            {
                // ʹ����С���˷��ۻ�����
                left_sum_x += y;
                left_sum_y += x3;
                left_sum_xy += y * x3;
                left_sum_x2 += y * y;
                left_valid_points++;
                
                // ����ҵ��㹻��ĵ��ֹͣ
                if(left_valid_points >= MAX_COLLECT_POINTS) break;
            }
        }
    }
    
    // ������߽�б��
    if(left_valid_points >= MIN_VALID_POINTS)
    {
        float denominator = left_valid_points * left_sum_x2 - left_sum_x * left_sum_x;
        if(fabs(denominator) > 0.001f)
        {
            *left_slope = (left_valid_points * left_sum_xy - left_sum_x * left_sum_y) / denominator;
            left_valid = 1;
        }
    }
    
    // �ұ߽�б�ʼ��㣨�߼����ƣ�
    int right_valid_points = 0;
    float right_sum_x = 0, right_sum_y = 0, right_sum_xy = 0, right_sum_x2 = 0;
    
    for(int y = SLOPE_CALC_HEIGHT_LIMIT; y >= highest + 10 && y >= SAMPLE_INTERVAL * 2; y -= SAMPLE_INTERVAL)
    {
        if(y >= SAMPLE_INTERVAL * 2 && y <= SLOPE_CALC_HEIGHT_LIMIT - SAMPLE_INTERVAL)
        {
            int x1 = r_border[y + SAMPLE_INTERVAL * 2];
            int x2 = r_border[y + SAMPLE_INTERVAL];
            int x3 = r_border[y];
            
            // �ų��������򸽽��ĵ�
            if(x1 >= CENTER_ZONE_MIN && x1 <= CENTER_ZONE_MAX) continue;
            if(x2 >= CENTER_ZONE_MIN && x2 <= CENTER_ZONE_MAX) continue;
            if(x3 >= CENTER_ZONE_MIN && x3 <= CENTER_ZONE_MAX) continue;
            
            // �ų����Եı߽綪ʧ�㣬������170�����
            if(x1 <= 5 || x2 <= 5 || x3 <= 5) continue;
            if(x1 >= SLOPE_CALC_WIDTH_LIMIT || x2 >= SLOPE_CALC_WIDTH_LIMIT || x3 >= SLOPE_CALC_WIDTH_LIMIT) continue;
            
            // ��������Ƿ��������
            float slope1 = (float)(x2 - x1) / (float)SAMPLE_INTERVAL;
            float slope2 = (float)(x3 - x2) / (float)SAMPLE_INTERVAL;
            
            if(fabs(slope1 - slope2) <= SLOPE_DIFF_THRESHOLD)
            {
                right_sum_x += y;
                right_sum_y += x3;
                right_sum_xy += y * x3;
                right_sum_x2 += y * y;
                right_valid_points++;
                
                if(right_valid_points >= MAX_COLLECT_POINTS) break;
            }
        }
    }
    
    // �����ұ߽�б��
    if(right_valid_points >= MIN_VALID_POINTS)
    {
        float denominator = right_valid_points * right_sum_x2 - right_sum_x * right_sum_x;
        if(fabs(denominator) > 0.001f)
        {
            *right_slope = (right_valid_points * right_sum_xy - right_sum_x * right_sum_y) / denominator;
            right_valid = 1;
        }
    }
    
    // ������Ч�ԣ�1��ʾ���߶���Ч��0��ʾ������һ����Ч
    return (left_valid && right_valid) ? 1 : 0;
}

// �߽�����ز���
#define BORDER_CHECK_START_ROW    40      // �����ʼ��
#define BORDER_CHECK_END_ROW      80      // ��������
#define BORDER_MIN_VALID          3       // �߽���С��Чֵ (�ſ�)
#define BORDER_MAX_VALID          170     // �߽������Чֵ (�ſ�)
#define MAX_INVALID_POINTS        15      // ����������Ч���� (�ſ�)

/**
 * @brief ���ָ����Χ�����ұ߽��Ƿ�����Ч��Χ��
 * @param l_border ��߽�����
 * @param r_border �ұ߽�����
 * @return ����� (1:�󲿷���Ч, 0:��Ч�����)
 * @note ����40��80�У�������ұ߽��Ƿ���3-185��Χ��
 *       ����������Ч�㣬�����ݴ���
 */
int check_border_validity(uint8 *l_border, uint8 *r_border)
{
    int invalid_count = 0;
    int total_points = 0;
    
    // ����ָ���з�Χ
    for(int y = BORDER_CHECK_START_ROW; y <= BORDER_CHECK_END_ROW; y++)
    {
        total_points += 2; // ÿ�м������������
        
        // �����߽��Ƿ�����Ч��Χ��
        if(left[y] <= BORDER_MIN_VALID )
        {
            invalid_count++;
        }
        
        // ����ұ߽��Ƿ�����Ч��Χ��
        if( right[y] >= BORDER_MAX_VALID)
        {
            invalid_count++;
        }
    }
    
    // ������Ϣ����ʾͳ�ƽ��
    // printf("Border check: invalid=%d, total=%d, threshold=%d\n", 
    //        invalid_count, total_points, MAX_INVALID_POINTS);
    
    // �����Ч���������ݴ�Χ�ڣ���Ϊ�߽���Ч
    return (invalid_count <= MAX_INVALID_POINTS) ? 1 : 0;
}

/**
* @brief ����״̬��⺯��
* @param uint8(*image)[image_w]     ��ֵ��ͼ��
* @param uint8* hightest            ��ߵ�λ��
* @param uint8 *l_border            ��߽�����
* @param uint8 *r_border            �ұ߽�����
* @param uint8 monotonicity_l       ��߽絥����
* @param uint8 monotonicity_r       �ұ߽絥����
* @return �޷���ֵ��ͨ��jump_positionȫ�ֱ����������״̬
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
uint8 jump_position_flag=0;
int blake_line = 0; // ��¼����ȫ���е�����
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r)
{
    if( Crossroad_memory!=0||
        Crossroad_Flag!=0||
        (annulus_R_memory!=0&&annulus_R_memory!=7)||
        (annulus_L_memory!=0&&annulus_L_memory!=7)||
        BridgeState != SINGLE_BRIDGE_NOT_ACTIVE||
        jump_active
        )return;
    blake_line = 0; // ��¼����ȫ���е�����
    // ����ߵ�λ�á�55ʱ�����������
    if(*hightest>=30)
    {
        // ����ߵ�λ������ɨ�赽��50��
        for (int y = *hightest; y >= 50; y--)
        {
            // �ڵ�ǰ�е����ұ߽�֮��ɨ��
            for (int x = l_border[*hightest+1]; x <= r_border[*hightest+1]; x++)
            {
                // ���������ɫ���أ�����������ǰ��ɨ��
                if(image[y][x] == 255)
                {
                    break;
                }
                // ���ɨ�赽�ұ߽���δ������ɫ����
                if(x == r_border[*hightest+1])
                {
                    blake_line++;// ȫ���м���+1
                }
            }
        }
    }
    // ������ȫ����������3ʱ����һ���ж�
    if(blake_line >= 20)//���һ������������ʹ�һ�����40�ɣ�Զһ���������Сһ����С������ɣ�����Ч������delay���������delay,Ҳ�����������������
    {
        // mode_switch(jump);
        // if(mode!=stopworking)ips200_full(RGB565_MAGENTA);
        // ����ض�����߽��Ƿ��������޶�ʧ����˵ʵ������û��Ҫ�����������󴥷���Ծ�Ļ���ͼ��border
        //if(l_loss_judge(l_border,90 ,110) == 0 &&  // ��߽�90-110������
         //  l_loss_judge(l_border,70 ,90) == 0 &&   // ��߽�70-90������
        // r_loss_judge(r_border,90 ,110) == 0   // �ұ߽�90-110������
         //  r_loss_judge(r_border,70 ,90) == 0 // �ұ߽�70-90������
            //&&monotonicity_l == 1 && monotonicity_r == 1
         //   )
    
             // ȷ�������������������š�������ʮ���ߣ�
            {
                //  �����mode�ĳ�jumpģʽ��Ҳ��flexibleģʽ����Ȼ����ȷ�Ϊ�Գƣ����ö�������ٶȻ�
            //mode=jump;
            //ServoPID.highleft=3.3;//�����ȸ�һ��
            //ServoPID.highright=3.3;
            //  delay,������ø����ٶ�д����ʱ���������ƣ��ٶȿ����������ٶ��������
            //jump_delay(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data,int kp)�������������jump_judge��������
            if(mode!=stopworking)ips200_full(RGB565_GREEN);
            jump_act();
            }
        }
}
//
/**
* @brief ����״̬�������亯��
* @param uint8(*image)[image_w]     ��ֵ��ͼ��
* @param uint8 *l_border            ��߽�����
* @param uint8 *r_border            �ұ߽�����
* @param uint8 *center_line         ���������飨�����
* @param uint8* hightest   nili         ��ߵ�ָ��
* @return �޷���ֵ��ͨ��BridgeStateȫ�ֱ����������״̬
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
int loss_1 = 0;       // �߽��ȱ仯������
int loss_2 = 0;       // �߽�λ��ͻ�������
int bridge_number = 0; // ��������������
int bridge_out_flag=0;
int bridge_in_flag=0;
int bridge_in_flag_num=0;
int bridge_out_flag_num=0;
int bridge_entry_cooldown=0;
// ���ȵȸ�ʱ������� (��λ��10ms��2��=200��)
static int legs_equal_height_counter = 0;
int white_line1 = 0;  // ��46�а�ɫ���ؼ���
int white_line2 = 0;  // ��44�а�ɫ���ؼ���
int white_line3 = 0;  // ��60�а�ɫ���ؼ���
int white_line4 = 0;  // ��58�а�ɫ���ؼ���
int left_pattern_found = 0;  // ����-��-��ģʽ
int right_pattern_found = 0; // �Ҳ��-��-��-��-��ģʽ
int bridge_mode_choice=0;
// ��¼��һ����߽���ұ߽�ͻ�������
int first_boundary_change_x = -1;
int first_boundary_change_y = -1;
int first_boundary_change_recorded = 0;  
// ��¼���һ����߽���ұ߽�ͻ�������
int last_boundary_change_x = -1;
int last_boundary_change_y = -1;
// ��¼ͻ�����
int boundary_change_count = 0;
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line, uint8* hightest)
{
    if(jump_position_flag != 0||
        Crossroad_memory!=0||
        Crossroad_Flag!=0||
        (annulus_R_memory!=0&&annulus_R_memory!=7)||
        (annulus_L_memory!=0&&annulus_L_memory!=7)||
        Endline>40||
        bridge_in_flag==-1||
        jump_active
    )return;
    get_left_cicle(data_stastics_l);
    get_right_cicle(data_stastics_r);
    white_line1 = 0;  // ��75�а�ɫ���ؼ���
    white_line2 = 0;  // ��70�а�ɫ���ؼ���
    int long_start_l = 0; // ��߽�ͻ����ʼ��
    int long_end_l = 0;   // ��߽�ͻ�������
    int long_start_r = 0; // �ұ߽�ͻ����ʼ��
    int long_end_r = 0;   // �ұ߽�ͻ�������
     loss_1 = 0;       // �߽��ȱ仯������
     loss_2 = 0;       // �߽�λ��ͻ�������
     bridge_number = 0; // ��������������
     bridge_in_flag=0;
    // ɨ��ͼ���м����򣬷����߽�仯
    //������״̬Ϊ"δ����"ʱ������Ƿ��������
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        // ���ȼ��߽��Ƿ�ʧ�������ʧ�򲻽��к������
        if (Lost_left_Flag == 0 && Lost_right_Flag == 0) {
            // ���3�����ֵ�ͻ��
            int part1_changes = 0;  // ��һ����ͻ�����
            int part2_changes = 0;  // �ڶ�����ͻ�����
            int part3_changes = 0;  // ��������ͻ�����
            int total_changes = 0;  // ��ͻ�����
            // ��һ���֣���Endline+10��40�����4��ͻ��
            for (int y = Endline + 10; y < 40 && y < image_h - 1; y++) {
                if (part1_changes < 2) {  // ÿ���������2��ͻ��
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 4 || abs(right_cicle[y] - right_cicle[y+1]) > 4) {
                        part1_changes++;
                        total_changes++;
                    }
                }
            }
            // �ڶ����֣���40��80�����7��ͻ��
            for (int y = 40; y < 80 && y < image_h - 1; y++) {
                if (part2_changes < 2) {  // ÿ���������2��ͻ��
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 7 || abs(right_cicle[y] - right_cicle[y+1]) > 7) {
                        part2_changes++;
                        total_changes++;
                    }
                }
            }
            // �������֣���80��118�����10��ͻ��
            for (int y = 80; y < 118 && y < image_h - 1; y++) {
                if (part3_changes < 2) {  // ÿ���������2��ͻ��
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 10 || abs(right_cicle[y] - right_cicle[y+1]) > 10) {
                        part3_changes++;
                        total_changes++;
                    }
                }
            }
            // �����������¼4��ͻ��
            if (total_changes >= 4) {
                if(mode!=stopworking){
                    ips200_full(RGB565_39C5BB);
                    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);
                    show_string_value(11,part1_changes,3,"part1_changes");
                    show_string_value(12,part2_changes,3,"part2_changes");
                    show_string_value_float(13,part3_changes,3,3,"part3_changes");
                }
                bridge_in_flag=1;
            }
        }
    }
    // ������״̬Ϊ"����"ʱ������������������������
    if(BridgeState == SINGLE_BRIDGE_ACTIVE){
        first_boundary_change_x = -1;
        first_boundary_change_y = -1;
        // ��ʼ��ͻ�����
        boundary_change_count = 0;
        // ��¼��һ��ͻ���
        for (int y = image_h-60; y > Endline+30; y--) {
            // �����߽�ͻ��
            if (abs(left_cicle[y] - left_cicle[y-3]) > 25) {
                first_boundary_change_x = (left_cicle[y] > left_cicle[y-3]) ? left_cicle[y] : left_cicle[y-3];  // ȡ�߽�������
                first_boundary_change_y = y;
                first_boundary_change_recorded = 1;  
                boundary_change_count++;  // ͻ�������1
                //break;
            }
            // ����ұ߽�ͻ��
            if (abs(right_cicle[y] - right_cicle[y-3]) > 20) {
                first_boundary_change_x = (right_cicle[y] < right_cicle[y-3]) ? right_cicle[y] : right_cicle[y-3];  // ȡ�߽�С������
                first_boundary_change_y = y;
                first_boundary_change_recorded = 1;
                boundary_change_count++;  // ͻ�������1
                //break;
            }
        }
        // ��¼���һ��ͻ���
        for (int y = Endline + 60; y < image_h; y++) {
            // �����߽�ͻ��
            if (abs(left_cicle[y] - left_cicle[y-8]) > 55) {
                last_boundary_change_x = (left_cicle[y] > left_cicle[y-8]) ? left_cicle[y] : left_cicle[y-8];  // ȡ�߽�������
                last_boundary_change_y = y;
                boundary_change_count++;  // ͻ�������1
            }
            // ����ұ߽�ͻ��
            if (abs(right_cicle[y] - right_cicle[y-8]) > 55) {
                last_boundary_change_x = (right_cicle[y] < right_cicle[y-8]) ? right_cicle[y] : right_cicle[y-8];  // ȡ�߽�С������
                last_boundary_change_y = y;
                boundary_change_count++;  // ͻ�������1
            }
            if(left_cicle[118]>50&&left[118]-left[58]>30){
                last_boundary_change_x=left_cicle[118];
                last_boundary_change_y=118;
                boundary_change_count++;  // ͻ�������1
            }
            else if(right_cicle[118]<110&&right[58]-right[118]>30){
                last_boundary_change_x=right_cicle[118];
                last_boundary_change_y=118;
                boundary_change_count++;  // ͻ�������1
            }
            else {
                last_boundary_change_x=(right_cicle[118]+left_cicle[118])/2;
                last_boundary_change_y=118;
                boundary_change_count++;  // ͻ�������1
            }
        }        
        
        // �����ҵ��ı߽�仯��������в�ͬ�Ĳ��߲���
        if ((first_boundary_change_x == -1 || first_boundary_change_y == -1) && 
            (last_boundary_change_x == -1 || last_boundary_change_y == -1)) {
            // ������¶�û�ҵ���ʹ�������е㲹��
            Addingline( 3, (right[50]+left[50])/2, 50, (right[118]+left[118])/2, 118 );
        } 
        else if (first_boundary_change_x == -1 || first_boundary_change_y == -1) {
            // �������û�ҵ��������ҵ��ˣ�ʹ���е㵽�±߽�㲹��
            Addingline( 3, (right[Endline+20]+left[Endline+20])/2, Endline+20, last_boundary_change_x, last_boundary_change_y );
        } 
        else {
            // ������¶��ҵ��ˣ�ʹ��ԭ����Addingline
            Addingline( 3, first_boundary_change_x, first_boundary_change_y, last_boundary_change_x, last_boundary_change_y );
        }
        
        // ������ȸ߶Ȳ��Ƿ��ڵȸ߷�Χ�ڣ���ֵΪ0.2��
        float height_diff = ServoPID.highleft - ServoPID.highright;
        if (abs(height_diff) <= 0.4f) {
            legs_equal_height_counter++;
        } else {
            legs_equal_height_counter = 0; // ���ü�����
        }
        
        // ��û�б߽��ȱ仯��û������������û�б߽�λ��ͻ��ʱ���������ȵȸ߳���2��ʱ����Ϊ��������
        if( legs_equal_height_counter >= 100)
        {
            //bridge_out_flag=1;
            legs_equal_height_counter = 0; // ���ü�����
        }
    }
}
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
* @return �޷���ֵ������洢��ȫ�ֱ�����
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */

void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
   uint16 i;
   int black_line_sum = 0; // ��ɫ��������
   int black_sum_1 = 0;// �Ҷ�ֵ״̬���
   int black_sum_2 = 0;
   uint8 break_num_l = 0;// ���ұ�Եͻ����к�
   uint8 break_num_r = 0;
   uint8 end_num_l = 0;// ���ұ�Ե�ָ����к�
   uint8 end_num_r = 0;
   uint8 start, end;  // ���������ֹ�к�
   int ap = 1;  // �����Ч�Ա��
   float slope_l_rate = 0, intercept_l = 0; // ֱ����ϲ���
  // �������������Ե�ǵ������ұ�Ե������
   if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
   {
         // ������Ե����λ��
        broken_line_judge(1,*hightest,110,l_border);
        // ����ұ�Ե�Ƿ����ڲ�����������������
        for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
        {
            if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
            {
                ap = 0;// ���Ϊ��Ч���
                break;
            }
        }
         // ͳ��ͼ���еĺ�ɫ����������Ϊ�����ߣ�
        for (i = broken_line_y+5; i > *hightest; i--)
        {
            for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
            {
                // ����ɫ������ʼ
                if (image[i][x] == 0 && image[i][x-1] == 255)
                {
                    black_sum_1 = 1;
                }
                // ����ɫ��������
                if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
                {
                    black_sum_2 = 1;
                }
                 // ����״̬���
                if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
                {
                    black_sum_1 = 0;
                    black_sum_2 = 0;
                    break;
                }
            }
            // �ۼ���Ч��ɫ������
            if(black_sum_2 == 1)
            {
                black_sum_1 = 0;
                black_sum_2 = 0;
                black_line_sum++;
            }
            if(i==15)// �����ǰ15��
                break;
        }
        // �ۺ��ж��Ƿ�Ϊ�����
        if(black_line_sum>=5 /*ȷ��������ʶ�����*/
                && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
                /*������������Ե��ʧ�ж�*/
                && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
            /*����������ʮ�ַ����ŽӲ���ͻ*/
        {
                sum_island = 1;// ���Ϊ�������
                island = 1;// ���Ϊ�����
                black_line_sum = 0;
        }
    }
    // �����������ұ�Ե�ǵ��������Ե������
    if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
    {
        // ����ұ�Ե����λ��
        broken_line_judge(1,*hightest,110,r_border);
        // ������Ե�Ƿ����ڲ�����������������
        for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
        {
            if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || l_border[w] != border_min))
            {
                ap = 0;// ���Ϊ��Ч���
                break;
            }
        }
        // ͳ��ͼ���еĺ�ɫ�������߼���������ͬ��
        for (i = broken_line_y+5; i > *hightest; i--)
        {
            for (int x = (int)r_border[i]; x >= (int)l_border[i]; x--)
            {
                if (image[i][x] == 0 && image[i][x+1] == 255)
                {
                    black_sum_1 = 1;
                }
                if (black_sum_1 == 1 && image[i][x+1] == 0 && image[i][x] == 255)
                {
                    black_sum_2 = 1;
                }
                if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x+1] == 255)
                {
                    black_sum_1 = 0;
                    black_sum_2 = 0;
                    break;
                }
            }
            if(black_sum_2 == 1)
            {
                black_sum_1 = 0;
                black_sum_2 = 0;
                black_line_sum++;
            }
            if(i==15)//???��????????
                break;
        }
        // �ۺ��ж��Ƿ�Ϊ��������߼���������ͬ��
        if(black_line_sum>=5 /*ȷ��������ʶ�����*/
                && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*������������Ե��ʧ�ж�*/
                && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
            /*����������ʮ�ַ����ŽӲ���ͻ*/
        {
            sum_island = 1;  // ���Ϊ�������
            island = 2;      // ���Ϊ�����
            black_line_sum = 0;
        }
    }
    // �����������island=1��
    if(island == 1)
    {
         // ȷ���������sum_island=1��
        
        if(sum_island == 1)
        {
            // ���¼�����Ե����λ��
            broken_line_judge(1,*hightest,110,l_border);
            // ����ұ�Ե�Ƿ�������ڲ�����
            for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
            {
                if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
                {
                    ap = 0;// �����Ч���˳�״̬
                    break;
                }
            }
             // �������Ч������״̬
            if(ap == 0)
            {
                island = 0;
                sum_island = 0;
            }
            // �������Ч�Ҷ���λ�ú��ʣ��޸����Ե
            if(broken_line_y >= 20 && ap == 1)
            {
                // ��С���˷�������Ե
                start = broken_line_y+5;
                end = broken_line_y+10;
                calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
                // �����ֱ���޸����Ե
                for (i = 1; i < broken_line_y+1; i++)
                {
                    l_border[i] = slope_l_rate * (i)+intercept_l;
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }
                // ����Ƿ���Ҫ��һ������
                if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
                {
                    // ͨ����Ե��ȷ�����״̬
                    for (i = 106; i > 15; i--)
                    {
                        
                        if(points_l[l_index[i]][0]>points_l[l_index[i-5]][0] && points_l[l_index[i]][0]>points_l[l_index[i+5]][0]
                            && points_l[l_index[i-5]][0] != border_min)
                        {
                            sum_island = 2;// ����״̬Ϊȷ�����
                        }
                    }
                }
            }
        }
        // ȷ���������sum_island=2������һ������
        if(sum_island == 2)
        {
            int dp = 0;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            // ���ͼ��ײ����أ�����Ϊ����������
            if (!image[image_h - 5][5] && !image[image_h - 3][3])
            {
                dp = 1;
            }
            // ����Եͻ��λ��
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            // ȷ�ϱ�Եͻ���
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]
                         &&points_l[j][1] > points_l[j + 5][1]
                         &&points_l[j][1] >= points_l[j - 3][1]
                         &&points_l[j][1] >= points_l[j - 5][1])
                    {
                        vp = h;
                        break;
                    }
                }
            }
             // Ѱ�ұ�Ե��͵�
            for (i = 25; i < image_h - 15; i++)
            {
                if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
                   && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
                   && l_border[i-5] != border_min && l_border[i+5] != border_min)
                {
                    end_num_l = (uint8)i;
                }
                // ״̬����Ϊ׼�����
                if(vp && end_num_l >= 80)
                    sum_island = 3;
            }
                // ����ұ�Ե����������������Ե�ָ���
                slope_l_rate = (float)(118-end_num_l) / ((border_max-r_border[118]+border_min)-l_border[end_num_l]);//��??k=y/x
                intercept_l = 118 - slope_l_rate*(border_max-r_border[118]+border_min);//???b=y-kx
                for (i = end_num_l; i < image_h - 1; i++)
                {
                    l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }

        }
        //����ȷ�������
        if(sum_island == 3)
        {
            uint16 h = 0;
            int temph_l = 0;
            // 1. Ѱ�����Եͻ��㣨�����ʼλ�ã�
            for (h = image_h - 15; h > 5; h--)// ��ͼ��ײ�����ɨ��
            {
                if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    temph_l = h;  // ��¼ͻ���к�
                    break;  // �ҵ��������˳�ѭ��
                }
            }
             // 2. ��ͻ��㸽��Ѱ�����ת�۵�
            if (temph_l)// ����ҵ���ͻ���
            {
                for (int i = total_num_l - 10; i > l_index[h + 1]; i--)
                {
                    // Ѱ��y����ֲ����ֵ���������������
                    if (points_l[i][1] >= points_l[i + 3][1]  // ��ǰ��y����ݺ�3����
                        && points_l[i][1] > points_l[i + 5][1]  // ��ǰ��y����>��5����
                        && points_l[i][1] >= points_l[i - 3][1]  // ��ǰ��y�����ǰ3����
                        && points_l[i][1] >= points_l[i - 5][1]  // ��ǰ��y�����ǰ5����
                        // λ�ú�����Լ��
                        && points_l[i][0] > points_l[i - 5][0]  // ��ǰ��x����>ǰ5����
                        && points_l[i][0] <= points_l[i + 5][0])  // ��ǰ��x����ܺ�5����
                    {
                        break_num_l = (uint8)points_l[i][1];  // ��¼ת�۵�y����
                        end_num_l  = (uint8)points_l[i][0];  // ��¼ת�۵�x����
                        break;  // �ҵ��������˳�ѭ��
                    }
                }
            }
            //ʹ�ø���Χ�ıȽ�
//            for (i = 40; i < total_num_l-30; i++)
//            {
//                if (points_l[i][1]>points_l[i+9][1]&&points_l[i][1]>points_l[i-9][1]
//                      &&points_l[i][1]>points_l[i+15][1]&&points_l[i][1]>points_l[i-15][1])
//                  {
//                     break_num_l = (uint8)points_l[i][1];//????y????
//                     end_num_l  = (uint8)points_l[i][0];//????x????
//                  }
//            }

            // 4. ���ͼ��ײ���Ե������ȷ���Ƿ���������
            if((*hightest >= 30)&&  // ��ߵ�λ�ú���ȷ������������
              (!image[image_h - 1][3] && !image[image_h - 3][3] // ͼ�����½�Ϊ��ɫ�������߽磩
                && !image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))// ͼ�����½�Ϊ��ɫ
            {
                sum_island = 4;//�������������״̬4
            }

            // 5. �����ұ߽磨����ת�۵����ֱ�߷��̣�
            if(break_num_l && end_num_l)  // ����ҵ���ת�۵�
            {
                // ����б��k = ��y/��x����(186,118)Ϊ�ο��㣩
                slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);
                intercept_l = 118 - slope_l_rate*186;// ����ؾ�b = y - kx
                 // ����ֱ�߷��̸����ұ߽�x = (y - b)/k
                for (i = 1; i < image_h - 1; i++)
                {
                    r_border[i] = ((i)-intercept_l)/slope_l_rate;// ����ÿ��y��Ӧ��x����
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max); // ��������Ч��Χ��
                }
            }
        }
        // ��ǰ����״̬4����֤���������
        if(sum_island == 4)
        {
             int g=0;  // ��������ƥ���־
    
             // 1. Ѱ���ض��������У�4��4��6��6��6����ʾ����ת��
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    g=1;
                    break;
                }
            }
            // 2. ����ұ߽�ͻ�䣨���ʱ仯�㣩
            for (i = image_h - 20; i > *hightest; i--)//?????��?
            {
                if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>30)
                {
                    if(i>=30 && i<=105 && g)
                        sum_island = 5;
                }
            }
        }
        // ��ǰ����״̬5�������������
        if(sum_island == 5)
        {
            // 1. Ѱ���ұ߽�ͻ��㣨������󴦣�
            for (uint16 w = image_h - 15; w > *hightest; w--)//?????��?
            {
                if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
                {
                    break_num_l = (uint8)w;//????y????
                    break;
                }
            }
            // 2. �ٴ�ȷ�Ϸ�������������ͬ״̬4��
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    end_num_l = (uint8)points_l[i][1];//????y????
                    break;
                }
            }
            // 3. ����ұ߽��Ƿ���ڣ�δ��ʧ��
            if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
                sum_island = 6;
            // 4. �����ұ߽磨�����µ����ʼ���ֱ�߷��̣�
            slope_l_rate = (float)(break_num_l-end_num_l) / (r_border[break_num_l]-l_border[end_num_l]);//��??k=y/x
            intercept_l = 0 - slope_l_rate*0;//�����½ؾ�b=y-kx
             // ������ֱ�߷��̸����ұ߽磨�����µ�ͻ����·���
            for (i = 1; i < break_num_l-3; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        // �������״̬
        if(sum_island == 6)
        {
            int dp = 1;// �����Ƿ�ִ�������ѭ�����̶�Ϊ1����ʾִ�У�
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
             // 1. Ѱ�����Եͻ��㣨�������������
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            // 2. ��ͻ��㸽��Ѱ�������㣨y����ֲ����ֵ��
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]  // ��ǰ��y����ݺ�3����
                        && points_l[j][1] > points_l[j + 5][1]  // ��ǰ��y����>��5����
                        && points_l[j][1] >= points_l[j - 3][1]  // ��ǰ��y�����ǰ3����
                        && points_l[j][1] >= points_l[j - 5][1])  // ��ǰ��y�����ǰ5����
                    {
                        vp = h;
                        break;
                    }
                }
            }
             // 3. ״̬ת��������������������ұ߽��ض�����ʧ��
            if(vp && r_loss_judge(r_border, 70 ,90) == 0// �ұ߽���70-90�ж�ʧ
                    && r_loss_judge(r_border, 50 ,70) == 0)// �ұ߽���50-70�ж�ʧ
                sum_island = 7;
            // 4. �����ұ߽磨�ָ�Ĭ��ֱ��
            slope_l_rate = (float)(118-0) / (186-0);// ����Ĭ��б��k=y/x
            intercept_l = 0 - slope_l_rate*0;//����Ĭ�Ͻؾ�b=y-kx
            // ����Ĭ��ֱ�߷��̸����ұ߽�
            for (i = 1; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        // ��������ж�״̬
        if(sum_island == 7)
        {
            uint16 h = 0;
            int temph = 0;
            // 1. Ѱ�����Եͻ��㣨�������������
            for (h = image_h - 25; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
                {
                    temph = h;  // ��¼ͻ���к�
                    break_num_l = (uint8)h;  // ��¼ת�۵�y����
                    end_num_l  = (uint8)l_border[h];  // ��¼ת�۵�x����
                    break;  // �ҵ��������˳�ѭ��
                }
            }
//            if (temph)
//            {
//                for (int i = total_num_l - 10; i > l_index[h + 1]; i--)
//                {
//
//                    if (points_l[i][1] >= points_l[i + 3][1]
//                        &&points_l[i][1] > points_l[i + 5][1]
//                        &&points_l[i][1] >= points_l[i - 3][1]
//                        &&points_l[i][1] >= points_l[i - 5][1]
//                        &&points_l[i][0] > points_l[i - 5][0]
//                        &&points_l[i][0] <= points_l[i + 5][0])
//                    {
//                        break_num_l = (uint8)points_l[i][1];//????y????
//                        end_num_l  = (uint8)points_l[i][0];//????x????
//                        break;
//                    }
//                }
//            }
            // 3. ���ͼ��ײ���Ե������ȷ���Ƿ�ص�ֱ����
            if (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]  // ���½�Ϊ��ɫ
                && !image[image_h - 7][image_w - 3]  // ���½��Ϸ�Ϊ��ɫ
                && !image[image_h - 5][3] && !image[image_h - 3][3]  // ���½�Ϊ��ɫ
                && !image[image_h - 7][3])  // ���½��Ϸ�Ϊ��ɫ
            {
                if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0 // �ұ߽��ض�����ʧ
                   && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0) // ��߽��ض�����ʧ
                {
                    sum_island = 0;
                    island = 0;// �����������
                }
            }
            // 4. ������߽磨�����µ�б�ʣ�����˫�����߽磩
            slope_l_rate = (float)(118-end_num_l) / (2-l_border[end_num_l]);//������б��k=y/x
            intercept_l = 118 - slope_l_rate*2;//�����½ؾ�b=y-kx
            // ������ֱ�߷��̸�����߽磨������ת�۵��·�����
            for (i = end_num_l-10; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
    }
     // ��ǰʶ��Ϊ�������island=2��
    if(island == 2)
    {
        // ����״̬1�����������
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,r_border); // ����ұ߽����
            // ��֤��߽������ԣ���ֹ����Ϊ�����
            for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
            {
                if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || r_border[w] != border_min))
                {
                         ap = 0;// ���Ϊ�������߽�
                         break;
                }
            }
            // �߽粻������ȡ��������
            if(ap == 0)
             {
                 island = 0;
                 sum_island = 0;
             }
             // ����λ�ú����ұ߽�����ʱ�����ұ߽�
            if(broken_line_y >= 20 && ap == 1)
            {
                start = broken_line_y+5;  // �����ʼ��
                end = broken_line_y+10;    // ��Ͻ�����
                calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
                // ����ֱ�߷��̸����ұ߽�
                for (i = 1; i < broken_line_y+1; i++)
                {
                    r_border[i] = slope_l_rate * (i)+intercept_l;
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                }
                 // ����ұ߽�ͻ���ʧ��������һ״̬
                if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
                {
                    for (i = 106; i > 15; i--)
                    {
                        // Ѱ���ұ߽�ֲ���Сֵ�����������
                        if(points_r[r_index[i]][0]<points_r[r_index[i-5]][0] && points_r[r_index[i]][0]<points_r[r_index[i+5]][0]
                            && points_r[r_index[i-5]][0] != border_max)
                        {
                            sum_island = 2;
                        }
                    }
                }
            }
        }
     // ״̬2��ȷ�����������
    if(sum_island == 2)
    {
        int dp = 0;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
         // ���ͼ��ײ������㣨�Ҳ��Ե��
        if (!image[image_h - 5][183] && !image[image_h - 3][185])
        {
            dp = 1;// ���Ϊ��Ч�������
        }
         // Ѱ���ұ�Եͻ��㣨�����ʼ��
        if (dp)
        {
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
                {
                    temph = h;
                    break;
                }
            }
        }
        // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
         // Ѱ���ұ߽�ֲ���Сֵ���������㣩
        for (i = 25; i < image_h - 15; i++)
        {
            if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
               && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
               && r_border[i-5] != border_max && r_border[i+5] != border_max)
            {
                end_num_r = (uint8)i;
            }
            // �����������λ�ú���ʱ����״̬3
            if(vp && end_num_r >= 80)
                sum_island = 3;
        }
             // �����ұ߽磨�������ʼ��㣩
            slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//��??k=y/x
            intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
            for (i = end_num_r-10; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
    }
    // ״̬3���������������
    if(sum_island == 3)
    {
        uint16 h = 0;
        int temph_l = 0;
        // Ѱ���ұ�Եͻ��㣨ϸ��λ�ã�
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
            {
                temph_l = h-5;
                break;
            }
        }
         // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
        if (temph_l)
        {
            for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
            {

                if (points_r[i][1] >= points_r[i + 3][1]// y����ֲ����
                    &&points_r[i][1] > points_r[i + 5][1]
                    &&points_r[i][1] >= points_r[i - 3][1]
                    &&points_r[i][1] >= points_r[i - 5][1]
                    &&points_r[i][0] < points_r[i - 5][0]
                    &&points_r[i][0] >= points_r[i + 5][0])
                {
                    break_num_r = (uint8)points_r[i][1];// ��¼������y����
                    end_num_r  = (uint8)points_r[i][0];// ��¼������x����
                    break;
                }
            }
        }
//        for (i = 40; i < total_num_r-30; i++)
//        {
//            if (points_r[i][1]>points_r[i+9][1]&&points_r[i][1]>points_r[i-9][1]
//                  &&points_r[i][1]>points_r[i+15][1]&&points_r[i][1]>points_r[i-15][1])
//              {
//                 break_num_r = (uint8)points_r[i][1];//????y????
//                 end_num_r  = (uint8)points_r[i][0];//????x????
//              }
//        }
        // ��ߵ�λ�ú���ʱ����״̬4
        if(*hightest >= 30)
        {
            sum_island = 4;
        }
        // ������߽磨��������������㣩
        if(break_num_r && end_num_r)
        {
           
                slope_l_rate = (float)(break_num_r-118) / (end_num_r-2);//k=y/x
                intercept_l = 118 - slope_l_rate*2;//b=y-kx
                for (i = 1; i < image_h - 1; i++)
                {
                    l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }
        }
    }
    // ״̬4�������߽�ͻ��
    if(sum_island == 4)
    {
        for (i = image_h - 15; i > *hightest; i--)//?????��?
        {
            if (l_border[i] > l_border[i - 3] && (l_border[i] - l_border[i-3])>20)
            {
                if(i>=30 && i<=105)
                  sum_island = 5;
            }
        }
    }
     // ״̬5��������߽����
    if(sum_island == 5)
    {
        for (uint16 w = image_h - 15; w > *hightest; w--)//?????��?
        {
            if (l_border[w] > l_border[w - 3] && (l_border[w] - l_border[w-3])>20)
            {
                break_num_r = (uint8)w;//????y????
                break;
            }
        }
//        for (i = 1; i < total_num_l; i++)
//        {
//            if (dir_l[i - 10] >= 4 && dir_l[i - 10] <= 6 && dir_l[i-5] >= 4 && dir_l[i-5] <= 6
//                    && dir_l[i] >= 2 && dir_l[i] <= 4 && dir_l[i + 5] <= 4 && dir_l[i + 5] >= 2
//                    && dir_l[i + 10] <= 4 && dir_l[i + 10] >= 2)
//            {
//                break_num_r = (uint8)points_r[i][1];//????y????
//                break;
//            }
//        }
        // �ұ߽����ʱ����״̬6
        if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
            sum_island = 6;
        // ������߽磨����ͻ���
        end_num_r = l_border[break_num_r];
        slope_l_rate = (float)(break_num_r-0) / (end_num_r-188);//��??k=y/x
        intercept_l = 0 - slope_l_rate*188;//b=y-kx
        for (i = 1; i < break_num_r-3; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
    // ״̬6�����������
    if(sum_island == 6)
    {
        int dp = 1;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (dp)
        {
            // Ѱ���ұ�Եͻ��㣨�������������
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
                {
                    temph = h;
                    break;
                }
            }
        }
         // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
        // �߽綪ʧʱ����״̬7����������жϣ�
        if(vp && l_loss_judge(r_border, 90 ,110) == 0 && l_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(r_border, 50 ,70) == 0)
            sum_island = 7;
         // ������߽磨�ָ�Ĭ��ֱ�ߣ�
        slope_l_rate = (float)(118-0) / (0-188);//k=y/x
        intercept_l = 0 - slope_l_rate*188;//b=y-kx
        for (i = 1; i < image_h - 1; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
     // ״̬7������������ж�
    if(sum_island == 7)
    {
        uint16 h = 0;
        int temph = 0;
        // Ѱ���ұ�Եͻ��㣨�������
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
            {
                temph = h;
                break;
            }
        }
        // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
        if (temph)
        {
            for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
            {

                if (points_r[i][1] >= points_r[i + 3][1]
                    &&points_r[i][1] > points_r[i + 5][1]
                    &&points_r[i][1] >= points_r[i - 3][1]
                    &&points_r[i][1] >= points_r[i - 5][1]
                    &&points_r[i][0] < points_r[i - 5][0]
                    &&points_r[i][0] >= points_r[i + 5][0])
                {
                    break_num_r = (uint8)points_r[i][1];//????y????
                    end_num_r  = (uint8)points_r[i][0];//????x????
                    break;
                }
            }
        }
        // �ײ���Ե��������ʱ�ص���ʼ״̬
        if (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]
             && !image[image_h - 7][image_w - 3]
             && !image[image_h - 5][3] && !image[image_h - 3][3]
             && !image[image_h - 7][3])
        {
            if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
               && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0
               && r_loss_judge(r_border, 50 ,70) == 0 && l_loss_judge(l_border, 50 ,70) == 0)
            {
                sum_island = 0;
                island = 0;
            }
        }
         // �����ұ߽磨�����������
        slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//��??k=y/x
        intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
        for (i = end_num_l-10; i < image_h - 1; i++)
        {
            r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
   }
}

//===================================================��  ʾ===================================================
void IPS_show(void){
   int i;
  //********************ͼ����ʾ*****************************
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);

  //********************������ʾ*****************************
        //��������ѭ�����������߽��
       /* for (i = 0; i < data_stastics_l; i++)
        {
            ips200_draw_point(points_l[i][0]+2, points_l[i][1],  RGB565_GREEN);
        }
        for (i = 0; i < data_stastics_r; i++)
        {
            ips200_draw_point(points_r[i][0]-2, points_r[i][1],  RGB565_GREEN);
        }*/

        for (i = Endline; i < image_h-1; i++)
        {
          //  middle[i] = (left[i] + right[i]) >> 1;//������

            //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
            ips200_draw_point(middle[i], i,  RGB565_GREEN);
            ips200_draw_point(left[i], i, RGB565_RED);
            ips200_draw_point(right[i], i, RGB565_BLUE);

        }
          if(Endline<115){
              for(int i=119;i>Endline;i--)
               {
                   //ips200_draw_line(middle[scanning_line], 1, middle[scanning_line], 119, RGB565_RED); //����
                   //ips200_draw_line(left[scanning_line], 1, left[scanning_line], 119, RGB565_CYAN); //�������
                   //ips200_draw_line(right[scanning_line], 1, right[scanning_line], 119, RGB565_YELLOW);  //�ҿ�����
                   if(Endline>1&&Endline<120)
                     {
                       ips200_draw_line(1, Endline, 187, Endline,RGB565_GREEN); //��ֹ��
                     }
                }
          }
        }



//===================================================Ԫ��ʶ��===================================================
void Element_recognition(void)
{

    inflection_point();//�յ����ж�
    left_straight();//��ֱ��
    right_straight();//��ֱ��
    monotonicity_line((uint8*)&Endline,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
    crossroad();
    annulus_L();//��Բ��
    annulus_R();//��Բ��
    middle_line();//����
    //bridge_fill(imag,left, right, middle, (uint8*)&Endline);//������

    //jump_judge(imag, (uint8*)&Endline,(uint8*)left, (uint8*)right, (int)Left_straight_flag, (int)Right_straight_flag);//��Ծ
    //zebra_crossing(imag,left,right);//������
    //around_fill( imag,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, (uint8*)&Endline,l_index,r_index,monotonicity_l, monotonicity_r);

}

//===================================================ͼ����===================================================

void image_process(void)
{


        //if(mt9v03x_finish_flag){
            //system_start ();

             //Get_image(mt9v03x_image);
            // binaryzation();
             //image_filter(imag);
             //image_draw_rectan(imag);

            data_stastics_l = 0;
            data_stastics_r = 0;
            if (get_start_point(image_h - 2))
            {
                search_l_r((uint16)USE_num, imag, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
                get_left(data_stastics_l);
                get_right(data_stastics_r);
                lost_left();
                lost_right();
                Element_recognition();
            }
            //Finish_Flag=1;
            //image_process_time=system_getval ();
            //mt9v03x_finish_flag=0;

        border = (float)((middle[image_h-2] * 0.07) + (middle[image_h-12] * 0.10)
                           + (middle[image_h-22] * 0.25) + (middle[image_h-27] * 0.20)
                           + (middle[image_h-32] * 0.12) + (middle[image_h-42] * 0.09)
                           + (middle[image_h-52] * 0.07) + (middle[image_h-62] * 0.06)
                           + (middle[image_h-72] * 0.04));
}
void get_turn_value(float kp,float kp2,float kd,float gkd)
{
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
        //border=border-((middle[image_h-2] * 0.07) + (middle[image_h-12] * 0.10)
        //                   + (middle[image_h-22] * 0.25) + (middle[image_h-27] * 0.20)
        //                   + (middle[image_h-32] * 0.12) + (middle[image_h-42] * 0.09));
        //border*=5.88;
        //kp2*=2;
    }
    float border_abs=0;
    border_abs=(94-border)>0?(94-border):-(94-border);//����ͼ��ƫ��ľ���ֵ
    //������Ը����ٶ�������С��630�������
    turn_value=(94-border)*kp
            +(94-border)*border_abs*kp2
            +((94-border)-border_last)*kd
            +imu660ra_gyro_z*gkd;
    //turn_value=turn_value*(�����ڵ�Ŀ���ٶ�/����õ��ٶ�)
    //��������Ϊv=w*r�������ٶȱ��֮����Ŀ����ٶ�ҲҪ���ű��������뾶���ܲ��䣬�����ߵ�·�Ż�һ��
    //����Ӧ���ȵ���һ���ٶȣ�֮���ٰ�turn_value=turn_value*(�����ڵ�Ŀ���ٶ�/����õ��ٶ�)����
    border_last=(94-border);
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
        {
            
        }
}
