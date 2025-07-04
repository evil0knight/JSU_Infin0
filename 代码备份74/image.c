#include  "zf_common_headfile.h"

#define White 255   //�����ɫ���ص�ֵΪ255
#define Black 0     //�����ɫ���ص�ֵΪ0

#define Left 1     // ��������ʶΪ1
#define Right 2    // �����Ҳ��ʶΪ2
int x,y;
float parameterB,parameterA;   //y=parameterB*x+parameterA
float k1,k2,k3,k4,k5,k6;
int left[120]={2};             //������СΪ120�����߽����飬��ʼֵΪ2
int right[120]={185};          //������СΪ120���Ҳ�߽����飬��ʼֵΪ185
int middle[120]={93};          //������СΪ120���м������飬��ʼֵΪ93
int Endline=1;                 //�����յ��߱�־����ʼΪ1
int WhiteNum=0;                //��ɫ���ؼ���������ʼΪ0
int X1,Y1;//���²��ߵ㣨Բ����
uint8 right_lost_num=0;        //ͳ���ұ߽綪ʧ�Ĵ���
uint8 left_lost_num=0;         //ͳ����߽綪ʧ�Ĵ���
uint8 imag[120][188];          //����120��188�е�ͼ������      ���ڴ洢ͼ������
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
uint8 Finish_Flag=0; //������ɱ�ʶλ

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


//��ֵ������   ��ͼ��ת��Ϊ�ڰ׶�ֵͼ��
void binaryzation(void)
{
  uint8 i,j;
//���� OtsuThreshold �������������ֵ������ 5��
threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+5;
  // ����ͼ�����
  for(i = 0;i<image_h;i++)
  {
       //����ͼ�����
      for(j = 0;j<image_w;j++)
      {
          //�����ǰ����ֵ������ֵ�����丳ֵΪ��ɫ����
          if(original_image[i][j]>threshold_value)imag[i][j] = white_pixel;
          //�����丳ֵΪ��ɫ����
          else imag[i][j] = black_pixel;
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
        //��������������������ͬ������ѭ��
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("���ν���ͬһ���㣬�˳�\n");
            break;
        }
        //������ұ߽�ĵ�ľ���С�� 2������ѭ���������½����ߵ�λ��
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n���������˳�\n");
            *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
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
    for(i=110;i>10;i--){
        //�����ǰ�еĵ� 2 �е�����Ϊ��ɫ
        if(imag[i][2]==White){
            left_lost_num++;//��߽綪ʧ������ 1
            Lost_point_L_scan_line=i+4;//��¼��߽綪ʧ���ɨ����λ��
        }
        //�����߽綪ʧ�������� 15
        if(left_lost_num>15){
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

    }
    h = image_h - 2;
    //�����ұ߽�ĵ�
    for (j = 0; j < total_R; j++)
    {
        //�����ǰ��� y ��������к�
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;

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
    for(i=110;i>10;i--){
        if(imag[i][185]==White){
            right_lost_num++;
            Lost_point_R_scan_line=i+4;
        }
        if(right_lost_num>15){
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
            if((left[y]-left[y-4])>5&&left[y-4]==2&&(left[y]-left[y+2])<5&&left[y]>10){

                // �����������Ƿ������¹յ㲢��¼����
                Lower_left_inflection_Flag=1;
                Lower_left_inflection_X =left[y];
                Lower_left_inflection_Y =y;
                return;
            }
        }
    }
    //----------------------------��һ�棺��ɨ�߷����ж�--------------------------------
    //    for(y=110;y>(Endline+10);y--){
    //        for(x=left[y+1];x<186&&x>1;x++){
    //              if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
    //                 Lower_left_inflection_X =x;
    //                 Lower_left_inflection_Y =y;
    //              //  ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y , RGB565_RED);
    //                 break;
    //              }
    //              if(x>(left[y-1]+3)&&imag[y][x]==White){
    //                  Lower_left_inflection_Flag=1;
    //                  X1=Lower_left_inflection_X;
    //                  Y1=Lower_left_inflection_Y;
    //                  break;
    //                  }
    //            }
    //
    //
    //        }
}

//-------------------------------------���¹յ�---------------------------------
//-----------------------------�ڶ��棺�öϵ��ж�------------------------
void Lower_right(void){
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
    for(y=image_h-3;y>(Endline+10);y--){
        if(y>30){
            if((right[y-4]-right[y])>5&&right[y-4]==185&&(right[y+2]-right[y])<5&&left[y]<170){
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                return;
            }
        }

     }
    //----------------------------��һ�棺��ɨ�߷����ж�--------------------------------
    //���ɶ����¹յ�ʶ�𣬵�����ת��ʱ�Ĺս�Ҳ�ᱻ�ж���
    //        for(y=115;y>(Endline+15);y--){
    //            for(x=right[y+1];x<186&&x>1;x--){
    //                  if(imag[y][x-1]==White&&imag[y][x]==Black&&imag[y][x+1]==Black){
    //                     Lower_right_inflection_X =x;
    //                     Lower_right_inflection_Y =y;
    //                    // ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y , RGB565_RED);
    //                     break;
    //                  }
    //                  if(x<(right[y-1]-3)&&imag[y][x]==White){
    //                      Lower_right_inflection_Flag=1;
    //                      break;
    //                      }
    //                }
    //
    //            }

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
              for(h=Lost_point_L_scan_line+5;h>(Endline+10);h--){//�Ķ�
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
//float k11,k22,k33,k44;
void right_straight(void){
    float k1,k2,k3,k4;
    Right_straight_flag=0;   // ��ʼ����ֱ�߱�־

    // ���㲻ͬ�����б�������ж�ֱ�߶�
    k1=((float)right[90]-(float)right[60])/30;  // �ײ�����б��
    k2=((float)right[80]-(float)right[70])/10;  // �в�����б��
    k3=((float)right[20]-(float)right[70])/50;   // �������ײ�б��
    k4=((float)right[20]-(float)right[40])/20;   // �������в�б��
    //k5=((float)right[0]-(float)right[20])/20;   // �������ϲ�б��
    //k11=k1;k22=k2;k33=k3;k44=k4;
    // �ж��Ƿ�Ϊֱ��
    if(Endline<20&&absolute(k1-k2)<1&&absolute(k2-k3)<2.5&&absolute(k3-k1)<2.5&&k4<1&&k1!=0&&k2!=0&&k3!=0&&k4!=0&&Lost_right_Flag==0&&Lower_right_inflection_Flag==0&&(float)right[40]>80){
        Right_straight_flag=1;

        //printf(" %.4f, %.4f, %.4f\n",absolute(k1-k2),absolute(k2-k3),absolute(k3-k1));



    }
    //ips200_show_float(0, 180,k1,3,3);
    //ips200_show_float(110, 120,k2,3,3);
    //ips200_show_float(130, 120,k3,3,3);
    //ips200_show_float(150, 120,k4,3,3);
    //ips200_show_float(170, 120,(float)Lost_right_Flag,3,3);
    //ips200_show_float(190, 120,(float)Lower_right_inflection_Flag,3,3);
    //ips200_show_float(210, 120,(float)right[40],3,3);
}

//==================================================��ֱ��ʶ��===========================================================
void left_straight(void){
    float k1, k2, k3, k4;
    Left_straight_flag = 0;   // ��ʼ����ֱ�߱�־

    // ���㲻ͬ�����б�������ж�ֱ�߶�
    k1 = ((float)left[90] - (float)left[60]) / 30;   // �ײ�����б��
    k2 = ((float)left[80] - (float)left[70]) / 10;   // �в�����б��
    k3 = ((float)left[20] - (float)left[70]) / 50;   // �������ײ�б��
    k4 = ((float)left[20] - (float)left[40]) / 20;   // �������в�б��

    // �ж��Ƿ�Ϊֱ��
    if(Endline < 20
        && absolute(k1 - k2) < 1
        && absolute(k2 - k3) < 2.5
        && absolute(k3 - k1) < 2.5
        && k4 < 1
        && k1 != 0 && k2 != 0 && k3 != 0 && k4 != 0
        && Lost_left_Flag == 0
        && Lower_left_inflection_Flag == 0
        && (float)left[40] < 100)
    {
        Left_straight_flag = 1;
    }
}
//ips200_show_float(0, 260,k4,3,3);
//         ips200_show_float(70, 260,k5,3,3);
//         ips200_show_float(140, 260,k6,3,3);

//        ips200_show_float(100, 170,absolute(k4-k5),3,3);
//        ips200_show_float(100, 190,absolute(k5-k6),3,3);
//        ips200_show_float(100, 210,absolute(k6-k4),3,3);

//        k7=((float)middle[90]-(float)middle[60])/((float)middle[80]-(float)middle[70]);


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

//=============================================Բ��===================================================
//================����ʶ��=====================
bool annulus_3_to_4_flag(void){

}

void roundabout_L(void){

    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
        if((left[y]-left[y-8])>5&&(left[y]-left[y+2])<5&&left[117]==2&&Lost_left_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =left[y];
            roundabout_Y =y;
            return;
        }
     }

}

void roundabout_R(void){
    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=image_h-3;y>10;y--){
        if((right[y-8]-right[y])>5&&(right[y+2]-right[y])<5&&right[117]==185&&Lost_right_Flag==1){
            y+=4;
            roundabout_Flag=1;
            roundabout_X =right[y];
            roundabout_Y =y;
            return;
        }
     }

}
//================�����յ�ʶ��=====================
void Exit_loop_L_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //��Բ����ʶ���ұ߹յ�
    for(i=110;i>Endline+10;i--){
        if(right[i]<right[i+4]&&right[i]<right[i-4]&&right[i]<right[i+3]&&right[i]<right[i-3]){
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
  //��Բ����ʶ���ұ߹յ�
    for(i=110;i>Endline+10;i--){
        if(left[i+4]<left[i]&&left[i-4]<left[i]&&left[i+3]<left[i]&&left[i-3]<left[i]){
            Exit_loop_Flag=1;
            Exit_loop_X=left[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

//================��Բ��ʶ��=====================

void annulus_L(void){

    //ʶ��Բ��
    if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1&&Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==0&&annulus_L_memory==0&&annulus_L_Flag==0
    &&BridgeState==0&&jump_position_flag==0&&zebra_crossing_flag==0
    ){
        annulus_L_Flag=1;
        annulus_L_memory =1;

    }
    else if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1&&Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0&&annulus_L_memory==0&&annulus_L_Flag==0
    &&BridgeState==0&&jump_position_flag==0&&zebra_crossing_flag==0
    ){
        annulus_L_Flag=1;
        annulus_L_memory =2;
    }


    if(annulus_L_Flag==1){
//         duty_value = -1800;

         //״̬ 1  ʶ��Բ����δʶ�𵽻�������ǿ�в��ߣ��򲻲��ߣ������ԣ�
        if (annulus_L_memory == 1)
        {

            if(Lower_left_inflection_Flag==1){
                Addingline1( 1, Lower_left_inflection_X, Lower_left_inflection_Y);

            }
             else if(Lower_left_inflection_Flag==0/*&&(imag[90][3]==255&&imag[100][3]==255&&imag[110][3]==255&&imag[120][3]==255)*/){
                 annulus_L_memory = 2;
             }
         }
         //״̬2 ʶ��Բ����������������߽��в���
          if (annulus_L_memory == 2 )//2
         {

              roundabout_L();          //����

            if(roundabout_Flag==1){
                Addingline( 1, roundabout_X, roundabout_Y,24 , 118 );

             }
            else
            {
                Addingline( 1, 70, 5,24 , 118 );
            }
            if(Upper_left_inflection_Flag==1&&Upper_left_inflection_Y>25/*&&(Upper_left_inflection_Y>Upper_left_inflection_X/2)*/){
                            annulus_L_memory = 3;
            }
            else return;
         }
         //״̬3 ����Բ����ڣ���סǰ·�������뻷
          if (annulus_L_memory == 3 )//3
          {
//              mid_error = 5;
//              Roll_Zero = 4;


                  if(Upper_left_inflection_Flag==1){
                     Addingline( 2, Upper_left_inflection_X-38,Upper_left_inflection_Y,186 , 118);
                  }
                  else{
                    Addingline( 2, 2,50,186 , 118);
                      }
                      if(Upper_left_inflection_Flag==0&&Endline>10){
                                annulus_L_memory_flag = 1;
                            }
              else return;
          }

         //״̬4 ��Բ������ʻ�����������¹յ�ʱ������һ״̬
         if (annulus_L_memory == 4)
         {
            Exit_loop_L_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
            if(Lost_left_Flag==1&&Lost_right_Flag==1&&Endline<9){
                                          annulus_L_memory = 6;
                                      }
                         else return;
         }

//         ״̬6 ����ʱ���¹յ���ʧ�����ǳ���û��ȫ��������ʱ����Ҫ���ߴ���
         if (annulus_L_memory == 6 )
          {
             if(Lost_left_Flag==1&&Lost_right_Flag==1){
                 Addingline( 2, 3, Endline+3, 184, 117);
             }
             if(Upper_left_inflection_Flag==1&&Lost_left_Flag==1&&Lost_right_Flag==0){
                              annulus_L_memory = 7;
                       }
             }

         //״̬7 ��������
         if (annulus_L_memory == 7)
          {
             if(Upper_left_inflection_Flag==1){
                 Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 20, 118);
                 return;
             }
             if(Lost_right_Flag==0&&Lost_left_Flag==0){
                 annulus_L_memory =0;
                 annulus_L_Flag=0;
                 return;
             }


          }

     }
}



//================��Բ��ʶ��=====================
void annulus_R(void){

    //ʶ��Բ��
    if(Lost_right_Flag==1&&Lost_left_Flag==0&&Left_straight_flag==1&&Lower_right_inflection_Flag==1&&Lower_left_inflection_Flag==0&&annulus_R_memory==0&&annulus_R_Flag==0){
        annulus_R_Flag=1;
        annulus_R_memory =1;

    }

         if(annulus_R_Flag==1){


             //״̬ 1  ʶ��Բ����δʶ�𵽻�������ǿ�в��ߣ��򲻲��ߣ������ԣ�
             if (annulus_R_memory == 1)
             {
                 if(Lower_right_inflection_Flag==1){
//                     Addingline1( 2, Lower_right_inflection_X, Lower_right_inflection_Y);
                    Addingline( 2,130,2, Lower_right_inflection_X, Lower_right_inflection_Y );
                    //roundabout();
                 }
                 else if(Lower_left_inflection_Flag==0){
                     annulus_R_memory = 2;
                 }


             }
             //״̬2 ʶ��Բ����������������߽��в���
              if (annulus_R_memory == 2 )
             {
                  roundabout_R();          //����
                if(roundabout_Flag==1){
                    Addingline( 2, roundabout_X, roundabout_Y,178 , 118 );

                 }
                else {
                    Addingline( 2, 134, 30,178 , 118 );  //�����յ�
                }
                if(Upper_right_inflection_Flag==1){   //�ı�Upper_right_inflection_Y���Ƽ�Ӹı��뻷λ�ã�����֤
                    annulus_R_memory = 3;
                }
                else return;
             }
         //״̬3 ����Բ����ڣ���סǰ·�������뻷
          if (annulus_R_memory == 3 )
          {
              if(Upper_right_inflection_Flag==1){
                  //Addingline( 2, Upper_left_inflection_X,Upper_left_inflection_Y,right[119] , 119);
                  if(Upper_right_inflection_X>25){
                      Addingline( 1, Upper_right_inflection_X+15,Upper_right_inflection_Y,left[119] , 119);
                  }
                  else{
                      Addingline( 1, Upper_right_inflection_X,Upper_right_inflection_Y,left[119] , 119);

                  }

                  return;
              }
              else if(Upper_right_inflection_Flag==0&&Endline>10){
                  annulus_R_memory = 4;
              }
              else return;

          }
         //״̬4 ��Բ������ʻ�����������¹յ�ʱ������һ״̬
         if (annulus_R_memory == 4)
         {
            Exit_loop_R_inflection();
            if(Exit_loop_Flag==0){
                return;
            }
            if(Exit_loop_Flag==1&&Lost_left_Flag==1&&Endline<5){
                annulus_R_memory = 5;
            }

         }

         //״̬5 ����ʱ�������¹յ�,���䲹�ߴ���
         if (annulus_R_memory == 5 )
          {
             Exit_loop_R_inflection();
             if(Exit_loop_Flag==1){
                 if(Exit_loop_Y>35){
                     Addingline( 1, 187, Exit_loop_Y-30, Exit_loop_X, Exit_loop_Y);//�˴����߽����������
                     return;
                 }
                 else {
                     Addingline( 1, 187, 50, Exit_loop_X, Exit_loop_Y);//�˴����߽����������
                     return;
                 }
             }
             else if(Lost_right_Flag==1&&Lost_left_Flag==1&&Exit_loop_Flag==0){
                 annulus_R_memory = 6;
             }
             else return;

          }

         //״̬6 ����ʱ���¹յ���ʧ�����ǳ���û��ȫ��������ʱ����Ҫ���ߴ���
         if (annulus_R_memory == 6 )
          {

                 Addingline( 1, 187, Endline+3, 2, 117);

             if(Upper_right_inflection_Flag==1){
                 annulus_R_memory = 7;
             }
             else return;
          }
         //״̬7 ��������
         if (annulus_R_memory == 7)
          {
             if(Upper_right_inflection_Flag==1){
                 Addingline( 2, Upper_right_inflection_X+5, Upper_right_inflection_Y, 178, 118);
                 return;
             }
             else{
                 Addingline( 2, 130, 19, 178, 118); // Ҫ��
             }
//             Addingline( 2, left[2]+50, 2,left[118]+150 , 118 );
//             Addingline(2,140,2, 175,118 );
             if(Lost_right_Flag==0&&Lost_left_Flag==0){
                 annulus_R_memory =0;
                 annulus_R_Flag=0;
                 return;
             }

          }

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
     if(banmaxian_hangshu >= 4 &&  // ������4�м�⵽��Ч������
       BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&  // ���ڵ�����
       jump_position_flag == 0 &&  // û������λ�ñ��
       //sum_island == 0 && island == 0 &&  // ���������
       Crossroad_Flag == 0
       
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
    jump_position_flag = 0;// ��������λ��
    blake_line = 0; // ��¼����ȫ���е�����
    // ����ߵ�λ�á�55ʱ�����������
    if(*hightest>=55)
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
    if(blake_line >= 25)//���һ������������ʹ�һ�����40�ɣ�Զһ���������Сһ����С������ɣ�����Ч������delay���������delay,Ҳ�����������������
    {
        // ����ض�����߽��Ƿ��������޶�ʧ����˵ʵ������û��Ҫ�����������󴥷���Ծ�Ļ���ͼ��border
        //if(l_loss_judge(l_border,90 ,110) == 0 &&  // ��߽�90-110������
         //  l_loss_judge(l_border,70 ,90) == 0 &&   // ��߽�70-90������
        // r_loss_judge(r_border,90 ,110) == 0   // �ұ߽�90-110������
         //  r_loss_judge(r_border,70 ,90) == 0 // �ұ߽�70-90������
            //&&monotonicity_l == 1 && monotonicity_r == 1
         //   )
        {
             // ȷ�������������������š�������ʮ���ߣ�
            if(
                BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&
                //sum_island == 0 && island == 0
                annulus_L_memory==0&&
                zebra_crossing_flag==0&&
                Crossroad_Flag==0&&
            //      && cross_sum == 0
            annulus_L_memory_flag==0
            )
                //  �����mode�ĳ�jumpģʽ��Ҳ��flexibleģʽ����Ȼ����ȷ�Ϊ�Գƣ����ö�������ٶȻ�
            //mode=jump;
            //ServoPID.highleft=3.3;//�����ȸ�һ��
            //ServoPID.highright=3.3;
                //  delay,������ø����ٶ�д����ʱ���������ƣ��ٶȿ����������ٶ��������
            //  jump_delay(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data,int kp)�������������jump_judge����������
            jump_position_flag = 1;// ��������λ��,
        }

    }

}
// /**

// @brief �Ȳ��߶�ƽ����ƺ������������ٶȷֱ�����汾��
// @param void
// @return void
// @note ���ȸ����������ٶȣ����ȸ����������ٶȣ�11�ǵ�����б�ߵĳ���9.8��ֱ�Ǳߵĳ�
// */
// static uint8 left_adjust_flag = 0;
// static uint8 right_adjust_flag = 0;
// static float original_left_speed = 0; // ��¼����ԭʼ�ٶ�
// static float original_right_speed = 0; // ��¼����ԭʼ�ٶ�
// static float original_run_speed = 0; // ��¼ԭʼ�����ٶ�
// static uint8 speed_initialized = 0;
// static uint8 run_speed_stopped = 0; // �����ٶ�ֹͣ��־
// static float prev_left_height = 0; // ��¼�ϴ����ȸ߶�
// static float prev_right_height = 0; // ��¼�ϴ����ȸ߶�

// void leg_height_balance_control_wheel_speed(void)
// {
// float height_diff = ServoPID.highleft - ServoPID.highright;
// float adjustment_ratio;

// // ����߶ȱ仯����
// float left_height_change = ServoPID.highleft - prev_left_height;
// float right_height_change = ServoPID.highright - prev_right_height;

// // ���ݸ߶Ȳ���ͱ仯����ȷ����������
// if(height_diff > 0.1f)  // ���ȱ����ȸ�
// {
//     if(left_height_change >= 0)  // ���ȸ߶��������ӻ��ȶ�
//     {
//         adjustment_ratio = 11.0f / 9.8f * 3;  // ����ĵ�������
//     }
//     else  // ���ȸ߶��ڼ���
//     {
//         adjustment_ratio = 11.0f / 9.8f;      // ������������
//     }
// }
// else if(height_diff < -0.1f)  // ���ȱ����ȸ�
// {
//     if(right_height_change >= 0)  // ���ȸ߶��������ӻ��ȶ�
//     {
//         adjustment_ratio = 11.0f / 9.8f * 3;  // ����ĵ�������
//     }
//     else  // ���ȸ߶��ڼ���
//     {
//         adjustment_ratio = 11.0f / 9.8f;      // ������������
//     }
// }
// else
// {
//     adjustment_ratio = 1;  // �߶Ȳ���Сʱ������
// }

// // �״�����ʱ��¼ԭʼ�ٶ�
// if(speed_initialized == 0)
// {
//     original_left_speed = pid2_flexible.sudu_left.kp;   // ��¼����ԭʼ�ٶ�
//     original_right_speed = pid2_flexible.sudu_right.kp; // ��¼����ԭʼ�ٶ�
//     original_run_speed = pid2_flexible.run_speed;       // ��¼ԭʼ�����ٶ� (200)
//     speed_initialized = 1;
// }

// // ����Ƿ���Ҫֹͣ�����ٶ�
// // ���������Ȳ�һ���� �� ��һ����������
// if((my_abs((int)(height_diff * 10)) > 1) &&  // ���Ȳ�һ���ߣ��߶Ȳ� > 0.1��
//    ((left_height_change > 0.05f) || (right_height_change > 0.05f)))  // ��һ����������
// {
//     if(run_speed_stopped == 0)
//     {
//         pid2_flexible.run_speed = 0;  // ֹͣ�����ٶ�
//         run_speed_stopped = 1;        // �����ֹͣ
//     }
// }
// // ������������һ���߻�����һ�����ڼ���ʱ���ָ������ٶ�
// else if((my_abs((int)(height_diff * 10)) <= 1) ||  // ����һ���ߣ��߶Ȳ� <= 0.1��
//         ((left_height_change < -0.05f) || (right_height_change < -0.05f)))  // ��һ�����ڼ���
// {
//     if(run_speed_stopped == 1)
//     {
//         pid2_flexible.run_speed = original_run_speed;  // �ָ�ԭʼ�����ٶ�
//         run_speed_stopped = 0;                         // ����ֹͣ��־
//     }
// }

// // ������ȱ����ȸߣ����������ٶ�
// if(height_diff > 0.1f && right_adjust_flag == 0)
// {
//     pid2_flexible.sudu_right.kp *= adjustment_ratio;  // �����ٶȳ���11/9.8
//     right_adjust_flag = 1;
    
//     // �޷�����
//     if(pid2_flexible.sudu_right.kp > 10.0f)  // ����ʵ�������������
//     {
//         pid2_flexible.sudu_right.kp = 10.0f;
//     }
// }
// // ������ȱ����ȸߣ����������ٶ�
// else if(height_diff < -0.1f && left_adjust_flag == 0)
// {
//     pid2_flexible.sudu_left.kp *= adjustment_ratio;   // �����ٶȳ���11/9.8
//     left_adjust_flag = 1;
    
//     // �޷�����
//     if(pid2_flexible.sudu_left.kp > 10.0f)  // ����ʵ�������������
//     {
//         pid2_flexible.sudu_left.kp = 10.0f;
//     }
// }
// // �߶Ȼָ�ʱ���ٶ�Ҳ�ָ�
// else if(my_abs((int)(height_diff * 10)) <= 1)
// {
//     // �ָ������ٶ�
//     if(left_adjust_flag == 1)
//     {
//         pid2_flexible.sudu_left.kp = original_left_speed;
//     }
    
//     // �ָ������ٶ�
//     if(right_adjust_flag == 1)
//     {
//         pid2_flexible.sudu_right.kp = original_right_speed;
//     }
    
//     // ���ñ�־λ
//     left_adjust_flag = 0;
//     right_adjust_flag = 0;
// }

// // ������ʷ�߶�ֵ
// prev_left_height = ServoPID.highleft;
// prev_right_height = ServoPID.highright;
// }
// /**
//  * @brief �����Ȳ��߶ȵ������ϰ�����
//  * @param void
//  * @return void
//  * @note ����Ȳ��߶ȱ仯�������״̬
//  */
// void wheel_obstacle_height_control(void)
// {
//     static float prev_left_height = 0;
//     static float prev_right_height = 0;
//     static uint8 left_climbing_flag = 0;
//     static uint8 right_climbing_flag = 0;
    
//     // ����߶ȱ仯��
//     float left_height_change = ServoPID.highleft - prev_left_height;
//     float right_height_change = ServoPID.highright - prev_right_height;
    
//     // ��ȡ��ǰ����
//     float current_left_speed = motor_value.receive_left_speed_data;
//     float current_right_speed = -motor_value.receive_right_speed_data;
    
//     // �����������״̬
//     if(left_height_change > 0.05f && current_left_speed < (pid2_flexible.run_speed * 0.7f))
//     {
//         if(left_climbing_flag == 0)
//         {
//             left_climbing_flag = 1;
//             // �������ֹ���
//             pid2_flexible.sudu_left.kp *= 1.8f; // ����80%
//             // �������ֵ
//             if(pid2_flexible.sudu_left.kp > 20.0f)
//                 pid2_flexible.sudu_left.kp = 20.0f;
//         }
//     }
//     // �����Ѿ�����ȥ��
//     else if(left_height_change < -0.02f && left_climbing_flag == 1)
//     {
//         left_climbing_flag = 0;
//         // �ָ�������
//         pid2_flexible.sudu_left.kp = 0.18f;
//         pid2_flexible.run_speed = 50; // �������
//     }
    
//     // �����߼���ͬ
//     if(right_height_change > 0.05f && current_right_speed < (pid2_flexible.run_speed * 0.7f))
//     {
//         if(right_climbing_flag == 0)
//         {
//             right_climbing_flag = 1;
//             pid2_flexible.sudu_right.kp *= 1.8f;
//             if(pid2_flexible.sudu_right.kp > 20.0f)
//                 pid2_flexible.sudu_right.kp = 20.0f;
//         }
//     }
//     else if(right_height_change < -0.02f && right_climbing_flag == 1)
//     {
//         right_climbing_flag = 0;
//         pid2_flexible.sudu_right.kp = 0.18f;
//         pid2_flexible.run_speed = 50;
//     }
    
//     // ������ʷ�߶�
//     prev_left_height = ServoPID.highleft;
//     prev_right_height = ServoPID.highright;
// }

    
/**
* @brief ����״̬�������亯��
* @param uint8(*image)[image_w]     ��ֵ��ͼ��
* @param uint8 *l_border            ��߽�����
* @param uint8 *r_border            �ұ߽�����
* @param uint8 *center_line         ���������飨�����
* @param uint8* hightest            ��ߵ�ָ��
* @return �޷���ֵ��ͨ��BridgeStateȫ�ֱ����������״̬
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
int loss_1 = 0;       // �߽��ȱ仯������
int loss_2 = 0;       // �߽�λ��ͻ�������
int bridge_number = 0; // ��������������
int bridge_out_flag=0;
int bridge_in_flag=0;
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line, uint8* hightest)
{
    int white_line1 = 0;  // ��75�а�ɫ���ؼ���
    int white_line2 = 0;  // ��70�а�ɫ���ؼ���
    int long_start_l = 0; // ��߽�ͻ����ʼ��
    int long_end_l = 0;   // ��߽�ͻ�������
    int long_start_r = 0; // �ұ߽�ͻ����ʼ��
    int long_end_r = 0;   // �ұ߽�ͻ�������
     loss_1 = 0;       // �߽��ȱ仯������
     loss_2 = 0;       // �߽�λ��ͻ�������
     bridge_number = 0; // ��������������
    // ɨ��ͼ���м����򣬷����߽�仯
    for (int i = 30; i < image_h-10; i++)//for (int i = 30; i < image_h-10; i++)
    {
        // �жϵ�ǰ�б߽����Ƿ�����С����һ�У��������������
        if((right[i] - left[i]) *10*3/5> (right[i+3] - left[i+3])*10)
            loss_1++;
         // �ж���߽���ұ߽��Ƿ������ͻ�䣨����10���أ�
        if(my_abs(left[i] - left[i+3]) >= 10)
            loss_2++;
        if(my_abs(right[i] - right[i+3]) >= 10)
            loss_2++;
    }
    //������״̬Ϊ"δ����"ʱ������Ƿ��������
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        if(ServoPID.highleft>=3.4){ServoPID.highleft-=0.5;ServoPID.highright-=0.5;}
        gpio_set_level(BUZZER_PIN, GPIO_LOW);
        // ��ߵ�λ�ú���<=30��ʱ���м��
        if(*hightest <= 30)
        {
            // ͳ�Ƶ�75�к͵�70�еİ�ɫ��������
            for (int x = left[image_h-10]; x <= left[image_h-10]; x++)
            {
                if(image[75][x] == 255)
                {
                    white_line1++;
                }
                if(image[70][x] == 255)
                {
                    white_line2++;
                }
            }
            // ������������������
            // 1. ��70�а�ɫ�����������ڵ�75��
            // 2. �߽��ȱ仯������2
            // 3. �߽�λ��ͻ�������4
            // 4. �������������������������㡢ʮ���ߣ�
            if(white_line2 <= (white_line1*3/5) && loss_1>=2 && loss_2>=4)//70?????????????????????????????
            {
                if(//sum_island == 0 &&
                   //     island == 0 &&
                        jump_position_flag == 0&&
                        Crossroad_memory==0&&
                        Crossroad_Flag==0
                        )
                pid1_walk.run_speed=0;
                if(bridge_in_flag==0)bridge_in_flag=1;
                // ServoPID.highleft=7;
                // ServoPID.highright=7;
                
                // float temp=pid2_flexible.run_speed;
                // pid2_flexible.run_speed=0;
                // system_delay_ms(1600);
                // pid2_flexible.run_speed=temp;

            }
        }
    }
    // ������״̬Ϊ"����"ʱ������������������������
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
        if(ServoPID.highleft<5.3){ServoPID.highleft+=0.7;ServoPID.highright+=0.7;}
        //leg_height_balance_control_wheel_speed();
        // �ӵ�30�п�ʼɨ����߽磨ͼ���м�ƫ������
        for (int i = 30; i < image_h-1; i++)
        {
             // �����30�У�������ڿ���λ��
            if(i == 30)
            {
                // �����߽��Ƿ��ڱ�׼λ�ø�����94��7���أ�
                if(my_abs(left[i]-94)<=7)
                {
                    // �ӵ�31�п�ʼѰ����߽�ͻ���
                    for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        // �������ͼ��ײ���a=image_h-5������Ϊ��������
                        if(a == image_h-5)
                        {
                            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE; // ��������״̬Ϊδ����
                            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
                            mode=walk;
                            break;
                        }
                        // �����߽�ͻ�䣨����3�б仯����10���أ�������ͼ������
                        if(my_abs(left[a] - left[a-3]) >= 10 &&
                            image[a+2][left[a-3]] == 255 &&// ͻ����Ϸ�2�ж�Ӧλ��Ϊ��ɫ
                             image[a-3][left[a-3]-5] == 0)// ͻ����·�3�����5����Ϊ��ɫ
                        {
                            long_start_l = i;    // ��¼ͻ����ʼ�У���30�У�
                            long_end_l = a;      // ��¼ͻ������У���ǰ��a��
                            bridge_number++;     // ������������+1
                            break;
                        }

                    }
                }
            }
            // ����30��֮�������i>30��
            if(i>30 && my_abs(left[i] - left[i+3]) >= 10 && long_start_l == 0 && long_end_l == 0)
            {
                // ���ͼ�������Ƿ���������߽�����
                if(image[i][left[i+3]] == 255 && image[i+6][left[i+3]-5] == 0)
                {
                    long_start_l = i+3;// ��¼ͻ����ʼ�У���ǰ��+3��
                    // ��i+5�е�ͼ��ײ�֮��Ѱ��ͻ�������
                    for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        if(a == image_h-5)
                        {
                            long_end_l = a;// ����ײ�ʱ��¼������
                            bridge_number++;
                            break;
                        }
                        // ���߽�ͻ�䲢����ͼ������
                        if(my_abs(left[a] - left[a-3]) >= 10 &&
                                image[a+2][left[a-3]] == 255 && image[a-3][left[a-3]-5] == 0)
                        {
                            long_end_l = a;// ��¼ͻ�������
                            bridge_number++;
                            break;
                        }

                    }
                }
            }
            // ��ͻ�����������������ߣ�ʹ����߽���Ϊ��ʱ�����ߣ�
            if(i <= long_end_l && i >= long_start_l && long_start_l != 0 && long_end_l != 0)
            {
                middle[i] = left[i];// ����߽�ֵ��������������
                if(i+1 == long_end_l)// ����ͻ������ĩβʱ���ñ��
                {
                    long_end_l = 0;
                    long_start_l = 0;
                }
            }
        }
        // �ӵ�30�п�ʼɨ���ұ߽�
        for (int i = 30; i < image_h-1; i++)
        {
            if(i == 30)
            {
                // ����ұ߽��Ƿ��ڱ�׼λ�ø�����94��7����)
                if(my_abs(r_border[i]-94)<=7)
                {
                    // �ӵ�31�п�ʼѰ���ұ߽�ͻ���
                    for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        if(a == image_h-5)
                        {
                            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;// ��������
                            break;
                        }
                         // ����ұ߽�ͻ�䲢����ͼ������������߽�Գƣ�
                        if(my_abs(right[a] - right[a-3]) >= 10 &&
                                image[a+2][right[a-3]] == 255 &&
                                 image[a-3][right[a-3]+5] == 0)// �Ҳ�5����Ϊ��ɫ
                        {
                            long_start_r = i;// ��¼�ұ߽�ͻ����ʼ��
                            long_end_r = a;  // ��¼�ұ߽�ͻ�������
                            bridge_number++;
                            break;
                        }
                    }
                }
            }
            if(i>30 && my_abs(right[i] - right[i+3]) >= 10 && long_start_r == 0 && long_end_r == 0)
            {
                if(image[i][right[i+3]] == 255 && image[i+6][right[i+3]+5] == 0)
                {
                    long_start_r = i+3; // ��¼ͻ����ʼ��
                    for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        if(a == image_h-5)
                        {
                            long_end_r = a;
                            bridge_number++;
                            break;
                        }
                        if(my_abs(right[a] - right[a-3]) >= 10 &&
                            image[a+2][right[a-3]] == 255 && image[a-3][right[a-3]+5] == 0)
                        {
                            long_end_r = a;
                            bridge_number++;
                            break;
                        }
                    }
                }
            }
            // ���ұ߽�ͻ������������������
            if(i <= long_end_r && i >= long_start_r && long_start_r != 0 && long_end_r != 0)
            {
                middle[i] = right[i];// ���ұ߽�ֵ��������������
                // ����ͻ������ĩβʱ���ñ��
                if(i+1==long_end_r)
                {
                    long_end_r = 0;
                    long_start_r = 0;
                }
            }
        }
        // ��û�б߽��ȱ仯��û������������û�б߽�λ��ͻ��ʱ����Ϊ��������
        if(loss_1 == 0 && bridge_number == 0 && loss_2 == 0)
        {
            
             bridge_out_flag=1;
        }
    }
}/**
// * @brief ����������亯��
// * @param uint8(*image)[image_w]     �Ҷ�ͼ������
// * @param uint8 *l_border            ���Ե����
// * @param uint8 *r_border            �ұ�Ե����
// * @param uint16 total_num_l         ���Ե������
// * @param uint16 total_num_r         �ұ�Ե������
// * @param uint16 *dir_l              ���Ե��������
// * @param uint16 *dir_r              �ұ�Ե��������
// * @param uint16(*points_l)[2]       ���Ե����������
// * @param uint16(*points_r)[2]       �ұ�Ե����������
// * @param uint8* hightest            �����Ч��ָ��
// * @param uint16 *l_index            ���Ե��������
// * @param uint16 *r_index            �ұ�Ե��������
// * @param int monotonicity_l         ���Ե������
// * @param int monotonicity_r         �ұ�Ե������
// * @return �޷���ֵ������洢��ȫ�ֱ�����
// *     -<em>false</em> fail
// *     -<em>true</em> succeed
//  */
// void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
//                  uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
//                  uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
// {
//    uint16 i;
//    int black_line_sum = 0; // ��ɫ��������
//    int black_sum_1 = 0;// �Ҷ�ֵ״̬���
//    int black_sum_2 = 0;
//    uint8 break_num_l = 0;// ���ұ�Եͻ����к�
//    uint8 break_num_r = 0;
//    uint8 end_num_l = 0;// ���ұ�Ե�ָ����к�
//    uint8 end_num_r = 0;
//    uint8 start, end;  // ���������ֹ�к�
//    int ap = 1;  // �����Ч�Ա��
//    float slope_l_rate = 0, intercept_l = 0; // ֱ����ϲ���
//   // �������������Ե�ǵ������ұ�Ե������
//    if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
//    {
//          // ������Ե����λ��
//         broken_line_judge(1,*hightest,110,l_border);
//         // ����ұ�Ե�Ƿ����ڲ�����������������
//         for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//         {
//             if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
//             {
//                 ap = 0;// ���Ϊ��Ч���
//                 break;
//             }
//         }
//          // ͳ��ͼ���еĺ�ɫ����������Ϊ�����ߣ�
//         for (i = broken_line_y+5; i > *hightest; i--)
//         {
//             for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
//             {
//                 // ����ɫ������ʼ
//                 if (image[i][x] == 0 && image[i][x-1] == 255)
//                 {
//                     black_sum_1 = 1;
//                 }
//                 // ����ɫ��������
//                 if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
//                 {
//                     black_sum_2 = 1;
//                 }
//                  // ����״̬���
//                 if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
//                 {
//                     black_sum_1 = 0;
//                     black_sum_2 = 0;
//                     break;
//                 }
//             }
//             // �ۼ���Ч��ɫ������
//             if(black_sum_2 == 1)
//             {
//                 black_sum_1 = 0;
//                 black_sum_2 = 0;
//                 black_line_sum++;
//             }
//             if(i==15)// �����ǰ15��
//                 break;
//         }
//         // �ۺ��ж��Ƿ�Ϊ�����
//         if(black_line_sum>=5 /*ȷ��������ʶ�����*/
//                 && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
//                 /*������������Ե��ʧ�ж�*/
//                 && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
//             /*����������ʮ�ַ����ŽӲ���ͻ*/
//         {
//                 sum_island = 1;// ���Ϊ�������
//                 island = 1;// ���Ϊ�����
//                 black_line_sum = 0;
//         }
//     }
//     // �����������ұ�Ե�ǵ��������Ե������
//     if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
//     {
//         // ����ұ�Ե����λ��
//         broken_line_judge(1,*hightest,110,r_border);
//         // ������Ե�Ƿ����ڲ�����������������
//         for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//         {
//             if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || l_border[w] != border_min))
//             {
//                 ap = 0;// ���Ϊ��Ч���
//                 break;
//             }
//         }
//         // ͳ��ͼ���еĺ�ɫ�������߼���������ͬ��
//         for (i = broken_line_y+5; i > *hightest; i--)
//         {
//             for (int x = (int)r_border[i]; x >= (int)l_border[i]; x--)
//             {
//                 if (image[i][x] == 0 && image[i][x+1] == 255)
//                 {
//                     black_sum_1 = 1;
//                 }
//                 if (black_sum_1 == 1 && image[i][x+1] == 0 && image[i][x] == 255)
//                 {
//                     black_sum_2 = 1;
//                 }
//                 if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x+1] == 255)
//                 {
//                     black_sum_1 = 0;
//                     black_sum_2 = 0;
//                     break;
//                 }
//             }
//             if(black_sum_2 == 1)
//             {
//                 black_sum_1 = 0;
//                 black_sum_2 = 0;
//                 black_line_sum++;
//             }
//             if(i==15)//???��????????
//                 break;
//         }
//         // �ۺ��ж��Ƿ�Ϊ��������߼���������ͬ��
//         if(black_line_sum>=5 /*ȷ��������ʶ�����*/
//                 && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
//                /*������������Ե��ʧ�ж�*/
//                 && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
//             /*����������ʮ�ַ����ŽӲ���ͻ*/
//         {
//             sum_island = 1;  // ���Ϊ�������
//             island = 2;      // ���Ϊ�����
//             black_line_sum = 0;
//         }
//     }
//     // �����������island=1��
//     if(island == 1)
//     {
//          // ȷ���������sum_island=1��
        
//         if(sum_island == 1)
//         {
//             // ���¼�����Ե����λ��
//             broken_line_judge(1,*hightest,110,l_border);
//             // ����ұ�Ե�Ƿ�������ڲ�����
//             for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//             {
//                 if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
//                 {
//                     ap = 0;// �����Ч���˳�״̬
//                     break;
//                 }
//             }
//              // �������Ч������״̬
//             if(ap == 0)
//             {
//                 island = 0;
//                 sum_island = 0;
//             }
//             // �������Ч�Ҷ���λ�ú��ʣ��޸����Ե
//             if(broken_line_y >= 20 && ap == 1)
//             {
//                 // ��С���˷�������Ե
//                 start = broken_line_y+5;
//                 end = broken_line_y+10;
//                 calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
//                 // �����ֱ���޸����Ե
//                 for (i = 1; i < broken_line_y+1; i++)
//                 {
//                     l_border[i] = slope_l_rate * (i)+intercept_l;
//                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//                 }
//                 // ����Ƿ���Ҫ��һ������
//                 if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
//                 {
//                     // ͨ����Ե��ȷ�����״̬
//                     for (i = 106; i > 15; i--)
//                     {
                        
//                         if(points_l[l_index[i]][0]>points_l[l_index[i-5]][0] && points_l[l_index[i]][0]>points_l[l_index[i+5]][0]
//                             && points_l[l_index[i-5]][0] != border_min)
//                         {
//                             sum_island = 2;// ����״̬Ϊȷ�����
//                         }
//                     }
//                 }
//             }
//         }
//         // ȷ���������sum_island=2������һ������
//         if(sum_island == 2)
//         {
//             int dp = 0;
//             int temph = 0;
//             int vp = 0;
//             uint16 h = 0;
//             // ���ͼ��ײ����أ�����Ϊ����������
//             if (!image[image_h - 5][5] && !image[image_h - 3][3])
//             {
//                 dp = 1;
//             }
//             // ����Եͻ��λ��
//             if (dp)
//             {
//                 for (h = image_h - 15; h > 5; h--)
//                 {
//                     if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
//                     {
//                         temph = h;
//                         break;
//                     }
//                 }
//             }
//             // ȷ�ϱ�Եͻ���
//             if (temph)
//             {
//                 for (int j = l_index[h]; j > 0; j--)
//                 {
//                     if (points_l[j][1] >= points_l[j + 3][1]
//                          &&points_l[j][1] > points_l[j + 5][1]
//                          &&points_l[j][1] >= points_l[j - 3][1]
//                          &&points_l[j][1] >= points_l[j - 5][1])
//                     {
//                         vp = h;
//                         break;
//                     }
//                 }
//             }
//              // Ѱ�ұ�Ե��͵�
//             for (i = 25; i < image_h - 15; i++)
//             {
//                 if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
//                    && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
//                    && l_border[i-5] != border_min && l_border[i+5] != border_min)
//                 {
//                     end_num_l = (uint8)i;
//                 }
//                 // ״̬����Ϊ׼�����
//                 if(vp && end_num_l >= 80)
//                     sum_island = 3;
//             }
//                 // ����ұ�Ե����������������Ե�ָ���
//                 slope_l_rate = (float)(118-end_num_l) / ((border_max-r_border[118]+border_min)-l_border[end_num_l]);//��??k=y/x
//                 intercept_l = 118 - slope_l_rate*(border_max-r_border[118]+border_min);//???b=y-kx
//                 for (i = end_num_l; i < image_h - 1; i++)
//                 {
//                     l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//                 }

//         }
//         //����ȷ�������
//         if(sum_island == 3)
//         {
//             uint16 h = 0;
//             int temph_l = 0;
//             // 1. Ѱ�����Եͻ��㣨�����ʼλ�ã�
//             for (h = image_h - 15; h > 5; h--)// ��ͼ��ײ�����ɨ��
//             {
//                 if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
//                 {
//                     temph_l = h;  // ��¼ͻ���к�
//                     break;  // �ҵ��������˳�ѭ��
//                 }
//             }
//              // 2. ��ͻ��㸽��Ѱ�����ת�۵�
//             if (temph_l)// ����ҵ���ͻ���
//             {
//                 for (int i = total_num_l - 10; i > [h + 1]; i--)
//                 {
//                     // Ѱ��y����ֲ����ֵ���������������
//                     if (points_l[i][1] >= points_l[i + 3][1]  // ��ǰ��y����ݺ�3����
//                         && points_l[i][1] > points_l[i + 5][1]  // ��ǰ��y����>��5����
//                         && points_l[i][1] >= points_l[i - 3][1]  // ��ǰ��y�����ǰ3����
//                         && points_l[i][1] >= points_l[i - 5][1]  // ��ǰ��y�����ǰ5����
//                         // λ�ú�����Լ��
//                         && points_l[i][0] > points_l[i - 5][0]  // ��ǰ��x����>ǰ5����
//                         && points_l[i][0] <= points_l[i + 5][0])  // ��ǰ��x����ܺ�5����
//                     {
//                         break_num_l = (uint8)points_l[i][1];  // ��¼ת�۵�y����
//                         end_num_l  = (uint8)points_l[i][0];  // ��¼ת�۵�x����
//                         break;  // �ҵ��������˳�ѭ��
//                     }
//                 }
//             }
//             //ʹ�ø���Χ�ıȽ�
// //            for (i = 40; i < total_num_l-30; i++)
// //            {
// //                if (points_l[i][1]>points_l[i+9][1]&&points_l[i][1]>points_l[i-9][1]
// //                      &&points_l[i][1]>points_l[i+15][1]&&points_l[i][1]>points_l[i-15][1])
// //                  {
// //                     break_num_l = (uint8)points_l[i][1];//????y????
// //                     end_num_l  = (uint8)points_l[i][0];//????x????
// //                  }
// //            }

//             // 4. ���ͼ��ײ���Ե������ȷ���Ƿ���������
//             if((*hightest >= 30)&&  // ��ߵ�λ�ú���ȷ������������
//               (!image[image_h - 1][3] && !image[image_h - 3][3] // ͼ�����½�Ϊ��ɫ�������߽磩
//                 && !image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))// ͼ�����½�Ϊ��ɫ
//             {
//                 sum_island = 4;//�������������״̬4
//             }

//             // 5. �����ұ߽磨����ת�۵����ֱ�߷��̣�
//             if(break_num_l && end_num_l)  // ����ҵ���ת�۵�
//             {
//                 // ����б��k = ��y/��x����(186,118)Ϊ�ο��㣩
//                 slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);
//                 intercept_l = 118 - slope_l_rate*186;// ����ؾ�b = y - kx
//                  // ����ֱ�߷��̸����ұ߽�x = (y - b)/k
//                 for (i = 1; i < image_h - 1; i++)
//                 {
//                     r_border[i] = ((i)-intercept_l)/slope_l_rate;// ����ÿ��y��Ӧ��x����
//                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max); // ��������Ч��Χ��
//                 }
//             }
//         }
//         // ��ǰ����״̬4����֤���������
//         if(sum_island == 4)
//         {
//              int g=0;  // ��������ƥ���־
    
//              // 1. Ѱ���ض��������У�4��4��6��6��6����ʾ����ת��
//             for (i = 1; i < total_num_l; i++)
//             {
//                 if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
//                 {
//                     g=1;
//                     break;
//                 }
//             }
//             // 2. ����ұ߽�ͻ�䣨���ʱ仯�㣩
//             for (i = image_h - 20; i > *hightest; i--)//?????��?
//             {
//                 if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>30)
//                 {
//                     if(i>=30 && i<=105 && g)
//                         sum_island = 5;
//                 }
//             }
//         }
//         // ��ǰ����״̬5�������������
//         if(sum_island == 5)
//         {
//             // 1. Ѱ���ұ߽�ͻ��㣨������󴦣�
//             for (uint16 w = image_h - 15; w > *hightest; w--)//?????��?
//             {
//                 if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
//                 {
//                     break_num_l = (uint8)w;//????y????
//                     break;
//                 }
//             }
//             // 2. �ٴ�ȷ�Ϸ�������������ͬ״̬4��
//             for (i = 1; i < total_num_l; i++)
//             {
//                 if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
//                 {
//                     end_num_l = (uint8)points_l[i][1];//????y????
//                     break;
//                 }
//             }
//             // 3. ����ұ߽��Ƿ���ڣ�δ��ʧ��
//             if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
//                 sum_island = 6;
//             // 4. �����ұ߽磨�����µ����ʼ���ֱ�߷��̣�
//             slope_l_rate = (float)(break_num_l-end_num_l) / (r_border[break_num_l]-l_border[end_num_l]);//��??k=y/x
//             intercept_l = 0 - slope_l_rate*0;//�����½ؾ�b=y-kx
//              // ������ֱ�߷��̸����ұ߽磨�����µ�ͻ����·���
//             for (i = 1; i < break_num_l-3; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//         }
//         // �������״̬
//         if(sum_island == 6)
//         {
//             int dp = 1;// �����Ƿ�ִ�������ѭ�����̶�Ϊ1����ʾִ�У�
//             int temph = 0;
//             int vp = 0;
//             uint16 h = 0;
//              // 1. Ѱ�����Եͻ��㣨�������������
//             if (dp)
//             {
//                 for (h = image_h - 15; h > 5; h--)
//                 {
//                     if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
//                     {
//                         temph = h;
//                         break;
//                     }
//                 }
//             }
//             // 2. ��ͻ��㸽��Ѱ�������㣨y����ֲ����ֵ��
//             if (temph)
//             {
//                 for (int j = [h]; j > 0; j--)
//                 {
//                     if (points_l[j][1] >= points_l[j + 3][1]  // ��ǰ��y����ݺ�3����
//                         && points_l[j][1] > points_l[j + 5][1]  // ��ǰ��y����>��5����
//                         && points_l[j][1] >= points_l[j - 3][1]  // ��ǰ��y�����ǰ3����
//                         && points_l[j][1] >= points_l[j - 5][1])  // ��ǰ��y�����ǰ5����
//                     {
//                         vp = h;
//                         break;
//                     }
//                 }
//             }
//              // 3. ״̬ת��������������������ұ߽��ض�����ʧ��
//             if(vp && r_loss_judge(r_border, 70 ,90) == 0// �ұ߽���70-90�ж�ʧ
//                     && r_loss_judge(r_border, 50 ,70) == 0)// �ұ߽���50-70�ж�ʧ
//                 sum_island = 7;
//             // 4. �����ұ߽磨�ָ�Ĭ��ֱ��
//             slope_l_rate = (float)(118-0) / (186-0);// ����Ĭ��б��k=y/x
//             intercept_l = 0 - slope_l_rate*0;//����Ĭ�Ͻؾ�b=y-kx
//             // ����Ĭ��ֱ�߷��̸����ұ߽�
//             for (i = 1; i < image_h - 1; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//         }
//         // ��������ж�״̬
//         if(sum_island == 7)
//         {
//             uint16 h = 0;
//             int temph = 0;
//             // 1. Ѱ�����Եͻ��㣨�������������
//             for (h = image_h - 25; h > 5; h--)
//             {
//                 if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
//                 {
//                     temph = h;  // ��¼ͻ���к�
//                     break_num_l = (uint8)h;  // ��¼ת�۵�y����
//                     end_num_l  = (uint8)l_border[h];  // ��¼ת�۵�x����
//                     break;  // �ҵ��������˳�ѭ��
//                 }
//             }
// //            if (temph)
// //            {
// //                for (int i = total_num_l - 10; i > l_index[h + 1]; i--)
// //                {
// //
// //                    if (points_l[i][1] >= points_l[i + 3][1]
// //                        &&points_l[i][1] > points_l[i + 5][1]
// //                        &&points_l[i][1] >= points_l[i - 3][1]
// //                        &&points_l[i][1] >= points_l[i - 5][1]
// //                        &&points_l[i][0] > points_l[i - 5][0]
// //                        &&points_l[i][0] <= points_l[i + 5][0])
// //                    {
// //                        break_num_l = (uint8)points_l[i][1];//????y????
// //                        end_num_l  = (uint8)points_l[i][0];//????x????
// //                        break;
// //                    }
// //                }
// //            }
//             // 3. ���ͼ��ײ���Ե������ȷ���Ƿ�ص�ֱ����
//             f (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]  // ���½�Ϊ��ɫ
//                 && !image[image_h - 7][image_w - 3]  // ���½��Ϸ�Ϊ��ɫ
//                 && !image[image_h - 5][3] && !image[image_h - 3][3]  // ���½�Ϊ��ɫ
//                 && !image[image_h - 7][3])  // ���½��Ϸ�Ϊ��ɫ
//             {
//                 if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0 // �ұ߽��ض�����ʧ
//                    && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0) // ��߽��ض�����ʧ
//                 {
//                     sum_island = 0;
//                     island = 0;// �����������
//                 }
//             }
//             // 4. ������߽磨�����µ�б�ʣ�����˫�����߽磩
//             slope_l_rate = (float)(118-end_num_l) / (2-l_border[end_num_l]);//������б��k=y/x
//             intercept_l = 118 - slope_l_rate*2;//�����½ؾ�b=y-kx
//             // ������ֱ�߷��̸�����߽磨������ת�۵��·�����
//             for (i = end_num_l-10; i < image_h - 1; i++)
//             {
//                 l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//             }
//         }
//     }
//      // ��ǰʶ��Ϊ�������island=2��
//     if(island == 2)
//     {
//         // ����״̬1�����������
//         if(sum_island == 1)
//         {
//             broken_line_judge(1,*hightest,110,r_border); // ����ұ߽����
//             // ��֤��߽������ԣ���ֹ����Ϊ�����
//             for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//             {
//                 if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || r_border[w] != border_min))
//                 {
//                          ap = 0;// ���Ϊ�������߽�
//                          break;
//                 }
//             }
//             // �߽粻������ȡ��������
//             if(ap == 0)
//              {
//                  island = 0;
//                  sum_island = 0;
//              }
//              // ����λ�ú����ұ߽�����ʱ�����ұ߽�
//             if(broken_line_y >= 20 && ap == 1)
//             {
//                 start = broken_line_y+5;  // �����ʼ��
//                 end = broken_line_y+10;    // ��Ͻ�����
//                 calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
//                 // ����ֱ�߷��̸����ұ߽�
//                 for (i = 1; i < broken_line_y+1; i++)
//                 {
//                     r_border[i] = slope_l_rate * (i)+intercept_l;
//                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//                 }
//                  // ����ұ߽�ͻ���ʧ��������һ״̬
//                 if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
//                 {
//                     for (i = 106; i > 15; i--)
//                     {
//                         // Ѱ���ұ߽�ֲ���Сֵ�����������
//                         if(points_r[r_index[i]][0]<points_r[r_index[i-5]][0] && points_r[r_index[i]][0]<points_r[r_index[i+5]][0]
//                             && points_r[r_index[i-5]][0] != border_max)
//                         {
//                             sum_island = 2;
//                         }
//                     }
//                 }
//             }
//         }
//      // ״̬2��ȷ�����������
//     if(sum_island == 2)
//     {
//         int dp = 0;
//         int temph = 0;
//         int vp = 0;
//         uint16 h = 0;
//          // ���ͼ��ײ������㣨�Ҳ��Ե��
//         if (!image[image_h - 5][183] && !image[image_h - 3][185])
//         {
//             dp = 1;// ���Ϊ��Ч�������
//         }
//          // Ѱ���ұ�Եͻ��㣨�����ʼ��
//         if (dp)
//         {
//             for (h = image_h - 15; h > 5; h--)
//             {
//                 if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
//                 {
//                     temph = h;
//                     break;
//                 }
//             }
//         }
//         // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
//         if (temph)
//         {
//             for (int j = r_index[h]; j > 0; j--)
//             {
//                 if (points_r[j][1] >= points_r[j + 3][1]
//                      &&points_r[j][1] > points_r[j + 5][1]
//                      &&points_r[j][1] >= points_r[j - 3][1]
//                      &&points_r[j][1] >= points_r[j - 5][1])
//                 {
//                     vp = h;
//                     break;
//                 }
//             }
//         }
//          // Ѱ���ұ߽�ֲ���Сֵ���������㣩
//         for (i = 25; i < image_h - 15; i++)
//         {
//             if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
//                && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
//                && r_border[i-5] != border_max && r_border[i+5] != border_max)
//             {
//                 end_num_r = (uint8)i;
//             }
//             // �����������λ�ú���ʱ����״̬3
//             if(vp && end_num_r >= 80)
//                 sum_island = 3;
//         }
//              // �����ұ߽磨�������ʼ��㣩
//             slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//��??k=y/x
//             intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
//             for (i = end_num_r-10; i < image_h - 1; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//     }
//     // ״̬3���������������
//     if(sum_island == 3)
//     {
//         uint16 h = 0;
//         int temph_l = 0;
//         // Ѱ���ұ�Եͻ��㣨ϸ��λ�ã�
//         for (h = image_h - 15; h > 5; h--)
//         {
//             if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
//             {
//                 temph_l = h-5;
//                 break;
//             }
//         }
//          // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
//         if (temph_l)
//         {
//             for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
//             {

//                 if (points_r[i][1] >= points_r[i + 3][1]// y����ֲ����
//                     &&points_r[i][1] > points_r[i + 5][1]
//                     &&points_r[i][1] >= points_r[i - 3][1]
//                     &&points_r[i][1] >= points_r[i - 5][1]
//                     &&points_r[i][0] < points_r[i - 5][0]
//                     &&points_r[i][0] >= points_r[i + 5][0])
//                 {
//                     break_num_r = (uint8)points_r[i][1];// ��¼������y����
//                     end_num_r  = (uint8)points_r[i][0];// ��¼������x����
//                     break;
//                 }
//             }
//         }
// //        for (i = 40; i < total_num_r-30; i++)
// //        {
// //            if (points_r[i][1]>points_r[i+9][1]&&points_r[i][1]>points_r[i-9][1]
// //                  &&points_r[i][1]>points_r[i+15][1]&&points_r[i][1]>points_r[i-15][1])
// //              {
// //                 break_num_r = (uint8)points_r[i][1];//????y????
// //                 end_num_r  = (uint8)points_r[i][0];//????x????
// //              }
// //        }
//         // ��ߵ�λ�ú���ʱ����״̬4
//         if(*hightest >= 30)
//         {
//             sum_island = 4;
//         }
//         // ������߽磨��������������㣩
//         if(break_num_r && end_num_r)
//         {
           
//                 slope_l_rate = (float)(break_num_r-118) / (end_num_r-2);//k=y/x
//                 intercept_l = 118 - slope_l_rate*2;//b=y-kx
//                 for (i = 1; i < image_h - 1; i++)
//                 {
//                     l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//                 }
//         }
//     }
//     // ״̬4�������߽�ͻ��
//     if(sum_island == 4)
//     {
//         for (i = image_h - 15; i > *hightest; i--)//?????��?
//         {
//             if (l_border[i] > l_border[i - 3] && (l_border[i] - l_border[i-3])>20)
//             {
//                 if(i>=30 && i<=105)
//                   sum_island = 5;
//             }
//         }
//     }
//      // ״̬5��������߽����
//     if(sum_island == 5)
//     {
//         for (uint16 w = image_h - 15; w > *hightest; w--)//?????��?
//         {
//             if (l_border[w] > l_border[w - 3] && (l_border[w] - l_border[w-3])>20)
//             {
//                 break_num_r = (uint8)w;//????y????
//                 break;
//             }
//         }
// //        for (i = 1; i < total_num_l; i++)
// //        {
// //            if (dir_l[i - 10] >= 4 && dir_l[i - 10] <= 6 && dir_l[i-5] >= 4 && dir_l[i-5] <= 6
// //                    && dir_l[i] >= 2 && dir_l[i] <= 4 && dir_l[i + 5] <= 4 && dir_l[i + 5] >= 2
// //                    && dir_l[i + 10] <= 4 && dir_l[i + 10] >= 2)
// //            {
// //                break_num_r = (uint8)points_r[i][1];//????y????
// //                break;
// //            }
// //        }
//         // �ұ߽����ʱ����״̬6
//         if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
//             sum_island = 6;
//         // ������߽磨����ͻ���
//         end_num_r = l_border[break_num_r];
//         slope_l_rate = (float)(break_num_r-0) / (end_num_r-188);//��??k=y/x
//         intercept_l = 0 - slope_l_rate*188;//b=y-kx
//         for (i = 1; i < break_num_r-3; i++)
//         {
//             l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//         }
//     }
//     // ״̬6�����������
//     if(sum_island == 6)
//     {
//         int dp = 1;
//         int temph = 0;
//         int vp = 0;
//         uint16 h = 0;
//         if (dp)
//         {
//             // Ѱ���ұ�Եͻ��㣨�������������
//             for (h = image_h - 15; h > 5; h--)
//             {
//                 if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
//                 {
//                     temph = h;
//                     break;
//                 }
//             }
//         }
//          // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
//         if (temph)
//         {
//             for (int j = r_index[h]; j > 0; j--)
//             {
//                 if (points_r[j][1] >= points_r[j + 3][1]
//                      &&points_r[j][1] > points_r[j + 5][1]
//                      &&points_r[j][1] >= points_r[j - 3][1]
//                      &&points_r[j][1] >= points_r[j - 5][1])
//                 {
//                     vp = h;
//                     break;
//                 }
//             }
//         }
//         // �߽綪ʧʱ����״̬7����������жϣ�
//         if(vp && l_loss_judge(r_border, 90 ,110) == 0 && l_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(r_border, 50 ,70) == 0)
//             sum_island = 7;
//          // ������߽磨�ָ�Ĭ��ֱ�ߣ�
//         slope_l_rate = (float)(118-0) / (0-188);//k=y/x
//         intercept_l = 0 - slope_l_rate*188;//b=y-kx
//         for (i = 1; i < image_h - 1; i++)
//         {
//             l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//         }
//     }
//      // ״̬7������������ж�
//     if(sum_island == 7)
//     {
//         uint16 h = 0;
//         int temph = 0;
//         // Ѱ���ұ�Եͻ��㣨�������
//         for (h = image_h - 15; h > 5; h--)
//         {
//             if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
//             {
//                 temph = h;
//                 break;
//             }
//         }
//         // Ѱ���ұ�Ե�����㣨�ֲ���Сֵ��
//         if (temph)
//         {
//             for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
//             {

//                 if (points_r[i][1] >= points_r[i + 3][1]
//                     &&points_r[i][1] > points_r[i + 5][1]
//                     &&points_r[i][1] >= points_r[i - 3][1]
//                     &&points_r[i][1] >= points_r[i - 5][1]
//                     &&points_r[i][0] < points_r[i - 5][0]
//                     &&points_r[i][0] >= points_r[i + 5][0])
//                 {
//                     break_num_r = (uint8)points_r[i][1];//????y????
//                     end_num_r  = (uint8)points_r[i][0];//????x????
//                     break;
//                 }
//             }
//         }
//         // �ײ���Ե��������ʱ�ص���ʼ״̬
//         if (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]
//              && !image[image_h - 7][image_w - 3]
//              && !image[image_h - 5][3] && !image[image_h - 3][3]
//              && !image[image_h - 7][3])
//         {
//             if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
//                && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0
//                && r_loss_judge(r_border, 50 ,70) == 0 && l_loss_judge(l_border, 50 ,70) == 0)
//             {
//                 sum_island = 0;
//                 island = 0;
//             }
//         }
//          // �����ұ߽磨�����������
//         slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//��??k=y/x
//         intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
//         for (i = end_num_l-10; i < image_h - 1; i++)
//         {
//             r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//         }
//     }
//    }
// }

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


/*
  //********************�յ���ʾ*****************************
          //---------------���¹յ�--------------------
          //  if(Lower_left_inflection_Flag==1){
                ips200_show_string(0,125,"low left");

                ips200_show_int(0, 140, Lower_left_inflection_X,3);
                ips200_show_int(0, 155, Lower_left_inflection_Y,3);
          //      ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y,  RGB565_BLUE);
          //
          //  }
//            ips200_show_string(100,160,"low Lflag");
//            ips200_show_int(200, 160,Lower_left_inflection_Flag,3);
            //---------------���¹յ�-------------------
          //  if(Lower_right_inflection_Flag==1){
//                ips200_show_string(0,170,"low right");
//                ips200_show_int(0, 185, Lower_right_inflection_X,3);
//                ips200_show_int(0, 200, Lower_right_inflection_Y,3);
          //      ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y,  RGB565_BLUE);
          //
          //  }
//            ips200_show_string(100,150,"low Rflag");
//            ips200_show_int(200, 150,Lower_right_inflection_Flag,3);
            //---------------���Ϲյ�-------------------
          //  if(Upper_left_inflection_Flag==1){
                ips200_show_string(100,125,"upper left");
                ips200_show_int(100,140, Upper_left_inflection_X,3);
                ips200_show_int(100,155, Upper_left_inflection_Y,3);
          //      ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y,  RGB565_RED);
          //
          //  }
          //  ips200_show_string(100,170,"upper Lflag");
          //  ips200_show_int(200, 170,Upper_left_inflection_Flag,3);
            //---------------���Ϲյ�--------------------
          //  if(Upper_right_inflection_Flag==1){
//                ips200_show_string(100,170,"upper right");
//                ips200_show_int(100, 185,Upper_right_inflection_X,3);
//                ips200_show_int(100, 200, Upper_right_inflection_Y,3);
          //      ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y,  RGB565_RED);
          //
          //  }
          //  ips200_show_string(100,190,"upper Rflag");
          //  ips200_show_int(200, 190,Upper_right_inflection_Flag,3);

            //********************����*****************************
            if(roundabout_Flag==1){
          //      ips200_show_string(130,215,"roundabout");
          //      ips200_show_int(130, 230,roundabout_X,3);
          //      ips200_show_int(130, 245, roundabout_Y,3);
                ips200_draw_point(roundabout_X, roundabout_Y,  RGB565_RED);

            }
//            ips200_show_string(0,125,"roundabout");//��
//            ips200_show_int(0, 140, roundabout_X,3);
//            ips200_show_int(0, 155, roundabout_Y,3);
//            ips200_show_string(0,170,"roundabout");//��
//            ips200_show_int(0, 185, roundabout_X,3);
//            ips200_show_int(0, 200, roundabout_Y,3);
            //********************��������*****************************
            if(Exit_loop_Flag==1){
                ips200_show_string(130,255,"exit");
                ips200_show_int(130, 270,Exit_loop_X,3);
                ips200_show_int(130, 285, Exit_loop_Y,3);
                ips200_draw_point(Exit_loop_X, Exit_loop_Y,  RGB565_RED);
            }
            ips200_show_string (80,290, "Exit_loop_Flag");
                  ips200_show_int(200, 290,Exit_loop_Flag,3);
            //********************��ֱ��*****************************
//            if( Right_straight_flag==1){
//                ips200_show_string (0, 215, "right straight1");
//                ips200_show_float(0, 260,k1,3,3);
//                ips200_show_float(70, 260,k2,3,3);
//                ips200_show_float(140, 260,k3,3,3);
//            }
//            else{
//                ips200_show_string (0, 215, "right straight0");
//            }
//            if( Left_straight_flag==1){
//                            ips200_show_string (0, 180, "left straight1");
//
//                        }
//                        else{
//                            ips200_show_string (0, 180, "left straight0");
//                        }
                  ips200_show_string (0, 120, "right straight1");
                  ips200_show_int(130, 120,Right_straight_flag,3);
//                  ips200_show_float(0, 140,k1,3,3);
//                  ips200_show_float(0, 160,k2,3,3);
//                  ips200_show_float(0, 180,k3,3,3);
//                  ips200_show_int(0, 140,k1,3);
//                  ips200_show_int(0, 160,k2,3);
//                  ips200_show_int(0, 180,k3,3);

            //********************Բ��*****************************
//           ips200_show_string (0,230, "annulus_R");
//            ips200_show_int (80,230, annulus_R_memory,3);
            ips200_show_string (0,215, "annulus_L");
            ips200_show_int (80,215, annulus_L_memory,3);


            //********************ʮ��*****************************
            ips200_show_string (100,215, "Crossroad");
            ips200_show_int (170,215, Crossroad_memory,3);

            //********************������ʾ*****************************
            ips200_show_float(0,250,Lost_point_L_scan_line,3,3);
            ips200_show_float(0, 270,Lost_point_R_scan_line,3,3);
            ips200_show_string (80,250, "Lost_L_Flag");
            ips200_show_int(200, 250,Lost_left_Flag,3);
            ips200_show_string (80,270, "Lost_R_Flag");
            ips200_show_int(200, 270,Lost_right_Flag,3);

            //********************������*****************************
          //  ips200_show_string (0,260, "zebra");
          //  ips200_show_int (50,260, zebra_crossing_flag,3);

          //  //********************�ٶ�*****************************
          //  ips200_show_float(0, 275, speed1,3,3);
          //  ips200_show_float(60, 275,speed2,3,3);

            //********************��������ʾ*****************************
          //  ips200_show_int(0, 275, speed2, 5);
          //  ips200_show_int(60,275, motor2.pid_actual_val, 5);
          //  ips200_show_int(0, 290, speed1, 5);
          //  ips200_show_int(60,290, motor1.pid_actual_val, 5);
            //********************������ʾ*****************************
            ips200_show_string (0,290, "Endline");
            ips200_show_int(60, 290,Endline,3);
          //  ips200_show_float(100, 275,Lost_point_R_scan_line,3,3);
           ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y,  RGB565_BLUE);
           ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y,  RGB565_BLUE);



          // ips200_show_int (15,275, Lost_right_Flag,3);


          //ips200_show_string(100,152,"mid:");
          //ips200_show_int(150,152,middle[110] - 100,5);
          ips200_show_string(100,230,"err:");
          ips200_show_float(150,230,middle[90] - 98,5,2);*/

          //ips200_show_string              (0, 280, "Endline");
           //   ips200_show_float               (120, 280, (float)Endline, 4, 4);
              /*ips200_show_string              (0, 260, "absolute(k1-k2)");
              ips200_show_float               (140, 260, (float)absolute(k11-k22), 4, 4);
              ips200_show_string              (0, 240, "absolute(k2-k3)");
              ips200_show_float               (140, 240, (float)absolute(k22-k33), 4, 4);
              ips200_show_string              (0, 120, "absolute(k3-k1)");
              ips200_show_float               (140, 120, (float)absolute(k33-k11), 4, 4);
              ips200_show_string              (0, 140, "k4");
              ips200_show_float               (140, 140, (float)k44, 4, 4);
              ips200_show_string              (0, 160, "Right_straight");
              ips200_show_float               (140, 160, (float)Right_straight_flag, 4, 4);
              ips200_show_string              (0, 220, "Endline");
              ips200_show_float               (140, 220, (float)Endline, 4, 4);
              ips200_show_string              (0, 180, "Lower_right_flag");
              ips200_show_float               (140, 180, (float)Lower_right_inflection_Flag, 4, 4);
              ips200_show_string              (0, 200, "Lost_right_Flag");
              ips200_show_float               (140, 200, (float)Lost_right_Flag, 4, 4);
              ips200_show_float               (0, 280, (float)k11, 3, 1);
              ips200_show_float               (40, 280, (float)k22, 3, 1);
              ips200_show_float               (80, 280, (float)k33, 3, 1);
              ips200_show_float               (120, 280, (float)k44, 3, 1);
              ips200_show_float               (160, 280, (float)right[40], 3, 1);*/
          ips200_show_string              (0, 240, "annulus_L_memory");
          ips200_show_float               (140, 240, (float)annulus_L_memory, 4, 4);
          ips200_show_string              (0, 120, "Upper_left_Flag");
          ips200_show_float               (140, 120, (float)Upper_left_inflection_Flag, 4, 4);
          ips200_show_string              (0, 140, "Upper_left_X");
          ips200_show_float               (140, 140, (float)Upper_left_inflection_X, 4, 4);
          ips200_show_string              (0, 160, "Upper_left_y");
          ips200_show_float               (140, 160, (float)Upper_left_inflection_Y, 4, 4);
          ips200_show_string              (0, 220, "l[h]-l[h+4]");
          ips200_show_float               (140, 220, (float)left[Upper_left_inflection_Y]-left[Upper_left_inflection_Y+2], 4, 4);
          ips200_show_string              (0, 180, "left[h+4]");
          ips200_show_float               (140, 180, (float)left[Upper_left_inflection_Y+2], 4, 4);
          ips200_show_string              (0, 200, "left[h]");
          ips200_show_float               (140, 200, (float)left[Upper_left_inflection_Y], 4, 4);
          ips200_show_string              (0, 200, "left[h-4]-left[h]");
          ips200_show_float               (140, 200, (float)left[Upper_left_inflection_Y-4]-left[Upper_left_inflection_Y], 4, 4);

          }


//===================================================Ԫ��ʶ��===================================================
void Element_recognition(void)
{

    inflection_point();//�յ����ж�
    left_straight();//��ֱ��
    right_straight();//��ֱ��
    crossroad();//ʮ��
    //annulus_L();//��Բ��
    //annulus_R();//��Բ��
    middle_line();//����
    bridge_fill(imag,left, right, middle, (uint8*)&Endline);//������
    jump_judge(imag, (uint8*)&Endline,(uint8*)left, (uint8*)right, (int)Left_straight_flag, (int)Right_straight_flag);//��Ծ
    zebra_crossing(imag,left,right);//������
    //around_fill( imag,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest,l_index,r_index,monotonicity_l, monotonicity_r);

}

//===================================================ͼ����===================================================

void image_process(void)
{


        if(mt9v03x_finish_flag){
            system_start ();

            Get_image(mt9v03x_image);
            binaryzation();
            image_filter(imag);
            image_draw_rectan(imag);

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
                Finish_Flag=1;
            }

            image_process_time=system_getval ();
            mt9v03x_finish_flag=0;
           }
        border = (float)((middle[image_h-2] * 0.07) + (middle[image_h-12] * 0.10)
                           + (middle[image_h-22] * 0.25) + (middle[image_h-27] * 0.20)
                           + (middle[image_h-32] * 0.12) + (middle[image_h-42] * 0.09)
                           + (middle[image_h-52] * 0.07) + (middle[image_h-62] * 0.06)
                           + (middle[image_h-72] * 0.04));
}
void get_turn_value(float kp,float kp2,float kd,float gkd)
{
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
}
