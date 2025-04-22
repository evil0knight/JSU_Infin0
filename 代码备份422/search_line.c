#include "zf_common_headfile.h"

uint8 reference_point = 0;  //��̬�ο���
uint8 reference_col = 0;    //��̬�ο���
uint8 white_max_point = 0;  //��̬�׵����ֵ
uint8 white_min_point = 0;  //��̬�׵���Сֵ
uint8 reference_contrast_ratio = 20;//�ο��Աȶ�


uint8 remote_distance[SEARCH_IMAGE_W]       ={ 0 }; //�׵�Զ�˾���
uint8 reference_col_line[SEARCH_IMAGE_H]    ={ 0 }; //�ο��л���
uint8 left_edge_line[SEARCH_IMAGE_H]        ={ 0 }; //��߽�
uint8 right_edge_line[SEARCH_IMAGE_H]       ={ 0 }; //�ұ߽�

uint32 if_count = 0;

//------------------------------------------------------------------------------------------
//����˵��      array_value                 ����ָ��
//����˵��      num0                        ��ʼ��Χ
//����˵��      num1                        ��ֹ��Χ
//����˵��      mode1                       1   �������ֵ   0   ������Сֵ
//��������      uint8
//ʹ��ʾ��      find_extreme_value(remote_distance,10,SEARCH_IMAGE_W - 10, 0);
//��ע��Ϣ
//------------------------------------------------------------------------------------------
uint8 find_extreme_value(uint8 *array_value, uint8 num0 ,uint8 num1, uint8 mode1)
{
    uint8 i = 0, temp = 0,temp1 = 0,temp2 = 0, value = 0;
    if(num0 > num1)
    {
        temp1 = num0 - num1;
        temp2 = num1;
        array_value += num0;
        value = *array_value;
        if(mode1)
        {
            for(i = 0; i <= temp1; i ++)
            {
                temp = *(array_value - i);
                if(temp > value) {temp2 = num0 -i; value = temp; }
            }
        }
        else
        {
            for(i = 0;i <= temp1 ; i ++)
            {
                temp = *(array_value - i);
                if(temp < value) { temp2 = num0 - i;value = temp; }
            }
        }
    }
    else
    {
        temp1 = num1 - num0;
        temp2 = num0;
        array_value += num0;
        value = *array_value;
        if(mode1)
        {
            for(i = 0;i <= temp1; i ++)
            {
                temp = *(array_value + i);
                if(temp > value) {temp2 = i + num0; value = temp; }
            }
        }
        else {
            for(i = 0; i <= temp1; i ++)
            {
                temp = *(array_value + i);
                if(temp < value) {temp2 = i + num0; value = temp;}
            }
        }
    }
    return temp2;
}

//------------------------------------------------------------------------------------------
//�������      ��ȡ�ο���λ
//����˵��      image       ͼ������ָ��
//��������      void
//ʹ��ʾ��      get_reference_point(mt9v03x_image[0]);
//��ע��Ϣ
//------------------------------------------------------------------------------------------
void get_reference_point(const uint8 *image)
{
    uint8 *p=(uint8 *)&image[(SEARCH_IMAGE_H-REFRENCEROW)*SEARCH_IMAGE_W];//ͼ������ָ���ͳ������
    uint16 temp  = 0;                                        //����ͳ�Ƶ�������
    uint32 temp1 = 0;                                        //��������ͳ�Ƶ�������ĺ�
    temp = REFRENCEROW * SEARCH_IMAGE_W;                     //�����ͳ�Ƶ�������
    for(int i=0;i<temp;i++){
        temp1 += *(p + i);                                   //ͳ�����
    }
    reference_point = (uint8)(temp1 / temp);                 //����ƽ��ֵ����Ϊ����ͼ��Ĳο���
    white_max_point = (uint8)func_limit_ab((int32)reference_point * WHITEMAXMUL / 10,BLACKPOINT,255);   //���ݲο������׵����ֵ
    white_min_point = (uint8)func_limit_ab((int32)reference_point * WHITEMINMUL / 10,BLACKPOINT,255);   //���ݲο������׵���Сֵ
}

//--------------------------------------------------------------------------------------------
//�������          ����ͼ��ο���
//����˵��          image           ͼ������ָ��
//��������          void
//ʹ��ʵ��          search_reference_col(mt9v03_image[0]);
//��ע��Ϣ
//--------------------------------------------------------------------------------------------
void search_reference_col(const uint8 *image)
{
    int col,row;
    int16 temp1 = 0,temp2 = 0,temp3 = 0;

    for(col = 0;col<SEARCH_IMAGE_W;col ++)
    {
        remote_distance[col]  =SEARCH_IMAGE_H - 1;
    }
    for(col = 0;col<SEARCH_IMAGE_W;col +=CONTRASTOFFSET)
    {
        for(row = SEARCH_IMAGE_H-1;row>STOPROW;row -= CONTRASTOFFSET)
        {
            temp1 = *(image + row *SEARCH_IMAGE_W +col);            //��ȡ��ǰ��Ҷ�ֵ

            temp2 = *(image + (row - STOPROW)*SEARCH_IMAGE_W + col);//��ȡ�Աȵ�Ҷ�ֵ

            if(temp2 > white_max_point)                               //�ж϶Աȵ��Ƿ�Ϊ�׵� ��Ϊ�׵���ֱ������
            {
                continue;
            }
            if(temp1 < white_min_point)
            {
                remote_distance[col] = (uint8)row;
                break;
            }

            temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);             // ����Աȶ�

            if(temp3 > reference_contrast_ratio || row == STOPROW)
            {
                remote_distance[col] = (uint8)row;
                break;
            }
        }
    }
    reference_col = find_extreme_value(remote_distance, 10,SEARCH_IMAGE_W - 10, 0) + CONTRASTOFFSET;
    reference_col = (uint8)func_limit_ab(reference_col, 1 ,SEARCH_IMAGE_W - 2);

    for(int i = 0; i < SEARCH_IMAGE_H; i++)
    {
        reference_col_line[i] = reference_col;
    }
}
//----------------------------------------------------------------------------------------------------------------
//�������                  ���������߽�
//����˵��                  image                   ͼ������ָ��
//��������                  void
//ʹ��ʾ��                  search_reference_col(mt9v03x_image[0]);
//��ע��Ϣ
//----------------------------------------------------------------------------------------------------------------
void search_line(const uint8 *image)
{
    uint8 *p = (uint8 *)image[0];                               //ͼ������ָ��
    uint8 row_max = SEARCH_IMAGE_H - 1;                         //�����ֵ
    uint8 row_min = STOPROW;                                    //����Сֵ
    uint8 col_max = SEARCH_IMAGE_W - CONTRASTOFFSET;            //�����ֵ
    uint8 col_min = CONTRASTOFFSET;                             //����Сֵ
    int16 leftstartcol  = reference_col;                        //��������ʼ��
    int16 rightstartcol = reference_col;                        //��������ʼ��
    int16 leftendcol    = 0;                                    //��������ֹ��
    int16 rightendcol   = SEARCH_IMAGE_W -1;                    //��������ֹ��
    uint8 search_time   = 0;                                    //�������ߴ���
    uint8 temp1 = 0 , temp2 = 0;                                //��ʱ���������ڴ洢ͼ������
    int temp3 = 0;                                              //��ʱ���������ڴ洢�Աȶ�
    int leftstop = 0,rightstop = 0,stoppoint = 0;               //������������

    int col, row;

    for(row = row_max;row >= row_min;row--)
    {
        left_edge_line[row]     =col_min - CONTRASTOFFSET;
        right_edge_line[row]    =col_max + CONTRASTOFFSET;
    }

    for(row = row_max; row >= row_min; row --)
    {
        p = (uint8 *)&image[row * SEARCH_IMAGE_W];              //��ȡ�������λ��ָ��
        if(!leftstop)
        {
            search_time = 2;
            do
            {
                if(search_time == 1)
                {
                    leftstartcol    = reference_col;
                    leftendcol      = col_min;
                }
                search_time --;
                for(col = leftstartcol;col >= leftendcol;col --){

                    temp1 = *(p + col);                         //��ȡ��ǰ��Ҷ�ֵ

                    temp2 = *(p + col - CONTRASTOFFSET);        //��ȡ��ǰ�Աȵ�Ҷ�ֵ

                    if(temp1 < white_min_point  && col == leftstartcol && leftstartcol ==reference_col) //�жϲο����Ƿ�Ϊ�ڵ�
                    {
                        leftstop = 1;                           //���������� ���������������
                        for(stoppoint = row;stoppoint >= 0;stoppoint --)
                        {
                            left_edge_line[stoppoint] = col_min;
                        }
                        search_time = 0;
                        break;
                    }

                    if(temp1 < white_min_point)                 //�ж��Ƿ�Ϊ�ڵ㣬��Ϊ�ڵ����ý��жԱ�ֱ�Ӹ�ֵ
                    {
                        left_edge_line[row] = (uint8)col;
                        break;
                    }

                    if(temp2 > white_max_point)
                    {
                        continue;
                    }

                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);        //����Աȶ�

                    if(temp3 > reference_contrast_ratio || col ==col_min)   //�ж϶Աȶ��Ƿ������ֵ����������ֵ������Ϊ���б߽磬�����Ѿ��������������ֵ
                    {
                        left_edge_line[row] = col - CONTRASTOFFSET;         //���浱ǰ�б߽�

                        leftstartcol  = (uint8)func_limit_ab(col + SEARCHRANGE,col,col_max);//ˢ�����߷�Χ
                        leftendcol    = (uint8)func_limit_ab(col - SEARCHRANGE,col_min,col);
                        search_time = 0;
                        break;
                    }
                }
            }while(search_time);
        }
        if(!rightstop)
        {
            search_time = 2;
            do
            {
                if(search_time == 1){
                    rightstartcol   = reference_col;
                    rightendcol     = col_max;
                }
                search_time --;
                for(col = rightstartcol;col <= rightendcol; col++)
                {
                    temp1 = *(p + col);                 //��ȡ��ǰ��Ҷ�ֵ

                    temp2 = *(p + col +CONTRASTOFFSET); //��ȡ�Աȵ�Ҷ�ֵ

                    if(temp1 < white_min_point && col == reference_col &&rightstartcol ==reference_col)
                    {
                        rightstop = 1;
                        for(stoppoint = row;stoppoint >= 0;stoppoint --)
                        {
                            right_edge_line[stoppoint] = col_max;
                        }
                        search_time = 0;
                        break;
                    }

                    if(temp1 < white_min_point)                                 //�жϵ�ǰ���Ƿ�Ϊ�ڵ�
                    {
                        right_edge_line[row] = (uint8)col;
                        break;
                    }
                    if(temp2 > white_max_point)
                    {
                        continue;
                    }
                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);        //����Աȶ�

                    if(temp3 > reference_contrast_ratio || col ==col_min)   //�ж϶Աȶ��Ƿ������ֵ����������ֵ������Ϊ���б߽磬�����Ѿ��������������ֵ
                    {
                        right_edge_line[row] = col - CONTRASTOFFSET;         //���浱ǰ�б߽�

                        rightstartcol  = (uint8)func_limit_ab(col + SEARCHRANGE,col,col_max);//ˢ�����߷�Χ
                        rightendcol    = (uint8)func_limit_ab(col - SEARCHRANGE,col_min,col);
                        search_time = 0;
                        break;
                    }
                }
            }while(search_time);

        }
    }
}


