#include "zf_common_headfile.h"

uint8 reference_point = 0;  //动态参考点
uint8 reference_col = 0;    //动态参考列
uint8 white_max_point = 0;  //动态白点最大值
uint8 white_min_point = 0;  //动态白点最小值
uint8 reference_contrast_ratio = 20;//参考对比度


uint8 remote_distance[SEARCH_IMAGE_W]       ={ 0 }; //白点远端距离
uint8 reference_col_line[SEARCH_IMAGE_H]    ={ 0 }; //参考列绘制
uint8 left_edge_line[SEARCH_IMAGE_H]        ={ 0 }; //左边界
uint8 right_edge_line[SEARCH_IMAGE_H]       ={ 0 }; //右边界

uint32 if_count = 0;

//------------------------------------------------------------------------------------------
//参数说明      array_value                 数组指针
//参数说明      num0                        起始范围
//参数说明      num1                        终止范围
//参数说明      mode1                       1   返回最大值   0   返回最小值
//返回类型      uint8
//使用示例      find_extreme_value(remote_distance,10,SEARCH_IMAGE_W - 10, 0);
//备注信息
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
//函数简介      获取参考点位
//参数说明      image       图像数组指针
//返回类型      void
//使用示例      get_reference_point(mt9v03x_image[0]);
//备注信息
//------------------------------------------------------------------------------------------
void get_reference_point(const uint8 *image)
{
    uint8 *p=(uint8 *)&image[(SEARCH_IMAGE_H-REFRENCEROW)*SEARCH_IMAGE_W];//图像数组指向待统计区域
    uint16 temp  = 0;                                        //保存统计点总数量
    uint32 temp1 = 0;                                        //保存所有统计点加起来的和
    temp = REFRENCEROW * SEARCH_IMAGE_W;                     //计算待统计点总数量
    for(int i=0;i<temp;i++){
        temp1 += *(p + i);                                   //统计求和
    }
    reference_point = (uint8)(temp1 / temp);                 //计算平均值，作为本幅图像的参考点
    white_max_point = (uint8)func_limit_ab((int32)reference_point * WHITEMAXMUL / 10,BLACKPOINT,255);   //根据参考点计算白点最大值
    white_min_point = (uint8)func_limit_ab((int32)reference_point * WHITEMINMUL / 10,BLACKPOINT,255);   //根据参考点计算白点最小值
}

//--------------------------------------------------------------------------------------------
//函数简介          搜索图像参考列
//参数说明          image           图像数组指针
//返回类型          void
//使用实例          search_reference_col(mt9v03_image[0]);
//备注信息
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
            temp1 = *(image + row *SEARCH_IMAGE_W +col);            //获取当前点灰度值

            temp2 = *(image + (row - STOPROW)*SEARCH_IMAGE_W + col);//获取对比点灰度值

            if(temp2 > white_max_point)                               //判断对比点是否为白点 若为白点则直接跳过
            {
                continue;
            }
            if(temp1 < white_min_point)
            {
                remote_distance[col] = (uint8)row;
                break;
            }

            temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);             // 计算对比度

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
//函数简介                  搜索赛道边界
//参数说明                  image                   图像数组指针
//返回类型                  void
//使用示例                  search_reference_col(mt9v03x_image[0]);
//备注信息
//----------------------------------------------------------------------------------------------------------------
void search_line(const uint8 *image)
{
    uint8 *p = (uint8 *)image[0];                               //图像数组指针
    uint8 row_max = SEARCH_IMAGE_H - 1;                         //行最大值
    uint8 row_min = STOPROW;                                    //行最小值
    uint8 col_max = SEARCH_IMAGE_W - CONTRASTOFFSET;            //列最大值
    uint8 col_min = CONTRASTOFFSET;                             //列最小值
    int16 leftstartcol  = reference_col;                        //搜线左起始列
    int16 rightstartcol = reference_col;                        //搜线右起始列
    int16 leftendcol    = 0;                                    //搜线左终止列
    int16 rightendcol   = SEARCH_IMAGE_W -1;                    //搜线右终止列
    uint8 search_time   = 0;                                    //单点搜线次数
    uint8 temp1 = 0 , temp2 = 0;                                //临时变量，用于存储图像数据
    int temp3 = 0;                                              //临时变量，用于存储对比度
    int leftstop = 0,rightstop = 0,stoppoint = 0;               //搜线自锁变量

    int col, row;

    for(row = row_max;row >= row_min;row--)
    {
        left_edge_line[row]     =col_min - CONTRASTOFFSET;
        right_edge_line[row]    =col_max + CONTRASTOFFSET;
    }

    for(row = row_max; row >= row_min; row --)
    {
        p = (uint8 *)&image[row * SEARCH_IMAGE_W];              //获取本行起点位置指针
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

                    temp1 = *(p + col);                         //获取当前点灰度值

                    temp2 = *(p + col - CONTRASTOFFSET);        //获取当前对比点灰度值

                    if(temp1 < white_min_point  && col == leftstartcol && leftstartcol ==reference_col) //判断参考列是否为黑点
                    {
                        leftstop = 1;                           //线搜索自锁 不进行左边线搜索
                        for(stoppoint = row;stoppoint >= 0;stoppoint --)
                        {
                            left_edge_line[stoppoint] = col_min;
                        }
                        search_time = 0;
                        break;
                    }

                    if(temp1 < white_min_point)                 //判断是否为黑点，若为黑点则不用进行对比直接赋值
                    {
                        left_edge_line[row] = (uint8)col;
                        break;
                    }

                    if(temp2 > white_max_point)
                    {
                        continue;
                    }

                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);        //计算对比度

                    if(temp3 > reference_contrast_ratio || col ==col_min)   //判断对比度是否大于阈值，若大于阈值，则认为是行边界，或者已经搜索到改行最大值
                    {
                        left_edge_line[row] = col - CONTRASTOFFSET;         //保存当前行边界

                        leftstartcol  = (uint8)func_limit_ab(col + SEARCHRANGE,col,col_max);//刷新搜线范围
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
                    temp1 = *(p + col);                 //获取当前点灰度值

                    temp2 = *(p + col +CONTRASTOFFSET); //获取对比点灰度值

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

                    if(temp1 < white_min_point)                                 //判断当前点是否为黑点
                    {
                        right_edge_line[row] = (uint8)col;
                        break;
                    }
                    if(temp2 > white_max_point)
                    {
                        continue;
                    }
                    temp3 = (temp1 - temp2) * 200 / (temp1 + temp2);        //计算对比度

                    if(temp3 > reference_contrast_ratio || col ==col_min)   //判断对比度是否大于阈值，若大于阈值，则认为是行边界，或者已经搜索到改行最大值
                    {
                        right_edge_line[row] = col - CONTRASTOFFSET;         //保存当前行边界

                        rightstartcol  = (uint8)func_limit_ab(col + SEARCHRANGE,col,col_max);//刷新搜线范围
                        rightendcol    = (uint8)func_limit_ab(col - SEARCHRANGE,col_min,col);
                        search_time = 0;
                        break;
                    }
                }
            }while(search_time);

        }
    }
}


