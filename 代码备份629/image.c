#include  "zf_common_headfile.h"

#define White 255   //定义白色像素的值为255
#define Black 0     //定义黑色像素的值为0

#define Left 1     // 定义左侧标识为1
#define Right 2    // 定义右侧标识为2
int x,y;
float parameterB,parameterA;   //y=parameterB*x+parameterA
float k1,k2,k3,k4,k5,k6;
int left[120]={2};             //声明大小为120的左侧边界数组，初始值为2
int right[120]={185};          //声明大小为120的右侧边界数组，初始值为185
int middle[120]={93};          //声明大小为120的中间线数组，初始值为93
int Endline=1;                 //声明终点线标志，初始为1
int WhiteNum=0;                //白色像素计数器，初始为0
int X1,Y1;//左下补线点（圆环）
uint8 right_lost_num=0;        //统计右边界丢失的次数
uint8 left_lost_num=0;         //统计左边界丢失的次数
uint8 imag[120][188];          //声明120行188列的图像数组      用于存储图像数据
uint8 threshold_value=175;     //二值化阈值，初始175
uint32 image_process_time=0;   //图像处理时间
float turn_value=0;       // 转角值，由border在中断中计算得出
float border = 96;        // 边界阈值,图像偏差
float border_last = 96;        // 边界阈值,上一次图像偏差
int annulus_L_memory_flag=0;

uint8 Right_straight_flag=0; //右直线
uint8 Left_straight_flag=0; //左直线

uint8 annulus_L_Flag=0;       //左圆环
uint8 annulus_R_Flag=0;       //右圆环
uint8 annulus_L_memory=0;     //左圆环计步        用于存储左环形区域的记忆状态
uint8 annulus_R_memory=0;     //右圆环计步        用于存储右环形区域的记忆状态
uint8 zebra_crossing_flag=0;//斑马线


//圆环凸起点
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;     //圆环检测标志（0=未检测到，1=检测到）

//出环识别点
uint8 Exit_loop_X=0;
uint8 Exit_loop_Y=0;
uint8 Exit_loop_Flag=0;     //出环检测标志（0=未出环，1=已出环）

//十字
uint8 Crossroad_Flag=0;      //十字(0=无十字，1=检测到十字）
uint8 Crossroad_memory=0;     //十字计步
uint8 Finish_Flag=0; //处理完成标识位

//丢线
uint8 Lost_left_Flag = 0;          //左侧丢线标志（0=未丢线，1=左侧丢线）
uint8 Lost_right_Flag = 0;         //右侧丢线标志（0=未丢线，1=右侧丢线）
uint8 Lost_point_L_scan_line = 0;  //左侧丢线的扫描行（Y坐标）
uint8 Lost_point_R_scan_line = 0;  //右侧丢线的扫描行（Y坐标）

//左下拐点
uint8 Lower_left_inflection_X =0;
uint8 Lower_left_inflection_Y =0;
uint8 Lower_left_inflection_Flag=0;

//右下拐点
uint8 Lower_right_inflection_X =0;
uint8 Lower_right_inflection_Y =0;
uint8 Lower_right_inflection_Flag=0;

//左上拐点
uint8 Upper_left_inflection_X =0;
uint8 Upper_left_inflection_Y =0;
uint8 Upper_left_inflection_Flag=0;

//右上拐点
uint8 Upper_right_inflection_X =0;
uint8 Upper_right_inflection_Y =0;
uint8 Upper_right_inflection_Flag=0;



//计算一个浮点数的绝对值
float absolute(float z)
{
    z = z< 0 ? (-z) : z;
uint8 Upper_left_inflection_Flag=0;
    return z;
}
/*
函数名称：int my_abs(int value)
功能说明：计算绝对值
参数说明：
返回值：绝对值
修改时间：2025年1月5日
备注：
示例：my_abs(x)
*/
int my_abs(int value)
{
    if(value>=0) return value;
    else return -value;
}
//将x限制在a和b之间
int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

//x限制在-y和y之间
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}

//图像采集函数
//从 mt9v03x_image 中获取图像数据，并存储到 original_image 中
uint8 original_image[image_h][image_w];
void Get_image(uint8(*mt9v03x_image)[image_w])
{
    //use_num控制是否压缩图像(1不压缩，2隔行采样压缩)
#define use_num     1   //控制图像数据的采样间隔
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}

//大津法(Otsu)二值化    使用 Otsu 算法计算图像的最佳阈值
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{

#define GrayScale 256               //定义灰度级为 256
    uint16 Image_Width  = col;      //定义图像的宽度
    uint16 Image_Height = row;      //定义图像的高度
    int X; uint16 Y;
    uint8* data = image;            //将输入的图像数据指针赋值给 data
    int HistGram[GrayScale] = {0};  //存储每个灰度级的像素数量。

    uint32 Amount = 0;                  // 总像素数
    uint32 PixelBack = 0;               // 背景像素数
    uint32 PixelIntegralBack = 0;       // 背景像素灰度总和
    uint32 PixelIntegral = 0;           // 所有像素灰度总和
    int32 PixelIntegralFore = 0;        // 前景像素灰度总和
    int32 PixelFore = 0;                // 前景像素数
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差
    uint8 MinValue=0, MaxValue=0;       // 图像中的最小和最大灰度值
    uint8 Threshold = 0;                // 最终阈值

    //统计每个灰度值的像素数量
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }



//确定图像中实际存在的最小和最大灰度值，避免在后续计算中考虑不存在的灰度级，提高效率。
    // 获取最小灰度值
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;
    // 获取最大灰度值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--) ;
    // 处理特殊情况：图像中只有一种或两种灰度
    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色  返回该颜色的灰度值。
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;          // 图像中只有二个颜色   返回较小的灰度值。
    }



    // 遍历所有灰度级，计算总像素数量
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }
    // 计算所有像素的灰度总和
    PixelIntegral = 0;     //初始化像素积分值为 0
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }

    //寻找最佳阈值
    SigmaB = -1 ;                        //初始化类间方差为 -1
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差对应的阈值
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }

   return Threshold;//返回最佳阈值
}


//二值化处理   将图像转换为黑白二值图像
void binaryzation(void)
{
  uint8 i,j;
//调用 OtsuThreshold 函数计算最佳阈值，并加 5。
threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+5;
  // 遍历图像的行
  for(i = 0;i<image_h;i++)
  {
       //遍历图像的列
      for(j = 0;j<image_w;j++)
      {
          //如果当前像素值大于阈值，将其赋值为白色像素
          if(original_image[i][j]>threshold_value)imag[i][j] = white_pixel;
          //否则将其赋值为黑色像素
          else imag[i][j] = black_pixel;
      }
  }
}

//赛道边界起点检测
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//初始化左起始点的 x 坐标为 0
    start_point_l[1] = 0;//初始化左起始点的 y 坐标为 0

    start_point_r[0] = 0;//初始化右起始点的 x 坐标为 0
    start_point_r[1] = 0;//初始化右起始点的 y 坐标为 0

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//更新左起始点的 y 坐标
        start_point_l[1] = start_row;//更新左起始点的 y 坐标
        //如果当前像素为白色，且前一个像素为黑色，说明找到了左起始点
        if (imag[start_row][i] == 255 && imag[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;//设置左起始点找到的标志位
            break;
        }
    }

    //从图像中间向右遍历，查找右起始点
    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//更新右起始点的 x 坐标
        start_point_r[1] = start_row;//更新右起始点的 y 坐标
        //如果当前像素为白色，且后一个像素为黑色，说明找到了右起始点
        if (imag[start_row][i] == 255 && imag[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;//设置右起始点找到的标志位
            break;
        }
    }

    //如果左右起始点都找到了，返回1
    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");
        return 0;
    }
}

//赛道边界跟踪
#define USE_num image_h*3   //用于指定存储点的数组大小
 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//存储左边界的点
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//存储右边界的点
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边界点的方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边界点的方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
//接受多个参数，包括终止标志、图像数据、左右边界点数量指针、左右起始点坐标和结束线指针
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*Endline)
{

    uint8 i = 0, j = 0;
    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };//存储左边界的搜索区域
    uint8 index_l = 0;                     //存储左边界搜索区域的索引
    uint8 temp_l[8][2] = { {  0 } };      //临时存储左边界的点
    uint8 center_point_l[2] = {  0 };     //存储左边界的中心点
    uint16 l_data_statics;                //统计左边界找到的点的数量
    //定义八个邻域，表示左边界的搜索方向
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };//存储右边界的搜索区域
    uint8 center_point_r[2] = { 0 };        //存储右边界的中心点
    uint8 index_r = 0;                      //存储右边界搜索区域的索引
    uint8 temp_r[8][2] = { {  0 } };        //临时存储右边界的点
    uint16 r_data_statics;                  //统计右边界找到的点的数量
    //定义八个邻域    表示右边界的搜索方向
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//初始化左边界的中心点的 x 坐标为左起始点的 x 坐标
    center_point_l[1] = l_start_y;//初始化左边界的中心点的 y 坐标为左起始点的 y 坐标
    center_point_r[0] = r_start_x;//初始化右边界的中心点的 x 坐标为右起始点的 x 坐标
    center_point_r[1] = r_start_y;//初始化右边界的中心点的 y 坐标为右起始点的 y 坐标

    //开启邻域循环
    while (break_flag--)
    {

        //遍历左边界的搜索方向
        for (i = 0; i < 8; i++)//传递8F坐标
        {   //计算左边界搜索区域的 x 坐标
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];
            //计算左边界搜索区域的 y 坐标
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//将左边界的中心点的 x 坐标存储到 points_l 中
        points_l[l_data_statics][1] = center_point_l[1];//将左边界的中心点的 y 坐标存储到 points_l 中
        l_data_statics++;//索引加一

        //遍历右边界的搜索方向
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            //计算右边界搜索区域的 x 坐标
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];
            //计算右边界搜索区域的 y 坐标
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        //遍历左边界的搜索区域
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用    初始化临时存储左边界点的数组
            temp_l[i][1] = 0;//先清零，后使用    初始化临时存储左边界点的数组
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {   //如果当前搜索区域为黑色，且下一个搜索区域为白色，说明找到了左边界的点
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                //将左边界的点的 x 坐标存储到 temp_l 中
                temp_l[index_l][0] = search_filds_l[(i)][0];
                //将左边界的点的 y 坐标存储到 temp_l 中
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录左边界点的方向
            }

            //如果找到了左边界的点
            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//更新左边界的中心点的 x 坐标
                center_point_l[1] = temp_l[0][1];//更新左边界的中心点的 y 坐标
                //遍历左边界的点
                for (j = 0; j < index_l; j++)
                {
                    //如果当前点的 y 坐标小于中心点的 y 坐标，更新中心点的坐标
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        //如果连续三个点的坐标相同，跳出循环
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        //如果左右边界的点的距离小于 2，跳出循环，并更新结束线的位置
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*Endline);
            break;
        }
        //如果右边界的点的 y 坐标小于左边界的点的 y 坐标，左边界等待右边界。
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
           // printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        //如果左边界的点的方向为 7，且右边界的点的 y 坐标大于左边界的点的 y 坐标，左边界回退一步。
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//右边界点数量加 1

        index_r = 0;//先清零，后使用
        //遍历右边界的搜索区域
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            //如果当前搜索区域为黑色，且下一个搜索区域为白色，说明找到了右边界的点。
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                //将右边界的点的 x 坐标存储到 temp_r 中
                temp_r[index_r][0] = search_filds_r[(i)][0];
                //将右边界的点的 y 坐标存储到 temp_r 中
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录右边界点的方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            //如果找到了右边界的点
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//更新右边界的中心点的 x 坐标
                center_point_r[1] = temp_r[0][1];//更新右边界的中心点的 y 坐标
                //遍历右边界的点
                for (j = 0; j < index_r; j++)
                {
                    //如果当前点的 y 坐标小于中心点的 y 坐标，更新中心点的坐标
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        //更新右边界的中心点的坐标
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }
            }
        }
    }

    //取出循环次数
    *l_stastic = l_data_statics;//更新左边界点数量指针的值
    *r_stastic = r_data_statics;

}


//获取左边界的坐标
void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    //初始化
    for (i = 0;i<image_h;i++)
    {
        left[i] = border_min;

    }
    h = image_h - 2;//初始化行号为图像高度减 2
    //遍历左边界的点
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        //如果当前点的 y 坐标等于行号
        if (points_l[j][1] == h)
        {
            //将左边界数组的当前行的元素更新为当前点的 x 坐标加 1
            left[h] = points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}

//判断左边界是否丢失
void lost_left(void){
    uint8 i=0;
    left_lost_num=0;
    Lost_left_Flag=0;
    //从第 110 行到第 10 行遍历图像
    for(i=110;i>10;i--){
        //如果当前行的第 2 列的像素为白色
        if(imag[i][2]==White){
            left_lost_num++;//左边界丢失次数加 1
            Lost_point_L_scan_line=i+4;//记录左边界丢失点的扫描线位置
        }
        //如果左边界丢失次数大于 15
        if(left_lost_num>15){
            Lost_left_Flag=1; //判断左边下方是否丢线
            return;
        }
    }
}

//该函数用于获取右边界的坐标
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //遍历图像的行
    for (i = 0; i < image_h; i++)
    {
        right[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据

    }
    h = image_h - 2;
    //遍历右边界的点
    for (j = 0; j < total_R; j++)
    {
        //如果当前点的 y 坐标等于行号
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;

        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

//判断右边界是否丢失
void lost_right(void){
    uint8 i=0;
    right_lost_num=0;
    Lost_right_Flag=0;
    //从第 110 行到第 10 行遍历图像
    for(i=110;i>10;i--){
        if(imag[i][185]==White){
            right_lost_num++;
            Lost_point_R_scan_line=i+4;
        }
        if(right_lost_num>15){
            Lost_right_Flag=1;  //判断右边下方是否丢线
            return;
        }
    }
}

//寻找中线
void middle_line(void){
    for(y=119;y>Endline;y--){
        //计算中间线的坐标，即左右边界坐标的平均值
        middle[y]=(right[y]+left[y])/2;
    }
}


//定义膨胀和腐蚀的阈值区间
#define threshold_max   255*5
#define threshold_min   255*2
void image_filter(uint8(*imag)[image_w])//形态学滤波，膨胀和腐蚀的思想    用于表示图像数据
{
    uint16 i, j;
    uint32 num = 0;
    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];

            //如果周围像素的总和大于等于阈值的最大值，且当前像素为黑色
            if (num >= threshold_max && imag[i][j] == 0)
            {
                imag[i][j] = 255;//白  可以搞成宏定义，方便更改
            }
            //如果周围像素的总和小于等于阈值的最小值，且当前像素为白色
            if (num <= threshold_min && imag[i][j] == 255)
            {
                imag[i][j] = 0;//黑
            }
        }
    }
}


//用于在图像的边缘绘制矩形
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;            //将图像的第一列的像素设置为黑色
        image[i][1] = 0;            //将图像的第二列的像素设置为黑色
        image[i][image_w - 1] = 0;  //将图像的最后一列的像素设置为黑色
        image[i][image_w - 2] = 0;  //将图像的倒数第二列的像素设置为黑色
    }

    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;        //将图像的第一行的像素设置为黑色
        image[1][i] = 0;        //将图像的第二行的像素设置为黑色
        //image[image_h-1][i] = 0;

    }
}

//==========================================================左右连续判断=================================================
/**
 * @brief 左边缘丢失判断
 * @param uint8 *l_border     左边缘数组的地址
 * @param uint8 start         检查起始行
 * @param uint8 end           检查终止行
 * @see CTest       l_loss_judge(l_border,20 ,0)
 * @return 返回值说明
 * -1 无效
 * 1  丢失
 * 0  正常
 */
int l_loss_judge(uint8 *l_border,uint8 start ,uint8 end)
{
    uint16 i;         // 循环计数器
    uint16 sum = 0;   // 记录符合"边缘丢失"条件的行数
    
    // 限制start和end在有效范围内（2到image_h-2）
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
     // 检查参数有效性：如果起始行大于等于终止行，返回无效
    if(start >= end)
        return -1;
    // 遍历[start, end]范围内的每一行
    for(i = start;i <= end; i++)
    {
        // 如果当前行的左边缘位置小于等于border_min+2
       // 说明左边缘过于靠近图像左侧，可能丢失
       if(l_border[i] <= border_min+2 )
           sum++;
    }
    // 如果符合"边缘丢失"条件的行数超过总检查行数的80%
    // 则认为整个区域的左边缘丢失
    if(sum >= (my_abs(start - end)/5*4))
        return 1;  // 左边缘丢失
    else
        return 0;  // 左边缘正常
}

/**
 * @brief 右边缘丢失判断
 * @param uint8 *r_border     右边缘数组的地址
 * @param uint8 start         检查起始行
 * @param uint8 end           检查终止行
 * @see CTest       r_loss_judge(l_border,20 ,0)
 * @return 返回值说明
 * -1 无效
 * 1  丢失
 * 0  正常
 */
int r_loss_judge(uint8 *r_border,uint8 start ,uint8 end)
{
    uint16 i;         // 循环计数器
    uint16 sum = 0;   // 记录符合"边缘丢失"条件的行数
    
    // 限制start和end在有效范围内（2到image_h-2）
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    // 检查参数有效性：如果起始行大于等于终止行，返回无效
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


//==========================================================左右连续判断=================================================

//==========================================================拐点识别====================================================

//---------------------------------左下拐点--------------------------------
//-----------------------------第二版：用断点判断------------------------
void Lower_left(void){
    // 初始化左下拐点标志和坐标
    Lower_left_inflection_Flag=0;
    Lower_left_inflection_X =0;
    Lower_left_inflection_Y =0;

    // 从图像底部向上扫描寻找拐点
    for(y=image_h-3;y>Endline+10;y--){
        // 确保扫描行有效
        if(y>30){
            // 拐点判断条件：当前行与上方行的差值>5且上方行接近边界(2)
            // 同时当前行与下方行差值<5且当前行位置合理(>10)
            if((left[y]-left[y-4])>5&&left[y-4]==2&&(left[y]-left[y+2])<5&&left[y]>10){

                // 满足条件则标记发现左下拐点并记录坐标
                Lower_left_inflection_Flag=1;
                Lower_left_inflection_X =left[y];
                Lower_left_inflection_Y =y;
                return;
            }
        }
    }
    //----------------------------第一版：用扫线方法判断--------------------------------
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

//-------------------------------------右下拐点---------------------------------
//-----------------------------第二版：用断点判断------------------------
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
    //----------------------------第一版：用扫线方法判断--------------------------------
    //（可对右下拐点识别，但是右转弯时的拐角也会被判定）
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

//-------------------------------------左上拐点------------------------------------
//-----------------------------第二版：用断点判断------------------------
void Upper_left(void){
    uint8 h=image_h-3;
    uint8 i;

    // 初始化左上拐点标志和坐标
    Upper_left_inflection_Flag=0;
    Upper_left_inflection_X =0;
    Upper_left_inflection_Y =0;

    // 当左侧丢失线时执行特殊处理
    if(Lost_left_Flag==1){
        //针对圆环写的找点方式
          if(annulus_L_Flag==1){
              // 从丢失点下方开始扫描
              for(h=Lost_point_L_scan_line+5;h>(Endline+10);h--){//改动
                // 检测特征点：当前行与下方行差值>3且下方行接近边界

                  if((left[h]-left[h+2])>40&&left[h+4]<=10&&left[h]!=2&&((left[h-4]-left[h])<4||(left[h-4]-left[h])>1000)&&left[h]>20){
                   if(left[h]>93) {            //针对圆环状态4补线出现的断层问题进行的尝试优化
                       Upper_left_inflection_Flag=1;
                       Upper_left_inflection_X =left[h];
                       Upper_left_inflection_Y =h;
                       //-----------第一版：用扫线方法判断-----------
                       uint8 Find_Flag=0;
                       for(y=h;y<110;y++){
                           Find_Flag=0;
                           for(x=left[h]+10;x>70;x--){
                                // 寻找边界点：左侧黑右侧白
                                if(imag[y][x-1]==Black&&imag[y][x]==Black&&imag[y][x+1]==White){
                                    Find_Flag=1;
                                    break;
                                 }

                               }
                            // 如果找不到边界点，说明已过拐点
                            if(Find_Flag==0){
                               Upper_left_inflection_Y =y-1;
                               return;
                               }

                           }

                    }
                    else{
                         // 普通场景直接记录拐
                        Upper_left_inflection_Flag=1;
                        Upper_left_inflection_X =left[h];
                        Upper_left_inflection_Y =h;
                        return;
                    }
                }
            }
        }
        // 非圆环场景的处理
        else{
            for(h=Lost_point_L_scan_line+3;h>(Endline+10);h--){
                // 检测特征点：当前行与下方行差值>3且下方行接近边界
                if((left[h]-left[h+4])>3&&left[h+10]==2&&left[h]!=2&&(left[h-3]-left[h])<3){

                    // 标记并记录拐点
                    Upper_left_inflection_Flag=1;
                    Upper_left_inflection_X =left[h];
                    Upper_left_inflection_Y =h;
                    return;

                }
             }
          }

    }
    //----------------------------第一版：用扫线方法判断--------------------------------
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

//-----------------------------------右上拐点-----------------------------------
//-----------------------------第二版：用断点判断------------------------
void Upper_right(void){
    uint8 h=image_h-3;
    Upper_right_inflection_Flag=0;
    Upper_right_inflection_X =0;
    Upper_right_inflection_Y =0;
    if(Lost_right_Flag==1){
        //针对圆环写的找点方式
          if(annulus_R_Flag==1){
              for(h=Lost_point_R_scan_line+5;h>(Endline+10);h--){
                if((right[h+8]-right[h])>3&&right[h+8]==185&&right[h]!=185&&(right[h]-right[h-4])<5&&h<60){
                   if(right[h]>93) {            //针对圆环状态4补线出现的断层问题进行的尝试优化
                       Upper_right_inflection_Flag=1;
                       Upper_right_inflection_X =right[h];
                       Upper_right_inflection_Y =h;
                       //-----------第一版：用扫线方法判断-----------
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
    //----------------------------第一版：用扫线方法判断--------------------------------
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

//--------拐点总判断-----------
void inflection_point(void){

    Upper_left();
    Upper_right();
    Lower_left();
    Lower_right();
}


//==================================================右直线识别===========================================================
float k11,k22,k33,k44;
void right_straight(void){
    float k1,k2,k3,k4,k5;
    Right_straight_flag=0;   // 初始化右直线标志

    // 计算不同区域的斜率用于判断直线度
    k1=((float)right[90]-(float)right[60])/30;  // 底部区域斜率
    k2=((float)right[80]-(float)right[70])/10;  // 中部区域斜率
    k3=((float)right[20]-(float)right[70])/50;   // 顶部到底部斜率
    k4=((float)right[20]-(float)right[40])/20;   // 顶部到中部斜率
    //k5=((float)right[0]-(float)right[20])/20;   // 顶部到上部斜率
    k11=k1;k22=k2;k33=k3;k44=k4;
    // 判断是否为直线
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

//==================================================左直线识别===========================================================
void left_straight(void){
    float k4,k5,k6;
    Left_straight_flag=0;
    k4=((float)left[90]-(float)left[60])/30;
    k5=((float)left[80]-(float)left[70])/10;
    k6=((float)left[0]-(float)left[70])/70;
    if(Endline<5&&absolute(k4-k5)<0.7&&absolute(k5-k6)<0.7&&absolute(k6-k4)<0.7&&k5!=0&&k6!=0&&k4!=0&&Lost_left_Flag==0){
        Left_straight_flag=1;
//        printf(k4,k5,k6);


    }
}
//ips200_show_float(0, 260,k4,3,3);
//         ips200_show_float(70, 260,k5,3,3);
//         ips200_show_float(140, 260,k6,3,3);

//        ips200_show_float(100, 170,absolute(k4-k5),3,3);
//        ips200_show_float(100, 190,absolute(k5-k6),3,3);
//        ips200_show_float(100, 210,absolute(k6-k4),3,3);

//        k7=((float)middle[90]-(float)middle[60])/((float)middle[80]-(float)middle[70]);


//=================================================十字识别=======================================================
void crossroad(void){

    //提前十字处理
    if(annulus_R_Flag==0&&annulus_L_Flag==0&&Lost_left_Flag==1&&Lost_point_L_scan_line>60&&Lost_point_R_scan_line>60&&Upper_right_inflection_Flag==1&&Upper_left_inflection_Flag==1&&Crossroad_Flag==0&&Crossroad_memory==0&&Endline<5){
        Crossroad_Flag=1;
        Crossroad_memory=1;
    }
    if(Crossroad_Flag==1){

        //状态1 看到十字，和左右下面两个拐点
        if(Crossroad_memory==1){

            //方案二：延斜率补线
            // 根据拐点情况进行补线
            if(Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1){
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, Lower_left_inflection_X , Lower_left_inflection_Y );
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y,  Lower_right_inflection_X, Lower_right_inflection_Y);
            }
            else{
                 // 默认补线位置
                Addingline( 1, 77,20, 37,83);
                Addingline( 2, 128,29, 160, 99);
            }

            // 处理单边有拐点的情况
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
            // 没有拐点时切换状态
            if ( Lower_right_inflection_Flag==0&&Lower_left_inflection_Flag==0)
            {
                Crossroad_memory=2;
            }
        }

        //状态2 看到左上拐点和右上拐点，向下拉线
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
            // 单边有拐点的处理
            if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==1){
                Addingline( 2,Upper_right_inflection_X, Upper_right_inflection_Y, 187,118);
            }
            if(Upper_left_inflection_Flag==1&&Upper_right_inflection_Flag==0){
                Addingline( 1, Upper_left_inflection_X, Upper_left_inflection_Y, 2, 118);
            }
//           if(Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==1&&Upper_right_inflection_Flag==1&&Upper_left_inflection_Flag==1){
//               Crossroad_memory=3;
//           }
            // 没有拐点时退出十字状态
            if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
                Crossroad_Flag=0;
                Crossroad_memory=0;
                return;
            }
            return;
       }

       //状态3 在十字中行驶,出环遇拐点补线
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
       //状态4 补线出十字
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
           //退出
//           if(Upper_left_inflection_Flag==0&&Upper_right_inflection_Flag==0){
//               Crossroad_Flag=0;
//               Crossroad_memory=0;
//               return;
//           }

//
  }
}

//=============================================圆环===================================================
//================环岛识别=====================
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
//================出环拐点识别=====================
void Exit_loop_L_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //左圆环，识别右边拐点
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
  //右圆环，识别右边拐点
    for(i=110;i>Endline+10;i--){
        if(left[i+4]<left[i]&&left[i-4]<left[i]&&left[i+3]<left[i]&&left[i-3]<left[i]){
            Exit_loop_Flag=1;
            Exit_loop_X=left[i];
            Exit_loop_Y=i;
            return;
        }
    }
}

//================左圆环识别=====================

void annulus_L(void){

    //识别圆环
    if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1&&Lower_left_inflection_Flag==1&&Lower_right_inflection_Flag==0&&annulus_L_memory==0&&annulus_L_Flag==0){
        annulus_L_Flag=1;
        annulus_L_memory =1;

    }
    else if(Lost_left_Flag==1&&Lost_right_Flag==0&&Right_straight_flag==1&&Lower_left_inflection_Flag==0&&Lower_right_inflection_Flag==0&&annulus_L_memory==0&&annulus_L_Flag==0){
        annulus_L_Flag=1;
        annulus_L_memory =2;
    }


    if(annulus_L_Flag==1){
//         duty_value = -1800;

         //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
        if (annulus_L_memory == 1)
        {

            if(Lower_left_inflection_Flag==1){
                Addingline1( 1, Lower_left_inflection_X, Lower_left_inflection_Y);

            }
             else if(Lower_left_inflection_Flag==0/*&&(imag[90][3]==255&&imag[100][3]==255&&imag[110][3]==255&&imag[120][3]==255)*/){
                 annulus_L_memory = 2;
             }
         }
         //状态2 识别到圆环，环岛，并对左边进行补线
          if (annulus_L_memory == 2 )//2
         {

              roundabout_L();          //环岛

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
         //状态3 到达圆环入口，封住前路，补线入环
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

         //状态4 在圆环中行驶，当看到右下拐点时进入下一状态
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

//         状态6 出环时右下拐点消失，但是车还没完全出环，此时还需要补线处理
         if (annulus_L_memory == 6 )
          {
             if(Lost_left_Flag==1&&Lost_right_Flag==1){
                 Addingline( 2, 3, Endline+3, 184, 117);
             }
             if(Upper_left_inflection_Flag==1&&Lost_left_Flag==1&&Lost_right_Flag==0){
                              annulus_L_memory = 7;
                       }
             }

         //状态7 出环补线
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



//================右圆环识别=====================
void annulus_R(void){

    //识别圆环
    if(Lost_right_Flag==1&&Lost_left_Flag==0&&Left_straight_flag==1&&Lower_right_inflection_Flag==1&&Lower_left_inflection_Flag==0&&annulus_R_memory==0&&annulus_R_Flag==0){
        annulus_R_Flag=1;
        annulus_R_memory =1;

    }

         if(annulus_R_Flag==1){


             //状态 1  识别到圆环，未识别到环岛（可强行补线，或不补线，待测试）
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
             //状态2 识别到圆环，环岛，并对左边进行补线
              if (annulus_R_memory == 2 )
             {
                  roundabout_R();          //环岛
                if(roundabout_Flag==1){
                    Addingline( 2, roundabout_X, roundabout_Y,178 , 118 );

                 }
                else {
                    Addingline( 2, 134, 30,178 , 118 );  //环岛拐点
                }
                if(Upper_right_inflection_Flag==1){   //改变Upper_right_inflection_Y限制间接改变入环位置，待验证
                    annulus_R_memory = 3;
                }
                else return;
             }
         //状态3 到达圆环入口，封住前路，补线入环
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
         //状态4 在圆环中行驶，当看到右下拐点时进入下一状态
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

         //状态5 出环时看到右下拐点,对其补线处理
         if (annulus_R_memory == 5 )
          {
             Exit_loop_R_inflection();
             if(Exit_loop_Flag==1){
                 if(Exit_loop_Y>35){
                     Addingline( 1, 187, Exit_loop_Y-30, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
                 else {
                     Addingline( 1, 187, 50, Exit_loop_X, Exit_loop_Y);//此处补线结束点待测试
                     return;
                 }
             }
             else if(Lost_right_Flag==1&&Lost_left_Flag==1&&Exit_loop_Flag==0){
                 annulus_R_memory = 6;
             }
             else return;

          }

         //状态6 出环时右下拐点消失，但是车还没完全出环，此时还需要补线处理
         if (annulus_R_memory == 6 )
          {

                 Addingline( 1, 187, Endline+3, 2, 117);

             if(Upper_right_inflection_Flag==1){
                 annulus_R_memory = 7;
             }
             else return;
          }
         //状态7 出环补线
         if (annulus_R_memory == 7)
          {
             if(Upper_right_inflection_Flag==1){
                 Addingline( 2, Upper_right_inflection_X+5, Upper_right_inflection_Y, 178, 118);
                 return;
             }
             else{
                 Addingline( 2, 130, 19, 178, 118); // 要改
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



//---------------------------------------线性拟合---------------------------------
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
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
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
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
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
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
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
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
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
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
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
        averageX = (float)(sumX / (sumlines1 + sumlines2));     //x的平均值
        averageY = (float)(sumY / (sumlines1 + sumlines2));     //y的平均值
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
//------------------------------判断直线斜率是否相同------------------------------
int Judgment_symbol(float x, float y)
{
    int a;
    a = 0;
    if (x < 0 && y < 0) a = 1;
    if (x >= 0 && y >= 0) a = 1;
    return a;
}
//-----------------------------------------补线-----------------------------------

//有起点和终点的补线函数
void Addingline( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;

    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    switch(choice)
    {
      case 1://左补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;

            for(y = startY; y < endY; y++)
            {
                left[y] = (uint8)(k * y + b);
            }
            break;
        }

      case 2://右补线
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


void Addingline1( uint8 choice, uint8 startX, uint8 startY)    //看到拐点延斜率向上延长
{

    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
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

      case 2://右补线  待测试
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

void Addingline2( uint8 choice, uint8 startX, uint8 startY)   //找到上拐点延斜率向下拉线
{

    // 直线 x = k*y + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
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

     case 2://右补线  待测试
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
* @brief 斑马线状态检测函数
* @param uint8(*image)[image_w]     二值化图像
* @param uint8 *l_border            左边界数组指针
* @param uint8 *r_border            右边界数组指针
*  @see CTest       cross_stop(image,l_border,r_border);
* @return 返回说明
*     -<em>false</em> 检测失败
*     -<em>true</em> 检测成功
 */
void zebra_crossing(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border)
{
    uint8 start_point  = 85;  // 检测起始行（垂直位置）
    uint8 end_point = 105;    // 检测结束行（垂直位置）
    // 初始化计数器
    int banmaxian_kuandu = 0;  // 斑马线宽度（单个黑色条带）
    int banmaxian_hangshu = 0; // 有效斑马线行数
    int banmaxian_geshu = 0;   // 当前行检测到的黑色条带数量
    zebra_crossing_flag=0;

     // 从下往上逐行扫描（end_point → start_point）
    for (uint16 y = end_point; y >= start_point; y--)
    {
        // 在左右边界之间进行水平扫描
        for (int x = (int)l_border[y]; x <= (int)r_border[y]; x++)
        {
            int baidian_heng=0;  // 存储白色点的水平坐标（用于计算条带宽度）

            // 检测从白色到黑色的跳变点（上升沿）
            // image[y][x]是当前像素，image[y][x-1]是前一个像素
            if (image[y][x] == 0 && image[y][x-1] == 255)
            {
                // 从跳变点开始向右扫描，寻找下一个从黑色到白色的跳变点（下降沿）
                for(int a=x+1;a<x+15;a++)
                {
                    if(image[y][a-1] == 0 && image[y][a] == 255)
                    {
                        baidian_heng = a;  // 记录下降沿的x坐标
                        break;  // 找到后立即退出内层循环
                    }
                }
                // 计算黑色条带的宽度（两个跳变点之间的距离）
                banmaxian_kuandu = baidian_heng - x;
                // 判断是否为有效斑马线（宽度在4-8像素之间）
                if (banmaxian_kuandu >= 4 && banmaxian_kuandu <= 8)
                {
                    banmaxian_geshu++;  // 有效条带计数+1
                    banmaxian_kuandu = 0;  // 重置宽度计数器，准备下一个条带检测
                }
                //斑马线的宽度不在4~8之间则不是有效斑马线
                else
                {
                   // 无效宽度，重置计数器
                    banmaxian_kuandu = 0;
                }
            }
            
        }

        // 判断当前行是否为有效斑马线行（4-9个黑色条带）
        if (banmaxian_geshu >= 4 && banmaxian_geshu <= 9)
        {
            banmaxian_hangshu++;// 有效行数+1
        }
    }
    // 判断是否满足十字线检测条件
     if(banmaxian_hangshu >= 4 /*&& */ // 至少有4行检测到有效斑马线
       /*BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&  // 不在单桥上
       jump_position == 0*/ /*&&*/  // 没有跳变位置标记
       /*sum_island == 0 && island == 0 &&  // 不在弯道上
       cross_sum == 0*/)  // 没有十字线累积标记
    {
        // 设置停车位置标记（1表示检测到十字线）
        pid1_walk.run_speed=0;
        zebra_crossing_flag=1;
    }
}

/**
* @brief 跳变状态检测函数
* @param uint8(*image)[image_w]     二值化图像
* @param uint8* hightest            最高点位置
* @param uint8 *l_border            左边界数组
* @param uint8 *r_border            右边界数组
* @param uint8 monotonicity_l       左边界单调性
* @param uint8 monotonicity_r       右边界单调性
* @return 无返回值，通过jump_position全局变量标记跳变状态
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
uint8 jump_position_flag=0;
int blake_line = 0; // 记录连续全黑行的数量
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r)
{
    jump_position_flag = 0;// 标记跳变点位置
    // 当最高点位置≥55时进行跳变点检测
    if(*hightest>=55)
    {
        // 从最高点位置向下扫描到第50行
        for (int y = *hightest; y >= 50; y--)
        {
            // 在当前行的左右边界之间扫描
            for (int x = l_border[*hightest+1]; x <= r_border[*hightest+1]; x++)
            {
                // 如果遇到白色像素，立即跳出当前行扫描
                if(image[y][x] == 255)
                {
                    break;
                }
                // 如果扫描到右边界仍未遇到白色像素
                if(x == r_border[*hightest+1])
                {
                    blake_line++;// 全黑行计数+1
                }
            }
        }
    }
    // 当连续全黑行数量≥3时，进一步判断
    if(blake_line >= 3)
    {
        // 检查特定区域边界是否连续（无丢失）
        if(l_loss_judge(l_border,90 ,110) == 0 &&  // 左边界90-110行连续
           l_loss_judge(l_border,70 ,90) == 0 &&   // 左边界70-90行连续
           r_loss_judge(r_border,90 ,110) == 0 &&  // 右边界90-110行连续
           r_loss_judge(r_border,70 ,90) == 0) // 右边界70-90行连续
            //&&monotonicity_l == 1 && monotonicity_r == 1)ad
        {
             // 确保不在其他特殊区域（桥、环岛、十字线）
            //if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&/* sum_island == 0 && island == 0*/
            //      && cross_sum == 0)
            jump_position_flag = 1;// 标记跳变点位置
        }
    }

}
//===================================================显  示===================================================
void IPS_show(void){
   int i;
  //********************图像显示*****************************
    ips200_displayimage03x(imag[0], MT9V03X_W, MT9V03X_H);

  //********************边线显示*****************************
        //根据最终循环次数画出边界点
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


/*
  //********************拐点显示*****************************
          //---------------左下拐点--------------------
          //  if(Lower_left_inflection_Flag==1){
                ips200_show_string(0,125,"low left");

                ips200_show_int(0, 140, Lower_left_inflection_X,3);
                ips200_show_int(0, 155, Lower_left_inflection_Y,3);
          //      ips200_draw_point(Lower_left_inflection_X, Lower_left_inflection_Y,  RGB565_BLUE);
          //
          //  }
//            ips200_show_string(100,160,"low Lflag");
//            ips200_show_int(200, 160,Lower_left_inflection_Flag,3);
            //---------------右下拐点-------------------
          //  if(Lower_right_inflection_Flag==1){
//                ips200_show_string(0,170,"low right");
//                ips200_show_int(0, 185, Lower_right_inflection_X,3);
//                ips200_show_int(0, 200, Lower_right_inflection_Y,3);
          //      ips200_draw_point(Lower_right_inflection_X, Lower_right_inflection_Y,  RGB565_BLUE);
          //
          //  }
//            ips200_show_string(100,150,"low Rflag");
//            ips200_show_int(200, 150,Lower_right_inflection_Flag,3);
            //---------------左上拐点-------------------
          //  if(Upper_left_inflection_Flag==1){
                ips200_show_string(100,125,"upper left");
                ips200_show_int(100,140, Upper_left_inflection_X,3);
                ips200_show_int(100,155, Upper_left_inflection_Y,3);
          //      ips200_draw_point(Upper_left_inflection_X, Upper_left_inflection_Y,  RGB565_RED);
          //
          //  }
          //  ips200_show_string(100,170,"upper Lflag");
          //  ips200_show_int(200, 170,Upper_left_inflection_Flag,3);
            //---------------右上拐点--------------------
          //  if(Upper_right_inflection_Flag==1){
//                ips200_show_string(100,170,"upper right");
//                ips200_show_int(100, 185,Upper_right_inflection_X,3);
//                ips200_show_int(100, 200, Upper_right_inflection_Y,3);
          //      ips200_draw_point(Upper_right_inflection_X, Upper_right_inflection_Y,  RGB565_RED);
          //
          //  }
          //  ips200_show_string(100,190,"upper Rflag");
          //  ips200_show_int(200, 190,Upper_right_inflection_Flag,3);

            //********************环岛*****************************
            if(roundabout_Flag==1){
          //      ips200_show_string(130,215,"roundabout");
          //      ips200_show_int(130, 230,roundabout_X,3);
          //      ips200_show_int(130, 245, roundabout_Y,3);
                ips200_draw_point(roundabout_X, roundabout_Y,  RGB565_RED);

            }
//            ips200_show_string(0,125,"roundabout");//上
//            ips200_show_int(0, 140, roundabout_X,3);
//            ips200_show_int(0, 155, roundabout_Y,3);
//            ips200_show_string(0,170,"roundabout");//下
//            ips200_show_int(0, 185, roundabout_X,3);
//            ips200_show_int(0, 200, roundabout_Y,3);
            //********************出环岛口*****************************
            if(Exit_loop_Flag==1){
                ips200_show_string(130,255,"exit");
                ips200_show_int(130, 270,Exit_loop_X,3);
                ips200_show_int(130, 285, Exit_loop_Y,3);
                ips200_draw_point(Exit_loop_X, Exit_loop_Y,  RGB565_RED);
            }
            ips200_show_string (80,290, "Exit_loop_Flag");
                  ips200_show_int(200, 290,Exit_loop_Flag,3);
            //********************右直线*****************************
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

            //********************圆环*****************************
//           ips200_show_string (0,230, "annulus_R");
//            ips200_show_int (80,230, annulus_R_memory,3);
            ips200_show_string (0,215, "annulus_L");
            ips200_show_int (80,215, annulus_L_memory,3);


            //********************十字*****************************
            ips200_show_string (100,215, "Crossroad");
            ips200_show_int (170,215, Crossroad_memory,3);

            //********************左丢线显示*****************************
            ips200_show_float(0,250,Lost_point_L_scan_line,3,3);
            ips200_show_float(0, 270,Lost_point_R_scan_line,3,3);
            ips200_show_string (80,250, "Lost_L_Flag");
            ips200_show_int(200, 250,Lost_left_Flag,3);
            ips200_show_string (80,270, "Lost_R_Flag");
            ips200_show_int(200, 270,Lost_right_Flag,3);

            //********************斑马线*****************************
          //  ips200_show_string (0,260, "zebra");
          //  ips200_show_int (50,260, zebra_crossing_flag,3);

          //  //********************速度*****************************
          //  ips200_show_float(0, 275, speed1,3,3);
          //  ips200_show_float(60, 275,speed2,3,3);

            //********************编码器显示*****************************
          //  ips200_show_int(0, 275, speed2, 5);
          //  ips200_show_int(60,275, motor2.pid_actual_val, 5);
          //  ips200_show_int(0, 290, speed1, 5);
          //  ips200_show_int(60,290, motor1.pid_actual_val, 5);
            //********************调试显示*****************************
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


//===================================================元素识别===================================================
void Element_recognition(void)
{

    inflection_point();//拐点总判断
    left_straight();//左直线
    right_straight();//右直线
    crossroad();//十字
    annulus_L();//左圆环
    annulus_R();//右圆环
    middle_line();//中线
    jump_judge(imag, (uint8*)&Endline,(uint8*)left, (uint8*)right, (int)Left_straight_flag, (int)Right_straight_flag);
    zebra_crossing(imag,left,right);//斑马线
    

}

//===================================================图像处理===================================================

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
    border_abs=(94-border)>0?(94-border):-(94-border);//计算图像偏差的绝对值
    turn_value=(94-border)*kp
            +(94-border)*border_abs*kp2
            +((94-border)-border_last)*kd
            +imu660ra_gyro_z*gkd;
    border_last=(94-border);


}
