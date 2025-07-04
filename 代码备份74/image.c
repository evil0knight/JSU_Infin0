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

SingleBridgeState BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;

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
int  my_abs(int value)
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
/*
函数名称：int my_limit(int num,int value)
功能说明：数值限制函数
参数说明：
返回值：a <= c <= b
修改时间：2025年3月14日
备注：
示例：my_auu(c,b,a)
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
//float k11,k22,k33,k44;
void right_straight(void){
    float k1,k2,k3,k4;
    Right_straight_flag=0;   // 初始化右直线标志

    // 计算不同区域的斜率用于判断直线度
    k1=((float)right[90]-(float)right[60])/30;  // 底部区域斜率
    k2=((float)right[80]-(float)right[70])/10;  // 中部区域斜率
    k3=((float)right[20]-(float)right[70])/50;   // 顶部到底部斜率
    k4=((float)right[20]-(float)right[40])/20;   // 顶部到中部斜率
    //k5=((float)right[0]-(float)right[20])/20;   // 顶部到上部斜率
    //k11=k1;k22=k2;k33=k3;k44=k4;
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
    float k1, k2, k3, k4;
    Left_straight_flag = 0;   // 初始化左直线标志

    // 计算不同区域的斜率用于判断直线度
    k1 = ((float)left[90] - (float)left[60]) / 30;   // 底部区域斜率
    k2 = ((float)left[80] - (float)left[70]) / 10;   // 中部区域斜率
    k3 = ((float)left[20] - (float)left[70]) / 50;   // 顶部到底部斜率
    k4 = ((float)left[20] - (float)left[40]) / 20;   // 顶部到中部斜率

    // 判断是否为直线
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
     if(banmaxian_hangshu >= 4 &&  // 至少有4行检测到有效斑马线
       BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&  // 不在单桥上
       jump_position_flag == 0 &&  // 没有跳变位置标记
       //sum_island == 0 && island == 0 &&  // 不在弯道上
       Crossroad_Flag == 0
       
        )  // 没有十字线累积标记
    {
        // 设置停车位置标记（1表示检测到十字线）
        pid1_walk.run_speed=0;
        zebra_crossing_flag=1;
    }
}

/**
* @brief 根据速度计算延时函数
* @param int16  speed       速度
* @param int    kp          速度转化为时间的比例，用于调参
* @return 延时时间
 */
int jump_delay(int16 speed,int kp)
{
    int time=speed*kp;
    //这里加一个限幅的判断
    //if(time>max)return max;
    //else if(time<min)return min;
    //else
    return time;

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
    blake_line = 0; // 记录连续全黑行的数量
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
    if(blake_line >= 25)//想近一点再跳，这里就大一点最多40吧，远一点跳这里就小一点最小就这个吧，看看效果，用delay的舒服就用delay,也可以两个结合着来调
    {
        // 检查特定区域边界是否连续（无丢失），说实话这里没必要，如果弯道会误触发跳跃的话你就检测border
        //if(l_loss_judge(l_border,90 ,110) == 0 &&  // 左边界90-110行连续
         //  l_loss_judge(l_border,70 ,90) == 0 &&   // 左边界70-90行连续
        // r_loss_judge(r_border,90 ,110) == 0   // 右边界90-110行连续
         //  r_loss_judge(r_border,70 ,90) == 0 // 右边界70-90行连续
            //&&monotonicity_l == 1 && monotonicity_r == 1
         //   )
        {
             // 确保不在其他特殊区域（桥、环岛、十字线）
            if(
                BridgeState == SINGLE_BRIDGE_NOT_ACTIVE &&
                //sum_island == 0 && island == 0
                annulus_L_memory==0&&
                zebra_crossing_flag==0&&
                Crossroad_Flag==0&&
            //      && cross_sum == 0
            annulus_L_memory_flag==0
            )
                //  这里把mode改成jump模式（也是flexible模式），然后把腿放为对称，不用舵机控制速度环
            //mode=jump;
            //ServoPID.highleft=3.3;//左右腿高一致
            //ServoPID.highright=3.3;
                //  delay,这里最好根据速度写个延时函数来控制，速度快就早点跳，速度慢就晚点
            //  jump_delay(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data,int kp)，这个函数就在jump_judge函数的上面
            jump_position_flag = 1;// 标记跳变点位置,
        }

    }

}
// /**

// @brief 腿部高度平衡控制函数（左右轮速度分别调整版本）
// @param void
// @return void
// @note 左腿高增加右轮速度，右腿高增加左轮速度，11是单边桥斜边的长，9.8是直角边的长
// */
// static uint8 left_adjust_flag = 0;
// static uint8 right_adjust_flag = 0;
// static float original_left_speed = 0; // 记录左轮原始速度
// static float original_right_speed = 0; // 记录右轮原始速度
// static float original_run_speed = 0; // 记录原始运行速度
// static uint8 speed_initialized = 0;
// static uint8 run_speed_stopped = 0; // 运行速度停止标志
// static float prev_left_height = 0; // 记录上次左腿高度
// static float prev_right_height = 0; // 记录上次右腿高度

// void leg_height_balance_control_wheel_speed(void)
// {
// float height_diff = ServoPID.highleft - ServoPID.highright;
// float adjustment_ratio;

// // 计算高度变化趋势
// float left_height_change = ServoPID.highleft - prev_left_height;
// float right_height_change = ServoPID.highright - prev_right_height;

// // 根据高度差异和变化趋势确定调整比例
// if(height_diff > 0.1f)  // 左腿比右腿高
// {
//     if(left_height_change >= 0)  // 左腿高度正在增加或稳定
//     {
//         adjustment_ratio = 11.0f / 9.8f * 3;  // 更大的调整比例
//     }
//     else  // 左腿高度在减少
//     {
//         adjustment_ratio = 11.0f / 9.8f;      // 正常调整比例
//     }
// }
// else if(height_diff < -0.1f)  // 右腿比左腿高
// {
//     if(right_height_change >= 0)  // 右腿高度正在增加或稳定
//     {
//         adjustment_ratio = 11.0f / 9.8f * 3;  // 更大的调整比例
//     }
//     else  // 右腿高度在减少
//     {
//         adjustment_ratio = 11.0f / 9.8f;      // 正常调整比例
//     }
// }
// else
// {
//     adjustment_ratio = 1;  // 高度差异小时不调整
// }

// // 首次运行时记录原始速度
// if(speed_initialized == 0)
// {
//     original_left_speed = pid2_flexible.sudu_left.kp;   // 记录左轮原始速度
//     original_right_speed = pid2_flexible.sudu_right.kp; // 记录右轮原始速度
//     original_run_speed = pid2_flexible.run_speed;       // 记录原始运行速度 (200)
//     speed_initialized = 1;
// }

// // 检查是否需要停止运行速度
// // 条件：两腿不一样高 且 有一个腿在增加
// if((my_abs((int)(height_diff * 10)) > 1) &&  // 两腿不一样高（高度差 > 0.1）
//    ((left_height_change > 0.05f) || (right_height_change > 0.05f)))  // 有一个腿在增加
// {
//     if(run_speed_stopped == 0)
//     {
//         pid2_flexible.run_speed = 0;  // 停止运行速度
//         run_speed_stopped = 1;        // 标记已停止
//     }
// }
// // 新增：当两腿一样高或者有一个腿在减少时，恢复运行速度
// else if((my_abs((int)(height_diff * 10)) <= 1) ||  // 两腿一样高（高度差 <= 0.1）
//         ((left_height_change < -0.05f) || (right_height_change < -0.05f)))  // 有一个腿在减少
// {
//     if(run_speed_stopped == 1)
//     {
//         pid2_flexible.run_speed = original_run_speed;  // 恢复原始运行速度
//         run_speed_stopped = 0;                         // 重置停止标志
//     }
// }

// // 如果左腿比右腿高，增加右轮速度
// if(height_diff > 0.1f && right_adjust_flag == 0)
// {
//     pid2_flexible.sudu_right.kp *= adjustment_ratio;  // 右轮速度乘以11/9.8
//     right_adjust_flag = 1;
    
//     // 限幅保护
//     if(pid2_flexible.sudu_right.kp > 10.0f)  // 根据实际情况调整上限
//     {
//         pid2_flexible.sudu_right.kp = 10.0f;
//     }
// }
// // 如果右腿比左腿高，增加左轮速度
// else if(height_diff < -0.1f && left_adjust_flag == 0)
// {
//     pid2_flexible.sudu_left.kp *= adjustment_ratio;   // 左轮速度乘以11/9.8
//     left_adjust_flag = 1;
    
//     // 限幅保护
//     if(pid2_flexible.sudu_left.kp > 10.0f)  // 根据实际情况调整上限
//     {
//         pid2_flexible.sudu_left.kp = 10.0f;
//     }
// }
// // 高度恢复时，速度也恢复
// else if(my_abs((int)(height_diff * 10)) <= 1)
// {
//     // 恢复左轮速度
//     if(left_adjust_flag == 1)
//     {
//         pid2_flexible.sudu_left.kp = original_left_speed;
//     }
    
//     // 恢复右轮速度
//     if(right_adjust_flag == 1)
//     {
//         pid2_flexible.sudu_right.kp = original_right_speed;
//     }
    
//     // 重置标志位
//     left_adjust_flag = 0;
//     right_adjust_flag = 0;
// }

// // 更新历史高度值
// prev_left_height = ServoPID.highleft;
// prev_right_height = ServoPID.highright;
// }
// /**
//  * @brief 基于腿部高度的轮子障碍控制
//  * @param void
//  * @return void
//  * @note 结合腿部高度变化检测轮子状态
//  */
// void wheel_obstacle_height_control(void)
// {
//     static float prev_left_height = 0;
//     static float prev_right_height = 0;
//     static uint8 left_climbing_flag = 0;
//     static uint8 right_climbing_flag = 0;
    
//     // 计算高度变化率
//     float left_height_change = ServoPID.highleft - prev_left_height;
//     float right_height_change = ServoPID.highright - prev_right_height;
    
//     // 获取当前轮速
//     float current_left_speed = motor_value.receive_left_speed_data;
//     float current_right_speed = -motor_value.receive_right_speed_data;
    
//     // 检测左轮爬坡状态
//     if(left_height_change > 0.05f && current_left_speed < (pid2_flexible.run_speed * 0.7f))
//     {
//         if(left_climbing_flag == 0)
//         {
//             left_climbing_flag = 1;
//             // 增加左轮功率
//             pid2_flexible.sudu_left.kp *= 1.8f; // 增加80%
//             // 限制最大值
//             if(pid2_flexible.sudu_left.kp > 20.0f)
//                 pid2_flexible.sudu_left.kp = 20.0f;
//         }
//     }
//     // 左轮已经爬上去了
//     else if(left_height_change < -0.02f && left_climbing_flag == 1)
//     {
//         left_climbing_flag = 0;
//         // 恢复并减速
//         pid2_flexible.sudu_left.kp = 0.18f;
//         pid2_flexible.run_speed = 50; // 大幅减速
//     }
    
//     // 右轮逻辑相同
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
    
//     // 更新历史高度
//     prev_left_height = ServoPID.highleft;
//     prev_right_height = ServoPID.highright;
// }

    
/**
* @brief 单桥状态检测与填充函数
* @param uint8(*image)[image_w]     二值化图像
* @param uint8 *l_border            左边界数组
* @param uint8 *r_border            右边界数组
* @param uint8 *center_line         中心线数组（输出）
* @param uint8* hightest            最高点指针
* @return 无返回值，通过BridgeState全局变量标记桥梁状态
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
int loss_1 = 0;       // 边界宽度变化计数器
int loss_2 = 0;       // 边界位置突变计数器
int bridge_number = 0; // 桥梁特征计数器
int bridge_out_flag=0;
int bridge_in_flag=0;
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line, uint8* hightest)
{
    int white_line1 = 0;  // 第75行白色像素计数
    int white_line2 = 0;  // 第70行白色像素计数
    int long_start_l = 0; // 左边界突变起始行
    int long_end_l = 0;   // 左边界突变结束行
    int long_start_r = 0; // 右边界突变起始行
    int long_end_r = 0;   // 右边界突变结束行
     loss_1 = 0;       // 边界宽度变化计数器
     loss_2 = 0;       // 边界位置突变计数器
     bridge_number = 0; // 桥梁特征计数器
    // 扫描图像中间区域，分析边界变化
    for (int i = 30; i < image_h-10; i++)//for (int i = 30; i < image_h-10; i++)
    {
        // 判断当前行边界宽度是否明显小于下一行（桥梁入口特征）
        if((right[i] - left[i]) *10*3/5> (right[i+3] - left[i+3])*10)
            loss_1++;
         // 判断左边界或右边界是否发生大幅突变（超过10像素）
        if(my_abs(left[i] - left[i+3]) >= 10)
            loss_2++;
        if(my_abs(right[i] - right[i+3]) >= 10)
            loss_2++;
    }
    //当桥梁状态为"未激活"时，检测是否进入桥梁
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        if(ServoPID.highleft>=3.4){ServoPID.highleft-=0.5;ServoPID.highright-=0.5;}
        gpio_set_level(BUZZER_PIN, GPIO_LOW);
        // 最高点位置合理（<=30）时进行检测
        if(*hightest <= 30)
        {
            // 统计第75行和第70行的白色像素数量
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
            // 触发桥梁激活条件：
            // 1. 第70行白色像素明显少于第75行
            // 2. 边界宽度变化计数≥2
            // 3. 边界位置突变计数≥4
            // 4. 不在其他特殊区域（弯道、跳变点、十字线）
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
    // 当桥梁状态为"激活"时，处理桥梁区域并生成中心线
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
        if(ServoPID.highleft<5.3){ServoPID.highleft+=0.7;ServoPID.highright+=0.7;}
        //leg_height_balance_control_wheel_speed();
        // 从第30行开始扫描左边界（图像中间偏上区域）
        for (int i = 30; i < image_h-1; i++)
        {
             // 处理第30行（桥梁入口可能位置
            if(i == 30)
            {
                // 检查左边界是否在标准位置附近（94±7像素）
                if(my_abs(left[i]-94)<=7)
                {
                    // 从第31行开始寻找左边界突变点
                    for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        // 如果到达图像底部（a=image_h-5），认为桥梁结束
                        if(a == image_h-5)
                        {
                            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE; // 重置桥梁状态为未激活
                            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
                            mode=walk;
                            break;
                        }
                        // 检测左边界突变（相邻3行变化超过10像素）且满足图像特征
                        if(my_abs(left[a] - left[a-3]) >= 10 &&
                            image[a+2][left[a-3]] == 255 &&// 突变点上方2行对应位置为白色
                             image[a-3][left[a-3]-5] == 0)// 突变点下方3行左侧5像素为黑色
                        {
                            long_start_l = i;    // 记录突变起始行（第30行）
                            long_end_l = a;      // 记录突变结束行（当前行a）
                            bridge_number++;     // 桥梁特征计数+1
                            break;
                        }

                    }
                }
            }
            // 处理30行之后的区域（i>30）
            if(i>30 && my_abs(left[i] - left[i+3]) >= 10 && long_start_l == 0 && long_end_l == 0)
            {
                // 检查图像特征是否符合桥梁边界条件
                if(image[i][left[i+3]] == 255 && image[i+6][left[i+3]-5] == 0)
                {
                    long_start_l = i+3;// 记录突变起始行（当前行+3）
                    // 在i+5行到图像底部之间寻找突变结束点
                    for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        if(a == image_h-5)
                        {
                            long_end_l = a;// 到达底部时记录结束行
                            bridge_number++;
                            break;
                        }
                        // 检测边界突变并满足图像特征
                        if(my_abs(left[a] - left[a-3]) >= 10 &&
                                image[a+2][left[a-3]] == 255 && image[a-3][left[a-3]-5] == 0)
                        {
                            long_end_l = a;// 记录突变结束行
                            bridge_number++;
                            break;
                        }

                    }
                }
            }
            // 在突变区域内生成中心线（使用左边界作为临时中心线）
            if(i <= long_end_l && i >= long_start_l && long_start_l != 0 && long_end_l != 0)
            {
                middle[i] = left[i];// 将左边界值赋给中心线数组
                if(i+1 == long_end_l)// 到达突变区域末尾时重置标记
                {
                    long_end_l = 0;
                    long_start_l = 0;
                }
            }
        }
        // 从第30行开始扫描右边界
        for (int i = 30; i < image_h-1; i++)
        {
            if(i == 30)
            {
                // 检查右边界是否在标准位置附近（94±7像素)
                if(my_abs(r_border[i]-94)<=7)
                {
                    // 从第31行开始寻找右边界突变点
                    for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        if(a == image_h-5)
                        {
                            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;// 桥梁结束
                            break;
                        }
                         // 检测右边界突变并满足图像特征（与左边界对称）
                        if(my_abs(right[a] - right[a-3]) >= 10 &&
                                image[a+2][right[a-3]] == 255 &&
                                 image[a-3][right[a-3]+5] == 0)// 右侧5像素为黑色
                        {
                            long_start_r = i;// 记录右边界突变起始行
                            long_end_r = a;  // 记录右边界突变结束行
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
                    long_start_r = i+3; // 记录突变起始行
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
            // 在右边界突变区域内生成中心线
            if(i <= long_end_r && i >= long_start_r && long_start_r != 0 && long_end_r != 0)
            {
                middle[i] = right[i];// 将右边界值赋给中心线数组
                // 到达突变区域末尾时重置标记
                if(i+1==long_end_r)
                {
                    long_end_r = 0;
                    long_start_r = 0;
                }
            }
        }
        // 当没有边界宽度变化、没有桥梁特征、没有边界位置突变时，认为桥梁结束
        if(loss_1 == 0 && bridge_number == 0 && loss_2 == 0)
        {
            
             bridge_out_flag=1;
        }
    }
}/**
// * @brief 弯道检测与填充函数
// * @param uint8(*image)[image_w]     灰度图像数组
// * @param uint8 *l_border            左边缘数组
// * @param uint8 *r_border            右边缘数组
// * @param uint16 total_num_l         左边缘点总数
// * @param uint16 total_num_r         右边缘点总数
// * @param uint16 *dir_l              左边缘方向数组
// * @param uint16 *dir_r              右边缘方向数组
// * @param uint16(*points_l)[2]       左边缘点坐标数组
// * @param uint16(*points_r)[2]       右边缘点坐标数组
// * @param uint8* hightest            最高有效行指针
// * @param uint16 *l_index            左边缘索引数组
// * @param uint16 *r_index            右边缘索引数组
// * @param int monotonicity_l         左边缘单调性
// * @param int monotonicity_r         右边缘单调性
// * @return 无返回值，结果存储在全局变量中
// *     -<em>false</em> fail
// *     -<em>true</em> succeed
//  */
// void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
//                  uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
//                  uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
// {
//    uint16 i;
//    int black_line_sum = 0; // 黑色线条计数
//    int black_sum_1 = 0;// 灰度值状态标记
//    int black_sum_2 = 0;
//    uint8 break_num_l = 0;// 左右边缘突变点行号
//    uint8 break_num_r = 0;
//    uint8 end_num_l = 0;// 左右边缘恢复点行号
//    uint8 end_num_r = 0;
//    uint8 start, end;  // 拟合区域起止行号
//    int ap = 1;  // 弯道有效性标记
//    float slope_l_rate = 0, intercept_l = 0; // 直线拟合参数
//   // 检测左弯道（左边缘非单调，右边缘单调）
//    if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
//    {
//          // 检测左边缘断线位置
//         broken_line_judge(1,*hightest,110,l_border);
//         // 检查右边缘是否向内侧弯曲（左弯特征）
//         for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//         {
//             if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
//             {
//                 ap = 0;// 标记为无效弯道
//                 break;
//             }
//         }
//          // 统计图像中的黑色线条（可能为赛道线）
//         for (i = broken_line_y+5; i > *hightest; i--)
//         {
//             for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
//             {
//                 // 检测黑色线条起始
//                 if (image[i][x] == 0 && image[i][x-1] == 255)
//                 {
//                     black_sum_1 = 1;
//                 }
//                 // 检测黑色线条结束
//                 if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
//                 {
//                     black_sum_2 = 1;
//                 }
//                  // 重置状态标记
//                 if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
//                 {
//                     black_sum_1 = 0;
//                     black_sum_2 = 0;
//                     break;
//                 }
//             }
//             // 累计有效黑色线条数
//             if(black_sum_2 == 1)
//             {
//                 black_sum_1 = 0;
//                 black_sum_2 = 0;
//                 black_line_sum++;
//             }
//             if(i==15)// 仅检查前15行
//                 break;
//         }
//         // 综合判断是否为左弯道
//         if(black_line_sum>=5 /*确认特征：识别弯道*/
//                 && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
//                 /*特殊条件：边缘丢失判断*/
//                 && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
//             /*附加条件：十字符和桥接不冲突*/
//         {
//                 sum_island = 1;// 标记为疑似弯道
//                 island = 1;// 标记为左弯道
//                 black_line_sum = 0;
//         }
//     }
//     // 检测右弯道（右边缘非单调，左边缘单调）
//     if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
//     {
//         // 检测右边缘断线位置
//         broken_line_judge(1,*hightest,110,r_border);
//         // 检查左边缘是否向内侧弯曲（右弯特征）
//         for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//         {
//             if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || l_border[w] != border_min))
//             {
//                 ap = 0;// 标记为无效弯道
//                 break;
//             }
//         }
//         // 统计图像中的黑色线条（逻辑与左弯相同）
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
//             if(i==15)//???п????????
//                 break;
//         }
//         // 综合判断是否为右弯道（逻辑与左弯相同）
//         if(black_line_sum>=5 /*确认特征：识别弯道*/
//                 && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
//                /*特殊条件：边缘丢失判断*/
//                 && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
//             /*附加条件：十字符和桥接不冲突*/
//         {
//             sum_island = 1;  // 标记为疑似弯道
//             island = 2;      // 标记为右弯道
//             black_line_sum = 0;
//         }
//     }
//     // 处理左弯道（island=1）
//     if(island == 1)
//     {
//          // 确认左弯道（sum_island=1）
        
//         if(sum_island == 1)
//         {
//             // 重新检测左边缘断线位置
//             broken_line_judge(1,*hightest,110,l_border);
//             // 检查右边缘是否持续向内侧弯曲
//             for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//             {
//                 if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
//                 {
//                     ap = 0;// 弯道无效，退出状态
//                     break;
//                 }
//             }
//              // 若弯道无效，重置状态
//             if(ap == 0)
//             {
//                 island = 0;
//                 sum_island = 0;
//             }
//             // 若弯道有效且断线位置合适，修复左边缘
//             if(broken_line_y >= 20 && ap == 1)
//             {
//                 // 最小二乘法拟合左边缘
//                 start = broken_line_y+5;
//                 end = broken_line_y+10;
//                 calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
//                 // 用拟合直线修复左边缘
//                 for (i = 1; i < broken_line_y+1; i++)
//                 {
//                     l_border[i] = slope_l_rate * (i)+intercept_l;
//                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//                 }
//                 // 检查是否需要进一步处理
//                 if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
//                 {
//                     // 通过边缘点确认弯道状态
//                     for (i = 106; i > 15; i--)
//                     {
                        
//                         if(points_l[l_index[i]][0]>points_l[l_index[i-5]][0] && points_l[l_index[i]][0]>points_l[l_index[i+5]][0]
//                             && points_l[l_index[i-5]][0] != border_min)
//                         {
//                             sum_island = 2;// 升级状态为确认弯道
//                         }
//                     }
//                 }
//             }
//         }
//         // 确认左弯道（sum_island=2），进一步处理
//         if(sum_island == 2)
//         {
//             int dp = 0;
//             int temph = 0;
//             int vp = 0;
//             uint16 h = 0;
//             // 检查图像底部像素（可能为赛道特征）
//             if (!image[image_h - 5][5] && !image[image_h - 3][3])
//             {
//                 dp = 1;
//             }
//             // 检测边缘突变位置
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
//             // 确认边缘突变点
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
//              // 寻找边缘最低点
//             for (i = 25; i < image_h - 15; i++)
//             {
//                 if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
//                    && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
//                    && l_border[i-5] != border_min && l_border[i+5] != border_min)
//                 {
//                     end_num_l = (uint8)i;
//                 }
//                 // 状态升级为准备拟合
//                 if(vp && end_num_l >= 80)
//                     sum_island = 3;
//             }
//                 // 拟合右边缘（假设弯道结束后边缘恢复）
//                 slope_l_rate = (float)(118-end_num_l) / ((border_max-r_border[118]+border_min)-l_border[end_num_l]);//б??k=y/x
//                 intercept_l = 118 - slope_l_rate*(border_max-r_border[118]+border_min);//???b=y-kx
//                 for (i = end_num_l; i < image_h - 1; i++)
//                 {
//                     l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//                 }

//         }
//         //初步确认左弯道
//         if(sum_island == 3)
//         {
//             uint16 h = 0;
//             int temph_l = 0;
//             // 1. 寻找左边缘突变点（弯道起始位置）
//             for (h = image_h - 15; h > 5; h--)// 从图像底部向上扫描
//             {
//                 if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
//                 {
//                     temph_l = h;  // 记录突变行号
//                     break;  // 找到后立即退出循环
//                 }
//             }
//              // 2. 在突变点附近寻找最佳转折点
//             if (temph_l)// 如果找到了突变点
//             {
//                 for (int i = total_num_l - 10; i > [h + 1]; i--)
//                 {
//                     // 寻找y坐标局部最大值（弯道顶点特征）
//                     if (points_l[i][1] >= points_l[i + 3][1]  // 当前点y坐标≥后3个点
//                         && points_l[i][1] > points_l[i + 5][1]  // 当前点y坐标>后5个点
//                         && points_l[i][1] >= points_l[i - 3][1]  // 当前点y坐标≥前3个点
//                         && points_l[i][1] >= points_l[i - 5][1]  // 当前点y坐标≥前5个点
//                         // 位置合理性约束
//                         && points_l[i][0] > points_l[i - 5][0]  // 当前点x坐标>前5个点
//                         && points_l[i][0] <= points_l[i + 5][0])  // 当前点x坐标≤后5个点
//                     {
//                         break_num_l = (uint8)points_l[i][1];  // 记录转折点y坐标
//                         end_num_l  = (uint8)points_l[i][0];  // 记录转折点x坐标
//                         break;  // 找到后立即退出循环
//                     }
//                 }
//             }
//             //使用更大范围的比较
// //            for (i = 40; i < total_num_l-30; i++)
// //            {
// //                if (points_l[i][1]>points_l[i+9][1]&&points_l[i][1]>points_l[i-9][1]
// //                      &&points_l[i][1]>points_l[i+15][1]&&points_l[i][1]>points_l[i-15][1])
// //                  {
// //                     break_num_l = (uint8)points_l[i][1];//????y????
// //                     end_num_l  = (uint8)points_l[i][0];//????x????
// //                  }
// //            }

//             // 4. 检查图像底部边缘特征（确认是否进入弯道）
//             if((*hightest >= 30)&&  // 最高点位置合理（确保不是噪声）
//               (!image[image_h - 1][3] && !image[image_h - 3][3] // 图像左下角为黑色（赛道边界）
//                 && !image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))// 图像右下角为黑色
//             {
//                 sum_island = 4;//满足条件则进入状态4
//             }

//             // 5. 更新右边界（基于转折点计算直线方程）
//             if(break_num_l && end_num_l)  // 如果找到了转折点
//             {
//                 // 计算斜率k = Δy/Δx（以(186,118)为参考点）
//                 slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);
//                 intercept_l = 118 - slope_l_rate*186;// 计算截距b = y - kx
//                  // 根据直线方程更新右边界x = (y - b)/k
//                 for (i = 1; i < image_h - 1; i++)
//                 {
//                     r_border[i] = ((i)-intercept_l)/slope_l_rate;// 计算每个y对应的x坐标
//                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max); // 限制在有效范围内
//                 }
//             }
//         }
//         // 当前处于状态4：验证左弯道特征
//         if(sum_island == 4)
//         {
//              int g=0;  // 方向序列匹配标志
    
//              // 1. 寻找特定方向序列（4→4→6→6→6，表示向左转）
//             for (i = 1; i < total_num_l; i++)
//             {
//                 if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
//                 {
//                     g=1;
//                     break;
//                 }
//             }
//             // 2. 检测右边界突变（曲率变化点）
//             for (i = image_h - 20; i > *hightest; i--)//?????б?
//             {
//                 if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>30)
//                 {
//                     if(i>=30 && i<=105 && g)
//                         sum_island = 5;
//                 }
//             }
//         }
//         // 当前处于状态5：计算弯道曲率
//         if(sum_island == 5)
//         {
//             // 1. 寻找右边界突变点（曲率最大处）
//             for (uint16 w = image_h - 15; w > *hightest; w--)//?????б?
//             {
//                 if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
//                 {
//                     break_num_l = (uint8)w;//????y????
//                     break;
//                 }
//             }
//             // 2. 再次确认方向序列特征（同状态4）
//             for (i = 1; i < total_num_l; i++)
//             {
//                 if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
//                 {
//                     end_num_l = (uint8)points_l[i][1];//????y????
//                     break;
//                 }
//             }
//             // 3. 检查右边界是否存在（未丢失）
//             if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
//                 sum_island = 6;
//             // 4. 更新右边界（基于新的曲率计算直线方程）
//             slope_l_rate = (float)(break_num_l-end_num_l) / (r_border[break_num_l]-l_border[end_num_l]);//б??k=y/x
//             intercept_l = 0 - slope_l_rate*0;//计算新截距b=y-kx
//              // 根据新直线方程更新右边界（仅更新到突变点下方）
//             for (i = 1; i < break_num_l-3; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//         }
//         // 弯道过渡状态
//         if(sum_island == 6)
//         {
//             int dp = 1;// 控制是否执行下面的循环（固定为1，表示执行）
//             int temph = 0;
//             int vp = 0;
//             uint16 h = 0;
//              // 1. 寻找左边缘突变点（弯道结束特征）
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
//             // 2. 在突变点附近寻找特征点（y坐标局部最大值）
//             if (temph)
//             {
//                 for (int j = [h]; j > 0; j--)
//                 {
//                     if (points_l[j][1] >= points_l[j + 3][1]  // 当前点y坐标≥后3个点
//                         && points_l[j][1] > points_l[j + 5][1]  // 当前点y坐标>后5个点
//                         && points_l[j][1] >= points_l[j - 3][1]  // 当前点y坐标≥前3个点
//                         && points_l[j][1] >= points_l[j - 5][1])  // 当前点y坐标≥前5个点
//                     {
//                         vp = h;
//                         break;
//                     }
//                 }
//             }
//              // 3. 状态转换条件（特征点存在且右边界特定区域丢失）
//             if(vp && r_loss_judge(r_border, 70 ,90) == 0// 右边界在70-90行丢失
//                     && r_loss_judge(r_border, 50 ,70) == 0)// 右边界在50-70行丢失
//                 sum_island = 7;
//             // 4. 重置右边界（恢复默认直线
//             slope_l_rate = (float)(118-0) / (186-0);// 计算默认斜率k=y/x
//             intercept_l = 0 - slope_l_rate*0;//计算默认截距b=y-kx
//             // 根据默认直线方程更新右边界
//             for (i = 1; i < image_h - 1; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//         }
//         // 弯道结束判断状态
//         if(sum_island == 7)
//         {
//             uint16 h = 0;
//             int temph = 0;
//             // 1. 寻找左边缘突变点（弯道真正结束）
//             for (h = image_h - 25; h > 5; h--)
//             {
//                 if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
//                 {
//                     temph = h;  // 记录突变行号
//                     break_num_l = (uint8)h;  // 记录转折点y坐标
//                     end_num_l  = (uint8)l_border[h];  // 记录转折点x坐标
//                     break;  // 找到后立即退出循环
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
//             // 3. 检测图像底部边缘特征（确认是否回到直道）
//             f (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]  // 右下角为黑色
//                 && !image[image_h - 7][image_w - 3]  // 右下角上方为黑色
//                 && !image[image_h - 5][3] && !image[image_h - 3][3]  // 左下角为黑色
//                 && !image[image_h - 7][3])  // 左下角上方为黑色
//             {
//                 if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0 // 右边界特定区域丢失
//                    && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0) // 左边界特定区域丢失
//                 {
//                     sum_island = 0;
//                     island = 0;// 清除左弯道标记
//                 }
//             }
//             // 4. 更新左边界（基于新的斜率，处理双车道边界）
//             slope_l_rate = (float)(118-end_num_l) / (2-l_border[end_num_l]);//计算新斜率k=y/x
//             intercept_l = 118 - slope_l_rate*2;//计算新截距b=y-kx
//             // 根据新直线方程更新左边界（仅更新转折点下方区域）
//             for (i = end_num_l-10; i < image_h - 1; i++)
//             {
//                 l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//             }
//         }
//     }
//      // 当前识别为右弯道（island=2）
//     if(island == 2)
//     {
//         // 处于状态1：疑似右弯道
//         if(sum_island == 1)
//         {
//             broken_line_judge(1,*hightest,110,r_border); // 检测右边界断线
//             // 验证左边界连续性（防止误判为弯道）
//             for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
//             {
//                 if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || r_border[w] != border_min))
//                 {
//                          ap = 0;// 标记为非连续边界
//                          break;
//                 }
//             }
//             // 边界不连续，取消弯道标记
//             if(ap == 0)
//              {
//                  island = 0;
//                  sum_island = 0;
//              }
//              // 断线位置合理且边界连续时更新右边界
//             if(broken_line_y >= 20 && ap == 1)
//             {
//                 start = broken_line_y+5;  // 拟合起始行
//                 end = broken_line_y+10;    // 拟合结束行
//                 calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
//                 // 根据直线方程更新右边界
//                 for (i = 1; i < broken_line_y+1; i++)
//                 {
//                     r_border[i] = slope_l_rate * (i)+intercept_l;
//                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//                 }
//                  // 检测右边界突变或丢失，进入下一状态
//                 if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
//                 {
//                     for (i = 106; i > 15; i--)
//                     {
//                         // 寻找右边界局部最小值（弯道特征）
//                         if(points_r[r_index[i]][0]<points_r[r_index[i-5]][0] && points_r[r_index[i]][0]<points_r[r_index[i+5]][0]
//                             && points_r[r_index[i-5]][0] != border_max)
//                         {
//                             sum_island = 2;
//                         }
//                     }
//                 }
//             }
//         }
//      // 状态2：确认右弯道特征
//     if(sum_island == 2)
//     {
//         int dp = 0;
//         int temph = 0;
//         int vp = 0;
//         uint16 h = 0;
//          // 检测图像底部特征点（右侧边缘）
//         if (!image[image_h - 5][183] && !image[image_h - 3][185])
//         {
//             dp = 1;// 标记为有效检测区域
//         }
//          // 寻找右边缘突变点（弯道起始）
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
//         // 寻找右边缘特征点（局部最小值）
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
//          // 寻找右边界局部最小值（曲率最大点）
//         for (i = 25; i < image_h - 15; i++)
//         {
//             if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
//                && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
//                && r_border[i-5] != border_max && r_border[i+5] != border_max)
//             {
//                 end_num_r = (uint8)i;
//             }
//             // 特征点存在且位置合理时进入状态3
//             if(vp && end_num_r >= 80)
//                 sum_island = 3;
//         }
//              // 更新右边界（基于曲率计算）
//             slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б??k=y/x
//             intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
//             for (i = end_num_r-10; i < image_h - 1; i++)
//             {
//                 r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//                 r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//             }
//     }
//     // 状态3：计算右弯道参数
//     if(sum_island == 3)
//     {
//         uint16 h = 0;
//         int temph_l = 0;
//         // 寻找右边缘突变点（细化位置）
//         for (h = image_h - 15; h > 5; h--)
//         {
//             if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
//             {
//                 temph_l = h-5;
//                 break;
//             }
//         }
//          // 寻找右边缘特征点（局部最小值）
//         if (temph_l)
//         {
//             for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
//             {

//                 if (points_r[i][1] >= points_r[i + 3][1]// y坐标局部最大
//                     &&points_r[i][1] > points_r[i + 5][1]
//                     &&points_r[i][1] >= points_r[i - 3][1]
//                     &&points_r[i][1] >= points_r[i - 5][1]
//                     &&points_r[i][0] < points_r[i - 5][0]
//                     &&points_r[i][0] >= points_r[i + 5][0])
//                 {
//                     break_num_r = (uint8)points_r[i][1];// 记录特征点y坐标
//                     end_num_r  = (uint8)points_r[i][0];// 记录特征点x坐标
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
//         // 最高点位置合理时进入状态4
//         if(*hightest >= 30)
//         {
//             sum_island = 4;
//         }
//         // 更新左边界（基于右弯道特征点）
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
//     // 状态4：检测左边界突变
//     if(sum_island == 4)
//     {
//         for (i = image_h - 15; i > *hightest; i--)//?????б?
//         {
//             if (l_border[i] > l_border[i - 3] && (l_border[i] - l_border[i-3])>20)
//             {
//                 if(i>=30 && i<=105)
//                   sum_island = 5;
//             }
//         }
//     }
//      // 状态5：计算左边界参数
//     if(sum_island == 5)
//     {
//         for (uint16 w = image_h - 15; w > *hightest; w--)//?????б?
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
//         // 右边界存在时进入状态6
//         if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
//             sum_island = 6;
//         // 更新左边界（基于突变点
//         end_num_r = l_border[break_num_r];
//         slope_l_rate = (float)(break_num_r-0) / (end_num_r-188);//б??k=y/x
//         intercept_l = 0 - slope_l_rate*188;//b=y-kx
//         for (i = 1; i < break_num_r-3; i++)
//         {
//             l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//         }
//     }
//     // 状态6：右弯道过渡
//     if(sum_island == 6)
//     {
//         int dp = 1;
//         int temph = 0;
//         int vp = 0;
//         uint16 h = 0;
//         if (dp)
//         {
//             // 寻找右边缘突变点（弯道结束特征）
//             for (h = image_h - 15; h > 5; h--)
//             {
//                 if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
//                 {
//                     temph = h;
//                     break;
//                 }
//             }
//         }
//          // 寻找右边缘特征点（局部最小值）
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
//         // 边界丢失时进入状态7（弯道结束判断）
//         if(vp && l_loss_judge(r_border, 90 ,110) == 0 && l_loss_judge(r_border, 70 ,90) == 0
//                 && l_loss_judge(r_border, 50 ,70) == 0)
//             sum_island = 7;
//          // 重置左边界（恢复默认直线）
//         slope_l_rate = (float)(118-0) / (0-188);//k=y/x
//         intercept_l = 0 - slope_l_rate*188;//b=y-kx
//         for (i = 1; i < image_h - 1; i++)
//         {
//             l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
//         }
//     }
//      // 状态7：右弯道结束判断
//     if(sum_island == 7)
//     {
//         uint16 h = 0;
//         int temph = 0;
//         // 寻找右边缘突变点（弯道结束
//         for (h = image_h - 15; h > 5; h--)
//         {
//             if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
//             {
//                 temph = h;
//                 break;
//             }
//         }
//         // 寻找右边缘特征点（局部最小值）
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
//         // 底部边缘特征满足时回到初始状态
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
//          // 更新右边界（结束弯道处理）
//         slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б??k=y/x
//         intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
//         for (i = end_num_l-10; i < image_h - 1; i++)
//         {
//             r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
//             r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
//         }
//     }
//    }
// }

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
    //annulus_L();//左圆环
    //annulus_R();//右圆环
    middle_line();//中线
    bridge_fill(imag,left, right, middle, (uint8*)&Endline);//单边桥
    jump_judge(imag, (uint8*)&Endline,(uint8*)left, (uint8*)right, (int)Left_straight_flag, (int)Right_straight_flag);//跳跃
    zebra_crossing(imag,left,right);//斑马线
    //around_fill( imag,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest,l_index,r_index,monotonicity_l, monotonicity_r);

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
    //这个可以根据速度扩大缩小，630完成任务
    turn_value=(94-border)*kp
            +(94-border)*border_abs*kp2
            +((94-border)-border_last)*kd
            +imu660ra_gyro_z*gkd;
    //turn_value=turn_value*(你现在的目标速度/你调好的速度)
    //这里是因为v=w*r，车的速度变大之后，你目标角速度也要跟着变大，这样你半径才能不变，车子走的路才会一样
    //所以应该先调好一个速度，之后再把turn_value=turn_value*(你现在的目标速度/你调好的速度)用上
    border_last=(94-border);
}
