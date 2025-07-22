#include  "zf_common_headfile.h"

#define White 255   //定义白色像素的值为255
#define Black 0     //定义黑色像素的值为0

#define Left 1     // 定义左侧标识为1
#define Right 2    // 定义右侧标识为2
#define up 0       // 定义上边界标识为0
#define down 1     // 定义下边界标识为1
#define START_H 110 // 定义起始扫描行
#define END_H 10   // 定义结束扫描行
int x,y;
float parameterB,parameterA;   //y=parameterB*x+parameterA
extern int flexible_leg_high;
float k1,k2,k3,k4,k5,k6;
int left[120]={2};             //声明大小为120的左侧边界数组，初始值为2
int right[120]={185};          //声明大小为120的右侧边界数组，初始值为185
int left_cicle[120]={2};             //声明大小为120的左侧边界数组，初始值为2
int right_cicle[120]={185};          //声明大小为120的右侧边界数组，初始值为185
int middle[120]={93};          //声明大小为120的中间线数组，初始值为93
int Endline=1;                 //声明终点线标志，初始为1
int WhiteNum=0;                //白色像素计数器，初始为0
uint8_t imageOut[2][image_w]; //声明上下边界数组，[0]为上边界，[1]为下边界
uint8 right_lost_num=0;        //统计右边界丢失的次数
uint8 left_lost_num=0;         //统计左边界丢失的次数
uint8 imag[120][188];          //声明120行188列的图像数组      用于存储图像数据
uint8 imag_copy[120][188];
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

// RoundaboutGetArc滤波变量
uint8 roundabout_arc_filter_left = 0;   // 左圆环弧线检测滤波计数器
uint8 roundabout_arc_filter_right = 0;  // 右圆环弧线检测滤波计数器
#define ROUNDABOUT_ARC_FILTER_THRESHOLD 2  // 滤波阈值，连续检测到3次才确认

// roundabout_L/R滤波变量
uint8 roundabout_L_filter = 0;   // 左圆环上拐点检测滤波计数器
uint8 roundabout_R_filter = 0;   // 右圆环上拐点检测滤波计数器
#define ROUNDABOUT_FILTER_THRESHOLD 3  // roundabout滤波阈值，连续检测到2次才确认


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
uint8 Finish_Flag=1; //处理完成标识位

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

int sum_island = 0;       // 岛屿总数
int island = 0;       
int cross_sum = 0;        // 交叉点计数

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

int threshold_adjust=0;
//二值化处理   将图像转换为黑白二值图像
void binaryzation(void)
{
  uint8 i,j;
//调用 OtsuThreshold 函数计算最佳阈值，并加 5。
threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+threshold_adjust;
  // 遍历图像的行
  for(i = 0;i<image_h;i++)
  {
       //遍历图像的列
      for(j = 0;j<image_w;j++)
      {
          //如果当前像素值大于阈值，将其赋值为白色像素
          if(original_image[i][j]>threshold_value)imag_copy[i][j] = white_pixel;
          //否则将其赋值为黑色像素
          else imag_copy[i][j] = black_pixel;
      }
  }
  
  // 针对左下角区域(暗角效应区域)单独进行大津法二值化
  // 区域范围: 宽度 image_w*3/4 到 image_w, 高度 image_h*2/3 到 image_h
  uint8 corner_start_x = image_w * 3 / 4;  // 左下角区域起始x坐标
  uint8 corner_end_x = image_w;            // 左下角区域结束x坐标  
  uint8 corner_start_y = image_h * 2 / 3;  // 左下角区域起始y坐标
  uint8 corner_end_y = image_h;            // 左下角区域结束y坐标
  
  // 创建临时数组存储左下角区域的像素数据
  uint8 corner_width = corner_end_x - corner_start_x;
  uint8 corner_height = corner_end_y - corner_start_y;
  uint8 corner_image[corner_height * corner_width];
  
  // 提取左下角区域的像素数据
  uint16 corner_index = 0;
  for(i = corner_start_y; i < corner_end_y; i++)
  {
      for(j = corner_start_x; j < corner_end_x; j++)
      {
          corner_image[corner_index++] = original_image[i][j];
      }
  }
  
  // 对左下角区域单独计算大津法阈值
  uint8 corner_threshold = OtsuThreshold(corner_image, corner_width, corner_height) + threshold_adjust;
  
  // 用新阈值重新对左下角区域进行二值化
  for(i = corner_start_y; i < corner_end_y; i++)
  {
      for(j = corner_start_x; j < corner_end_x; j++)
      {
          //如果当前像素值大于左下角区域阈值，将其赋值为白色像素
          if(original_image[i][j] > corner_threshold) imag_copy[i][j] = white_pixel;
          //否则将其赋值为黑色像素
          else imag_copy[i][j] = black_pixel;
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

    // 根据单边桥状态调整搜索范围
    if (BridgeState != SINGLE_BRIDGE_ACTIVE)
    {
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
    }
    else
    {
        // 单边桥状态：放宽搜索范围，从更大范围搜索起点
        //从中间往左边，先找起点 - 扩大搜索范围
        for (i = image_w * 2 / 3; i > border_min; i--)
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

        //从图像中间向右遍历，查找右起始点 - 扩大搜索范围
        for (i = image_w / 3; i < border_max; i++)
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
        //如果连续三个点的坐标相同，跳出循环 - 修改：只在图像上部才允许因相同点退出
        if (((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
            && (points_r[r_data_statics][1] < 30 || points_l[l_data_statics-1][1] < 30)) // 只有在图像上部才允许因相同点退出
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        //如果左右边界的点的距离小于 2，跳出循环，并更新结束线的位置 - 修改：增加位置和搜索深度限制
        if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2
            ) 
        {
            //printf("\n左右相遇退出\n");
            *Endline   = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
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
uint16 l_index[120] = {0};  // 每行对应在points_l中的索引
uint16 r_index[120] = {0};  // 每行对应在points_r中的索引

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
        l_index[i] = 0;
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
            l_index[h] = j;                    // 记录点在原始数组中的索引
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}
void get_left_cicle(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //初始化
    for (i = 0;i<image_h;i++)
    {
        left_cicle[i] = border_min;
    }
    h = image_h - 2;//初始化行号为图像高度减 2
    //遍历左边界的点
    for (j = 0; j < total_L; j++)
    {
        if (points_l[j][1] == h)
        {
            left_cicle[h] = points_l[j][0]+1;
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
    for(i=Endline+10;i<110;i++){
        //如果当前行的第 2 列的像素为白色
        if(imag[i][3]==White){
            left_lost_num++;//左边界丢失次数加 1
            Lost_point_L_scan_line=i+4;//记录左边界丢失点的扫描线位置
        }
        //如果左边界丢失次数大于 15
        if(left_lost_num>(100-Endline)/9){
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
        r_index[i] = border_max; // 初始化为最大边界值
    }
    h = image_h - 2;
    //遍历右边界的点
    for (j = 0; j < total_R; j++)
    {
        //如果当前点的 y 坐标等于行号
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;
            r_index[h] = j;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}
void get_right_cicle(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //遍历图像的行
    for (i = 0; i < image_h; i++)
    {
        right_cicle[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //遍历右边界的点
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right_cicle[h] = points_r[j][0] - 1;
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
    for(i=Endline+10;i<110;i++){
        //如果当前行的第 185 列的像素为白色
        if(imag[i][185]==White){
            right_lost_num++;//右边界丢失次数加 1
            Lost_point_R_scan_line=i+4;//记录右边界丢失点的扫描线位置
        }
        //如果右边界丢失次数大于阈值
        if(right_lost_num>(100-Endline)/9){
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
            if((left[y]-left[y-4])>10&&left[y-4]<=4&&(left[y]-left[y+2])<5&&left[y]>10){
                // 满足条件则标记发现左下拐点并记录坐标
                Lower_left_inflection_Flag=1;
                Lower_left_inflection_X =left[y];
                Lower_left_inflection_Y =y;
                return;
            }
        }
    }
}

//-------------------------------------右下拐点---------------------------------
//-----------------------------第二版：用断点判断------------------------
void Lower_right(void){
    // 初始化右下拐点标志和坐标
    Lower_right_inflection_Flag=0;
    Lower_right_inflection_X =0;
    Lower_right_inflection_Y =0;
    // 从图像底部向上扫描寻找拐点
    for(y=image_h-3;y>Endline+10;y--){
        // 确保扫描行有效
        if(y>30){
            // 拐点判断条件：当前行与上方行的差值>5且上方行接近边界(185)
            // 同时当前行与下方行差值<5且当前行位置合理(<170)
            if((right[y-4]-right[y])>10&&right[y-4]>=185&&(right[y+2]-right[y])<5&&right[y]<170){
                // 满足条件则标记发现右下拐点并记录坐标
                Lower_right_inflection_Flag=1;
                Lower_right_inflection_X =right[y];
                Lower_right_inflection_Y =y;
                return;
            }
        }
    }
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
              for(h=Lost_point_L_scan_line+25;h>(Endline+25);h--){//改动
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
float k11,k22,k33,k44,k55,k66,k77,k88;
void right_straight(void){
    float k1,k2,k3,k4,k5,k6,k7,k8;
    Right_straight_flag=0;   // 初始化右直线标志
    if(Endline>17)return;
    // 计算不同区域的斜率用于判断直线度
    k1=((float)right[Endline+70]-(float)right[Endline+40])/30;  // 底部区域斜率
    k2=((float)right[Endline+60]-(float)right[Endline+50])/10;  // 中部区域斜率
    k3=((float)right[Endline+50]-(float)right[Endline+10])/40;   // 顶部到底部斜率
    k4=((float)right[Endline+30]-(float)right[Endline+10])/20;   // 顶部到中部斜率
    k5=((float)right[Endline+70]-(float)right[Endline+60])/10;   // 最底部区域斜率
    k6=((float)right[Endline+40]-(float)right[Endline+30])/10;   // 中下部区域斜率
    k7=((float)right[Endline+40]-(float)right[Endline+15])/25;   // 中上部区域斜率
    k8=((float)right[Endline+20]-(float)right[Endline+10])/10;   // 上部细分区域斜率
    // 判断是否为直线 - 使用更多斜率点进行精确判断
    if(absolute(k1-k2)<0.3&&absolute(k2-k3)<0.3&&absolute(k3-k4)<0.3&&
       absolute(k4-k5)<0.3&&absolute(k5-k6)<0.3&&absolute(k6-k7)<0.3&&
       absolute(k7-k8)<0.3&&absolute(k1-k8)<0.5&&
       k1!=0&&k2!=0&&k3!=0&&k4!=0&&k5!=0&&k6!=0&&k7!=0&&k8!=0){
            Right_straight_flag=1;
    }
}
//==================================================左直线识别===========================================================
void left_straight(void){
    float k1, k2, k3, k4, k5, k6, k7, k8;
    Left_straight_flag = 0;   // 初始化左直线标志
    if(Endline>17)return;
    // 计算不同区域的斜率用于判断直线度
    k1=((float)left[Endline+80]-(float)left[Endline+50])/30;  // 底部区域斜率
    k2=((float)left[Endline+70]-(float)left[Endline+60])/10;  // 中部区域斜率
    k3=((float)left[Endline+60]-(float)left[Endline+10])/50;   // 顶部到底部斜率
    k4=((float)left[Endline+30]-(float)left[Endline+10])/20;   // 顶部到中部斜率
    k5=((float)left[Endline+80]-(float)left[Endline+70])/10;   // 最底部区域斜率
    k6=((float)left[Endline+50]-(float)left[Endline+30])/20;   // 中下部区域斜率
    k7=((float)left[Endline+40]-(float)left[Endline+15])/25;   // 中上部区域斜率
    k8=((float)left[Endline+20]-(float)left[Endline+10])/10;   // 上部细分区域斜率
    // 判断是否为直线 - 使用更多斜率点进行精确判断
    if(absolute(k1-k2)<0.3&&absolute(k2-k3)<0.3&&absolute(k3-k4)<0.3&&
       absolute(k4-k5)<0.3&&absolute(k5-k6)<0.3&&absolute(k6-k7)<0.3&&
       absolute(k7-k8)<0.3&&absolute(k1-k8)<0.5&&
       k1!=0&&k2!=0&&k3!=0&&k4!=0&&k5!=0&&k6!=0&&k7!=0&&k8!=0){
            Left_straight_flag=1;
    }
}
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
/**
 * @brief 十字交叉检测与填充函数
 * @param uint8(*image)[image_w]     灰度图像数组
 * @param uint8 *l_border            左边缘数组
 * @param uint8 *r_border            右边缘数组
 * @param uint16 total_num_l         左边缘点总数
 * @param uint16 total_num_r         右边缘点总数
 * @param uint16 *dir_l              左边缘方向数组
 * @param uint16 *dir_r              右边缘方向数组
 * @param uint16(*points_l)[2]       左边缘点坐标数组
 * @param uint16(*points_r)[2]       右边缘点坐标数组
 * @return 无返回值，结果存储在全局变量中
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                                         uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint16 i;
    uint8 break_num_l = 0;  // 左边缘突变点的行号
    uint8 break_num_r = 0;  // 右边缘突变点的行号
    uint8 end_num_l = 0;    // 左边缘恢复点的行号
    uint8 end_num_r = 0;    // 右边缘恢复点的行号
    uint8 start, end;       // 拟合区域的起止行号
    float slope_l_rate = 0, intercept_l = 0;  // 直线拟合的斜率和截距
    // 当未检测到十字交叉时，进行交叉路口判断
    if(cross_sum == 0)
    {
         // 检查上下区域边缘丢失情况，符合特定组合时判定为十字交叉
        if(((l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 60 ,80) == 1)
                || (l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 60 ,80) == 1)
                || (l_loss_judge(l_border,90 ,110) == 1 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 60 ,80) == 1))
                && sum_island == 0 // 确保没有孤岛点干扰
                )
        {
            cross_sum = 1; // 标记为十字交叉检测状态
        }
    }
     // 当处于十字交叉检测状态时
    if(cross_sum == 1)
    {
        // 检测左边缘的方向突变点和恢复点
        for (i = 1; i < total_num_l; i++)
        {   
            // 情况1：方向从4变为非4（边缘方向突变）
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                  /* 第一阶段判断边缘方向是否从4变为非4 */
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /* 第二阶段确认是否为左边缘（靠近图像左侧） */
            {
                break_num_l = (uint8)points_l[i][1];  // 记录突变点的行号
                break;
            }
            // 情况2：方向从非4变为4，表示边缘方向恢复
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /* 第一阶段判断边缘方向是否从非4变为4 */
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
               /* 第二阶段确认是否为左边缘 */
            {
                end_num_l = (uint8)points_l[i][1];// 记录恢复点的行号
            }
        }
        // 检测右边缘的方向突变点和恢复点（逻辑与左边缘相同）
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                   /* 第一阶段判断边缘方向是否突变 */
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /* 第二阶段确认是否为右边缘（靠近图像右侧） */
            {
                break_num_r = (uint8)points_r[i][1];// 记录突变点的行号
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                      /* 第一阶段判断边缘方向是否恢复 */
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                        /* 第二阶段确认是否为右边缘 */
            {
                end_num_r = (uint8)points_r[i][1];// 记录恢复点的行号
            }
        }
        // 根据突变点和恢复点的情况，决定是否转换到下一状态
        if ((end_num_l == 0 && end_num_r == 0)||(end_num_l >= 110 && end_num_r >= 110)
                ||(break_num_l && break_num_r == 0 && end_num_l == 0 && end_num_r)
                ||(break_num_r && break_num_l == 0 && end_num_r == 0 && end_num_l)
                )
        {
            cross_sum = 2;// 转换到十字交叉后处理状态
        }
        // 当仅左边缘有突变和恢复点时，修复左边缘
        if (break_num_l && break_num_r == 0 && end_num_l && end_num_r == 0)
        {
             // 计算拟合直线的斜率（k = Δy/Δx）
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б??k=y/x
             // 计算拟合直线的截距（b = y - kx）
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//???b=y-kx
            // 用直线方程修复边缘（x = (y - b)/k）
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
        // 当仅右边缘有突变和恢复点时，修复右边缘（逻辑与左边缘相同）
        if (break_num_r && break_num_l == 0 && end_num_r && end_num_l == 0)
         {
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б??k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//???b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
         }
        // 当双侧边缘均有突变和恢复点时，同时修复两侧边缘
        if (break_num_l && break_num_r && end_num_l && end_num_r)
        {
             // 修复左边缘
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б??k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//???b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            // 修复右边缘
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б??k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//???b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }

    }
    // 当处于十字交叉后处理状态时
    if(cross_sum == 2)
    {
        // 重新检测左边缘的方向突变点（调整搜索起始位置)
        for (i = 10; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                  /* 判断边缘方向是否突变 */
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                 /* 确认是否为左边缘 */
            {
                break_num_l = (uint8)points_l[i][1]-5;// 记录突变点行号并微调
                break;
            }
        }
        // 重新检测右边缘的方向突变点（逻辑与左边缘相同）
        for (i = 10; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                   /* 判断边缘方向是否突变 */
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /* 确认是否为右边缘 */
            {
                break_num_r = (uint8)points_r[i][1]-5;// 记录突变点行号并微调
                break;
            }
        }
         // 判断是否需要退出十字交叉状态
        if((break_num_l == 0 && break_num_r == 0)||(break_num_l >= 115 && break_num_r >= 115)
                ||(l_loss_judge(l_border,90 ,110) == 0 && r_loss_judge(r_border,90 ,110) == 0))
        {
            cross_sum = 0;// 退出十字交叉状态
        }
         // 使用最小二乘法拟合直线，延伸修复左边缘
        start = break_num_l - 10;  // 拟合起始行
        start = (uint8)limit_a_b(start, 0, image_h);  // 确保起始行有效
        end = break_num_l - 5;     // 拟合终止行
        calculate_s_i(start, end, left, &slope_l_rate, &intercept_l);  // 最小二乘法拟合
         // 用拟合直线延伸修复边缘到图像底部
        for (i = break_num_l - 5; i < image_h - 1 + 50; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//???
        }

        // 使用最小二乘法拟合直线，延伸修复右边缘（逻辑与左边缘相同）
        start = break_num_r - 10;  // 拟合起始行
        start = (uint8)limit_a_b(start, 0, image_h);  // 确保起始行有效
        end = break_num_r - 5;     // 拟合终止行
        calculate_s_i(start, end, right, &slope_l_rate, &intercept_l);  // 最小二乘法拟合
         // 用拟合直线延伸修复边缘到图像底部
        for (i = break_num_r - 5; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }

}
//=============================================圆环===================================================
//================环岛识别=====================

void roundabout_L(void){//OK,上拐点识别
    uint8 detected = 0;  // 本次检测结果
    uint8 temp_x = 0, temp_y = 0; // 临时坐标
    
    for(y=50;y>10;y--){
        if(my_abs(left[y]-left[y-1])<3&&my_abs(left[y]-left[y+1])>10&&left[y]>10&&Lost_left_Flag==1){
            //y+=4;
            detected = 1;
            temp_x = left[y];
            temp_y = y;
            break;
        }
    }
    
    // 滤波处理
    if(detected) {
        roundabout_L_filter++;
        if(roundabout_L_filter >= ROUNDABOUT_FILTER_THRESHOLD) {
            roundabout_Flag = 1;
            roundabout_X = temp_x;
            roundabout_Y = temp_y;
            roundabout_L_filter = ROUNDABOUT_FILTER_THRESHOLD; // 防止溢出
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

void roundabout_R(void){//OK,上拐点识别
    uint8 detected = 0;  // 本次检测结果
    uint8 temp_x = 0, temp_y = 0; // 临时坐标
    
    for(y=50;y>10;y--){
        if(my_abs(right[y]-right[y-1])<3&&my_abs(right[y]-right[y+1])>10&&right[y]<178&&Lost_right_Flag==1){
            //y+=4;
            detected = 1;
            temp_x = right[y];
            temp_y = y;
            break;
        }
    }
    
    // 滤波处理
    if(detected) {
        roundabout_R_filter++;
        if(roundabout_R_filter >= ROUNDABOUT_FILTER_THRESHOLD) {
            roundabout_Flag = 1;
            roundabout_X = temp_x;
            roundabout_Y = temp_y;
            roundabout_R_filter = ROUNDABOUT_FILTER_THRESHOLD; // 防止溢出
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
  * @brief    重置roundabout_L/R滤波器
  * @param    无
  * @return   无
  * @note     在圆环状态改变时调用，清零滤波计数器
  */
void Roundabout_ResetFilter(void)
{
    roundabout_L_filter = 0;
    roundabout_R_filter = 0;
}

//================出环拐点识别=====================
void Exit_loop_L_inflection(void){
   uint8 i;
   Exit_loop_Flag=0;
   Exit_loop_X=0;
   Exit_loop_Y=0;
  //左圆环，识别右边拐点
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
  //右圆环，识别左边拐点
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
 * @brief 判断左右边界白色数量是否过多
 * @param void
 * @return 检测结果 (1:白色数量过多, 0:正常)
 * @note 从Endline+5开始检测，逐渐增加阈值到10
 */
int check_border_white_excess(void)
{
    int left_white_count = 0;   // 左边界白色像素计数
    int right_white_count = 0;  // 右边界白色像素计数
    int total_check_rows = 0;   // 检查的总行数
    
    // 从Endline+5开始向下扫描到图像底部
    for(int i = Endline + 5; i < image_h - 5; i++)
    {
        total_check_rows++;
        
        // 检查左边界位置的像素（第3列）
        if(imag[i][3] == White)
        {
            left_white_count++;
        }
        
        // 检查右边界位置的像素（第185列）
        if(imag[i][185] == White)
        {
            right_white_count++;
        }
    }
    
    // 动态计算阈值：从5开始逐渐增加到10
    int threshold_base = 15;                    // 基础阈值
    int threshold_increment = 5;               // 增量
    int max_threshold = 60;                    // 最大阈值
    
    // 根据检查行数计算当前阈值
    int current_threshold = threshold_base;
    if(total_check_rows > 20)
    {
        current_threshold = threshold_base + ((total_check_rows - 20)*3 / 4);
        if(current_threshold > max_threshold)
        {
            current_threshold = max_threshold;
        }
    }
    
    // 判断白色数量是否超过阈值
    if(left_white_count >= current_threshold && right_white_count >= current_threshold)
    {
        return 1;  // 白色数量过多
    }
    
    return 0;  // 正常
}
/**
* @brief 最小二乘法
* @param uint8 begin                起始点
* @param uint8 end                  结束点
* @param uint8 *border              指向需要计算斜率的边界首地址
* @see CTest       Slope_Calculate(start, end, border);//斜率
* @return 返回值说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast; // 静态变量保存上一次计算结果

    // 计算各项累加和
    for (i = begin; i < end; i++)
    {
        xsum += i;               // x值累加
        ysum += border[i];       // y值累加
        xysum += i * border[i];  // x*y累加
        x2sum += i * i;          // x平方累加

    }
    // 计算斜率(最小二乘法公式)
    if ((end - begin)*x2sum - xsum * xsum) // 判断分母是否为0
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;// 保存有效结果
    }
    else
    {
        result = resultlast;// 使用上一次的结果
    }
    return result;
}

/**
* @brief 计算斜率截距
* @param uint8 start                起始点
* @param uint8 end                  结束点
* @param uint8 *border              指向需要计算斜率的边界
* @param float *slope_rate          返回斜率的地址
* @param float *intercept           返回截距的地址
* @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回值说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;        // i：循环计数器，num：有效点数量
    uint16 xsum = 0, ysum = 0; // xsum/ysum：x和y坐标的累加和
    float y_average, x_average; // x_average/y_average：x和y坐标的平均值
    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;        // x坐标累加（i 作为 x 值）
        ysum += border[i];// y坐标累加（border[i] 作为 y 值）
        num++;            // 有效点数量计数
    }

    // 计算平均值
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);

    }

     // 计算斜率和截距
    *slope_rate = Slope_Calculate(start, end, border);  // 调用斜率计算函数
    *intercept = y_average - (*slope_rate) * x_average;  // 计算截距
}
uint8 broken_line_x = 0;
uint8 broken_line_y = 0;
/**
 * @brief 断点检测函数
 * @param uint8 dir        搜索方向（0：从上到下搜索，1：从下到上搜索）
 * @param uint8 start      搜索起始行号
 * @param uint8 *border    边缘位置数组的指针（索引为行号，值为列号）
 * @see CTest       broken_line_judge(1,90,120,l_border);
 * @return 无返回值，结果存储在全局变量中
 */
void broken_line_judge(int dir, uint8 start, uint8 end, uint8 *border)
{
     // 从start行开始，逐行向下搜索到end行
     if(dir==0){
    for (uint16 i = (uint16)start; i <= (uint16)end; i++)
    {
        // 计算当前行与上一行的边缘位置差值，判断是否超过阈值（4像素）
        for(uint16 i = (uint16)start ;i <= (uint16)end; i++)
        {
            if(my_abs(border[i] - border[i-1]) >= 4)
            {
                // 记录上一行的边缘位置作为断点坐标（因为突变发生在i和i-1行之间）
                broken_line_x = border[i-1];  // 断点的x坐标（列号）
                broken_line_y = (uint8)(i-1); // 断点的y坐标（行号）
                break;  // 找到第一个断点后立即退出循环
            }

             // 当搜索到倒数第三行（end-2）时，若未找到断点则标记为无效
            if(i == (uint16)(end-2))
            {
                broken_line_x = -1;  // 无效x坐标
                broken_line_y = -1;  // 无效y坐标
                break;
            }
        }
    }
}
    if(dir == 1)
    {
        // 从end行开始，逐行向上搜索到start行
        for(uint16 i = (uint16)end;i >= (uint16)start;i--)
        {
             // 计算当前行与下一行的边缘位置差值，判断是否超过阈值（4像素）
            if(my_abs(border[i] - border[i+1]) >= 4)
            {
                // 记录下一行的边缘位置作为断点坐标（因为突变发生在i和i+1行之间）
                broken_line_x = border[i+1];  // 断点的x坐标（列号）
                broken_line_y = (uint8)(i+1); // 断点的y坐标（行号）
                break;  // 找到第一个断点后立即退出循环
            }
            // 当搜索到正数第三行（start+2）时，若未找到断点则标记为无效
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
 * @brief 单调性判断函数
 * @param uint8* hightest            最高有效行指针
 * @param uint8 *l_border            左边缘数组的地址
 * @param uint8 *r_border            右边缘数组的地址
 * @param uint16 total_num_l         左边缘总点数
 * @param uint16 total_num_r         右边缘总点数
 * @param uint16 *dir_l              左边缘方向数组的地址
 * @param uint16 *dir_r              右边缘方向数组的地址
 * @param uint16(*points_l)[2]       左边缘点坐标数组的地址
 * @param uint16(*points_r)[2]       右边缘点坐标数组的地址
 * @param uint16 *l_index            左边缘索引数组的地址
 * @param uint16 *r_index            右边缘索引数组的地址
 * @see CTest      monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
 * @return 返回值说明
 * 1 单调
 * 0 非单调
 */
int monotonicity_l = -1;  // 左边缘单调性结果（全局变量）
int monotonicity_r = -1;  // 右边缘单调性结果（全局变量）
void monotonicity_line(uint8* hightest, uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
        uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],uint16 *l_index,uint16 *r_index)
{
    uint16 i;
    uint16 total_l = 0;
    uint16 total_r = 0;      // 左右边缘的总点数
    uint16 arr_l = 0;
    uint16 arr_r = 0;      // 左右边缘符合特定方向条件的点数
    uint16 abb_l = 0;
    uint16 abb_r = 0;      // 左右边缘相邻点变化超过阈值的次数
    uint16 acc_l = 0;
    uint16 acc_r = 0;      // 左右边缘索引变化超过阈值的次数
    // 左边缘的处理
    for (i = 1; i < total_num_l-20; i++)
    {
        // 找到边缘点与边界重合且行号小于等于20的点，跳出循环
        if(points_l[i][0] == (uint16)l_border[points_l[i][1]] && points_l[i][1] <= 20)
            break;
            
        // 统计方向为4、5、6的点（这些方向可能代表边缘的连续性较好）
        if ((dir_l[i] == 4) || (dir_l[i] == 5) || (dir_l[i] == 6))
        {
            arr_l++;
        }
        total_l++;
    }
  // 如果大部分点的方向符合条件（arr_l >= total_l-5）
    if(arr_l>=(total_l-5))
    {
        // 如果最高有效行大于等于20，从最高有效行开始检查
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                // 统计索引变化超过5的次数（可能表示边缘不连续）
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;  // 非单调
                    break;
                }
 // 统计边缘位置变化超过2的次数（可能表示边缘波动大）
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;  // 非单调
                    break;
                }
                
                // 如果检查到第105行还没有发现问题，认为是单调的
                if(i == 105)
                {
                    monotonicity_l = 1;  // 单调
                    break;
                }
            }
        }
        else// 如果最高有效行小于20，从第20行开始检查
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
    else monotonicity_l = 0;// 如果方向统计不符合条件，直接判定为非单调
    // 右边缘的处理（逻辑与左边缘相同）
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
       // 右边缘单调性判断（逻辑与左边缘相同）
    if(arr_r>=(total_r-5))
    {
          // 从最高有效行开始向下检查边缘单调性
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                // 检查索引变化是否超过阈值（可能反映边缘跳跃）
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /* 逻辑：相邻行索引变化≥5的次数≥5次，认为边缘索引不连续 */
                }
                // 检查边缘位置变化是否超过阈值（可能反映边缘波动）
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /* 逻辑：相邻行边缘位置变化≥2的次数≥5次，认为边缘位置波动大 */
                }
                // 强制终止条件：检查到第105行时判定为单调
                if(i == 105)
                   /* 逻辑：若前105行未发现异常，认为边缘整体单调 */
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
        else
        {
            // 当最高有效行<20时，从第20行开始检查（忽略顶部可能的噪声区域）
            for (int i = 20; i < image_h-2; i++)
            {
                // 与上一分支相同的索引变化检查
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
//================左圆环识别=====================

uint8_t UpdownSideGet(void)
{
    uint8_t i = 0, j = 0;
    uint8_t last = image_h/2;

    imageOut[0][image_w-1] = 0;
    imageOut[1][image_w-1] = image_h-1;
     //从图像中间行    从中到下     从中到上      扫描

    //处理中间单独那一列的上下边线
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
    //其他列的上下边线
    //从中到左
    for(i = image_w/2-1; i > 2; i--)//遍历每一列
    {
        imageOut[up][i] = 0;
        imageOut[down][i] = image_h-1;

        for(j = imageOut[0][i+1] + 30; j > 5; j--)//一列中的扫描每行  从上列的行数+10开始向上扫描
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

    //从中到右
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
  * @brief    判断左右边线是否存在弧形
  * 输出的 index 圆弧的顶点位置
  * @param    imageInput ： 二值图像信息
  * @param    imageOut   ： 边线数组
  * @param    status     ： 1：左边线  2：右边线
  * @param    num        ： 圆弧的大小 用点数表示  （连续N个增  连续N个减）
  * @return   1 有弧线    0  没弧线

  */
 uint8_t inc = 0, dec = 0, n = 0,inc_y=0;
uint8_t RoundaboutGetArc(uint8_t status, uint8_t num)
{
    int i = 0;
    uint8_t arc_detected = 0;  // 本次检测结果
    inc = 0; dec = 0; n = 0;inc_y=0;
    
    switch(status)
    {
      case 1:  // 左边线检测
        for(i = Endline+10; i < 100; i++)
        {
        	//没有丢线
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
                /* 有弧线 */
            }
        }
        
        // 左边线滤波处理
        if(arc_detected)
        {
            roundabout_arc_filter_left++;
            if(roundabout_arc_filter_left >= ROUNDABOUT_ARC_FILTER_THRESHOLD)
            {
                roundabout_arc_filter_left = ROUNDABOUT_ARC_FILTER_THRESHOLD; // 防止溢出
                return 1;
            }
        }
        else
        {
            roundabout_arc_filter_left = 0; // 未检测到则清零
        }
        break;

      case 2:  // 右边线检测
        for(i = Endline+10; i < 100; i++)
        {
            //没有丢线
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

                /* 有弧线 */
                if(inc>9&&dec>1)
                {
                    arc_detected = 1;
                    break;
                }
            }
        }
        
        // 右边线滤波处理
        if(arc_detected)
        {
            roundabout_arc_filter_right++;
            if(roundabout_arc_filter_right >= ROUNDABOUT_ARC_FILTER_THRESHOLD)
            {
                roundabout_arc_filter_right = ROUNDABOUT_ARC_FILTER_THRESHOLD; // 防止溢出
                return 1;
            }
        }
        else
        {
            roundabout_arc_filter_right = 0; // 未检测到则清零
        }
        break;
    }

    return 0;
}

/*!
  * @brief    重置RoundaboutGetArc滤波器
  * @param    无
  * @return   无
  * @note     在环岛状态改变时调用，清零滤波计数器
  */
void RoundaboutGetArc_ResetFilter(void)
{
    roundabout_arc_filter_left = 0;
    roundabout_arc_filter_right = 0;
}

/*!
  * @brief    判断上边线是否单调
  * @return   0：不单调or错误， 1：单调递增， 2：单调递减
  * @note
  * @see
  * @date     2021/11/30 星期二
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
                    ips200_draw_point(middle[i], i,  RGB565_GREEN);
                    ips200_draw_point(left_cicle[i], i, RGB565_RED);
                }
            }
        }
        else Addingline1( 1, Lower_left_inflection_X, Lower_left_inflection_Y);
    }
    if (annulus_L_memory == 2 )
    {
        roundabout_L();          //环岛
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
        roundabout_L();          //环岛
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
            RoundaboutGetArc_ResetFilter(); // 重置圆环弧线检测滤波器
            Roundabout_ResetFilter(); // 重置圆环上拐点检测滤波器
            if(mode!=stopworking)ips200_full(RGB565_BLACK);
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
    }
}

//================右圆环识别=====================
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
        roundabout_R();          //环岛
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
        roundabout_R();          //环岛
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
                    //  middle[i] = (left[i] + right[i]) >> 1;//求中线
                    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
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
            RoundaboutGetArc_ResetFilter(); // 重置圆环弧线检测滤波器
            Roundabout_ResetFilter(); // 重置圆环上拐点检测滤波器
            if(mode!=stopworking)ips200_full(RGB565_BLACK);
            yaw_cicle=0;
            yaw_cicle_flag=0;
        }
        else return;
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

      case 3://中补线
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

      case 3://中补线
      {
           // 使用左右边界的平均值来计算中线的斜率
           float left_k = (float)(((float)left[Lower_left_inflection_Y+1] - (float)left[Lower_left_inflection_Y+5]) /(-4));
           float right_k = (float)(((float)right[Lower_right_inflection_Y+1] - (float)right[Lower_right_inflection_Y+5]) /(-4));
           
           // 取左右斜率的平均值作为中线斜率
           k = (left_k + right_k) / 2;
           
           // 使用起始点计算截距
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
     if(banmaxian_hangshu >= 4  // 至少有4行检测到有效斑马线
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

// 斜率计算相关阈值
#define SLOPE_CALC_HEIGHT_LIMIT   110      // 处理高度限制
#define SLOPE_CALC_WIDTH_LIMIT    170      // 处理宽度限制
#define SLOPE_DIFF_THRESHOLD      1.5f     // 斜率差异阈值
#define CENTER_ZONE_MIN           85       // 中线区域最小值
#define CENTER_ZONE_MAX           103      // 中线区域最大值
#define MIN_VALID_POINTS          3        // 最少有效点数
#define SAMPLE_INTERVAL           3        // 采样间隔
#define MAX_COLLECT_POINTS        8        // 最大收集点数

/**
 * @brief 计算左右边界的稳定斜率（排除突变点）
 * @param l_border 左边界数组
 * @param r_border 右边界数组
 * @param highest 最高点位置
 * @param left_slope 返回的左边界斜率指针
 * @param right_slope 返回的右边界斜率指针
 * @return 有效性标志 (1:两边都有效, 0:无效)
 * @note 如果左边界三个x间隔相差3的点基本连成一条线，且值不是94附近那么求出他的斜率
 *       右侧同理，这个函数的作用是在有突变的情况下求出除突变以外的线条斜率
 */
int calculate_border_slopes(uint8 *l_border, uint8 *r_border, uint8 highest, float *left_slope, float *right_slope)
{
    *left_slope = 1.0f;   // 无效时设置为1
    *right_slope = 1.0f;  // 无效时设置为1
    
    int left_valid = 0, right_valid = 0;
    
    // 左边界斜率计算
    int left_valid_points = 0;
    float left_sum_x = 0, left_sum_y = 0, left_sum_xy = 0, left_sum_x2 = 0;
    
    // 从底部向上扫描，寻找连续的有效点，限制在110高度内
    for(int y = SLOPE_CALC_HEIGHT_LIMIT; y >= highest + 10 && y >= SAMPLE_INTERVAL * 2; y -= SAMPLE_INTERVAL)
    {
        if(y >= SAMPLE_INTERVAL * 2 && y <= SLOPE_CALC_HEIGHT_LIMIT - SAMPLE_INTERVAL)
        {
            int x1 = l_border[y + SAMPLE_INTERVAL * 2];  // 当前点下方6行
            int x2 = l_border[y + SAMPLE_INTERVAL];      // 当前点下方3行
            int x3 = l_border[y];                        // 当前点
            
            // 排除中线区域附近的点
            if(x1 >= CENTER_ZONE_MIN && x1 <= CENTER_ZONE_MAX) continue;
            if(x2 >= CENTER_ZONE_MIN && x2 <= CENTER_ZONE_MAX) continue;
            if(x3 >= CENTER_ZONE_MIN && x3 <= CENTER_ZONE_MAX) continue;
            
            // 排除明显的边界丢失点，限制在170宽度内
            if(x1 <= 5 || x2 <= 5 || x3 <= 5) continue;
            if(x1 >= SLOPE_CALC_WIDTH_LIMIT || x2 >= SLOPE_CALC_WIDTH_LIMIT || x3 >= SLOPE_CALC_WIDTH_LIMIT) continue;
            
            // 检查三点是否基本共线（斜率变化不大）
            float slope1 = (float)(x2 - x1) / (float)SAMPLE_INTERVAL;
            float slope2 = (float)(x3 - x2) / (float)SAMPLE_INTERVAL;
            
            // 如果两段斜率相差不大，认为是直线段
            if(fabs(slope1 - slope2) <= SLOPE_DIFF_THRESHOLD)
            {
                // 使用最小二乘法累积计算
                left_sum_x += y;
                left_sum_y += x3;
                left_sum_xy += y * x3;
                left_sum_x2 += y * y;
                left_valid_points++;
                
                // 如果找到足够多的点就停止
                if(left_valid_points >= MAX_COLLECT_POINTS) break;
            }
        }
    }
    
    // 计算左边界斜率
    if(left_valid_points >= MIN_VALID_POINTS)
    {
        float denominator = left_valid_points * left_sum_x2 - left_sum_x * left_sum_x;
        if(fabs(denominator) > 0.001f)
        {
            *left_slope = (left_valid_points * left_sum_xy - left_sum_x * left_sum_y) / denominator;
            left_valid = 1;
        }
    }
    
    // 右边界斜率计算（逻辑类似）
    int right_valid_points = 0;
    float right_sum_x = 0, right_sum_y = 0, right_sum_xy = 0, right_sum_x2 = 0;
    
    for(int y = SLOPE_CALC_HEIGHT_LIMIT; y >= highest + 10 && y >= SAMPLE_INTERVAL * 2; y -= SAMPLE_INTERVAL)
    {
        if(y >= SAMPLE_INTERVAL * 2 && y <= SLOPE_CALC_HEIGHT_LIMIT - SAMPLE_INTERVAL)
        {
            int x1 = r_border[y + SAMPLE_INTERVAL * 2];
            int x2 = r_border[y + SAMPLE_INTERVAL];
            int x3 = r_border[y];
            
            // 排除中线区域附近的点
            if(x1 >= CENTER_ZONE_MIN && x1 <= CENTER_ZONE_MAX) continue;
            if(x2 >= CENTER_ZONE_MIN && x2 <= CENTER_ZONE_MAX) continue;
            if(x3 >= CENTER_ZONE_MIN && x3 <= CENTER_ZONE_MAX) continue;
            
            // 排除明显的边界丢失点，限制在170宽度内
            if(x1 <= 5 || x2 <= 5 || x3 <= 5) continue;
            if(x1 >= SLOPE_CALC_WIDTH_LIMIT || x2 >= SLOPE_CALC_WIDTH_LIMIT || x3 >= SLOPE_CALC_WIDTH_LIMIT) continue;
            
            // 检查三点是否基本共线
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
    
    // 计算右边界斜率
    if(right_valid_points >= MIN_VALID_POINTS)
    {
        float denominator = right_valid_points * right_sum_x2 - right_sum_x * right_sum_x;
        if(fabs(denominator) > 0.001f)
        {
            *right_slope = (right_valid_points * right_sum_xy - right_sum_x * right_sum_y) / denominator;
            right_valid = 1;
        }
    }
    
    // 返回有效性：1表示两边都有效，0表示至少有一边无效
    return (left_valid && right_valid) ? 1 : 0;
}

// 边界检查相关参数
#define BORDER_CHECK_START_ROW    40      // 检查起始行
#define BORDER_CHECK_END_ROW      80      // 检查结束行
#define BORDER_MIN_VALID          3       // 边界最小有效值 (放宽)
#define BORDER_MAX_VALID          170     // 边界最大有效值 (放宽)
#define MAX_INVALID_POINTS        15      // 允许的最大无效点数 (放宽)

/**
 * @brief 检查指定范围内左右边界是否都在有效范围内
 * @param l_border 左边界数组
 * @param r_border 右边界数组
 * @return 检查结果 (1:大部分有效, 0:无效点过多)
 * @note 遍历40到80行，检查左右边界是否都在3-185范围内
 *       允许少量无效点，增加容错性
 */
int check_border_validity(uint8 *l_border, uint8 *r_border)
{
    int invalid_count = 0;
    int total_points = 0;
    
    // 遍历指定行范围
    for(int y = BORDER_CHECK_START_ROW; y <= BORDER_CHECK_END_ROW; y++)
    {
        total_points += 2; // 每行检查左右两个点
        
        // 检查左边界是否在有效范围内
        if(left[y] <= BORDER_MIN_VALID )
        {
            invalid_count++;
        }
        
        // 检查右边界是否在有效范围内
        if( right[y] >= BORDER_MAX_VALID)
        {
            invalid_count++;
        }
    }
    
    // 调试信息：显示统计结果
    // printf("Border check: invalid=%d, total=%d, threshold=%d\n", 
    //        invalid_count, total_points, MAX_INVALID_POINTS);
    
    // 如果无效点数量在容错范围内，认为边界有效
    return (invalid_count <= MAX_INVALID_POINTS) ? 1 : 0;
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
    if( Crossroad_memory!=0||
        Crossroad_Flag!=0||
        (annulus_R_memory!=0&&annulus_R_memory!=7)||
        (annulus_L_memory!=0&&annulus_L_memory!=7)||
        BridgeState != SINGLE_BRIDGE_NOT_ACTIVE||
        jump_active
        )return;
    blake_line = 0; // 记录连续全黑行的数量
    // 当最高点位置≥55时进行跳变点检测
    if(*hightest>=30)
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
    if(blake_line >= 20)//想近一点再跳，这里就大一点最多40吧，远一点跳这里就小一点最小就这个吧，看看效果，用delay的舒服就用delay,也可以两个结合着来调
    {
        // mode_switch(jump);
        // if(mode!=stopworking)ips200_full(RGB565_MAGENTA);
        // 检查特定区域边界是否连续（无丢失），说实话这里没必要，如果弯道会误触发跳跃的话你就检测border
        //if(l_loss_judge(l_border,90 ,110) == 0 &&  // 左边界90-110行连续
         //  l_loss_judge(l_border,70 ,90) == 0 &&   // 左边界70-90行连续
        // r_loss_judge(r_border,90 ,110) == 0   // 右边界90-110行连续
         //  r_loss_judge(r_border,70 ,90) == 0 // 右边界70-90行连续
            //&&monotonicity_l == 1 && monotonicity_r == 1
         //   )
    
             // 确保不在其他特殊区域（桥、环岛、十字线）
            {
                //  这里把mode改成jump模式（也是flexible模式），然后把腿放为对称，不用舵机控制速度环
            //mode=jump;
            //ServoPID.highleft=3.3;//左右腿高一致
            //ServoPID.highright=3.3;
            //  delay,这里最好根据速度写个延时函数来控制，速度快就早点跳，速度慢就晚点
            //jump_delay(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data,int kp)，这个函数就在jump_judge函数的上
            if(mode!=stopworking)ips200_full(RGB565_GREEN);
            jump_act();
            }
        }
}
//
/**
* @brief 单桥状态检测与填充函数
* @param uint8(*image)[image_w]     二值化图像
* @param uint8 *l_border            左边界数组
* @param uint8 *r_border            右边界数组
* @param uint8 *center_line         中心线数组（输出）
* @param uint8* hightest   nili         最高点指针
* @return 无返回值，通过BridgeState全局变量标记桥梁状态
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
int loss_1 = 0;       // 边界宽度变化计数器
int loss_2 = 0;       // 边界位置突变计数器
int bridge_number = 0; // 桥梁特征计数器
int bridge_out_flag=0;
int bridge_in_flag=0;
int bridge_in_flag_num=0;
int bridge_out_flag_num=0;
int bridge_entry_cooldown=0;
// 两腿等高时间计数器 (单位：10ms，2秒=200次)
static int legs_equal_height_counter = 0;
int white_line1 = 0;  // 第46行白色像素计数
int white_line2 = 0;  // 第44行白色像素计数
int white_line3 = 0;  // 第60行白色像素计数
int white_line4 = 0;  // 第58行白色像素计数
int left_pattern_found = 0;  // 左侧白-黑-白模式
int right_pattern_found = 0; // 右侧白-黑-白-黑-白模式
int bridge_mode_choice=0;
// 记录第一个左边界或右边界突变点坐标
int first_boundary_change_x = -1;
int first_boundary_change_y = -1;
int first_boundary_change_recorded = 0;  
// 记录最后一个左边界或右边界突变点坐标
int last_boundary_change_x = -1;
int last_boundary_change_y = -1;
// 记录突变次数
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
    white_line1 = 0;  // 第75行白色像素计数
    white_line2 = 0;  // 第70行白色像素计数
    int long_start_l = 0; // 左边界突变起始行
    int long_end_l = 0;   // 左边界突变结束行
    int long_start_r = 0; // 右边界突变起始行
    int long_end_r = 0;   // 右边界突变结束行
     loss_1 = 0;       // 边界宽度变化计数器
     loss_2 = 0;       // 边界位置突变计数器
     bridge_number = 0; // 桥梁特征计数器
     bridge_in_flag=0;
    // 扫描图像中间区域，分析边界变化
    //当桥梁状态为"未激活"时，检测是否进入桥梁
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        // 首先检查边界是否丢失，如果丢失则不进行后续检测
        if (Lost_left_Flag == 0 && Lost_right_Flag == 0) {
            // 检测3个部分的突变
            int part1_changes = 0;  // 第一部分突变计数
            int part2_changes = 0;  // 第二部分突变计数
            int part3_changes = 0;  // 第三部分突变计数
            int total_changes = 0;  // 总突变计数
            // 第一部分：从Endline+10到40，相差4算突变
            for (int y = Endline + 10; y < 40 && y < image_h - 1; y++) {
                if (part1_changes < 2) {  // 每个部分最多2个突变
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 4 || abs(right_cicle[y] - right_cicle[y+1]) > 4) {
                        part1_changes++;
                        total_changes++;
                    }
                }
            }
            // 第二部分：从40到80，相差7算突变
            for (int y = 40; y < 80 && y < image_h - 1; y++) {
                if (part2_changes < 2) {  // 每个部分最多2个突变
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 7 || abs(right_cicle[y] - right_cicle[y+1]) > 7) {
                        part2_changes++;
                        total_changes++;
                    }
                }
            }
            // 第三部分：从80到118，相差10算突变
            for (int y = 80; y < 118 && y < image_h - 1; y++) {
                if (part3_changes < 2) {  // 每个部分最多2个突变
                    if (abs(left_cicle[y] - left_cicle[y+1]) > 10 || abs(right_cicle[y] - right_cicle[y+1]) > 10) {
                        part3_changes++;
                        total_changes++;
                    }
                }
            }
            // 检测条件：记录4个突变
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
    // 当桥梁状态为"激活"时，处理桥梁区域并生成中心线
    if(BridgeState == SINGLE_BRIDGE_ACTIVE){
        first_boundary_change_x = -1;
        first_boundary_change_y = -1;
        // 初始化突变次数
        boundary_change_count = 0;
        // 记录第一个突变点
        for (int y = image_h-60; y > Endline+30; y--) {
            // 检测左边界突变
            if (abs(left_cicle[y] - left_cicle[y-3]) > 25) {
                first_boundary_change_x = (left_cicle[y] > left_cicle[y-3]) ? left_cicle[y] : left_cicle[y-3];  // 取边界大的坐标
                first_boundary_change_y = y;
                first_boundary_change_recorded = 1;  
                boundary_change_count++;  // 突变次数加1
                //break;
            }
            // 检测右边界突变
            if (abs(right_cicle[y] - right_cicle[y-3]) > 20) {
                first_boundary_change_x = (right_cicle[y] < right_cicle[y-3]) ? right_cicle[y] : right_cicle[y-3];  // 取边界小的坐标
                first_boundary_change_y = y;
                first_boundary_change_recorded = 1;
                boundary_change_count++;  // 突变次数加1
                //break;
            }
        }
        // 记录最后一个突变点
        for (int y = Endline + 60; y < image_h; y++) {
            // 检测左边界突变
            if (abs(left_cicle[y] - left_cicle[y-8]) > 55) {
                last_boundary_change_x = (left_cicle[y] > left_cicle[y-8]) ? left_cicle[y] : left_cicle[y-8];  // 取边界大的坐标
                last_boundary_change_y = y;
                boundary_change_count++;  // 突变次数加1
            }
            // 检测右边界突变
            if (abs(right_cicle[y] - right_cicle[y-8]) > 55) {
                last_boundary_change_x = (right_cicle[y] < right_cicle[y-8]) ? right_cicle[y] : right_cicle[y-8];  // 取边界小的坐标
                last_boundary_change_y = y;
                boundary_change_count++;  // 突变次数加1
            }
            if(left_cicle[118]>50&&left[118]-left[58]>30){
                last_boundary_change_x=left_cicle[118];
                last_boundary_change_y=118;
                boundary_change_count++;  // 突变次数加1
            }
            else if(right_cicle[118]<110&&right[58]-right[118]>30){
                last_boundary_change_x=right_cicle[118];
                last_boundary_change_y=118;
                boundary_change_count++;  // 突变次数加1
            }
            else {
                last_boundary_change_x=(right_cicle[118]+left_cicle[118])/2;
                last_boundary_change_y=118;
                boundary_change_count++;  // 突变次数加1
            }
        }        
        
        // 根据找到的边界变化点情况进行不同的补线策略
        if ((first_boundary_change_x == -1 || first_boundary_change_y == -1) && 
            (last_boundary_change_x == -1 || last_boundary_change_y == -1)) {
            // 如果上下都没找到，使用两个中点补线
            Addingline( 3, (right[50]+left[50])/2, 50, (right[118]+left[118])/2, 118 );
        } 
        else if (first_boundary_change_x == -1 || first_boundary_change_y == -1) {
            // 如果上面没找到，下面找到了，使用中点到下边界点补线
            Addingline( 3, (right[Endline+20]+left[Endline+20])/2, Endline+20, last_boundary_change_x, last_boundary_change_y );
        } 
        else {
            // 如果上下都找到了，使用原来的Addingline
            Addingline( 3, first_boundary_change_x, first_boundary_change_y, last_boundary_change_x, last_boundary_change_y );
        }
        
        // 检查两腿高度差是否在等高范围内（阈值为0.2）
        float height_diff = ServoPID.highleft - ServoPID.highright;
        if (abs(height_diff) <= 0.4f) {
            legs_equal_height_counter++;
        } else {
            legs_equal_height_counter = 0; // 重置计数器
        }
        
        // 当没有边界宽度变化、没有桥梁特征、没有边界位置突变时，或者两腿等高超过2秒时，认为桥梁结束
        if( legs_equal_height_counter >= 100)
        {
            //bridge_out_flag=1;
            legs_equal_height_counter = 0; // 重置计数器
        }
    }
}
/**
* @brief 弯道检测与填充函数
* @param uint8(*image)[image_w]     灰度图像数组
* @param uint8 *l_border            左边缘数组
* @param uint8 *r_border            右边缘数组
* @param uint16 total_num_l         左边缘点总数
* @param uint16 total_num_r         右边缘点总数
* @param uint16 *dir_l              左边缘方向数组
* @param uint16 *dir_r              右边缘方向数组
* @param uint16(*points_l)[2]       左边缘点坐标数组
* @param uint16(*points_r)[2]       右边缘点坐标数组
* @param uint8* hightest            最高有效行指针
* @param uint16 *l_index            左边缘索引数组
* @param uint16 *r_index            右边缘索引数组
* @param int monotonicity_l         左边缘单调性
* @param int monotonicity_r         右边缘单调性
* @return 无返回值，结果存储在全局变量中
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */

void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
   uint16 i;
   int black_line_sum = 0; // 黑色线条计数
   int black_sum_1 = 0;// 灰度值状态标记
   int black_sum_2 = 0;
   uint8 break_num_l = 0;// 左右边缘突变点行号
   uint8 break_num_r = 0;
   uint8 end_num_l = 0;// 左右边缘恢复点行号
   uint8 end_num_r = 0;
   uint8 start, end;  // 拟合区域起止行号
   int ap = 1;  // 弯道有效性标记
   float slope_l_rate = 0, intercept_l = 0; // 直线拟合参数
  // 检测左弯道（左边缘非单调，右边缘单调）
   if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
   {
         // 检测左边缘断线位置
        broken_line_judge(1,*hightest,110,l_border);
        // 检查右边缘是否向内侧弯曲（左弯特征）
        for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
        {
            if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
            {
                ap = 0;// 标记为无效弯道
                break;
            }
        }
         // 统计图像中的黑色线条（可能为赛道线）
        for (i = broken_line_y+5; i > *hightest; i--)
        {
            for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
            {
                // 检测黑色线条起始
                if (image[i][x] == 0 && image[i][x-1] == 255)
                {
                    black_sum_1 = 1;
                }
                // 检测黑色线条结束
                if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
                {
                    black_sum_2 = 1;
                }
                 // 重置状态标记
                if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
                {
                    black_sum_1 = 0;
                    black_sum_2 = 0;
                    break;
                }
            }
            // 累计有效黑色线条数
            if(black_sum_2 == 1)
            {
                black_sum_1 = 0;
                black_sum_2 = 0;
                black_line_sum++;
            }
            if(i==15)// 仅检查前15行
                break;
        }
        // 综合判断是否为左弯道
        if(black_line_sum>=5 /*确认特征：识别弯道*/
                && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
                /*特殊条件：边缘丢失判断*/
                && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
            /*附加条件：十字符和桥接不冲突*/
        {
                sum_island = 1;// 标记为疑似弯道
                island = 1;// 标记为左弯道
                black_line_sum = 0;
        }
    }
    // 检测右弯道（右边缘非单调，左边缘单调）
    if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
    {
        // 检测右边缘断线位置
        broken_line_judge(1,*hightest,110,r_border);
        // 检查左边缘是否向内侧弯曲（右弯特征）
        for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
        {
            if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || l_border[w] != border_min))
            {
                ap = 0;// 标记为无效弯道
                break;
            }
        }
        // 统计图像中的黑色线条（逻辑与左弯相同）
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
            if(i==15)//???п????????
                break;
        }
        // 综合判断是否为右弯道（逻辑与左弯相同）
        if(black_line_sum>=5 /*确认特征：识别弯道*/
                && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*特殊条件：边缘丢失判断*/
                && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
            /*附加条件：十字符和桥接不冲突*/
        {
            sum_island = 1;  // 标记为疑似弯道
            island = 2;      // 标记为右弯道
            black_line_sum = 0;
        }
    }
    // 处理左弯道（island=1）
    if(island == 1)
    {
         // 确认左弯道（sum_island=1）
        
        if(sum_island == 1)
        {
            // 重新检测左边缘断线位置
            broken_line_judge(1,*hightest,110,l_border);
            // 检查右边缘是否持续向内侧弯曲
            for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
            {
                if (r_border[w - 10] > r_border[w] && (r_border[w - 10] != border_max || r_border[w] != border_max))
                {
                    ap = 0;// 弯道无效，退出状态
                    break;
                }
            }
             // 若弯道无效，重置状态
            if(ap == 0)
            {
                island = 0;
                sum_island = 0;
            }
            // 若弯道有效且断线位置合适，修复左边缘
            if(broken_line_y >= 20 && ap == 1)
            {
                // 最小二乘法拟合左边缘
                start = broken_line_y+5;
                end = broken_line_y+10;
                calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
                // 用拟合直线修复左边缘
                for (i = 1; i < broken_line_y+1; i++)
                {
                    l_border[i] = slope_l_rate * (i)+intercept_l;
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }
                // 检查是否需要进一步处理
                if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
                {
                    // 通过边缘点确认弯道状态
                    for (i = 106; i > 15; i--)
                    {
                        
                        if(points_l[l_index[i]][0]>points_l[l_index[i-5]][0] && points_l[l_index[i]][0]>points_l[l_index[i+5]][0]
                            && points_l[l_index[i-5]][0] != border_min)
                        {
                            sum_island = 2;// 升级状态为确认弯道
                        }
                    }
                }
            }
        }
        // 确认左弯道（sum_island=2），进一步处理
        if(sum_island == 2)
        {
            int dp = 0;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            // 检查图像底部像素（可能为赛道特征）
            if (!image[image_h - 5][5] && !image[image_h - 3][3])
            {
                dp = 1;
            }
            // 检测边缘突变位置
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
            // 确认边缘突变点
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
             // 寻找边缘最低点
            for (i = 25; i < image_h - 15; i++)
            {
                if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
                   && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
                   && l_border[i-5] != border_min && l_border[i+5] != border_min)
                {
                    end_num_l = (uint8)i;
                }
                // 状态升级为准备拟合
                if(vp && end_num_l >= 80)
                    sum_island = 3;
            }
                // 拟合右边缘（假设弯道结束后边缘恢复）
                slope_l_rate = (float)(118-end_num_l) / ((border_max-r_border[118]+border_min)-l_border[end_num_l]);//б??k=y/x
                intercept_l = 118 - slope_l_rate*(border_max-r_border[118]+border_min);//???b=y-kx
                for (i = end_num_l; i < image_h - 1; i++)
                {
                    l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }

        }
        //初步确认左弯道
        if(sum_island == 3)
        {
            uint16 h = 0;
            int temph_l = 0;
            // 1. 寻找左边缘突变点（弯道起始位置）
            for (h = image_h - 15; h > 5; h--)// 从图像底部向上扫描
            {
                if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    temph_l = h;  // 记录突变行号
                    break;  // 找到后立即退出循环
                }
            }
             // 2. 在突变点附近寻找最佳转折点
            if (temph_l)// 如果找到了突变点
            {
                for (int i = total_num_l - 10; i > l_index[h + 1]; i--)
                {
                    // 寻找y坐标局部最大值（弯道顶点特征）
                    if (points_l[i][1] >= points_l[i + 3][1]  // 当前点y坐标≥后3个点
                        && points_l[i][1] > points_l[i + 5][1]  // 当前点y坐标>后5个点
                        && points_l[i][1] >= points_l[i - 3][1]  // 当前点y坐标≥前3个点
                        && points_l[i][1] >= points_l[i - 5][1]  // 当前点y坐标≥前5个点
                        // 位置合理性约束
                        && points_l[i][0] > points_l[i - 5][0]  // 当前点x坐标>前5个点
                        && points_l[i][0] <= points_l[i + 5][0])  // 当前点x坐标≤后5个点
                    {
                        break_num_l = (uint8)points_l[i][1];  // 记录转折点y坐标
                        end_num_l  = (uint8)points_l[i][0];  // 记录转折点x坐标
                        break;  // 找到后立即退出循环
                    }
                }
            }
            //使用更大范围的比较
//            for (i = 40; i < total_num_l-30; i++)
//            {
//                if (points_l[i][1]>points_l[i+9][1]&&points_l[i][1]>points_l[i-9][1]
//                      &&points_l[i][1]>points_l[i+15][1]&&points_l[i][1]>points_l[i-15][1])
//                  {
//                     break_num_l = (uint8)points_l[i][1];//????y????
//                     end_num_l  = (uint8)points_l[i][0];//????x????
//                  }
//            }

            // 4. 检查图像底部边缘特征（确认是否进入弯道）
            if((*hightest >= 30)&&  // 最高点位置合理（确保不是噪声）
              (!image[image_h - 1][3] && !image[image_h - 3][3] // 图像左下角为黑色（赛道边界）
                && !image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))// 图像右下角为黑色
            {
                sum_island = 4;//满足条件则进入状态4
            }

            // 5. 更新右边界（基于转折点计算直线方程）
            if(break_num_l && end_num_l)  // 如果找到了转折点
            {
                // 计算斜率k = Δy/Δx（以(186,118)为参考点）
                slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);
                intercept_l = 118 - slope_l_rate*186;// 计算截距b = y - kx
                 // 根据直线方程更新右边界x = (y - b)/k
                for (i = 1; i < image_h - 1; i++)
                {
                    r_border[i] = ((i)-intercept_l)/slope_l_rate;// 计算每个y对应的x坐标
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max); // 限制在有效范围内
                }
            }
        }
        // 当前处于状态4：验证左弯道特征
        if(sum_island == 4)
        {
             int g=0;  // 方向序列匹配标志
    
             // 1. 寻找特定方向序列（4→4→6→6→6，表示向左转）
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    g=1;
                    break;
                }
            }
            // 2. 检测右边界突变（曲率变化点）
            for (i = image_h - 20; i > *hightest; i--)//?????б?
            {
                if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>30)
                {
                    if(i>=30 && i<=105 && g)
                        sum_island = 5;
                }
            }
        }
        // 当前处于状态5：计算弯道曲率
        if(sum_island == 5)
        {
            // 1. 寻找右边界突变点（曲率最大处）
            for (uint16 w = image_h - 15; w > *hightest; w--)//?????б?
            {
                if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
                {
                    break_num_l = (uint8)w;//????y????
                    break;
                }
            }
            // 2. 再次确认方向序列特征（同状态4）
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    end_num_l = (uint8)points_l[i][1];//????y????
                    break;
                }
            }
            // 3. 检查右边界是否存在（未丢失）
            if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
                sum_island = 6;
            // 4. 更新右边界（基于新的曲率计算直线方程）
            slope_l_rate = (float)(break_num_l-end_num_l) / (r_border[break_num_l]-l_border[end_num_l]);//б??k=y/x
            intercept_l = 0 - slope_l_rate*0;//计算新截距b=y-kx
             // 根据新直线方程更新右边界（仅更新到突变点下方）
            for (i = 1; i < break_num_l-3; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        // 弯道过渡状态
        if(sum_island == 6)
        {
            int dp = 1;// 控制是否执行下面的循环（固定为1，表示执行）
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
             // 1. 寻找左边缘突变点（弯道结束特征）
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
            // 2. 在突变点附近寻找特征点（y坐标局部最大值）
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]  // 当前点y坐标≥后3个点
                        && points_l[j][1] > points_l[j + 5][1]  // 当前点y坐标>后5个点
                        && points_l[j][1] >= points_l[j - 3][1]  // 当前点y坐标≥前3个点
                        && points_l[j][1] >= points_l[j - 5][1])  // 当前点y坐标≥前5个点
                    {
                        vp = h;
                        break;
                    }
                }
            }
             // 3. 状态转换条件（特征点存在且右边界特定区域丢失）
            if(vp && r_loss_judge(r_border, 70 ,90) == 0// 右边界在70-90行丢失
                    && r_loss_judge(r_border, 50 ,70) == 0)// 右边界在50-70行丢失
                sum_island = 7;
            // 4. 重置右边界（恢复默认直线
            slope_l_rate = (float)(118-0) / (186-0);// 计算默认斜率k=y/x
            intercept_l = 0 - slope_l_rate*0;//计算默认截距b=y-kx
            // 根据默认直线方程更新右边界
            for (i = 1; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        // 弯道结束判断状态
        if(sum_island == 7)
        {
            uint16 h = 0;
            int temph = 0;
            // 1. 寻找左边缘突变点（弯道真正结束）
            for (h = image_h - 25; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && abs(l_border[h] - l_border[h + 1]) > 20)
                {
                    temph = h;  // 记录突变行号
                    break_num_l = (uint8)h;  // 记录转折点y坐标
                    end_num_l  = (uint8)l_border[h];  // 记录转折点x坐标
                    break;  // 找到后立即退出循环
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
            // 3. 检测图像底部边缘特征（确认是否回到直道）
            if (!image[image_h - 5][image_w - 3] && !image[image_h - 3][image_w - 3]  // 右下角为黑色
                && !image[image_h - 7][image_w - 3]  // 右下角上方为黑色
                && !image[image_h - 5][3] && !image[image_h - 3][3]  // 左下角为黑色
                && !image[image_h - 7][3])  // 左下角上方为黑色
            {
                if(r_loss_judge(r_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0 // 右边界特定区域丢失
                   && l_loss_judge(l_border, 90 ,110) == 0 && l_loss_judge(l_border, 70 ,90) == 0) // 左边界特定区域丢失
                {
                    sum_island = 0;
                    island = 0;// 清除左弯道标记
                }
            }
            // 4. 更新左边界（基于新的斜率，处理双车道边界）
            slope_l_rate = (float)(118-end_num_l) / (2-l_border[end_num_l]);//计算新斜率k=y/x
            intercept_l = 118 - slope_l_rate*2;//计算新截距b=y-kx
            // 根据新直线方程更新左边界（仅更新转折点下方区域）
            for (i = end_num_l-10; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
    }
     // 当前识别为右弯道（island=2）
    if(island == 2)
    {
        // 处于状态1：疑似右弯道
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,r_border); // 检测右边界断线
            // 验证左边界连续性（防止误判为弯道）
            for (uint16 w = broken_line_y + 5; w > broken_line_y - 15; w--)
            {
                if (l_border[w - 10] < l_border[w] && (l_border[w - 10] != border_min || r_border[w] != border_min))
                {
                         ap = 0;// 标记为非连续边界
                         break;
                }
            }
            // 边界不连续，取消弯道标记
            if(ap == 0)
             {
                 island = 0;
                 sum_island = 0;
             }
             // 断线位置合理且边界连续时更新右边界
            if(broken_line_y >= 20 && ap == 1)
            {
                start = broken_line_y+5;  // 拟合起始行
                end = broken_line_y+10;    // 拟合结束行
                calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
                // 根据直线方程更新右边界
                for (i = 1; i < broken_line_y+1; i++)
                {
                    r_border[i] = slope_l_rate * (i)+intercept_l;
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                }
                 // 检测右边界突变或丢失，进入下一状态
                if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
                {
                    for (i = 106; i > 15; i--)
                    {
                        // 寻找右边界局部最小值（弯道特征）
                        if(points_r[r_index[i]][0]<points_r[r_index[i-5]][0] && points_r[r_index[i]][0]<points_r[r_index[i+5]][0]
                            && points_r[r_index[i-5]][0] != border_max)
                        {
                            sum_island = 2;
                        }
                    }
                }
            }
        }
     // 状态2：确认右弯道特征
    if(sum_island == 2)
    {
        int dp = 0;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
         // 检测图像底部特征点（右侧边缘）
        if (!image[image_h - 5][183] && !image[image_h - 3][185])
        {
            dp = 1;// 标记为有效检测区域
        }
         // 寻找右边缘突变点（弯道起始）
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
        // 寻找右边缘特征点（局部最小值）
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
         // 寻找右边界局部最小值（曲率最大点）
        for (i = 25; i < image_h - 15; i++)
        {
            if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
               && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
               && r_border[i-5] != border_max && r_border[i+5] != border_max)
            {
                end_num_r = (uint8)i;
            }
            // 特征点存在且位置合理时进入状态3
            if(vp && end_num_r >= 80)
                sum_island = 3;
        }
             // 更新右边界（基于曲率计算）
            slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б??k=y/x
            intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
            for (i = end_num_r-10; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
    }
    // 状态3：计算右弯道参数
    if(sum_island == 3)
    {
        uint16 h = 0;
        int temph_l = 0;
        // 寻找右边缘突变点（细化位置）
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 10)
            {
                temph_l = h-5;
                break;
            }
        }
         // 寻找右边缘特征点（局部最小值）
        if (temph_l)
        {
            for (int i = total_num_r - 10; i > r_index[h + 1]; i--)
            {

                if (points_r[i][1] >= points_r[i + 3][1]// y坐标局部最大
                    &&points_r[i][1] > points_r[i + 5][1]
                    &&points_r[i][1] >= points_r[i - 3][1]
                    &&points_r[i][1] >= points_r[i - 5][1]
                    &&points_r[i][0] < points_r[i - 5][0]
                    &&points_r[i][0] >= points_r[i + 5][0])
                {
                    break_num_r = (uint8)points_r[i][1];// 记录特征点y坐标
                    end_num_r  = (uint8)points_r[i][0];// 记录特征点x坐标
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
        // 最高点位置合理时进入状态4
        if(*hightest >= 30)
        {
            sum_island = 4;
        }
        // 更新左边界（基于右弯道特征点）
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
    // 状态4：检测左边界突变
    if(sum_island == 4)
    {
        for (i = image_h - 15; i > *hightest; i--)//?????б?
        {
            if (l_border[i] > l_border[i - 3] && (l_border[i] - l_border[i-3])>20)
            {
                if(i>=30 && i<=105)
                  sum_island = 5;
            }
        }
    }
     // 状态5：计算左边界参数
    if(sum_island == 5)
    {
        for (uint16 w = image_h - 15; w > *hightest; w--)//?????б?
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
        // 右边界存在时进入状态6
        if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
            sum_island = 6;
        // 更新左边界（基于突变点
        end_num_r = l_border[break_num_r];
        slope_l_rate = (float)(break_num_r-0) / (end_num_r-188);//б??k=y/x
        intercept_l = 0 - slope_l_rate*188;//b=y-kx
        for (i = 1; i < break_num_r-3; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
    // 状态6：右弯道过渡
    if(sum_island == 6)
    {
        int dp = 1;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (dp)
        {
            // 寻找右边缘突变点（弯道结束特征）
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
                {
                    temph = h;
                    break;
                }
            }
        }
         // 寻找右边缘特征点（局部最小值）
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
        // 边界丢失时进入状态7（弯道结束判断）
        if(vp && l_loss_judge(r_border, 90 ,110) == 0 && l_loss_judge(r_border, 70 ,90) == 0
                && l_loss_judge(r_border, 50 ,70) == 0)
            sum_island = 7;
         // 重置左边界（恢复默认直线）
        slope_l_rate = (float)(118-0) / (0-188);//k=y/x
        intercept_l = 0 - slope_l_rate*188;//b=y-kx
        for (i = 1; i < image_h - 1; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
     // 状态7：右弯道结束判断
    if(sum_island == 7)
    {
        uint16 h = 0;
        int temph = 0;
        // 寻找右边缘突变点（弯道结束
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && abs(r_border[h] - r_border[h + 1]) > 20)
            {
                temph = h;
                break;
            }
        }
        // 寻找右边缘特征点（局部最小值）
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
        // 底部边缘特征满足时回到初始状态
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
         // 更新右边界（结束弯道处理）
        slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б??k=y/x
        intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);//???b=y-kx
        for (i = end_num_l-10; i < image_h - 1; i++)
        {
            r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
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
        }



//===================================================元素识别===================================================
void Element_recognition(void)
{

    inflection_point();//拐点总判断
    left_straight();//左直线
    right_straight();//右直线
    monotonicity_line((uint8*)&Endline,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
    crossroad();
    annulus_L();//左圆环
    annulus_R();//右圆环
    middle_line();//中线
    //bridge_fill(imag,left, right, middle, (uint8*)&Endline);//单边桥

    //jump_judge(imag, (uint8*)&Endline,(uint8*)left, (uint8*)right, (int)Left_straight_flag, (int)Right_straight_flag);//跳跃
    //zebra_crossing(imag,left,right);//斑马线
    //around_fill( imag,left, right, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, (uint8*)&Endline,l_index,r_index,monotonicity_l, monotonicity_r);

}

//===================================================图像处理===================================================

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
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
        {
            
        }
}
