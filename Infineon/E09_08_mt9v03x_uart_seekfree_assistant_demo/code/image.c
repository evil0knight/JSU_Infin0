#include "image(3).h"
#include "zf_common_headfile.h"

float border = 96;        // 边界阈值,图像偏差
float border_last = 96;        // 边界阈值,上一次图像偏差
int buzzer = 0;           // 蜂鸣器状态
int cross_sum = 0;        // 交叉点计数
int sum_island = 0;       // 岛屿总数
int island=0;             // 岛屿标识(1为左环岛，2为右环岛)
float turn_value=0;       // 转角值，由border在中断中计算得出

// 单边桥状态变量
SingleBridgeState BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;
int jump_position = 0;    // 跳跃位置
int stop_position = 0;    // 停止位置
int monotonicity_line_l =0;  // 左侧单调线
int monotonicity_line_r =0;  // 右侧单调线
int string_not = 0;       // 字符串标志
int m=0;                  // 通用计数器
float points_x;           // 点坐标x
float points_y;           // 点坐标y

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
 /*
函数名称：int my_limit(int num,int value)
功能说明：下限归零
参数说明：
返回值：限制值
修改时间：2025年3月14日
备注：
示例：my_abslimit(num,value)
*/
int my_limit(int num,int value)
{
    if(value < num)
        return 0;     // 当value小于num时返回0
    else
        return value;  // 否则返回原值
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
int limit_a_b(int x, int a, int b)
{
    if(x<a) x = a;    // 小于下限时取a
    if(x>b) x = b;    // 大于上限时取b
    return x;         // 返回限制后的值
}
/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：限制x在±y范围内
参数说明：
返回值：限制后的值
修改时间：2025年1月5日
备注：
示例：limit(x, y)
*/
int16 limit1(int16 x, int16 y)
{
    if (x > y)       return y;    // 超过正限幅
    else if (x < -y) return -y;   // 超过负限幅
    else             return x;    // 正常范围
}



uint8 original_image[image_h][image_w];  // 原始图像数组
uint8 image_thereshold;                 // 图像二值化阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief
//  @since      v1.0
//------------------------------------------------------------------------------------------------------------------
// 获取灰度图像
void Get_image(uint8(*mt9v03x_image)[image_w]) {
#define use_num 1  // 1表示不压缩，2表示压缩一倍

    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num) {
        for (j = 0; j <image_w; j += use_num) {
            original_image[row][line] = mt9v03x_image[i][j];  // 将摄像头采集的图像存入原始图像数组
            line++;
        }
        line = 0;
        row++;
    }
}


/**
 * @brief 使用大津法计算图像二值化阈值
 * @param image 输入图像数据指针
 * @param col 图像宽度
 * @param row 图像高度
 * @return 计算得到的最佳阈值
 * @note 该方法通过最大化类间方差自动确定最佳阈值
 */
// 使用大津法计算图像二值化阈值
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row) {
#define GrayScale 256
    uint16 Image_Width = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};  // 灰度直方图

    uint32 Amount = 0;              // 总像素数
    uint32 PixelBack = 0;           // 背景像素数
    uint32 PixelIntegralBack = 0;   // 背景积分
    uint32 PixelIntegral = 0;       // 图像积分
    int32 PixelIntegralFore = 0;    // 前景积分
    int32 PixelFore = 0;            // 前景像素数
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0;
    uint8 MinValue=0, MaxValue=0;   // 最小和最大灰度值
    uint8 Threshold = 0;            // 最终阈值

    // 统计灰度直方图
    for (Y = 0; Y <Image_Height; Y++) {
        for (X = 0; X < Image_Width; X++) {
            HistGram[(int)data[Y*Image_Width + X]]++;
        }
    }

    // 找出最小和最大灰度值
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++);
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--);

    if (MaxValue == MinValue) return MaxValue;  // 图像只有一种颜色
    if (MinValue + 1 == MaxValue) return MinValue; // 图像只有两种颜色

    // 计算总像素数
    for (Y = MinValue; Y <= MaxValue; Y++) {
        Amount += HistGram[Y];
    }

    // 计算灰度积分
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++) {
        PixelIntegral += HistGram[Y] * Y;
    }

    // 大津法计算最佳阈值
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++) {
        PixelBack += HistGram[Y];    // 背景像素数
        PixelFore = Amount - PixelBack; // 前景像素数
        OmegaBack = (double)PixelBack / Amount; // 背景比例
        OmegaFore = (double)PixelFore / Amount; // 前景比例
        PixelIntegralBack += HistGram[Y] * Y;  // 背景积分
        PixelIntegralFore = PixelIntegral - PixelIntegralBack; // 前景积分
        MicroBack = (double)PixelIntegralBack / PixelBack; // 背景平均灰度
        MicroFore = (double)PixelIntegralFore / PixelFore; // 前景平均灰度
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // 类间方差
        if (Sigma > SigmaB) {  // 寻找最大类间方差
            SigmaB = Sigma;
            Threshold = (uint8)Y;
        }
    }
    return Threshold;
}

/**
 * @brief 图像二值化处理
 * @note 使用Otsu算法计算的阈值对图像进行二值化
 */
uint8 bin_image[image_h][image_w]; // 二值化图像存储数组

void turn_to_bin(void) {
    uint8 i, j;

    // 1. 计算Otsu阈值
    image_thereshold = otsuThreshold(original_image[0], image_w, image_h);

    // 2. 执行二值化
    for(i = 0; i < image_h; i++) {
        for(j = 0; j < image_w; j++) {
            if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
            else bin_image[i][j] = black_pixel;
        }
    }
}

/*
函数名称：void get_start_point(uint8 start_row)
函数说明：寻找赛道边界的边界点作为后续循环起始点
输入说明：起始行数
输出说明：无
返回值：找到返回1，未找到返回0
修改时间：2025年1月5日
注  释：
示例：get_start_point(image_h-2)
*/
uint8 start_point_l[2] = { 0 }; // 左边界的x和y值
uint8 start_point_r[2] = { 0 }; // 右边界的x和y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    // 初始化边界点坐标
    start_point_l[0] = 0; // x
    start_point_l[1] = 0; // y
    start_point_r[0] = 0; // x
    start_point_r[1] = 0; // y

     // 从中间向左侧搜索左边界
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }
    // 从中间向右侧搜索右边界
    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found && r_found) return 1; // 左右边界都找到
    else if(l_found == 0 && r_found) // 只找到右边界
    {
        // 在右侧3/4区域重新搜索左边界
        for (i = image_w * 3 / 4; i > border_min; i--)
        {
            start_point_l[0] = i;//x
            start_point_l[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
            {
                //printf("image[%d][%d]\n", start_row,i);
                l_found = 1;
                break;
            }
        }

        if(l_found&&r_found)return 1;
        else return 0;
    }
    else if(l_found && r_found == 0) // 只找到左边界
    {
        // 在左侧1/4区域重新搜索右边界
        for (i = image_w / 4; i < border_max; i++)
        {
            start_point_r[0] = i;//x
            start_point_r[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
            {
                //printf("image[%d][%d]\n",start_row, i);
                r_found = 1;
                break;
            }
        }
        if(l_found&&r_found)return 1;
        else return 0;
    }
    else {
        // 左右边界都没找到
        //printf("\n");
        return 0;
    }
}


/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                        uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

函数说明：以种子填充方式开始搜索左右边界的连续点，搜索时要漏判，记录最后的一个坐标点。
输入说明：
break_flag_r            循环需要循环的次数
(*image)[image_w]       需要被搜索的图像数组，一般是二值图,可以是其他图像处理
                       特别注意，不要用宏定义宽度，因为这样传递的数据可能无法递归
*l_stastic             左统计的数据，用于记录起始点以便后续信号获取循环次数
*r_stastic             右统计的数据，用于记录起始点以便后续信号获取循环次数
l_start_x              左边起始点x坐标
l_start_y              左边起始点y坐标
r_start_x              右边起始点x坐标
r_start_y              右边起始点y坐标
hightest               在循环过程中得到的最高高度
输出说明：无
返回值：无
修改时间：2025年1月5日
注  释：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
*/
#define USE_num image_h*3   //最大搜索点数，例如300个点可能放不下，这些参数需要根据实际情况调整

//存储点的x和y坐标
uint16 points_l[(uint16)USE_num][2] = { { 0 } }; //左边点集
uint16 points_r[(uint16)USE_num][2] = { { 0 } }; //右边点集
uint16 dir_r[(uint16)USE_num] = { 0 }; //数组存储右边搜索方向
uint16 dir_l[(uint16)USE_num] = { 0 }; //数组存储左边搜索方向
uint16 data_stastics_l = 0; //统计左边找到的点的个数
uint16 data_stastics_r = 0; //统计右边找到的点的个数
uint8 hightest = 0; //最高点y坐标
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{


    uint8 i = 0, j = 0;

    //左边相关变量
    uint8 search_filds_l[8][2] = { { 0 } }; //8邻域搜索区域
    uint8 index_l = 0; //临时索引
    uint8 temp_l[8][2] = { { 0 } }; //临时存储点
    uint8 center_point_l[2] = { 0 }; //当前中心点
    uint16 l_data_statics; //左边点计数器

    //左边8邻域搜索方向(顺时针)
    static int8 seeds_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //

    //右边相关变量
    uint8 search_filds_r[8][2] = { { 0 } }; //8邻域搜索区域
    uint8 center_point_r[2] = { 0 }; //当前中心点
    uint8 index_r = 0; //临时索引
    uint8 temp_r[8][2] = { { 0 } }; //临时存储点
    uint16 r_data_statics; //右边点计数器

    //右边8邻域搜索方向(逆时针)
    static int8 seeds_r[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1} };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //

    l_data_statics = *l_stastic; //初始化左边点计数器
    r_data_statics = *r_stastic; //初始化右边点计数器

    //设置初始中心点
    center_point_l[0] = l_start_x; //左边起始x
    center_point_l[1] = l_start_y; //左边起始y
    center_point_r[0] = r_start_x; //右边起始x
    center_point_r[1] = r_start_y; //右边起始y
        //
    while (break_flag--)
    {
        //左边处理
        //计算8邻域坐标
        for (i = 0; i < 8; i++) {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0]; //x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1]; //y
        }

        //存储当前点
        points_l[l_data_statics][0] = center_point_l[0]; //x
        points_l[l_data_statics][1] = center_point_l[1]; //y
        l_data_statics++; //计数器增加

        //右边处理
        for (i = 0; i < 8; i++) {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0]; //x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1]; //y
        }

        //存储当前点
        points_r[r_data_statics][0] = center_point_r[0]; //x
        points_r[r_data_statics][1] = center_point_r[1]; //y

        index_l = 0;//
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//
            temp_l[i][1] = 0;//
        }

    //左边边界判断
        for (i = 0; i < 8; i++) {
            //判断边界条件：当前点为黑(0)且下一个点为白(255)
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255) {
                //记录边界点
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i); //记录搜索方向
            }
            if (index_l)
            {
                 //选择下一个中心点(选择y坐标最小的点)
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        //终止条件1:连续三个点相同
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("\n");
            break;
        }

        //终止条件2:左右点距离过近
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//
            //printf("\n\n",*hightest);
            break;
        }

         //其他特殊情况处理
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
//            printf("\n\n");
            continue; //右边比左边高，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//
        {
            //左边已经循环一周，等待右边
            //printf("\n \n");
            center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//

        index_r = 0;//
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//
            temp_r[i][1] = 0;//
        }

        //右边边界判断
        for (i = 0; i < 8; i++)
        {
            //判断边界条件：当前点为黑(0)且下一个点为白(255)
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//
                dir_r[r_data_statics - 1] = (i);//
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }

            //选择下一个中心点(选择y坐标最小的点)
            if (index_r)
            {

                //
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }
    //更新计数器
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}

/*
函数名称：void get_left(uint16 total_L)
函数说明：从左边点集中提取需要的边界
输入说明：
total_L  找到的点总数
输出说明：无
返回值：无
修改时间：2025年1月5日
注  释：
example： get_left(data_stastics_l);
*/
uint8 l_border[image_h];  // 左边界数组，存储每行的左边界x坐标
uint8 r_border[image_h];  // 右边界数组，存储每行的右边界x坐标
uint16 l_index[image_h];  // 左边界点在原始点集中的索引
uint16 r_index[image_h];  // 右边界点在原始点集中的索引
uint8 center_line[image_h]; // 中心线数组

// 只使用X边界
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    // 初始化左边界数组
    for (i = 0; i < image_h; i++)
    {
        l_border[i] = border_min;  // 初始化为最小边界值
        l_index[i] = 0;            // 初始化索引为0
    }

    h = image_h - 2;  // 从倒数第二行开始

    // 遍历所有左边点
    for (j = 0; j < total_L; j++)
    {    //printf("%d\n", j);
        // 如果当前点的y坐标等于当前处理行
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0] + 1;  // 记录x坐标(加1作为边界)
            l_index[h] = j;                    // 记录点在原始数组中的索引
        }
        else continue;  // 每行只取一个点，不是当前行就跳过

        h--;  // 处理上一行
        if (h == 0)
        {
            break;  // 处理完第一行就退出
        }
    }
}



/*
函数名称：void get_right(uint16 total_R)
函数说明：从右边点集中提取需要的边界
输入说明：
total_R  找到的点总数
输出说明：无
返回值：无
修改时间：2025年1月5日
注  释：
example：get_right(data_stastics_r);
*/
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
 // 初始化右边界数组
    for (i = 0; i < image_h; i++)
    {
        r_index[i] = 0;
        r_border[i] = border_max; // 初始化为最大边界值
    }
    h = image_h - 2; // 从倒数第二行开始

    //遍历所有右边点
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;  // 记录x坐标(减1作为边界)
            r_index[h] = j;                   // 记录点在原始数组中的索引
        }
        else continue;  // 每行只取一个点，不是当前行就跳过

        h--;  // 处理上一行
        if (h == 0) break;  // 处理完第一行就退出
    }
}



#define threshold_max   255*5 // 膨胀阈值，可根据实际情况调整
#define threshold_min   255*2 // 腐蚀阈值，可根据实际情况调整
void image_filter(uint8(*bin_image)[image_w])// 形态学滤波函数，结合了膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    // 遍历图像(避开边缘像素)
    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            // 计算8邻域像素值总和
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];

            // 膨胀条件：周围白色像素多且当前点为黑
            if (num >= threshold_max && bin_image[i][j] == 0)
            {
                bin_image[i][j] = 255; // 将黑点变白(膨胀)

            }
            // 腐蚀条件：周围白色像素少且当前点为白
            if (num <= threshold_min && bin_image[i][j] == 255)
            {
                bin_image[i][j] = 0; // 将白点变黑(腐蚀)
            }

        }
    }

}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
函数说明：给图像绘制一个边框
输入说明：uint8(*image)[image_w] 图像首地址
输出说明：无
返回值：无
修改时间：2025年1月5日
注  释：
example： image_draw_rectan(bin_image);
*/
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;

    // 绘制左右两侧的垂直边框(两像素宽)
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;            // 最左侧第一列像素设为黑
        image[i][1] = 0;            // 左侧第二列像素设为黑
        image[i][image_w - 1] = 0;  // 最右侧第一列像素设为黑
        image[i][image_w - 2] = 0;  // 右侧第二列像素设为黑
    }
    // 绘制顶部的水平边框(两像素宽)
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;  // 顶部第一行像素设为黑
        image[1][i] = 0;  // 顶部第二行像素设为黑
        //image[image_h-1][i] = 0;  // 注释掉的底部边框绘制
    }
}


/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能描述：从指定起始点开始，同时向左右两个方向搜索图像中的边缘，并记录边缘点的相关信息。搜索过程中需要考虑各种边界情况，确保算法的鲁棒性。

参数说明：
break_flag_r            需要循环的最大次数
(*image)[image_w]       需要搜索边缘的图像数据，这里是二值化图像，函数内部会进行边界检查
                       注意：需要使用预定义的常量作为图像宽度，以确保数据访问的安全性
*l_stastic              统计左边边缘数据，从起始点开始计数，函数结束后可以通过该指针获取实际数量
*r_stastic              统计右边边缘数据，从起始点开始计数，函数结束后可以通过该指针获取实际数量
l_start_x               左边搜索的起始x坐标
l_start_y               左边搜索的起始y坐标
r_start_x               右边搜索的起始x坐标
r_start_y               右边搜索的起始y坐标
hightest                循环结束时找到的最高边缘点的高度

返回值：无
修改时间：2025年1月5日
    注意
example：
    fill_search_l_r((uint16)USE_num,image,&data_stastics_l_fill, &data_stastics_r_fill,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest_fill);
*/

//存储左右边缘点的坐标
uint16 points_l_fill[(uint16)USE_num][2] = { {  0 } };//左边缘点
uint16 points_r_fill[(uint16)USE_num][2] = { {  0 } };//右边缘点
uint16 dir_r_fill[(uint16)USE_num] = { 0 };//存储右边缘点的搜索方向
uint16 dir_l_fill[(uint16)USE_num] = { 0 };//存储左边缘点的搜索方向
uint16 data_stastics_l_fill = 0;//统计左边缘点的数量
uint16 data_stastics_r_fill = 0;//统计右边缘点的数量
uint8 hightest_fill = 0;//记录最高边缘点的高度

void fill_search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest_fill)
{

    uint8 i = 0, j = 0;

    // 左边缘搜索相关变量
    uint8 search_filds_l[8][2] = { { 0 } };  // 存储左边缘当前中心点的八邻域坐标
    uint8 index_l = 0;                      // 有效邻域点计数器
    uint8 temp_l[8][2] = { { 0 } };         // 临时存储符合边缘条件的点
    uint8 center_point_l[2] = { 0 };        // 当前左边缘中心点坐标
    uint16 l_data_statics;                  // 左边缘点统计计数
    //左边缘搜索的八邻域方向定义（顺时针方向）
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //搜索顺序为顺时针

    // 右边缘搜索相关变量
    uint8 search_filds_r[8][2] = { { 0 } };  // 存储右边缘当前中心点的八邻域坐标
    uint8 center_point_r[2] = { 0 };// 当前右边缘中心点坐标
    uint8 index_r = 0;// 有效邻域点计数器
    uint8 temp_r[8][2] = { { 0 } }; // 临时存储符合边缘条件的点
    uint16 r_data_statics;// 右边缘点统计计数
    //右边缘搜索的八邻域方向定义（顺时针方向）
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //搜索顺序为顺时针
    // 初始化统计计数器
    l_data_statics = *l_stastic;  // 获取传入的初始统计值
    r_data_statics = *r_stastic;  // 获取传入的初始统计值

    // 设置搜索起始点
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

    // 主循环：控制搜索步数，防止无限循环
    while (break_flag--)
    {
        //生成八邻域点
        for (i = 0; i < 8; i++)//遍历八个方向
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //记录当前中心点到结果数组
        points_l_fill[l_data_statics][0] = center_point_l[0];//x
        points_l_fill[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;// 左边缘点计数加一

        // 生成右边缘当前中心点的八邻域坐标
        for (i = 0; i < 8; i++)
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
         // 记录当前中心点到结果数组
        points_r_fill[r_data_statics][0] = center_point_r[0];//x
        points_r_fill[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;// 重置左边缘临时存储和计数器
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;
            temp_l[i][1] = 0;
        }

        // 左边缘点筛选：寻找符合边缘条件的邻域点
        for (i = 0; i < 8; i++)
        {
            //关键条件：当前点为背景(0)且下一个点为前景(255)
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                 //保存符合条件的点
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l_fill[l_data_statics - 1] = (i);//记录搜索方向
            }

            //从候选点中选择最优的新中心点（y坐标最小的点，即最高的点）
            if (index_l)
            {
                // 默认为第一个点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                // 遍历所有候选点，寻找y坐标最小的点
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        //终止条件1：连续多个点坐标相同，认为搜索陷入循环，退出
        if ((points_r_fill[r_data_statics][0]== points_r_fill[r_data_statics-1][0]&& points_r_fill[r_data_statics][0] == points_r_fill[r_data_statics - 2][0]
            && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 1][1] && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 2][1])
            ||(points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 2][0] && points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 3][0]
                && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 2][1] && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 3][1]))
        {
             //printf("连续多个点坐标相同，退出\n");
            break;
        }

        //终止条件2：左右边缘点距离足够近，认为找到了顶部，退出
        if (my_abs(points_r_fill[r_data_statics][0] - points_l_fill[l_data_statics - 1][0]) < 2
            && my_abs(points_r_fill[r_data_statics][1] - points_l_fill[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右边缘相遇，退出\n");
            *hightest_fill = (points_r_fill[r_data_statics][1] + points_l_fill[l_data_statics - 1][1]) >> 1;//ȡ    ߵ
            //printf("\n顶部y=%d，退出\n",*hightest);
            break;
        }

        //条件3：左边缘低于右边缘，暂停左边缘搜索，等待右边缘
        if ((points_r_fill[r_data_statics][1] < points_l_fill[l_data_statics - 1][1]))
        {
            //printf("\n左边缘比右边缘低，左边缘等待右边缘\n");
            continue;//左边缘比右边缘低，左边缘等待右边缘
        }

        //条件4：左边缘搜索方向为右下(7)且右边缘更高，回退左边缘
        if (dir_l_fill[l_data_statics - 1] == 7
            && (points_r_fill[r_data_statics][1] > points_l_fill[l_data_statics - 1][1]))//  ߱  ұ߸    Ѿ
        {
            //printf("\n左边缘开始向下搜索，等待右边缘... \n");
            center_point_l[0] = (uint8)points_l_fill[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l_fill[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        // 右边缘处理逻辑（与左边缘对称）
        r_data_statics++;// 右边缘点计数加一

        // 重置右边缘临时存储和计数器
        index_r = 0;
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;
            temp_r[i][1] = 0;
        }

        // 右边缘点筛选：寻找符合边缘条件的邻域点
        for (i = 0; i < 8; i++)
        {
            // 关键条件：当前点为背景(0)且下一个点为前景(255)
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                // 保存符合条件的点到临时数组
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;// 有效点计数加一
                dir_r_fill[r_data_statics - 1] = (i);;// 记录搜索方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            // 从候选点中选择最优的新中心点（y坐标最小的点，即最高的点）
            if (index_r)
            {

                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    // 输出结果：通过指针返回左右边缘点的实际数量
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/*
函数名称：void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
函数功能：根据左右边界点数据生成边界数组
输入参数：
- total_L：左边界有效点数
- total_R：右边界有效点数
- h：起始行位置
输出参数：无（修改全局边界数组l_border和r_border）
创建日期：2025年1月5日
备    注：example: get_direction_fill(data_stastics_l_fill, data_stastics_r_fill, h);
*/

void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h_if = 0;
    //调整起始行到有效范围
    if(h>image_h - 3)
    {
        h_if = image_h;
        h=image_h - 2;
    }
    else if(h<=0)
    {
        h_if = 0;
        h=0;
    }
    else
    {
        h_if = h;
    }
    // 初始化上方行边界（未检测到边界的区域）
    for (i = 0;i<h_if;i++)
    {
        l_border[i] = border_min;// 左边界设为最小值
    }
    for (i = 0; i < h_if; i++)
    {
        r_border[i] = border_max;// 右边界设为最大值
    }
     // 填充左边界数据
    for (j = 0; j < total_L; j++)
    {
        if (h == 0)
        {
            break; // 到达图像顶部时退出
        }
        //printf("%d\n", j);
        // 查找与当前行h匹配的左边界
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l_fill[j][0]+1;
        }
        else continue;
        h--;
    }
    // 填充右边界数据（逻辑与左边界对称）
    for (j = 0; j < total_R; j++)
    {
        if (h == 0)
        {
            break;
        }
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r_fill[j][0] - 1;
        }
        else continue;
        h--;
    }
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
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б  k=y/x
             // 计算拟合直线的截距（b = y - kx）
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];// ؾ b=y-kx
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
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б  k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];// ؾ b=y-kx
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
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б  k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];// ؾ b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            // 修复右边缘
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б  k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];// ؾ b=y-kx
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
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);  // 最小二乘法拟合
         // 用拟合直线延伸修复边缘到图像底部
        for (i = break_num_l - 5; i < image_h - 1 + 50; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);// ޷
        }

        // 使用最小二乘法拟合直线，延伸修复右边缘（逻辑与左边缘相同）
        start = break_num_r - 10;  // 拟合起始行
        start = (uint8)limit_a_b(start, 0, image_h);  // 确保起始行有效
        end = break_num_r - 5;     // 拟合终止行
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);  // 最小二乘法拟合
         // 用拟合直线延伸修复边缘到图像底部
        for (i = break_num_r - 5; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
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
            if(i==15)//ǰʮ п
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
                slope_l_rate = (float)(118-end_num_l) / ((border_max-r_border[118]+border_min)-l_border[end_num_l]);//б  k=y/x
                intercept_l = 118 - slope_l_rate*(border_max-r_border[118]+border_min);// ؾ b=y-kx
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
//                     break_num_l = (uint8)points_l[i][1];//    y
//                     end_num_l  = (uint8)points_l[i][0];//    x
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
            for (i = image_h - 20; i > *hightest; i--)//     б
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
            for (uint16 w = image_h - 15; w > *hightest; w--)//     б
            {
                if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
                {
                    break_num_l = (uint8)w;//    y
                    break;
                }
            }
            // 2. 再次确认方向序列特征（同状态4）
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    end_num_l = (uint8)points_l[i][1];//    y
                    break;
                }
            }
            // 3. 检查右边界是否存在（未丢失）
            if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
                sum_island = 6;
            // 4. 更新右边界（基于新的曲率计算直线方程）
            slope_l_rate = (float)(break_num_l-end_num_l) / (r_border[break_num_l]-l_border[end_num_l]);//б  k=y/x
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
//                        break_num_l = (uint8)points_l[i][1];//    y
//                        end_num_l  = (uint8)points_l[i][0];//    x
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
            slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б  k=y/x
            intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);// ؾ b=y-kx
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
//                 break_num_r = (uint8)points_r[i][1];//    y
//                 end_num_r  = (uint8)points_r[i][0];//    x
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
        for (i = image_h - 15; i > *hightest; i--)//     б
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
        for (uint16 w = image_h - 15; w > *hightest; w--)//     б
        {
            if (l_border[w] > l_border[w - 3] && (l_border[w] - l_border[w-3])>20)
            {
                break_num_r = (uint8)w;//    y
                break;
            }
        }
//        for (i = 1; i < total_num_l; i++)
//        {
//            if (dir_l[i - 10] >= 4 && dir_l[i - 10] <= 6 && dir_l[i-5] >= 4 && dir_l[i-5] <= 6
//                    && dir_l[i] >= 2 && dir_l[i] <= 4 && dir_l[i + 5] <= 4 && dir_l[i + 5] >= 2
//                    && dir_l[i + 10] <= 4 && dir_l[i + 10] >= 2)
//            {
//                break_num_r = (uint8)points_r[i][1];//    y
//                break;
//            }
//        }
        // 右边界存在时进入状态6
        if(r_loss_judge(r_border, 90 ,110) == 1 && r_loss_judge(r_border, 70 ,90) == 1)
            sum_island = 6;
        // 更新左边界（基于突变点
        end_num_r = l_border[break_num_r];
        slope_l_rate = (float)(break_num_r-0) / (end_num_r-188);//б  k=y/x
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
                    break_num_r = (uint8)points_r[i][1];//    y
                    end_num_r  = (uint8)points_r[i][0];//    x
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
        slope_l_rate = (float)(118-end_num_r) / ((border_max-l_border[118]+border_min)-r_border[end_num_r]);//б  k=y/x
        intercept_l = 118 - slope_l_rate*(border_max-l_border[118]+border_min);// ؾ b=y-kx
        for (i = end_num_l-10; i < image_h - 1; i++)
        {
            r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
   }
}
/**
* @brief 十字线状态检测函数
* @param uint8(*image)[image_w]     二值化图像
* @param uint8 *l_border            左边界数组指针
* @param uint8 *r_border            右边界数组指针
*  @see CTest       cross_stop(image,l_border,r_border);
* @return 返回说明
*     -<em>false</em> 检测失败
*     -<em>true</em> 检测成功
 */
void cross_stop(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border)
{
    uint8 start_point  = 85;  // 检测起始行（垂直位置）
    uint8 end_point = 105;    // 检测结束行（垂直位置）
    // 初始化计数器
    int banmaxian_kuandu = 0;  // 斑马线宽度（单个黑色条带）
    int banmaxian_hangshu = 0; // 有效斑马线行数
    int banmaxian_geshu = 0;   // 当前行检测到的黑色条带数量

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
                    m = banmaxian_geshu;  // 存储到全局变量
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
       jump_position == 0 &&  // 没有跳变位置标记
       sum_island == 0 && island == 0 &&  // 不在弯道上
       cross_sum == 0)  // 没有十字线累积标记
    {
        // 设置停车位置标记（1表示检测到十字线）
        stop_position = 1;
    }
}

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
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line, uint8* hightest)
{
    int white_line1 = 0;  // 第75行白色像素计数
    int white_line2 = 0;  // 第70行白色像素计数
    int long_start_l = 0; // 左边界突变起始行
    int long_end_l = 0;   // 左边界突变结束行
    int long_start_r = 0; // 右边界突变起始行
    int long_end_r = 0;   // 右边界突变结束行
    int loss_1 = 0;       // 边界宽度变化计数器
    int loss_2 = 0;       // 边界位置突变计数器
    int bridge_number = 0; // 桥梁特征计数器
    // 扫描图像中间区域，分析边界变化
    for (int i = 30; i < image_h-10; i++)
    {
        // 判断当前行边界宽度是否明显小于下一行（桥梁入口特征）
        if((my_abs(r_border[i] - l_border[i])*3/5) >= my_abs(r_border[i+3] - l_border[i+3]))
            loss_1++;
         // 判断左边界或右边界是否发生大幅突变（超过10像素）
        if(my_abs(l_border[i] - l_border[i+3]) >= 10)
            loss_2++;
        if(my_abs(r_border[i] - r_border[i+3]) >= 10)
            loss_2++;
    }
    // 当桥梁状态为"未激活"时，检测是否进入桥梁
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        // 最高点位置合理（<=30）时进行检测
        if(*hightest <= 30)
        {
            // 统计第75行和第70行的白色像素数量
            for (int x = l_border[image_h-10]; x <= r_border[image_h-10]; x++)
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
            if(white_line2 <= (white_line1*3/5) && loss_1>=2 && loss_2>=4)//70   ǵ     ʱ   룬         ϵ
            {
                if(sum_island == 0 && island == 0 && jump_position == 0
                    && cross_sum == 0)
                BridgeState = SINGLE_BRIDGE_ACTIVE;// 激活桥梁状态
            }
        }
    }
    // 当桥梁状态为"激活"时，处理桥梁区域并生成中心线
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
        // 从第30行开始扫描左边界（图像中间偏上区域）
        for (int i = 30; i < image_h-1; i++)
        {
             // 处理第30行（桥梁入口可能位置
            if(i == 30)
            {
                // 检查左边界是否在标准位置附近（94±7像素）
                if(my_abs(l_border[i]-94)<=7)
                {
                    // 从第31行开始寻找左边界突变点
                    for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                    {
                        // 如果到达图像底部（a=image_h-5），认为桥梁结束
                        if(a == image_h-5)
                        {
                            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE; // 重置桥梁状态为未激活
                            break;
                        }
                        // 检测左边界突变（相邻3行变化超过10像素）且满足图像特征
                        if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                            image[a+2][l_border[a-3]] == 255 &&// 突变点上方2行对应位置为白色
                             image[a-3][l_border[a-3]-5] == 0)// 突变点下方3行左侧5像素为黑色
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
            if(i>30 && my_abs(l_border[i] - l_border[i+3]) >= 10 && long_start_l == 0 && long_end_l == 0)
            {
                // 检查图像特征是否符合桥梁边界条件
                if(image[i][l_border[i+3]] == 255 && image[i+6][l_border[i+3]-5] == 0)
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
                        if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                                image[a+2][l_border[a-3]] == 255 && image[a-3][l_border[a-3]-5] == 0)
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
                center_line[i] = l_border[i];// 将左边界值赋给中心线数组
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
                        if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                                image[a+2][r_border[a-3]] == 255 &&
                                 image[a-3][r_border[a-3]+5] == 0)// 右侧5像素为黑色
                        {
                            long_start_r = i;// 记录右边界突变起始行
                            long_end_r = a;  // 记录右边界突变结束行
                            bridge_number++;
                            break;
                        }
                    }
                }
            }
            if(i>30 && my_abs(r_border[i] - r_border[i+3]) >= 10 && long_start_r == 0 && long_end_r == 0)
            {
                if(image[i][r_border[i+3]] == 255 && image[i+6][r_border[i+3]+5] == 0)
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
                        if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                            image[a+2][r_border[a-3]] == 255 && image[a-3][r_border[a-3]+5] == 0)
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
                center_line[i] = r_border[i];// 将右边界值赋给中心线数组
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
            BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;// 重置桥梁状态为未激活
        }
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
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r)
{
    int blake_line = 0; // 记录连续全黑行的数量
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
           r_loss_judge(r_border,70 ,90) == 0 // 右边界70-90行连续
           /*&& monotonicity_l == 1 && monotonicity_r == 1*/)
        {
             // 确保不在其他特殊区域（桥、环岛、十字线）
            if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE && sum_island == 0 && island == 0
                    && cross_sum == 0)
                jump_position = 1;// 标记跳变点位置
        }
    }

}

/*
函数名称：void image_process(void)
函数功能：图像主处理函数
输入参数：无
输出参数：无
创建日期：2025年1月5日
备    注：处理摄像头图像并识别赛道特征
example：image_process();
*/
void image_process(void)
{
    uint8 hightest = 0;// 记录图像中最高点的y坐标（值越小，位置越靠上）
    /* 图像采集与二值化处理 */
    Get_image(mt9v03x_image);// 从MT9V03X摄像头获取原始图像数据
    turn_to_bin();// 将原始图像转换为二值图（0=黑，255=白）

     /* 图像预处理：滤波与区域标记 */
    image_filter(bin_image);// 对二值图进行滤波处理（去噪）
    image_draw_rectan(bin_image);// 在图像上绘制预处理区域矩形

    /* 边界数据统计变量初始化 */
    data_stastics_l = 0; // 左边界有效点统计计数
    data_stastics_r = 0; // 右边界有效点统计计数
    data_stastics_l_fill = 0; // 左边界填充点统计计数
    data_stastics_r_fill = 0; // 右边界填充点统计计数

    /* 当检测到起始点时，执行赛道特征检测 */
    if (get_start_point(image_h - 2))// 检测图像底部第2行是否为起始点
    {
        // 搜索左右边界并获取最高点位置
        search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);

        // 从统计数据中提取左右边界具体值
        get_left(data_stastics_l); // 提取左边界数据
        get_right(data_stastics_r); // 提取右边界数据

        // 分析边界单调性（判断弯道方向）
        monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
        // 赛道特征检测（按优先级依次执行）
        // 十字线填充与检测
        cross_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
        // 跳跃检测
        jump_judge(bin_image,&hightest,l_border,r_border,monotonicity_l, monotonicity_r);
        // 十字线停车检测
        cross_stop(bin_image,l_border,r_border);
        // 环岛检测与处理
        around_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest,l_index,r_index,monotonicity_l, monotonicity_r);
    }
    /* 异常情况处理：未检测到起始点且无跳变点时 */
    if(get_start_point(image_h - 2) == 0 && jump_position == 0)
    {
        //target_velocity = 0;// 目标速度设为0（停车）
        string_not = 1;// 标记为无有效赛道字符串
    }


    /* 计算赛道中心线（左右边界平均值） */
    for (int i = hightest; i < image_h-1; i++)
        {
         // 中心线 = (左边界+右边界)/2（右移1位等效除以2）
        center_line[i] = (l_border[i] + r_border[i]) >> 1;
        }
    /* 桥梁检测与中心线生成 */
    bridge_fill(bin_image,l_border, r_border, center_line, &hightest);

     /* 蜂鸣器状态控制：有特殊状态时报警 */
    if( stop_position == 0
        && cross_sum == 0 && island == 0 && sum_island == 0)
    {
        // 无跳变点、无停车点、不在桥上、无十字线、无环岛时，蜂鸣器关闭
        buzzer = 0;
    }
    else
        buzzer = 1;// 存在任意特殊状态时，蜂鸣器开启

    /* 显示原始图像（左上角(0,0)开始） */
    //tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);
    /* 显示二值化图像（左上角(0,60)开始） */
    //tft180_show_gray_image (0,60, bin_image[0], image_w, image_h, image_w/2, image_h/2,0);
    /* 绘制左边界点（蓝色线条） */
    for (int i = 0; i+1 < data_stastics_l; i++)
    {
        //tft180_draw_point((points_l[i][0]+2)/2, points_l[i][1]/2, RGB565_BLUE);
        //tft180_draw_line((points_l[i][0]+2)/2, points_l[i][1]/2, (points_l[i+1][0]+2)/2, points_l[i+1][1]/2,RGB565_BLUE);
    }
    /* 绘制右边界点（红色线条） */
    for (int i = 0; i+1 < data_stastics_r; i++)
    {
        //tft180_draw_point((points_r[i][0]-2)/2, points_r[i][1]/2, RGB565_RED);
        //tft180_draw_line((points_r[i][0]-2)/2, points_r[i][1]/2, (points_r[i+1][0]-2)/2, points_r[i+1][1]/2,RGB565_RED);
    }
    /* 绘制中心线和边界点（绿色） */
    for (int i = hightest; i < image_h-1; i++)
    {
//      center_line[i] = (l_border[i] + r_border[i]) >> 1;

        //tft180_draw_point(center_line[i]/2, i/2, RGB565_GREEN);// 中心线
        //tft180_draw_point(l_border[i]/2, i/2, RGB565_GREEN);// 左边界
        //tft180_draw_point(r_border[i]/2, i/2, RGB565_GREEN);// 右边界
        x1_boundary[i] = l_border[i]; // 存储左边界到x1_boundary数组
        x2_boundary[i] = center_line[i]; // 存储中心线到x2_boundary数组
        x3_boundary[i] = r_border[i]; // 存储右边界到x3_boundary数组
    }
     /* 显示调试信息到TFT屏幕 */
    int len;
    char buffer[32];
    /* 计算加权边界值（用于转向控制，越远的行权重越大） */
    border = (float)((center_line[image_h-2] * 0.07) + (center_line[image_h-12] * 0.10)
           + (center_line[image_h-22] * 0.25) + (center_line[image_h-27] * 0.20)
           + (center_line[image_h-32] * 0.12) + (center_line[image_h-42] * 0.09)
           + (center_line[image_h-52] * 0.07) + (center_line[image_h-62] * 0.06)
           + (center_line[image_h-72] * 0.04));
 //   len = snprintf(buffer, sizeof(buffer),"%.3f",border);
 //   len = snprintf(buffer, sizeof(buffer),"%d",monotonicity_l);

    len = snprintf(buffer, sizeof(buffer),"%d",m);// 将计数器m的值转为字符串
    //tft180_show_string(0, 140, buffer);// 在(0,140)位置显示m的值

 //   len = snprintf(buffer, sizeof(buffer),"%d",broken_line_y);
//    len = snprintf(buffer, sizeof(buffer),"%d",jump_position);
//    len = snprintf(buffer, sizeof(buffer),"%d",sum_island);

    len = snprintf(buffer, sizeof(buffer),"%d",cross_sum);// 十字线计数
 //   len = snprintf(buffer, sizeof(buffer),"%d",bridge_position);
  //  len = snprintf(buffer, sizeof(buffer),"%d",stop_position);
    //tft180_show_string(100, 140, buffer);// 在(100,140)位置显示cross_sum

    //len = snprintf(buffer, sizeof(buffer),"%d",island);

    len = snprintf(buffer, sizeof(buffer),"%d",monotonicity_r);// 右边界单调性
    //tft180_show_string(100, 90, buffer);// 在(100,90)位置显示monotonicity_r
}
/*函数名称：void get_turn_value(float kp,float kp2,float kd,float gkd)
 *函数功能：图像偏差值转换为转角值
 *输入参数：kp
 *输入参数：kp2
 *输入参数：kd
 *输入参数：gkd
 *输出参数：无
 *创建日期：2025年6月17日
 *备    注：逐飞公式，看逐飞的轮腿车模浅析
 *example：get_turn_value(0,0,0,0)
 */
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

