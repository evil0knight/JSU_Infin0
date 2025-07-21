#include "zf_common_headfile.h"


//ISLAND_H_1是轮腿开源佬的圆环代码,ISLAND_H_2是B站的代码，ISLAND_H_3是100元代码
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
*/
void CheckCircle(uint8 *l_border, uint8 *r_border,
                 uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r,
                 uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,
                 uint16 *l_index, uint16 *r_index,
                 int monotonicity_l, int monotonicity_r)
{
    // 将原函数中的变量替换为参数
    // 原条件1: circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1
    if (circle_type == CIRCLE_NONE &&
        total_num_l > 0 &&           // Lpt0_found: 左边缘存在
        total_num_r == 0 &&          // !Lpt1_found: 右边缘不存在
        monotonicity_r == 1)         // is_straight1: 右边缘单调性为1 (直道)
    {
        circle_type = CIRCLE_LEFT_BEGIN;
        //beel = 500;
    }

    // 原条件2: circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0
    if (circle_type == CIRCLE_NONE &&
        total_num_l == 0 &&          // !Lpt0_found: 左边缘不存在
        total_num_r > 0 &&           // Lpt1_found: 右边缘存在
        monotonicity_l == 1)         // is_straight0: 左边缘单调性为1 (直道)
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

        // 替换：pts_resample_left_count -> total_num_l
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

        // 替换：pts_resample_right -> points_r
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

        // 替换：Lpt1_found -> (r_index 存在性逻辑)
        if (/* r_index 存在 */) {
            // 替换：Lpt1_rpts1s_id -> r_index 对应值
            total_num_r = mid_right_count = /* r_index 值 */;
        }
        if (/* r_index 存在 */ && /* r_index 值 */ < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }
    else if (circle_type == CIRCLE_LEFT_OUT)
    {
        track_type = TRACK_LEFT;

        // 替换：is_straight1 -> monotonicity_r 判断
        if (monotonicity_r == STRAIGHT) {
            circle_type = CIRCLE_LEFT_END;
        }
    }
    else if (circle_type == CIRCLE_LEFT_END)
    {
        track_type = TRACK_RIGHT;

        // 替换：pts_resample_left_count -> total_num_l
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

        // 替换：pts_resample_right_count -> total_num_r
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

        // 替换：pts_resample_left -> points_l
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

        // 替换：Lpt0_found -> (l_index 存在性逻辑)
        if (/* l_index 存在 */) {
            // 替换：Lpt0_rpts0s_id -> l_index 对应值
            total_num_l = mid_left_count = /* l_index 值 */;
        }
        if (/* l_index 存在 */ && /* l_index 值 */ < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_OUT)
    {
        track_type = TRACK_RIGHT;

        // 替换：is_straight0 -> monotonicity_l 判断
        if (monotonicity_l == STRAIGHT) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }
    else if (circle_type == CIRCLE_RIGHT_END)
    {
        track_type = TRACK_LEFT;

        // 替换：pts_resample_right_count -> total_num_r
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
* @param int monotonicity_r         右边缘单调性，单调是1，不单调是0
* @return 无返回值，结果存储在全局变量中
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void around_fill_1(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
    if(/*island==2&&*/monotonicity_r==0&&monotonicity_l==1&&*hightest<15&&(circle_type==CIRCLE_NONE||circle_type==CIRCLE_RIGHT_BEGIN))//检测到了右环条件右边出现断裂,右边搜到黑框上,左边单调
    {
    /*右*/circle_type=CIRCLE_RIGHT_BEGIN;//开始圆环

    /*右*/
    }



}
#elif defined(ISLAND_H_3)
void circle_find(void)
{
//       if(circle_flag)
//         k= -0.16f;
       circle_width_left = circle_width_right = 0;
       // 左圆环检测条件
       if(L_corner_flag==1&&L_corner_row>5&&L_corner_row<MT9V03X_H-5&&// 左拐角有效且位于图像中间区域
          abs(zx[L_corner_row]-zx[L_corner_row+2])<=3&&// 检查拐点附近连续性（差值≤3）
          abs(zx[L_corner_row+1]-zx[L_corner_row+3])<=3&&
           abs(zx[L_corner_row+2]-zx[L_corner_row+4])<=3&&
              (zx[L_corner_row]-zx[L_corner_row-2])>=5&&// 检查拐点突变性（差值≥5）
              (zx[L_corner_row]-zx[L_corner_row-3])>=5&&//  -3偏移的边缘变化≥5
              (zx[L_corner_row]-zx[L_corner_row-4])>=7
              && (yx[R_corner_row]-yx[R_corner_row-2])<=3&&// 右侧拐点无突变（差值≤3）
              (yx[R_corner_row]-yx[R_corner_row-3])<=3&&
              (yx[R_corner_row]-yx[R_corner_row-4])<=3)
       {
           circle_width_left = yx[L_corner_row-10]-zx[L_corner_row-10];// 计算左圆环宽度
           L_circle_corner_row = L_corner_row;// 记录拐点行号
           L_circle_corner_flag = 1;;// 标记左圆环检测成功
       }

       if(R_corner_flag==1&&R_corner_row>5&&R_corner_row<MT9V03X_H-5&&// 右圆环检测
            abs(yx[R_corner_row]-yx[R_corner_row+2])<=2&&//  右边缘行（R_corner_row）+2偏移的边缘变化≤2，判断平缓
            abs(yx[R_corner_row+1]-yx[R_corner_row+3])<=2&&
            abs(yx[R_corner_row+2]-yx[R_corner_row+4])<=2&&
             (yx[R_corner_row]-yx[R_corner_row-2])<=-5&&// 右侧拐点突变方向相反（负值）
             (yx[R_corner_row]-yx[R_corner_row-3])<=-5&&
             (yx[R_corner_row]-yx[R_corner_row-4])<=-7
            && (zx[L_corner_row]-zx[L_corner_row-2])<=3&&// 左侧拐点无突变
            (zx[L_corner_row]-zx[L_corner_row-3])<=3&&
            (zx[L_corner_row]-zx[L_corner_row-4])<=3)
        {
            circle_width_right = yx[R_corner_row-10]-zx[R_corner_row-10];// 计算右圆环宽度
            R_circle_corner_row = R_corner_row;//  记录右圆环特征行号
            R_circle_corner_flag = 1;//  置右圆环检测标志为1（检测到疑似右圆环
        }
        base_width = yx[MT9V03X_H-2]-zx[MT9V03X_H-2];//  计算图像底部（MT9V03X_H-2行）的左右边缘差，作为基础宽度
       //进入左圆环模式
        if(circle_mode==0&&L_circle_corner_flag==1&&
            Continuity_Change_Right(MT9V03X_H-10,30)==0&&// 右侧连续性无变化
            Right_TotalLost(MT9V03X_H-30,Finnalline)<2&&// 右侧丢失边界少于2处
            L_corner_row>MT9V03X_H-80&&// 拐点靠近图像底部
            Left_Lost_Time>15)// 左侧丢失时间较长
            //&&Monotonicity_Change_Right()==0
            {
                left_circle_cnt++;// 累计左圆环触发计数
            }
        else if(left_circle_cnt!=0)
        {
            left_circle_cnt--;// 未满足条件时递减计数（防误触）
        }
        if(left_circle_cnt>=2)// 连续检测到2次
        {
            left_circle_cnt=0;
            left_circle_flag_cnt++;
            circle_flag = 1;
            circle_mode = 1;//进入阶段1（初始转弯）
            left_circle_flag = 1;
            buzzer();// 蜂鸣器提示
        }

        //左圆环触发条件
        if(T_2s_flag==1&&//2秒定时标志触发（用于防抖或周期性检测）
            circle_mode==0&&//当前未处于任何圆环处理阶段
            L_circle_corner_flag==1&&//左圆环拐角已检测到
            Continuity_Change_Right(MT9V03X_H-10,30)==0&&//图像底部30行内右侧边界连续无突变
            Right_TotalLost(MT9V03X_H-50,Finnalline)<2&&//右侧在底部50行内丢失边界少于2处
            L_corner_row>MT9V03X_H-80&&//左拐角位于图像底部80行内，靠近车辆
            circle_width_left>135)//左圆环宽度超过135像素（经验阈值）
            //&&Monotonicity_Change_Right()==0
        {
                circle_flag = 1;     // 全局圆环标志置位
                circle_mode = 1;     // 进入阶段1（初始转弯）
                left_circle_flag = 1; // 左圆环标志置位
                buzzer();           // 蜂鸣器鸣响（提示驾驶员或调试
        }

        //阶段1：初始转弯
        if(left_circle_flag==1)//  ?
        {
            if(circle_mode==1)
            {
                gpio_toggle_level(BUZZER_PIN);// 切换蜂鸣器状态（闪烁提示）

//            buzzer();
                yaw_flag=1; // 启用陀螺仪控制
//            Circle_in_Left();
            if(T1_ms_cricle1_flag==1&&Boundry_Start_Left==MT9V03X_H-2&&Left_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-40)==0&&Right_TotalLost(MT9V03X_H-40,Finnalline)<5)//
            {
                t2_ms_cricle1 = 0;       // 清零计时器
                T1_ms_cricle1_flag = 0;  // 清除阶段1标志
                circle_mode = 2;         // 进入阶段2
                now_yaw=0;               // 重置陀螺仪角度
            }
        }

        if(circle_mode==2)
        {
            gpio_set_level(BUZZER_PIN, GPIO_LOW);// 关闭蜂鸣器

            yaw_flag=1;
//            gpio_init(BUZZER_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//            Circle_in_Left();
            if(abs(now_yaw)>=16200)//?   ?   50  //Lin_circleup_row>MT9V03X_H-50&&Right_Lost_Time<=2
            {
//                gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
                t2_ms_cricle2 = 0;
                circle_mode=3;//进入阶段3
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
            // 角度进一步增加
            if(abs(now_yaw)>=22800)//&&Rout_circleup_row>MT9V03X_H-80)//?   ?   200
            {
                t2_ms_cricle3 = 0;
                circle_mode=4;
                now_yaw=0;

            }
        }
        if(circle_mode==4)
        {

            get_upturningleft_point();// 获取左转出环的参考点（未实现）
//            Circle_out_Left();
            yaw_flag=1;
            // 角度接近完成
            if(abs(now_yaw)>=24800)//?   ?   10
            {
                if(Finnalline<=20)// 角度接近完成
                {
                    circle_mode = 5;// 最终边界线较短（接近直道）
                    now_yaw=0;

                }
            }
        }
        if(circle_mode==5)
        {
            gpio_toggle_level(BUZZER_PIN);// 蜂鸣器闪烁
//            get_upturningleft_point();
//            Circle_out_Left();
//            buzzer();
//            buzzer();
//            buzzer();
//            Circle_in_Left();
            if(T1_ms_cricle5_flag==1&&Left_sideIsNoLost(MT9V03X_H-35,MT9V03X_H-80)==0)//Left_Lost_Time<15&&Lin_circleup_flag!=1&&Lin_circleup_row>MT9V03X_H-40
            {
                gpio_set_level(BUZZER_PIN, GPIO_LOW);// 关闭蜂鸣器
//                buzzer();
                yaw_flag=0;// 禁用陀螺仪控制
                now_yaw=0;
                circle_cnt++; // 圆环计数增加
                T1_ms_cricle5_flag = 0;
                circle_flag=0;// 清除所有标志
                circle_mode=0;
                left_circle_flag=0;
                buzzer();// 完成提示音
            }
        }
      }


        if(circle_mode==0&&//系统未处理其他圆环状态
            R_circle_corner_flag==1&&//右圆环拐角已通过图像识别确认
            Continuity_Change_Left(MT9V03X_H-10,30)==0&&//左侧边界在底部30行内变化平缓（无突变）
            Left_TotalLost(MT9V03X_H-30,Finnalline)<2&&//左侧边界丢失次数少于2次（保证赛道连续性）
            R_corner_row>MT9V03X_H-80&&//拐点位于图像底部80行内（近场区域）
            Right_Lost_Time>15)//&&Monotonicity_Change_Left()==0
        {
            right_circle_cnt++;// 条件满足时增加计数
        }
        else if(right_circle_cnt!=0)
        {
            right_circle_cnt--;// 条件不满足时递减计数
        }
        if(right_circle_cnt>=3)// 累计3次确认
        {
            right_circle_cnt=0;
            right_circle_flag_cnt++;
            circle_flag = 1;     // 全局圆环标志
            circle_mode = 1;     // 进入阶段1（右转切入）
            right_circle_flag = 1;// 右圆环专属标志
//            buzzer();
//            buzzer();
//            buzzer();
        }
        //
        if(T_2s_flag==1&&circle_mode==0&&R_circle_corner_flag==1&&Continuity_Change_Left(MT9V03X_H-10,30)==0
                &&Left_TotalLost(MT9V03X_H-5,Finnalline)<2&&R_corner_row>MT9V03X_H-80&&circle_width_right>135)//&&Monotonicity_Change_Left()==0
        {
            // 强制进入圆环模式（不依赖计数
            circle_flag = 1;
            circle_mode = 1;//???
            right_circle_flag = 1;
//            buzzer();
//            buzzer();
//            buzzer();
        }
        if(right_circle_flag==1)//  ?
        {
            //阶段1：初始右转切入
            if(circle_mode==1)
            {
                gpio_toggle_level(BUZZER_PIN);// 蜂鸣器状态翻转
                yaw_flag=1;// 启用陀螺仪控制

    //              Circle_in_Right();
                if(T1_ms_cricle1_flag==1&&
                    Boundry_Start_Right==MT9V03X_H-2&&// 右边界从图像底部开始
                    Right_sideIsNoLost(MT9V03X_H-5,MT9V03X_H-30)==0&&Left_TotalLost(MT9V03X_H-40,Finnalline)<5)//
                {
                    t2_ms_cricle1 = 0;
                    T1_ms_cricle1_flag = 0;
                    circle_mode = 2;//
                    now_yaw=0;
                }
            }
            //阶段2：稳定右转
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
            //阶段3：出环准备
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
            //阶段4：出环执行
            if(circle_mode==4)
            {
    //              get_upturningright_point();
    //              Circle_out_Right();

                yaw_flag=1;

                if(abs(now_yaw)>=24800)// 接近完成角度
                {
                    if(Finnalline<=20)// 最终边界线足够短（接近直道）
                    {
                        circle_mode = 5;// 进入收尾阶段
                        now_yaw=0;
                    }
                }
            }
            //阶段5：恢复直行
            if(circle_mode==5)
            {
                gpio_toggle_level(BUZZER_PIN);// 蜂鸣器闪烁
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
