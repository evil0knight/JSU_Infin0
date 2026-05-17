/*
 * tracking.c
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */

 #include "headfile.h"
 #include <float.h>
 #include <math.h>

 float (*mid_track)[2];
 int32_t mid_track_count;
 float pure_angle = 0;
 float pure_angle_half;
 float curvature_ave;
 float curvature_now;
 float dx_near;
 float last_curvature_ave;
 float last_curvature_now;
 float (*rpts)[2];
 int rpts_num;
 float last_pure_angle = 0.0f;
 int8_t turn_flag      = 0;
 int circle_flag       = 1;
 int pingbi_num        = 0;
 int last_garage_type  = GARAGE_NONE;
 float turn_threshold  = 12.0f;
 int begin_start = 0;

 KalmanFilter KF_curvature_now;
 KalmanFilter KF_curvature_ave;
 KalmanFilter KF_lateral_deviation;
 float standardized_curvature_now = 0;
 float standardized_curvature_ave = 0;
 float lateral_deviation = 0;
 float preview_distance = 0;
 float K_v_sq = 0;
 float K_v = 0;
 float K_curvature = 0;
 float K_lateral_deviation = 0;
 float K_lateral_deviation_straight = 0;
 int is_supplement_line = 0;

void state_clear()
{
    cross_type = CROSS_NONE;
    circle_type = CIRCLE_NONE;
    barrier_type = BARRIER_NONE;
}

void tracking_init(void)
{
    kalman_init(&KF_curvature_now, 0, 1, 1, 1);
    kalman_init(&KF_curvature_ave, 0, 1, 1, 1);
    kalman_init(&KF_lateral_deviation, 0, 1, 1, 1);
}

// 计算最小二乘法斜率的函数
float leastSquaresSlope(float points[][2], int n)
{
//    float sum_x         = 0;
//    float sum_y         = 0;
//    float sum_xy        = 0;
//    float sum_x_squared = 0;
//
//    // 计算各项和
//    for (int i = 0; i < n; i++) {
//        sum_x += points[i][1];
//        sum_y += points[i][0];
//        sum_xy += points[i][1] * points[i][0];
//        sum_x_squared += points[i][1] * points[i][1];
//    }
//
//    // 计算斜率
//    float numerator   = (float)n * sum_xy - sum_x * sum_y;
//    float denominator = (float)n * sum_x_squared - sum_x * sum_x;
//
//    if (denominator == 0) {
//        // 避免除以零错误
////        printf("Error: denominator is zero.\n");
//        return 0;
//    }
//
//    float temp = denominator / numerator;

//    return temp;
    float sum_x = 0;
    float sum_y = 0;
    float sum_xy = 0;
    float sum_x_squared = 0;

    for (int i = 0; i < n; i++) {
        float x = points[i][1];  // 正确获取x坐标
        float y = points[i][0];  // 正确获取y坐标

        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x_squared += x * x;
    }

    float numerator = n * sum_xy - sum_x * sum_y;
    float denominator = n * sum_x_squared - sum_x * sum_x;

    if (denominator == 0) {
        // 处理垂直线（完全多重共线性）
        return INFINITY;  // 或返回NaN，需包含math.h
    }

    return numerator / denominator;  // 正确返回斜率
}

// 计算中心点的函数
void center_point(float points[][2], float n, float point[2])
{
    float sum_x = 0;
    float sum_y = 0;

    // 计算各项和
    for (int i = 0; i < n; i++)
    {
        sum_x += points[i][1];
        sum_y += points[i][0];
    }

    // 计算斜率
    point[1] = sum_x / n;
    point[0] = sum_y / n;
}
// 计算点到直线距离的函数
float calculate_distance(float line_point[2], float slope, float point[2])
{
    if(fabsf(slope) > 1e4)
    {
        return point[1]-line_point[1];
    }
    else if(fabsf(slope) < 1e-4)
    {
        return point[0]-line_point[0];
    }
    float numerator = fabsf(slope) * (point[1] - line_point[1]) - point[0] + line_point[0];
    float denominator = sqrtf(slope*slope + 1.0f);
    return numerator / denominator;
}

// 计算曲率的函数
float calculate_curvature(float x[], float y[], int n)
{
    float total_curvature = 0.0;
    n = (int)((float)n * 0.8f);

    for (int i = 11; i < n -1; i++) {
        float x1 = x[i - 1], y1 = y[i - 1];
        float x2 = x[i], y2 = y[i];
        float x3 = x[i + 1], y3 = y[i + 1];

        float dx1 = x2 - x1;
        float dy1 = y2 - y1;
        float dx2 = x3 - x2;
        float dy2 = y3 - y2;

        float dx = (dx1 + dx2) / 2;
        float dy = (dy1 + dy2) / 2;

        float ddx = x3 - 2 * x2 + x1;
        float ddy = y3 - 2 * y2 + y1;

        float numerator   = dx * ddy - dy * ddx;
        float denominator = powf(dx * dx + dy * dy, 1.5f);

        if (fabs(denominator) > FLT_EPSILON) {
//            total_curvature += numerator / denominator * (n - 1 - i / 2) / (n - 1);
            total_curvature += numerator / denominator;
        }
    }

//    return total_curvature / (float)(n - 2) * 4.0f / 3.0f;
    return total_curvature / (float)(n - 2);
}

float calculate_curvature_now(float x[], float y[])
{
    float total_curvature = 0.0;

    for (int i = 1; i < 4; i++) {
        float x1 = x[i - 1], y1 = y[i - 1];
        float x2 = x[i], y2 = y[i];
        float x3 = x[i + 1], y3 = y[i + 1];

        float dx1 = x2 - x1;
        float dy1 = y2 - y1;
        float dx2 = x3 - x2;
        float dy2 = y3 - y2;

        float dx = (dx1 + dx2) / 2;
        float dy = (dy1 + dy2) / 2;

        float ddx = x3 - 2 * x2 + x1;
        float ddy = y3 - 2 * y2 + y1;

        float numerator   = dx * ddy - dy * ddx;
        float denominator = powf(dx * dx + dy * dy, 1.5f);

        if (fabs(denominator) > FLT_EPSILON) {
            total_curvature += numerator / denominator;
        }
    }

    return total_curvature / 3.0f;
}

float calculateX(float a_x, float a_y, float slope, float b_y)
{
    float b_x = a_x - (b_y - a_y) * slope;
    return b_x;
}

void tracking_line(void)
{

    if (pts_resample_left_count < pts_resample_right_count *2/ 3 && pts_resample_left_count < 100) {
        track_type = TRACK_RIGHT;
    } else if (pts_resample_right_count < pts_resample_left_count *2/ 3 && pts_resample_right_count < 100) {
        track_type = TRACK_LEFT;
    } else if (pts_resample_left_count < 20 && pts_resample_right_count > pts_resample_left_count) {
        track_type = TRACK_RIGHT;
    } else if (pts_resample_right_count < 20 && pts_resample_left_count > pts_resample_right_count) {
        track_type = TRACK_LEFT;
    }

}


void ElementJudge(void)
{
    CheckGarage();
    if (garage_type == GARAGE_NONE) {
        CheckCross();
        if (cross_type == CROSS_NONE) {
            CheckBarrier();
            if (barrier_type == BARRIER_NONE) {
                    CheckCircle();
                    if(circle_type==CIRCLE_NONE){
                        CheckLost();
                }
            }
        }
    }

    if (garage_type != GARAGE_NONE) {
        cross_type   = CROSS_NONE;
        circle_type  = CIRCLE_NONE;
        lost_type    = LOST_NONE;
    }
    if (cross_type != CROSS_NONE) {
        circle_type  = CIRCLE_NONE;
        lost_type    = LOST_NONE;
    }
    if(circle_type!=CIRCLE_NONE){
        lost_type  = LOST_NONE;
    }
}

void ElementRun(void)
{
    if (garage_type != GARAGE_NONE) {
        RunGarage();
    }

    else if (cross_type != CROSS_NONE) {
        RunCross();
    } else if (barrier_type != BARRIER_NONE) {
        RunBarrier();
    } else if (circle_type != CIRCLE_NONE) {
        RunCircle();
    }
}


void aim_distance_select(void)
{
    if (cross_type != CROSS_NONE) {
        aim_distance = cross_aim;
    } else if (circle_type != CIRCLE_NONE) {
        aim_distance = cricle_aim;
    } else if (barrier_type != BARRIER_NONE) {
        aim_distance = barrier_aim;
    }
}


void MidLineTrack(void){
    if (cross_type == CROSS_IN) {
        if (track_type == TRACK_LEFT) {
            mid_track = mid_left; // 这是为了预先分配内存
            GetMidLine_Left(pts_far_resample_left + far_Lpt0_rpts0s_id, pts_far_resample_left_count - far_Lpt0_rpts0s_id, mid_left, (int)round(ANGLEDIST / RESAMPLEDIST), K[0][0] * ROADWIDTH / 2);
            mid_track_count = pts_far_resample_left_count - far_Lpt0_rpts0s_id;
        } else {
            mid_track = mid_right; // 这是为了预先分配内存
            GetMidLine_Right(pts_far_resample_right + far_Lpt1_rpts1s_id, pts_far_resample_right_count - far_Lpt1_rpts1s_id, mid_right, (int)round(ANGLEDIST / RESAMPLEDIST), K[0][0] * ROADWIDTH / 2);
            mid_track_count = pts_far_resample_right_count - far_Lpt1_rpts1s_id;
        }
    }
    else if(barrier_type == BARRIER_BRIDGE_IN)
        {
            if (bridge_flag == 0)
            {
                if(bridge_Lpt0_rpts0s_id > bridge_Lpt1_rpts1s_id)
                {
                    track_type = TRACK_LEFT;
                }
                else
                {
                    track_type = TRACK_RIGHT;
                }
            }
            else if(bridge_flag == 1)
            {
                track_type = TRACK_RIGHT;
            }
            else if(bridge_flag == 2)
            {
                track_type = TRACK_LEFT;
            }

            if (track_type == TRACK_LEFT) {
                mid_track = mid_left; // 这是为了预先分配内存
                GetMidLine_Left(pts_bridge_resample_left,bridge_Lpt0_rpts0s_id, mid_left, (int)round(ANGLEDIST / RESAMPLEDIST), K[0][0] * ROADWIDTH / 2);
                mid_track_count = bridge_Lpt0_rpts0s_id;
            } else {
                mid_track = mid_right; // 这是为了预先分配内存
                GetMidLine_Right(pts_bridge_resample_right,bridge_Lpt1_rpts1s_id, mid_right, (int)round(ANGLEDIST / RESAMPLEDIST), K[0][0] * ROADWIDTH / 2);
                mid_track_count = bridge_Lpt1_rpts1s_id;
            }

            //单边桥不怎么需要补线，直接求曲率了
            float x[100] = {0};
            float y[100] = {0};

            for (int i = 0; i < mid_track_count; i++) {
                x[i] = mid_track[i][1];
                y[i] = mid_track[i][0];
            }

            if (mid_track_count <= 15&&cross_type != CROSS_IN) {
                curvature_ave = last_curvature_ave;
                curvature_now = last_curvature_now;
            } else {
                curvature_ave = calculate_curvature(x, y, mid_track_count);
                curvature_now = calculate_curvature_now(x, y);
            }
            last_curvature_ave = curvature_ave;
            last_curvature_now = curvature_now;
        }
    else {
        if(barrier_type == BARRIER_BESIDES_LEFT_BEGIN||barrier_type == BARRIER_BESIDES_RIGHT_BEGIN||barrier_type == BARRIER_BESIDES_LEFT_RUNNING||barrier_type == BARRIER_BESIDES_RIGHT_RUNNING)
         {
             if (track_type == TRACK_LEFT) {
                 mid_track       = mid_left_barrier;
                 mid_track_count = mid_left_barrier_count;
             } else {
                 mid_track       = mid_right_barrier;
                 mid_track_count = mid_right_barrier_count;
             }
         }
         else
         {
             if (track_type == TRACK_LEFT) {
                 mid_track       = mid_left;
                 mid_track_count = mid_left_count;
             } else {
                 mid_track       = mid_right;
                 mid_track_count = mid_right_count;
             }
         }

        float x[100] = {0};
        float y[100] = {0};

        for (int i = 0; i < mid_track_count; i++) {
            x[i] = mid_track[i][1];
            y[i] = mid_track[i][0];
        }

        if (mid_track_count <= 15&&cross_type != CROSS_IN) {
            curvature_ave = last_curvature_ave;
            curvature_now = last_curvature_now;
        } else {
            curvature_ave = calculate_curvature(x, y, mid_track_count);
            curvature_now = calculate_curvature_now(x, y);
        }
        last_curvature_ave = curvature_ave;
        last_curvature_now = curvature_now;
    }

    //以下为偏移角度计算
        // 车轮对应点 (纯跟踪起始点)
//    adaptive_reverse_perspective((int)(IMAGE_H * 1.0f),(int)K[0][2]);
    float cx,cy;


    cx = K[0][2]+4;
    cy = IMAGE_H;
//    if(track_type == TRACK_LEFT)
//    {
//        cx = K[0][2] +6;
//        cy = IMAGE_H;
//    }
//    else if(track_type == TRACK_RIGHT)
//    {
//        cx = K[0][2]-1;
//        cy = IMAGE_H;
//    }

    int neary = (int)mid_track[0][0];
    int nearx = (int)mid_track[0][1];

    // 找最近点 (起始点中线归一化)
    float min_dist = 1e10;

    int begin_id = -1;
    for (int i = 0; i < mid_track_count; i++) {
        float dx   = mid_track[i][1] - cx;
        float dy   = mid_track[i][0] - cy;
        float dist = Q_sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            begin_id = i;
        }
    }

    if (begin_id >= 0 && mid_track_count - begin_id >= 3) {
        // 归一化中线
        mid_track[begin_id][0] = cy;
        mid_track[begin_id][1] = cx;
        rptsn_num              = sizeof(rptsn) / sizeof(rptsn[0]);

        GetLinesResample(mid_track + begin_id, mid_track_count - begin_id, rptsn, &rptsn_num, RESAMPLEDIST * PIXPERMETER);

        //十字路口中的时候计算曲率
            if (cross_type == CROSS_IN) {
                float x[rptsn_num];
                float y[rptsn_num];

                for (int i = 0; i < rptsn_num; i++) {
                    x[i] = rptsn[i][1];
                    y[i] = rptsn[i][0];
                }
                if (rptsn_num <= 15) {
                    curvature_ave = last_curvature_ave;
                    curvature_now = last_curvature_now;

                } else {
                    curvature_ave = calculate_curvature(x, y, rptsn_num);
                    curvature_now = calculate_curvature_now(x, y);
                }
                last_curvature_ave = curvature_ave;
                last_curvature_now = curvature_now;
            }

            if (barrier_type == BARRIER_JUMP_BEGIN)
            {
                mid_track_count -= (Lpt0_rpts0s_id_barrier + Lpt1_rpts1s_id_barrier)/2;
            }
//        tft180_show_float(0, 120, curvature, 3, 5);
//        float K_curvature = 0.8;
//        preview_distance = 0;
        standardized_curvature_now = kalman_update(&KF_curvature_now, fclip(curvature_now * 100.0f, -4.0f, 4.0f));
        standardized_curvature_ave = kalman_update(&KF_curvature_ave, fclip(curvature_ave * 100.0f, -4.0f, 4.0f));
//        float K_lateral_deviation = 0.05f;
        if(Lpt1_found_barrier && Lpt0_found_barrier){rptsn_num = Lpt0_rpts0s_id_barrier < Lpt1_rpts1s_id_barrier ? Lpt0_rpts0s_id_barrier : Lpt1_rpts1s_id_barrier;}
        if(rptsn_num - 1 >= 10)
        {
        float relative_slope = leastSquaresSlope((rptsn + 10), 5);
        float line_point[2] = { 0 };
        float car_point[2] = { cy , cx };
        center_point((rptsn + 10), 5, line_point);
        float distance = calculate_distance(line_point, relative_slope, car_point) / PIXPERMETER;

        lateral_deviation = kalman_update(&KF_lateral_deviation, fclip(distance , -0.25f, 0.25f));
//        lateral_deviation = kalman_update(&KF_lateral_deviation, distance);

        }
        else
        {
            lateral_deviation = kalman_update(&KF_lateral_deviation, fclip((rptsn[2][1] - cx) / PIXPERMETER, -0.25f, 0.25f));
//            lateral_deviation = kalman_update(&KF_lateral_deviation, (rptsn[2][1] - cx) / PIXPERMETER);
        }
//        Instantaneous_speed = 0.9;
        if(fabsf(standardized_curvature_ave) < 0.4)
        {
            preview_distance = Instantaneous_speed * Instantaneous_speed * K_v_sq + K_v * Instantaneous_speed + K_curvature * fabsf(standardized_curvature_ave) - K_lateral_deviation_straight * fabsf(lateral_deviation);
        }
        else
        {
            if(standardized_curvature_ave < 0)
            {
                preview_distance = Instantaneous_speed * Instantaneous_speed * K_v_sq + K_v * Instantaneous_speed + K_curvature * fabsf(standardized_curvature_ave) + K_lateral_deviation * lateral_deviation;
            }
            else
            {
                preview_distance = Instantaneous_speed * Instantaneous_speed * K_v_sq + K_v * Instantaneous_speed + K_curvature * fabsf(standardized_curvature_ave) - K_lateral_deviation * lateral_deviation;
            }
        }
        preview_distance = fmaxf(fabsf(preview_distance), 0.25);  // 避免过小


//        preview_distance = Instantaneous_speed * Instantaneous_speed / 2.0f / braking_acceleration + 2.0f * pit_time0_ms / 1000.0f * Instantaneous_speed + K_curvature * standardized_curvature - K_lateral_deviation * lateral_deviation;
//        preview_distance = Instantaneous_speed * Instantaneous_speed * K_v_sq + K_v * Instantaneous_speed + K_curvature * standardized_curvature - K_lateral_deviation * lateral_deviation;

//        preview_distance = 0.5;
//        preview_distance *= K[0][0];


        // 远预锚点位置-
//        int aim_idx       = clip(round(aim_distance / RESAMPLEDIST), 0, rptsn_num - 1);
        int aim_idx_judge = clip(round(aim_judge_far / RESAMPLEDIST), 0, mid_track_count - 1);

        // 近锚点位置
        //int aim_idx_near = clip(round(aim_distance / 2 / RESAMPLEDIST), 0, rptsn_num - 1);

        float dx1     = mid_track[3 * (mid_track_count / 4)][1] - mid_track[aim_idx_judge][1];
        float dy1     = mid_track[3 * (mid_track_count / 4)][0] - mid_track[aim_idx_judge][0];
        float dn1     = Q_sqrt(dx1 * dx1 + dy1 * dy1);
        float dx2     = mid_track[aim_idx_judge][1] - nearx;
        float dy2     = mid_track[aim_idx_judge][0] - neary;
        float dn2     = Q_sqrt(dx2 * dx2 + dy2 * dy2);
        float c1      = dx1 / dn1;
        float s1      = dy1 / dn1;
        float c2      = dx2 / dn2;
        float s2      = dy2 / dn2;
        float angle_1 = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);

        if (angle_1 >= 0.2f || angle_1 <= -0.2f) {
            state_type = TURN_STATE;
        } else {
            state_type = STRAIGHT_STATE;
        }

        // 计算远锚点偏差值
        int aim_idx       = clip(round(preview_distance / RESAMPLEDIST), 1, rptsn_num - 1);
        float estimate_distance = (cy - rptsn[aim_idx][0]) / PIXPERMETER;
        is_supplement_line = 0;
        if(fabsf(estimate_distance - preview_distance) >= RESAMPLEDIST)
        {
            if(estimate_distance - preview_distance > 0)
            {
                while(estimate_distance - preview_distance >= RESAMPLEDIST && aim_idx > 1)
                {
                    aim_idx--;
                    estimate_distance = (cy - rptsn[aim_idx][0]) / PIXPERMETER;
                }
            }
            else
            {
                while(estimate_distance - preview_distance < RESAMPLEDIST)
                {
                    if(cy - rptsn[aim_idx][0] > cy - rptsn[++aim_idx][0])
                    {
                        is_supplement_line = 1;
                        break;
                    }
//                    aim_idx++;
                    estimate_distance = (cy - rptsn[aim_idx][0]) / PIXPERMETER;
                    if(aim_idx >= rptsn_num - 1)
                    {
                        is_supplement_line = 1;
                        break;
                    }
                }
            }
        }
        if(cross_type != CROSS_NONE)
        {
            aim_idx = cross_aim / RESAMPLEDIST;
        }
        float dx = rptsn[aim_idx][1] - cx;
        float dy = cy - rptsn[aim_idx][0];
//        printf("%f,%f,%f,%d,%d\n",dy,preview_distance,dy/PIXPERMETER-preview_distance,aim_idx,is_supplement_line);
//        if(is_supplement_line)
//        {
//            dx *= (preview_distance / estimate_distance * (fabsf(standardized_curvature_ave)>1.0f?fabsf(standardized_curvature_ave):1.0f));
//        }
//        if(cross_type != CROSS_IN)
//        {
//            if(preview_distance / RESAMPLEDIST > rptsn_num - 1){is_supplement_line = 1;}
//            else{is_supplement_line = 0;}
        if(is_supplement_line)
        {
//            if(fabsf(standardized_curvature_ave) > 0.4f)
//            {
                dx *= (preview_distance / estimate_distance * (fabsf(standardized_curvature_ave) > 1.0f ? fabsf(standardized_curvature_ave) : 1.0f));
//                dx *= preview_distance / estimate_distance;
//                if(dx >= 0)
//                {
//                    dx += PIXPERMETER * (sqrtf(fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave - dy * dy / PIXPERMETER / PIXPERMETER )) - sqrtf(fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave - preview_distance * preview_distance)));
//                }
//                else
//                {
//                    dx -= PIXPERMETER * (sqrtf(fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave - dy * dy / PIXPERMETER / PIXPERMETER )) - sqrtf(fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave - preview_distance * preview_distance)));
//                }
//            }
        }
//        if(is_supplement_line && dx >= 0)
//        {
//            dx += sqrtf( fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave * K[0][0] * K[0][0]  - ((float)rptsn_num - 1.0f)*((float)rptsn_num - 1.0f))/ RESAMPLEDIST/ RESAMPLEDIST) - K[0][0] * sqrtf( fabsf(1.0f / standardized_curvature_ave / standardized_curvature_ave - preview_distance*preview_distance));
//                    }
//        else if(is_supplement_line && dx < 0)
//        {
//            dx -= sqrtf( fabsf( 1.0f / standardized_curvature_ave / standardized_curvature_ave * K[0][0] * K[0][0] - ((float)rptsn_num - 1.0f)*((float)rptsn_num - 1.0f))/ RESAMPLEDIST/ RESAMPLEDIST) - K[0][0] * sqrtf( fabsf(1.0f / standardized_curvature_ave / standardized_curvature_ave - preview_distance*preview_distance));
//        }
//        }
//        is_supplement_line = (preview_distance / RESAMPLEDIST > rptsn_num - 1) ? 1 : 0;



//        if (is_supplement_line && cross_type != CROSS_IN) {
//            float curvature_inv = (fabsf(standardized_curvature_ave) > 0.01f) ?
//                                 1.0f / standardized_curvature_ave : 100.0f;  // 防除零
//            float path_radius = curvature_inv * K[0][0];
//            float max_dist = (rptsn_num - 1) * RESAMPLEDIST;

            // 计算补线偏移量（几何修正）
//            float supplement_offset = sqrtf(fabsf(path_radius * path_radius - max_dist * max_dist));
//            dx += (dx >= 0) ? supplement_offset : -supplement_offset;
//            if(is_supplement_line && dx >= 0)
//            {
//                dx += sqrtf( fabsf( path_radius * path_radius  - ((float)rptsn_num - 1.0f)*((float)rptsn_num - 1.0f))/ RESAMPLEDIST/ RESAMPLEDIST) - K[0][0] * sqrtf( fabsf(curvature_inv * curvature_inv - preview_distance*preview_distance));
//            }
//            else if(is_supplement_line && dx < 0)
//            {
//                dx -= sqrtf( fabsf( path_radius * path_radius - ((float)rptsn_num - 1.0f)*((float)rptsn_num - 1.0f))/ RESAMPLEDIST/ RESAMPLEDIST) - K[0][0] * sqrtf( fabsf(curvature_inv * curvature_inv - preview_distance*preview_distance));
//            }
//        }

//        float dy = cy - rptsn[aim_idx][0]; // + 0.2f * PIXPERMETER;


        float dn = (dx * dx + dy * dy);
        //float temp_near = 0;

        switch (barrier_type) {
            case BARRIER_BESIDES_LEFT_BEGIN:
                // dx_near    = mid_track[aim_idx_near][1] - cx + barrier_offset;
                pure_angle = -atanf(PIXPERMETER * 2.0f * 0.2f * 0.5f * dx / dn);
                break;
            case BARRIER_BESIDES_RIGHT_BEGIN:
                pure_angle = -atanf(PIXPERMETER * 2.0f * 0.2f * 0.5f * dx / dn) ;
                break;
            default:
                // pure_angle  = -atanf(PIXPERMETER * 2.0f * 0.2f * 0.5f * dx / dn) / PI32 * 180.0f;
                if (dy > 0) {

                    pure_angle      = -atanf(dx / dy);
                    last_pure_angle = pure_angle;

                    // last_pure_angle_half = pure_angle_half;
                } else {
                    pure_angle = last_pure_angle;
                    // pure_angle_half = last_pure_angle_half;
                }
        }
        if (garage_type == GARAGE_FOUND) {
            pure_angle = 0.f;
        }
        if (state_type == STRAIGHT_STATE || state_type == CIRCLE_STATE ){
            pure_angle /= 1;//角度减半，减少平移带来的偏差影响
        }
//        if (cross_type == CROSS_BEGIN||cross_type == CROSS_IN)
//        {
//            if(abs(pure_angle)>=0.15)
//            {
//                pure_angle = -0.5;
//            }
//        }
        if (cross_type == CROSS_IN) {
            if(mid_track_count<=0)
                pure_angle = 0;//进入十字瞬间会有短暂丢线，导致角度瞬间变大并固定，此处为减少变大影响
            if(abs(pure_angle)>=0.5)
            {
                pure_angle = 0;
            }
        }
        if (barrier_type == BARRIER_JUMP_BEGIN)
        {
            if(abs(pure_angle)>=0.1)
            {
                pure_angle = 0;
            }
        }
        if (circle_type == CIRCLE_LEFT_RUNNING)
        {
            if(pts_left_count<5||pts_right_count<5)
            {
                pure_angle = last_pure_angle;
            }
        }
    }


    //以下为大模式切换和描点切换
    if (circle_type == CIRCLE_LEFT_IN || circle_type == CIRCLE_RIGHT_IN || circle_type == CIRCLE_LEFT_BEGIN || circle_type == CIRCLE_RIGHT_BEGIN) {
        state_type = CIRCLE_STATE;
        Atimer_clear(3);
        Atimer_start(3);
    }
    if (circle_type == CIRCLE_LEFT_RUNNING || circle_type == CIRCLE_RIGHT_RUNNING || circle_type == CIRCLE_RIGHT_OUT || circle_type == CIRCLE_LEFT_OUT) {
        if (circle_flag == 1) {
            state_type = CIRCLE_STATE;
        }

        uint16 ti = Atimer_get(3);
        if (ti >= 1000) {
            Atimer_stop(3);
            Atimer_clear(3);

            circle_flag = 0;
        }
        if (circle_flag == 0) {
            state_type = CIRCLE_RUNNING_STATE;
        }
    }

    if (circle_type == CIRCLE_NONE) {
        circle_flag = 1;
    }

    if (circle_type == CIRCLE_LEFT_END || circle_type == CIRCLE_RIGHT_END) {
        state_type = STRAIGHT_STATE;
    }

//    if (barrier_type == BARRIER_LEFT_BEGIN || barrier_type == BARRIER_LEFT_RUNNING || barrier_type == BARRIER_RIGHT_BEGIN || barrier_type == BARRIER_RIGHT_RUNNING) {
//        state_type = BARRIER_STATE;
//    }

    if (cross_type == CROSS_BEGIN || cross_type == CROSS_IN) {
        state_type = STRAIGHT_STATE;
    }

    if (state_type == STRAIGHT_STATE) {
        aim_distance = straight_aim;
    } else if (state_type == TURN_STATE) {
        aim_distance = turn_aim;
        /*
        if (fabs(curvature * 1000.f) >= turn_threshold) {
            aim_distance = turn_aim;
        }
        else if (fabs(curvature * 1000.f) < turn_threshold) {
            aim_distance = mid_aim;
        }
        */
    }
    if (cross_type != CROSS_NONE) {
        aim_distance = cross_aim;
    }

    if (garage_type == GARAGE_FOUND) {
        last_garage_type = GARAGE_FOUND;
    }

    if (last_garage_type == GARAGE_FOUND) {
        pingbi_num++;
        if (pingbi_num >= 300) {
            pingbi_num = 300;
        }

        if (pingbi_num >= 100) {
            if (state_type != BARRIER_STATE || garage_type != GARAGE_FOUND) {
                if ((pts_left_count <= 2) && (pts_right_count <= 2) && (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_L, IMAGE_W / 2 - BEGINW_R) <= FIX_BINTHRESHOLD) && (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_L, IMAGE_W / 2 + BEGINW_L) <= FIX_BINTHRESHOLD)) {
                    garage_type = OUT_STOP;
                }
            }
        } else {
            if (garage_type == OUT_STOP) {
                garage_type = GARAGE_FOUND;
            }
        }
    } else {
        if (state_type != BARRIER_STATE || garage_type != GARAGE_FOUND) {
            if ((pts_left_count <= 2) && (pts_right_count <= 2) && (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_L, IMAGE_W / 2 - BEGINW_R) <= FIX_BINTHRESHOLD) && (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_L, IMAGE_W / 2 + BEGINW_L) <= FIX_BINTHRESHOLD)) {
                garage_type = OUT_STOP;
            }
        }
    }

    if(lost_type == LOST_YES){
        state_type = LOST_STATE;
    }

}
