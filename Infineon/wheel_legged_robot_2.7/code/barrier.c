/*
 * barrier.c
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */
#include "headfile.h"

enum barrier_type_e barrier_type = BARRIER_NONE;
enum s_type_e s_type             = S_NONE;
enum shield_type_e shield_type   = SHIELD_NONE;
float (*mid_track_s)[2];
int32_t mid_track_count_s;
int32_t None_Barrier = 0;
int32_t OUT_Barrier = 0;
int temp_min_s;
int temp_max_s;
int barrier_begin_flag = 0;
int barrier_count      = 0;
float transection_distance = -1;
float barrirer_time=1000.f;
int32 bridge_flag = 0;
int bridge_count = 0;

void CheckBarrier()
{
//    if (barrier_type == BARRIER_NONE && Lpt0_found_barrier_in && Lpt1_found_barrier_in && abs(Lpt1_rpts1s_barrier[0]-Lpt0_rpts0s_barrier[0])<=5) {
//        barrier_type = BARRIER_JUMP_BEGIN;
//        beel = 500;
//    }
//    if (Lpt0_found_barrier_in && Lpt1_found_barrier_in) {
//        barrier_type = BARRIER_BRIDGE_IN;
//    }
//    if (circle_type == CIRCLE_NONE && barrier_type == BARRIER_NONE && Lpt0_found_barrier_in && !Lpt1_found_barrier_in && is_straight1) {
//        barrier_type = BARRIER_BESIDES_LEFT_BEGIN;
//        beel = 500;
//    }
//    if (circle_type == CIRCLE_NONE && barrier_type == BARRIER_NONE && !Lpt0_found_barrier_in && Lpt1_found_barrier_in && is_straight0) {
//        barrier_type = BARRIER_BESIDES_RIGHT_BEGIN;
//        beel = 500;
//    }

}

void RunBarrier()
{
    //单边桥元素巡线
    if(barrier_type == BARRIER_BRIDGE_IN)
    {

        //判断初始巡线点是否为白，若不是更换巡线点
        if ((GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_BRIDGE_L, 74) > FIX_BINTHRESHOLD)&&(GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_BRIDGE_L, 149) > FIX_BINTHRESHOLD))
        {
            //该标志位用于判断初始巡线点的位置 0为中心 1为右边 2为左边
            bridge_flag = 0;
            int w1 = IMAGE_W / 2 , h1 = BEGINH_BRIDGE_L;
            pts_bridge_left_count = sizeof(pts_bridge_left) / sizeof(pts_bridge_left[0]);
            for (; w1 > 0; w1--) {
                if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1 - 1) < FIX_BINTHRESHOLD)
                    break;
            }

            if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1) >= FIX_BINTHRESHOLD){
                SearchLineAdaptive_Left(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h1, w1, pts_bridge_left, &pts_bridge_left_count);
            }
            else pts_bridge_left_count = 0;

            int w2 = IMAGE_W / 2, h2 = BEGINH_BRIDGE_L;
            pts_bridge_right_count = sizeof(pts_bridge_right) / sizeof(pts_bridge_right[0]);
            for (; w2 < IMAGE_W - 1; w2++) {
                if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2 + 1) < FIX_BINTHRESHOLD)
                    break;
            }
            if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2) >= FIX_BINTHRESHOLD){
                SearchLineAdaptive_Right(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h2, w2, pts_bridge_right, &pts_bridge_right_count);
            }
            else pts_bridge_right_count = 0;

            //图像自适应逆透视更新
            external_reference_update();
            for (int i = 0; i < pts_bridge_left_count; i++) {
                float InverseW,InverseH;
                if(i==5){}

                adaptive_reverse_perspective(pts_bridge_left[i][1],pts_bridge_left[i][0]);
                InverseH = IMAGE_H - inv[0] * K[1][1];
                InverseW = inv[1] * K[0][0] + K[0][2];

                pts_bridge_inv_l[i][1] = fclip(InverseW,0,IMAGE_W);
                pts_bridge_inv_l[i][0] = fclip(InverseH,0,IMAGE_H);
            }
            pts_bridge_inv_l_count = pts_bridge_left_count;

            for (int i = 0; i < pts_bridge_right_count; i++) {
                float InverseW,InverseH;
                adaptive_reverse_perspective(pts_bridge_right[i][1],pts_bridge_right[i][0]);
                InverseH = IMAGE_H - inv[0] * K[1][1];
                InverseW = inv[1] * K[0][0] + K[0][2];

                pts_bridge_inv_r[i][1] = fclip(InverseW,0,IMAGE_W);
                pts_bridge_inv_r[i][0] = fclip(InverseH,0,IMAGE_H);
            }
            pts_bridge_inv_r_count = pts_bridge_right_count;

            // 边线滤波
            GetLinesFilter(pts_bridge_inv_l, pts_bridge_inv_l_count, pts_bridge_filter_l, (int)round(FILTER_KERNELSIZE));
            pts_bridge_filter_l_count = pts_bridge_inv_l_count;
            GetLinesFilter(pts_bridge_inv_r, pts_bridge_inv_r_count, pts_bridge_filter_r, (int)round(FILTER_KERNELSIZE));
            pts_bridge_filter_r_count = pts_bridge_inv_r_count;

            // 边线等距采样
            pts_bridge_resample_left_count = sizeof(pts_bridge_resample_left) / sizeof(pts_bridge_resample_left[0]);
            GetLinesResample(pts_bridge_filter_l, pts_bridge_filter_l_count, pts_bridge_resample_left, &pts_bridge_resample_left_count, RESAMPLEDIST * PIXPERMETER);
            pts_bridge_resample_right_count = sizeof(pts_bridge_resample_right) / sizeof(pts_bridge_resample_right[0]);
            GetLinesResample(pts_bridge_filter_r, pts_bridge_filter_r_count, pts_bridge_resample_right, &pts_bridge_resample_right_count, RESAMPLEDIST * PIXPERMETER);

            int valid_bridge_points_count = 0;

            for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                int x = pts_bridge_resample_left[i][1];
                int y = pts_bridge_resample_left[i][0];

                if (y < IMAGE_H - 3 && x > 3 && x < IMAGE_W - 3 && y >= 3) {
                    pts_bridge_resample_left[valid_bridge_points_count][0] = pts_bridge_resample_left[i][0];
                    pts_bridge_resample_left[valid_bridge_points_count][1] = pts_bridge_resample_left[i][1];
                    valid_bridge_points_count++;
                }
            }
            pts_bridge_resample_left_count = valid_bridge_points_count;

            valid_bridge_points_count = 0;

            for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                int x = pts_bridge_resample_right[i][1];
                int y = pts_bridge_resample_right[i][0];

                if (y <= IMAGE_H - 3 && x >= 3 && x <= IMAGE_W - 3 && y >= 3) {
                    pts_bridge_resample_right[valid_bridge_points_count][0] = pts_bridge_resample_right[i][0];
                    pts_bridge_resample_right[valid_bridge_points_count][1] = pts_bridge_resample_right[i][1];
                    valid_bridge_points_count++;
                }
            }

            pts_bridge_resample_right_count = valid_bridge_points_count;

            // 边线角度变化率
            local_angle_points(pts_bridge_resample_left, pts_bridge_resample_left_count, bridge_angle_left, (int)round(ANGLEDIST / RESAMPLEDIST));
            bridge_angle_left_num = pts_bridge_resample_left_count;
            local_angle_points(pts_bridge_resample_right, pts_bridge_resample_right_count, bridge_angle_right, (int)round(ANGLEDIST / RESAMPLEDIST));
            bridge_angle_right_num = pts_bridge_resample_right_count;

            // 角度变化率非极大值抑制
            nms_angle(bridge_angle_left, bridge_angle_left_num, bridge_angle_new_left, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
            bridge_angle_new_left_num = bridge_angle_left_num;
            nms_angle(bridge_angle_right, bridge_angle_right_num, bridge_angle_new_right, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
            bridge_angle_new_right_num = bridge_angle_right_num;

            // 找远线上的L角点
            bridge_Lpt0_found = bridge_Lpt1_found = false;
            for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                if (bridge_angle_new_left[i] == 0) continue;
                int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                float conf = fabs(bridge_angle_left[i]) - (fabs(bridge_angle_left[im1]) + fabs(bridge_angle_left[ip1])) / 2;

                // L角点阈值
                if (bridge_Lpt0_found == false && (30. / 180. * PI32) < conf && conf < (140. / 180. * PI32) && i < 0.7 / RESAMPLEDIST) {
                    bridge_Lpt0_rpts0s_id = i;
                    bridge_Lpt0_found     = true;
                }
            }

            for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                if (bridge_angle_new_right[i] == 0) continue;
                int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                float conf = fabs(bridge_angle_right[i]) - (fabs(bridge_angle_right[im1]) + fabs(bridge_angle_right[ip1])) / 2;

                if (bridge_Lpt1_found == false && (30. / 180. * PI32) < conf && conf < 140. / 180. * PI32 && i < 0.7 / RESAMPLEDIST) {
                    bridge_Lpt1_rpts1s_id = i;
                    bridge_Lpt1_found     = true;
                }
            }
        }
        //判断初始巡线点是否为白，若不是更换巡线点
        else if (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_BRIDGE_L, 149) > FIX_BINTHRESHOLD)
        {
            //该标志位用于判断初始巡线点的位置 0为中心 1为右边 2为左边
            bridge_flag = 1;
            int w1 = 150 , h1 = BEGINH_BRIDGE_L;
            pts_bridge_left_count = sizeof(pts_bridge_left) / sizeof(pts_bridge_left[0]);
            for (; w1 > 0; w1--) {
                if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1 - 1) < FIX_BINTHRESHOLD)
                    break;
            }

            if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1) >= FIX_BINTHRESHOLD){
                SearchLineAdaptive_Left(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h1, w1, pts_bridge_left, &pts_bridge_left_count);
            }
            else pts_bridge_left_count = 0;

            int w2 = 150 , h2 = BEGINH_BRIDGE_L;
            pts_bridge_right_count = sizeof(pts_bridge_right) / sizeof(pts_bridge_right[0]);
            for (; w2 < IMAGE_W - 1; w2++) {
                if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2 + 1) < FIX_BINTHRESHOLD)
                    break;
            }
            if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2) >= FIX_BINTHRESHOLD){
                SearchLineAdaptive_Right(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h2, w2, pts_bridge_right, &pts_bridge_right_count);
            }
            else pts_bridge_right_count = 0;

            //图像自适应逆透视更新
            external_reference_update();
            for (int i = 0; i < pts_bridge_left_count; i++) {
                float InverseW,InverseH;
                if(i==5){}

                adaptive_reverse_perspective(pts_bridge_left[i][1],pts_bridge_left[i][0]);
                InverseH = IMAGE_H - inv[0] * K[1][1];
                InverseW = inv[1] * K[0][0] + K[0][2];

                pts_bridge_inv_l[i][1] = fclip(InverseW,0,IMAGE_W);
                pts_bridge_inv_l[i][0] = fclip(InverseH,0,IMAGE_H);
            }
            pts_bridge_inv_l_count = pts_bridge_left_count;

            for (int i = 0; i < pts_bridge_right_count; i++) {
                float InverseW,InverseH;
                adaptive_reverse_perspective(pts_bridge_right[i][1],pts_bridge_right[i][0]);
                InverseH = IMAGE_H - inv[0] * K[1][1];
                InverseW = inv[1] * K[0][0] + K[0][2];

                pts_bridge_inv_r[i][1] = fclip(InverseW,0,IMAGE_W);
                pts_bridge_inv_r[i][0] = fclip(InverseH,0,IMAGE_H);
            }
            pts_bridge_inv_r_count = pts_bridge_right_count;

            // 边线滤波
            GetLinesFilter(pts_bridge_inv_l, pts_bridge_inv_l_count, pts_bridge_filter_l, (int)round(FILTER_KERNELSIZE));
            pts_bridge_filter_l_count = pts_bridge_inv_l_count;
            GetLinesFilter(pts_bridge_inv_r, pts_bridge_inv_r_count, pts_bridge_filter_r, (int)round(FILTER_KERNELSIZE));
            pts_bridge_filter_r_count = pts_bridge_inv_r_count;

            // 边线等距采样
            pts_bridge_resample_left_count = sizeof(pts_bridge_resample_left) / sizeof(pts_bridge_resample_left[0]);
            GetLinesResample(pts_bridge_filter_l, pts_bridge_filter_l_count, pts_bridge_resample_left, &pts_bridge_resample_left_count, RESAMPLEDIST * PIXPERMETER);
            pts_bridge_resample_right_count = sizeof(pts_bridge_resample_right) / sizeof(pts_bridge_resample_right[0]);
            GetLinesResample(pts_bridge_filter_r, pts_bridge_filter_r_count, pts_bridge_resample_right, &pts_bridge_resample_right_count, RESAMPLEDIST * PIXPERMETER);

            int valid_bridge_points_count = 0;

            for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                int x = pts_bridge_resample_left[i][1];
                int y = pts_bridge_resample_left[i][0];

                if (y < IMAGE_H - 3 && x > 3 && x < IMAGE_W - 3 && y >= 3) {
                    pts_bridge_resample_left[valid_bridge_points_count][0] = pts_bridge_resample_left[i][0];
                    pts_bridge_resample_left[valid_bridge_points_count][1] = pts_bridge_resample_left[i][1];
                    valid_bridge_points_count++;
                }
            }
            pts_bridge_resample_left_count = valid_bridge_points_count;

            valid_bridge_points_count = 0;

            for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                int x = pts_bridge_resample_right[i][1];
                int y = pts_bridge_resample_right[i][0];

                if (y <= IMAGE_H - 3 && x >= 3 && x <= IMAGE_W - 3 && y >= 3) {
                    pts_bridge_resample_right[valid_bridge_points_count][0] = pts_bridge_resample_right[i][0];
                    pts_bridge_resample_right[valid_bridge_points_count][1] = pts_bridge_resample_right[i][1];
                    valid_bridge_points_count++;
                }
            }

            pts_bridge_resample_right_count = valid_bridge_points_count;

            // 边线角度变化率
            local_angle_points(pts_bridge_resample_left, pts_bridge_resample_left_count, bridge_angle_left, (int)round(ANGLEDIST / RESAMPLEDIST));
            bridge_angle_left_num = pts_bridge_resample_left_count;
            local_angle_points(pts_bridge_resample_right, pts_bridge_resample_right_count, bridge_angle_right, (int)round(ANGLEDIST / RESAMPLEDIST));
            bridge_angle_right_num = pts_bridge_resample_right_count;

            // 角度变化率非极大值抑制
            nms_angle(bridge_angle_left, bridge_angle_left_num, bridge_angle_new_left, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
            bridge_angle_new_left_num = bridge_angle_left_num;
            nms_angle(bridge_angle_right, bridge_angle_right_num, bridge_angle_new_right, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
            bridge_angle_new_right_num = bridge_angle_right_num;

            // 找远线上的L角点
            bridge_Lpt0_found = bridge_Lpt1_found = false;
            for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                if (bridge_angle_new_left[i] == 0) continue;
                int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                float conf = fabs(bridge_angle_left[i]) - (fabs(bridge_angle_left[im1]) + fabs(bridge_angle_left[ip1])) / 2;

                // L角点阈值
                if (bridge_Lpt0_found == false && (30. / 180. * PI32) < conf && conf < (140. / 180. * PI32) && i < 0.7 / RESAMPLEDIST) {
                    bridge_Lpt0_rpts0s_id = i;
                    bridge_Lpt0_found     = true;
                }
            }

            for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                if (bridge_angle_new_right[i] == 0) continue;
                int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                float conf = fabs(bridge_angle_right[i]) - (fabs(bridge_angle_right[im1]) + fabs(bridge_angle_right[ip1])) / 2;

                if (bridge_Lpt1_found == false && (30. / 180. * PI32) < conf && conf < 140. / 180. * PI32 && i < 0.7 / RESAMPLEDIST) {
                    bridge_Lpt1_rpts1s_id = i;
                    bridge_Lpt1_found     = true;
                }
            }
        }
        //判断初始巡线点是否为白，若不是更换巡线点
        else if (GET_PIX_1C(mt9v03x_image_copy[0], BEGINH_BRIDGE_L, 74) > FIX_BINTHRESHOLD)
         {
            //该标志位用于判断初始巡线点的位置 0为中心 1为右边 2为左边
             bridge_flag = 2;
             int w1 = 75 , h1 = BEGINH_BRIDGE_L;
             pts_bridge_left_count = sizeof(pts_bridge_left) / sizeof(pts_bridge_left[0]);
             for (; w1 > 0; w1--) {
                 if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1 - 1) < FIX_BINTHRESHOLD)
                     break;
             }

             if (GET_PIX_1C(mt9v03x_image_copy[0], h1, w1) >= FIX_BINTHRESHOLD){
                 SearchLineAdaptive_Left(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h1, w1, pts_bridge_left, &pts_bridge_left_count);
             }
             else pts_bridge_left_count = 0;

             int w2 = 75, h2 = BEGINH_BRIDGE_L;
             pts_bridge_right_count = sizeof(pts_bridge_right) / sizeof(pts_bridge_right[0]);
             for (; w2 < IMAGE_W - 1; w2++) {
                 if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2 + 1) < FIX_BINTHRESHOLD)
                     break;
             }
             if (GET_PIX_1C(mt9v03x_image_copy[0], h2, w2) >= FIX_BINTHRESHOLD){
                 SearchLineAdaptive_Right(mt9v03x_image_copy[0], SELFADAPT_KERNELSIZE, SELFADAPT_OFFSET, h2, w2, pts_bridge_right, &pts_bridge_right_count);
             }
             else pts_bridge_right_count = 0;

             //图像自适应逆透视更新
             external_reference_update();
             for (int i = 0; i < pts_bridge_left_count; i++) {
                 float InverseW,InverseH;
                 if(i==5){}

                 adaptive_reverse_perspective(pts_bridge_left[i][1],pts_bridge_left[i][0]);
                 InverseH = IMAGE_H - inv[0] * K[1][1];
                 InverseW = inv[1] * K[0][0] + K[0][2];

                 pts_bridge_inv_l[i][1] = fclip(InverseW,0,IMAGE_W);
                 pts_bridge_inv_l[i][0] = fclip(InverseH,0,IMAGE_H);
             }
             pts_bridge_inv_l_count = pts_bridge_left_count;

             for (int i = 0; i < pts_bridge_right_count; i++) {
                 float InverseW,InverseH;
                 adaptive_reverse_perspective(pts_bridge_right[i][1],pts_bridge_right[i][0]);
                 InverseH = IMAGE_H - inv[0] * K[1][1];
                 InverseW = inv[1] * K[0][0] + K[0][2];

                 pts_bridge_inv_r[i][1] = fclip(InverseW,0,IMAGE_W);
                 pts_bridge_inv_r[i][0] = fclip(InverseH,0,IMAGE_H);
             }
             pts_bridge_inv_r_count = pts_bridge_right_count;

             // 边线滤波
             GetLinesFilter(pts_bridge_inv_l, pts_bridge_inv_l_count, pts_bridge_filter_l, (int)round(FILTER_KERNELSIZE));
             pts_bridge_filter_l_count = pts_bridge_inv_l_count;
             GetLinesFilter(pts_bridge_inv_r, pts_bridge_inv_r_count, pts_bridge_filter_r, (int)round(FILTER_KERNELSIZE));
             pts_bridge_filter_r_count = pts_bridge_inv_r_count;

             // 边线等距采样
             pts_bridge_resample_left_count = sizeof(pts_bridge_resample_left) / sizeof(pts_bridge_resample_left[0]);
             GetLinesResample(pts_bridge_filter_l, pts_bridge_filter_l_count, pts_bridge_resample_left, &pts_bridge_resample_left_count, RESAMPLEDIST * PIXPERMETER);
             pts_bridge_resample_right_count = sizeof(pts_bridge_resample_right) / sizeof(pts_bridge_resample_right[0]);
             GetLinesResample(pts_bridge_filter_r, pts_bridge_filter_r_count, pts_bridge_resample_right, &pts_bridge_resample_right_count, RESAMPLEDIST * PIXPERMETER);

             int valid_bridge_points_count = 0;

             for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                 int x = pts_bridge_resample_left[i][1];
                 int y = pts_bridge_resample_left[i][0];

                 if (y < IMAGE_H - 3 && x > 3 && x < IMAGE_W - 3 && y >= 3) {
                     pts_bridge_resample_left[valid_bridge_points_count][0] = pts_bridge_resample_left[i][0];
                     pts_bridge_resample_left[valid_bridge_points_count][1] = pts_bridge_resample_left[i][1];
                     valid_bridge_points_count++;
                 }
             }
             pts_bridge_resample_left_count = valid_bridge_points_count;

             valid_bridge_points_count = 0;

             for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                 int x = pts_bridge_resample_right[i][1];
                 int y = pts_bridge_resample_right[i][0];

                 if (y <= IMAGE_H - 3 && x >= 3 && x <= IMAGE_W - 3 && y >= 3) {
                     pts_bridge_resample_right[valid_bridge_points_count][0] = pts_bridge_resample_right[i][0];
                     pts_bridge_resample_right[valid_bridge_points_count][1] = pts_bridge_resample_right[i][1];
                     valid_bridge_points_count++;
                 }
             }

             pts_bridge_resample_right_count = valid_bridge_points_count;

             // 边线角度变化率
             local_angle_points(pts_bridge_resample_left, pts_bridge_resample_left_count, bridge_angle_left, (int)round(ANGLEDIST / RESAMPLEDIST));
             bridge_angle_left_num = pts_bridge_resample_left_count;
             local_angle_points(pts_bridge_resample_right, pts_bridge_resample_right_count, bridge_angle_right, (int)round(ANGLEDIST / RESAMPLEDIST));
             bridge_angle_right_num = pts_bridge_resample_right_count;

             // 角度变化率非极大值抑制
             nms_angle(bridge_angle_left, bridge_angle_left_num, bridge_angle_new_left, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
             bridge_angle_new_left_num = bridge_angle_left_num;
             nms_angle(bridge_angle_right, bridge_angle_right_num, bridge_angle_new_right, (int)round(ANGLEDIST / RESAMPLEDIST) * 2 + 1);
             bridge_angle_new_right_num = bridge_angle_right_num;

             // 找远线上的L角点
             bridge_Lpt0_found = bridge_Lpt1_found = false;
             for (int i = 0; i < pts_bridge_resample_left_count; i++) {
                 if (bridge_angle_new_left[i] == 0) continue;
                 int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                 int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_left_count - 1);
                 float conf = fabs(bridge_angle_left[i]) - (fabs(bridge_angle_left[im1]) + fabs(bridge_angle_left[ip1])) / 2;

                 // L角点阈值
                 if (bridge_Lpt0_found == false && (30. / 180. * PI32) < conf && conf < (140. / 180. * PI32) && i < 0.7 / RESAMPLEDIST) {
                     bridge_Lpt0_rpts0s_id = i;
                     bridge_Lpt0_found     = true;
                 }
             }

             for (int i = 0; i < pts_bridge_resample_right_count; i++) {
                 if (bridge_angle_new_right[i] == 0) continue;
                 int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                 int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_bridge_resample_right_count - 1);
                 float conf = fabs(bridge_angle_right[i]) - (fabs(bridge_angle_right[im1]) + fabs(bridge_angle_right[ip1])) / 2;

                 if (bridge_Lpt1_found == false && (30. / 180. * PI32) < conf && conf < 140. / 180. * PI32 && i < 0.7 / RESAMPLEDIST) {
                     bridge_Lpt1_rpts1s_id = i;
                     bridge_Lpt1_found     = true;
                 }
             }

         }
        //三点全寄
        else
        {

        }

        if(!Lpt0_found_barrier_in && !Lpt1_found_barrier_in)
        {
            bridge_count ++;
            if(bridge_count > 5)
            {
                beel = 2000;
                barrier_type = BARRIER_NONE;
            }
        }

    }

    if (barrier_type == BARRIER_BESIDES_LEFT_BEGIN || barrier_type == BARRIER_BESIDES_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;
        if (barrier_type == BARRIER_BESIDES_LEFT_BEGIN)
        {
            Atimer_clear(2);
            Atimer_start(2);
            barrier_type = BARRIER_BESIDES_LEFT_RUNNING;
        }
        if (barrier_type == BARRIER_BESIDES_LEFT_RUNNING)
        {
            if (Atimer_get(2) >= barrirer_time)
            {
                barrier_type = BARRIER_NONE;
                Atimer_stop(2);
                Atimer_clear(2);
                beel =500;
            }
            else{
                barrier_type = BARRIER_BESIDES_LEFT_RUNNING;
            }
        }


    }

    if (barrier_type == BARRIER_BESIDES_RIGHT_BEGIN || barrier_type == BARRIER_BESIDES_RIGHT_RUNNING) {
        track_type = TRACK_LEFT;
        if (barrier_type == BARRIER_BESIDES_RIGHT_BEGIN)
        {
            Atimer_clear(2);
            Atimer_start(2);
            barrier_type = BARRIER_BESIDES_RIGHT_RUNNING;
        }
        if (barrier_type == BARRIER_BESIDES_RIGHT_RUNNING)
        {
            if (Atimer_get(2) >= barrirer_time)
            {
                barrier_type = BARRIER_NONE;
                Atimer_stop(2);
                Atimer_clear(2);
                beel =500;
            }
            else{
                barrier_type = BARRIER_BESIDES_RIGHT_RUNNING;
            }
        }

    }

    if(barrier_type == BARRIER_JUMP_BEGIN)
    {
        if(Lpt0_found_barrier_in&&Lpt1_found_barrier_in)
        {

            transection_distance = (IMAGE_H - pts_resample_left[Lpt0_rpts0s_id_barrier][0] + IMAGE_H - pts_resample_right[Lpt1_rpts1s_id_barrier][0]) / 2 / K[0][0];
            None_Barrier = 0;

        }
        else{
            None_Barrier++;

        }
        if(None_Barrier>=5||input.flag == 3)
        {
            transection_distance = -1;
            barrier_type = BARRIER_JUMP_OUT;
        }
    }
    else
    {
        transection_distance = -1;
    }

    if(barrier_type == BARRIER_JUMP_OUT)
    {
        if(!Lpt0_found_barrier_in&&!Lpt1_found_barrier_in)
        {
            transection_distance = -1;
            barrier_type = BARRIER_NONE;
        }
    }
}


void Check_s()
{
    if (track_type == TRACK_LEFT) {
        mid_track_s       = mid_left;
        mid_track_count_s = mid_left_count;
    } else {
        mid_track_s       = mid_right;
        mid_track_count_s = mid_right_count;
    }

    int min_s = 1000;
    int max_s = 1;

    for (int i = 0; i < mid_track_count_s; i++) {
        if (mid_track_s[i][1] < min_s) {
            min_s      = mid_track_s[i][1];
            temp_min_s = min_s;
        }
        if (mid_track_s[i][1] > max_s) {
            max_s      = mid_track_s[i][1];
            temp_max_s = max_s;
        }
    }

    if (is_straight0 == false && is_straight1 == false && (max_s - min_s) <= 30 && (max_s - min_s) >= 5) {
        s_type = S_BEGIN;
    }
}

void RunS()
{

    if (temp_max_s - temp_min_s > 30 || (is_straight0 && is_straight1)) {
        s_type = S_NONE;
    }
}
