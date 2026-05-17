/*
 * get_corners.c
 *
 *  Created on: 2024年10月8日
 *      Author: A
 */
//找角点
#include "math.h"
#include "headfile.h"

float left_conf;
float right_conf;

void get_corners()
{
    Lpt0_found = Lpt1_found = false;
    Lpt_in0_found = Lpt_in1_found = false;
    is_straight0                  = pts_resample_left_count > 0.6 / RESAMPLEDIST;
    is_straight1                  = pts_resample_right_count > 0.6 / RESAMPLEDIST;
    // is_turn0_l = is_turn1_l = is_turn0_r = is_turn1_r = 0;
    for (int i = 0; i < pts_resample_left_count; i++) {
        if (angle_new_left[i] == 0) continue;
        int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_resample_left_count - 1);
        int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_resample_left_count - 1);
        float conf = fabs(angle_left[i]) - (fabs(angle_left[im1]) + fabs(angle_left[ip1])) / 2;
            // L 角点阈值
            if (Lpt0_found == false && (30. / 180. * PI32) < conf && conf < (140. / 180. * PI32) && i < 0.8 / RESAMPLEDIST&&pts_resample_left[i][1] >= pts_resample_left[clip(i + 2, 0, pts_resample_left_count - 1)][1])
        {
            Lpt0_rpts0s_id = i;
            Lpt0_rpts0s[1] = pts_resample_left[i][1];
            Lpt0_rpts0s[0] = 120 - pts_resample_left[i][0];
            Lpt0_found     = true;
            left_conf = conf/ 180. * PI32;
        }
            else
            {
                Lpt0_rpts0s_id = 0;
            }
        // 长直道阈值
        if (conf > (4. / 180. * PI32) && i < 0.8 / RESAMPLEDIST) {
            is_straight0 = false;
        }
        if (Lpt0_found == true && is_straight0 == false) break;

    }


    Lpt0_found_barrier = Lpt1_found_barrier = false;
    Lpt0_found_barrier_in = Lpt1_found_barrier_in = false;
    for (int i = 0; i < pts_resample_left_count; i++) {
        if (angle_new_left_barrier[i] == 0) continue;
        int im1    = clip(i - (int)round(ANGLEDIST_barrier / RESAMPLEDIST), 0, pts_resample_left_count - 1);
        int ip1    = clip(i + (int)round(ANGLEDIST_barrier / RESAMPLEDIST), 0, pts_resample_left_count - 1);
        float conf = fabs(angle_left_barrier[i]) - (fabs(angle_left_barrier[im1]) + fabs(angle_left_barrier[ip1])) / 2;

        // L 角点阈值
        if (Lpt0_found_barrier == false && (35. / 180. * PI32) < conf && conf < (160. / 180. * PI32) && i < 0.8 / RESAMPLEDIST) {
            Lpt0_rpts0s_id_barrier = i;
            if (pts_resample_left[i][1] <= pts_resample_left[clip(i + 2, 0, pts_resample_left_count - 1)][1]-2)
            {
                Lpt0_rpts0s_barrier[1] = pts_resample_left[i][1];
                Lpt0_rpts0s_barrier[0] = 120 - pts_resample_left[i][0];
                Lpt0_found_barrier     = true;
            }
            else{
                Lpt0_rpts0s_barrier[1] = 0;
                Lpt0_rpts0s_barrier[0] = 0;
                Lpt0_found_barrier     = false;
            }


        }
    }
    if (Lpt0_found_barrier) {
        float angle1 = calculate_vector_angle(pts_resample_left[Lpt0_rpts0s_id_barrier][1], pts_resample_left[Lpt0_rpts0s_id_barrier][0], pts_resample_left[Lpt0_rpts0s_id_barrier + 5][1], pts_resample_left[Lpt0_rpts0s_id_barrier + 5][0]);
        if (angle1 < 85.) {
            Lpt0_found_barrier_in    = true;
            Lpt0_found_barrier       = false;
            Lpt0_found_barrier_in_id = Lpt0_rpts0s_id_barrier;
        }
    }




    for (int i = 0; i < pts_resample_right_count; i++) {
        if (angle_new_right[i] == 0) continue;
        int im1    = clip(i - (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_resample_right_count - 1);
        int ip1    = clip(i + (int)round(ANGLEDIST / RESAMPLEDIST), 0, pts_resample_right_count - 1);
        float conf = fabs(angle_right[i]) - (fabs(angle_right[im1]) + fabs(angle_right[ip1])) / 2;

        if (Lpt1_found == false && (30. / 180. * PI32) < conf && conf < 140. / 180. * PI32 && i < 0.8 / RESAMPLEDIST&&pts_resample_right[i][1] <= pts_resample_right[clip(i + 2, 0, pts_resample_left_count - 1)][1]) {
            Lpt1_rpts1s_id = i;
            Lpt1_rpts1s[1] = pts_resample_right[i][1];
            Lpt1_rpts1s[0] = 120 - pts_resample_right[i][0];
            Lpt1_found     = true;
            right_conf = conf/ 180. * PI32;
            // transform(pts_resample_right[Lpt1_rpts1s_id][1],pts_resample_right[Lpt1_rpts1s_id][0],&Lpt1[1],&Lpt1[0]);
        }
        else
        {
            Lpt1_rpts1s_id = 0;
        }

        if (conf > (4. / 180. * PI32) && i < 0.8 / RESAMPLEDIST) is_straight1 = false;
        if (Lpt1_found == true && is_straight1 == false) break;

    }



    for (int i = 0; i < pts_resample_right_count; i++) {
        if (angle_new_right_barrier[i] == 0) continue;
        int im1    = clip(i - (int)round(ANGLEDIST_barrier / RESAMPLEDIST), 0, pts_resample_right_count - 1);
        int ip1    = clip(i + (int)round(ANGLEDIST_barrier / RESAMPLEDIST), 0, pts_resample_right_count - 1);
        float conf = fabs(angle_right_barrier[i]) - (fabs(angle_right_barrier[im1]) + fabs(angle_right_barrier[ip1])) / 2;

        if (Lpt1_found_barrier == false && (35. / 180. * PI32) < conf && conf < 160. / 180. * PI32 && i < 0.8 / RESAMPLEDIST) {
            Lpt1_rpts1s_id_barrier = i;

            if (pts_resample_right[i][1] >= pts_resample_right[clip(i + 2, 0, pts_resample_left_count - 1)][1]+2)
            {
                Lpt1_rpts1s_barrier[1] = pts_resample_right[i][1];
                Lpt1_rpts1s_barrier[0] = 120 - pts_resample_right[i][0];
                Lpt1_found_barrier     = true;
            }
            else{
                Lpt1_rpts1s_barrier[1] = 0;
                Lpt1_rpts1s_barrier[0] = 0;
                Lpt1_found_barrier     = false;
            }

        }
    }
    if (Lpt1_found_barrier) {
        float angle2 = calculate_vector_angle(pts_resample_right[Lpt1_rpts1s_id_barrier][1], pts_resample_right[Lpt1_rpts1s_id_barrier][0], pts_resample_right[Lpt1_rpts1s_id_barrier + 5][1], pts_resample_right[Lpt1_rpts1s_id_barrier + 5][0]);
        if (angle2 > 100.) {
            Lpt1_found_barrier_in    = true;
            Lpt1_found_barrier       = false;
            Lpt1_found_barrier_in_id = Lpt1_rpts1s_id_barrier;
        }
    }

}



