/*
 * circle.c
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */
//环岛
#include "headfile.h"

enum circle_type_e circle_type = CIRCLE_NONE;
int beel = 0;

int32_t Left_Border_None_Circle  = 0;
int32_t Right_Border_None_Circle = 0;

int32_t Left_Border_Have_Circle  = 0;
int32_t Right_Border_Have_Circle = 0;

int32_t Left_Border_ToLeft_Circle  = 0;
int32_t Right_Border_ToLeft_Circle = 0;

int32_t Left_Border_ToRight_Circle  = 0;
int32_t Right_Border_ToRight_Circle = 0;

int32_t ceshi = 0;

void CheckCircle()
{
    // 非圆环模式下，单边L角点, 单边长直道(暂时删去长直道，避免过弯后立刻接环岛无法识别)
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1) {
        circle_type = CIRCLE_LEFT_BEGIN;
        beel = 500;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0) {
        circle_type = CIRCLE_RIGHT_BEGIN;
        beel = 500;
    }
}


void RunCircle(){
    if (circle_type == CIRCLE_LEFT_BEGIN) // 左环开始，寻外直道右线
    {
        track_type = TRACK_RIGHT;

        // 先丢左线后有线
        if (pts_resample_left_count < 0.06 / RESAMPLEDIST) {
            Left_Border_None_Circle++;
        }
        if (pts_resample_left_count > 0.2 / RESAMPLEDIST && Left_Border_None_Circle > FRAMENONE) {
            Left_Border_Have_Circle++;
            if (Left_Border_Have_Circle > FRAMENONE) {
                circle_type             = CIRCLE_LEFT_IN;
                Left_Border_None_Circle = 0;
                Left_Border_Have_Circle = 0;
//                beel = 500;
            }
        }
    } else if (circle_type == CIRCLE_LEFT_IN) // 入环，寻内圆左线
    {
        track_type = TRACK_LEFT;

        if (pts_resample_right[(int32_t)(0.2 / RESAMPLEDIST)][1] -
                pts_resample_right[0][1] <
            -2) {
            Right_Border_ToLeft_Circle++;
        }
        if (Right_Border_ToLeft_Circle > FRAMETOLEFT) {
            circle_type                = CIRCLE_LEFT_RUNNING;
            Right_Border_ToLeft_Circle = 0;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_LEFT_RUNNING) // 正常巡线，寻外圆右线
    {
         track_type = TRACK_RIGHT;
        //track_type = TRACK_LEFT; // 看看加一个如果丢线才切换
        if (Lpt1_found) {
            pts_resample_right_count = mid_right_count = Lpt1_rpts1s_id;
        }
        if (Lpt1_found && Lpt1_rpts1s_id < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_LEFT_OUT;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_LEFT_OUT) // 出环，寻内圆
    {
        track_type = TRACK_LEFT;

        if (is_straight1) // 右线为长直道
        {
            circle_type = CIRCLE_LEFT_END;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_LEFT_END) // 走过圆环，寻右线
    {
        track_type = TRACK_RIGHT;

        if (pts_resample_left_count < 0.1 / RESAMPLEDIST) // 左线先丢后有
        {
            Left_Border_None_Circle++;
        }
        if (pts_resample_left_count > 0.5 / RESAMPLEDIST &&
            Left_Border_None_Circle > FRAMENONE) {
            circle_type                 = CIRCLE_NONE;
            Left_Border_None_Circle     = 0;
            Left_Border_Have_Circle     = 0;
            Right_Border_ToLeft_Circle  = 0;
            Right_Border_ToRight_Circle = 0;
            beel = 500;
        }

    } else if (circle_type == CIRCLE_RIGHT_BEGIN) // 右环控制，前期寻左直道
    {
        track_type = TRACK_LEFT;

        // 先丢右线后有线
        if (pts_resample_right_count < 0.06 / RESAMPLEDIST) {
            Right_Border_None_Circle++;
        }
        if (pts_resample_right_count > 0.2 / RESAMPLEDIST && Right_Border_None_Circle > FRAMENONE) {
            Right_Border_Have_Circle++;
            if (Right_Border_Have_Circle > FRAMENONE) {
                circle_type              = CIRCLE_RIGHT_IN;
                Right_Border_None_Circle = 0;
                Right_Border_Have_Circle = 0;
//                beel = 500;
            }
        }
    } else if (circle_type == CIRCLE_RIGHT_IN) // 入右环，寻右内圆环
    {
        track_type = TRACK_RIGHT;

        if (pts_resample_left[(int32_t)(0.2 / RESAMPLEDIST)][1] -
                pts_resample_left[0][1] >
            2) {
            Left_Border_ToRight_Circle++;
        }
        if (Left_Border_ToRight_Circle > FRAMETORIGHT) {
            circle_type                = CIRCLE_RIGHT_RUNNING;
            Left_Border_ToRight_Circle = 0;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_RIGHT_RUNNING) // 正常巡线，寻外圆左线
    {
        track_type = TRACK_RIGHT;
        // track_type = TRACK_LEFT; // 看看加一个如果丢线才切换
        if (Lpt0_found) // 外环存在拐点，可再加拐点距离判据 (左 L 点)
        {
            pts_resample_left_count = mid_left_count = Lpt0_rpts0s_id;
        }
        if (Lpt0_found && Lpt0_rpts0s_id < 0.4 / RESAMPLEDIST) {
            circle_type = CIRCLE_RIGHT_OUT;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_RIGHT_OUT) // 出环，寻内圆
    {
        track_type = TRACK_RIGHT;

        if (is_straight0) // 加个有线长度判断
        {
            circle_type = CIRCLE_RIGHT_END;
//            beel = 500;
        }
    } else if (circle_type == CIRCLE_RIGHT_END) // 走过圆环，寻左线
    {
        track_type = TRACK_LEFT;

        if (pts_resample_right_count < 0.1 / RESAMPLEDIST) // 左线先丢后有
        {
            Right_Border_None_Circle++;
        }
        if (pts_resample_right_count > (0.5 / RESAMPLEDIST) && Right_Border_None_Circle > FRAMENONE) {
            circle_type                = CIRCLE_NONE;
            Right_Border_None_Circle   = 0;
            Right_Border_Have_Circle   = 0;
            Left_Border_ToLeft_Circle  = 0;
            Left_Border_ToRight_Circle = 0;
            beel = 500;
        }

    }
}
