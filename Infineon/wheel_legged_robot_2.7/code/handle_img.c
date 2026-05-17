/*
 * handle_img.c
 *
 *  Created on: 2024年10月7日
 *      Author: A
 */

#include "math.h"
#include "headfile.h"

const int32_t direction_front[4][2] = {{0,  -1},{1,  0},{0,  1},{-1, 0}};
const int32_t direction_frontleft[4][2] = {{-1, -1},{1,  -1},{1,  1},{-1, 1}};
const int32_t direction_frontright[4][2] ={{1,  -1},{1,  1},{-1, 1},{-1, -1}};


void GetImgBin_Average(uint8_t img_gray[], float *threshold)//均值法
{
    uint16 H, W;
    uint32 threshold_temp = 0;
    for (H = 0; H < IMAGE_H; H++)
    {
        for (W = 0; W < IMAGE_W; W++)
        {
            threshold_temp += GET_PIX_1C(img_gray, H, W);
        }
    }
    *threshold=(float)(threshold_temp / (IMAGE_H * IMAGE_W))+10;
}




void SearchLineAdaptive_Left(uint8_t img_gray[], int32_t block_size, int32_t down_value, int32_t h, int32_t w, int32_t pts[][2], int32_t* line_num)
{
    const int32_t half = block_size / 2;
    int step = 0;
    int dir = 0;
    int turn = 0;
    while (step < *line_num && half < w && w < IMAGE_W - half -1 && half < h && h < IMAGE_H - half -1 && turn < 4) {
        int local_thres = 0;
        for (int dh = -1 * half; dh <= half; dh++) {
            for (int dw = -1 * half; dw <= half; dw++) {
                local_thres += GET_PIX_1C(img_gray, h + dh, w + dw);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= down_value;

        int front_value = GET_PIX_1C(img_gray, h+direction_front[dir][1], w + direction_front[dir][0]);
        int frontleft_value = GET_PIX_1C(img_gray, h + direction_frontleft[dir][1], w + direction_frontleft[dir][0]);
        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            w += direction_front[dir][0];
            h += direction_front[dir][1];
            pts[step][1] = w;
            pts[step][0] = h;
            step++;
            turn = 0;
        } else {

            w += direction_frontleft[dir][0];
            h += direction_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][1] = w;
            pts[step][0] = h;
            step++;
            turn = 0;

        }


    }
    *line_num = step;
}

void SearchLineAdaptive_Right(uint8_t img_gray[], int32_t block_size, int32_t down_value, int32_t h, int32_t w, int32_t pts[][2], int32_t* line_num)
{
    int half = block_size / 2;
        int step = 0, dir = 0, turn = 0;
        while (step < *line_num && 0 < w && w < IMAGE_W - 1   && 0 < h && h < IMAGE_H -1  && turn < 4) {
            int local_thres = 0;
            for (int dh = -half; dh <= half; dh++) {
                for (int dw = -half; dw <= half; dw++) {
                    local_thres += GET_PIX_1C(img_gray, h + dh, w + dw);
                }
            }
            local_thres /= block_size * block_size;
            local_thres -= down_value;


            int front_value = GET_PIX_1C(img_gray, h + direction_front[dir][1], w + direction_front[dir][0]);
            int frontright_value = GET_PIX_1C(img_gray, h + direction_frontright[dir][1], w + direction_frontright[dir][0]);
            if (front_value < local_thres) {
                dir = (dir + 3) % 4;
                turn++;
            } else if (frontright_value < local_thres) {
                w += direction_front[dir][0];
                h += direction_front[dir][1];
                pts[step][1] = w;
                pts[step][0] = h;
                step++;
                turn = 0;


            } else {
                w += direction_frontright[dir][0];
                h += direction_frontright[dir][1];
                dir = (dir + 1) % 4;
                pts[step][1] = w;
                pts[step][0] = h;
                step++;
                turn = 0;


            }

        }
        *line_num = step;
}



void GetLinesFilter(float pts_in[][2], int32_t pts_in_count, float pts_out[][2], int32_t kernelsize)
{
        int half = kernelsize / 2;
        for (int i = 0; i < pts_in_count; i++) {
            pts_out[i][0] = 0;
            pts_out[i][1] = 0;
            for (int j = -half; j <= half; j++) {
                pts_out[i][0] += pts_in[clip(i + j, 0, pts_in_count - 1)][0] * (half + 1 - abs(j));
                pts_out[i][1] += pts_in[clip(i + j, 0, pts_in_count - 1)][1] * (half + 1 - abs(j));
            }
            pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
            pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
        }
}

void GetLinesResample(float pts_in[][2], int32_t num1, float pts_out[][2], int32_t* num2, float dist)
{
    if (num1 < 0) {
            *num2 = 0;
            return;
        }
        pts_out[0][0] = pts_in[0][0];
        pts_out[0][1] = pts_in[0][1];
        int len = 1;
        for (int i = 0; i < num1 - 1 && len < *num2; i++) {
            float x0 = pts_in[i][1];
            float y0 = pts_in[i][0];
            float x1 = pts_in[i + 1][1];
            float y1 = pts_in[i + 1][0];

            do {
                float x = pts_out[len - 1][1];
                float y = pts_out[len - 1][0];

                float dx0 = x0 - x;
                float dy0 = y0 - y;
                float dx1 = x1 - x;
                float dy1 = y1 - y;

                float dist0 = Q_sqrt(dx0 * dx0 + dy0 * dy0);
                float dist1 = Q_sqrt(dx1 * dx1 + dy1 * dy1);

                float r0 = (dist1 - dist) / (dist1 - dist0);
                float r1 = 1 - r0;

                if (r0 < 0 || r1 < 0) break;
                x0 = x0 * r0 + x1 * r1;
                y0 = y0 * r0 + y1 * r1;
                pts_out[len][1] = x0;
                pts_out[len][0] = y0;
                len++;
            } while (len < *num2);

        }
        *num2 = len;
}

//*************************** 函数中x,y的位置似乎反了 *************************//
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);   //得到弧度 类似angle_out = 0.3491
    }
}

void nms_angle(float angle_in[], int num, float angle_out[], int kernel) {

    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

//*********************************** 交叉相加减，dx接近与0导致y基本不变；dy接近于1导致x的值变得较多 *********************************//
void GetMidLine_Left(float pts_left[][2], int32_t pts_left_count, float mid_left[][2], int32_t approx_num, float dist)
{
    for (int i = 0; i < pts_left_count; i++) {
        float dx = pts_left[clip(i + approx_num, 0, pts_left_count - 1)][1] - pts_left[clip(i - approx_num, 0, pts_left_count - 1)][1];
        float dy = pts_left[clip(i + approx_num, 0, pts_left_count - 1)][0] - pts_left[clip(i - approx_num, 0, pts_left_count - 1)][0];
        float dn = Q_sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        mid_left[i][0] = pts_left[i][0] + dx * dist;
        mid_left[i][1] = pts_left[i][1] - dy * dist;
    }

}


void GetMidLine_Right(float pts_right[][2], int32_t pts_right_count, float mid_right[][2], int32_t approx_num, float dist)
{
    for (int i = 0; i < pts_right_count; i++) {
        float dx = pts_right[clip(i + approx_num, 0, pts_right_count -1)][1] - pts_right[clip(i - approx_num, 0, pts_right_count -1 )][1];
        float dy = pts_right[clip(i + approx_num, 0, pts_right_count -1)][0] - pts_right[clip(i - approx_num, 0, pts_right_count -1)][0];
        float dn = Q_sqrt(dx * dx + dy * dy);
        dx /= dn;//sin
        dy /= dn;//cos
            mid_right[i][0] = pts_right[i][0] - dx * dist;
            mid_right[i][1] = pts_right[i][1] + dy * dist;


    }
}


int is_curve(float angle[], int n, float threshold) {
    for (int i = 1; i < n - 1; i++) {
        float da = fabs(angle[i] - angle[i-3]);
        float db = fabs(angle[i+3] - angle[i]);
        if (da > threshold && db > threshold) {
            return 1; // 是弯道
        }
    }
    return 0; // 不是弯道
}


