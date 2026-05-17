/*
 * Adaptive_binarization.c
 *
 *  Created on: 2025年4月29日
 *      Author: lenovo
 */
//二值化
#include "headfile.h"

#define WIDTH 120
#define HEIGHT 188
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

static uint32_t integral_buffer[120][188];

void adaptive_threshold_integral(
    int height,
    int width,
    const uint8_t src[height][width],
    uint8_t dst[height][width],
    int window_size,
    int C)
{
    int half = window_size / 2;

    // 计算积分图
    for (int y = 0; y < height; y++) {
        uint32_t row_sum = 0;
        for (int x = 0; x < width; x++) {
            row_sum += src[y][x];
            if (y == 0)
                integral_buffer[y][x] = row_sum;
            else
                integral_buffer[y][x] = integral_buffer[y - 1][x] + row_sum;
        }
    }

    // 自适应阈值处理
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int x1 = MAX(x - half, 0);
            int y1 = MAX(y - half, 0);
            int x2 = MIN(x + half, width - 1);
            int y2 = MIN(y + half, height - 1);

            int area = (x2 - x1 + 1) * (y2 - y1 + 1);
            if (area == 0) area = 1;

            uint32_t A = (x1 > 0 && y1 > 0) ? integral_buffer[y1 - 1][x1 - 1] : 0;
            uint32_t B = (y1 > 0) ? integral_buffer[y1 - 1][x2] : 0;
            uint32_t C_ = (x1 > 0) ? integral_buffer[y2][x1 - 1] : 0;
            uint32_t D = integral_buffer[y2][x2];

            // 二值化部分改成这样：
            int sum = D - B - C_ + A;
            int mean = sum / area;

            dst[y][x] = (src[y][x] < (mean + C)) ? 0 : 255;
        }
    }
}
