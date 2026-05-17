/*
 * Adaptive_binarization.h
 *
 *  Created on: 2025Äê4ÔÂ29ÈÕ
 *      Author: lenovo
 */

#ifndef CODE_ADAPTIVE_BINARIZATION_H_
#define CODE_ADAPTIVE_BINARIZATION_H_

void adaptive_threshold_integral(
    int height,
    int width,
    const uint8_t src[height][width],
    uint8_t dst[height][width],
    int window_size,
    int C);



#endif /* CODE_ADAPTIVE_BINARIZATION_H_ */
