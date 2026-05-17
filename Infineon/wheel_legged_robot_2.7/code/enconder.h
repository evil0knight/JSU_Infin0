/*
 * enconder.h
 *
 *  Created on: 2025��4��15��
 *      Author: lenovo
 */

#ifndef CODE_ENCONDER_H_
#define CODE_ENCONDER_H_

#include "zf_common_headfile.h"

#define encoder_A   (P13_3) /*A*/
#define encoder_B   (P13_0) /*B*/

typedef enum{
    encoder_none,
    encoder_foreward,
    encoder_reversal
}encoder_status_enum;

typedef struct {
    gpio_pin_enum encoder_A_port;
    gpio_pin_enum encoder_B_port;
    encoder_status_enum encoder_flag;
    encoder_status_enum encoder_prev_flag;
    encoder_status_enum encoder_output;

    int encoder_gpio[2];
    int encoder_prev_gpio[2];
}encoder_struct;

extern encoder_struct encoder;

void my_encoder_init(void);
void scan_encoder_status(encoder_struct *encoder);
int get_encoder_status(void);

#endif /* CODE_ENCONDER_H_ */
