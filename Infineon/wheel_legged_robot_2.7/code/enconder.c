
/*
 * enconder.c
 *
 *  Created on: 2025??4??15??
 *      Author: lenovo
 */

#include "enconder.h"

encoder_struct encoder = { 0 };

void my_encoder_init(void)
{
    gpio_init(encoder_A, GPI, 1, GPI_PULL_UP);
    gpio_init(encoder_B, GPI, 1, GPI_PULL_UP);

    encoder.encoder_A_port = encoder_A;
    encoder.encoder_B_port = encoder_B;

    encoder.encoder_output = encoder_none;
    encoder.encoder_flag = encoder_none;
    encoder.encoder_prev_flag = encoder_none;
    int i;
    for(i = 0; i < 2; i++)
    {
        encoder.encoder_gpio[i] = 0;
        encoder.encoder_prev_gpio[i] = 0;
    }

}

void scan_encoder_status(encoder_struct *encoder)
{
    encoder->encoder_gpio[0] = gpio_get_level(encoder->encoder_A_port);
    encoder->encoder_gpio[1] = gpio_get_level(encoder->encoder_B_port);

    if(encoder->encoder_gpio[0] == 1 && encoder->encoder_gpio[1] == 1)
    {
        encoder->encoder_output = encoder_none;
        encoder->encoder_flag = encoder_none;
    }
    else if(encoder->encoder_prev_gpio[0] == 1 && encoder->encoder_gpio[0] == 0 && encoder->encoder_gpio[1] == 1)
    {
        encoder->encoder_flag = encoder_foreward;
    }
    else if(encoder->encoder_prev_gpio[1] == 1 && encoder->encoder_gpio[1] == 0 && encoder->encoder_gpio[0] == 1)
    {
        encoder->encoder_flag = encoder_reversal;
    }

//    if(encoder->encoder_prev_flag == encoder_none && encoder->encoder_flag == encoder_foreward)
//    {
//        encoder->encoder_output = encoder_foreward;
//    }
//    else if(encoder->encoder_prev_flag == encoder_none && encoder->encoder_flag == encoder_reversal)
//    {
//        encoder->encoder_output = encoder_reversal;
//    }
//    else
//    {
//        encoder->encoder_output = encoder_none;
//    }

    encoder->encoder_output = encoder->encoder_flag;

    encoder->encoder_prev_flag = encoder->encoder_flag;
    encoder->encoder_prev_gpio[0] = encoder->encoder_gpio[0];//±£´æµ±Ç°×´Ì¬
    encoder->encoder_prev_gpio[1] = encoder->encoder_gpio[1];
}

int get_encoder_status(void)
{

    scan_encoder_status(&encoder);

    if(encoder.encoder_output == encoder_foreward)
    {
        return 5;
    }
    if(encoder.encoder_output == encoder_reversal)
    {
        return 6;
    }
    return 0;
}
