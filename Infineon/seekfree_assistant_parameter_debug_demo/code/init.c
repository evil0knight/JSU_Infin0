#include "zf_common_headfile.h"


void all_init(void)
{
    ips200_init(IPS200_TYPE_SPI);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    servo_init();
}




void servo_init(void)
{
    pwm_init(SERVO_1, SERVO_FREQ, SERVO1_MID);
    pwm_init(SERVO_2, SERVO_FREQ, SERVO2_MID);
    pwm_init(SERVO_3, SERVO_FREQ, SERVO3_MID);
    pwm_init(SERVO_4, SERVO_FREQ, SERVO4_MID);
}


