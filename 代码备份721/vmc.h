#include "zf_common_headfile.h"
#ifndef CODE_VMC_H_
#define CODE_VMC_H_


/******************************************************************************************************************************************/
#define L1  6.0f    //腿高
#define L2  9.0f    //腿高
#define L3  9.0f    //腿高
#define L4  6.0f    //腿高
#define L5  3.7f    //腿高
#define L6  12.6f   //车宽
/******************************************************************************************************************************************/

typedef enum {
    JUMP_IDLE = 0,
    JUMP_PREPARE,      // 
    JUMP_DOWN,         // 
    JUMP_UP,           // 
    JUMP_LANDING       // 
} jump_state_t;
extern jump_state_t jump_state;
extern int jump_timer;
extern int jump_active;

/******************************************************************************************************************************************/
void servo_control_table(float p, float angle, int16* pwm1, int16* pwm2);
void servo_control_pitch(float Phi);
void jump_act(void);
void jump_process(void);
/******************************************************************************************************************************************/


#endif /* CODE_VMC_H_ */
