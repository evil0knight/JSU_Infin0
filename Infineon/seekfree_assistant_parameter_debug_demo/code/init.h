#include "zf_common_headfile.h"
#ifndef CODE_INIT_H_
#define CODE_INIT_H_

#define SERVO_1                 (ATOM0_CH0_P21_2)       //瀹氫箟涓绘澘涓婅埖鏈�1瀵瑰簲寮曡剼
#define SERVO_2                 (ATOM0_CH1_P21_3)       //瀹氫箟涓绘澘涓婅埖鏈�2瀵瑰簲寮曡剼
#define SERVO_3                 (ATOM0_CH2_P21_4)       //瀹氫箟涓绘澘涓婅埖鏈�3瀵瑰簲寮曡剼
#define SERVO_4                 (ATOM0_CH3_P21_5)       //瀹氫箟涓绘澘涓婅埖鏈�4瀵瑰簲寮曡剼
#define SERVO_FREQ              (300)                   //瀹氫箟涓绘澘涓婅埖鏈洪鐜�
#define SERVO1_MID              (4523)                    //鑸垫満1涓��     宸︿笅
#define SERVO2_MID              (4563)                    //鑸垫満2涓��     宸︿笂
#define SERVO3_MID              (4350)                    //4367
#define SERVO4_MID              (4000)                    //4533



void all_init(void);
void servo_init(void);


#endif /* CODE_INIT_H_ */
