/*
 * time.h
 *
 *  Created on: 2024Äê11ÔÂ21ÈÕ
 *      Author: A
 */

#ifndef USER_CAMERA_ATIME_H_
#define USER_CAMERA_ATIME_H_

#include "zf_common_headfile.h"

void Atimer_start(uint8 timer_id);
void Atimer_stop(uint8 timer_id);
void Atimer_clear(uint8 timer_id);
uint16 Atimer_get(uint8 timer_id);


#endif /* USER_CAMERA_ATIME_H_ */
