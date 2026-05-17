/*
 * rampway.h
 *
 *  Created on: 2023Äê6ÔÂ18ÈÕ
 *      Author: 11150
 */

#ifndef CODE_RAMPWAY_H_
#define CODE_RAMPWAY_H_

#include "zf_common_typedef.h"

void rampway(void);

extern uint8 rampway_state;



typedef struct
{
      int16 row;
    int16 column;
    int16 flag;
}
inflection_crossroads_point;



void find_point_crossroads(uint8 mode, uint8 start_row, uint8 end_row);
void crossroads(void);
void crossroads_new(int biaozhi);
void crossroads_electricity(void);

extern uint8 crossroads_flag;
extern uint8 crossroads_state;


extern inflection_crossroads_point left_guai_crossroads[2],right_guai_crossroads[2];




#endif /* CODE_RAMPWAY_H_ */
