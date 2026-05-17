/*
 * roadblock.h
 *
 *  Created on: 2023Äê6ÔÂ18ÈÕ
 *      Author: 11150
 */

#ifndef CODE_ROADBLOCK_H_
#define CODE_ROADBLOCK_H_
#include "zf_common_typedef.h"


/*void roadblock_xiaowan(void);
void roadblock_xiaowan_two(void) ;
void roadblock_dawan(void);
void roadblock_zhidao_zhengpao(void);
void roadblock_zhidao_fanpao(void);*/

void roadblock_judge(void);
void roadblock_control(int biaozhi);

void white_juedge(void);
void broken_road_judge(void);
void broken_road_control(void);




extern uint8 roadblock_state;
extern uint8 broken_road_state;

void broken_road_new(int biaozhi);
float rampway_turn(float true_jiaodu,float jiaodu);
#endif /* CODE_ROADBLOCK_H_ */
