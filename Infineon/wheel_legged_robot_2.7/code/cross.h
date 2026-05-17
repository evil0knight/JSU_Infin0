/*
 * cross.h
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */

#ifndef USER_CAMERA_CROSS_H_
#define USER_CAMERA_CROSS_H_

enum cross_type_e {
  CROSS_NONE,  // 非十字模式
  CROSS_BEGIN, // 找到左右两个L角点
  CROSS_IN, // 两个L角点很近，即进入十字内部(此时切换远线控制)
};

extern enum cross_type_e cross_type;
extern int32_t Both_Boder_None_Cross;
extern int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
extern float cross_open;

void CheckCross(void);
void RunCross(void);
void cross_farline(void);

#endif /* USER_CAMERA_CROSS_H_ */
