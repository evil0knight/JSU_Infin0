/*
 * circle.h
 *
 *  Created on: 2024年10月12日
 *      Author: A
 */

#ifndef USER_CAMERA_CIRCLE_H_
#define USER_CAMERA_CIRCLE_H_

enum circle_type_e {
  CIRCLE_NONE, // 非圆环模式
  CIRCLE_LEFT_BEGIN,
  CIRCLE_RIGHT_BEGIN, // 圆环开始，识别到单侧L角点另一侧长直道。
  CIRCLE_LEFT_IN,
  CIRCLE_RIGHT_IN, // 圆环进入，即走到一侧直道，一侧圆环的位置。
  CIRCLE_LEFT_RUNNING,
  CIRCLE_RIGHT_RUNNING, // 圆环内部。
  CIRCLE_LEFT_OUT,
  CIRCLE_RIGHT_OUT, // 准备出圆环，即识别到出环处的L角点。
  CIRCLE_LEFT_END,
  CIRCLE_RIGHT_END // 圆环结束，即再次走到单侧直道的位置。
};

extern enum circle_type_e circle_type;
extern int beel;

void CheckCircle(void);
void RunCircle(void);


#endif /* USER_CAMERA_CIRCLE_H_ */
