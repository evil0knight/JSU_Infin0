/*
 * wifi_lineless.h
 *
 *  Created on: 2025年4月22日
 *      Author: lenovo
 */

#ifndef CODE_WIFI_LINELESS_H_
#define CODE_WIFI_LINELESS_H_


#include "zf_common_headfile.h"

#define WIFI_SSID_TEST          "12345678"
#define WIFI_PASSWORD_TEST      "12345678"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.137.1"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8080"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "8889"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

#define BOUNDARY_NUM            (MT9V03X_H * 2)

extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
// 只有X边界
extern uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
// 只有Y边界
extern uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];



void wifi_disp(void);
void wifi_init(void);

#endif /* CODE_WIFI_LINELESS_H_ */
