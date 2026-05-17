/*
 * flash.h
 *
 *  Created on: 2024Äê11ÔÂ25ÈÕ
 *      Author: 17104
 */

#ifndef CODE_DATABASE_H_
#define CODE_DATABASE_H_

#include "zf_common_headfile.h"

#define data_num 10

extern flash_data_union database_buffer[12][data_num * 2];

void clear_all_data(void);
uint8 write_database(uint32 page_num,int order_num,void * data,int is_float);
float read_database_float(uint32 page_num,int order_num);
int read_database_int(uint32 page_num,int order_num);
#endif /* CODE_DATABASE_H_ */
