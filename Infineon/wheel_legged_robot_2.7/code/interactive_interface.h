/*
 * interactive_interface.h
 *
 *  Created on: 2024Äê11ÔÂ24ÈÕ
 *      Author: 17104
 */

#ifndef CODE_INTERACTIVE_INTERFACE_H_
#define CODE_INTERACTIVE_INTERFACE_H_

#include <universal_filter.h>
#include "key_status.h"
#include "menu.h"
#include <Base.h>
#include "enconder.h"

extern encoder_struct encoder;

extern key_struct key_up;
extern key_struct key_down;
extern key_struct key_next;
extern key_struct key_back;

extern directory all_dir[dir_num];
extern int mouse;
extern int have_chosen;
extern int image;

//extern Matrix LS_STRAIGHT_Q;
//extern Matrix LS_STRAIGHT_R;
//extern Matrix LS_STRAIGHT_K;

void interactive_interface_init(void);
void interactive_interface_run(void);

#endif /* CODE_INTERACTIVE_INTERFACE_H_ */
