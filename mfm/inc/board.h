/*
 * board.h
 *
 *  Created on: August 1, 2015
 *      Author: djg
 */

#ifndef BOARD_H_
#define BOARD_H_

void board_initialize(void);
int board_get_revision(void);
int board_set_restore_max_cpu_speed(int restore);
#endif /* BOARD_H_ */
