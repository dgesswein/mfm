/*
 * read_deltas.h
 *
 *  Created on: Dec 23, 2013
 *      Author: djg
 */

#ifndef READ_DELTAS_H_
#define READ_DELTAS_H_

void deltas_start_read(int cyl, int head);
void deltas_start_thread(DRIVE_PARAMS *drive_params);
void deltas_stop_thread();
void *deltas_setup(int ddr_mem_size);
int deltas_wait_read_finished();
int deltas_get_count(int cur_delta);
void deltas_update_count(int num_deltas_in, int streaming_in);

#endif /* READ_DELTAS_H_ */
