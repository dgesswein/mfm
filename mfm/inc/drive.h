/*
 * drive.h
 *
 *  Created on: Dec 23, 2013
 *      Author: djg
 */

#ifndef DRIVE_H_
#define DRIVE_H_

void drive_select(int drive);
void drive_set_head(int head);
void drive_seek_track0(void);
void drive_setup(DRIVE_PARAMS *drive_params);
void drive_read_disk(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas);
int drive_at_track0(void);
uint32_t drive_get_drive_status(void);
void drive_print_drive_status(int level, uint32_t status);
double drive_rpm(void);
#define DRIVE_STEP_SLOW 0
#define DRIVE_STEP_FAST 1
#define DRIVE_STEP_FATAL_ERR 1
#define DRIVE_STEP_RET_ERR 0
#define DRIVE_STEP_NO_UPDATE_CYL 0
#define DRIVE_STEP_UPDATE_CYL 1
int drive_step(int seek_speed, int steps, int update_cyl, int err_fatal);
void drive_read_track(DRIVE_PARAMS *drive_params, int cyl, int head,
      void *deltas, int max_deltas);
void drive_initialize(void);
int drive_get_board_revision(void);
#endif /* DRIVE_H_ */
