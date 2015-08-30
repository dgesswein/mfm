/*
 * parse_cmdline.h
 *
 *  Created on: Dec 21, 2013
 *      Author: djg
 */

#ifndef PARSE_CMDLINE_H_
#define PARSE_CMDLINE_H_

#define MAX_HEAD 16

#define MAX_DRIVES 2
// This is the main structure defining the drive characteristics
typedef struct {
   char *command;
   double scale;
   double threshold;
   double wait;
   char *powercmd;
   int debug;
} DRIVE_PARAMS;
void parse_print_cmdline(DRIVE_PARAMS *drive_params);
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params);
#endif /* PARSE_CMDLINE_H_ */
