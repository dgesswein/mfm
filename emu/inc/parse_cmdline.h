/*
 * parse_cmdline.h
 *
 * 05/17/21 DJG Added option to initialize
 * 11/09/14 DJG Added new command line options
 * 10/24/14 DJG Changed needed to support mfm_emu write buffer

 *  Created on: Dec 21, 2013
 *      Author: djg
 */

#ifndef PARSE_CMDLINE_H_
#define PARSE_CMDLINE_H_

#ifndef DEF_DATA
#define DEF_EXTERN extern
#else
#define DEF_EXTERN
#endif

#define MAX_HEAD 16

#define MAX_DRIVES 2
typedef struct {
   char *name;
   int value;
} CONTROLLER;

#define CONTROLLER_DEFAULT 1
#define CONTROLLER_CROMEMCO 2
DEF_EXTERN CONTROLLER mfm_controller_info[]
#ifdef DEF_DATA
   = {
     {"Default", CONTROLLER_DEFAULT},
     {"Cromemco", CONTROLLER_CROMEMCO},
     {NULL, 0}
}
#endif
;

// This is the main structure defining the drive characteristics
typedef struct {
   // The number of cylinders and heads
   int num_cyl;
   int num_head;
   char *filename[MAX_DRIVES];
   int fd[MAX_DRIVES];
   int drive[MAX_DRIVES];  // Drive select number
   EMU_FILE_INFO emu_file_info[MAX_DRIVES];
   int num_drives;
   int initialize;
   int buffer_count;		// Number of track buffers
   float buffer_max_time;	// Max time to delay when last buffer used
   float buffer_time;		// time parameter for delay calculation
   char *cmdline;               // Decode parameters from command line
   char *options;               // Extra options specified for saving in file
   char *note;                  // File information string
   uint32_t sample_rate_hz;	// MFM clock and data bit rate
   uint32_t rpm;                // Drive RPM. 0 if not set.
   uint32_t start_time_ns;	// Time to shift start of reading from index
} DRIVE_PARAMS;
char *parse_print_cmdline(DRIVE_PARAMS *drive_params, int print);
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params);
#endif /* PARSE_CMDLINE_H_ */
