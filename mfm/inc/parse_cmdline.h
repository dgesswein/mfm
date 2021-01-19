/*
 * parse_cmdline.h
 *
 *  11/09/14 DJG Changes for new command line options
 *  Created on: Dec 21, 2013
 *      Author: djg
 */

#ifndef PARSE_CMDLINE_H_
#define PARSE_CMDLINE_H_

char *parse_print_cmdline(DRIVE_PARAMS *drive_params, int print, 
   int no_retries_drive_interleave);
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params,
     char *delete_options, int initialize, int only_deleted,
     int allow_invalid_options, int track_layout_format_only);
void parse_validate_options(DRIVE_PARAMS *drive_params, int mfm_read);
void parse_validate_options_listed(DRIVE_PARAMS *drive_params, char *opt);
void parse_set_drive_params_from_controller(DRIVE_PARAMS *drive_params,
   int controller);
#endif /* PARSE_CMDLINE_H_ */
