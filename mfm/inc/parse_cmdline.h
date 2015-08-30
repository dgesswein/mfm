/*
 * parse_cmdline.h
 *
 *  11/09/14 DJG Changes for new command line options
 *  Created on: Dec 21, 2013
 *      Author: djg
 */

#ifndef PARSE_CMDLINE_H_
#define PARSE_CMDLINE_H_

char *parse_print_cmdline(DRIVE_PARAMS *drive_params, int print);
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params,
     char *delete_options, int initialize, int only_deleted,
     int allow_invalid_options);
void parse_validate_options(DRIVE_PARAMS *drive_params, int mfm_read);
#endif /* PARSE_CMDLINE_H_ */
