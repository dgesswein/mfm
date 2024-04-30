// Parse the command line.
//
// Call parse_cmdline to parse the command line
// Call parse_print_cmdline to print drive parameter information in command
//   line format
// Call parse_validate_options to perform some validation on mfm_write options
//
// Copyright 2024 David Gesswein.
// This file is part of MFM disk utilities.
//
// 04/29/24 DJG Made separate file for mfm_write since options different than
//    mfm_util/mfm_read
//
// MFM disk utilities is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MFM disk utilities is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MFM disk utilities.  If not, see <http://www.gnu.org/licenses/>.
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>


#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "drive.h"

#include "parse_cmdline.h"
#include "version.h"

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// Print to buffer and exit if no space left.
// ptr: buffer to write to
// left: how many characters left
// format: format string
// ...: arguments to print
void safe_print(char **ptr, int *left, char *format, ...) {
   va_list va;
   int rc;

   va_start(va, format);
   rc = vsnprintf(*ptr, *left, format, va);
   *left -= rc;
   if (left <= 0) {
      msg(MSG_FATAL, "Command line exceeded buffer\n");
      exit(1);
   }
   *ptr += rc;
   va_end(va);
}

// Delete bit n from v shifting higher bits down
#define DELETE_BIT(v, n) (v & ((1 << n)-1)) | (((v & ~((1 << (n+1))-1)) >> 1))

// If you change this fix parse_print_cmdline and check if ext2emu should
// delete new option
static struct option long_options[] = {
         {"drive", 1, NULL, 'd'},
         {"unbuffered_seek", 0, NULL, 'u'},
         {"quiet", 1, NULL, 'q'},
         {"emulation_file", 1, NULL, 'm'},
         {"precomp_ns", 1, NULL, 'p'},
         {"precomp_cyl", 1, NULL, 'c'},
         {"version", 0, NULL, 'v'},
         {NULL, 0, NULL, 0}
};
static char short_options[] = "d:uq:m:p:c:v";


// Main routine for parsing command lines
//
// argc, argv: Main argc, argv
// drive_parameters: Drive parameters where most of the parsed values are stored
// delete_options: Options to delete from list of valid options (short option)
// initialize: 1 if drive_params should be initialized with defaults
// only_deleted: 1 if we only want to process options specified in 
//    delete_options. Other options are ignored, not error
// ignore_invalid_options: Don't exit if option is not known
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params, 
    char *delete_options, int initialize, int only_deleted,
    int ignore_invalid_options, int track_layout_format_only)
{
   int rc;
   // Loop counters
   int i,j;
   int options_index;
   // Bit vector of which options were specified
   char *tok;
   char delete_list[sizeof(short_options)];

   // If only deleted then copy all options to delete list that aren't
   // in delete_options
   if (only_deleted) {
      j = 0;
      for (i = 0; i < sizeof(short_options); i++) {
         if (short_options[i] != ':' && 
               strchr(delete_options, short_options[i]) == 0) {
            delete_list[j++] = short_options[i];
         }
      }
      delete_list[j] = 0;
      delete_options = delete_list;
   }
   // Options above are superset for all the programs to ensure options stay consistent
   if (initialize) {
      // Enable all errors other than debug
      msg_set_err_mask(~0 ^ (MSG_DEBUG | MSG_DEBUG_DATA));
      memset(drive_params, 0, sizeof(*drive_params));
      // Set defaults. A lot of this isn't used by mfm_write
      drive_params->emu_fd = -1;
      drive_params->tran_fd = -1;
      drive_params->ext_fd = -1;
      drive_params->step_speed = DRIVE_STEP_FAST;
      drive_params->retries = 50;
      drive_params->no_seek_retries = 4;
      drive_params->sector_size = 512;
      drive_params->emulation_output = 0;
      drive_params->analyze = 0;
      drive_params->start_time_ns = 0;
      drive_params->header_crc.length = -1; // 0 is valid
      drive_params->first_logical_sector = -1;
      // Default no precompensation
      drive_params->early_precomp_ns = 0;
      drive_params->late_precomp_ns = 0;
      drive_params->write_precomp_cyl = 9999;
   }
   // Handle the options. The long options are converted to the short
   // option name for the switch by getopt_long.
   options_index = -1;
   optind = 1; // Start with first element. We may call again with same argv
   while ((rc = getopt_long(argc, argv, short_options, long_options,
         &options_index)) != -1) {
      // Short options don't set options_index so look it up
      if (options_index == -1) {
         for (i = 0; i < ARRAYSIZE(long_options); i++) {
            if (rc == long_options[i].val) {
               options_index = i;
               break;
            }
         }
         if (options_index == -1) {
            // If only deleted specified don't print error. Option will
            // be ignored below. The valid options would be printed with
            // only the few options selected which would confuse the user
            if (!ignore_invalid_options) {
               //msg(MSG_FATAL, "Error parsing option %c\n",rc);
               msg(MSG_FATAL,"Valid options:\n");
               for (i = 0; long_options[i].name != NULL; i++) {
                 if (strchr(delete_options, long_options[i].val) == 0) {
                    msg(MSG_FATAL, "%c %s\n", long_options[i].val, 
                       long_options[i].name);
                 }
               }
               exit(1);
            }
         }
      }
      // If option is deleted or not found either error or ignore
      if (strchr(delete_options, rc) != 0) {
         if (!ignore_invalid_options) {
            msg(MSG_FATAL,"Option '%c' %s not valid for this program\n",
               rc, long_options[options_index].name);
            exit(1);
         }
      } else {
         drive_params->opt_mask |= 1 << options_index;
         switch(rc) {
         case 'p':
            tok = strtok(optarg,",");
            drive_params->early_precomp_ns = atoi(tok);
            tok = strtok(NULL,",");
            if (tok != NULL) {
               drive_params->late_precomp_ns = atoi(tok);
            } else {
               drive_params->late_precomp_ns = drive_params->early_precomp_ns;
            }
            if (drive_params->late_precomp_ns < 0 || 
                  drive_params->late_precomp_ns > 30 ||
                  drive_params->early_precomp_ns < 0 || 
                  drive_params->early_precomp_ns > 30) {
               msg(MSG_FATAL,"Precompensation ns must be 0 to 30\n");
               if (!ignore_invalid_options) {
                 exit(1);
               }
            }
            drive_params->early_precomp_ns = round(drive_params->early_precomp_ns / 5.0) * 5;
            drive_params->late_precomp_ns = round(drive_params->late_precomp_ns / 5.0) * 5;
            msg(MSG_INFO,"Using early ns %d, late ns %d for precompensation\n",
               drive_params->early_precomp_ns, drive_params->late_precomp_ns);
            break;
         case 'c':
            drive_params->write_precomp_cyl = atoi(optarg);
            break;
         case 'u':
            drive_params->step_speed = DRIVE_STEP_SLOW;
            break;
         case 'm':
            drive_params->emulation_filename = optarg;
            // Caller will correct if file is actually input.
            drive_params->emulation_output = 1; 
            break;
         case 'd':
            drive_params->drive = atoi(optarg);
            break;
         case 'q':
            msg_set_err_mask(~strtoul(optarg, NULL, 0));
            break;
         case 'v':
            msg(MSG_INFO_SUMMARY,"Version %s\n",VERSION);
            break;
         case '?':
            if (!ignore_invalid_options) {
               exit(1);
            }
            break;
         default:
            msg(MSG_FATAL, "Didn't process argument %c\n", rc);
            if (!ignore_invalid_options) {
               exit(1);
            }
         }
      }
      options_index = -1;
   }
   if (optind < argc && !ignore_invalid_options) {
      msg(MSG_FATAL, "Uknown option %s specified\n",argv[optind]);
      exit(1);
   }
}

// This validates options where we need the options list for messages
//
// drive_params: Drive parameters
// mfm_read: Not used.

void parse_validate_options(DRIVE_PARAMS *drive_params, int mfm_read) {
   // For mfm_util drive doesn't need to be specified. This
   // option error handling is getting messy.
   // Drive 1-4 valid if specified. If analyze specified drive will be 0
   if (drive_params->drive < 1 || drive_params->drive > 4)  {
         msg(MSG_FATAL, "Drive must be between 1 and 4\n");
         exit(1);
   }
}
