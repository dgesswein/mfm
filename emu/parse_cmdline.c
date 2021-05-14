// Parse the command line.
//
// Call parse_cmdline to parse the command line
// Call parse_print_cmdline to print drive parameter information in command
//   line format
//
// 05/13/2021 DJG Add --fill to set value used to fill emulator data for 
//    --initialize
// 04/15/19 DJG Added RPM option
// 01/04/15 DJG Changed buffer to pool, added begin_time and rate options.
// 11/09/14 DJG Added command option to set number of buffers and max delay
//    added note and options command line options.
//
// Copyright 2021 David Gesswein.
// This file is part of MFM disk utilities.
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
#include <inttypes.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdio.h>


#include "msg.h"
#include "emu_tran_file.h"
#include "parse_cmdline.h"
#include "version.h"

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

static void parse_drive_list(char *arg, DRIVE_PARAMS *drive_params);
static void parse_buffer_list(char *arg, DRIVE_PARAMS *drive_params);
static void parse_filename_list(char *arg, DRIVE_PARAMS *drive_params);

// Main routine for parsing command lines
//
// argc, argv: Main argc, argv
// drive_parameters: Drive parameters where most of the parsed values are stored
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params)
{
#define MIN_OPTS (0x03)
#define MIN_INITIALIZE_OPTS (0x0f)
   // If you change this fix parse_print_cmdline
   struct option long_options[] = {
         {"file", 1, NULL, 'f'},
         {"drive", 1, NULL, 'd'},
         {"heads", 1, NULL, 'h'},
         {"cylinders", 1, NULL, 'c'},
         {"rate", 1, NULL, 'r'},
         {"begin_time", 1, NULL, 'b'},
         {"initialize", 0, NULL, 'i'},
         {"pool", 1, NULL, 'p'},
         {"quiet", 1, NULL, 'q'},
         {"version", 0, NULL, 'v'},
         {"note", 1, NULL, 'n'},
         {"options", 1, NULL, 'o'},
         {"rpm", 1, NULL, 'R'},
         {"fill", 1, NULL, 'F'},
         {NULL, 0, NULL, 0}
   };
   char short_options[] = "f:d:h:c:r:b:ip:q:vn:o:R:F:";
   int rc;
   // Loop counters
   int i;
   int options_index;
   // Bit vector of which options were specified
   uint32_t opt_mask = 0;

   // Enable all errors other than debug
   msg_set_err_mask(~0 ^ MSG_DEBUG);

   // Set defaults
   memset(drive_params, 0, sizeof(*drive_params));
   // Default number of buffers and maximum seek delay time
   drive_params->buffer_count = 75;
   drive_params->buffer_max_time = .6;
   drive_params->sample_rate_hz = 10000000;
   drive_params->fill = 0xaa;

   //drive_params->initialize and ->num_drives need to be zero

   // Handle the options. The long options are converted to the short
   // option name for the switch by getopt_long.
   options_index = -1;
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
            msg(MSG_FATAL,"Valid options:\n");
            for (i = 0; long_options[i].name != NULL; i++) {
               msg(MSG_FATAL, "%c %s\n", 
                  long_options[i].val, long_options[i].name);
            }
            exit(1);
         }
      }
      opt_mask |= 1 << options_index;
      switch(rc) {
      case 'h':
         drive_params->num_head = atoi(optarg);
         if (drive_params->num_head <= 0 || drive_params->num_head > MAX_HEAD) {
            msg(MSG_FATAL,"Number of heads must be 1 to %d\n", MAX_HEAD);
            exit(1);
         }
         break;
      case 'c':
         drive_params->num_cyl = atoi(optarg);
         if (drive_params->num_cyl <= 0) {
            msg(MSG_FATAL,"Number of cylinders must be greater than 0 %d\n");
            exit(1);
         }
         break;
      case 'f':
         parse_filename_list(optarg, drive_params);
         break;
      case 'd':
         parse_drive_list(optarg, drive_params);
         break;
      case 'i':
         drive_params->initialize = 1;
         break;
      case 'b':
         drive_params->start_time_ns = strtoul(optarg, NULL, 0);
         break;
      case 'p':
         parse_buffer_list(optarg, drive_params);
         break;
      case 'q':
         msg_set_err_mask(~strtol(optarg, NULL, 0));
         break;
      case 'v':
         msg(MSG_INFO_SUMMARY,"Version %s\n",VERSION);
         break;
      case 'n':
         drive_params->note = optarg;
         break;
      case 'o':
         drive_params->options = optarg;
         break;
      case 'r':
         drive_params->sample_rate_hz = strtoul(optarg, NULL, 0);
         break;
      case 'R':
         drive_params->rpm = strtoul(optarg, NULL, 0);
         break;
      case 'F':
         drive_params->fill = strtoul(optarg, NULL, 0);
         break;
      case '?':
         exit(1);
         break;
      default:
         msg(MSG_FATAL, "Didn't process argument %c\n", rc);
         exit(1);
      }
      options_index = -1;
   }
   if (optind < argc) {
      msg(MSG_FATAL, "Uknown option %s specified\n",argv[optind]);
      exit(1);
   }
   if ((opt_mask & MIN_OPTS) != MIN_OPTS) {
      msg(MSG_FATAL, "Program requires options:");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(opt_mask & bit) && (MIN_OPTS & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   }
   if (drive_params->initialize && (opt_mask & MIN_INITIALIZE_OPTS) != 
         MIN_INITIALIZE_OPTS) {
      msg(MSG_FATAL, "Initialize requires options:");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(opt_mask & bit) && (MIN_INITIALIZE_OPTS & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   }
   drive_params->buffer_time = drive_params->buffer_max_time /
      drive_params->buffer_count;
}

// Routine for parsing comma separated buffer information
//
// arg: Argument string
// drive_params: Drive parameters to store buffer information in
static void parse_buffer_list(char *arg, DRIVE_PARAMS *drive_params) {
   int i;
   char *str, *tok;

   str = arg;

   i = 0;
   while (1) {
      tok = strtok(str,",");
      if (tok == NULL) {
         break;
      }
      switch(i) {
         case 0:
            drive_params->buffer_count = atoi(tok);
         break;
         case 1:
            drive_params->buffer_max_time = atof(tok);
         break;
         default:
            msg(MSG_FATAL, "Maximum of %d buffer parameters may be specified\n",
               i);
            exit(1);
      }
      str = NULL;  // For next strtok call
      i++;
   }
}

// Routine for parsing comma separated drive number list
//
// arg: Argument string
// drive_params: Drive parameters to store drive numbers in
static void parse_drive_list(char *arg, DRIVE_PARAMS *drive_params) {
   int i;
   char *str, *tok;

   str = arg;

   i = 0;
   while (1) {
      tok = strtok(str,",");
      if (tok == NULL) {
         break;
      }
      if (i >= ARRAYSIZE(drive_params->drive)) {
         msg(MSG_FATAL, "Maximum of %d drives may be specified\n",
               ARRAYSIZE(drive_params->drive));
         exit(1);
      }
      str = NULL;  // For next strtok call
      drive_params->drive[i] = strtoull(tok, NULL, 0);
      i++;
   }
   if (drive_params->num_drives == 0) {
      drive_params->num_drives = i;
   } else {
      if (drive_params->num_drives != i) {
         msg(MSG_FATAL,"Number of drive selects specified must match number of filenames\n");
         exit(1);
      }
   }
}

// Routine for parsing comma separated file name list
//
// arg: Argument string
// drive_params: Drive parameters to store file names to
static void parse_filename_list(char *arg, DRIVE_PARAMS *drive_params) {
   int i;
   char *str, *tok;

   str = arg;

   i = 0;
   while (1) {
      tok = strtok(str,",");
      if (tok == NULL) {
         break;
      }
      if (i >= ARRAYSIZE(drive_params->drive)) {
         msg(MSG_FATAL, "Maximum of %d drives may be specified\n",
               ARRAYSIZE(drive_params->drive));
         exit(1);
      }
      str = NULL;  // For next strtok call
      drive_params->filename[i] = tok;
      i++;
   }
   if (drive_params->num_drives == 0) {
      drive_params->num_drives = i;
   } else {
      if (drive_params->num_drives != i) {
         msg(MSG_FATAL,"Number of drive selects specified must match number of filenames\n");
         exit(1);
      }
   }
}

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

// Print command line to give current drive parameter settings
//
// drive_params: Drive parameters
char *parse_print_cmdline(DRIVE_PARAMS *drive_params, int print) {
   static char cmdline[2048];
   int cmdleft = sizeof(cmdline)-1;
   char *cmdptr = cmdline;

   safe_print(&cmdptr, &cmdleft, "--heads %d --cylinders %d ",
         drive_params->num_head, drive_params->num_cyl);
   if (drive_params->start_time_ns) {
      safe_print(&cmdptr, &cmdleft, " --begin_time %u",
         drive_params->start_time_ns);
   }
   safe_print(&cmdptr, &cmdleft, " --rate %u", drive_params->sample_rate_hz);

   if (drive_params->options != NULL) {
      safe_print(&cmdptr, &cmdleft, "%s ",drive_params->options);
   }
#if 0
   if (drive_params->note != NULL) {
      char *p;
      safe_print(&cmdptr, &cmdleft, " --note \"");
      for (p = drive_params->note; *p != 0; p++) {
         if (*p == '"') {
            safe_print(&cmdptr, &cmdleft, "\\%c", *p);
         } else {
            safe_print(&cmdptr, &cmdleft, "%c", *p);
         }
      }
      safe_print(&cmdptr, &cmdleft, "\"");
   }
   msg(MSG_INFO_SUMMARY," --transitions_file %s --extracted_data_file %s",
         drive_params->transitions_filename, drive_params->extract_filename);
   msg(MSG_INFO_SUMMARY," --emulation_file %s",
         drive_params->emulation_file);
#endif
   if (print) {
      msg(MSG_INFO_SUMMARY, "Command line to generate file:\n%s\n", cmdline);
   }
   return cmdline;
}
