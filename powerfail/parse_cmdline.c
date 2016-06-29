#define VERSION "0.3"
// Parse the command line.
//
// Call parse_cmdline to parse the command line
// Call parse_print_cmdline to print drive parameter information in command
//   line format
//
// Copyright 2014 David Gesswein.
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

#include "parse_cmdline.h"

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// argc, argv: Main argc, argv
// drive_params: Drive parameters where most of the parsed values are stored
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params)
{
#define MIN_OPTS (0x01)
   // If you change this fix parse_print_cmdline
   struct option long_options[] = {
         {"powercmd", 1, NULL, 'p'},
         {"command", 1, NULL, 'c'},
         {"debug", 0, NULL, 'd'},
         {"scale", 1, NULL, 's'},
         {"threshold", 1, NULL, 't'},
         {"wait", 1, NULL, 'w'},
         {"version", 0, NULL, 'v'},
         {NULL, 0, NULL, 0}
   };
   char short_options[] = "p:c:ds:t:w:v";
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
   drive_params->scale = .125;
   drive_params->threshold = 11.5;
   drive_params->wait = .1;

   // Handle the options. The long options are converted to the short
   // option name for the switch by getopt_long.
   options_index = -1;
   while ((rc = getopt_long(argc, argv, short_options, long_options,
         &options_index)) != -1 ) {
      // Short options don't set options_index so look it up
      if (options_index == -1) {
         for (i = 0; i < ARRAYSIZE(long_options); i++) {
            if (rc == long_options[i].val) {
               options_index = i;
               break;
            }
         }
         if (options_index == -1) {
            //msg(MSG_FATAL, "Error parsing option %c\n",rc);
            msg(MSG_FATAL,"Valid options:\n");
            for (i = 0; long_options[i].name != NULL; i++) {
               msg(MSG_FATAL, "%c %s\n", long_options[i].val, long_options[i].name);
            }
            exit(1);
         }
      }
      opt_mask |= 1 << options_index;
      switch(rc) {
      case 's':
         drive_params->scale = atof(optarg);
         break;
      case 't':
         drive_params->threshold = atof(optarg);
         break;
      case 'w':
         drive_params->wait = atof(optarg);
         break;
      case 'p':
         drive_params->powercmd = optarg;
         break;
      case 'd':
         drive_params->debug = 1;
         break;
      case 'c':
         drive_params->command = optarg;
         break;
      case 'v':
         msg(MSG_INFO_SUMMARY,"Version %s\n",VERSION);
         break;
      case '?':
         msg(MSG_FATAL, "Error parsing option\n");
         exit(1);
         break;
      default:
         msg(MSG_FATAL, "Didn't process argument %c\n", rc);
         exit(1);
      }
      options_index = -1;
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
}
