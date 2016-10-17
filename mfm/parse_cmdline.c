// Parse the command line.
//
// Call parse_cmdline to parse the command line
// Call parse_print_cmdline to print drive parameter information in command
//   line format
// Call parse_validate_options to perform some validation on options that
//   both mfm_util and mfm_read need
//
// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
//
// 10/16/16 DJG Added parameter to control seeks to --retry
// 01/13/16 DJG Changes for ext2emu related changes on how drive formats will
//     be handled. If controller defines other parameters such as polynomial
//     set them
// 01/06/16 DJG Rename structure
// 01/02/16 DJG Add --mark_bad support
// 12/31/15 DJG Changes for ext2emu
// 11/01/15 DJG Validate options required when format is specified.
// 05/17/15 DJG Made drive -d and data_crc -j so -d would be drive in all
//    of the MFM programs.
// 01/04/15 DJG Suppressed printing command line options that weren't set
//    Added begin_time. Made failure to decode options non fatal when
//    reading options stored in file headers.
// 11/09/14 DJG Modified option parsing to allow mfm_util to reparse 
//    options stored in transitions file
// 10/01/14 DJG Incremented version number
// 09/06/14 DJG Made new class of info errors not print by default.
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
// Print command line to give current drive parameter settings
//
// drive_params: Drive parameters
char *parse_print_cmdline(DRIVE_PARAMS *drive_params, int print,
      int no_retries_drive_interleave) {
   static char cmdline[2048];
   int cmdleft = sizeof(cmdline)-1;
   char *cmdptr = cmdline;

   if (drive_params->num_sectors != 0) {
      safe_print(&cmdptr, &cmdleft, "--sectors %d,%d ",
         drive_params->num_sectors, drive_params->first_sector_number);
   }
   safe_print(&cmdptr, &cmdleft, "--heads %d --cylinders %d ",
         drive_params->num_head, drive_params->num_cyl);
   if (drive_params->header_crc.length != 0) {
      safe_print(&cmdptr, &cmdleft,
         "--header_crc 0x%llx,0x%llx,%d,%d ",
         drive_params->header_crc.init_value, drive_params->header_crc.poly,
         drive_params->header_crc.length, drive_params->header_crc.ecc_max_span);
   }
   if (drive_params->data_crc.length != 0) {
      safe_print(&cmdptr, &cmdleft,
         "--data_crc  0x%llx,0x%llx,%d,%d ",
         drive_params->data_crc.init_value, drive_params->data_crc.poly,
         drive_params->data_crc.length, drive_params->data_crc.ecc_max_span);
   }
   if (drive_params->controller != CONTROLLER_NONE) {
      safe_print(&cmdptr, &cmdleft, "--format %s ",
         mfm_controller_info[drive_params->controller].name);
   }
   safe_print(&cmdptr, &cmdleft,
         "--sector_length %d ", drive_params->sector_size);
   if (!no_retries_drive_interleave) {
      safe_print(&cmdptr, &cmdleft, "--retries %d,%d --drive %d ",
         drive_params->retries, drive_params->no_seek_retries,
         drive_params->drive);
   }
   if (drive_params->step_speed == DRIVE_STEP_SLOW) {
      safe_print(&cmdptr, &cmdleft, "--unbuffered_seek ");
   }
   if (drive_params->head_3bit) {
      safe_print(&cmdptr, &cmdleft, "--head_3bit ");
   }
   if (!no_retries_drive_interleave && drive_params->sector_numbers != NULL) {
      int i;
      safe_print(&cmdptr, &cmdleft, " --interleave ");
      for (i = 0; i < drive_params->num_sectors; i++) {
         if (i == drive_params->num_sectors - 1) {
            safe_print(&cmdptr, &cmdleft, "%d", drive_params->sector_numbers[i]);
         } else {
            safe_print(&cmdptr, &cmdleft, "%d,", drive_params->sector_numbers[i]);
         }
      }
   }
   if (drive_params->start_time_ns) {
      safe_print(&cmdptr, &cmdleft, " --begin_time %u", 
         drive_params->start_time_ns);
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
      msg(MSG_INFO_SUMMARY, "Command line to read disk:\n%s\n", cmdline);
   }
   return cmdline;
}

// Parse CRC values
//
// arg: CRC value string
// return: CRC information
static CRC_INFO parse_crc(char *arg) {
   CRC_INFO info;
   int i;
   char *str, *tok;
   uint64_t val;

   str = arg;
   memset(&info, 0, sizeof(info));

   for (i = 0; i < 4; i++) {
      tok = strtok(str,",");
      if (tok == NULL) {
         break;
      }
      str = NULL;
      val = strtoull(tok, NULL, 0);
      switch(i) {
      case 0:
         info.init_value = val;
         break;
      case 1:
         info.poly = val;
         break;
      case 2:
         info.length = val;
         break;
      case 3:
         info.ecc_max_span = val;
         break;
      }
   }
   if (i < 3) {
      msg(MSG_FATAL,"Minimum for CRC is initial value, polynomial, and polynomial size\n");
      exit(1);
   }
   return info;
}

// Parse controller value (track/sector header format). The formats are named
// after the controller that wrote the format. Multiple controllers may use
// the same format.
//
// arg: Controller string
// ignore_invalid_option: Don't print if controller is invalid
// return: Controller number
static int parse_controller(char *arg, int ignore_invalid_options,
      DRIVE_PARAMS *drive_params, int *params_set) {
   int i;
   int controller = -1;

   *params_set = 0;
   for (i = 0; mfm_controller_info[i].name != NULL; i++) {
      if (strcasecmp(mfm_controller_info[i].name, arg) == 0) {
         controller = i;
      }
   }
   if (controller == -1) {
      if (!ignore_invalid_options) {
         msg(MSG_FATAL, "Unknown controller %s. Choices are\n",arg);
         for (i = 0; mfm_controller_info[i].name != NULL; i++) {
            msg(MSG_FATAL,"%s\n",mfm_controller_info[i].name);
         }
         exit(1);
      }
   } else {
      CONTROLLER *contp = &mfm_controller_info[controller];
      if (contp->analyze_search == CONT_MODEL) {
         drive_params->header_crc = contp->write_header_crc;
         drive_params->data_crc = contp->write_data_crc;
         drive_params->num_sectors = contp->write_num_sectors;
         drive_params->first_sector_number = contp->write_first_sector_number;
         drive_params->sector_size = contp->write_sector_size;
         *params_set = 1;
      }
   }
   return controller;
}

// Parse the interleave information. It may either be an interleave number
// or a comma separated list of sector number.
//
// arg: Interleave information string
// drive_params: Drive parameters
// return: Pointer to list of sector number
static uint8_t *parse_interleave(char *arg, DRIVE_PARAMS *drive_params) {
   // Must be static since we return pointer. Sector numbers for interleave
   static uint8_t sectors[MAX_SECTORS];
   // Sector numbers already used.
   uint8_t used_sectors[MAX_SECTORS];
   int i;
   char *str, *tok;
   int interleave, sectnum;

   str = arg;
   for (i = 0; i < MAX_SECTORS; i++) {
      tok = strtok(str,",");
      if (tok == NULL) {
         break;
      }
      str = NULL;
      sectors[i] = atoi(tok);
   }
   // If one parameter it is interleave value so we calculate the sector
   // numbers for each sector
   if (i == 1) {
      if (drive_params->num_sectors == 0) {
         msg(MSG_FATAL, "Number of sectors must be set before interleave is specified\n");
         exit(1);
      }
      interleave = sectors[0];
      sectnum = 0;
      memset(used_sectors, 0, sizeof(used_sectors));
      for (i = 0; i < drive_params->num_sectors; i++) {
         // Find next unused sector number if already used.
         while (used_sectors[sectnum]) {
            sectnum = (sectnum + 1) % drive_params->num_sectors +
                  drive_params->first_sector_number;
         }
         sectors[i] = sectnum;
         used_sectors[sectnum] = 1;
         sectnum = (sectnum + interleave) % drive_params->num_sectors +
               drive_params->first_sector_number;
      }
   } else {
      // allow 2 values for ext2emu
      if (drive_params->num_sectors != 0 && i != 0 && i != 2 &&
            drive_params->num_sectors != i) {
         msg(MSG_FATAL, "Number of sectors in interleave list doesn't match number of sectors\n");
         exit(1);
      }
   }
   // If "" specified then return null for no sector list
   if (i == 0) 
      return NULL;
   else 
      return sectors;
}

// Parse analyze optional arguments
//
// arg: Interleave information string
// drive_params: Drive parameters
static void parse_analyze(char *arg, DRIVE_PARAMS *drive_params) {
   char *tok;

   drive_params->analyze_cyl = 0;
   drive_params->analyze_head = 0;
   if (arg == NULL) {
      return;
   }

   tok = strtok(arg,",");
   if (tok == NULL) {
      return;
   }
   drive_params->analyze_cyl = atoi(tok);
   tok = strtok(NULL,",");
   if (tok == NULL) {
      return;
   }
   drive_params->analyze_head = atoi(tok);
}

static int mark_bad_compare(const void *a, const void *b) {
   const MARK_BAD_INFO *mba, *mbb;
   mba = a;
   mbb = b;
   if (mba->cyl > mbb->cyl || (mba->cyl == mbb->cyl && 
         ((mba->head > mbb->head) || (mba->head == mbb->head && 
           mba->sector > mbb->sector)))) {
      return 1;
   }  else if (mba->cyl == mbb->cyl && mba->head == mbb->head && 
          mba->sector == mbb->sector) {
      return 0;
   } else {
      return -1;
   }
}
// Parse the mark bad sector information. Format is cyl,head,sect:cyl,head,sect
//
// arg: Bad sector information string
// drive_params: Drive parameters
// return: Pointer to list of bad sector data sorted ascending
static MARK_BAD_INFO *parse_mark_bad(char *arg, DRIVE_PARAMS *drive_params) {
   int i;
   char *str, *tok;
   int num_bad;
   MARK_BAD_INFO *mark_bad_list;

   str = arg;
   num_bad = 1;
   while (*str != 0) {
      if (*str++ == ':') { 
         num_bad++;
      }
   }
   mark_bad_list = msg_malloc(num_bad * sizeof(MARK_BAD_INFO),
      "Mark bad list");

   str = arg;
   for (i = 0; i < num_bad; i++) {
      tok = strtok(str,":");
      if (sscanf(tok, "%d,%d,%d", &mark_bad_list[i].cyl, &mark_bad_list[i].head,
           &mark_bad_list[i].sector) != 3) {
         msg(MSG_FATAL,"Error parsing mark bad list %s\n",tok); 
         exit(1);
      }
      mark_bad_list[i].last = 0;
      str = NULL;
   }
   qsort(mark_bad_list, num_bad, sizeof(MARK_BAD_INFO), mark_bad_compare);
   mark_bad_list[num_bad-1].last = 1;

   return mark_bad_list;
}

// Delete bit n from v shifting higher bits down
#define DELETE_BIT(v, n) (v & ((1 << n)-1)) | (((v & ~((1 << (n+1))-1)) >> 1))

// Minimum to generate extract file (sector data)
static int min_read_opts = 0x7f;
// Minimum to generation emulation file (MFM clock & data)
static int min_read_transitions_opts = 0x46;
// The options set when a CONT_MODEL controller used
static int controller_model_params = 0x99;
// Drive option bitmask
static int drive_opt = 0x40;
// data_crc option bitmask
static int data_crc_opt = 0x10;
// If you change this fix parse_print_cmdline and check if ext2emu should
// delete new option
static struct option long_options[] = {
         {"sectors", 1, NULL, 's'},
         {"heads", 1, NULL, 'h'},
         {"cylinders", 1, NULL, 'c'},
         {"header_crc", 1, NULL, 'g'},
         {"data_crc", 1, NULL, 'j'},
         {"format", 1, NULL, 'f'},
         {"drive", 1, NULL, 'd'},
         {"sector_length", 1, NULL, 'l'},
         {"unbuffered_seek", 0, NULL, 'u'},
         {"interleave", 1, NULL, 'i'},
         {"head_3bit", 0, NULL, '3'},
         {"retries", 1, NULL, 'r'},
         {"analyze", 2, NULL, 'a'},
         {"quiet", 1, NULL, 'q'},
         {"begin_time", 1, NULL, 'b'},
         {"transitions_file", 1, NULL, 't'},
         {"extracted_data_file", 1, NULL, 'e'},
         {"emulation_file", 1, NULL, 'm'},
         {"version", 0, NULL, 'v'},
         {"note", 1, NULL, 'n'},
         {"mark_bad", 1, NULL, 'M'},
         {NULL, 0, NULL, 0}
};
static char short_options[] = "s:h:c:g:d:f:j:l:ui:3r:a::q:b:t:e:m:vn:M:";

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
    int ignore_invalid_options)
{
   int rc;
   // Loop counters
   int i,j;
   int options_index;
   // Bit vector of which options were specified
   char *tok;
   char delete_list[sizeof(short_options)];
   int params_set;

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
      // Set defaults
      drive_params->emu_fd = -1;
      drive_params->tran_fd = -1;
      drive_params->ext_fd = -1;
      drive_params->step_speed = DRIVE_STEP_FAST;
      drive_params->retries = 50;
      drive_params->no_seek_retries = 4;
      drive_params->sector_size = 512;
      drive_params->emulation_output = 1; // This will be adjusted by caller
      drive_params->analyze = 0;
      drive_params->start_time_ns = 0;
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
         case 's':
            tok = strtok(optarg,",");
            drive_params->num_sectors = atoi(tok);
            tok = strtok(NULL,",");
            if (tok != NULL) {
               drive_params->first_sector_number = atoi(tok);
            }
            if (drive_params->num_sectors <= 0 ||
                   drive_params->num_sectors > MAX_SECTORS) {
               msg(MSG_FATAL,"Sectors must be 1 to %d\n", MAX_SECTORS);
               if (!ignore_invalid_options) {
                 exit(1);
               }
            }
            break;
         case 'h':
            drive_params->num_head = atoi(optarg);
            if (drive_params->num_head <= 0 || 
                  drive_params->num_head > MAX_HEAD) {
               msg(MSG_FATAL,"Heads must be 1 to %d\n", MAX_HEAD);
               if (!ignore_invalid_options) {
                  exit(1);
               }
            }
            break;
         case 'c':
            drive_params->num_cyl = atoi(optarg);
            if (drive_params->num_cyl <= 0) {
               msg(MSG_FATAL,"Cylinders must be greater than 0\n");
               if (!ignore_invalid_options) {
                  exit(1);
               }
            }
            break;
         case 'g':
            drive_params->header_crc = parse_crc(optarg);
            break;
         case 'j':
            drive_params->data_crc = parse_crc(optarg);
            break;
         case 'u':
            drive_params->step_speed = DRIVE_STEP_SLOW;
            break;
         case 'i':
            drive_params->sector_numbers = parse_interleave(optarg, drive_params);
            break;
         case '3':
            drive_params->head_3bit = 1;
            break;
         case 'f':
            drive_params->controller = parse_controller(optarg, 
               ignore_invalid_options, drive_params, &params_set);
            if (params_set) {
               drive_params->opt_mask |= controller_model_params;
            }
            break;
         case 'l':
            drive_params->sector_size = atoi(optarg);
            if (drive_params->sector_size <= 0 || 
                   drive_params->sector_size > MAX_SECTOR_SIZE) {
               msg(MSG_FATAL,"Sector size must be 1 to %d\n", MAX_SECTOR_SIZE);
               if (!ignore_invalid_options) {
                  exit(1);
               }
            }
            break;
         case 'r':
            drive_params->retries = atoi(optarg);
            tok = strstr(optarg,",");
            if (tok != NULL) {
               drive_params->no_seek_retries = atoi(tok+1);
            }
            break;
         case 'a':
            drive_params->analyze = 1;
            parse_analyze(optarg, drive_params);
            break;
         case 'b':
            drive_params->start_time_ns = atoi(optarg);
            break;
         case 't':
            drive_params->transitions_filename = optarg;
            break;
         case 'e':
            drive_params->extract_filename = optarg;
            break;
         case 'm':
            drive_params->emulation_filename = optarg;
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
         case 'n':
            drive_params->note = optarg;
            break;
         case '?':
            if (!ignore_invalid_options) {
               exit(1);
            }
            break;
         case 'M':
            drive_params->mark_bad_list = parse_mark_bad(optarg, drive_params);
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
// mfm_read: 1 if mfm_read, 0 if mfm_util

void parse_validate_options(DRIVE_PARAMS *drive_params, int mfm_read) {
   int i;
   // For mfm_util drive doesn't need to be specified. This
   // option error handling is getting messy.
   if (!mfm_read) {
      min_read_opts &= ~drive_opt;
   }
   // Corvus H  and Cromemco drive doesn't have separate header and data 
   // CRC. We use header for both
   if (drive_params->controller == CONTROLLER_CORVUS_H ||
       drive_params->controller == CONTROLLER_CROMEMCO) {
      min_read_opts &= ~data_crc_opt;
   }
   if ((drive_params->extract_filename != NULL)
         && !drive_params->analyze && (drive_params->opt_mask & min_read_opts)
             != min_read_opts) {
      msg(MSG_FATAL, "Generating extract file without analyze requires options:");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(drive_params->opt_mask & bit) && (min_read_opts & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   } 
   if (drive_params->controller != CONTROLLER_NONE
         && !drive_params->analyze && (drive_params->opt_mask & min_read_opts)
             != min_read_opts) {
      msg(MSG_FATAL, "Format specified without analyze requires options:");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(drive_params->opt_mask & bit) && (min_read_opts & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   } 
   if (mfm_read & (drive_params->transitions_filename != NULL || 
         drive_params->emulation_filename != NULL) && 
         !drive_params->analyze && 
        (drive_params->opt_mask & min_read_transitions_opts) !=
               min_read_transitions_opts) {
      msg(MSG_FATAL, "Generation of transition or emulation file without analyze requires options:\n");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(drive_params->opt_mask & bit) && 
               (min_read_transitions_opts & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   }
}

void parse_validate_options_listed(DRIVE_PARAMS *drive_params, char *opt) {
   int i;
   int fatal = 0;

   while (*opt != 0) {
      for (i = 0; i < ARRAYSIZE(long_options); i++) {
          if ( (*opt == long_options[i].val) && 
             !(drive_params->opt_mask & (1 << i)) ) {
            msg(MSG_FATAL, "Option %s must be specified\n", long_options[i].name);
            fatal = 1;
          }
      }
      opt++;
   }
   if (fatal) {
      exit(1);
   }
}
