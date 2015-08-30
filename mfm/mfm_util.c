// This is a utility program to process existing MFM delta transition data.
// Used to extract the sector contents to a file
//
// 07/30/15 DJG Added support for revision B board.
// 05/17/15 DJG Added ability to analyze transition and emulation files.
//              Added ability to analyze specific cylinder and head.
//
// 10/24/14 DJG Changes necessary due to addition of mfm_emu write buffer
//     using decode options stored in header, and minor reformatting
//
// Copyright 2015 David Gesswein.
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
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <errno.h>
#include <libiberty.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#define DEF_DATA
#include "mfm_decoder.h"
#include "parse_cmdline.h"
#include "analyze.h"
#include "deltas_read.h"  // Code is from deltas_read_file.c
#include "board.h"

#define MAX_DELTAS 131072

// Main routine
int main (int argc, char *argv[])
{
   uint16_t deltas[MAX_DELTAS];
   int num_deltas;
   int cyl, head;
   int last_cyl = -1, last_head = -1;
   DRIVE_PARAMS drive_params;
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   int seek_difference;
   //SECTOR_DECODE_STATUS status;
   int transition_file = 1;
   EMU_FILE_INFO emu_file_info;
   TRAN_FILE_INFO tran_file_info;

   // If they specified a transistions file get options that were stored
   // in it.
   parse_cmdline(argc, argv, &drive_params, "tm", 1, 1, 1);
   if (drive_params.transitions_filename != NULL ||
         drive_params.emulation_filename != NULL) {
      char **tran_argv, **targs;
      int tran_argc = 0;
      char *orig_cmdline;
      char *orig_note;
      char *cmdline;

      if (drive_params.transitions_filename != NULL) {
         drive_params.tran_fd = tran_file_read_header(
            drive_params.transitions_filename, &tran_file_info);
         drive_params.start_time_ns = tran_file_info.start_time_ns;
         drive_params.tran_file_info = &tran_file_info;
         orig_cmdline = tran_file_info.decode_cmdline;
         orig_note = tran_file_info.note;
      } else {
         drive_params.emu_fd = emu_file_read_header(
            drive_params.emulation_filename, &emu_file_info, 0);
         drive_params.emu_file_info = &emu_file_info;
         orig_cmdline = emu_file_info.decode_cmdline;
         orig_note = emu_file_info.note;
      }
      if (orig_cmdline != NULL) {
         msg(MSG_INFO, "Origional decode arguments: %s\n", orig_cmdline);
         if (orig_note != NULL && strlen(orig_note) != 0) {
            msg(MSG_INFO, "Note: %s\n", orig_note);
         }
         // We need first argument of new argv to be program name argv[0]
         cmdline = msg_malloc(strlen(orig_cmdline) + strlen(argv[0]) + 1, 
            "main cmdline");
         strcpy(cmdline, argv[0]);
         strcat(cmdline, " ");
         strcat(cmdline, orig_cmdline);

         tran_argv = buildargv(cmdline);
         for (targs = tran_argv; *targs != NULL; targs++) {
            tran_argc++;
         }
         // Allow the arj options from stored command line, we will ignore
         // them in drive_params. Don't make errors fatal in case bad
         // option gets in header
         parse_cmdline(tran_argc, tran_argv, &drive_params, "", 0, 0, 1);
       
         free(cmdline);
      }
   }
   // Now parse the full command line. This allows overriding options that
   // were in the transition file header.
   parse_cmdline(argc, argv, &drive_params, "rd", 0, 0, 0);
   // Save final parameters
   drive_params.cmdline = parse_print_cmdline(&drive_params, 0);

   // And do some checking
   parse_validate_options(&drive_params, 0);


   if (drive_params.transitions_filename == NULL) {
      if (drive_params.emulation_filename == NULL) {
         msg(MSG_FATAL, "Transition or emulation filename must be specified\n");
         exit(1);
      } else {
         transition_file = 0;
         // Fix up from parse_cmdline. We read emulation file not write it if
         // no transition file is specified
         // Open if it we didn't open above in getting saved command arguments
         drive_params.emulation_output = 0;
         if (drive_params.emu_fd == -1) {
            drive_params.emu_fd = emu_file_read_header(
               drive_params.emulation_filename, &emu_file_info, 0);
         }
      }
   }
   // In analyze requested get drive parameters.
   if (drive_params.analyze && 
         (drive_params.transitions_filename != NULL || 
               drive_params.emulation_filename != NULL)) {
      analyze_disk(&drive_params, deltas, MAX_DELTAS, 1);
      msg(MSG_INFO,"\n");
      // Print analysis results
      parse_print_cmdline(&drive_params, 1);
      // If we are also converting data set up for it. Otherwise exit
      if ((drive_params.transitions_filename != NULL && 
            drive_params.emulation_filename != NULL) ||
            drive_params.extract_filename != NULL) {
         // Go back to begining of file
         if (drive_params.tran_fd != -1) {
            tran_file_seek_track(drive_params.tran_fd, 0, 0,
                  drive_params.tran_file_info->file_header_size_bytes);
         } else {
            emu_file_seek_track(drive_params.emu_fd, 0, 0,
                  drive_params.emu_file_info);
         }
      } else {
         exit(1);
      }
   }

   if (drive_params.transitions_filename != NULL && 
       drive_params.emulation_filename == NULL && 
       drive_params.extract_filename == NULL) {
      msg(MSG_FATAL, "Must specify emulation and or extract file to be generated\n");
      exit(1);
   }
   if (drive_params.transitions_filename == NULL && 
       drive_params.emulation_filename != NULL && 
       drive_params.extract_filename == NULL) {
      msg(MSG_FATAL, "Must specify extract file to be generated\n");
      exit(1);
   }

   // Setup decoding of transitions and possible file to write to
   mfm_decode_setup(&drive_params, 1);

   if (transition_file) {
      num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
            deltas, MAX_DELTAS, &cyl, &head);
   } else {
      num_deltas = emu_file_read_track_deltas(drive_params.emu_fd,
            &emu_file_info, deltas, MAX_DELTAS, &cyl, &head);
   }
   // Read and process a track at a time until all read
   while (num_deltas >= 0) {
      // Only clear status if we are moving to the next track. If retries
      // were done we may have multiple reads of the same track.
      if (last_cyl != cyl || last_head != head) {
         mfm_init_sector_status_list(sector_status_list,
               drive_params.num_sectors);
      }
      //printf("Decoding new track %d %d\n",cyl, head);
      deltas_update_count(num_deltas, 0);
      // If head & cylinder haven't changed assume it's a retry.
      mfm_decode_track(&drive_params, cyl, head, deltas,
            &seek_difference, sector_status_list);
#if 0
      if (1 || status != SECT_HEADER_FOUND) {
         if (cyl == 0 && head == 0) {
            FILE *file;
            char fn[256];
            int i;
            sprintf(fn, "trk%d-%d.txt",cyl, head);
            file = fopen(fn, "w");
            for (i = 0; i < num_deltas; i++) {
               fprintf(file, "%d\n",deltas[i]);
            }
            fclose(file);
         }
      }
#endif
      last_cyl = cyl;
      last_head = head;
      if (transition_file) {
         num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
               deltas, MAX_DELTAS, &cyl, &head);
         while (head >= drive_params.num_head) {
            static int msg_printed = 0;
            if (!msg_printed) {
               msg(MSG_INFO, "Warning, data has more heads than specified. Data for head >= %d ignored\n", head);
               msg_printed = 1;
            }
            num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
               deltas, MAX_DELTAS, &cyl, &head);
         }
      } else {
         num_deltas = emu_file_read_track_deltas(drive_params.emu_fd,
               &emu_file_info, deltas, MAX_DELTAS, &cyl, &head);
      }
   }
   mfm_decode_done(&drive_params);
   return 0;
}
