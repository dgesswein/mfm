// This is a replacement for drive.c that reads from a transition file or
// a emulation file instead of a actual drive. See drive.c for function
// definitions
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
#include <inttypes.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "drive.h"
#include "deltas_read.h"



uint32_t drive_get_drive_status(void) {
   msg(MSG_FATAL, "drive_get_drive_status called\n");
   exit(1);
}

void drive_print_drive_status(int level, uint32_t status) {
   msg(MSG_FATAL, "drive_print_drive_status called\n");
   exit(1);
}

void drive_set_head(int head) {
}

int drive_step(int seek_speed, int steps, int update_cyl, int err_fatal) {
   return 0;
}

int drive_at_track0(void) {
   return 0;
}

void drive_seek_track0(void) {
}

void drive_select(int drive) {
   msg(MSG_FATAL, "drive_select called\n");
   exit(1);
}

void drive_setup(DRIVE_PARAMS *drive_params) {
   msg(MSG_FATAL, "drive_setup called\n");
   exit(1);
}

double drive_rpm() {
   return 0;
}

// This reads the proper track from the file and puts it in the delta array
// for decoding. Its a little wasteful to convert decoded bits from an
// emulation file back to deltas so we can convert back to bits but it
// allowed for minimum code changes.
//
void drive_read_track(DRIVE_PARAMS *drive_params, int cyl, int head,
      void *deltas, int max_deltas) {
   int num_deltas;

   if (drive_params->tran_fd != -1) {
      if (tran_file_seek_track(drive_params->tran_fd, cyl, head, 
            drive_params->tran_file_info)) {
         num_deltas = 0;
      } else {
         num_deltas = tran_file_read_track_deltas(drive_params->tran_fd,
               deltas, max_deltas, &cyl, &head);
      }
   } else {
      if (emu_file_seek_track(drive_params->emu_fd, cyl, head, 
            drive_params->emu_file_info)) {
         num_deltas = 0;
      } else {
         num_deltas = emu_file_read_track_deltas(drive_params->emu_fd,
               drive_params->emu_file_info,
               deltas, max_deltas, &cyl, &head);
      }
   }
   deltas_update_count(num_deltas, 0);
}
