// This module is routines for determing which version of the MFM board
//    is present.
// board_initialize sets up this module.
// board_get_revision returns the revision of the MFM emulator board
// 
// 03/22/2019 DJG Added REV C support
// 02/08/2017 DJG Fix incorrect format for print
// 10/16/2016 DJG Improved error message
// 12/24/2015 DJG Fix wrong value in error print
// 08/01/2015 DJG New module to support revision B board.
//
// Copyright 2019 David Gesswein.
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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

#include "msg.h"
#include "cmd.h"
#include "board.h"

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// 0 = first/A, 1 = B. Used to index arrays so can't change encoding
static int board_revision = -1; 

// Perform any setup needed by this module. Call once before any other
// routine
void board_initialize(void) {
   int pins[] = {REVC_DETECT_PIN, REVB_DETECT_PIN};
   int fd;
   int i;
   char str[128];
   if (board_revision == -1) {
         // Default is A
      board_revision = 0;
      for (i = 0; i < ARRAYSIZE(pins); i++) { 
         sprintf(str, "/sys/class/gpio/gpio%d/value", pins[i]);
         fd = open(str, O_RDONLY);
         if (fd < 0) {
            msg(MSG_FATAL, "Unable to open pin %d, did you run the setup script?\n", pins[i]);
            exit(1);
         }
         // Pin grounded to indicate version 
         pread(fd, str, sizeof(str), 0);
         close(fd);
         if (str[0] == '0') {
            board_revision = ARRAYSIZE(pins) - i;
            break;
         }
      }
   }
   msg(MSG_INFO, "Board revision %c detected\n", 'A' + board_revision);
}

// Return board revision 
int board_get_revision(void) {
   return board_revision;
}
