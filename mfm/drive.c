// This module is routines for controlling the disk drive.
// drive_select selects the specified drive or none.
// drive_set_head sets the head select lines to select the specified head.
// drive_seek_track0 returns the head to track 0.
// drive_setup sets up the drive for reading.
// drive_read_disk reads the disk drive.
// drive_current_cyl gets the current cylinder the drive heads are on
// drive_rpm get drive rpm
// drive_read_track read a track of deltas from a drive. It steps the
//    head if necessary.
// drive_step steps the head the requested number of cylinders.
// drive_is_file: Indicate if real drive or reading from file
// 
// The drive must be at track 0 on startup or drive_seek_track0 called.
//
// 03/09/2018 DJG Make sure correct setup script run so pins are in correct
//    direction.
// 10/02/2016 DJG Rob Jarratt change for DEC RD drives to detect when
//    it recalibrates back to track 0 when stepping past end
// 02/20/2016 DJG Split for drive reading and writing
// 01/06/2016 DJG Detect reversed J4 cable
// 12/24/2015 DJG Fix comment
// 07/30/2015 DJG Added support for revision B board.
// 05/16/2015 DJG Changes for drive_file.c
//
// Copyright 2014-2016 David Gesswein.
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

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "cmd_write.h"
#include "deltas_read.h"
#include "pru_setup.h"
#include "drive.h"
#include "board.h"


// The cylinder the drive heads are at
static int current_cyl;

// Activate drive selects to select specified drive
//
// drive: drive number to select (1-4) or zero to select none.
void drive_select(int drive)
{
   char *drive_pins[] = {
         "/sys/class/gpio/gpio22/direction",
         "/sys/class/gpio/gpio23/direction",
         "/sys/class/gpio/gpio26/direction",
         "/sys/class/gpio/gpio27/direction"
   };
   static int first_time = 1;
   static int fd[4];
   int i;

   if (drive < 0 || drive > 4) {
      msg(MSG_FATAL, "Invalid drive %d\n", drive);
      exit(1);
   }
   // Open the files once
   if (first_time) {
      int rc;
      char buf[100];

      first_time = 0;
      for (i = 0; i < 4; i++) {
         fd[i] = open(drive_pins[i], O_RDWR);
         if (fd[i] < 0) {
            msg(MSG_FATAL, "Unable to open pin %s, did you run the setup script?\n", drive_pins[i]);
            exit(1);
         }
      }
      i = 0;
      // Make sure correct setup run
      rc = read(fd[i], buf, sizeof(buf));
      if (rc != 4) {
         if (rc < 0) {
            msg(MSG_FATAL, "Error reading pin %s, did you run the setup script?\n", drive_pins[i]);
            exit(1);
         }
         // Remove newline
         if (rc > 1) {
            buf[rc-1] = 0;
         }
         msg(MSG_FATAL, "Wrong pin setting pin %s - %s, must reboot between reading and emulation.\n", drive_pins[i], buf);
         exit(1);
      }
   }
   // Set the signals to the correct state
   for (i = 0; i < 4; i++) {
      if (drive == (i + 1)) {
         write(fd[i], "high", 4); // Signal is active low with inverting driver
      } else {
         write(fd[i], "low", 3); 
      }
   }
   usleep(10); // Allow lines to settle
}

// Select the disk head. Head 0-15 supported. The MSB of head select
// is reduced write current on older drives but since we aren't writing
// we don't have to worry about it.
//
// head: Head to select 0-15
void drive_set_head(int head)
{
   char *head_pins[2][4] = {
      {
         "/sys/class/gpio/gpio2/direction",
         "/sys/class/gpio/gpio3/direction",
         "/sys/class/gpio/gpio4/direction",
         "/sys/class/gpio/gpio5/direction"
      },
      {
         "/sys/class/gpio/gpio8/direction",
         "/sys/class/gpio/gpio9/direction",
         "/sys/class/gpio/gpio10/direction",
         "/sys/class/gpio/gpio11/direction"
      }
   };
   static int first_time = 1;
   static int fd[4];
   int i;
   int board_revision;

   board_revision = board_get_revision();
   if (head < 0 || head > 15) {
      msg(MSG_FATAL, "Invalid head %d\n", head);
      exit(1);
   }
   // Open the files once
   if (first_time) {
      for (i = 0; i < 4; i++) {
         fd[i] = open(head_pins[board_revision][i], O_WRONLY);
         if (fd[i] < 0) {
            msg(MSG_FATAL, "Unable to open pin %s\n", 
               head_pins[board_revision][i]);
            exit(1);
         }
      }
      first_time = 0;
   }
   // Set the signals to the correct state
   for (i = 0; i < 4; i++) {
      if (head & (1 << i)) {
         write(fd[i], "high", 4); // Signal is active low with inverting driver
      } else {
         write(fd[i], "low", 3);
      }
   }
   usleep(10); // Allow lines to settle
}

// Returns non zero if track 0 signal is active (zero)
int drive_at_track0(void)
{
   static int fd = -1;
   char str[128];
   if (fd == -1) {
      sprintf(str, "/sys/class/gpio/gpio%d/value", GPIO0_TRACK_0);
      fd = open(str, O_RDONLY);
      if (fd < 0) {
         msg(MSG_FATAL, "Unable to open pin %d\n", GPIO0_TRACK_0);
         exit(1);
      }
   }
   pread(fd, str, sizeof(str), 0);
   // 0 is at track 0
   return str[0] == '0';
}

// Return the heads to track 0. This is slow so if you know what track you
// are on use a normal seek.
void drive_seek_track0(void)
{
   int rc;
   rc = pru_exec_cmd(CMD_SEEK_SLOW_TRACK0, -MAX_CYL);
   if (rc != 0) {
      drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
      exit(1);
   };
   // Low indicates we are at track 0
   if (!drive_at_track0()) {
      msg(MSG_FATAL,"Failed to reach track 0\n");
      exit(1);
   }
   current_cyl = 0;
}

// Step requested number of cylinders
// buffered_seek: nonzero if drive supports buffered seeks
// steps: Number of cylinders to step, negative is towards lower cylinder
// update_cyl: Non zero if internal current cylinder should be updated
// err_fatal: If set a timeout error will terminate, otherwise returned
//    as SEEK_TIMEOUT. SEEK_RECAL indicates the drive recalibrated to track 0
int drive_step(int step_speed, int steps, int update_cyl, int err_fatal) {
   int seek_cmd;
   int rc;
   int wait_count = 0, ready = 0;
   int start_cyl = current_cyl;

   if (update_cyl == DRIVE_STEP_UPDATE_CYL) {
      current_cyl += steps;
   }
   if (step_speed == DRIVE_STEP_FAST) {
      seek_cmd = CMD_SEEK_FAST;
   } else {
      seek_cmd = CMD_SEEK_SLOW;
   }
   if ((rc = pru_exec_cmd(seek_cmd, steps)) != 0) {
      if (err_fatal) {
         msg(MSG_FATAL,"seek command failed\n");
         drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
         exit(1);
      } else {
         // Wait for seek complete for 5 seconds. This prevents errors if we
         // next read the disk
         while (wait_count++ < 50 && !ready) {
            // Bit is active low
            ready = !(drive_get_drive_status() & BIT_MASK(R31_SEEK_COMPLETE_BIT));
            usleep(100000);
         }
      }
      rc = DRIVE_STEP_TIMEOUT;
   }

   if (drive_at_track0() && start_cyl != 0 && current_cyl != 0) {
      msg(MSG_INFO, "Disk has recalibrated to track 0\n");
      current_cyl = 0;
      rc = DRIVE_STEP_RECAL;
   }

   return rc;
}

// Get current cylinder
int drive_current_cyl() {
   return current_cyl;
}

// Select drive, check if drive is ready and then return to track zero if needed
//
// drive_params: Drive parameters
void drive_setup(DRIVE_PARAMS *drive_params)
{
   drive_select(drive_params->drive);

   if (pru_exec_cmd(CMD_CHECK_READY, 0)) {
      drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
      exit(1);
   }

   if (!drive_at_track0()) {
      msg(MSG_INFO, "Returning to track 0\n");
      drive_seek_track0();
   }
}

// Return the drive status value. See print routine for bit definitions.
uint32_t drive_get_drive_status(void)
{
   return pru_read_word(MEM_PRU0_DATA, PRU0_STATUS);
}

// Decodes and prints the disk status value from the PRU.
//
// status: Disk status value (from pru_get_drive_status)
void drive_print_drive_status(int level, uint32_t status)
{
   struct {
      char *desc;
      int bit;
   } reg_bits[] = {
         {"Write fault", R31_WRITE_FAULT_BIT},
         {"Seek complete", R31_SEEK_COMPLETE_BIT},
         {"Index", R31_INDEX_BIT},
         {"Ready", R31_READY_BIT},
         {"Drive selected", R31_DRIVE_SEL},
 // TODO        {"Track 0", R31_TRACK_0}
   };
   int n;

   for (n = 0; n < ARRAYSIZE(reg_bits); n++) {
      // Control lines are active low
      if (status & (1 << reg_bits[n].bit)) {
         msg(level, "Not %s \n", reg_bits[n].desc);
      } else {
         msg(level, "%s \n", reg_bits[n].desc);
      }
   }
}


// Print the drive Revolutions Per Minute (RPM)
// return drive RPM
double drive_rpm(void) {
   if (pru_exec_cmd(CMD_RPM, 0) == 0) {
      return(200e6 / pru_get_cmd_data() * 60);
   } else {
      msg(MSG_FATAL, "Drive RPM failed\n");
      if ((pru_read_word(MEM_PRU0_DATA, PRU0_STATUS) & 
         ((1 << R31_INDEX_BIT) | (1 << R31_WRITE_FAULT_BIT))) == 0) {
         msg(MSG_FATAL, "Is J4 cable plugged in backward?\n");
      }
      exit(1);
   }
}
