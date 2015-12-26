// This module is routines for controlling the disk drive.
// drive_select selects the specified drive or none.
// drive_set_head sets the head select lines to select the specified head.
// drive_seek_track0 returns the head to track 0.
// drive_setup sets up the drive for reading.
// drive_read_disk reads the disk drive.
// drive_rpm get drive rpm
// drive_read_track read a track of deltas from a drive. It steps the
//    head if necessary.
// drive_step steps the head the requested number of cylinders.
// 
// The drive must be at track 0 on startup or drive_seek_track0 called.
//
// 12/24/2015 DJG Fix comment
// 07/30/2015 DJG Added support for revision B board.
// 05/16/2015 DJG Changes for drive_file.c
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
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
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
      for (i = 0; i < 4; i++) {
         fd[i] = open(drive_pins[i], O_WRONLY);
         if (fd[i] < 0) {
            msg(MSG_FATAL, "Unable to open pin %s\n", drive_pins[i]);
            exit(1);
         }
      }
      first_time = 0;
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
// err_fatal: If set an error will terminate, otherwise returned
int drive_step(int step_speed, int steps, int update_cyl, int err_fatal) {
   int seek_cmd;
   int rc;
   int wait_count = 0, ready = 0;

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
   }
   return rc;
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

// Read the disk.
//  drive params specifies the information needed to decode the drive and what
//    files should be written from the data read
//
// drive_params: Drive parameters
// deltas: Memory array containing the raw MFM delta time transition data
void drive_read_disk(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas)
{
   // Loop variable
   int cyl, head;
   int cntr;
   // Retry counter
   int err_cnt;
   // Read status
   SECTOR_DECODE_STATUS sector_status = 0;
   // The status of each sector
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   // We use this to summarize the list above has any errors
   int sect_err;
   // For retry we do various length seeks. This is the maximum length we will do
   int max_seek, seek_len;
   // Cylinder difference when a seek error occurs
   int seek_difference;

   // Timing stuff for testing
#ifdef CLOCK_MONOTONIC_RAW
#define CLOCK CLOCK_MONOTONIC_RAW
#else
#define CLOCK CLOCK_MONOTONIC
#endif
struct timespec tv_start;
   double start, last = 0;
   double min = 9e9, max = 0, tot = 0;
   int count = 0;

   // Determine maximum seek distance. Round up since we will
   // later limit to end of disk. Seeks are done in power of 2
   // steps with shift so calculate this in a power of 2 for the shift
   max_seek = ceil(log(drive_params->num_cyl) / log(2));
   if (max_seek == 0) {
      max_seek = 1; // Prevent possibility of divide by zero
   }

   // Start up delta reader
   deltas_start_thread(drive_params);

   mfm_decode_setup(drive_params, 1);

   for (cyl = 0; cyl < drive_params->num_cyl; cyl++) {
         // Slow seek doesn't slow down reading so always use it. When
         // only reading transition files this should work for all drives
         // without needing to specify slow seek
      drive_step(DRIVE_STEP_SLOW, 1, DRIVE_STEP_UPDATE_CYL, 
         DRIVE_STEP_FATAL_ERR);
      if (cyl % 5 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);
      for (head = 0; head < drive_params->num_head; head++) {
         int recovered = 0;

         err_cnt = 0;
         // Clear sector status here so we can see if different sectors
         // successfully read on different reads.
         mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
         do {
            // First error retry without seek, second with third without etc
            // Hopefully by moving head it will settle slightly differently
            // allowing data to be read
            if (err_cnt > 0 && ((err_cnt - 1) & 1)) {
               seek_len = 1 << (((err_cnt - 1) >> 2) % max_seek);
               if ((err_cnt - 1) & 2) {
                  seek_len = -seek_len;
               }
               if (cyl + seek_len >= drive_params->num_cyl) {
                  seek_len = drive_params->num_cyl - cyl - 1;
               }
               if (cyl + seek_len < 0) {
                  seek_len = -cyl;
               }
               //printf("seeking %d %d %d\n",err_cnt, seek_len, seek_len + cyl);
               if (seek_len != 0) {
                  drive_step(drive_params->step_speed, seek_len, 
                     DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
                  drive_step(drive_params->step_speed, -seek_len, 
                     DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
               }
            }
            clock_gettime(CLOCK, &tv_start);
            start = tv_start.tv_sec + tv_start.tv_nsec / 1e9;
            if (last != 0) {
#if 0
               if (start-last > 40e-3)
                  printf("gettime delta ms %f\n", (start-last) * 1e3);
#endif
               if (start-last > max)
                  max = start-last;
               if (start-last < min)
                  min = start-last;
               tot += start-last;
               count++;
            }
            last = start;

            drive_read_track(drive_params, cyl, head, deltas, max_deltas);

            sector_status = mfm_decode_track(drive_params, cyl, head,
               deltas, &seek_difference, sector_status_list);

            // See if sector list shows any with errors. The sector list
            // contains the information on the best read for each sector so
            // even if the last read had errors we may have recovered all the
            // data without errors.
            sect_err = 0;
            for (cntr = 0; cntr < drive_params->num_sectors; cntr++) {
               if (UNRECOVERED_ERROR(sector_status_list[cntr].status)) {
                  sect_err = 1;
               }
            }
            // Looks like a seek error. Sector headers which don't have the
            // expected cylinder such as when tracks are spared can trigger
            // this also
            if (sector_status & SECT_WRONG_CYL) {
               if (err_cnt < drive_params->retries) {
                  msg(MSG_ERR, "Retrying seek cyl %d, cyl off by %d\n", cyl,
                        seek_difference);
                  drive_step(drive_params->step_speed, seek_difference, 
                     DRIVE_STEP_NO_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
               }
            }

            if (UNRECOVERED_ERROR(sector_status) && !sect_err) {
               recovered = 1;
            }
         // repeat until we get all the data or run out of retries
         } while (sect_err && err_cnt++ < drive_params->retries);
         if (err_cnt > 0) {
            if (err_cnt == drive_params->retries + 1) {
               msg(MSG_ERR, "Retries failed cyl %d head %d\n", cyl, head);
            } else {
               msg(MSG_INFO, "All sectors recovered %safter %d retries cyl %d head %d\n",
                     recovered ? "from multiple reads " : "", err_cnt, cyl, head);
            }
         }
      }
   }

   mfm_decode_done(drive_params);

   printf("Track read time in ms min %f max %f avg %f\n", min * 1e3, 
         max * 1e3, tot * 1e3 / count);

   deltas_stop_thread();

   return;
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
      exit(1);
   }
}

// Read a track of deltas from a drive
//
// drive_params: Drive parameters
// cyl, head: Cylinder and head reading. Head is stepped to cylinder
//    and head selected
// deltas: Memory array containing the raw MFM delta time transition data
// max_deltas: Size of deltas array
void drive_read_track(DRIVE_PARAMS *drive_params, int cyl, int head, 
      void *deltas, int max_deltas) {

   if (cyl != current_cyl) {
      drive_step(drive_params->step_speed, cyl - current_cyl, 
         DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
   }

      // Analyze can change so set it every time
   pru_write_word(MEM_PRU0_DATA, PRU0_START_TIME_CLOCKS,
      drive_params->start_time_ns / CLOCKS_TO_NS);

   drive_set_head(head);

   if (pru_exec_cmd(CMD_READ_TRACK, 0) != 0) {
      drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
      exit(1);
   }
   // OK to start reading deltas
   deltas_start_read(cyl, head);
}
