// This module is the routines needed to read a MFM disk. It can analyze the
// disk to determine format, read the disk and store raw delta transition data
// and decoded sector data.

// TODO Make handle more complex interleave like RD53 (cyl to cyl is 8, track
// to track is -1 or 16)
// TODO Use recovery line on Seagates to microstep instead of big seeks
//
// 03/22/19 DJG Added REV C support
// 04/20/18 DJG Fixed previous change to work properly with analyze
// 03/09/18 DJG Added ability to request reading more heads or cylinders
//   than analyze detects
// 09/07/16 DJG Report possible reversal of 20 pin cable
// 12/31/15 DJG Parameter change to parse_print_cmdline
// 08/02/15 DJG Added support for rev B board
// 05/17/15 DJG Added analyze of specified cylinder and head.
// 01/04/15 DJG Added support for Corvus_H and NorthStar Advantage
//    These had a sector straddle the index pulse so the start_time_ns
//    logic added to allow read to start with start of physical first sector.
//    Corvus_H also uses 11 MHz for clock/data bit cell so various changes to
//    support other rates.
//    Fixed off by 1 allocating command line storage
// 11/09/14 DJG Changes for note option
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
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <signal.h>

// MFM program include files
#include <prussdrv.h>

#include "msg.h"
#include "pru_setup.h"
#include "crc_ecc.h"
#define DEF_DATA
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "parse_cmdline.h"
#include "analyze.h"
#include "deltas_read.h"
#include "drive.h"
#include "board.h"

#include "cmd.h"

// This routine will either set the CPU speed to maximum or restore it
// back to the previous setting. The normal governor didn't boost the
// processor sufficiently to prevent getting behind in processing.
//
// restore: 0 to set speed to maximum, 1 to restore value from previous
//    call
int set_restore_max_cpu_speed(int restore) {
   FILE *file;
   static char governor[100];
   static int freq_changed = 0;
   char maxfreq[100];

   if (!restore) {
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "r");
      if (file == NULL) {
         return -1;
      }
      if (fscanf(file, "%100s", governor) < 1) {
         return -1;
      }
      fclose(file);
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "w");
      if (file == NULL) {
         return -1;
      }
      if (fprintf(file,"userspace") < 1) {
         return -1;
      }
      fclose(file);
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq", "r");
      if (file == NULL) {
         return -1;
      }
      if (fscanf(file, "%100s", maxfreq) < 1) {
         return -1;
      }
      fclose(file);

      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed", "w");
      if (file == NULL) {
         return -1;
      }
      if (fprintf(file,maxfreq) < 1) {
         return -1;
      }
      fclose(file);
      freq_changed = 1;
   } else {
      if (freq_changed) {
         file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "w");
         if (file == NULL) {
            return -1;
         }
         if (fprintf(file,governor) < 1) {
            return -1;
         }
         fclose(file);
      }
      freq_changed = 0;
   }
   return 0;
}


// This routine is for cleaning up when shutting down. It is installed as an
// atexit routine and called by SIGINT handler.
void shutdown(void)
{
   static int called = 0;

   if (called)
      return;
   called = 1;

   set_restore_max_cpu_speed(1);
   // Turn off selected light on drive
   drive_select(0);

   pru_restart(0);

   deltas_stop_thread();
   pru_exec_cmd(CMD_EXIT, 0);
   pru_shutdown();
}

// SIGINT/ control-c handler. Call our shutdown and exit
void shutdown_signal(void)
{
   shutdown();
   exit(1);
}

// Main routine. If specified analyze disk format and read disk contents
int main(int argc, char *argv[])
{
   // Drive parameters either from command line or analyzing the disk
   DRIVE_PARAMS drive_params;
   // What we wish to do
   int read = 0;
   // Memory deltas from PRU are put in
   uint32_t *deltas;
   // Size of shared memory/deltas in bytes
   int ddr_mem_size;
   char *cmdline;
   int max_deltas;

   board_initialize();

   // Find out what we should do
   parse_cmdline(argc, argv, &drive_params, "M", 1, 0, 0);
   parse_validate_options(&drive_params, 1);

   // If they specified a file name then we read the disk
   if (drive_params.extract_filename != NULL ||
         drive_params.emulation_filename != NULL ||
         drive_params.transitions_filename != NULL) {
      read = 1;
   }
   if (!read && !drive_params.analyze) {
      msg(MSG_FATAL, "Analyze and/or output filenames must be specified\n");
      exit(1);
   }

   // Initialize PRU
   ddr_mem_size = pru_setup(1);
   if (ddr_mem_size == -1) {
      exit(1);
   }

   // And start our code
   if (prussdrv_exec_program(0, "prucode0.bin") != 0) {
      msg(MSG_FATAL, "Unable to execute prucode0.bin\n");
      exit(1);
   }

   pru_write_word(MEM_PRU0_DATA, PRU0_START_TIME_CLOCKS, 
      drive_params.start_time_ns / CLOCKS_TO_NS);

   pru_write_word(MEM_PRU0_DATA, PRU0_BOARD_REVISION, board_get_revision());

   deltas = deltas_setup(ddr_mem_size);
   max_deltas = ddr_mem_size / sizeof(deltas[0]);

   // Cleanup when we exit
   signal(SIGINT,(__sighandler_t) shutdown_signal);
   atexit(shutdown);

   // This is needed to keep up with the data. Without it we will take
   // more than 2 revolutions per track due to the default governor not
   // increasing the CPU speed enough. We switch frequently between busy and
   // sleeping.
   if (set_restore_max_cpu_speed(0)) {
      msg(MSG_ERR, "Unable to set CPU to maximum speed\n");
   }

   if (!(drive_get_drive_status() & BIT_MASK(R31_DRIVE_SEL))) {
      msg(MSG_ERR,"** Drive selected without select, is J3 20 pin cable reversed? **\n");
   }

   drive_params.noretry_head = drive_params.num_head;
   drive_params.noretry_cyl = drive_params.num_cyl;
   if (drive_params.analyze) {
      int head_cmdline = drive_params.num_head;
      int cyl_cmdline = drive_params.num_cyl;
      analyze_disk(&drive_params, deltas, max_deltas, 0);
      msg(MSG_INFO,"\n");
      // Print analysis results
      parse_print_cmdline(&drive_params, 1, 0);
      drive_params.noretry_head = drive_params.num_head;
      if (head_cmdline > drive_params.num_head) {
         drive_params.num_head = head_cmdline;
      }
      drive_params.noretry_cyl = drive_params.num_cyl;
      if (cyl_cmdline > drive_params.num_cyl) {
         drive_params.num_cyl = cyl_cmdline;
      }
   }
   cmdline = parse_print_cmdline(&drive_params, 0, 0);
   drive_params.cmdline = msg_malloc(strlen(cmdline)+1,"main cmdline");
   strcpy(drive_params.cmdline, cmdline);

   if (read) {
      drive_setup(&drive_params);
      drive_read_disk(&drive_params, deltas, max_deltas);
   }

   pru_exec_cmd(CMD_EXIT, 0);
   drive_select(0);

   return (0);
}

