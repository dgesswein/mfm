// This module is the routines needed to write a MFM disk. It does not
// attempt to handle bad locations on the disk. It does not have
// proper command line processing. Edit main. 
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
//
// 09/17/23 Changed to calling pru_exec_program to set correct path for file 
//          to load and board_set_restore_max_cpu_speed to have one copy
// 09/12/23 JST Changes to support 5.10 kernel and --sync option
// 05/05/23 GL  Fix by Gleb Larionov for hang on first write
// 01/18/21 DJG Updated function call
// 03/22/19 DJG Added REV C support
// 03/14/19 DJG Set PRU clock rate. Needed for SA1000 support
// 03/09/18 DJG Use drive specified on command line
// 06/30/17 DJG Set drive parameters to number of cylinder and heads in
//          emulation file. Get rid of hard coded values.
//
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
#include "drive.h"
#include "board.h"

#include "cmd.h"
#include "cmd_write.h"

// This routine is for cleaning up when shutting down. It is installed as an
// atexit routine and called by SIGINT handler.
void shutdown(void)
{
   static int called = 0;

   if (called)
      return;
   called = 1;

   board_set_restore_max_cpu_speed(1);
   // Turn off selected light on drive
   drive_select(0);

   pru_restart(0);

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
   // Size of shared memory in bytes
   int ddr_mem_size;
   EMU_FILE_INFO emu_file_info;


   board_initialize();

   // Find out what we should do
   parse_cmdline(argc, argv, &drive_params, "", 1, 0, 0, 0);
   parse_validate_options(&drive_params, 0);

   if (drive_params.emulation_filename == NULL) {
      msg(MSG_FATAL, "Emulation filenames must be specified\n");
      exit(1);
   }

   if (drive_params.drive == 0) {
      msg(MSG_FATAL, "Drive must be specified\n");
      exit(1);
   }

   drive_params.emu_fd = emu_file_read_header(
      drive_params.emulation_filename, &emu_file_info, 0, 0);
   drive_params.start_time_ns = emu_file_info.start_time_ns;
   drive_params.emu_file_info = &emu_file_info;
   drive_params.num_cyl = emu_file_info.num_cyl;
   drive_params.num_head = emu_file_info.num_head;


   // Initialize PRU
   ddr_mem_size = pru_setup(2);
   pru_set_clock(drive_params.emu_file_info->sample_rate_hz, PRU_SET_CLOCK_NO_HALT);
   if (ddr_mem_size == -1) {
      exit(1);
   }

   // And start our code
   if (pru_exec_program(0, "mfm_write0.bin") != 0) {
      msg(MSG_FATAL, "Unable to execute mfm_write0.bin\n");
      exit(1);
   }
   // DMA channel 7 and PaRAM blocks 7-8 are reserved in our new dto
   pru_write_word(MEM_PRU1_DATA,PRU1_DMA_CHANNEL, 7);		//  (Moved here by Gleb Larionov 05.05.2023)
   if (pru_exec_program(1, "mfm_write1.bin") != 0) {
      msg(MSG_FATAL, "Unable to execute mfm_write1.bin\n");
      exit(1);
   }

   pru_write_word(MEM_PRU0_DATA, PRU0_START_TIME_CLOCKS, 
      drive_params.start_time_ns / CLOCKS_TO_NS);
   pru_write_word(MEM_PRU0_DATA, PRU0_BOARD_REVISION, board_get_revision());
   pru_write_word(MEM_PRU1_DATA,PRU1_DRIVE0_TRACK_HEADER_BYTES,
      drive_params.emu_file_info->track_header_size_bytes);
   pru_write_word(MEM_PRU1_DATA,PRU1_DRIVE0_TRACK_DATA_BYTES,
      drive_params.emu_file_info->track_data_size_bytes);


   // Cleanup when we exit
   signal(SIGINT,(__sighandler_t) shutdown_signal);
   atexit(shutdown);

   // This is needed to keep up with the data. Without it we will take
   // more than 2 revolutions per track due to the default governor not
   // increasing the CPU speed enough. We switch frequently between busy and
   // sleeping.
   if (board_set_restore_max_cpu_speed(0)) {
      msg(MSG_ERR, "Unable to set CPU to maximum speed\n");
   }

   drive_setup(&drive_params);
   drive_write_disk(&drive_params);

   pru_exec_cmd(CMD_EXIT, 0);
   drive_select(0);

   return (0);
}

