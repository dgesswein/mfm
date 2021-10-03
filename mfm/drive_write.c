// This module is routines for writing a disk drive.
// drive_write_disk writes the disk
// 
// The drive must be at track 0 on startup or drive_seek_track0 called.
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
// 09/07/21 DJG Handle error from write track
// 03/07/21 DJG Only pad end if non zero
// 06/30/17 DJG Use emulator file number of heads, not command line.
//
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


static void generate_pwm_table(DRIVE_PARAMS *drive_params, int do_precomp);

// Write the disk.
//
// drive_params: Drive parameters
// deltas: Memory array containing the raw MFM delta time transition data
void drive_write_disk(DRIVE_PARAMS *drive_params)
{
   // Loop variable
   int cyl, head;
   int track_size, cyl_size;
   // Buffer for the drive data
   uint8_t *data;

   track_size = drive_params->emu_file_info->track_data_size_bytes +
         drive_params->emu_file_info->track_header_size_bytes;
   cyl_size = track_size * drive_params->emu_file_info->num_head;
   data = msg_malloc(cyl_size,"Write data buffer");

   generate_pwm_table(drive_params, 0);
   for (cyl = 0; cyl < drive_params->num_cyl; cyl++) {
      if (cyl >= drive_params->write_precomp_cyl) {
         generate_pwm_table(drive_params, 1);
      }
 
      if (cyl != 0) {
            // Slow seek doesn't slow down writing so always use it.
         drive_step(DRIVE_STEP_SLOW, 1, DRIVE_STEP_UPDATE_CYL, 
            DRIVE_STEP_FATAL_ERR);
      }
      if (cyl % 5 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);
      emu_file_read_cyl(drive_params->emu_fd,
         drive_params->emu_file_info, cyl, data,  cyl_size);
//TODO need to handle this better either in emulator or here.
// May not be needed now that files are no longer padded with zeros.
{ int n;
for (n = 0; n < drive_params->emu_file_info->num_head; n++) {
   int *d = (int *) data;
   int index = track_size/4 - 1 + n * track_size/4;
   // No transitions at end of write will cause emulator to fail
   if (d[index] == 0) {
      d[index] = 0x55555555;
   }
}
}
      pru_write_mem(MEM_DDR, data, cyl_size, 0);
      for (head = 0; head < drive_params->num_head; head++) {
         drive_set_head(head); 
         pru_write_word(MEM_PRU1_DATA, PRU1_CUR_HEAD, head);
         if (pru_exec_cmd(CMD_WRITE_TRACK, 0)) {
            drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
            exit(1);
         }
      }
   }
}


// Make a table which we index with the six MSB of the MFM bitstream
// to determine the next PWM word. 

// The MFM bitstream is the MFM data and clock bits referred to as
// mfm encoded here:
// http://en.wikipedia.org/wiki/Modified_Frequency_Modulation 

// All the words generated must have a minimum duration of 2 bit cells,
// nominally 40 clocks, 200 ns.
// For example bit pattern 10010[1] is sent as a pulse with period 60
// counts and a pulse with period 40 for 10 MHz data rate.

// If we find a 11 we drop the second bit since we can't generate it.
// It's bad MFM data so shouldn't cause problems. 01 we flag as an
// error since we should never see it unless something has gone
// wrong. Without changing the PWM polarity we can't generate a zero
// followed by a 1. We pick the bits we process to ensure that valid
// MFM data won't generate the patterns we can't encode.  For example
// bit pattern 1000 we only process the first two (10) and leave the
// last two bits (00) for the next lookup.  For valid MFM code we
// could generate the waveform for all four bits instead since the
// next bit should be a 1. I don't in case it is a zero.
// 
// Bits 31-28 are how many bits to remove from data. This is the MFM
//   clock and data bits we will shift off, not anything to do with the
//   data bits encoded by the MFM encoding.
//   The bits we remove were choosen to make sure that the left over
//   bits aren't a pattern we can't encode.
// Bits 27-25 unused
// bit  24 is flag for illegal bit pattern found
// Bits 23-16 are duration of 1. 0 is no one. (PWM ACMP)
// Bits 15-8 are signed period adjustment to apply to next word for write
//    precompensation in this word.
// Bits 7-0 is period to next bit time. (PWM APRD)
// Period values are one less than actual period generated.

// Drives construction of PRU1 bit lookup table
typedef struct {
   // Number of bits we can represent in a single PWM word (2 or 3)
   uint32_t bitcount;
   // Value of first bit (0 or 1)
   uint32_t leading_bit;
   // Error flag (see below)
   uint32_t error_flag;
} table_rec_t;

static table_rec_t bit_table[] = {
   { 2, 0, 0 }, // 00 0000
   { 3, 0, 0 }, // 00 0001
   { 2, 0, 0 }, // 00 0010
   { 2, 0, 0 }, // 00 0011

   // If the bit shifting algorithm is working correctly, we should
   // never see a pattern starting with '01'.  If we do, then
   // something is wrong and and error is flagged.

   { 2, 0, 1 }, // 00 0100 treated as 0000
   { 3, 0, 1 }, // 00 0101 treated as 0001
   { 2, 0, 1 }, // 00 0110 treated as 0010
   { 2, 0, 1 }, // 00 0111 treated as 0011

   { 2, 1, 0 }, // 00 1000
   { 3, 1, 0 }, // 00 1001
   { 2, 1, 0 }, // 00 1010
   { 2, 1, 0 }, // 00 1011

   // The next four patterns are not valid MFM data and cannot be
   // generated.  We just drop the second 1.

   { 2, 1, 0 }, // 00 1100 treated as 1000
   { 3, 1, 0 }, // 00 1101 treated as 1001
   { 2, 1, 0 }, // 00 1110 treated as 1010
   { 2, 1, 0 }  // 00 1111 treated as 1011
};

static void generate_pwm_table(DRIVE_PARAMS *drive_params, int do_precomp) {

   int idx;
   int pat;
   EMU_FILE_INFO *curr_info = drive_params->emu_file_info;
   int bit_period = lround(200e6 / curr_info->sample_rate_hz);
   int early_precomp_clk = lround(drive_params->early_precomp_ns / 1e9 * 200e6);
   int late_precomp_clk = lround(drive_params->late_precomp_ns / 1e9 * 200e6);

   pru_write_word(MEM_PRU0_DATA, PRU0_DEFAULT_PULSE_WIDTH, bit_period*2-1);


   for (idx = 0; idx < 64; idx++) {
      table_rec_t rec = bit_table[(idx >> 2) & 0xf];
      uint32_t bits = (rec.bitcount<<28) | (rec.error_flag<<24) | ((rec.leading_bit*bit_period)<<16) | ((rec.bitcount*bit_period)-1);
      if (rec.leading_bit) {
         // If generating 10 which will be followed be 100 then
         // gnerate the second one early
         if (do_precomp && (idx >> 1) == 0x14) {
            bits -= early_precomp_clk;
            bits |= (early_precomp_clk & 0xff) << 8;
         } 
      } else {
         pat = (idx << rec.bitcount) >> 3;
         // If two more zeros followed by 101 then we delay the first one
         if (do_precomp && pat == 0x5) {
            bits += late_precomp_clk;
            bits |= (-late_precomp_clk & 0xff) << 8;
         }
      }

      pru_write_word(MEM_PRU1_DATA, PRU1_BIT_TABLE + idx * PRU_WORD_SIZE_BYTES,
          bits);
   }

}
