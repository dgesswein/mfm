// This routine decodes Xebec formated disks.
// The format is Headers is marked with an 0xa1 byte. That is followed
// by a sync bit which is a one between a bunch of zeros. That synchronizes
// the byte decoding. The data portion of the sector only has the one
// sync bit.
//
// 04/23/15 DJG Added support for EC1841, Thanks to Denis Kushch for changes
//    needed.
// 12/31/15 DJG Parameter change to mfm_mark_*
// 11/01/15 DJG Use new drive_params field and comment changes
// 05/17/15 DJG Code cleanup
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
#define DEBUG 0
#define PRINT_ERR 1
#define WINDOW_FILTER 0
#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "msg.h"
#include "deltas_read.h"

#define DATA_IGNORE_BYTES 34

// Type II PLL. Here so it will inline. Converted from continuous time
// by bilinear transformation. Coefficients adjusted to work best with
// my data. Could use some more work.
static inline float filter(float v, float *delay)
{
   float in, out;
                 
   in = v + *delay;
   out = in * 0.034446428576716f + *delay * -0.034124999994713f;
   *delay = in;
   return out;
}

// Decode bytes into header or sector data for the various formats we know about.
// The decoded data will be written to a file if one was specified.
// Since processing a header with errors can overwrite other good sectors this routine
// shouldn't be called with header data that has a CRC error.
//
// The format names are arbitrarily assigned to the first controller found
// writing that format.
// The format is
//   CONTROLLER_XEBEC_104786,
//   Closest manuals found http://bitsavers.trailing-edge.com/pdf/xebec/
//   9 byte header + 4 byte CRC
//      byte 0-1 0x00
//      byte 2 0xc2
//      byte 3 high bits of cylinder
//      byte 4 low 8 bits of cylinder
//      byte 5 bits 0-3 head number. Unknown if other bits used
//      byte 6 sector number
//      byte 7 flag bits. MSB is always set and bit 4 set on last sector
//      byte 8 0x00
//      bytes 9-12 32 bit ECC
//   Data
//      byte 0 0x00
//      byte 1 0xc9
//      Sector data for sector size
//      4 byte ECC code
//
//   CONTROLLER_EC1841
//      Same as XEBEC_104786 except compare byte is 0x00, not 0xc9
//      Sector numbers start at 3 for first sector and wrap back to 0 at
//      number of sectors.
//      Needs --begin_time 220000
//
// TODO: Same as WD decoder
SECTOR_DECODE_STATUS xebec_process_data(STATE_TYPE *state, uint8_t bytes[],
         uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
         DRIVE_PARAMS *drive_params, int *seek_difference,
         SECTOR_STATUS sector_status_list[], int ecc_span)
{
   static int sector_size;
   static int bad_block;
   static SECTOR_STATUS sector_status;
   int compare_byte;

   if (*state == PROCESS_HEADER) {
      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status |= SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      sector_status.cyl = bytes[3]<< 8;
      sector_status.cyl |= bytes[4];
      sector_status.head = bytes[5] & 0xf;
      if (drive_params->controller == CONTROLLER_EC1841) {
         if (bytes[6]<3) {
             sector_status.sector = bytes[6]+14;
         } else {
             sector_status.sector = bytes[6]-3;
         }
      } else {
         sector_status.sector = bytes[6];
      }
      // Don't know how/if these are encoded in header
      sector_size = drive_params->sector_size;
      bad_block = 0;
      if ((bytes[5] & 0xf0) != 0) {
         msg(MSG_INFO, "Upper bits set in head byte: %02x on cyl %d head %d sector %d\n",
               bytes[5],
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_HEADER;
      }
      if (bytes[0] != 0 || bytes[1] != 0 || bytes[8] != 0) {
         msg(MSG_INFO, "Header gap bytes not zero: %02x, %02x, %02x on cyl %d head %d sector %d\n",
               bytes[0], bytes[1], bytes[8],
               sector_status.cyl, sector_status.head, sector_status.sector);
      }
      compare_byte = 0xc2;
      if (bytes[2] != compare_byte) {
         msg(MSG_INFO, "Header compare byte not 0x%02x: %02x on cyl %d head %d sector %d\n",
               compare_byte, bytes[2],
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_HEADER;
      }
      // More stuff likely in here but not documented.
      if (bytes[7] != 0x80 && bytes[7] != 0x90) {
         msg(MSG_INFO, "Header flag byte not 0x80 or 0x90: %02x on cyl %d head %d sector %d\n",
               bytes[7],
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_HEADER;
      }
      msg(MSG_DEBUG,
         "Got exp %d,%d cyl %d head %d sector %d size %d bad block %d\n",
            exp_cyl, exp_head, sector_status.cyl, sector_status.head, sector_status.sector,
            sector_size, bad_block);

      mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
            seek_difference, &sector_status, drive_params);

      *state = DATA_SYNC;
   } else { // Data
      if (bytes[0] != 0 ) {
         msg(MSG_INFO, "Data gap byte not zero %02x on cyl %d head %d sector %d\n",
               bytes[0],
               sector_status.cyl, sector_status.head, sector_status.sector);
      }
      if (drive_params->controller == CONTROLLER_EC1841) {
         compare_byte = 0x00;
      } else {
         compare_byte = 0xc9;
      }
      if (bytes[1] != compare_byte) {
         msg(MSG_INFO, "Data compare byte not 0xc9: %02x on cyl %d head %d sector %d\n",
               bytes[1],
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_DATA;
      }
      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      sector_status.ecc_span_corrected_data = ecc_span;
      if (!(sector_status.status & SECT_BAD_HEADER)) {
         if (mfm_write_sector(&bytes[2], drive_params, &sector_status,
               sector_status_list, &bytes[0], drive_params->sector_size +
               drive_params->data_crc.length + 2) == -1) {
            sector_status.status |= SECT_BAD_HEADER;
         }
      }
      *state = MARK_ID;
   }
   return sector_status.status;
}

// Decode a track's worth of deltas.
//
//
// drive_params: Drive parameters
// cyl,head: Physical Track data from
// deltas: MFM delta data to decode
// seek_difference: Return of difference between expected cyl and header
// sector_status_list: Return of status of decoded sector
// return: Or together of the status of each sector decoded
SECTOR_DECODE_STATUS xebec_decode_track(DRIVE_PARAMS *drive_params, int cyl,
      int head, uint16_t deltas[], int *seek_difference,
      SECTOR_STATUS sector_status_list[])
{
   // This is which MFM clock and data bits are valid codes. So far haven't
   // found a good way to use this.
   //int valid_code[16] = { 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 };

   // This converts the MFM clock and data bits into data bits.
   int code_bits[16] = { 0, 1, 0, 0, 2, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };
   // This is the raw MFM data decoded with above
   unsigned int raw_word = 0;
   // Counter to know when to decode the next two bits.
   int raw_bit_cntr = 0;
   // The decoded bits
   unsigned int decoded_word = 0;
   // Counter to know when we have a bytes worth
   int decoded_bit_cntr = 0;
   // loop counter
   int i;
   // These are variables for the PLL filter. avg_bit_sep_time is the
   // "VCO" frequency
   float avg_bit_sep_time = 20; // 200 MHz clocks
   // Clock time is the clock edge time from the VCO.
   float clock_time = 0;
   // How many bits the last delta corresponded to
   int int_bit_pos;
   // PLL filter state. Static works better since next track bit timing
   // similar to previous though a bad track can put it off enough that
   // the next track has errors. Retry should fix. TODO: Look at
   static float filter_state = 0;
   // Time in track for debugging
   int track_time = 0;
   // Counter for debugging
   int tot_raw_bit_cntr = 0;
   // Where we are in decoding a sector, Start looking for header ID mark
   STATE_TYPE state = MARK_ID;
   // Status of decoding returned
   int sector_status = SECT_NO_STATUS;
   // How many zeros we need to see before we will look for the 0xa1 byte.
   // When write turns on and off can cause codes that look like the 0xa1
   // so this avoids them.
#define MARK_NUM_ZEROS 8
   int sync_count = 0;
   // Number of deltas available so far to process
   int num_deltas;
   // And number from last time
   int last_deltas = 0;
   // Intermediate value
   int tmp_raw_word;
   // Collect bytes to further process here
   uint8_t bytes[MAX_SECTOR_SIZE + 50];
   // How many we need before passing them to the next routine
   int bytes_needed = 0;
   // Length to perform CRC over
   int bytes_crc_len = 0;
   // how many we have so far
   int byte_cntr = 0;
   // Sequential counter for counting sectors
   int sector_index = 0;
   // Count all the raw bits for emulation file
   int all_raw_bits_count = 0;

   num_deltas = deltas_get_count(0);

   raw_word = 0;
   i = 1;
   while (num_deltas >= 0) {
      // We process what we have then check for more.
      for (; i < num_deltas; i++) {
         track_time += deltas[i];
         // This is simulating a PLL/VCO clock sampling the data.
         clock_time += deltas[i];
         // Move the clock in current frequency steps and count how many bits
         // the delta time corresponds to
         for (int_bit_pos = 0; clock_time > avg_bit_sep_time / 2;
               clock_time -= avg_bit_sep_time, int_bit_pos++) {
         }
         // And then filter based on the time difference between the delta and
         // the clock
         avg_bit_sep_time = 20.0 + filter(clock_time, &filter_state);
#if DEBUG
         //printf("track %d clock %f\n", track_time, clock_time);
         printf
         ("  delta %d skew %.2f %.2f bit pos %.2f int bit pos %d avg_bit %.2f time %d\n",
               deltas[i], clock_time - track_time, skew, bit_pos,
               int_bit_pos, avg_bit_sep_time, track_time);
#endif
         if (all_raw_bits_count + int_bit_pos >= 32) {
            all_raw_bits_count = mfm_save_raw_word(drive_params, 
               all_raw_bits_count, int_bit_pos, raw_word);
         } else {
            all_raw_bits_count += int_bit_pos;
         }
         // Shift based on number of bit times then put in the 1 from the
         // delta. If we had a delta greater than the size of raw word we
         // will lose the unprocessed bits in raw_word. This is unlikely
         // to matter since this is invalid MFM data so the disk had a long
         // drop out so many more bits are lost.
         if (int_bit_pos >= sizeof(raw_word)*8) {
            raw_word = 1;
         } else {
            raw_word = (raw_word << int_bit_pos) | 1;
         }
         tot_raw_bit_cntr += int_bit_pos;
         raw_bit_cntr += int_bit_pos;

         // Are we looking for a mark code?
         if ((state == MARK_ID)) {
            // These patterns are MFM encoded all zeros or all ones.
            // We are looking for zeros so we assume they are zeros.
            if (raw_word == 0x55555555 || raw_word == 0xaaaaaaaa) {
               sync_count++;
            } else {
               if (sync_count < MARK_NUM_ZEROS) {
                  sync_count = 0;
               }
            }
            // This is the 0x891 missing clock MFM sync pattern for 0xA1
            // with all the bits for an 0xa1 so the 16 bit value is 0x4489.
            // This sync is used to mark the header and data fields
            // We want to see enough zeros to ensure we don't get a false
            // match at the boundaries where data is overwritten
            if ((raw_word & 0xffff) == 0x4489 && sync_count >= MARK_NUM_ZEROS) {
               sync_count = 0;
               state = HEADER_SYNC;
               raw_bit_cntr = 0;
               decoded_word = 0;
               decoded_bit_cntr = 0;
            }
         // We need to wait for the one bit to resynchronize. We want to see
         // enough zero bits to try to avoid triggering on a bit error
         // Using 49 seems to be more reliable than looking for 0x09 for
         // a single one. Bit errors were causing false syncs.
         } else if (state == HEADER_SYNC || state == DATA_SYNC) {
            if (sync_count++ > 90 && (raw_word & 0xff) == 0x49) {
               sync_count = 0;
               raw_bit_cntr = 3;
               decoded_word = 0;
               decoded_bit_cntr = 0;
               if (state == HEADER_SYNC) {
                  state = PROCESS_HEADER;
                  mfm_mark_header_location(all_raw_bits_count, tot_raw_bit_cntr);
                  // Figure out the length of data we should look for
                  bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes +
                        drive_params->header_crc.length / 8;
                  bytes_needed = bytes_crc_len;
               } else {
                  state = PROCESS_DATA;
                  mfm_mark_data_location(all_raw_bits_count, tot_raw_bit_cntr);
                  // Figure out the length of data we should look for
                  bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes +
                        mfm_controller_info[drive_params->controller].data_trailer_bytes +

                        drive_params->sector_size +
                        drive_params->data_crc.length / 8;
                  bytes_needed = DATA_IGNORE_BYTES + bytes_crc_len;
                  if (bytes_needed >= sizeof(bytes)) {
                     printf("Too many bytes needed %d\n",bytes_needed);
                     exit(1);
                  }
               }
               byte_cntr = 0;
            }
         } else {
            int entry_state = state;
            // If we have enough bits to decode do so. Stop if state changes
            while (raw_bit_cntr >= 4 && entry_state == state) {
               // If we have more than 4 only process 4 this time
               raw_bit_cntr -= 4;
               tmp_raw_word = raw_word >> raw_bit_cntr;
               decoded_word =
                     (decoded_word << 2) | code_bits[tmp_raw_word & 0xf];
               decoded_bit_cntr += 2;

               // And if we have a bytes worth store it
               if (decoded_bit_cntr >= 8) {
                  // Do we have enough to further process?
                  if (byte_cntr < bytes_needed) {
                     bytes[byte_cntr++] = decoded_word;
                  } else {
                     mfm_mark_end_data(all_raw_bits_count, drive_params);
                     sector_status |= mfm_process_bytes(drive_params, bytes,
                           bytes_crc_len, &state, cyl, head, &sector_index,
                           seek_difference, sector_status_list);
                  }
                  decoded_bit_cntr = 0;
               }
            }
         }
      }
      // Finished what we had, any more?
      // If we didn't get too many last time sleep so delta reader can run.
      // Thread priorities might be better.
      if (num_deltas - last_deltas <= 2000) {
         usleep(500);
      }
      last_deltas = num_deltas;
      num_deltas = deltas_get_count(i);
   }
   // Force last partial word to be saved
   mfm_save_raw_word(drive_params, all_raw_bits_count, 32-all_raw_bits_count,
      raw_word);
   // If we didn't find anything to decode return header error
   if (sector_status == SECT_NO_STATUS) {
      sector_status = SECT_BAD_HEADER;
   }
   return sector_status;
}
