// This routine decodes NorthStar formated disks and other with similar format. 
// Track format information 
//   http://www.classiccmp.org/dunfield/miscpm/advtech.pdf (pg 3-47)
// The format is 187 0xff, 3 0x55, 40 0xff at start of track.
// Each sector starts with 67 0x00, 0x01, 9 byte header, 512 data bytes,
// 4 byte crc, 45 unspecified bytes
// The 3 0x55 is used to syncronize finding the sector locations. The
// controller uses fixed delays from where it finds that pattern to start
// looking for the sector headers.
// It appears the controller uses the time from index to the 0x55 pattern
// as an estimate of drive RPM and ajusts the time it looks for the
// sector based on it. This code does not implement that method.
//
// The timing doesn't match real captures with the above formatting. The
// data in mfm_decoder.h has been adjusted to match captured data. 
//
// TODO: Too much code is being duplicated adding new formats. 
//
// 09/03/21 DJG Added CONTROLLER_SUPERBRAIN
// 07/05/19 DJG Improved 3 bit head field handling
// 04/22/18 DJG Added support for non 10 MHz bit rate
// 04/21/17 DJG Added parameter to mfm_check_header_values and added
//    determining --begin_time if needed
// 05/21/16 DJG Parameter change to mfm_mark_*
// 12/31/15 DJG Parameter change to mfm_mark_*
// 12/24/15 DJG Comment cleanup
// 11/01/15 DJG Use new drive_params field and comment changes
//
// Copyright 2018 David Gesswein.
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
// Since processing a header with errors can overwrite other good sectors this 
// routine shouldn't be called if data has a CRC error.
//
// The format names are arbitrarily assigned to the first controller found
// writing that format.
// The format is
//   CONTROLLER_NORTHSTAR_ADVANTAGE,
//   7 byte header + 2 checksum bytes
//      byte 0 sector number, upper cyl in upper nibble
//      byte 1 cylinder
//      byte 2 head in low 4 bits.
//      byte 3-6 Unknown
//      byte 7 checksum of header
//      byte 8 complement of checksum of header
//   Data immediately follows header
//      512 bytes data
//      16 bit checksum of data
//      16 bit complement of checksum of data
//      
//   CONTROLLER_SUPERBRAIN
//    superbrain2.trans
//    Not Northstar but seemed to match this logic the best of the decoders.
//    SuperFive disk system. No documentation on format.
//    Sector start by time and mfm patttern 0xa89
//    4 byte header + 2 checksum bytes
//       byte 0 0xfe
//       byte 1 cylinder
//       byte 2 head
//       byte 3 sector
//       bytes 4-5 CRC
//    Data
//        byte 0 0xf8
//        256 bytes data
//        2 byte crc
SECTOR_DECODE_STATUS northstar_process_data(STATE_TYPE *state, uint8_t bytes[],
         int total_bytes,
         uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
         DRIVE_PARAMS *drive_params, int *seek_difference,
         SECTOR_STATUS sector_status_list[], int ecc_span,
         SECTOR_DECODE_STATUS init_status)
{
   static int sector_size;
   static int bad_block;
   static SECTOR_STATUS sector_status;

   if (*state == PROCESS_HEADER) {
      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status |= init_status | SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
	 sector_status.cyl = bytes[1] | (((int) bytes[0] & 0xf0) << 4);
	 sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[2] & 0xf);
	 sector_status.sector = bytes[0] & 0xf;
	 // Don't know how/if these are encoded in header
	 sector_size = drive_params->sector_size;
	 bad_block = 0;
	 msg(MSG_DEBUG,
	    "Got exp %d,%d cyl %d head %d sector %d size %d bad block %d\n",
	       exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
	       sector_status.sector, sector_size, bad_block);

	 mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
	       seek_difference, &sector_status, drive_params, sector_status_list);

	 *state = DATA_SYNC;
      } else if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
	 sector_status.cyl = bytes[1];
	 sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[2]);
	 sector_status.sector = bytes[3];
	 // Don't know how/if these are encoded in header
	 sector_size = drive_params->sector_size;
	 bad_block = 0;

         if (bytes[0] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[0], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }


	 msg(MSG_DEBUG,
	    "Got exp %d,%d cyl %d head %d sector %d size %d bad block %d\n",
	       exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
	       sector_status.sector, sector_size, bad_block);

	 mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
	       seek_difference, &sector_status, drive_params, sector_status_list);

	 *state = DATA_SYNC2;
      }
   } else if (*state == PROCESS_DATA) {
      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      sector_status.ecc_span_corrected_data = ecc_span;

      if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
         if (bytes[0] != 0xf8) {
            msg(MSG_INFO, "Invalid data id byte %02x on cyl %d head %d sector %d\n",
               bytes[0], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_DATA;
         }
      }

   
      if (!(sector_status.status & SECT_BAD_HEADER)) {
         if (mfm_write_sector(&bytes[0], drive_params, &sector_status,
               sector_status_list, &bytes[0], total_bytes) == -1) {
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
SECTOR_DECODE_STATUS northstar_decode_track(DRIVE_PARAMS *drive_params, int cyl,
      int head, uint16_t deltas[], int *seek_difference,
      SECTOR_STATUS sector_status_list[])
{
   // This is which MFM clock and data bits are valid codes. So far haven't
   // found a good way to use this.
   //int valid_code[16] = { 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 };

   // This converts the MFM clock and data bits into data bits.
   int code_bits[16] =    { 0, 1, 0, 0, 2, 3, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };
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
   float avg_bit_sep_time;     // 200 MHz clocks
   float nominal_bit_sep_time; // 200 MHz clocks
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
   //STATE_TYPE state = PROCESS_HEADER;
   // Status of decoding returned
   int sector_status = SECT_NO_STATUS;
   // How many zeros we need to see before we will look for the mark byte.
   // When write turns on and off can cause codes that look like the mark
   // so this avoids them.
   #define MARK_NUM_ZEROS 30
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
   // Time to look for next header. This should be in beginning of 0 words
   int next_header_time = 0;
   // First address mark time in ns 
   int first_addr_mark_ns = 0;

   if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
      next_header_time = 74000;
   } else if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
      next_header_time = 55000;
   } else {
      msg(MSG_FATAL,"northstart_mfm_decoder got unknwon controller %d\n", 
         drive_params->controller);
   }

   // Adjust time for when data capture started
   next_header_time -= drive_params->start_time_ns / CLOCKS_TO_NS;

   num_deltas = deltas_get_count(0);

   raw_word = 0;
   nominal_bit_sep_time = 200e6 /
       mfm_controller_info[drive_params->controller].clk_rate_hz;
   avg_bit_sep_time = nominal_bit_sep_time;
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
         avg_bit_sep_time = nominal_bit_sep_time + filter(clock_time, &filter_state);
#if DEBUG
         //printf("track %d clock %f\n", track_time, clock_time);
         //if (cyl == 70 & head == 5 && track_time > next_header_time)
         if (cyl == 0 && head == 0)
         printf
         ("  delta %d %.2f int %d avg_bit %.2f time %d dec %08x raw %08x\n",
               deltas[i], clock_time,
               int_bit_pos, avg_bit_sep_time, track_time, decoded_word,
               raw_word);
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

//printf("Raw %08x %d %d %d %d\n", raw_word, state, sync_count, track_time, next_header_time);
         // Are we looking for a mark code?
         if (state == MARK_ID) {
            // These patterns are MFM encoded all zeros or all ones.
            // We are looking for zeros so we assume they are zeros.
            if (track_time > next_header_time && 
                 (raw_word == 0x55555555 || raw_word == 0xaaaaaaaa)) {
               sync_count++;
            } else {
               if (sync_count < MARK_NUM_ZEROS) {
                  sync_count = 0;
               }
            }
            // If we found enough zeros start looking for a 1
            if (sync_count >= MARK_NUM_ZEROS) {
//printf("Mark %d cyl %d head %d sect %d\n",tot_raw_bit_cntr, cyl, head, sector_index);
               sync_count = 0;
               state = HEADER_SYNC;
               raw_bit_cntr = 0;
               decoded_word = 0;
               decoded_bit_cntr = 0;
            }
         // We need to wait for the one bit to resynchronize.
         } else if (state == HEADER_SYNC) {
            int found = 0;
            if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE && (raw_word & 0xf) == 0x9) {
               raw_bit_cntr = 0;
               found = 1;
            } else if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
               //Better preventing false syncs but misses some with data
               //errors
               //if ((raw_word & 0xfff) == 0xa89) {
               if ((raw_word & 0xf) == 0x9) {
                  sync_count = 0;
                  raw_bit_cntr = 2;
                  found = 1;
               }
            }
            if (found) {
//printf("Sync %d,%d cyl %d head %d sect %d\n",tot_raw_bit_cntr, (track_time * 5 + drive_params->start_time_ns)/101, cyl, head, sector_index);
               if (first_addr_mark_ns == 0) {
                  first_addr_mark_ns = track_time * CLOCKS_TO_NS;
               }
               // Time next header should start at
               decoded_word = 0;
               decoded_bit_cntr = 0;
               state = PROCESS_HEADER;
               mfm_mark_header_location(all_raw_bits_count, raw_bit_cntr,
                  tot_raw_bit_cntr);
               // Figure out the length of data we should look for
               bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes +
                        drive_params->header_crc.length / 8;
               bytes_needed = bytes_crc_len;

               if (bytes_needed >= sizeof(bytes)) {
                  printf("Too many bytes needed %d\n",bytes_needed);
                  exit(1);
               }
               byte_cntr = 0;
            }
         } else if (state == DATA_SYNC) {
            state = PROCESS_DATA;
//printf("Start data %d\n",tot_raw_bit_cntr);
            mfm_mark_data_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
            // Figure out the length of data we should look for
            bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes +
                mfm_controller_info[drive_params->controller].data_trailer_bytes +

                drive_params->sector_size +
                drive_params->data_crc.length / 8;
            bytes_needed = bytes_crc_len;
            // Must read enough extra bytes to ensure we send last 32
            // bit word to mfm_save_raw_word
            bytes_needed += 2;

            if (bytes_needed >= sizeof(bytes)) {
               printf("Too many bytes needed %d\n",bytes_needed);
               exit(1);
            }
            byte_cntr = 0;
         } else if (state == DATA_SYNC2) {
            int found = 0;
            if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
               if (raw_word == 0x55555555 || raw_word == 0xaaaaaaaa) {
                  sync_count++;
               }
               //if ((raw_word & 0xfff) == 0xa89 && sync_count > 80) {
               if ((raw_word & 0xf) == 0x9 && sync_count > 80) {
		  raw_bit_cntr = 2;
		  found = 1;
               }
            }
            if (found) {
               sync_count = 0;
	       decoded_word = 0;
	       decoded_bit_cntr = 0;
	       state = PROCESS_DATA;
   //printf("Start data %d\n",tot_raw_bit_cntr);
	       mfm_mark_data_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
	       // Figure out the length of data we should look for
	       bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes +
		   mfm_controller_info[drive_params->controller].data_trailer_bytes +

		   drive_params->sector_size +
		   drive_params->data_crc.length / 8;
	       bytes_needed = bytes_crc_len;
	       // Must read enough extra bytes to ensure we send last 32
	       // bit word to mfm_save_raw_word
	       bytes_needed += 2;

	       if (bytes_needed >= sizeof(bytes)) {
		  printf("Too many bytes needed %d\n",bytes_needed);
		  exit(1);
	       }
               byte_cntr = 0;
            }
 
         } else { // PROCESS_HEADER or PROCESS_DATA
            int entry_state = state;
            // If we have enough bits to decode do so. Stop if state changes
            while (raw_bit_cntr >= 4 && entry_state == state) {
               // If we have more than 4 only process 4 this time
               raw_bit_cntr -= 4;
               tmp_raw_word = raw_word >> raw_bit_cntr;
               decoded_word =
                     (decoded_word << 2) | code_bits[tmp_raw_word & 0xf];
               decoded_bit_cntr += 2;
//printf("Decoded %x %d %d\n",decoded_word, byte_cntr, tot_raw_bit_cntr);

               // And if we have a bytes worth store it
               if (decoded_bit_cntr >= 8) {
                  // Do we have enough to further process?
                  bytes[byte_cntr++] = decoded_word;
                  if (byte_cntr >= bytes_needed) {
                     mfm_mark_end_data(all_raw_bits_count, drive_params);
//printf("End data %d,%d state %d cyl %d head %d sect %d\n",tot_raw_bit_cntr, 166666 - (track_time  * 5 + drive_params->start_time_ns)/100, state, cyl, head, sector_index);
                     sector_status |= mfm_process_bytes(drive_params, bytes,
                        bytes_crc_len, bytes_needed, &state, cyl, head, 
                        &sector_index, seek_difference, sector_status_list, 0);
                     // Look after the fill bytes. 8 is byte to bits, 40 is
                     // 200 MHz clocks per data bit (5 MHz for data bit,
                     // 10 for clock and data bit)
                     if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
                        // 45 bytes plus extra to get past any junk from overwriting
                        next_header_time = track_time + 55 * 8 * 40; 
                     } else if (drive_params->controller == CONTROLLER_SUPERBRAIN) {
                        next_header_time = track_time + 5 * 8 * 40; 
                     }
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
   if (state == PROCESS_DATA) {
      float begin_time =
         ((bytes_needed - byte_cntr) * 16.0 *
             1e9/mfm_controller_info[drive_params->controller].clk_rate_hz
             + first_addr_mark_ns) / 2 + drive_params->start_time_ns;
      msg(MSG_ERR, "Ran out of data on sector index %d, try reading with --begin_time %.0f\n",
         sector_index, round(begin_time / 1000.0) * 1000.0);
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
