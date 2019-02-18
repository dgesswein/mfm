#define VCD 0
// This routine decodes Corvus formated disks. No documentation was found
// for track format. The bit/clock timing is 11 MHz.
// The format is Headers is preceded by a large number of zero followed
// by a one. The header is 3 bytes immediately followed by the sector data.
// Sector start is determined by time from index. There is a gap between
// sectors where it does not write anything when formatting the track so
// whatever existing data (possibly at the normal MFM 10 MHz will be seen.
//
// Copyright 2019 David Gesswein.
//
// 02/09/19 DJG Added CONTROLLER_SAGA_FOX
// 04/22/18 DJG Made code for setting bit rate match other routines
// 04/21/17 DJG Added parameter to mfm_check_header_values and added
//    determining --begin_time if needed
// 11/20/16 DJG Add Vector4 format and fixes for marking data for
//   fixing emulator data.
// 10/28/16 DJG Document extra header Cromemco drives have
// 10/07/16 DJG More fixes for Cromemco format. Change when looking for
//    header to avoid false sync. Suppress out of data when all sectors read.
// 10/01/16 DJG Added Cromemco format
// 05/21/16 DJG Parameter change to mfm_mark* routines
// 12/31/15 DJG Parameter change to mfm_mark* routines
// 11/01/15 DJG Comment fixes
//
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
//   CONTROLLER_CORVUS_H,
//   3 byte header + 2 byte CRC
//      byte 0 head upper 3 bits, sector number lower 5
//      byte 1 low byte of cylinder
//      byte 2 high byte of cylinder
//      Sector data for sector size
//      2 byte CRC code (polynomial 0x8005)
//      
//   CONTROLLER_CROMEMCO,
//   At about 65,000 ns from index the track has a header
//      byte 0 0x04
//      byte 1 0xaa
//      byte 2 low cylinder
//      byte 3 high cylinder
//      byte 4 head
//
//   At about 190,000 ns the one sector has a header
//   9 byte header + 7 byte trailer + 2 byte CRC
//      byte 0 0x04
//      byte 1 0x00
//      byte 2 0xaa
//      byte 3 0xaa
//      byte 4 0xaa
//      byte 5 0x00
//      byte 6 low cylinder
//      byte 7 high cylinder
//      byte 8 head
//      10240 bytes of data
//      0x00
//      0xaa
//      0xaa
//      0x00
//      2 byte CRC code (polynomial 0x8005, init value 0 with all
//         above in the CRC))
//   
//   CONTROLLER_VECTOR4
//      format 7100-6501_Disktest_Technical_Information_Oct82.pdf,
//         pdf/vectorGraphics on bitsavers
//      30 bytes of zeros (not considered part of header
//      byte 0 0xff
//      byte 1 upper 4 bits head, lower 4 bits upper part of cylinder
//      byte 2 lower 8 bits of cylinder
//      byte 3 sector
//      256 bytes of sector data
//      4 byte ecc
//      
//   CONTROLLER_VECTOR4_ST506
//      format 7100-6501_Disktest_Technical_Information_Oct82.pdf,
//         pdf/vectorGraphics on bitsavers
//      30 bytes of zeros (not considered part of header
//      byte 0 0xff
//      byte 1 head
//      byte 2 cylinder
//      byte 3 sector
//      256 bytes of sector data
//      4 byte ecc
//
//   CONTROLLER_SAGA_FOX
//      No information on format available
//      Sector header mark seems to be a bunch of zeros followed by four ones
//      All data is bit reversed. Information below is after bit reversing bytes
//
//      Header
//      byte 0 0xf0
//      byte 1 cylinder
//      byte 2 unkown drive was too small to know if upper cyl. Code assumes
//          it is upper cyl.
//      byte 3 head
//      byte 4 sector
//      byte 5 xor of cyl and head
//      byte 6 xor of bytes 0-5
//
//      Data
//      byte 0 0xf0
//      256 bytes of data
//      byte 257 unknown
//      byte 258 xor of bytes 0-257
//      
SECTOR_DECODE_STATUS corvus_process_data(STATE_TYPE *state, uint8_t bytes[],
         int total_bytes,
         uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
         DRIVE_PARAMS *drive_params, int *seek_difference,
         SECTOR_STATUS sector_status_list[], int ecc_span,
         SECTOR_DECODE_STATUS init_status)

{
   static int sector_size;
   static int bad_block;
   static SECTOR_STATUS sector_status;
   uint8_t cromemco_sync[] = {0x04, 0x00, 0xaa, 0xaa, 0xaa, 0x00};

   if (*state == PROCESS_HEADER) {
      *state = MARK_ID;

      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status |= init_status | SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      if (drive_params->controller == CONTROLLER_CORVUS_H) {
         sector_status.cyl = bytes[1] | (bytes[2] << 8);
         sector_status.head = bytes[0] >> 5 ;
         sector_status.sector = bytes[0] & 0x1f;
      } else if (drive_params->controller == CONTROLLER_CROMEMCO) {
         sector_status.cyl = bytes[6] | (bytes[7] << 8);
         sector_status.head = bytes[8] ;
         sector_status.sector = 0; // Only 1 sector
         if (memcmp(bytes, cromemco_sync, sizeof(cromemco_sync)) != 0) {
            msg(MSG_ERR, "Bad alignment bytes %x %x %x %x %x %x on cyl %d,%d head %d,%d\n",
               bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5],
                  exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head);
         }
      } else if (drive_params->controller == CONTROLLER_VECTOR4) {
         if (bytes[0] != 0xff) {
            msg(MSG_ERR, "Bad sync byte %x on cyl %d,%d head %d,%d\n",
               bytes[0], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head);
            sector_status.status |= SECT_BAD_HEADER;
         }
         sector_status.cyl = ((bytes[1] & 0xf) << 8)  | bytes[2];
         sector_status.head = bytes[1] >> 4;
         sector_status.sector = bytes[3];
      } else if (drive_params->controller == CONTROLLER_VECTOR4_ST506) {
         if (bytes[0] != 0xff) {
            msg(MSG_ERR, "Bad sync byte %x on cyl %d,%d head %d,%d\n",
               bytes[0], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head);
            sector_status.status |= SECT_BAD_HEADER;
         }
         sector_status.cyl = bytes[2];
         sector_status.head = bytes[1];
         sector_status.sector = bytes[3];
      } else if (drive_params->controller == CONTROLLER_SAGA_FOX) {
         if (bytes[0] != 0xf0) {
            msg(MSG_ERR, "Bad sync byte %x on cyl %d,%d head %d,%d\n",
               bytes[0], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head);
            sector_status.status |= SECT_BAD_HEADER;
         }
         sector_status.cyl = bytes[1] | (bytes[2] << 8);
         sector_status.head = bytes[3];
         sector_status.sector = bytes[4];
         *state = MARK_DATA;
      } else {
         msg(MSG_FATAL,"Unknown controller type %d\n",drive_params->controller);
         exit(1);
      }


      // Don't know how/if these are encoded in header
      sector_size = drive_params->sector_size;
      bad_block = 0;
      msg(MSG_DEBUG,
         "Got exp %d,%d cyl %d head %d sector %d size %d bad block %d\n",
            exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
            sector_status.sector, sector_size, bad_block);

      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
            seek_difference, &sector_status, drive_params, sector_status_list);

      sector_status.ecc_span_corrected_data = ecc_span;
      if (!mfm_controller_info[drive_params->controller].separate_data &&
            !(sector_status.status & SECT_BAD_HEADER)) {
         int dheader_bytes = mfm_controller_info[drive_params->controller].data_header_bytes;
         if (mfm_write_sector(&bytes[dheader_bytes], drive_params, &sector_status,
               sector_status_list, &bytes[0], total_bytes) == -1) {
            sector_status.status |= SECT_BAD_HEADER;
         }
      }
   } else { // PROCESS_DATA
      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      sector_status.ecc_span_corrected_data = ecc_span;
      // TODO: If bad sector number the stats such as count of spare/bad
      // sectors is not updated. We need to know the sector # to update
      // our statistics array. This happens with RQDX3
      if (!(sector_status.status & (SECT_BAD_HEADER | SECT_BAD_SECTOR_NUMBER))) {
         int dheader_bytes = mfm_controller_info[drive_params->controller].data_header_bytes;

         // Bytes[1] is because 0xa1 can't be updated from bytes since
         // won't get encoded as special sync pattern
         if (mfm_write_sector(&bytes[dheader_bytes], drive_params, &sector_status,
               sector_status_list, &bytes[1], total_bytes-1) == -1) {
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
SECTOR_DECODE_STATUS corvus_decode_track(DRIVE_PARAMS *drive_params, int cyl,
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
   // "VCO" frequency in delta counts
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
   // Status of decoding returned
   int sector_status = SECT_NO_STATUS;
   // How many zeros we need to see before we will look for the sync bit.
   // When write turns on and off can cause codes that cause false sync
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
   // Time to look for next header in 200 MHz clock ticks. This will start 
   // looking at data a little into the beggining 0 words
   int next_header_time;
   // First address mark time in ns 
   int first_addr_mark_ns = 0;
   // This reverses the bit ordering in a byte. The controller writes
   // the header data LSB first not the normal MSB first.
   static unsigned char rev_lookup[16] = {
      0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
      0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
   #define REV_BYTE(n)( (rev_lookup[n&0xf] << 4) | rev_lookup[(n & 0xf0)>>4])


   nominal_bit_sep_time = 200e6 /
         mfm_controller_info[drive_params->controller].clk_rate_hz;
   avg_bit_sep_time = nominal_bit_sep_time;

   if (drive_params->controller == CONTROLLER_CORVUS_H) {
      next_header_time = 71500;
   } else if (drive_params->controller == CONTROLLER_CROMEMCO) {
      // Zeros found earlier cause false syncs unless skipped. TODO:
      // May be better to check for sync data following and resync if
      // not correct for Coromemco.
      next_header_time = 32000;
   } else if (drive_params->controller == CONTROLLER_VECTOR4 ||
       drive_params->controller == CONTROLLER_VECTOR4_ST506) {
      next_header_time = 58000;
   } else if (drive_params->controller == CONTROLLER_SAGA_FOX) {
      next_header_time = 91000;
   } else {
      msg(MSG_ERR, "Unknown controller\n");
      exit(1);
   }
#if VCD
long long int bit_time = 0;
FILE *out;
char str[100];
sprintf(str,"trk%d-%d.vcd",cyl,head);
out = fopen(str,"w");
fprintf(out,"$timescale 1ps $end\n");
fprintf(out,"$var wire 1 ^ data $end\n");
fprintf(out,"$var wire 1 & sector $end\n");
#endif
   // Adjust time for when data capture started
   next_header_time -= drive_params->start_time_ns / CLOCKS_TO_NS;

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
         avg_bit_sep_time = nominal_bit_sep_time + filter(clock_time, &filter_state);
#if DEBUG
         //printf("track %d clock %f\n", track_time, clock_time);
         //if (cyl == 70 & head == 5 && track_time > next_header_time)
         if (cyl == 0 && head == 0)
         printf
         ("  delta %d %.2f int %d avg_bit %.2f time %d dec %08x raw %08x byte %d\n",
               deltas[i], clock_time,
               int_bit_pos, avg_bit_sep_time, track_time, decoded_word,
               raw_word, byte_cntr);
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
         if (state == MARK_ID || state == MARK_DATA) {
//printf("Raw %d %d %x\n",tot_raw_bit_cntr, sync_count, raw_word);
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
            if (sync_count >= MARK_NUM_ZEROS ||
               (drive_params->controller == CONTROLLER_SAGA_FOX && 
                  sync_count >= 20)) {
               sync_count = 0;
               if (state == MARK_ID) {
                  state = HEADER_SYNC;
               } else {
                  state = DATA_SYNC;
               }
               raw_bit_cntr = 0;
               decoded_word = 0;
               decoded_bit_cntr = 0;
            }
         // We need to wait for the one bit to resynchronize.
         } else if (state == HEADER_SYNC || state == DATA_SYNC) {
//printf("Raw %d %d %x\n",tot_raw_bit_cntr, sync_count, raw_word);
            int found = 0;
            // SAGA_FOX synced incorrectly with just 0x9
            if (drive_params->controller == CONTROLLER_SAGA_FOX) {
               found = (raw_word & 0xfff) == 0xaa9;
            } else {
               found = (raw_word & 0xf) == 0x9;
            }
            if (found) {
#if VCD
bit_time = track_time / 198e6 * 1e12;
fprintf(out,"#%lld\n1&\n", bit_time);
printf("Found header at %d %d %d\n",tot_raw_bit_cntr, track_time, 
   track_time + drive_params->start_time_ns / CLOCKS_TO_NS);
#endif
               if (first_addr_mark_ns == 0) {
                  first_addr_mark_ns = track_time * CLOCKS_TO_NS;
               }

               // Figure out the length of data we should look for
               bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes +
                        drive_params->sector_size + 
                        mfm_controller_info[drive_params->controller].data_trailer_bytes +
                        drive_params->header_crc.length / 8;
               // Time next header should start at
               if (drive_params->controller == CONTROLLER_CORVUS_H) {
                  next_header_time += 164900;
                  raw_bit_cntr = -2;
                  // May be better to time from current track time in
               } else if (drive_params->controller == CONTROLLER_VECTOR4 ||
                     drive_params->controller == CONTROLLER_VECTOR4_ST506) {
                  // case drive rotation speed varies
                  next_header_time = track_time + 96000;
                  raw_bit_cntr = 2;
               } else if (drive_params->controller == CONTROLLER_CROMEMCO) {
                  // We need the 0x04 that is also the sync we are using
                  // to start decoding so back up
                  raw_bit_cntr = 12;
               } else if (drive_params->controller == CONTROLLER_SAGA_FOX) {
                  // case drive rotation speed varies
                  if (state == HEADER_SYNC) {
                     next_header_time = track_time + 4000;
                     raw_bit_cntr = 10;
                     // Figure out the length of header we should look for
                     bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes +
                        drive_params->header_crc.length / 8;
                  } else {
                     next_header_time = track_time + 82000;
                     raw_bit_cntr = 10;
                     // Figure out the length of data we should look for
                     bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes +
                        drive_params->sector_size + 
                        mfm_controller_info[drive_params->controller].data_trailer_bytes +
                        drive_params->data_crc.length / 8;
                  }
               }
//printf("Next header at %d track time %d\n",next_header_time, track_time);
               decoded_word = 0;
               decoded_bit_cntr = 0;
               // In this format header is attached to data so both
               // will be processed in this state
               if (state == HEADER_SYNC) {
                  state = PROCESS_HEADER;
               } else {
                  state = PROCESS_DATA;
               }
               mfm_mark_header_location(all_raw_bits_count, raw_bit_cntr, 
                  tot_raw_bit_cntr);
               mfm_mark_data_location(all_raw_bits_count, raw_bit_cntr,
                  tot_raw_bit_cntr);
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
         } else { // Collect decoded bytes in sector header or data
//printf("Raw %d %d %x\n",raw_bit_cntr, sync_count, raw_word);
            int entry_state = state;
            // If we have enough bits to decode do so. Stop if state changes
            while (raw_bit_cntr >= 4 && entry_state == state) {
               // If we have more than 4 only process 4 this time
               raw_bit_cntr -= 4;
               tmp_raw_word = raw_word >> raw_bit_cntr;
               decoded_word =
                     (decoded_word << 2) | code_bits[tmp_raw_word & 0xf];
               decoded_bit_cntr += 2;
#if VCD
fprintf(out,"#%lld\n%d^\n#%lld\n%d^\n", bit_time,
   ((decoded_word >> 1) & 1) ^ 1, bit_time + 90909*2, (decoded_word & 1) ^ 1);
   bit_time += 90909*4;
#endif

               // And if we have a bytes worth store it
               if (decoded_bit_cntr >= 8) {
                  // Do we have enough to further process?
                  if (byte_cntr < bytes_needed) {
                     if (drive_params->controller == CONTROLLER_SAGA_FOX) {
                        bytes[byte_cntr++] = REV_BYTE(decoded_word);
                     } else {
                        bytes[byte_cntr++] = decoded_word;
                     }
//printf("Decoded %d  %d %08x\n",byte_cntr, tot_raw_bit_cntr, decoded_word);
                  } else {
#if VCD
fprintf(out,"#%lld\n0&\n", bit_time);
#endif
                     mfm_mark_end_data(all_raw_bits_count, drive_params);
                     sector_status |= mfm_process_bytes(drive_params, bytes,
                        bytes_crc_len, bytes_needed, &state, cyl, head, 
                        &sector_index, seek_difference, sector_status_list, 0);
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
   if ((state == PROCESS_HEADER || state == PROCESS_DATA) && sector_index <= drive_params->num_sectors) {
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
#if VCD
fclose(out);
#endif
   return sector_status;
}
