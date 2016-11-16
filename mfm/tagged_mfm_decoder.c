// This routine decodes "Western Digital" (WD) formated disks with two
// headers on each sector. 
//
// TODO Improve data separator and resync better after end of 
// write transition area. Also why avg_bit_sep_time is < 20 for all drives
// We probably should be able to do better than just the PLL since we can 
// look ahead.
//
// 11/14/16 DJG Added Telenex Autoscope. Same as Xerox but without tags
//    so naming problems again
// 11/02/16 DJG Write out tag/metadata also if extract file speccified
// 10/16/16 DJG Fixes for XEROX_6085.
// 10/07/16 DJG Add support for disks with tag header in addition to the
//    normal header. Xerox 6085
//   
// Copyright 2016 David Gesswein.
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "deltas_read.h"

// Side of data to skip after header or data area. 
#define HEADER_IGNORE_BYTES 10
// For data this is the write splice area where you get corrupted data that
// may look like a sector start byte.
#define DATA_IGNORE_BYTES 10

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
// The general format is first byte of header or data area is an 0xa1 with a
// missing clock bit. This is the first byte in the bytes array though finding and
// validating it are done outside this routine. The format names are arbitrarily
// assigned to the first controller found writing that format.
// The formats are
//   CONTROLLER_XEROX_6085
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder high
//      byte 3 cylinder low
//      byte 4 head low four bits, unknown if upper used
//      byte 5 sector
//      byte 6-7 ECC code
//   Tag
//      byte 0 0xa1
//      byte 1 0xfc
//      20 bytes of data
//      ECC code (2 byte)
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size 
//         (alt cyl first 2 bytes msb first, head third. Also not known
//          if correct for this format)
//      ECC code (2 byte)
//
//   CONTROLLER_TELENEX_AUTOSCOPE
//      Same as XEROX_6085 except no tag
//      
// state: Current state in the decoding
// bytes: bytes to process
// crc: The crc of the bytes
// exp_cyl, exp_head: Track we think we are on
// sector_index: A sequential sector counter that may not match the sector
//    numbers
// drive_params: Drive parameters
// seek_difference: Return the difference between expected cylinder and
//   cylinder in header
// sector_status_list: The status of each sector (read errors etc)
// ecc_span: The maximum ECC span to correct (0 for no correction)
//
// TODO: Reformat drive with the various sector size/bad block/spare track
//    options and verify headers decoded properly.
// Handle spared/alternate tracks for extracted data
SECTOR_DECODE_STATUS tagged_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span, 
      SECTOR_DECODE_STATUS init_status)
{
   static int sector_size;
   // Non zero if sector is a bad block, has alternate track assigned,
   // or is an alternate track
   static SECTOR_STATUS sector_status;

   if (*state == PROCESS_HEADER) {
      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status = init_status;
      sector_status.status |= SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      if (drive_params->controller == CONTROLLER_XEROX_6085 ||
            drive_params->controller == CONTROLLER_TELENEX_AUTOSCOPE) {
         sector_status.cyl = bytes[2]<< 8;
         sector_status.cyl |= bytes[3];

         // More is in here but what is not documented in manual
         sector_status.head = bytes[4] & 0xf;
         if ((bytes[4] & 0xf0) != 0) {
            msg(MSG_INFO, "byte 4 upper bits not zero: %02x on cyl %d head %d sector %d\n",
                bytes[4], sector_status.cyl, sector_status.head, sector_status.sector);
           msg(MSG_INFO, "May indicate bad block or alternate sector\n");
           // Rest of format matches CONTROLLER_OMTI_5510 so this byte may also
         }
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[5];
         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else {
         msg(MSG_FATAL,"Unknown controller type %d\n",drive_params->controller);
         exit(1);
      }

      mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
            seek_difference, &sector_status, drive_params);

      msg(MSG_DEBUG,
         "Got exp %d,%d cyl %d head %d sector %d,%d size %d\n",
            exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
            sector_status.sector, *sector_index, sector_size);
      if (drive_params->controller == CONTROLLER_XEROX_6085) {
         *state = MARK_DATA1;
      } else {
         *state = MARK_DATA;
      }
   } else if (*state == PROCESS_HEADER2) {
         if (bytes[1] != 0xfc) {
            msg(MSG_INFO, "Invalid tag id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      sector_status.ecc_span_corrected_data = ecc_span;
      *state = MARK_DATA;
      if (!(sector_status.status & SECT_BAD_HEADER)) {
         mfm_write_metadata(&bytes[2], drive_params, &sector_status);
      }
   } else { // Data
      // Value and where to look for header mark byte
      int id_byte_expected = 0xfb;
      int id_byte_index = 1;
      if (bytes[id_byte_index] != id_byte_expected && crc == 0) {
         msg(MSG_INFO,"Invalid data id byte %02x expected %02x on cyl %d head %d sector %d\n", 
               bytes[id_byte_index], id_byte_expected,
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
         int dheader_bytes = mfm_controller_info[drive_params->controller].data_header_bytes;
         // TODO: Make handling of correction data for extract cleaner
         if (mfm_write_sector(&bytes[dheader_bytes], drive_params, &sector_status,
               sector_status_list, &bytes[1], drive_params->sector_size +
               drive_params->data_crc.length / 8 + 1) == -1) {
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
SECTOR_DECODE_STATUS tagged_decode_track(DRIVE_PARAMS *drive_params, int cyl,
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
   float filter_state = 0;
   // Time in track for debugging
   int track_time = 0;
   // Counter for debugging
   int tot_raw_bit_cntr = 0;
   // Where we are in decoding a sector, Start looking for header ID mark
   STATE_TYPE state = MARK_ID;
   // Status of decoding returned
   SECTOR_DECODE_STATUS all_sector_status = SECT_NO_STATUS;
   // How many zeros we need to see before we will look for the 0xa1 byte.
   // When write turns on and off can cause codes that look like the 0xa1
   // so this avoids them. Some drives seem to have small number of
   // zeros after sector marked bad in header.
   // TODO: look at  v170.raw file and see if number of bits since last
   // header should be used to help separate good from false ID.
   // also ignoring known write splice location should help.
#define MARK_NUM_ZEROS 2
   int zero_count = 0;
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
   // Used for analyze to distinguish formats with and without extra
   // header
   SECTOR_DECODE_STATUS init_status = 0;


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
#if 0
         if (cyl == 819 && head == 0) {
         printf
         ("  delta %d %d clock %.2f int bit pos %d avg_bit %.2f time %d\n",
               deltas[i], i, clock_time,
               int_bit_pos, avg_bit_sep_time, track_time);
         }
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
         if ((state == MARK_ID || state == MARK_DATA || state == MARK_DATA1)) {
            // These patterns are MFM encoded all zeros or all ones.
            // We are looking for zeros so we assume they are zeros.
            if (raw_word == 0x55555555 || raw_word == 0xaaaaaaaa) {
               zero_count++;
            } else {
               if (zero_count < MARK_NUM_ZEROS) {
                  zero_count = 0;
               }
            }
            // This is the 0x891 missing clock MFM sync pattern for 0xA1
            // with all the bits for an 0xa1 so the 16 bit value is 0x4489.
            // This sync is used to mark the header and data fields
            // We want to see enough zeros to ensure we don't get a false
            // match at the boundaries where data is overwritten
#if 0
   if (cyl == 0 && head == 1 && tot_raw_bit_cntr > 128070 && tot_raw_bit_cntr < 128110)
      printf("%d %x\n", tot_raw_bit_cntr, raw_word);

            if ((raw_word & 0xffff) == 0x4489) {
               printf("cyl %d head %d sector index %d zero %d byte %d %x tot raw %d\n",
                     cyl, head, sector_index, zero_count, byte_cntr, raw_word, tot_raw_bit_cntr);
            }
            if ((raw_word & 0xffff) == 0x4489) {
   //printf("mark at %d zero %d\n", tot_raw_bit_cntr, zero_count);
}
#endif
            if (((raw_word & 0xffff) == 0x4489)
                  && zero_count >= MARK_NUM_ZEROS) {
               zero_count = 0;
               bytes[0] = 0xa1;
               byte_cntr = 1;
               if (state == MARK_ID) {
                  state = PROCESS_HEADER;
                  mfm_mark_header_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
                  // Figure out the length of data we should look for
                  bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes + 
                        drive_params->header_crc.length / 8;
                  bytes_needed = bytes_crc_len + HEADER_IGNORE_BYTES;
               } else if (state == MARK_DATA1) {
                  state = PROCESS_HEADER2;
                  // Figure out the length of data we should look for
                  bytes_crc_len = 2 + mfm_controller_info[drive_params->controller].metadata_bytes + 
                        drive_params->header_crc.length / 8;
                  bytes_needed = bytes_crc_len + HEADER_IGNORE_BYTES;
               } else {
                  state = PROCESS_DATA;
                  mfm_mark_data_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
                  // Figure out the length of data we should look for
                  bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes + 
                        mfm_controller_info[drive_params->controller].data_trailer_bytes + 
                        drive_params->sector_size +
                        drive_params->data_crc.length / 8;
                  bytes_needed = DATA_IGNORE_BYTES + bytes_crc_len;
                  if (bytes_needed >= sizeof(bytes)) {
                     msg(MSG_FATAL,"Too many bytes needed %d\n", bytes_needed);
                     exit(1);
                  }
               }
               // Resync decoding to the mark
               raw_bit_cntr = 0;
               decoded_word = 0;
               decoded_bit_cntr = 0;
            }
         } else {
//printf("Rawb %08x tot %d\n",raw_word, tot_raw_bit_cntr);
            int entry_state = state;
            // If we have enough bits to decode do so. Stop if state changes
            while (raw_bit_cntr >= 4 && entry_state == state) {
               // If we have more than 4 only process 4 this time
               raw_bit_cntr -= 4;
               tmp_raw_word = raw_word >> raw_bit_cntr;
#if 0
               if (!valid_code[tmp_raw_word & 0xf]) {
                  printf("invalid code %x at %d bit %d\n", tmp_raw_word, i,
                        tot_raw_bit_cntr);
               }
#endif
               decoded_word =
                     (decoded_word << 2) | code_bits[tmp_raw_word & 0xf];
               decoded_bit_cntr += 2;

               // And if we have a bytes worth store it
               if (decoded_bit_cntr >= 8) {
                  // Do we have enough to further process?
                  if (byte_cntr < bytes_needed) {
                     bytes[byte_cntr++] = decoded_word;
                     //Ugly hack to handle if we missed a header. TODO
                     //when switching to table driven format the table and
                     //track time will indicate what header should be found
                     if (byte_cntr == 2) {
                        if (bytes[1] == 0xfe) {
                           if (state != PROCESS_HEADER) {
                              // Flag error for analyze
                              init_status = SECT_ANALYZE_ERROR;
                              msg(MSG_INFO,"Found sector header out of order (state %d)\n", state);
                              bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes + 
                                  drive_params->header_crc.length / 8;
                              bytes_needed = bytes_crc_len + HEADER_IGNORE_BYTES;
                              state = PROCESS_HEADER;
                           }
                        } else if (bytes[1] == 0xfc) {
                           if (state != PROCESS_HEADER2) {
                              // Flag error for analyze
                              init_status = SECT_ANALYZE_ERROR;
                              msg(MSG_INFO,"Found tag header out of order (state %d)\n", state);
                              bytes_crc_len = 22 + 
                                 drive_params->header_crc.length / 8;
                              bytes_needed = bytes_crc_len + HEADER_IGNORE_BYTES;
                              state = PROCESS_HEADER2;
                           }
                        } else if (bytes[1] == 0xfb) {
                           if (state != PROCESS_DATA) {
                              msg(MSG_INFO,"Found data header out of order (state %d)\n", state);
                              // Flag error for analyze
                              init_status = SECT_ANALYZE_ERROR;
                              bytes_crc_len = mfm_controller_info[drive_params->controller].data_header_bytes + 
                                  mfm_controller_info[drive_params->controller].data_trailer_bytes + 
                                  drive_params->sector_size +
                                  drive_params->data_crc.length / 8;
                              bytes_needed = DATA_IGNORE_BYTES + bytes_crc_len;
                              state = PROCESS_DATA;
                           }
                        }
                     }
                  } else {
                     mfm_mark_end_data(all_raw_bits_count, drive_params);
                     all_sector_status |= mfm_process_bytes(drive_params, bytes,
                           bytes_crc_len, &state, cyl, head, &sector_index,
                           seek_difference, sector_status_list, init_status);
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
   if (all_sector_status == SECT_NO_STATUS) {
      all_sector_status = SECT_BAD_HEADER;
   }
   return all_sector_status;
}
