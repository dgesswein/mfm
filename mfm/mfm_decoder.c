// These are the general routines for supporting MFM decoders.
// Call mfm_decode_setup once before trying to decode a disk
// call mfm_init_sector_status_list before decoding each track
// call mfm_update_deltas before decoding each track
// call mfm_decode_track to decode the track or
//   call mfm_wait_all_deltas to just wait for all deltas to be received and
//   possibly written to a file
// call mfm_check_deltas to get number of deltas in the buffer
// call mfm_check_header_values to verify header parameters
// call mfm_write_sector to check sector information and possibly write it to
//   the extract file
// call mfm_decode_done when all tracks have been processed
//
// TODO: make it use sector number information and checking CRC at data length to write data
// for sectors with bad headers. See if resyncing PLL at write boundaries improves performance when
// data bits are shifted at write boundaries.
//
// 11/07/15 DJG Added Symbolics 3640 support
// 11/01/15 DJG Renamed formats and other comment changes
// 05/17/15 DJG Added formats MIGHTYFRAME, ADAPTEC, NEWBURYDATA, SYMBOLICS, and
//          partially implement format RUSSIAN. Also code cleanup amd check for
//          trumcation of emulation file.
// 01/04/15 DJG Added Corvus_H and NorthStar Advantage decoders.
// 11/09/14 DJG Changes for note option
// 10/06/14 DJG Added new CONTROLLER_MACBOTTOM format
// 09/10/14 DJG Added new CONTROLLER_OLIVETTI format
//
// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
//
// 09/06/14 DJG Made sector number printed for sectors with errors
//   use drive sector numbering (drives may use 0 or 1 for first sector.
//   Separated dumping read data from error messages
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
#include <limits.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "deltas_read.h"

#define ARRAYSIZE(x)  (sizeof(x) / sizeof(x[0]))

// Hold for sector status and cylinder and head it was for. We save the
// data so when the cylinder and head changes we can print the final status.
// The same track may be reread.
static SECTOR_STATUS last_sector_list[MAX_SECTORS];
static int last_cyl;
static int last_head;
// File to write decoded data to. -1 if shoudln't write to file.
static int decode_fd = -1;

static void update_emu_track_sector(SECTOR_STATUS *sector_status, int sect_rel0,
      uint8_t bytes[], int num_bytes);
void update_emu_track_words(DRIVE_PARAMS * drive_params,
      SECTOR_STATUS sector_status_list[], int write_track, int new_track,
      int cyl, int head);

// These are various PLL constants I was trying. For efficiency the
// filter routine is put in the decoders so it can inline and has the
// "best" PLL constants I found.
#if 0
int filter_type;

static inline float filter(float v, float *delay)
{
   float in, out;

   in = v + *delay;
#if 1
   out = in * 0.034446428576716f + *delay * -0.034124999994713f;
#else
   switch (filter_type) {
   case 0:
      out = in * 0.034446428576716f + *delay * -0.034124999994713f;
      break;
   case 1:
      out = in * 0.114928571449721f + *delay * -0.113642857121708f;     // 1, 2e-5
      break;
   case 2:
      out = in * 0.034928571449721f + *delay * -0.033642857121708f;     // .3, 2e-5
      break;
   case 3:
      out = in * 0.071142857227454f + *delay * -0.065999999915403f;     // .3, 1e-5
      break;
   case 4:
      o1t = in * 0.023114285722745f + *delay * -0.022599999991540f;     // .3, 10e-5, /10
      break;
   case 5:
      out = in * 0.068828571437031f + *delay * -0.068314285705826f;     // .3, 10e-5, /10
      break;
   }
#endif

   *delay = in;
   return out;
}
#endif
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


// Print any errors and ECC correction from the sector_status_list.
//
// sector_status_list: List of sector statuses
// num_sectors: Length of list
// first_sector_number: Number of first sector in sector header
// cyl, head: Track status if for
static void print_sector_list_status(SECTOR_STATUS *sector_status_list,
      int num_sectors, int first_sector_number, int cyl, int head) {
   int cntr;
   int ecc_corrections = 0;
   int hard_errors = 0;

   // Find if anything needs printing
   for (cntr = 0; cntr < num_sectors; cntr++) {
      if (UNRECOVERED_ERROR(sector_status_list[cntr].status)) {
         hard_errors = 1;
      }
      if (sector_status_list[cntr].status & SECT_ECC_RECOVERED) {
         ecc_corrections = 1;
      }
   }
   // And if so print it
   if (hard_errors) {
      msg(MSG_ERR_SUMMARY, "Bad sectors on cylinder %d head %d:",cyl,head);
      for (cntr = 0; cntr < num_sectors; cntr++) {
         if (sector_status_list[cntr].status & SECT_BAD_HEADER) {
            msg(MSG_ERR_SUMMARY, " %dH", cntr + first_sector_number);
         }
         if (sector_status_list[cntr].status & SECT_BAD_DATA) {
            msg(MSG_ERR_SUMMARY, " %d", cntr + first_sector_number);
         }
      }
      msg(MSG_ERR_SUMMARY,"\n");
   }
   if (ecc_corrections) {
      msg(MSG_ERR_SUMMARY, "ECC Corrections on cylinder %d head %d:",cyl,head);
      for (cntr = 0; cntr < num_sectors; cntr++) {
         if (sector_status_list[cntr].status & SECT_ECC_RECOVERED) {
            msg(MSG_ERR_SUMMARY, " %d(", cntr + first_sector_number);
            if (sector_status_list[cntr].ecc_span_corrected_header) {
               msg(MSG_ERR_SUMMARY, "%dH%s",
                     sector_status_list[cntr].ecc_span_corrected_header,
                     sector_status_list[cntr].ecc_span_corrected_data != 0 ? "," : "");
            }
            if (sector_status_list[cntr].ecc_span_corrected_data) {
               msg(MSG_ERR_SUMMARY, "%d", sector_status_list[cntr].ecc_span_corrected_data);
            }
            msg(MSG_ERR_SUMMARY,")");
         }
      }
      msg(MSG_ERR_SUMMARY, "\n");
   }
}


// Update statistics for the read so we can print summary at the end.
// Since we may be called multiple times for the same track if errors are
// retried only update the statistics and print errors when the track
// changes.
//
// drive_params: Parameters for drive. Stats updated.
// cyl, head: cylinder and head from header
// sector_status_list: status of sectors
static void update_stats(DRIVE_PARAMS *drive_params, int cyl, int head,
      SECTOR_STATUS sector_status_list[])
{
   STATS *stats = &drive_params->stats;
   int i;
   int error_found = 0;

   // If track changed and list has been set (last_cyl != -1) then process
   if (last_cyl != -1 && (cyl != last_cyl || head != last_head)) {
      update_emu_track_words(drive_params, sector_status_list, 1, 1, 
          last_cyl, last_head);
      for (i = 0; i < drive_params->num_sectors; i++) {
         if (last_sector_list[i].status & SECT_ECC_RECOVERED) {
            stats->num_ecc_recovered++;
            error_found = 1;
         }
         if (last_sector_list[i].status & SECT_BAD_DATA) {
            stats->num_bad_data++;
            error_found = 1;
         } else if (last_sector_list[i].status & SECT_BAD_HEADER) {
            stats->num_bad_header++;
            error_found = 1;
         } else {
            stats->num_good_sectors++;
         }
      }
      if (error_found) {
         print_sector_list_status(last_sector_list, drive_params->num_sectors,
            drive_params->first_sector_number, last_cyl, last_head);
      }
   } else {
      update_emu_track_words(drive_params, sector_status_list, 0, last_cyl == -1, cyl, head);
   }
   // Save the sector information so we can use it when track changes
   if (sector_status_list != NULL) {
      memcpy(last_sector_list, sector_status_list, sizeof(last_sector_list));
   }
   last_cyl = cyl;
   last_head = head;
}

// Sets the sector status list to bad header. This is the default error.
// If we don't find the header when trying to read so can't update the list
// we will then have bad header status
//
// sector_status_list: status of sectors
// num_sectors: number of sectors in list
void mfm_init_sector_status_list(SECTOR_STATUS sector_status_list[], 
   int num_sectors)
{
   int i;

   memset(sector_status_list, 0, sizeof(*sector_status_list) * num_sectors);
   for (i = 0; i < num_sectors; i++) {
      sector_status_list[i].status = SECT_BAD_HEADER;
   }
}

// Decode a track's worth of deltas to clock and data. Used for
// generating emulation file from unknown format disk data
//
// drive_params: Drive parameters
// cyl,head: Physical Track data from
// deltas: MFM delta time transition data to analyze
// seek_difference: Return of difference between expected cyl and header
// sector_status_list: Return of status of decoded sector
// return: Or together of the status of each sector decoded
static SECTOR_DECODE_STATUS mfm_decode_track_deltas(DRIVE_PARAMS *drive_params,
      int cyl, int head, uint16_t deltas[], int *seek_difference,
      SECTOR_STATUS sector_status_list[])
{
   // This is the raw MFM data decoded with above
   unsigned int raw_word = 0;
   // Counter to know when to decode the next two bits.
   int raw_bit_cntr = 0;
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
   // Number of deltas available so far to process
   int num_deltas;
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

         if (drive_params->emulation_filename != NULL &&
               all_raw_bits_count + int_bit_pos >= 32) {
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
         raw_bit_cntr += int_bit_pos;
      }
      num_deltas = deltas_get_count(i);
   }
   return SECT_NO_STATUS;
}

// This routine calls the proper decoder for the drive format
// See called routines for parameters and return value
SECTOR_DECODE_STATUS mfm_decode_track(DRIVE_PARAMS * drive_params, int cyl, int head,
      uint16_t deltas[], int *seek_difference,
      SECTOR_STATUS sector_status_list[])
{
   int rc;
   int i;

   for (i = 0; i < drive_params->num_sectors; i++) {
      sector_status_list[i].last_status = SECT_BAD_HEADER;
   }
   // Change in mfm_process_bytes if this if is changed
   if (drive_params->controller == CONTROLLER_WD_1006 ||
         drive_params->controller == CONTROLLER_OMTI_5510 ||
         drive_params->controller == CONTROLLER_DEC_RQDX3 ||
         drive_params->controller == CONTROLLER_OLIVETTI ||
         drive_params->controller == CONTROLLER_MACBOTTOM ||
         drive_params->controller == CONTROLLER_MIGHTYFRAME ||
         drive_params->controller == CONTROLLER_ADAPTEC ||
         drive_params->controller == CONTROLLER_NEWBURYDATA ||
         drive_params->controller == CONTROLLER_ELEKTRONIKA_85 ||
         drive_params->controller == CONTROLLER_SYMBOLICS_3620 ||
         drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
      rc = wd_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_XEBEC_104786)  {
      rc = xebec_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_CORVUS_H)  {
      rc = corvus_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE)  {
      rc = northstar_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else {
      rc = mfm_decode_track_deltas(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   }
   update_stats(drive_params, cyl, head, sector_status_list);
   return rc;
}

// This makes the floating point faster with minor simplifications to the floating point processing
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0344k/Chdiihcd.html
#ifdef __arm__
static void enable_runfast()
{
   static const unsigned int x = 0x04086060;
   static const unsigned int y = 0x03000000;
   int r;
   asm volatile ("fmrx %0, fpscr         \n\t"  //r0 = FPSCR
         "and  %0, %0, %1        \n\t"  //r0 = r0 & 0x04086060
         "orr  %0, %0, %2        \n\t"  //r0 = r0 | 0x03000000
         "fmxr fpscr, %0         \n\t"  //FPSCR = r0
         :"=r" (r)
          :"r"(x), "r"(y)
   );
}
#endif

// Setup to start decoding. Pass null pointer for filename if decoded data
// shouldn't be written to a file.
//
// drive_params: Parameters for drive
// write_files: 1 if output files shouldn't be written
void mfm_decode_setup(DRIVE_PARAMS *drive_params, int write_files)
{
   STATS *stats = &drive_params->stats;


   // set to value indicating not yet set.
   last_head = -1;
   last_cyl = -1;

   if (write_files && drive_params->emulation_filename != NULL &&
         drive_params->emulation_output == 1) {
         // Assume 3600 RPM, 60 RPS. Make round number of words
      drive_params->emu_track_data_bytes = ceil(1/60.0 * 
         mfm_controller_info[drive_params->controller].clk_rate_hz / 8 / 4)*4;
      drive_params->emu_fd = emu_file_write_header(drive_params->emulation_filename,
        drive_params->num_cyl, drive_params->num_head, 
        drive_params->cmdline, drive_params->note,
        mfm_controller_info[drive_params->controller].clk_rate_hz,
        drive_params->start_time_ns, drive_params->emu_track_data_bytes);
   }

   if (!write_files || drive_params->extract_filename == NULL) {
      decode_fd = -1;
   } else {
      decode_fd = open(drive_params->extract_filename, O_WRONLY | O_CREAT |
            O_TRUNC, 0664);
      if (decode_fd < 0) {
         perror("Unable to create output file");
         exit(1);
      }
   }
   memset(stats, 0, sizeof(*stats));
   stats->min_sect = INT_MAX;
   stats->min_head = INT_MAX;
   stats->min_cyl = INT_MAX;

#if __arm__
   // Speed up floating point on beaglebone
   enable_runfast();
#endif
}

// This cleans up at end of processing data and prints summary information.
// Also checks that data agrees with the drive parameters.
//
// drive_params: Parameters for drive
void mfm_decode_done(DRIVE_PARAMS * drive_params)
{
   STATS *stats = &drive_params->stats;

   // Process last track sector list
   update_stats(drive_params, -1, -1, NULL);
   if (decode_fd >= 0) {
      close(decode_fd);
   }

   if (stats->min_cyl != INT_MAX) {
      msg(MSG_STATS,
            "Found cyl %d to %d, head %d to %d, sector %d to %d\n",
            stats->min_cyl, stats->max_cyl, stats->min_head, stats->max_head,
            stats->min_sect, stats->max_sect);
      if (stats->max_cyl - stats->min_cyl + 1 != drive_params->num_cyl) {
         msg(MSG_ERR_SUMMARY, "Expected cyls %d doesn't match cyls found %d\n",
               drive_params->num_cyl, stats->max_cyl - stats->min_cyl + 1);
      }
      if (stats->max_head - stats->min_head + 1 != drive_params->num_head) {
         msg(MSG_ERR_SUMMARY, "Expected heads %d doesn't match heads found %d\n",
               drive_params->num_head, stats->max_head - stats->min_head + 1);
      }
      if (stats->max_sect - stats->min_sect + 1 != drive_params->num_sectors) {
         msg(MSG_ERR_SUMMARY, "Expected sectors %d doesn't match sectors found %d\n",
               drive_params->num_sectors, stats->max_sect - stats->min_sect + 1);
      }
      if (stats->min_sect != drive_params->first_sector_number) {
         msg(MSG_ERR_SUMMARY, "Expected first sector number %d doesn't match first sector found %d\n",
               drive_params->first_sector_number, stats->min_sect);

      }

      msg(MSG_STATS,
            "Expected %d sectors got %d good sectors, %d bad header, %d bad data\n",
            drive_params->num_cyl * drive_params->num_head *
            drive_params->num_sectors, stats->num_good_sectors,
            stats->num_bad_header, stats->num_bad_data);
      msg(MSG_STATS,
            "%d sectors corrected with ECC. Max bits in burst corrected %d\n",
            stats->num_ecc_recovered, stats->max_ecc_span);
   }
   emu_file_close(drive_params->emu_fd, drive_params->emulation_output);
}

// This checks that the sector header values are reasonable and match the
// track and head we thought we were reading. If the cylinder doesn't match
// we return the difference between the actual and expected cylinder so
// the caller can try seeking again if needed.
//
// exp_cyl, exp_head: Track data was from
// sector_index: Counter for sectors starting at 0. With errors may not match
//   sector number
//  sector_size: Sector size from header
// seek_difference: Return of cylinder difference
// sector_status: Status of this sector
// drive_params: Parameters for drive
void mfm_check_header_values(int exp_cyl, int exp_head,
      int *sector_index, int sector_size, int *seek_difference,
      SECTOR_STATUS *sector_status, DRIVE_PARAMS *drive_params) {

   if (drive_params->ignore_header_mismatch) {
      sector_status->logical_sector = *sector_index;
      (*sector_index)++;
      return;
   }

   // WD 1003 controllers wrote 3 bit head code so head 8 is written as 0.
   // If requested and head seems correct fix read head value.
   if (drive_params->head_3bit && sector_status->head == (exp_head & 0x7)) {
      sector_status->head = exp_head;
   }
   if (sector_status->head != exp_head || sector_status->cyl != exp_cyl) {
      msg(MSG_ERR,"Mismatch cyl %d,%d head %d,%d index %d\n",
            sector_status->cyl, exp_cyl, sector_status->head, exp_head,
            *sector_index);
      sector_status->status |= SECT_BAD_HEADER;
      // Possibly a seek error, mark it
      if (sector_status->cyl != exp_cyl) {
         sector_status->status |= SECT_WRONG_CYL;
         if (seek_difference != NULL) {
            *seek_difference = exp_cyl - sector_status->cyl;
         }
      }
   }
   // If we have expected sector ordering information check the sector numbers
   // TODO: make this handle more complex sector numbering where they vary
   // between tracks
   if (drive_params->sector_numbers != NULL) {
      int orig_sector_index = *sector_index;
      for (; *sector_index < drive_params->num_sectors; (*sector_index)++) {
         if (sector_status->sector == drive_params->sector_numbers[*sector_index]) {
            break;
         }
      }
      if (*sector_index > orig_sector_index+1) {
         msg(MSG_ERR, "Cyl %d head %d Missed sector between %d(%d) and %d(%d)\n",
               sector_status->cyl, sector_status->head,
               drive_params->sector_numbers[orig_sector_index], orig_sector_index,
               drive_params->sector_numbers[*sector_index], *sector_index);
      }
      if (*sector_index >= drive_params->num_sectors) {
         msg(MSG_ERR_SERIOUS,"Cyl %d head %d Sector %d not found in expected sector list after %d(%d)\n",
               sector_status->cyl, sector_status->head,
               sector_status->sector,
               drive_params->sector_numbers[orig_sector_index],
               orig_sector_index);
         sector_status->status |= SECT_BAD_HEADER;
         *sector_index = orig_sector_index;
      }
      sector_status->logical_sector = *sector_index;
   } else {
      sector_status->logical_sector = *sector_index;
      (*sector_index)++;
   }
   if (sector_size != drive_params->sector_size) {
      msg(MSG_ERR,"Expected sector size %d header says %d cyl %d head %d sector %d\n",
            drive_params->sector_size, sector_size, sector_status->cyl,
            sector_status->head, sector_status->sector);
   }
}

// Write the sector data to file. We only write the best data so if the
// caller retries read with error we won't overwrite good data if this
// read has an error for this sector but the previous didn't.
// Bytes is data to write.
// drive_params: specifies the length and other information.
// sector_status: is the status of the sector writing
// sector_status_list: is the status of the data that would be written to the
//    file. Status is still updated if decode_fd is -1 to prevent writing.
// all_bytes: Includes data header bytes, used for emulator file writing
// all_bytes_len: Length of all_bytes
// return: -1 if error found, 0 if OK.
int mfm_write_sector(uint8_t bytes[], DRIVE_PARAMS * drive_params,
      SECTOR_STATUS *sector_status, SECTOR_STATUS sector_status_list[],
      uint8_t all_bytes[], int all_bytes_len)
{
   int rc;
   STATS *stats = &drive_params->stats;
   int update;

   // Some disks number sectors starting from 1. We need them starting
   // from 0.
   int sect_rel0 = sector_status->sector - drive_params->first_sector_number;

   // Collect statistics
   stats->max_sect = MAX(sector_status->sector, stats->max_sect);
   stats->min_sect = MIN(sector_status->sector, stats->min_sect);
   stats->max_head = MAX(sector_status->head, stats->max_head);
   stats->min_head = MIN(sector_status->head, stats->min_head);
   stats->max_cyl = MAX(sector_status->cyl, stats->max_cyl);
   stats->min_cyl = MIN(sector_status->cyl, stats->min_cyl);

   // Check for sector and head in range to prevent bad writes.
   // We don't check cyl against max since if it exceeds it things
   // will still work properly.
   if (sect_rel0 >= drive_params->num_sectors || sect_rel0 < 0) {
      msg(MSG_ERR_SERIOUS, "Logical sector %d out of range 0-%d sector %d cyl %d head %d\n",
            sect_rel0, drive_params->num_sectors-1, sector_status->sector,
            sector_status->cyl,sector_status->head);
      return -1;
   }
   if (sector_status->head > drive_params->num_head) {
      msg(MSG_ERR_SERIOUS,"Head out of range %d max %d cyl %d sector %d\n", 
            sector_status->head, drive_params->num_head, 
            sector_status->cyl, sector_status->sector);
      return -1;
   }

   sector_status_list[sect_rel0].last_status = sector_status->status;
   // If not written then write data. Otherwise only write if likely to be
   // better than sector previously written. Better is if we didn't get a CRC
   // error, or the ECC correction span is less than the last one.
   // We assume the header data is correct so we don't check if the header
   // ECC correction is better. If the header wasn't right we wrote the
   // data to the wrong spot in the file.
   update = 0;
   // If the previous header was bad update
   if (sector_status_list[sect_rel0].status & SECT_BAD_HEADER) {
      update = 1;
   }
   // If the previous data had an error then check if we should update
   if ((sector_status_list[sect_rel0].status & SECT_BAD_DATA) ||
          sector_status_list[sect_rel0].ecc_span_corrected_data > 0) {
      // If current doesn't have error and ECC correction if any at least
      // as good as previous
      if ( !(sector_status->status & SECT_BAD_DATA) &&
             (sector_status->ecc_span_corrected_data == 0 ||
             sector_status->ecc_span_corrected_data <
             sector_status_list[sect_rel0].ecc_span_corrected_data) ) {
         update = 1;
      }
   }
   if (update) {
      update_emu_track_sector(sector_status, sect_rel0, all_bytes, all_bytes_len);
      if (decode_fd >= 0) {
         if (lseek(decode_fd,
               (sect_rel0) * drive_params->sector_size +
               sector_status->head * (drive_params->sector_size *
                     drive_params->num_sectors) +
                     (off_t) sector_status->cyl * (drive_params->sector_size *
                           drive_params->num_sectors *
                           drive_params->num_head), SEEK_SET) < 0) {
            msg(MSG_FATAL, "Seek failed decoded data: %s\n", strerror(errno));
            exit(1);
         };
         if ((rc = write(decode_fd, bytes, drive_params->sector_size)) !=
               drive_params->sector_size) {
            msg(MSG_FATAL, "Write failed, rc %d: %s", rc, strerror(errno));
            exit(1);
         }
      }
      sector_status_list[sect_rel0] = *sector_status;
   }
   sector_status_list[sect_rel0].last_status = sector_status->status;

   return 0;
}

// This prints the header or data bytes for decoding new formats
void mfm_dump_bytes(uint8_t bytes[], int len, int cyl, int head,
      int sector_index, int msg_level)
{
   int i;

   msg(msg_level, "%4d %2d %2d:", cyl, head, sector_index);
   for (i = 0; i < len; i++) {
      //      MSG(MSG_INFO, " %02x",bytes[i]);
      msg(msg_level, "0x%02x,",bytes[i]);
      if (i % 16 == 15) {
         msg(msg_level, "\n");
      }
   }
   msg(msg_level, "\n");
}


// After we have found a valid header/data mark this routine is
// used to process the bytes. It checks the CRC and does ECC if needed then
// calls wd_process_data to finish the processing.
//
// drive_params: Drive parameters
// bytes: bytes to process
// bytes_crc_len: Length of bytes including CRC
// state: Where we are in the decoding process
// cyl,head: Physical Track data from
// sector_index: Sequential sector counter
// seek_difference: Return of difference between expected cyl and header
// sector_status_list: Return of status of decoded sector
// return: Status of sector decoded
SECTOR_DECODE_STATUS mfm_process_bytes(DRIVE_PARAMS *drive_params, 
   uint8_t bytes[], int bytes_crc_len, STATE_TYPE *state, int cyl, int head,
   int *sector_index, int *seek_difference, 
   SECTOR_STATUS sector_status_list[]) {

   uint64_t crc;
   // CRC to use to process these bytes
   CRC_INFO crc_info;
   char *name;
   // Length of ECC correction. 0 is no correction.
   int ecc_span = 0;
   SECTOR_DECODE_STATUS status = SECT_NO_STATUS;
   // Start byte for CRC decoding
   int start;

   if (*state == PROCESS_HEADER) {
      crc_info = drive_params->header_crc;
      if (msg_get_err_mask() & MSG_DEBUG_DATA) {
         mfm_dump_bytes(bytes, bytes_crc_len, cyl, head, *sector_index,
               MSG_DEBUG_DATA);
      }
      name = "header";
   } else {
#if 0
      static int dump_fd = 0;
      if  (dump_fd == 0) {
         dump_fd = open("dumpfile",O_WRONLY | O_CREAT | O_TRUNC, 0666);
      }
      write(dump_fd, &bytes[3], bytes_crc_len - 3);
#endif
      crc_info = drive_params->data_crc;
      if (msg_get_err_mask() & MSG_DEBUG_DATA) {
         mfm_dump_bytes(bytes, bytes_crc_len, cyl, head, *sector_index,
               MSG_DEBUG_DATA);
      }
      name = "data";
   }
   if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
      crc = checksum64(bytes, bytes_crc_len-crc_info.length/8, &crc_info);
      if (crc_info.length == 16) {
         crc = crc & 0xff;
         if (crc == bytes[bytes_crc_len-2] && 
               crc == (bytes[bytes_crc_len-1] ^ 0xff)) {
            crc = 0;
         } else {
            msg(MSG_DEBUG, "sum %02llx: %02x, %02x\n", crc, 
               bytes[bytes_crc_len-2], bytes[bytes_crc_len-1]);
            crc = 1; // Non zero indicates error
         }
      } else if (crc_info.length == 32) {
         crc = crc & 0xffff;
         uint16_t chksum1, chksum2;
         chksum1 = ((uint16_t) bytes[bytes_crc_len-4] << 8) | bytes[bytes_crc_len-3];
         chksum2 = ((uint16_t) bytes[bytes_crc_len-2] << 8) | bytes[bytes_crc_len-1];
         if (crc == chksum1 && crc == (chksum2 ^ 0xffff)) {
            crc = 0;
         } else {
            msg(MSG_DEBUG, "sum %04llx: %04x, %04x\n", crc, chksum1, chksum2);
            crc = 1; // Non zero indicates error
         }
      } else {
         msg(MSG_FATAL, "Invalid CRC/checksum length %d\n",crc_info.length);
         exit(1);
      }
   } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
      if (*state == PROCESS_HEADER) {
         crc = 0;
      } else {
         start = mfm_controller_info[drive_params->controller].data_crc_ignore;
         crc = crc64(&bytes[start], bytes_crc_len-start, &crc_info);
      }
   } else {
      if (*state == PROCESS_HEADER) {
         start = mfm_controller_info[drive_params->controller].header_crc_ignore;
      } else {
         start = mfm_controller_info[drive_params->controller].data_crc_ignore;
      }
      crc = crc64(&bytes[start], bytes_crc_len-start, &crc_info);
   }
   // Zero CRC is no error
   if (crc == 0) {
      if (*state == PROCESS_HEADER) {
         status |= SECT_ZERO_HEADER_CRC;
      } else {
         status |= SECT_ZERO_DATA_CRC;
      }
   }

   if (crc != 0) {
      msg(MSG_DEBUG,"Bad CRC %s cyl %d head %d sector index %d\n",
            name, cyl, head, *sector_index);
      // If ECC correction enabled then perform correction up to length
      // specified
      if (crc_info.ecc_max_span != 0) {
         ecc_span = ecc64(bytes, bytes_crc_len, crc, &crc_info);
         if (ecc_span != 0) {
            drive_params->stats.max_ecc_span = MAX(ecc_span,
                  drive_params->stats.max_ecc_span);
            crc = 0; // No longer have CRC error
         }
      }
   }

   // If no error process. Only process with errors if data. Without
   // valid header we don't know what sector we are decoding.
   if (*state != PROCESS_HEADER || crc == 0 || ecc_span != 0) {

      // If this is changed change in mfm_decode_track also
      if (drive_params->controller == CONTROLLER_WD_1006 ||
            drive_params->controller == CONTROLLER_OMTI_5510 ||
            drive_params->controller == CONTROLLER_DEC_RQDX3 ||
            drive_params->controller == CONTROLLER_OLIVETTI ||
            drive_params->controller == CONTROLLER_MACBOTTOM ||
            drive_params->controller == CONTROLLER_MIGHTYFRAME ||
            drive_params->controller == CONTROLLER_ADAPTEC ||
            drive_params->controller == CONTROLLER_NEWBURYDATA ||
            drive_params->controller == CONTROLLER_ELEKTRONIKA_85 ||
            drive_params->controller == CONTROLLER_SYMBOLICS_3620 ||
            drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         status |= wd_process_data(state, bytes, crc, cyl, head, sector_index,
               drive_params, seek_difference, sector_status_list, ecc_span);
      } else if (drive_params->controller == CONTROLLER_XEBEC_104786) {
         status |= xebec_process_data(state, bytes, crc, cyl, head,
               sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span);
      } else if (drive_params->controller == CONTROLLER_CORVUS_H) {
         status |= corvus_process_data(state, bytes, crc, cyl, head,
               sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span);
      } else if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
         status |= northstar_process_data(state, bytes, crc, cyl, head,
               sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span);
      } else if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
         status |= northstar_process_data(state, bytes, crc, cyl, head,
               sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span);
      } else {
         msg(MSG_FATAL, "Unexpected controller %d\n",
               drive_params->controller);
         exit(1);
      }
    } else {
      status |= SECT_BAD_HEADER;
      // Search for header in case we are out of sync. If we found
      // data next we can't process it anyway.
      *state = MARK_ID;
   }
   return status;
}

// TODO: Add ability to convert sector data back to track data to replace this.
// This requires determining the gap formats which we don't currently know.

// This attempts to piece a good track together out of multiple reads. It does
// not always succeed even though we have successfully recovered the sector
// data. Various issues with the data can cause the reassembled data to not
// make an error free track.


// Best_track is the best track read as one read. Best_fixed_track is
// the best track by putting together multiple reads
uint32_t best_track_words[MAX_TRACK_WORDS];
int best_track_weight;
int best_track_num_words;
uint32_t best_fixed_track_words[MAX_TRACK_WORDS];
int best_fixed_track_weight;
int best_fixed_track_num_words;
uint32_t current_track_words[MAX_TRACK_WORDS];
int current_track_words_ndx;
int last_sector_start_word;
int header_track_word_ndx;
int data_bit;
int data_word_ndx;

// Note that last call where data will be written had data for next track.
void update_emu_track_words(DRIVE_PARAMS * drive_params,
      SECTOR_STATUS sector_status_list[], int write_track, int new_track,
      int cyl, int head)
{
   int i;
   int last_weight = 0;
   int best_weight = 0;

   if (drive_params->emulation_filename == NULL || !drive_params->emulation_output) {
      return;
   }
   // Determine error value for best track and best_fixed track so we can
   // determine which to use.
   if (sector_status_list != NULL) {
      for (i = 0; i < drive_params->num_sectors; i++) {
         // Last status is the one for the last track read
         if (sector_status_list[i].last_status & SECT_BAD_DATA) {
            last_weight += 1;
         } else if (!(sector_status_list[i].last_status & SECT_BAD_HEADER)) {
            if (sector_status_list[i].last_status & SECT_ECC_RECOVERED) {
               last_weight += 9;
            } else {
               last_weight += 10;
            }
         }
         // This is the best status for all the reads of the track
         if (sector_status_list[i].status & SECT_BAD_DATA) {
            best_weight += 1;
         } else if (!(sector_status_list[i].status & SECT_BAD_HEADER)) {
            if (sector_status_list[i].status & SECT_ECC_RECOVERED) {
               best_weight += 9;
            } else {
               best_weight += 10;
            }
         }
      }
   }
   if (write_track) {
      // If it is at least as good use the track that was from one read
      // since it is more likely to be ok
      if (best_track_weight >= best_fixed_track_weight) {
         emu_file_write_track_bits(drive_params->emu_fd, best_track_words,
               best_track_num_words, cyl, head, 
               drive_params->emu_track_data_bytes);
      } else {
        //printf("Using fixed %d,%d,%d %d %d\n",best_weight, last_weight,
        //     best_track_weight, cyl, head);
         emu_file_write_track_bits(drive_params->emu_fd, best_fixed_track_words,
               best_fixed_track_num_words, cyl, head,
               drive_params->emu_track_data_bytes);
      }
   }
   // Keep best track. Should be last track the way mfm_read works.
   if (last_weight > best_track_weight || new_track) {
      best_track_weight = last_weight;
      memcpy(best_track_words, current_track_words, current_track_words_ndx *
            sizeof(best_track_words[0]));
      best_track_num_words = current_track_words_ndx;
   }
   // Take the first track for our best fixed track
   if (new_track && sector_status_list != NULL) {
      memcpy(best_fixed_track_words, current_track_words, current_track_words_ndx *
            sizeof(best_track_words[0]));
      best_fixed_track_num_words = current_track_words_ndx;
   }
   best_fixed_track_weight = best_weight;
   // Clear for next time
   current_track_words_ndx = 0;
}


// Mark start of header in track data we are building
// Bit count is bit location in work
void mfm_mark_header_location(int bit_count) {
   // Back up 1 word to ensure we copy the header mark pattern
   // We don't have to be accurate since we can change some of
   // the gap words.
   header_track_word_ndx = current_track_words_ndx - 1;
}

// Mark start of data in track data we are building
// Bit count is bit location in work
void mfm_mark_data_location(int bit_count) {
   // Here we have to be accurate since we need to just replace the data
   data_bit = bit_count;
   data_word_ndx = current_track_words_ndx;
}


// Mark end of data in track data we are building
// Bit count is bit location in work
void mfm_mark_end_data(int bit_count, DRIVE_PARAMS *drive_params) {

   if (drive_params->emu_track_data_bytes > 0 && current_track_words_ndx*4 >=
          drive_params->emu_track_data_bytes) {
      msg(MSG_ERR, "Warning: Track data truncated writing to emulation file by %d bytes\n",
           current_track_words_ndx*4 - drive_params->emu_track_data_bytes+
           (bit_count+7)/8);
   }
}


// If we fixed an ECC error we try to put the fixed data bits back into the
// sector. Header is not fixed if it has an ECC correction. We copy the
// track words into the best track in the area for that sector.
static void update_emu_track_sector(SECTOR_STATUS *sector_status, int sect_rel0,
      uint8_t bytes[], int num_bytes) {
   int i, bit;
   int word_ndx;
   int last_bit;
   int bit_num;
   uint64_t pat64, mask64, word64;

   if (sector_status->ecc_span_corrected_data != 0) {
      //printf("updating %d %d to %d, %d\n", sect_rel0, header_track_word_ndx,
      //      current_track_words_ndx, num_bytes);

      bit_num = 31 - data_bit;
      word_ndx = data_word_ndx;
      bit_num += 1;
      if (bit_num > 31) {
         bit_num -= 32;
         word_ndx--;
      }
      last_bit = (current_track_words[word_ndx] & (1 << bit_num)) >> bit_num;
      bit_num -= 2;
      if (bit_num < 0) {
         bit_num += 32;
         word_ndx++;
      }

      // This decodes the data bits back into MFM clock and data bits
      for (i = 0; i < num_bytes; i++) {
         for (bit = 0x80; bit != 0; bit >>= 1) {
            if (bytes[i] & bit) {
               pat64 = 1;
            } else if (last_bit) {
               pat64 = 0;
            } else {
               pat64 = 2;
            }
            last_bit = pat64 & 1;

            word64 = ((uint64_t) current_track_words[word_ndx-1] << 32) |
                  current_track_words[word_ndx];
            mask64 = 0x3ll << bit_num;
            word64 = (word64 & ~mask64) | (pat64 << bit_num);
            current_track_words[word_ndx-1] = word64 >> 32;
            current_track_words[word_ndx] = word64;
            bit_num -= 2;
            if (bit_num < 0) {
               bit_num += 32;
               word_ndx++;
            }
         }
      }
   }
   // If cyl or head changed we are starting a new track so don't copy it
   // here. update_emu_track_words will copy the entire track.
   if (sector_status->cyl == last_cyl && sector_status->head == last_head) {
      for (i = header_track_word_ndx; i < current_track_words_ndx; i++) {
         best_fixed_track_words[i] = current_track_words[i];
      }
   }
}

// Write the raw decoded clock and data words into the current track word
// buffer.
int mfm_save_raw_word(DRIVE_PARAMS *drive_params, int all_raw_bits_count, 
   int int_bit_pos, int raw_word)
{
   uint32_t tmp;
   int tmp_bit_pos = int_bit_pos;
   int shift = 32 - all_raw_bits_count;

   // If we aren't generating an emulation output file don't process the
   // raw words.
   if (drive_params->emulation_filename == NULL || 
          !drive_params->emulation_output) {
      return 0;
   }

   tmp = raw_word << shift;
   tmp_bit_pos -= shift;
   if (tmp_bit_pos == 0) {
      tmp |= 1;
   }
   if (current_track_words_ndx < ARRAYSIZE(current_track_words)) {
      current_track_words[current_track_words_ndx++] = tmp;
   } else {
      msg(MSG_FATAL, "Current track words overflow index %d\n", 
          current_track_words_ndx);
      exit(1);
   }

   while (tmp_bit_pos >= 32) {

      if (current_track_words_ndx < ARRAYSIZE(current_track_words)) {
         current_track_words[current_track_words_ndx++] = 0;
         tmp_bit_pos -= 32;
      } else {
         msg(MSG_FATAL, "Current track words overflow\n");
         exit(1);
      }
   }
   return tmp_bit_pos;
}

