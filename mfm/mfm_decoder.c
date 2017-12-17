#define PRINT_SPACING 0
// These are the general routines for supporting MFM decoders.
// Call mfm_decode_setup once before trying to decode a disk
// call mfm_init_sector_status_list before decoding each track
// call mfm_update_deltas before decoding each track
// call mfm_decode_track to decode the track or
//   call mfm_wait_all_deltas to just wait for all deltas to be received and
//   possibly written to a file
// call mfm_check_deltas to get number of deltas in the buffer
// call mfm_check_header_values to verify header parameters
// call mfm_write_metadata to write extra data for each sector
// call mfm_write_sector to check sector information and possibly write it to
//   the extract file
// call mfm_decode_done when all tracks have been processed
// call mfm_handle_alt_track_ch to add alternate track to list
//
// TODO: make it use sector number information and checking CRC at data length to write data
// for sectors with bad headers. See if resyncing PLL at write boundaries improves performance when
// data bits are shifted at write boundaries.
//
// 12/17/17 DJG Aded EDAX_PV9900
// 09/30/17 DJG Added Wang 2275
// 08/11/17 DJG Added support for Convergent AWS
// 05/19/17 DJG Previous fix prevented writing sectors with data error. Back
//   to writing the best data we have for sector in extracted data file.
// 04/21/17 DJG Added better tracking of information during read. When index
//   fell in a sector the sector_status_list wasn't properly updated.
// 03/08/17 DJG Fixed Intel iSBC 215
// 02/12/17 DJG Added support for Data General MV/2000
// 02/09/17 DJG Added support for AT&T 3B2
// 02/07/17 DJG Added support for Altos 586
// 01/17/17 DJG Add flag to ignore seek errors and report missing cylinders
// 12/11/16 DJG Added logic for detecting sectors with zero contect that
//    make polynomial detection ambiguous.
// 12/04/16 DJG Added Intel iSBC_215 format. Fixed handling of bad blocks
//   for Adaptec format.
// 11/20/16 DJG Added logic to determine how much emulator track size needs
//   to be increased by to make data fit. Added logic to pass data size
//   information. 
// 11/14/16 DJG Added Telenex Autoscope, Vector4,  and Xebec_S1420 formats
// 11/02/16 DJG Add ability to write tag/metadata if it exists
// 10/28/16 DJG Improved support for LBA format disks
// 10/22/16 DJG Added unknown format found on ST-212 disk
// 10/16/16 DJG Renamed OLIVETTI to DTC. Added MOTOROLA_VME10 and SOLOSYSTEMS
// 05/21/16 DJG Improvements in alternate track handling and fix marking
//    of header and data location for fixing emulator file with ECC corrections
// 04/23/16 DJG Added EC1841 support
// 01/24/16 DJG Add MVME320 controller support
// 01/13/16 DJG Changes for ext2emu related changes on how drive formats will
//     be handled.
// 01/06/16 DJG Add code to fix extracted data file when alternate tracks are
//          used. Only a few format know how to determine alternate track
// 12/31/15 DJG Changes for ext2emu
// 12/24/15 DJG Code cleanup
// 11/13/15 DJG Added Seagate ST11M support
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

static int cyl_found[4096];

// Last LBA address processed for detecting bad sectors
static int last_lba_addr;

static void update_emu_track_sector(DRIVE_PARAMS *drive_params,
       SECTOR_STATUS *sector_status, int sect_rel0,
      uint8_t bytes[], int num_bytes);
void update_emu_track_words(DRIVE_PARAMS * drive_params,
      SECTOR_STATUS sector_status_list[], int write_track, int new_track,
      int cyl, int head);
static void print_missing_cyl(DRIVE_PARAMS *drive_params);

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


// Compare two integers for qsort
static int cmpint(const void *i1, const void *i2) {
   if (*(int *) i1 < *(int *)i2) {
       return -1;
   } else if (*(int *) i1 == *(int *) i2) {
      return 0;
   } else {
      return 1;
   }
}

// Print any errors and ECC correction from the sector_status_list.
//
// drive_params: Parameters for drive. Stats updated.
// sector_status_list: List of sector statuses
// cyl, head: Track status if for
static void print_sector_list_status(DRIVE_PARAMS *drive_params,
      SECTOR_STATUS *sector_status_list, int cyl, int head) {
   int cntr;
   int ecc_corrections = 0;
   int hard_errors = 0;
   int data_errors = 0;
   int lba_addrs[MAX_SECTORS];
   int first_sector_number = drive_params->first_sector_number;
   int num_sectors = drive_params->num_sectors;

   // Find if anything needs printing
   for (cntr = 0; cntr < num_sectors; cntr++) {
      if (!(sector_status_list[cntr].status & SECT_SPARE_BAD)) {
         if (UNRECOVERED_ERROR(sector_status_list[cntr].status)) {
            hard_errors = 1;
         }
         if (sector_status_list[cntr].status & SECT_BAD_DATA) {
            data_errors = 1;
         }
         if (sector_status_list[cntr].status & SECT_ECC_RECOVERED) {
            ecc_corrections = 1;
         }
      }
   }
   if (mfm_controller_info[drive_params->controller].analyze_type == CINFO_LBA) {
      int lba_missing = 0;
   
      if (data_errors) {
         msg(MSG_ERR_SUMMARY, "Bad sectors on cylinder %d head %d LBA:",cyl,head);
      }
      // Make a list of all LBA addresses found. If SECT_BAD_HEADER set then
      // LBA address is not valid. We then sort the list and check that the
      // numbers are consecutive.
      for (cntr = 0; cntr < num_sectors; cntr++) {
         if (sector_status_list[cntr].status & (SECT_BAD_HEADER |
              SECT_BAD_LBA_NUMBER)) {
            lba_missing++;
         } else {
            lba_addrs[cntr - lba_missing] = sector_status_list[cntr].lba_addr;
            if (sector_status_list[cntr].status & SECT_BAD_DATA &&
               !(sector_status_list[cntr].status & SECT_SPARE_BAD)) {
               msg(MSG_ERR_SUMMARY, " %d", sector_status_list[cntr].lba_addr);
            }
         }
      }
      if (data_errors) {
         msg(MSG_ERR_SUMMARY,"\n");
      }
      if (ecc_corrections) {
         msg(MSG_ERR_SUMMARY, "ECC Corrections on cylinder %d head %d LBA:",cyl,head);
         for (cntr = 0; cntr < num_sectors; cntr++) {
            if (sector_status_list[cntr].status & SECT_ECC_RECOVERED &&
                 !(sector_status_list[cntr].status & SECT_SPARE_BAD)) {
               msg(MSG_ERR_SUMMARY, " %d(", sector_status_list[cntr].lba_addr);
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

      qsort(lba_addrs, num_sectors - lba_missing, sizeof(lba_addrs[0]), cmpint);
      for (cntr = 0; cntr < num_sectors - lba_missing; cntr++) {
         if (last_lba_addr + 1 != lba_addrs[cntr]) {
            msg(MSG_ERR, "Missing LBA address %d", last_lba_addr + 1);
            if (last_lba_addr + 2 != lba_addrs[cntr]) {
               msg(MSG_ERR, " to %d", lba_addrs[cntr] - 1);
            }
            msg(MSG_ERR, "\n");
         }
         last_lba_addr = lba_addrs[cntr];
      }
   } else {
      // Print CHS errors
      if (hard_errors) {
         int last_cyl = cyl;
         msg(MSG_ERR_SUMMARY, "Bad sectors on cylinder %d", cyl);
         for (cntr = 0; cntr < num_sectors; cntr++) {
            if (!(sector_status_list[cntr].status & SECT_BAD_HEADER)) {
               if (sector_status_list[cntr].cyl != cyl &&
                   sector_status_list[cntr].cyl != last_cyl) {
                  msg(MSG_ERR_SUMMARY, "/%d",sector_status_list[cntr].cyl);
                  last_cyl = sector_status_list[cntr].cyl;
               }
            }
         }
         msg(MSG_ERR_SUMMARY, " head %d:",head);
         for (cntr = 0; cntr < num_sectors; cntr++) {
            if (!(sector_status_list[cntr].status & SECT_SPARE_BAD)) {
               if (sector_status_list[cntr].status & SECT_BAD_HEADER) {
                  msg(MSG_ERR_SUMMARY, " %dH", cntr + first_sector_number);
               }
               if (sector_status_list[cntr].status & SECT_BAD_DATA) {
                  msg(MSG_ERR_SUMMARY, " %d", cntr + first_sector_number);
               }
            }
         }
         msg(MSG_ERR_SUMMARY,"\n");
      }
      if (ecc_corrections) {
         msg(MSG_ERR_SUMMARY, "ECC Corrections on cylinder %d head %d:",cyl,head);
         for (cntr = 0; cntr < num_sectors; cntr++) {
            if (sector_status_list[cntr].status & SECT_ECC_RECOVERED &&
                 !(sector_status_list[cntr].status & SECT_SPARE_BAD)) {
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

   // If track changed and list has been set (last_cyl != -1) then process
   if (last_cyl != -1 && (cyl != last_cyl || head != last_head)) {
      update_emu_track_words(drive_params, sector_status_list, 1, 1, 
          last_cyl, last_head);
      for (i = 0; i < drive_params->num_sectors; i++) {
         if (last_sector_list[i].status & SECT_ECC_RECOVERED &&
             !(last_sector_list[i].status & SECT_SPARE_BAD)) {
            stats->num_ecc_recovered++;
         }
         if (last_sector_list[i].status & SECT_SPARE_BAD) {
            stats->num_spare_bad++;
         } else if (last_sector_list[i].status & SECT_BAD_DATA) {
            stats->num_bad_data++;
         } else if (last_sector_list[i].status & SECT_BAD_HEADER) {
            stats->num_bad_header++;
         } else {
            stats->num_good_sectors++;
         }
      }
      print_sector_list_status(drive_params, last_sector_list, 
         last_cyl, last_head);
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
      sector_status_list[i].status = SECT_BAD_HEADER | SECT_NOT_WRITTEN;
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
SECTOR_DECODE_STATUS mfm_decode_track(DRIVE_PARAMS * drive_params, int cyl, 
      int head, uint16_t deltas[], int *seek_difference,
      SECTOR_STATUS sector_status_list[])
{
   int rc;
   int i;

   for (i = 0; i < drive_params->num_sectors; i++) {
      sector_status_list[i].last_status = SECT_BAD_HEADER;
   }
   // Change in mfm_process_bytes if this if is changed
   if (drive_params->controller == CONTROLLER_WD_1006 ||
         drive_params->controller == CONTROLLER_WD_3B1 ||
         drive_params->controller == CONTROLLER_MOTOROLA_VME10 ||
         drive_params->controller == CONTROLLER_OMTI_5510 ||
         drive_params->controller == CONTROLLER_MORROW_MD11 ||
         drive_params->controller == CONTROLLER_UNKNOWN1 ||
         drive_params->controller == CONTROLLER_DEC_RQDX3 ||
         drive_params->controller == CONTROLLER_MVME320 ||
         drive_params->controller == CONTROLLER_DTC ||
         drive_params->controller == CONTROLLER_MACBOTTOM ||
         drive_params->controller == CONTROLLER_MIGHTYFRAME ||
         drive_params->controller == CONTROLLER_DG_MV2000 ||
         drive_params->controller == CONTROLLER_ADAPTEC ||
         drive_params->controller == CONTROLLER_NEWBURYDATA ||
         drive_params->controller == CONTROLLER_ELEKTRONIKA_85 ||
         drive_params->controller == CONTROLLER_SEAGATE_ST11M ||
         drive_params->controller == CONTROLLER_ALTOS_586 ||
         drive_params->controller == CONTROLLER_ATT_3B2 ||
         drive_params->controller == CONTROLLER_WANG_2275 ||
         drive_params->controller == CONTROLLER_WANG_2275_B ||
         drive_params->controller == CONTROLLER_EDAX_PV9900 ||
         drive_params->controller == CONTROLLER_CONVERGENT_AWS ||
         drive_params->controller == CONTROLLER_ISBC_215 ||
         drive_params->controller == CONTROLLER_SYMBOLICS_3620 ||
         drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
      rc = wd_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_XEROX_6085 ||
           drive_params->controller == CONTROLLER_TELENEX_AUTOSCOPE) {
      rc = tagged_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_XEBEC_104786 ||
         drive_params->controller == CONTROLLER_XEBEC_S1420 ||
         drive_params->controller == CONTROLLER_EC1841 ||
         drive_params->controller == CONTROLLER_SOLOSYSTEMS)  {
      rc = xebec_decode_track(drive_params, cyl, head, deltas, seek_difference,
            sector_status_list);
   } else if (drive_params->controller == CONTROLLER_CORVUS_H ||
         drive_params->controller == CONTROLLER_CROMEMCO ||
         drive_params->controller == CONTROLLER_VECTOR4_ST506 ||
         drive_params->controller == CONTROLLER_VECTOR4)  {
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
   last_lba_addr = -1;
   memset(cyl_found, 0, sizeof(cyl_found));

   if (write_files && drive_params->emulation_filename != NULL &&
         drive_params->emulation_output == 1) {
         // Assume 3600 RPM, 60 RPS. Make round number of words
         // Set if not set on command line
      if (drive_params->emu_track_data_bytes == 0) {
         drive_params->emu_track_data_bytes = ceil(1/60.0 * 
            mfm_controller_info[drive_params->controller].clk_rate_hz / 8 / 4)*4;
      }
      drive_params->emu_fd = emu_file_write_header(drive_params->emulation_filename,
        drive_params->num_cyl, drive_params->num_head, 
        drive_params->cmdline, drive_params->note,
        mfm_controller_info[drive_params->controller].clk_rate_hz,
        drive_params->start_time_ns, drive_params->emu_track_data_bytes);
   }

   drive_params->metadata_fd = -1;
   drive_params->ext_fd = -1;
   if (write_files && drive_params->extract_filename != NULL) {
      drive_params->ext_fd = open(drive_params->extract_filename, O_RDWR | O_CREAT |
            O_TRUNC, 0664);
      if (drive_params->ext_fd < 0) {
         perror("Unable to create output extracted data file");
         exit(1);
      }
      if (mfm_controller_info[drive_params->controller].metadata_bytes != 0) {
         char extention[] = ".metadata";
         char fn[strlen(drive_params->extract_filename) + strlen(extention) + 1];

         strcpy(fn, drive_params->extract_filename);
         strcat(fn, extention);
         drive_params->metadata_fd = open(fn, O_RDWR | O_CREAT | O_TRUNC, 0664);
         if (drive_params->metadata_fd < 0) {
            perror("Unable to create metadata output file");
            exit(1);
         }
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

static void fix_ext_alt_tracks(DRIVE_PARAMS *drive_params) {
   ALT_INFO *alt_info = drive_params->alt_llist;

   while (alt_info != NULL) { 
      uint8_t bad_data[alt_info->length];
      uint8_t good_data[alt_info->length];
      msg(MSG_DEBUG,"Swapping start bad offset %d good offset %d\n",
         alt_info->bad_offset, alt_info->good_offset);
      if (pread(drive_params->ext_fd, bad_data, sizeof(bad_data),
             alt_info->bad_offset) != sizeof(bad_data)) {
         msg(MSG_FATAL, "bad alt pread failed\n");
         exit(1);
      }
      if (pread(drive_params->ext_fd, good_data, sizeof(good_data),
             alt_info->good_offset) != sizeof(good_data)) {
         msg(MSG_FATAL, "good alt pread failed\n");
         exit(1);
      }
      if (pwrite(drive_params->ext_fd, bad_data, sizeof(bad_data),
             alt_info->good_offset) != sizeof(bad_data)) {
         msg(MSG_FATAL, "good alt pwrite failed\n");
         exit(1);
      }
      if (pwrite(drive_params->ext_fd, good_data, sizeof(good_data),
             alt_info->bad_offset) != sizeof(good_data)) {
         msg(MSG_FATAL, "bad alt pwrite failed\n");
         exit(1);
      }
      alt_info = alt_info->next; 
   }
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
   if (drive_params->ext_fd >= 0) {
      fix_ext_alt_tracks(drive_params);
      close(drive_params->ext_fd);
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
      if (drive_params->ignore_seek_errors) {
         print_missing_cyl(drive_params);
      }

      msg(MSG_STATS,
            "Expected %d sectors got %d good sectors, %d bad header, %d bad data\n",
            drive_params->num_cyl * drive_params->num_head *
            drive_params->num_sectors, stats->num_good_sectors,
            stats->num_bad_header, stats->num_bad_data);
      msg(MSG_STATS, "%d sectors marked bad or spare\n", stats->num_spare_bad);
      msg(MSG_STATS,
            "%d sectors corrected with ECC. Max bits in burst corrected %d\n",
            stats->num_ecc_recovered, stats->max_ecc_span);

      if (stats->max_track_words * 4 > drive_params->emu_track_data_bytes &&
            stats->emu_data_truncated) {
         msg(MSG_ERR, "*** To create valid emulator file rerun with --track_words %d\n",
            stats->max_track_words + 2);
      }
   }
   emu_file_close(drive_params->emu_fd, drive_params->emulation_output);
}

// This checks that the sector header values are reasonable and match the
// track and head we thought we were reading. If the cylinder doesn't match
// we return the difference between the actual and expected cylinder so
// the caller can try seeking again if needed.
//
// It also updates the sector_status_list with the information gotten from
// the sector header.
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
      SECTOR_STATUS *sector_status, DRIVE_PARAMS *drive_params,
      SECTOR_STATUS sector_status_list[]) {

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
   if (sector_status->cyl >= 0 && sector_status->cyl < ARRAYSIZE(cyl_found)) {
      cyl_found[sector_status->cyl] = 1;
   }
   // If ignore seek error we will still declare an error if greater than 250
   // to make analyze work better.
   if (!sector_status->is_lba &&
       (sector_status->head != exp_head || 
        (sector_status->cyl != exp_cyl && !drive_params->ignore_seek_errors) ||
         abs(sector_status->cyl - exp_cyl) > 250)) {
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
         //  Code copies from mfm_write_sector
         //  so sectors marked bad will be properly handled. Something cleaner
         //  would be good.
         int sect_rel0 = sector_status->sector - drive_params->first_sector_number;
         if (sect_rel0 >= drive_params->num_sectors || sect_rel0 < 0) {
            msg(MSG_ERR_SERIOUS, "Logical sector %d out of range 0-%d sector %d cyl %d head %d\n",
               sect_rel0, drive_params->num_sectors-1, sector_status->sector,
               sector_status->cyl,sector_status->head);
         } else if (sector_status->head > drive_params->num_head) {
            msg(MSG_ERR_SERIOUS,"Head out of range %d max %d cyl %d sector %d\n",
               sector_status->head, drive_params->num_head,
               sector_status->cyl, sector_status->sector);
         } else {
            int written = sector_status_list[sect_rel0].status & SECT_NOT_WRITTEN;
            sector_status_list[sect_rel0] = *sector_status;
               // Set to bad data as default. If data found good this will 
               // be changed. Keep not written flag if it was set.
            sector_status_list[sect_rel0].status |= SECT_BAD_DATA | written;
         }

}

// Write the sector data to file. We only write the best data so if the
// caller retries read with error we won't overwrite good data if this
// read has an error for this sector but the previous didn't.
// Bytes is data to write.
// drive_params: specifies the length and other information.
// sector_status: is the status of the sector writing
// sector_status_list: is the status of the data that would be written to the
//    file. Status is still updated if drive_params->ext_fd is -1 to prevent 
//    writing.
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
   off_t offset;

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
   // If we haven't written the sector yet write it even if bad
   if (sector_status_list[sect_rel0].status & SECT_NOT_WRITTEN) {
      sector_status_list[sect_rel0].status &= ~SECT_NOT_WRITTEN;
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
   // If LBA number bad don't update
   if (sector_status->status & SECT_BAD_LBA_NUMBER) {
      // Update status that would normally be updated when sector written
      sector_status_list[sect_rel0] = *sector_status;
      update = 0;
   }
   // Always update errors in emu data in case it ends up being used as
   // the best data to write
   update_emu_track_sector(drive_params, sector_status, sect_rel0, 
      all_bytes, all_bytes_len);
   if (update) {
      if (drive_params->ext_fd >= 0) {
         if (sector_status->is_lba) {
            offset = (off_t) sector_status->lba_addr * drive_params->sector_size;
         } else {
            offset = (sect_rel0) * drive_params->sector_size +
               sector_status->head * (drive_params->sector_size *
                     drive_params->num_sectors) +
                     (off_t) sector_status->cyl * (drive_params->sector_size *
                           drive_params->num_sectors *
                           drive_params->num_head);
         }
         if (lseek(drive_params->ext_fd, offset, SEEK_SET) < 0) {
            msg(MSG_FATAL, "Seek failed decoded data: %s\n", strerror(errno));
            exit(1);
         };
         if ((rc = write(drive_params->ext_fd, bytes, drive_params->sector_size)) !=
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

// Write the sector metadata to file. We will write the last sector read
// data. TODO: Should we do the keep the best logic?
// drive_params: specifies the length and other information.
// sector_status: is the status of the sector writing
// return: -1 if error found, 0 if OK.
int mfm_write_metadata(uint8_t bytes[], DRIVE_PARAMS * drive_params,
      SECTOR_STATUS *sector_status)
{
   int size = mfm_controller_info[drive_params->controller].metadata_bytes;
   size_t offset;
   int sect_rel0 = sector_status->sector - drive_params->first_sector_number;
   int rc;


   if (drive_params->metadata_fd >= 0) {
      if (sector_status->is_lba) {
         offset = (off_t) sector_status->lba_addr * size;
      } else {
         offset = (sect_rel0) * size +
            sector_status->head * (size *
                  drive_params->num_sectors) +
                  (off_t) sector_status->cyl * (size *
                       drive_params->num_sectors * drive_params->num_head);
      }
      if (lseek(drive_params->metadata_fd, offset, SEEK_SET) < 0) {
         msg(MSG_FATAL, "Seek failed metadata: %s\n", strerror(errno));
         exit(1);
      };
      if ((rc = write(drive_params->metadata_fd, bytes, size)) != size) {
         msg(MSG_FATAL, "Metadata write failed, rc %d: %s", rc, strerror(errno));
         exit(1);
      }
   }
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
   uint8_t bytes[], int bytes_crc_len, int total_bytes,
   STATE_TYPE *state, int cyl, int head,
   int *sector_index, int *seek_difference, 
   SECTOR_STATUS sector_status_list[], SECTOR_DECODE_STATUS init_status) {

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
      start = mfm_controller_info[drive_params->controller].header_crc_ignore;

#if 0
      static int dump_fd = 0;
      static int first = 1;
      if (first) {
         printf("Dumping starting at byte %d for %d bytes\n",start, bytes_crc_len - start);
         first = 0;
      }
      if  (dump_fd == 0) {
         dump_fd = open("dumpheader",O_WRONLY | O_CREAT | O_TRUNC, 0666);
      }
      write(dump_fd, &bytes[start], bytes_crc_len - start);
#endif
      crc_info = drive_params->header_crc;
      if (msg_get_err_mask() & MSG_DEBUG_DATA) {
         mfm_dump_bytes(bytes, bytes_crc_len, cyl, head, *sector_index,
               MSG_DEBUG_DATA);
      }
      name = "header";
   } else {
      start = mfm_controller_info[drive_params->controller].data_crc_ignore;
#if 0
      static int dump_fd = 0;
      static int first = 1;
      if (first) {
         printf("Dumping starting at byte %d for %d bytes\n",start, bytes_crc_len - start);
         first = 0;
      }
      if  (dump_fd == 0) {
         dump_fd = open("dumpdata",O_WRONLY | O_CREAT | O_TRUNC, 0666);
      }
      write(dump_fd, &bytes[start], bytes_crc_len - start);
#endif

      crc_info = drive_params->data_crc;
      if (msg_get_err_mask() & MSG_DEBUG_DATA) {
         mfm_dump_bytes(bytes, bytes_crc_len, cyl, head, *sector_index,
               MSG_DEBUG_DATA);
      }
      name = "data";
   }
   if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE ||
       (drive_params->controller == CONTROLLER_WANG_2275 &&
         *state == PROCESS_HEADER)) {
      crc = checksum64(&bytes[start], bytes_crc_len-crc_info.length/8-start, &crc_info);
      if (crc_info.length == 8) {
        crc = crc & 0xff;
        if (crc == bytes[bytes_crc_len-1]) {
           crc = 0;
        }
      } else if (crc_info.length == 16) {
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
         msg(MSG_FATAL, "Invalid checksum length %d\n",crc_info.length);
         exit(1);
      }
   } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
      if (*state == PROCESS_HEADER) {
         crc = 0;
      } else {
         crc = crc64(&bytes[start], bytes_crc_len-start, &crc_info);
      }
   } else {
      int i;
      crc = crc64(&bytes[start], bytes_crc_len-start, &crc_info);
      // If all the data and CRC is zero and CRC returns zero
      // mark it as ambiguous crc since any polynomial will match 
      if (crc == 0 && drive_params->analyze_in_progress) {
         for (i = start; i < bytes_crc_len; i++) {
            if (bytes[i] != 0) {
               break;
            }
         }
         if (i == bytes_crc_len) {
            init_status |= SECT_AMBIGUOUS_CRC;
         }
      }
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
         // TODO: This includes SECT_SPARE_BAD ECC corrections in the
         // final value printed. We don't have the info to fix here
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
            drive_params->controller == CONTROLLER_WD_3B1 ||
            drive_params->controller == CONTROLLER_MOTOROLA_VME10 ||
            drive_params->controller == CONTROLLER_OMTI_5510 ||
            drive_params->controller == CONTROLLER_MORROW_MD11 ||
            drive_params->controller == CONTROLLER_UNKNOWN1 ||
            drive_params->controller == CONTROLLER_DEC_RQDX3 ||
            drive_params->controller == CONTROLLER_MVME320 ||
            drive_params->controller == CONTROLLER_DTC ||
            drive_params->controller == CONTROLLER_MACBOTTOM ||
            drive_params->controller == CONTROLLER_MIGHTYFRAME ||
            drive_params->controller == CONTROLLER_DG_MV2000 ||
            drive_params->controller == CONTROLLER_ADAPTEC ||
            drive_params->controller == CONTROLLER_NEWBURYDATA ||
            drive_params->controller == CONTROLLER_ELEKTRONIKA_85 ||
            drive_params->controller == CONTROLLER_SEAGATE_ST11M ||
            drive_params->controller == CONTROLLER_ALTOS_586 ||
            drive_params->controller == CONTROLLER_ATT_3B2 ||
            drive_params->controller == CONTROLLER_WANG_2275 ||
            drive_params->controller == CONTROLLER_WANG_2275_B ||
            drive_params->controller == CONTROLLER_EDAX_PV9900 ||
            drive_params->controller == CONTROLLER_CONVERGENT_AWS ||
            drive_params->controller == CONTROLLER_ISBC_215 ||
            drive_params->controller == CONTROLLER_SYMBOLICS_3620 ||
            drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         status |= wd_process_data(state, bytes, total_bytes, crc, cyl, 
               head, sector_index,
               drive_params, seek_difference, sector_status_list, ecc_span,
               init_status);
      } else if (drive_params->controller == CONTROLLER_XEROX_6085 ||
             drive_params->controller == CONTROLLER_TELENEX_AUTOSCOPE) {
         status |= tagged_process_data(state, bytes, total_bytes, crc, cyl,
               head, sector_index,
               drive_params, seek_difference, sector_status_list, ecc_span,
               init_status);
      } else if (drive_params->controller == CONTROLLER_XEBEC_104786 ||
            drive_params->controller == CONTROLLER_XEBEC_S1420 ||
            drive_params->controller == CONTROLLER_EC1841 ||
            drive_params->controller == CONTROLLER_SOLOSYSTEMS)  {
         status |= xebec_process_data(state, bytes, total_bytes, crc, cyl, head,
               sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span, init_status);
      } else if (drive_params->controller == CONTROLLER_CORVUS_H ||
            drive_params->controller == CONTROLLER_CROMEMCO ||
            drive_params->controller == CONTROLLER_VECTOR4_ST506 ||
            drive_params->controller == CONTROLLER_VECTOR4) {
         status |= corvus_process_data(state, bytes, total_bytes, crc, cyl,
               head, sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span, init_status);
      } else if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
         status |= northstar_process_data(state, bytes, total_bytes, crc, cyl,
               head, sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span, init_status);
      } else if (drive_params->controller == CONTROLLER_NORTHSTAR_ADVANTAGE) {
         status |= northstar_process_data(state, bytes, total_bytes, crc, cyl,
               head, sector_index, drive_params, seek_difference,
               sector_status_list, ecc_span, init_status);
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
// For examining track timing
int header_track_tot_bit_count;;
int data_tot_bit_count;

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
// Bit count is bit location in word
// bit_offset is difference between proper header location and bit
//    count when routine called
// tot_bit_count is for finding header separation
void mfm_mark_header_location(int bit_count, int bit_offset, int tot_bit_count) {
#if PRINT_SPACING
   if (header_track_tot_bit_count != 0 && (tot_bit_count) > 
        header_track_tot_bit_count) {
      msg(MSG_INFO, "Header difference %.1f bytes\n", 
        (tot_bit_count - header_track_tot_bit_count) / 16.0);
      msg(MSG_INFO, "Data to header difference %.1f %.1f bytes\n", 
        (tot_bit_count - data_tot_bit_count) / 16.0,
        (data_tot_bit_count - header_track_tot_bit_count) / 16.0);
   } else {
      msg(MSG_INFO, "First Header %.1f bytes\n", tot_bit_count / 16.0);
   }
#endif
   // Back up 1 word to ensure we copy the header mark pattern
   // We don't have to be accurate since we can change some of
   // the gap words.
   header_track_word_ndx = MAX(current_track_words_ndx - 1, 0);
   header_track_tot_bit_count = tot_bit_count - bit_offset;
}

// Mark start of data in track data we are building
// Bit count is bit location in word
// bit_offset is difference between proper header location and bit
//    count when routine called
// tot_bit_count is for finding header separation
void mfm_mark_data_location(int bit_count, int bit_offset, int tot_bit_count) {
   // Here we have to be accurate since we need to just replace the data
   data_word_ndx = current_track_words_ndx;
   // Shift to correct bit and word index based on bit_offset
   data_bit = bit_count - bit_offset;
   if (data_bit >= 32) {
      data_bit -= 32;
      data_word_ndx++;
   }
   if (data_bit < 0) {
      data_bit += 32;
      data_word_ndx--;
   }
   data_tot_bit_count = tot_bit_count - bit_offset;
}


// Mark end of data in track data we are building
// Bit count is bit location in work
void mfm_mark_end_data(int bit_count, DRIVE_PARAMS *drive_params) {

   if (drive_params->emu_track_data_bytes > 0 && current_track_words_ndx*4 >=
          drive_params->emu_track_data_bytes) {
      msg(MSG_ERR, "Warning: Track data truncated writing to emulation file by %d bytes, need %d words\n",
           current_track_words_ndx*4 - drive_params->emu_track_data_bytes+
           (bit_count+7)/8, current_track_words_ndx);
      drive_params->stats.emu_data_truncated = 1;
   }
   if (current_track_words_ndx > drive_params->stats.max_track_words) {
      drive_params->stats.max_track_words = current_track_words_ndx;
   }
}


// If we fixed an ECC error we try to put the fixed data bits back into the
// sector. Header is not fixed if it has an ECC correction. We copy the
// track words into the best track in the area for that sector.
static void update_emu_track_sector(DRIVE_PARAMS *drive_params, SECTOR_STATUS 
      *sector_status, int sect_rel0, uint8_t bytes[], int num_bytes) {
   int i, bit;
   int word_ndx;
   int last_bit;
   int bit_num;
   uint64_t pat64, mask64, word64;

   if (sector_status->ecc_span_corrected_data != 0) {
#if 0
      printf("updating %d %d to %d, %d\n", sect_rel0, header_track_word_ndx,
            current_track_words_ndx, num_bytes);
      printf("word ndx %d bit %d\n", data_word_ndx, data_bit);
#endif
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
      int start = MAX(0, header_track_word_ndx -  
         mfm_controller_info[drive_params->controller].copy_extra);
      for (i = start; i < current_track_words_ndx; i++) {
         best_fixed_track_words[i] = current_track_words[i];
      }
   }
}

// Write the raw decoded clock and data words into the current track word
// buffer. This routine only called if bits left in raw_word plus bits
// about to be added are >= 32 bits.
//
// drive_params: Drive parameters
// all_raw_bits_count: How many bits left in raw_word to process
// int_bit_pos: Number of zeros that are being added before next one
// raw_word: bits accululated so far
int mfm_save_raw_word(DRIVE_PARAMS *drive_params, int all_raw_bits_count, 
   int int_bit_pos, int raw_word)
{
   uint32_t tmp;
   // Get shift to move unprocessed bits to MSB
   int shift = 32 - all_raw_bits_count;

   // If we aren't generating an emulation output file don't process the
   // raw words.
   if (drive_params->emulation_filename == NULL || 
          !drive_params->emulation_output) {
      return 0;
   }

   // Shift unprocessed bits to MSB
   tmp = raw_word << shift;
   // If the LSB after shift is where int_bit_pos says a one goes then add it
   int_bit_pos -= shift;
   if (int_bit_pos == 0) {
      tmp |= 1;
   }
   // Save word
   if (current_track_words_ndx < ARRAYSIZE(current_track_words)) {
      current_track_words[current_track_words_ndx++] = tmp;
   } else {
      msg(MSG_FATAL, "Current track words overflow index %d\n", 
          current_track_words_ndx);
      exit(1);
   }

   // Add any more zeros in int_bit_pos until it is less than 32. Those
   // bits will be added to raw_word by caller.
   while (int_bit_pos >= 32) {
      if (current_track_words_ndx < ARRAYSIZE(current_track_words)) {
         current_track_words[current_track_words_ndx++] = 0;
         int_bit_pos -= 32;
      } else {
         msg(MSG_FATAL, "Current track words overflow\n");
         exit(1);
      }
   }
   // This will be all_raw_bits_count on next call
   return int_bit_pos;
}

// This adds to alternate track link list the data that needs to be
// swapped to put the good data in the proper location in the extracted
// data file.
//
// drive_params: Drive parameters
// bad_cyl: Cylinder that has alterinate track assigned for
// bad_head: Head that has alternate track assigned for
// good_cyl: The alternate cylinder assigned
// good_head: The alternate head assigned.
void mfm_handle_alt_track_ch(DRIVE_PARAMS *drive_params, unsigned int bad_cyl, 
      unsigned int bad_head, unsigned int good_cyl, unsigned int good_head) {
   ALT_INFO *alt_info;

   // Don't perform alt track processing if analyzing.
   if (drive_params->analyze_in_progress) {
      return;
   }
   if (bad_cyl >= drive_params->num_cyl) {
      msg(MSG_ERR, "Bad alternate cylinder %d out of valid range %d to %d\n",
         bad_cyl, 0, drive_params->num_cyl - 1);
      return;
   }
   if (good_cyl >= drive_params->num_cyl) {
      msg(MSG_ERR, "Good alternate cylinder %d out of valid range %d to %d\n",
         good_cyl, 0, drive_params->num_cyl - 1);
      return;
   }
   if (bad_head >= drive_params->num_head) {
      msg(MSG_ERR, "Bad alternate head %d out of valid range %d to %d\n",
         bad_head, 0, drive_params->num_head - 1);
      return;
   }
   if (good_head >= drive_params->num_head) {
      msg(MSG_ERR, "Bad alternate head %d out of valid range %d to %d\n",
         good_head, 0, drive_params->num_head - 1);
      return;
   }
   alt_info = msg_malloc(sizeof(ALT_INFO), "Alt info");

   memset(alt_info, 0, sizeof(ALT_INFO));
   alt_info->bad_offset = (bad_cyl * drive_params->num_head +
      bad_head) * drive_params->num_sectors * drive_params->sector_size;
   alt_info->good_offset = (good_cyl * drive_params->num_head +
      good_head) * drive_params->num_sectors * drive_params->sector_size;
   alt_info->length = drive_params->num_sectors * drive_params->sector_size;

   alt_info->next = drive_params->alt_llist; 
      // Alternate is reported with same information for each
      // sector. Only add one copy
   if (drive_params->alt_llist == NULL || 
         alt_info->bad_offset != drive_params->alt_llist->bad_offset ||
         alt_info->good_offset != drive_params->alt_llist->good_offset) {
      drive_params->alt_llist = alt_info;
   }
}

// Print sectors not found during read/decoding
//
// drive_params: Drive parameters
static void print_missing_cyl(DRIVE_PARAMS *drive_params) {
   int i;
   int cyl_missing = 0;

   for (i = 0; i < drive_params->num_cyl; i++) {
      if (cyl_found[i] == 0) {
         cyl_missing = 1;
         break;
      }
   }
   if (cyl_missing) {
      printf("Cylinders not found\n");
      for (i = 0; i < drive_params->num_cyl; i++) {
         if (cyl_found[i] == 0) {
            printf("  %d\n",i);
         }
      }
   }
}
