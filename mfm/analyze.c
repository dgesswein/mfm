// This module analyzes the disk format
// Call analyze_disk to perform the analysis
//
// We perform the analysis by trying the various formats we know of until we
// find one that matches and other tests. See the routines for details.

// Copyright 2018 David Gesswein.
// This file is part of MFM disk utilities.
//
// 03/09/20 DJG Fix finding number of heads for LBA disk where
//    selecting for example non existing head 4 gives head 0 data
// 01/25/20 DJG Give more time after seek for track0 to change to prevent
//    seek analyze failures with RD54 drive
// 10/05/19 DJG Fixes to detect when CONT_MODEL controller doesn't really
//    match format
// 06/19/19 DJG Added missing /n to error message
// 06/08/19 DJG Don't say disk is RLL if secondary period couldn't be
//    determined.
// 03/12/19 DJG Fixed detecting format with MODEL using wrong number or 
//    first sector number
// 11/03/18 DJG Renamed variable
// 09/10/18 DJG Made code not allowing larger sector sizes to match when
//    shorter found to allow larger sectors if not many matches for shorter.
// 06/22/18 DJG Fix checking of sector numbers for analyze_model. Detect
//    SA1000 bit rate.
// 03/31/18 DJG Added code to analyze drive formats marked as MODEL/
//     have track_layout defined.
// 03/09/18 DJG Added error message
// 10/20/17 DJG Fixed error print wording in analyze_seek
// 12/10/16 DJG Added logic to ignore Ambiguous CRC polynomial match on all
//    zero data.
// 11/14/16 DJG Make Data CRC max span reasonable for formats not using
//    separate data CRC. Doesn't really do anything, just prevents it
//    looking funny. Added error for analyze only to handle Telenex Autoscope
//    without hurting error recovery on Xerox 6085
// 10/31/16 DJG Redid how Logical block address disks are detected.
//    Previous method didn't work when spare/bad sectors marked.
// 10/19/16 DJG Fixed Mightyframe detection to reduce probability of false
//    deteection.
// 10/16/16 DJG Added logic to detect RLL disks.
// 10/02/16 DJG Rob Jarratt change to detect recalibrate when finding number
//    of cylinders and suggested simplification for determining slow/fast
//    seek.
// 10/01/16 DJG Handle tracks with one sector.
// 01/24/16 DJG Fix a couple errors with prints. 
// 01/13/16 DJG Changes for ext2emu related changes on how drive formats will
//     be handled.
// 11/22/15 DJG Add special logic for ST11M controller detection
// 11/01/15 DJG Analyze header and data together. The Elektronika_85 has
//    same header format as other drives but the data area is different.
//    Only CHS changed. TODO LBA should be redone to match
// 05/16/15 DJG Changes to support new formats, fix bugs, and allow
//    analyzing transition and emulation files.
// 01/04/15 DJG Moved data tables to mfm_decoder.h since mfm_controller
//    info needed to reference. Changed loop order analyze_header and
//    data to allow poly and initial value to only use valid values
//    for controller. NorthStar uses checksum so poly is ignored.
//    Set proper sector size for controller in analyze_header which
//    Corvus_H needs since it doesn't separate data and header.
//    Added setting start_time_ns.
//    Fixed detecting non buffered seek for ST506.
// 11/09/14 DJG Interleave will not be checked by default
// 10/01/14 DJG Added new polynomial to analyize and prefixed printing
//    of polynomial with 0x 
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
#include <inttypes.h>
#include <math.h>
#include <limits.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "deltas_read.h"
#include "drive.h"
#include "analyze.h"
#include "parse_cmdline.h"

// Prints CRC information
static void print_crc_info(CRC_INFO *crc_info, int msg_type) {
   msg(msg_type, "Polynomial 0x%llx length %d initial value 0x%llx\n", crc_info->poly, crc_info->length,
         crc_info->init_value);
}

// Errors we enable from MFM decoding routines while analyzing.
static int decode_errors;

// Clear upper bits to make value only havea length bits
static uint64_t trim_value(uint64_t value, int length) {
   if (length < 64) {
      value &= (((uint64_t) 1 << length) - 1);
   }
   return value;
}

// This routine finds the weighted index of the next peak in the histogram
// The bins used in the calculattion are cleared
// histogram: counts for each bin
// length: number of entries in histogram
static double avg_peak(int histogram[], int length)
{
   int start = 0;
   #define HIST_LIMIT 500
   int sum = 0, sum_mult = 0;
   int i;

   for (i = 0; i < length-1; i++) {
      if (histogram[i] > HIST_LIMIT) {
         start = i;
         break;
      }
   }
   if (start == 0) {
      msg(MSG_INFO, "No more peaks in histogram\n");
      return 0;
   } else {
      for (i = start; i < length && histogram[i] > HIST_LIMIT ; i++) {
         sum += histogram[i];
         sum_mult += histogram[i] * i;
         histogram[i] = 0;
      }
      return (double) sum_mult / sum;
   }
}

// Estimate the clock rate used to encode the data. We find the
// difference in average delta for the minimum delta and the next
// time in the histogram. The difference is the clock rate used to
// encode. Estimate is coarse.
static void analyze_rate(DRIVE_PARAMS *drive_params, int cyl, int head, 
   uint16_t *deltas, int max_deltas)
{
    int histogram[100];
    int i;
    double rate1, rate2;

    memset(histogram, 0, sizeof(histogram));

    deltas_wait_read_finished();
    for (i = 0; i < deltas_get_count(0); i++) {
       if (deltas[i] < ARRAYSIZE(histogram)) {
          histogram[deltas[i]]++;
       }
    }
#if 0
    for (i = 0; i < ARRAYSIZE(histogram); i++) {
      printf("%d, %d\n", i, histogram[i]);
    }
#endif
    rate1 = avg_peak(histogram, ARRAYSIZE(histogram)) * CLOCKS_TO_NS;
    rate2 = avg_peak(histogram, ARRAYSIZE(histogram)) * CLOCKS_TO_NS;
  
    if (fabs(rate1 - 230.4) <= 8.0) {
       msg(MSG_ERR, "Primary transition period %.0f ns, hopefully this is a SA1000 type disk\n",
         rate1);
    } else if (fabs(rate1 - 200) > 8.0) {
       msg(MSG_ERR, "Primary transition period %.0f ns, should be around 200\n",
         rate1);
    } else {
       if (rate2 <= 280 && rate2 != 0) {
          msg(MSG_ERR, "Secondary transition period %.0f ns, likely RLL\n",
            rate2);
          msg(MSG_ERR, "RLL is not currently supported\n");
       }
    }
    msg(MSG_DEBUG, "First two transition periods %.0f, %.0f ns\n",
         rate1, rate2);
}

// Try to find match for existing fully defined format. Some of the formats
// have all data defined such as CRC and other are just defining the header
// format.
//
// drive_params: Drive parameters determined so far and return what we have determined
// Cyl and head: What track the data was from.
// deltas: MFM delta time transition data to analyze. (filled after read)
// max_deltas: Size of deltas array
// match_count: Number of headers that matched for each format found
// return: Number of matching formats found
static int analyze_model(DRIVE_PARAMS *drive_params, int cyl, int head, 
   void *deltas, int max_deltas)
{
   int i;
   int cont;
   //int controller_type = -1;
   // Variable to restore global error print mask
   int msg_mask_hold;
   // The status of each sector decoded.
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   // Set if format doesn't match
   int not_match;
   // Number of matching formats and what formats matched
   int matches = 0;
   int match_list[50];

   drive_read_track(drive_params, cyl, head, deltas, max_deltas);

   analyze_rate(drive_params, cyl, head, deltas, max_deltas);

   drive_params->num_head = MAX_HEAD;

   // Search all the fully defined formats we know about.
   // If LBA format don't try to analyze if cyl and head are zero since CHS
   // headers will match
   for (cont = 0; mfm_controller_info[cont].name != NULL; cont++) {
      int missing_count = 0;

      if ((mfm_controller_info[cont].analyze_type == CINFO_LBA &&
           cyl == 0 && head == 0) ||
           mfm_controller_info[cont].write_data_crc.length == 0) {
         continue; // ****
      }
//printf("Checking %s\n", mfm_controller_info[cont].name);
      parse_set_drive_params_from_controller(drive_params, cont);

      mfm_init_sector_status_list(sector_status_list, MAX_SECTORS);
      msg_mask_hold = msg_set_err_mask(decode_errors);
      // Decode track
      mfm_decode_track(drive_params, cyl, head, deltas, NULL, 
                  sector_status_list);
      msg_set_err_mask(msg_mask_hold);
      not_match = 0;
      for (i = 0; i < MAX_SECTORS; i++) {
         // If we find a good sector outside the range we expect or are
         // missing a sector then don't consider it a match. Read errors
         // can cause format match to fail.
         if (sector_status_list[i].status & ANALYZE_WRONG_FORMAT) {
            not_match = 1;
         }
         if (sector_status_list[i].status & SECT_BAD_HEADER) {
            if (i < drive_params->num_sectors) {
               // Allow one missed sector
               if (missing_count++ >= 1) {
                  not_match = 1;
               }
            }
         } else {
            if (i >= drive_params->num_sectors) {
               not_match = 1;
            }
         }
      }
      int good_data_count = 0;
      for (i = drive_params->first_sector_number; 
            i < drive_params->first_sector_number + drive_params->num_sectors; i++) {
         if (!(sector_status_list[i].status & (SECT_BAD_DATA | SECT_BAD_HEADER))) {
            good_data_count++;
         }
      }
//printf("%s not match %d good %d\n", mfm_controller_info[cont].name, not_match, good_data_count);
      if (!not_match && good_data_count >= ceil(drive_params->num_sectors * 2 / 3.0) &&
             matches < ARRAYSIZE(match_list)) {
         match_list[matches++] = cont;
         msg(MSG_INFO, "Found matching format %s:\n", 
            mfm_controller_info[cont].name);
      }
   }

   // If we found at least one match set drive parameters to first match
   if (matches >= 1) {
      parse_set_drive_params_from_controller(drive_params, match_list[0]);
   }
   return matches;
}

// Try to find the controller type (header format) and CRC parameters for the
// header portion of sectors.
// The data to analyze has already been read before routine called.
//
// drive_params: Drive parameters determined so far and return what we have determined
// Cyl and head: What track the data was from.
// deltas: MFM delta time transition data to analyze. (filled after read)
// max_deltas: Size of deltas array
// drive_params_list: Data for all formats that match
// drive_params_list_len: Size of drive_params_list array
// match_count: Number of headers that matched for each format found
// return: Number of matching formats found
static int analyze_header(DRIVE_PARAMS *drive_params, int cyl, int head, 
   void *deltas, int max_deltas, DRIVE_PARAMS drive_params_list[], 
   int drive_params_list_len, int match_count[])
{
   // Loop variables
   int poly, init, cont;
   int i;
   int controller_type = -1;
   // Numbers of good sectors found
   int good_header_count, previous_good_header_count = 0;
   // Variable to restore global error print mask
   int msg_mask_hold;
   // The status of each sector decoded.
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   // And read status
   SECTOR_DECODE_STATUS status;
   // Index into drive_params_list
   int drive_params_list_index = 0;
   // Minimum and maximum LBA to verify they make sense
   int min_lba_addr, max_lba_addr;

   drive_read_track(drive_params, cyl, head, deltas, max_deltas);

   analyze_rate(drive_params, cyl, head, deltas, max_deltas);

   drive_params->num_sectors = MAX_SECTORS;
   drive_params->num_head = MAX_HEAD;
   // Don't use ECC while trying to find format
   drive_params->header_crc.ecc_max_span = 0;
   drive_params->data_crc.ecc_max_span = 0;

   // Try an exhaustive search of all the formats we know about. If we get too
   // many we may have to try something smarter.
   // If LBA format don't try to analyze if cyl and head are zero since CHS
   // headers will match
   for (cont = 0; mfm_controller_info[cont].name != NULL; cont++) {
      if ((mfm_controller_info[cont].analyze_type == CINFO_LBA &&
           cyl == 0 && head == 0) ||
           mfm_controller_info[cont].analyze_search == CONT_MODEL) {
         continue; // ****
      }
      // Make sure these get set at bottom for final controller picked
      // Sector size will be set when we analyze the data header
      drive_params->controller = cont;
      drive_params->sector_size = 
         mfm_controller_info[cont].analyze_sector_size;
      if (!drive_params->dont_change_start_time) {
         drive_params->start_time_ns = 
            mfm_controller_info[cont].start_time_ns;
      }
      // TODO: This would be faster if we just searched in mfm_process_bytes
      // where we check the CRC instead of decoding the MFM transitions
      // each time.
      for (poly = mfm_controller_info[cont].header_start_poly; 
             poly < mfm_controller_info[cont].header_end_poly; poly++) {
         drive_params->header_crc.poly = mfm_all_poly[poly].poly;
         drive_params->header_crc.length = mfm_all_poly[poly].length;
         for (init = mfm_controller_info[cont].start_init; 
                init < mfm_controller_info[cont].end_init; init++) {
            // If not correct size don't try this initial value
            if (!(mfm_all_init[init].length == -1 || mfm_all_init[init].length ==
                 drive_params->data_crc.length)) {
               continue;
            }
            drive_params->header_crc.init_value = trim_value(mfm_all_init[init].value,
               drive_params->header_crc.length);
            drive_params->data_crc = drive_params->header_crc;
            msg(MSG_DEBUG, "Trying controller %s ", 
               mfm_controller_info[drive_params->controller].name);
            print_crc_info(&drive_params->header_crc, MSG_DEBUG);
            
            // After the CRC has gone to zero additional zero bytes will
            // not cause CRC errors. If we found a valid header we won't
            // check any longer ones to prevent false matches.
            if (controller_type != -1 && mfm_controller_info[cont].header_bytes > 
                 mfm_controller_info[controller_type].header_bytes) {
               break;
            }
            mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
            msg_mask_hold = msg_set_err_mask(decode_errors);
            // Decode track
            status = mfm_decode_track(drive_params, cyl, head, deltas, NULL, 
                  sector_status_list);
            msg_set_err_mask(msg_mask_hold);
            if (status & SECT_ZERO_HEADER_CRC) {
               msg(MSG_DEBUG, "Found zero CRC header controller %s:\n",
                     mfm_controller_info[drive_params->controller].name);
               print_crc_info(&drive_params->header_crc, MSG_DEBUG);

            }
            // Now find out how many good sectors we got with these parameters
            good_header_count = 0;
            min_lba_addr = 0x7fffffff;
            max_lba_addr = -1;
            for (i = 0; i < drive_params->num_sectors; i++) {
               if (!(sector_status_list[i].status & SECT_BAD_HEADER) &&
                   !(sector_status_list[i].status & SECT_AMBIGUOUS_CRC)) {
                  good_header_count++;
                  if (sector_status_list[i].lba_addr < min_lba_addr) {
                     min_lba_addr = sector_status_list[i].lba_addr;
                  } else if (sector_status_list[i].lba_addr > max_lba_addr) {
                     max_lba_addr = sector_status_list[i].lba_addr;
                  }
               }
            }
            if (mfm_controller_info[cont].analyze_type == CINFO_LBA ) {
            }
            // If LBA drive make sure addresses are somewhat adjacent and
            // plausible for the cylinder. If not clear good header count
            if (mfm_controller_info[cont].analyze_type == CINFO_LBA &&
               (max_lba_addr - min_lba_addr > drive_params->num_sectors ||
                  max_lba_addr - min_lba_addr + 1 < good_header_count ||
                  min_lba_addr > cyl * 16 * 34)) {
               good_header_count = 0;
            }
            // If we found at least 2 sectors or 1 if sector is large
            // enough to only have one per track 
            if (good_header_count >= 2 || (good_header_count == 1 &&
                 drive_params->sector_size > 9000) ) {
               // Keep the best
               if (good_header_count > previous_good_header_count) {
                  controller_type = drive_params->controller;
                  previous_good_header_count = good_header_count;
               }
               if (drive_params_list_index >= drive_params_list_len) {
                  msg(MSG_FATAL, "Too many header formats found %d\n", 
                    drive_params_list_index);
                  exit(1);
               }
               // Save in list
               drive_params_list[drive_params_list_index] =
                 *drive_params;
               drive_params_list[drive_params_list_index].header_crc.ecc_max_span 
                  = mfm_all_poly[poly].ecc_span;
               // Set the data CRC span also so drives without seaparate data
               // and header ECC will have both fields correct.
               drive_params_list[drive_params_list_index].data_crc.ecc_max_span 
                  = mfm_all_poly[poly].ecc_span;
               match_count[drive_params_list_index++] = good_header_count; 
               msg(MSG_DEBUG, "Found %d headers matching:\n", good_header_count);
               print_crc_info(&drive_params->header_crc, MSG_DEBUG);
               msg(MSG_DEBUG, "Controller type %s\n", 
                   mfm_controller_info[drive_params->controller].name);
            }
         }
      }
   }
   return drive_params_list_index;
}

// Try to find the sector size and CRC parameters for the data portion of
// sectors.
// The data to analyze has already been read before routine called.
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze
// max_deltas: Number of deltas
// header_match: Number of headers found
// *best_match_count: Return Highest good sectors found for set of parameters
// return: Number of matching formats found.
static int analyze_data(DRIVE_PARAMS *drive_params, int cyl, int head, void *deltas, int max_deltas, int headers_match, int *best_match_count)
{
   // Return value
   int rc = 0;
   // Loop variables
   int poly, init, size_ndx;
   int i;
   // The best match CRC and sector size info so far
   CRC_INFO data_crc_info;
   // And read status
   SECTOR_DECODE_STATUS status;
   int sector_size = 0;
   // Numbers of good sectors found
   int good_data_count, previous_good_data_count = 0;
   // Variable to restore global error print mask
   int msg_mask_hold;
   // The status of each sector decoded.
   SECTOR_STATUS sector_status_list[MAX_SECTORS];

   *best_match_count = 0;

   drive_read_track(drive_params, cyl, head, deltas, max_deltas);

   data_crc_info.poly = 0;
   // Try an exhaustive search of all the formats we know about. If we get too
   // many we may have to try something smarter.
   for (poly = mfm_controller_info[drive_params->controller].data_start_poly; 
          poly < mfm_controller_info[drive_params->controller].data_end_poly; poly++) {
      drive_params->data_crc.poly = mfm_all_poly[poly].poly;
      drive_params->data_crc.length = mfm_all_poly[poly].length;
      // This sometimes gets false corrections when using wrong polynomial
      // We put it back when we save the best value.
      drive_params->data_crc.ecc_max_span = 0;
      for (init = mfm_controller_info[drive_params->controller].start_init; 
            init < mfm_controller_info[drive_params->controller].end_init; init++) {
         // If not correct size don't try this initial value
         if (!(mfm_all_init[init].length == -1 || mfm_all_init[init].length ==
              drive_params->data_crc.length)) {
            continue;
         }
         drive_params->data_crc.init_value = trim_value(mfm_all_init[init].value,
               drive_params->data_crc.length);
         for (size_ndx = 0; mfm_all_sector_size[size_ndx] != -1; size_ndx++) {
            drive_params->sector_size = mfm_all_sector_size[size_ndx];
            // If sector longer than one we already found don't try it. More
            // zeros after CRC match will still match. Still try larger
            // sector sizes if we didn't match that many sectors
            if (sector_size != 0 && drive_params->sector_size > sector_size &&
                 previous_good_data_count >= .6 * headers_match) {
               continue;
            }
            msg(MSG_DEBUG, "Trying controller data %s len %d ", 
               mfm_controller_info[drive_params->controller].name,
               drive_params->sector_size);
            print_crc_info(&drive_params->data_crc, MSG_DEBUG);

            mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
            msg_mask_hold = msg_set_err_mask(decode_errors);
            // Decode track
            status = mfm_decode_track(drive_params, cyl, head, deltas, NULL, sector_status_list);
            msg_set_err_mask(msg_mask_hold);
            if (status & SECT_ZERO_DATA_CRC) {
               msg(MSG_DEBUG, "Found zero CRC data size %d:\n", drive_params->sector_size);
               print_crc_info(&drive_params->data_crc, MSG_DEBUG);

            }
            // Now find out how many good sectors we got with these parameters
            good_data_count = 0;
            for (i = 0; i < drive_params->num_sectors; i++) {
               if (!UNRECOVERED_ERROR(sector_status_list[i].status) &&
                     !(sector_status_list[i].status & SECT_ANALYZE_ERROR) &&
                     !(sector_status_list[i].status & SECT_AMBIGUOUS_CRC)) {
                  good_data_count++;
               }
            }
            // If we found a good sector
            if (good_data_count > 0) {
               rc++;
               // If we have a previous match print both
               if (data_crc_info.poly != 0) {
                  msg(MSG_ERR_SERIOUS,
                     "Found multiple matching data CRC parameters. Largest matches will be used:\n");
                  msg(MSG_ERR_SERIOUS, "Matches %d sector size %d ", good_data_count,
                        drive_params->sector_size);
                  print_crc_info(&drive_params->data_crc, MSG_ERR_SERIOUS);
                  msg(MSG_ERR_SERIOUS, "Matches %d sector size %d ", previous_good_data_count,
                        sector_size);
                  print_crc_info(&data_crc_info, MSG_ERR_SERIOUS);
               }
               // And keep the best
               if (good_data_count > previous_good_data_count) {
                  data_crc_info = drive_params->data_crc;
                  data_crc_info.ecc_max_span = mfm_all_poly[poly].ecc_span;
                  sector_size = drive_params->sector_size;
                  previous_good_data_count = good_data_count;
               }
               *best_match_count = previous_good_data_count;
            }
         }
      }
   }
   drive_params->sector_size = sector_size;
   drive_params->data_crc = data_crc_info;
   return rc;
}

// Determine the number of heads, sectors per track, and interleave pattern
//
// drive_params: Drive parameters determined so far and return what we have determined
// cyl: cylinder the data is from
// deltas: MFM delta time transition data to analyze (filled after read)

// TODO, only handles drives where the interleave pattern is the same on all tracks
// TODO, this won't work when heads >= 8 for WD1003 which truncates head
// number in header to 3 bits.
// TODO, make it detect when sector overlaps index and adjust begin time
static void analyze_sectors(DRIVE_PARAMS *drive_params, int cyl, void *deltas,
      int max_deltas) {
   int msg_mask_hold;
   // We return a pointer to this so it must be static (may be better to malloc)
   static uint8_t interleave[MAX_SECTORS];
   int unknown_interleave;
   int head_mismatch = 0;
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   int head;
   int found_header;
   int unrecovered_error;
   int max_sector, min_sector, last_good_head;
   int i;
   SECTOR_DECODE_STATUS status;
   int last_min_lba = 0;

   // TODD, this should also detect the WD controllers that support 16 heads but only
    // put 0-7 in the header.
    max_sector = 0;
    min_sector = MAX_SECTORS;
    last_good_head = -1;
    unrecovered_error = 0;
    unknown_interleave = 0;
    memset(interleave, 255, sizeof(interleave));
    for (head = 0; head < MAX_HEAD && !head_mismatch; head++) {
       int found_bad_header;
       int err_count = 0;
       int min_lba = INT_MAX;

       msg_mask_hold = msg_set_err_mask(decode_errors);
       // Try to get a good read. sector_status_list will be the best from
       // all the reads.
       do {
          drive_read_track(drive_params, cyl, head, deltas, max_deltas);

          mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
          status = mfm_decode_track(drive_params, cyl, head, deltas, NULL, sector_status_list);
          if (UNRECOVERED_ERROR(status) && head == 8 && 
                drive_params->controller == CONTROLLER_WD_1006) {
             int good_header = 0;
             for (i = 0; i < drive_params->num_sectors; i++) {
                if ((sector_status_list[i].status & SECT_HEADER_FOUND) &&
                    !(sector_status_list[i].status & SECT_BAD_HEADER)) {
                   good_header = 1;
                }
             }
             // Mightyframe encodes head 8-15 differently. If we don't find
             // any good headers on head 8 see if its a Mightyframe.
             // TODO, Should we stop search after a few bad heads?
             if (!good_header && last_good_head == 7 && head == 8) {
                int orig_controller = drive_params->controller;
                drive_params->controller = CONTROLLER_MIGHTYFRAME;
                status = mfm_decode_track(drive_params, cyl, head, deltas, 
                    NULL, sector_status_list);
                for (i = 0; i < drive_params->num_sectors; i++) {
                   if ((sector_status_list[i].status & SECT_HEADER_FOUND) &&
                       !(sector_status_list[i].status & SECT_BAD_HEADER)) {
                      good_header = 1;
                   }
                }
                if (good_header) {
                   msg(MSG_FORMAT,"Changed controller type to %s\n",
                     mfm_controller_info[drive_params->controller].name);
                } else {
                   drive_params->controller = CONTROLLER_DG_MV2000;
                   status = mfm_decode_track(drive_params, cyl, head, deltas, 
                       NULL, sector_status_list);
                   for (i = 0; i < drive_params->num_sectors; i++) {
                      if ((sector_status_list[i].status & SECT_HEADER_FOUND) &&
                          !(sector_status_list[i].status & SECT_BAD_HEADER)) {
                         good_header = 1;
                      }
                   }
                   if (good_header) {
                      msg(MSG_FORMAT,"Changed controller type to %s\n",
                        mfm_controller_info[drive_params->controller].name);
                   } else {
                      drive_params->controller = orig_controller;
                      status = mfm_decode_track(drive_params, cyl, head, deltas, 
                          NULL, sector_status_list);
                   }
                }
             }
          }
       } while (UNRECOVERED_ERROR(status) && ++err_count < 8);
       msg_set_err_mask(msg_mask_hold);
       if (UNRECOVERED_ERROR(status)) {
          unrecovered_error = 1;
       }
       found_bad_header = 0;
       found_header = 0;
       for (i = 0; i < drive_params->num_sectors; i++) {
          // If we missed a header after finding one, stop looking at
          // interleave since it is likely to be wrong. With
          // read errors we still may get confused.
          if ((sector_status_list[i].status & SECT_BAD_HEADER) && found_header) {
             found_bad_header = 1;
          }
          if (sector_status_list[i].status & SECT_HEADER_FOUND) {
             found_header = 1;
             if (!found_bad_header) {
                if (interleave[sector_status_list[i].logical_sector] != 255 &&
                      interleave[sector_status_list[i].logical_sector] !=
                            sector_status_list[i].sector && !unknown_interleave) {
                   msg(MSG_ERR, "Interleave mismatch previous entry %d, %d was %d now %d\n",
                         i, sector_status_list[i].logical_sector,
                         interleave[sector_status_list[i].logical_sector],
                         sector_status_list[i].sector);
                   unknown_interleave = 1;
                }
                interleave[sector_status_list[i].logical_sector] = sector_status_list[i].sector;
             }
             max_sector = MAX(max_sector, sector_status_list[i].sector);
             min_sector = MIN(min_sector, sector_status_list[i].sector);
             if (mfm_controller_info[drive_params->controller].analyze_type == CINFO_LBA) {
                // If LBA address found is lower than minimum on previous head
                // this head doesn't exist
                if (sector_status_list[i].lba_addr < min_lba) {
                   min_lba = sector_status_list[i].lba_addr;
                }
                if (sector_status_list[i].lba_addr >= last_min_lba) {
                   last_good_head = head;
                } else if (!head_mismatch) {
                   msg(MSG_INFO, "Selected head %d found out of series LBA address, last good head found %d\n",
                         head, last_good_head);
                   head_mismatch = 1;
                }
             } else {
                if (sector_status_list[i].head == head) {
                   last_good_head = head;
                } else {
                   if (!head_mismatch) {
                      msg(MSG_INFO, "Selected head %d found %d, last good head found %d\n",
                         head, sector_status_list[i].head, last_good_head);
                      head_mismatch = 1;
                   }
                }
             }
          }
       }
       last_min_lba = min_lba;
    }
    // If we had a read error but got some good error warn. If nothing readable
    // assume we were trying to read an invalid head
    if (unrecovered_error && found_header) {
       msg(MSG_ERR, "Read errors trying to determine sector numbering, results may be in error\n");
    }
    if (last_good_head == -1) {
       msg(MSG_FATAL, "Unable to determine number of heads\n");
       exit(1);
    }
    // We store number of heads, not maximum head number (starting at 0)
    drive_params->num_head = last_good_head+1;
    drive_params->num_sectors = max_sector - min_sector + 1;
    drive_params->first_sector_number = min_sector;
    msg(MSG_INFO, "Number of heads %d number of sectors %d first sector %d\n",
          drive_params->num_head, drive_params->num_sectors,
          drive_params->first_sector_number);

    if (unknown_interleave) {
       msg(MSG_ERR, "Unable to determine interleave. Interleave value is not required\n");
       drive_params->sector_numbers = NULL;
    } else {
       msg(MSG_INFO, "Interleave (not checked):");
       for (i = 0; i < drive_params->num_sectors; i++) {
          msg(MSG_INFO, " %d",interleave[i]);
       }
       msg(MSG_INFO, "\n");
       // Too many drives have cylinders with different interleave (spare,
       // for testing) that cause confusing errors so checking is disabled
       // unless users specified directly
       //drive_params->sector_numbers = interleave;
       drive_params->sector_numbers = NULL;
    }
}

// Test if seek at specified rate works. We 
//
// drive_params: Drive parameters determined so far.
// return: 0 if seek at specified speed worked, 1 otherwise
static int analyze_seek(DRIVE_PARAMS *drive_params) {
   int seek;
   int rc = 0;
   int i;

   drive_seek_track0();
   seek = 30;
   // Step down the drive at the specified rate then verify
   // it takes the same number of slow steps to return back to
   // track 0.
   drive_step(drive_params->step_speed, seek, 
           DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
   // Give time for track0 to change before we check
   usleep(1000);
   if (drive_at_track0()) {
      msg(MSG_INFO, "Drive still at track 0 after seek\n");
      rc = 1;
   } else {
      for (i = 1; i <= seek && rc == 0; i++) {
         drive_step(DRIVE_STEP_SLOW, -1, 
            DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_FATAL_ERR);
         if (i == seek) {
            // Give time for track0 to change before we check
            usleep(1000);
            if (!drive_at_track0()) {
               msg(MSG_INFO, "Drive didn't reach track 0 testing %s seek\n",
                  step_speed_text(drive_params->step_speed));
               rc = 1;
            }
         } else {
            if (drive_at_track0()) {
               msg(MSG_INFO, "Drive prematurely at track 0 after %d of %d steps testing %s seek\n", 
                  i, seek, step_speed_text(drive_params->step_speed));
               rc = 1;
            }
         }
      }
      drive_seek_track0();
   }

   return rc;
}

// Find the number of cylinders the disk has. We step down the disk until we
// find unreadable tracks, the cylinder number in header doesn't match, or
// seek times out. Various disks have different behavior. The tracks past the end
// leading to the park zone are normally not formated. Many drives won't let you
// seek past the last cylinder. They either return to track zero or perform a
// recalibrate which may take enough time for the seek to timeout. Really old
// disk will just hit the end stop so can get the same cylinder. Hitting end stop
// probably isn't great.
//
// drive_params: Drive parameters determined so far and return what we have determined
// start_cyl: cylinder head is on at entry
// head: head currently selected
// deltas: MFM delta time transition data to analyze (filled after read)
//
// TODO, this won't deal well with spared cylinders when they have nonsequential
// values
static void analyze_disk_size(DRIVE_PARAMS *drive_params, int start_cyl,
      int head, void *deltas, int max_deltas) {
   int msg_mask_hold;
   int max_cyl;
   int not_next_cyl, not_next_cyl_count;
   int any_header_found;
   int no_header_count;
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   int cyl;
   int i;
   int last_printed_cyl;
   int rc;

   max_cyl = 0;
   no_header_count = 0;
   not_next_cyl_count = 0;

   for (cyl = start_cyl + 1; cyl < MAX_CYL; cyl++) {
      if (cyl % 5 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);

      rc = drive_step(drive_params->step_speed, 1, 
           DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_RET_ERR);
      if (rc == DRIVE_STEP_TIMEOUT) {
         msg(MSG_INFO, "Max cylinder set from drive timeout on seek\n");
         break;
      } else if (rc == DRIVE_STEP_RECAL) {
         msg(MSG_INFO, "Stopping end of disk search due to recalibration\n");
         break;
      }
      drive_read_track(drive_params, cyl, head, deltas, max_deltas);

      mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
      msg_mask_hold = msg_set_err_mask(decode_errors);
      mfm_decode_track(drive_params, cyl, head, deltas, NULL, sector_status_list);
      msg_set_err_mask(msg_mask_hold);
      any_header_found = 0;
      not_next_cyl = 0;
      last_printed_cyl = -1;
      for (i = 0; i < drive_params->num_sectors; i++) {
         if (sector_status_list[i].status & SECT_HEADER_FOUND) {
            any_header_found = 1;
            max_cyl = MAX(max_cyl, sector_status_list[i].cyl);
            if (cyl != sector_status_list[i].cyl && last_printed_cyl != cyl) {
               msg(MSG_INFO, "Found cylinder %d expected %d\n",sector_status_list[i].cyl, cyl);
               last_printed_cyl = cyl;
               not_next_cyl = 1;
            }
         }
      }
      if (!any_header_found) {
         if (++no_header_count >= 2) {
            msg(MSG_INFO, "Stopping end of disk search due to two unreadable tracks in a row\n");
            break;
         } else {
            msg(MSG_INFO, "No sectors readable from cylinder %d\n",cyl);
         }
      } else {
         no_header_count = 0;
      }
      if (not_next_cyl) {
         if (++not_next_cyl_count >= 2) {
            msg(MSG_INFO, "Stopping end of disk search due to mismatching cylinder count\n");
            break;
         }
      } else {
         not_next_cyl_count = 0;
      }
   }
   drive_params->num_cyl = max_cyl + 1;
   msg(MSG_INFO, "Number of cylinders %d, %.1f MB\n", drive_params->num_cyl,
         (double) drive_params->num_cyl * drive_params->num_head *
         drive_params->num_sectors * drive_params->sector_size / (1000.0*1000));
   // We don't know where head is so return to 0
   drive_seek_track0();
}

// Try to identify format of header
// Head will be left at specified cylinder on return
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
// max_delta: Size of deltas array
// cyl: cylinder to test
// head: head to test
// return: Number of matching formats found
int analyze_headers(DRIVE_PARAMS *drive_params, void *deltas, 
   int max_deltas, int cyl, int head)
{
   int headers_match;
   DRIVE_PARAMS drive_params_list[20];
   int match_count[ARRAYSIZE(drive_params_list)];
   int i;
   int max_match = 0, max_match_index = -1;
   int format_count = 0;
   int data_matches;

   headers_match = analyze_header(drive_params, cyl, head, deltas, max_deltas, 
      drive_params_list, ARRAYSIZE(drive_params_list), match_count);
   if (headers_match != 0) {
      // If drive has separate data area check it
      for (i = 0; i < headers_match; i++) {
         if (mfm_controller_info[drive_params_list[i].controller].separate_data) {
            if (analyze_data(&drive_params_list[i], cyl, head, deltas, 
                  max_deltas, match_count[i], &data_matches) > 1) {
               format_count++;
            }
         } else {
             data_matches = match_count[i];
         }
         if (data_matches > 0) {
            // Pick based on number of header and data matches
            data_matches += match_count[i];
            format_count++;
            if (max_match > 0) {
               msg(MSG_ERR_SERIOUS, "Found multiple matching header parameters. Will use largest matches or last if identical\n");
            }
            msg(MSG_ERR_SERIOUS, "Matches count %d for controller %s\nHeader CRC: ", data_matches,
                mfm_controller_info[drive_params_list[i].controller].name);
            print_crc_info(&drive_params_list[i].header_crc, MSG_ERR_SERIOUS);
            if (mfm_controller_info[drive_params_list[i].controller].separate_data) {
               msg(MSG_ERR_SERIOUS, "Sector length %d\nData CRC: ", 
                   drive_params_list[i].sector_size);
               print_crc_info(&drive_params_list[i].data_crc, MSG_ERR_SERIOUS);
            } else {
               msg(MSG_ERR_SERIOUS, "Sector length %d\n", 
                   drive_params_list[i].sector_size);
            }
            // Use sum of data & header match count
            if (data_matches >= max_match) {
               max_match = data_matches;
               max_match_index = i;
            }
         }
      }
   }

   if (max_match_index >= 0) {
      *drive_params = drive_params_list[max_match_index];
   }
   return format_count;
}

// Try to identify format of drive
// Head will be left at specified cylinder on return
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
// max_delta: Size of deltas array
// cyl: cylinder to test
// head: head to test
// return: Number of matching formats found
int analyze_format(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas,
   int cyl, int head)
{
   int rc;


   rc = analyze_model(drive_params, cyl, head, deltas, max_deltas);
   // More than one format found, try different track
   // TODO: This should be != 0 after model is primary method to detect.
   if (rc > 1) {
      // If non zero cylinder retry on zero. If drive has stuck heads this
      // may allow format to be detected
      if (cyl > 0) {
         cyl = 0;
      } else {
         cyl = cyl + 1;
      }
      head = 1;
      msg(MSG_INFO,"Retrying on cylinder %d head %d\n", cyl, head);
      rc = analyze_model(drive_params, cyl, head, deltas, max_deltas);
   }
   if (rc > 1) {
      msg(MSG_ERR, "Multiple matching formats found, using first\n");
   }
   if (rc >= 1) {
      DRIVE_PARAMS drive_params_hold;
      // FIXME: This may do more than we want. We only need to determine
      // number of heads. For now if it doesn't match we fall back to normal
      // format detection.
      analyze_sectors(drive_params, cyl, deltas, max_deltas);
      drive_params_hold = *drive_params;
      parse_set_drive_params_from_controller(drive_params, drive_params->controller);
      if (drive_params_hold.num_sectors != drive_params->num_sectors ||
        drive_params_hold.first_sector_number != drive_params->first_sector_number ||
       drive_params_hold.controller != drive_params->controller) {
         msg(MSG_INFO, "Sector information detected doesn't match expected format, trying again\n");
      } else {
         return rc;
      }
   }

   rc = analyze_headers(drive_params, deltas, max_deltas, cyl, head);
     // For the ST11M only the first two heads are used on the first
     // cylinder. Retry on the next to get proper number of heads.
   if (rc != 1 || (drive_params->controller == CONTROLLER_SEAGATE_ST11M &&
         cyl == 0)) {
      if (cyl != 0 && drive_at_track0()) {
         msg(MSG_ERR, "Drive indicating track 0 when should be on cylinder %d\n",
            cyl);
      } else if (cyl == 0 && !drive_at_track0()) {
         msg(MSG_ERR, "Drive not indicating track 0 when should be on cylinder %d\n",
            cyl);
      }
      // If non zero cylinder retry on zero. If drive has stuck heads this
      // may allow format to be detected
      if (cyl > 0) {
         cyl = 0;
      } else {
         cyl = cyl + 1;
      }
      head = 1;
      msg(MSG_INFO,"Retrying on cylinder %d head %d\n", cyl, head);
      // If we either got no valid information or multiple possible values try
      // cyl 1 head 1. Cyl 0 head 0 is poor for distinguishing different header
      // formats. Also may help if first track has too many read errors.
      // Analyzing track 0 is useful for detecting the weird multiple formats
      // for the DEC RQDX3 so it is tested first.
      rc = analyze_headers(drive_params, deltas, max_deltas, cyl, head);
      if (cyl != 0 && drive_at_track0()) {
         msg(MSG_ERR, "Drive indicating track 0 when should be on cylinder %d\n",
            cyl);
      } else if (cyl == 0 && !drive_at_track0()) {
         msg(MSG_ERR, "Drive not indicating track 0 when should be on cylinder %d\n",
            cyl);
      }
   }
   // Don't check sectors if we didn't figure out format
   if (rc != 0) {
      analyze_sectors(drive_params, cyl, deltas, max_deltas);
   }

   return rc;
}

// Routine to determine the various drive parameters. See individual routines
// for details on operation.
//
// drive_params: Return parameters we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
void analyze_disk(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas,
      int use_file)
{
   int ready = 0;
   int i;
   int cyl, head;
   DRIVE_PARAMS drive_params_hold;

   cyl = drive_params->analyze_cyl;
   head = drive_params->analyze_head;
   // Turn off errors from MFM routines while analyzing. Too many
   // messages otherwise.
   decode_errors = MSG_FATAL | MSG_FORMAT | (msg_get_err_mask() & MSG_DEBUG);
   // Turn on all errors to see why decoding fails
   //decode_errors = ~1;
   //decode_errors = ~0;

   // Save off parameters so we can put back ones we changed as side
   // effects of the analysis.
   drive_params_hold = *drive_params;
   // We don't want to write the analysis information to files
   drive_params->extract_filename = NULL;
   drive_params->transitions_filename = NULL;
   // We will analyze this later
   drive_params->sector_numbers = NULL;
   // Ignore header errors. Was used during testing.
   //drive_params->ignore_header_mismatch = 1;
   drive_params->analyze_in_progress = 1;

   if (!use_file) {
      // Step through all the head selects to find the drive
      for (i = 1; i <= 4 && !ready; i++) {
         int wait_count = 0;
         drive_params->drive = i;
         drive_select(drive_params->drive);

         // Wait up to 100 ms for drive to be selected
         while (wait_count++ < 100 && !ready) {
            usleep(1000);
            // Bit is active low
            ready = !(drive_get_drive_status() & BIT_MASK(R31_DRIVE_SEL));
         }
      }
      if (ready) {
         msg(MSG_INFO,"Found drive at select %d\n",drive_params->drive);
      } else {
         msg(MSG_INFO,"Unable to find drive. If just powered on retry in 30 seconds\n");
         exit(1);
      }
      drive_setup(drive_params);
      msg(MSG_INFO, "Drive RPM %.1f\n", drive_rpm());

      // We don't want to write transitions to file so pass null
      deltas_start_thread(NULL);

   }
   // For header and field analysis we read one track and reanalyze the same data
   mfm_decode_setup(drive_params, 0);

   if (analyze_format(drive_params, deltas, max_deltas, cyl, head) == 0) {
      msg(MSG_ERR, "Unable to determine drive format\n");
      exit(1);
   }
   if (!use_file) {
      // Try a buffered seek and if it doesn't work try an unbuffered seek
      drive_params->step_speed = DRIVE_STEP_FAST;
      if (analyze_seek(drive_params) != 0) {
         drive_params->step_speed = DRIVE_STEP_SLOW;
         if (analyze_seek(drive_params) != 0) {
            msg(MSG_FATAL, "Drive is not seeking properly\n");
            exit(1);
         }
      }
      if (drive_params->step_speed == DRIVE_STEP_FAST) {
         msg(MSG_INFO, "Drive supports buffered seeks (ST412)\n");
      } else {
         msg(MSG_INFO, "Drive doesn't support buffered seeks (ST506)\n");
      }

      analyze_disk_size(drive_params, cyl, head, deltas, max_deltas);
   } else {
      if (drive_params->tran_fd != -1) {
         drive_params->num_cyl = drive_params->tran_file_info->num_cyl;
      } else {
         drive_params->num_cyl = drive_params->emu_file_info->num_cyl;
      }
   }
   // And restore items we changed that aren't determined by the analysis
   drive_params->ignore_header_mismatch = drive_params_hold.ignore_header_mismatch;
   drive_params->extract_filename = drive_params_hold.extract_filename;
   drive_params->transitions_filename = drive_params_hold.transitions_filename;

   if (!use_file) {
      deltas_stop_thread();
   }
   drive_params->analyze_in_progress = 0;
}
