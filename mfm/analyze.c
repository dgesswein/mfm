// This module analyzes the disk format
// Call analyze_disk to perform the analysis
//
// We perform the analysis by trying the various formats we know of until we
// find one that matches and other tests. See the routines for details.

// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
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

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "deltas_read.h"
#include "drive.h"
#include "analyze.h"

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

   drive_read_track(drive_params, cyl, head, deltas, max_deltas);

   drive_params->num_sectors = MAX_SECTORS;
   drive_params->num_head = MAX_HEAD;
   // Don't use ECC while trying to find format
   drive_params->header_crc.ecc_max_span = 0;
   drive_params->data_crc.ecc_max_span = 0;

   // Try an exhaustive search of all the formats we know about. If we get too
   // many we may have to try something smarter.
   for (cont = 0; mfm_controller_info[cont].name != NULL; cont++) {
      if (mfm_controller_info[cont].analyze_type != CINFO_CHS) {
         continue; // ****
      }
      // Make sure these get set at bottom for final controller picked
      // Sector size will be set when we analyze the data header
      drive_params->controller = cont;
      drive_params->sector_size = 
         mfm_controller_info[cont].analyze_sector_size;
      drive_params->start_time_ns = 
         mfm_controller_info[cont].start_time_ns;
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
            msg(MSG_DEBUG, "Trying controller %s ", 
                        mfm_controller_info[drive_params->controller].name);
                  print_crc_info(&drive_params->header_crc, MSG_DEBUG);
            drive_params->header_crc.init_value = trim_value(mfm_all_init[init].value,
               drive_params->header_crc.length);
            drive_params->data_crc = drive_params->header_crc;
            
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
            for (i = 0; i < drive_params->num_sectors; i++) {
               if (!(sector_status_list[i].status & SECT_BAD_HEADER)) {
                  good_header_count++;
               }
            }
            // If we found at least 2 sectors
            if (good_header_count >= 2) {
               // Keep the best
               if (good_header_count > previous_good_header_count) {
                  controller_type = drive_params->controller;
                  previous_good_header_count = good_header_count;
               }
               if (drive_params_list_index >= drive_params_list_len) {
                  msg(MSG_FATAL, "Too many header formats found %s\n", 
                    drive_params_list_index);
                  exit(1);
               }
               // Save in list
               drive_params_list[drive_params_list_index] =
                 *drive_params;
               drive_params_list[drive_params_list_index].header_crc.ecc_max_span 
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
// return: Number of matching formats found.
static int analyze_data(DRIVE_PARAMS *drive_params, int cyl, int head, void *deltas, int max_deltas, int *best_match_count)
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
            // zeros after CRC match will still match
            if (sector_size != 0 && drive_params->sector_size > sector_size) {
               continue;
            }
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
               if (!UNRECOVERED_ERROR(sector_status_list[i].status)) {
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
             if (!good_header && head == 8) {
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
                   drive_params->controller = CONTROLLER_WD_1006;
                   status = mfm_decode_track(drive_params, cyl, head, deltas, 
                       NULL, sector_status_list);
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

// Determine what seek method should be used for drive
//
// drive_params: Drive parameters determined so far and return what we have determined
// cyl: cylinder the data is from
// deltas: MFM delta time transition data to analyze (filled after read)
static int analyze_seek(DRIVE_PARAMS *drive_params, int start_cyl, int head, void *deltas,
      int max_deltas) {
   int msg_mask_hold;
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   int i;
   int seek;
   int found_header;
   char *seek_desc;
   int rc = 0;

   if (drive_params->step_speed == DRIVE_STEP_FAST) {
      seek_desc = "buffered";
   } else {
      seek_desc = "unbuffered";
   }
   // We try using the fast seek command to move down the disk and see if
   // we end up on the correct cylinder. If we start far enough down the
   // disk seek towards 0 instead to make sure we don't hit end of disk
   seek = 30;
   if (start_cyl > seek) {
      seek = -seek;
   }
   start_cyl += seek;
   drive_read_track(drive_params, start_cyl, head, deltas, max_deltas);

   mfm_init_sector_status_list(sector_status_list, drive_params->num_sectors);
   msg_mask_hold = msg_set_err_mask(decode_errors);
   mfm_decode_track(drive_params, start_cyl, head, deltas, NULL, 
         sector_status_list);
   msg_set_err_mask(msg_mask_hold);
   found_header = 0;
   for (i = 0; i < drive_params->num_sectors; i++) {
      if (sector_status_list[i].status & SECT_HEADER_FOUND) {
         found_header = 1;
         if (sector_status_list[i].cyl != start_cyl) {
            msg(MSG_INFO,"Seek test expected cyl %d got %d %s seek\n",
               start_cyl, sector_status_list[i].cyl, seek_desc);
            rc = 1;
         }
      }
   }
   if (!found_header) {
      msg(MSG_INFO,"No readable data after %s seek test\n", seek_desc);
      rc = 1;
   }
   if (rc != 0) {
         // We don't know where head is so return to 0
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

   max_cyl = 0;
   no_header_count = 0;
   not_next_cyl_count = 0;

   for (cyl = start_cyl + 1; cyl < MAX_CYL; cyl++) {
      if (cyl % 5 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);

      if (drive_step(drive_params->step_speed, 1, 
           DRIVE_STEP_UPDATE_CYL, DRIVE_STEP_RET_ERR) != 0) {
         msg(MSG_INFO, "Max cylinder set from drive timeout on seek\n");
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

// Try to identify format of header of drives in cylinder head sector
// format (CHS).
// Head will be left at specified cylinder on return
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
// max_delta: Size of deltas array
// cyl: cylinder to test
// head: head to test
// return: Number of matching formats found
int analyze_chs_headers(DRIVE_PARAMS *drive_params, void *deltas, 
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
                  max_deltas, &data_matches) > 1) {
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
               printf("Sector length %d\nData CRC: ", 
                   drive_params_list[i].sector_size);
               print_crc_info(&drive_params_list[i].data_crc, MSG_ERR_SERIOUS);
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

// Try to identify format of drives in cylinder head sector format (CHS).
// Head will be left at specified cylinder on return
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
// max_delta: Size of deltas array
// cyl: cylinder to test
// head: head to test
// return: Number of matching formats found
int analyze_chs(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas,
   int cyl, int head)
{
   int rc;

   rc = analyze_chs_headers(drive_params, deltas, max_deltas, cyl, head);
   if (rc != 1) {
      cyl = cyl + 1;
      head = 1;
      msg(MSG_INFO,"Retrying on cylinder %d head %d\n", cyl, head);
      // If we either got no valid information or multiple possible values try
      // cyl 1 head 1. Cyl 0 head 0 is poor for distinguishing different header
      // formats. Also may help if first track has too many read errors.
      // Analyzing track 0 is useful for detecting the weird multiple formats
      // for the DEC RQDX3 so it is tested first.
      rc = analyze_chs_headers(drive_params, deltas, max_deltas, cyl, head);
   }
   analyze_sectors(drive_params, cyl, deltas, max_deltas);

   return rc;
}

// Try to identify format of drives with headers in logical block address
// format (LBA).
// Head will be left at specified cylinder on return
//
// drive_params: Drive parameters determined so far and return what we have determined
// deltas: MFM delta time transition data to analyze (filled after read)
// max_delta: Size of deltas array
// cyl: cylinder to test
// head: head to test
// return: Number of formats found
int analyze_lba(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas,
   int cyl, int head)
{
   int rc = 0;
   // Loop variables
   int poly, init, cont;
   int nsec_ndx;
   int i;
   // The best match CRC and controller type info so far
   CRC_INFO header_crc_info;
   int controller_type = -1;
   // Numbers of good sectors found
   int good_header_count, previous_good_header_count = 0;
   // Variable to restore global error print mask
   int msg_mask_hold;
   // The status of each sector decoded.
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   // And read status
   SECTOR_DECODE_STATUS status;
   int best_heads = 0;
   int best_sectors = 0;

#if 0
   decode_errors = ~1;
   msg_set_err_mask(decode_errors);
#endif

   header_crc_info.poly = 0;
   drive_params->num_sectors = MAX_SECTORS;
   drive_params->num_head = MAX_HEAD;
   // Don't use ECC while trying to find format
   drive_params->header_crc.ecc_max_span = 0;
   drive_params->data_crc.ecc_max_span = 0;
   drive_params->first_sector_number = 0;

   drive_read_track(drive_params, cyl, head, deltas, max_deltas);

   // Try an exhaustive search of all the formats we know about. If we get too
   // many we may have to try something smarter.
   for (cont = 0; mfm_controller_info[cont].name != NULL; cont++) {
      if (mfm_controller_info[cont].analyze_type != CINFO_LBA) {
         continue; // ****
      }
      // Make sure these get set at bottom for final controller picked
      // Sector size will be set when we analyze the data header
      drive_params->controller = cont;
      drive_params->sector_size = 
         mfm_controller_info[cont].analyze_sector_size;
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
            for (drive_params->num_head = 2; drive_params->num_head < 16; 
                  drive_params->num_head++) {
               for (nsec_ndx = 0; mfm_lba_num_sectors[nsec_ndx] != -1; nsec_ndx++) {
                  drive_params->num_sectors = mfm_lba_num_sectors[nsec_ndx];

                  msg(MSG_DEBUG, "Trying controller %s heads %d sectors %d\n",
                        mfm_controller_info[drive_params->controller].name,
                        drive_params->num_head, drive_params->num_sectors);
                  print_crc_info(&drive_params->header_crc, MSG_DEBUG);
                  drive_params->header_crc.init_value = trim_value(mfm_all_init[init].value,
                        drive_params->header_crc.length);
                  drive_params->data_crc = drive_params->header_crc;

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
                  for (i = 0; i < drive_params->num_sectors; i++) {
                     if (!(sector_status_list[i].status & SECT_BAD_HEADER)) {
                        good_header_count++;
                     }
                  }
                  // If we found at least 2 sectors
                  if (good_header_count >= 2) {
                     rc++;
                     // If we have a previous match print both
                     if (header_crc_info.poly != 0) {
                        msg(MSG_ERR_SERIOUS, "Found multiple matching header parameters. Will use largest matches:\n");
                        msg(MSG_ERR_SERIOUS, "Matches %d controller %s ", good_header_count,
                              mfm_controller_info[drive_params->controller].name);
                        print_crc_info(&drive_params->header_crc, MSG_ERR_SERIOUS);
                        msg(MSG_ERR_SERIOUS, "Matches %d controller %s ", previous_good_header_count,
                              mfm_controller_info[controller_type].name);
                        print_crc_info(&header_crc_info, MSG_ERR_SERIOUS);
                     }
                     // And keep the best
                     if (good_header_count > previous_good_header_count) {
                        best_heads = drive_params->num_head;
                        best_sectors = drive_params->num_sectors;
                        header_crc_info = drive_params->header_crc;
                        header_crc_info.ecc_max_span = mfm_all_poly[poly].ecc_span;
                        controller_type = drive_params->controller;
                        previous_good_header_count = good_header_count;
                     }
                  }
               }
            }
         }
      }
   }
   // Set for final controller we picked. We should be able to identify
   // the header format without start_time_ns properly set. We
   // Won't get all the sectors until we do so we set it here.
   drive_params->start_time_ns = 
      mfm_controller_info[controller_type].start_time_ns;
   drive_params->sector_size = 
      mfm_controller_info[controller_type].analyze_sector_size;
   drive_params->num_head = best_heads;
   drive_params->num_sectors = best_sectors; 
   // Print what we found and put it in drive_params
   if (header_crc_info.poly != 0) {
      msg(MSG_INFO, "Header CRC Information:\n");
      print_crc_info(&header_crc_info, MSG_INFO);
      msg(MSG_INFO, "Controller type %s\n", 
         mfm_controller_info[controller_type].name);
      msg(MSG_INFO, "Number of sectors %d number of heads %d\n", 
         drive_params->num_sectors, drive_params->num_head);
   } else {
      // This is fatal since if we can't continue to next analysis
      msg(MSG_FATAL, "Unable to determine CRC & Controller type\n");
   }
   drive_params->header_crc = header_crc_info;
   drive_params->controller = controller_type;
   if (rc != 0) {
      int match_count;
      rc = analyze_data(drive_params, cyl, head, deltas, max_deltas, 
         &match_count);
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

   if (analyze_chs(drive_params, deltas, max_deltas, cyl, head) == 0) {
      // Cyl 0 head 0 won't work for detecting format
      if (cyl == 0 && head == 0) {
         // Need to go enough cylinders so that 32/33 sectors will
         // mismatch head when we use the wrong one
         cyl = 9;
         head = 1;
      }
      msg(MSG_INFO,"Trying LBA controllers on cylinder %d and head %d\n",
          cyl, head);
      if (analyze_lba(drive_params, deltas, max_deltas, cyl, head) == 0) {
         exit(1);
      }
   }
   if (!use_file) {
      // Try a buffered seek and if it doesn't work try an unbuffered seek
      drive_params->step_speed = DRIVE_STEP_FAST;
      if (analyze_seek(drive_params, cyl, head, deltas, max_deltas) != 0) {
         drive_params->step_speed = DRIVE_STEP_SLOW;
         if (analyze_seek(drive_params, cyl, head, deltas, max_deltas) != 0) {
            msg(MSG_FATAL, "Unable to get valid data after a seek\n");
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
}
