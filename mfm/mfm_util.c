// This is a utility program to process existing MFM delta transition data.
// Used to extract the sector contents to a file
//
// 12/19/21 DJG Code cleanup
// 01/18/21 DJG Only print valid formats for ext2emu
// 10/08/20 DJG Added OP_XOR for sector data. Changed a couple of error messages
// 12/31/19 DJG Allow additional special encoded bytes
// 08/23/19 DJG Fixed typo and print format.
// 07/19/19 DJG Added ext2emu support for Xerox 8010
// 03/12/19 DJG Call parameter change
// 02/09/19 DJG Added CONTROLLER_SAGA_FOX support
// 01/20/18 DJG Set length of A1 list to MAX_SECTORS*2 to prevent overflow.
//    Minor code changes.
// 10/21/18 DJG Don't allow --begin_time to be changed from value in
//    input file. It doesn't do anything and makes calculation of
//    begin_time when last sector truncated wrong.
// 12/25/17 DJG Made progress indicator only print same value once
// 12/17/17 DJG Added progress indicator
// 10/16/16 DJG Fixed spelling error
// 01/24/16 DJG Fix for ext2emu when sectors start at 1
// 01/06/16 DJG Rename structure
// 01/02/16 DJG Fix ext2emu for Northstar
// 01/01/16 DJG Add --mark_bad support
// 12/31/15 DJG Added ext2emu functionality
// 07/30/15 DJG Added support for revision B board.
// 05/17/15 DJG Added ability to analyze transition and emulation files.
//              Added ability to analyze specific cylinder and head.
//
// 10/24/14 DJG Changes necessary due to addition of mfm_emu write buffer
//     using decode options stored in header, and minor reformatting
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
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <errno.h>
#include <libiberty.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#define DEF_DATA
#include "mfm_decoder.h"
#include "parse_cmdline.h"
#include "analyze.h"
#include "deltas_read.h"  // Code is from deltas_read_file.c
#include "board.h"

#define MAX_DELTAS 131072

// Location that needs special encoding such as A1 marking header
typedef struct {
   int index;
   uint16_t pattern;
} SPECIAL_LIST;

void ext2emu(int argc, char *argv[]);

// Main routine
int main (int argc, char *argv[])
{
   uint16_t deltas[MAX_DELTAS];
   int num_deltas;
   int cyl, head;
   int last_cyl = -1, last_head = -1;
   DRIVE_PARAMS drive_params;
   // This is in order of physcial sector. First entry is the first
   // physical sector even if the pysical sectors are numbered from 1.
   SECTOR_STATUS sector_status_list[MAX_SECTORS];
   int seek_difference;
   //SECTOR_DECODE_STATUS status;
   int transition_file = 1;
   EMU_FILE_INFO emu_file_info;
   TRAN_FILE_INFO tran_file_info;

   // Handle extracted data to emulator file conversion
   if (strcmp(basename(argv[0]),"ext2emu") == 0) {
      ext2emu(argc, argv);
      return 0;
   }

   // If they specified a transitions or emulation file get options that 
   // were stored in it.
   parse_cmdline(argc, argv, &drive_params, "tm", 1, 1, 1, 0);
   if (drive_params.transitions_filename != NULL ||
         drive_params.emulation_filename != NULL) {
      char **tran_argv, **targs;
      int tran_argc = 0;
      char *orig_cmdline;
      char *orig_note;
      char *cmdline;

      if (drive_params.transitions_filename != NULL) {
         drive_params.tran_fd = tran_file_read_header(
            drive_params.transitions_filename, &tran_file_info);
         drive_params.start_time_ns = tran_file_info.start_time_ns;
         drive_params.tran_file_info = &tran_file_info;
         orig_cmdline = tran_file_info.decode_cmdline;
         orig_note = tran_file_info.note;
      } else {
         drive_params.emu_fd = emu_file_read_header(
            drive_params.emulation_filename, &emu_file_info, 0);
         drive_params.start_time_ns = emu_file_info.start_time_ns;
         drive_params.emu_file_info = &emu_file_info;
         orig_cmdline = emu_file_info.decode_cmdline;
         orig_note = emu_file_info.note;
      }
      // This can't be changed from command line. Use value set from input
      // file
      drive_params.dont_change_start_time = 1;

      if (orig_cmdline != NULL) {
         msg(MSG_INFO, "Original decode arguments: %s\n", orig_cmdline);
         if (orig_note != NULL && strlen(orig_note) != 0) {
            msg(MSG_INFO, "Note: %s\n", orig_note);
         }
         // We need first argument of new argv to be program name argv[0]
         cmdline = msg_malloc(strlen(orig_cmdline) + strlen(argv[0]) + 1, 
            "main cmdline");
         strcpy(cmdline, argv[0]);
         strcat(cmdline, " ");
         strcat(cmdline, orig_cmdline);

         tran_argv = buildargv(cmdline);
         for (targs = tran_argv; *targs != NULL; targs++) {
            tran_argc++;
         }
         // Allow the arj options from stored command line, we will ignore
         // them in drive_params. Don't make errors fatal in case bad
         // option gets in header
         parse_cmdline(tran_argc, tran_argv, &drive_params, "", 0, 0, 1, 0);
       
         free(cmdline);
      }
   }

   // Now parse the full command line. This allows overriding options that
   // were in the transition file header.
   parse_cmdline(argc, argv, &drive_params, "Mrd", 0, 0, 0, 0);
   // Save final parameters
   drive_params.cmdline = parse_print_cmdline(&drive_params, 0, 0);

   // And do some checking
   parse_validate_options(&drive_params, 0);


   if (drive_params.transitions_filename == NULL) {
      if (drive_params.emulation_filename == NULL) {
         msg(MSG_FATAL, "Transition or emulation filename must be specified\n");
         exit(1);
      } else {
         transition_file = 0;
         // Fix up from parse_cmdline. We read emulation file not write it if
         // no transition file is specified
         // Open if it we didn't open above in getting saved command arguments
         drive_params.emulation_output = 0;
         if (drive_params.emu_fd == -1) {
            drive_params.emu_fd = emu_file_read_header(
               drive_params.emulation_filename, &emu_file_info, 0);
         }
      }
   }
   // If analyze requested get drive parameters.
   if (drive_params.analyze && 
         (drive_params.transitions_filename != NULL || 
               drive_params.emulation_filename != NULL)) {
      analyze_disk(&drive_params, deltas, MAX_DELTAS, 1);
      msg(MSG_INFO,"\n");
      // Print analysis results
      parse_print_cmdline(&drive_params, 1, 0);
      // If we are also converting data set up for it. Otherwise exit
      if ((drive_params.transitions_filename != NULL && 
            drive_params.emulation_filename != NULL) ||
            drive_params.extract_filename != NULL) {
         // Go back to beginning of file
         if (drive_params.tran_fd != -1) {
            tran_file_seek_track(drive_params.tran_fd, 0, 0,
                  drive_params.tran_file_info);
         } else {
            emu_file_seek_track(drive_params.emu_fd, 0, 0,
                  drive_params.emu_file_info);
         }
      } else {
         exit(1);
      }
   }

   if (drive_params.transitions_filename != NULL && 
       drive_params.emulation_filename == NULL && 
       drive_params.extract_filename == NULL) {
      msg(MSG_FATAL, "Must specify emulation and or extract file to be generated\n");
      exit(1);
   }
   if (drive_params.transitions_filename == NULL && 
       drive_params.emulation_filename != NULL && 
       drive_params.extract_filename == NULL) {
      msg(MSG_FATAL, "Must specify extract file to be generated\n");
      exit(1);
   }

   // Setup decoding of transitions and possible file to write to
   mfm_decode_setup(&drive_params, 1);

   if (transition_file) {
      num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
            deltas, MAX_DELTAS, &cyl, &head);
   } else {
      num_deltas = emu_file_read_track_deltas(drive_params.emu_fd,
            &emu_file_info, deltas, MAX_DELTAS, &cyl, &head);
   }
   // Read and process a track at a time until all read
   while (num_deltas >= 0) {
      if (cyl % 10 == 0 && head == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);
      // Only clear status if we are moving to the next track. If retries
      // were done we may have multiple reads of the same track.
      if (last_cyl != cyl || last_head != head) {
         mfm_init_sector_status_list(sector_status_list,
               drive_params.num_sectors);
      }
      //printf("Decoding new track %d %d\n",cyl, head);
      deltas_update_count(num_deltas, 0);
      // If head & cylinder haven't changed assume it's a retry.
      mfm_decode_track(&drive_params, cyl, head, deltas,
            &seek_difference, sector_status_list);
#if 0
      if (1 || status != SECT_HEADER_FOUND) {
         if (cyl == 0 && head == 0) {
            FILE *file;
            char fn[256];
            int i;
            sprintf(fn, "trk%d-%d.txt",cyl, head);
            file = fopen(fn, "w");
            for (i = 0; i < num_deltas; i++) {
               fprintf(file, "%d\n",deltas[i]);
            }
            fclose(file);
         }
      }
#endif
      last_cyl = cyl;
      last_head = head;
      if (transition_file) {
         num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
               deltas, MAX_DELTAS, &cyl, &head);
         while (head >= drive_params.num_head) {
            static int msg_printed = 0;
            if (!msg_printed) {
               msg(MSG_INFO, "Warning, data has more heads than specified. Data for head >= %d ignored\n", head);
               msg_printed = 1;
            }
            num_deltas = tran_file_read_track_deltas(drive_params.tran_fd,
               deltas, MAX_DELTAS, &cyl, &head);
         }
      } else {
         num_deltas = emu_file_read_track_deltas(drive_params.emu_fd,
               &emu_file_info, deltas, MAX_DELTAS, &cyl, &head);
      }
   }
   mfm_decode_done(&drive_params);
   return 0;
}


// Reverse bit ordering in word
// value: Value to reverse
// len_bits: Number of bits in value
// return: Reversed value
uint64_t reverse_bits(uint64_t value, int len_bits) {
   uint64_t new_value = 0;
   int i;

   for (i = 0; i < len_bits; i++) {
      new_value = (new_value << 1) | (value & 1);
      value >>= 1;
   } 
   return new_value;
}


// Non zero if the sector number has already been used
static int *sector_used_list;
// Size of sector_used_list in bytes
static int list_size_bytes;
// Number of sector used in the list
static int sector_used_count;
// Sector number to start the next track with
static int track_start_sector;
// Interleave to use on a track, sectors incremented by this value
static int sector_interleave;
// Increment to sector to use for start sector of each track
static int track_interleave;

// Current sector
static int sector;
// Current head and cylinder
static int head, cyl;

// Set the current head value
//
// head_i: Head value
static void set_head(int head_i) {
   head = head_i;
}

// Return the current head value
static int get_head() {
   return head;
}

// Set the current cylinder value
//
// cyl_i: Cylinder value
static void set_cyl(int cyl_i) {
   cyl = cyl_i;
}

// Return the current cylinder value
static int get_cyl() {
   return cyl;
}

// Set the sector interleave values
//
// drive_params: Drive parameters
// sector_interleave_i: Sector interleave. 1 for consecutive sectors
//track_interleave_i: Addition to start sector number for tracks
//   in a cylinder. 0 for all tracks to start with same sector number
static void set_sector_interleave(DRIVE_PARAMS *drive_params,
     int sector_interleave_i, int track_interleave_i) {

   list_size_bytes = sizeof(*sector_used_list) * drive_params->num_sectors; 
   sector_interleave = sector_interleave_i;
   sector_used_list = msg_malloc(list_size_bytes,"sector_used_list");
   track_interleave = track_interleave_i;

   memset(sector_used_list, 0, list_size_bytes);
}

// Update variables to start a new track.
//
// drive_params: Drive parameters
static void start_new_track(DRIVE_PARAMS *drive_params) {
      // Set sector to start sector for list and reset used sector list
   memset(sector_used_list, 0, list_size_bytes);
   sector_used_count = 0;
   if (track_interleave == 0) {
      sector = 0;
   } else {
      sector = track_start_sector;
      track_start_sector = (track_start_sector + track_interleave) %
       drive_params->num_sectors;
   }
}

// Update variables to start a new cylinder. Should be called before
// start_new_track. Cylinders always start with sector 0 first.
//
// drive_params: Drive parameters
static void start_new_cyl(DRIVE_PARAMS *drive_params) {
   track_start_sector = 0;
}

// Get current sector
static int get_sector(DRIVE_PARAMS *drive_params) {
  return sector + drive_params->first_sector_number; 
}

// Increment sector variable
//
// drive_params: Drive parameters
static void inc_sector(DRIVE_PARAMS *drive_params) 
{
      // Mark current sector used and increment by interleave if we
      // haven't generated all sectors.
   sector_used_list[sector] = 1;
   sector_used_count++;
   if (sector_used_count < drive_params->num_sectors) {
      sector = (sector + sector_interleave) % drive_params->num_sectors;
         // If sector is already used find next unused
      while (sector_used_list[sector]) {
         sector = (sector + 1) % drive_params->num_sectors;
      }
   }
}

// Get LBA value for current sector head and cylinder
//
// drive_params: Drive parameters
static int get_lba(DRIVE_PARAMS *drive_params) {
   return (get_cyl() * drive_params->num_head + get_head()) *
       drive_params->num_sectors + sector;
}

// Get check value for end of data
//
// track: Data to calculate over
// length: Number of bytes to over
// crc_info: Parameters for check calculation
// check_type; Type of check value to calculate
static uint64_t get_check_value(uint8_t track[], int length, CRC_INFO *crc_info,
   CHECK_TYPE check_type) {
   uint64_t value;

   if (check_type == CHECK_CRC) {
      value = crc64(track, length, crc_info);
   } else if (check_type == CHECK_CHKSUM) {
      value = checksum64(track, length, crc_info);
      // The length is twice the actual length due to not the best
      // implementation decision when Northstar format was added.
      if (crc_info->length == 16) {
         value = value & 0xff;
      } else if (crc_info->length == 32) {
         value = value & 0xffff;
      } else {
         msg(MSG_FATAL, "Unsupported checksum length %d\n",crc_info->length);
         exit(1);
      }
   } else if (check_type == CHECK_PARITY) {
      value = eparity64(track, length, crc_info);
   } else if (check_type == CHECK_XOR16) {
      int i;
      uint8_t x1 = 0, x2 = 0;

      for (i = 0; i < length; i += 2) {
         x1 ^= track[i];
      }
      for (i = 1; i < length; i += 2) {
         x2 ^= track[i];
      }
      value = (x2 << 8) | x1;
   } else if (check_type == CHECK_NONE) {
      value = 0; // Should be ignored
   } else {
      msg(MSG_FATAL, "Unknown check_type %d\n",check_type);
      exit(1);
   }
   return value;
}

// Get the sector data from the extract data file and put it in the track
// data array
//
// drive_params: Drive parameters
// track: Location to write data read to
// length: Number of bytes in track
static void get_data(DRIVE_PARAMS *drive_params, uint8_t track[], int length) {
   int block;
   int rc;

   if (drive_params->sector_size > length) {
      msg(MSG_FATAL, "Track overflow get_data\n");
      exit(1);
   }
   block = (get_cyl() * drive_params->num_head + get_head()) *
       drive_params->num_sectors + get_sector(drive_params) -
       drive_params->first_sector_number;

   if (lseek(drive_params->ext_fd, 
          block * drive_params->sector_size, SEEK_SET) == -1) {
      msg(MSG_FATAL, "Failed to seek to sector in extracted data file %s\n", 
           strerror(errno));
      exit(1);
   }
   if ((rc = read(drive_params->ext_fd, track, 
         drive_params->sector_size)) != drive_params->sector_size) {
      msg(MSG_FATAL, "Failed to read extracted data file rc %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   };
}

// Get the sector tag from the extract data tag file and put it in the track
// data array
//
// drive_params: Drive parameters
// track: Location to write data read to
// length: Number of bytes in track
static void get_metadata(DRIVE_PARAMS *drive_params, uint8_t track[], int length) {
   int block;
   int rc;

   if (drive_params->metadata_bytes > length) {
      msg(MSG_FATAL, "Track overflow get_data\n");
      exit(1);
   }
   block = (get_cyl() * drive_params->num_head + get_head()) *
       drive_params->num_sectors + get_sector(drive_params) -
       drive_params->first_sector_number;

   if (lseek(drive_params->ext_metadata_fd, 
          block * drive_params->metadata_bytes, SEEK_SET) == -1) {
      msg(MSG_FATAL, "Failed to seek to sector in extracted metadata file %s\n", 
           strerror(errno));
      exit(1);
   }
   if ((rc = read(drive_params->ext_metadata_fd, track, 
         drive_params->metadata_bytes)) != drive_params->metadata_bytes) {
      msg(MSG_FATAL, "Failed to read extracted metadata file rc %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   };
}

// Process field definitions to write the specified data to the track
//
// drive_params: Drive parameters
// full_track: Track data array
// start: Offset into track fields are referenced to
// length: Number of bytes in track
// field_def: The array of field definitions to process
// special_list: List of locations where special a1 code is in track
// special_list_ndx: Next free index in list
// special_list_len: Maximum number of entries in list
//
static void process_field(DRIVE_PARAMS *drive_params, 
   uint8_t full_track[], int start, int length, 
   FIELD_L field_def[], SPECIAL_LIST special_list[], int *special_list_ndx,
   int special_list_len)
{
   int ndx = 0;
   uint64_t value;
   int i;
      // set to 1 if handled in case otherwise code after case updates track
   int data_set;
      // Start and end to calculate crc over
   int crc_start = 0;
   int crc_end = -1;
      // Maximum location filled in field
   int field_filled = 0;
      // Pointer to where we start updating from
   uint8_t *track = &full_track[start];

//printf("Process field called start %d\n",start);

   while (field_def[ndx].len_bytes != -1) {
      data_set = 0;
      switch (field_def[ndx].type) {
            // Fill the specified range with the specified value
         case FIELD_FILL:
            if (field_def[ndx].byte_offset_bit_len +
                field_def[ndx].len_bytes > length) {
               msg(MSG_FATAL, "Track overflow field fill %d, %d, %d\n",
                 field_def[ndx].byte_offset_bit_len, field_def[ndx].len_bytes,
                 length);
               exit(1);
            }
            if (field_def[ndx].op == OP_SET) {
               memset(&track[field_def[ndx].byte_offset_bit_len], 
                  field_def[ndx].value, field_def[ndx].len_bytes);
            } else if (field_def[ndx].op == OP_XOR || 
                  field_def[ndx].op == OP_REVERSE_XOR) {
               for (i = 0; i < field_def[ndx].len_bytes; i++) {
                  track[field_def[ndx].byte_offset_bit_len + i] ^= 
                     field_def[ndx].value;
                }
            } else {
               msg(MSG_FATAL, "op %d not supported for FIELD_FILL\n", 
                  field_def[ndx].op);
               exit(1);
            }
            data_set = 1;
         break;
         case FIELD_CYL:
            value = get_cyl();
         break;
         case FIELD_CYL_SEAGATE_ST11M:
            value = get_cyl();
            // Controller first cylinder and first user cylinder are both 0
            if (value > 0) {
               value--;
            }
         break;
         case FIELD_HEAD:
            value = get_head();
         break;
         case FIELD_HEAD_SEAGATE_ST11M:
            value = get_head();
            if (get_cyl() == 0) {
               value = 0xff;
            }
         break;
         case FIELD_SECTOR:
            value = get_sector(drive_params);
         break;
         case FIELD_LBA:
            value = get_lba(drive_params);
         break;
         case FIELD_HDR_CRC:
            // If end of CRC not specified assume it ends with byte before
            // where CRC goes.
            if (crc_end == -1) {
               crc_end = field_def[ndx].byte_offset_bit_len - 1;
            }
            value = get_check_value(&track[crc_start], crc_end - crc_start + 1,
               &drive_params->header_crc,
               mfm_controller_info[drive_params->controller].header_check);
         break;
         case FIELD_DATA_CRC:
            if (crc_end == -1) {
               crc_end = field_def[ndx].byte_offset_bit_len - 1;
            }
            value = get_check_value(&track[crc_start], crc_end - crc_start + 1,
               &drive_params->data_crc,
               mfm_controller_info[drive_params->controller].data_check);
            if (drive_params->mark_bad_list != NULL) {
               MARK_BAD_INFO *mark_bad_list = 
                    &drive_params->mark_bad_list[drive_params->next_mark_bad];
               if (get_cyl() == mark_bad_list->cyl &&
                    get_head() == mark_bad_list->head &&  
                    get_sector(drive_params) == mark_bad_list->sector) {
                  // Invert the check value to invalidiate
                  value = ~value;
                  if (!mark_bad_list->last) {
                     drive_params->next_mark_bad++;
                  }
               }
            }
         break;
         case FIELD_MARK_CRC_START:
            crc_start = field_def[ndx].byte_offset_bit_len;
            data_set = 1;
         break;
         case FIELD_MARK_CRC_END:
            crc_end = field_def[ndx].byte_offset_bit_len;
            data_set = 1;
         break;
         case FIELD_SECTOR_DATA:
            if (field_def[ndx].len_bytes != drive_params->sector_size) {
               msg(MSG_FATAL, "Sector length mismatch %d %d\n",
                   field_def[ndx].len_bytes, drive_params->sector_size);
               exit(1);
            }
            if (field_def[ndx].op == OP_XOR) {
               uint8_t buf[length - field_def[ndx].byte_offset_bit_len];
               int start = field_def[ndx].byte_offset_bit_len;
               get_data(drive_params, buf, length - start);
               for (i = start; i < start + field_def[ndx].len_bytes; i++) {
                  track[i] ^= buf[i - start];
               }
            } else {
               get_data(drive_params, &track[field_def[ndx].byte_offset_bit_len],
                  length - field_def[ndx].byte_offset_bit_len);
            }
            if (field_def[ndx].op == OP_REVERSE) {
               int start = field_def[ndx].byte_offset_bit_len;
               for (i = start; i < start + field_def[ndx].len_bytes; i++) {
                  track[i] = reverse_bits(track[i], 8);
               }
            }
            data_set = 1;
         break;
         case FIELD_SECTOR_METADATA:
            if (field_def[ndx].len_bytes != drive_params->metadata_bytes) {
               msg(MSG_FATAL, "Sector length mismatch metadata %d %d\n",
                  field_def[ndx].len_bytes, drive_params->metadata_bytes);
               exit(1);
            }
            get_metadata(drive_params, &track[field_def[ndx].byte_offset_bit_len],
               length - field_def[ndx].byte_offset_bit_len);
            if (field_def[ndx].op == OP_REVERSE) {
               int start = field_def[ndx].byte_offset_bit_len;
               for (i = start; i < start + field_def[ndx].len_bytes; i++) {
                  track[i] = reverse_bits(track[i], 8);
               }
            }
            data_set = 1;
         break;
            // Place holder for code to handle marking a sector
         case FIELD_BAD_SECTOR:
            value = 0;
         break;
            // Special A1 with missing clock. We put a1 in the data and fix the
            // encoded MFM data curing the conversion
         case FIELD_A1:
            if (*special_list_ndx >= special_list_len) {
               msg(MSG_FATAL, "Special list overflow\n");
               exit(1);
            }      
            special_list[*special_list_ndx].index = start + 
                field_def[ndx].byte_offset_bit_len;
            special_list[(*special_list_ndx)++].pattern = 0x4489;
            value = 0xa1;
         break;
            // Special E3 with missing clock. We put E3 in the data and fix the
            // encoded MFM data curing the conversion.
         case FIELD_C0:
            if (*special_list_ndx >= special_list_len) {
               msg(MSG_FATAL, "Special list overflow\n");
               exit(1);
            }      
            special_list[*special_list_ndx].index = start + 
                field_def[ndx].byte_offset_bit_len;
            special_list[(*special_list_ndx)++].pattern = 0x12aa;
            value = 0xC0;
         break;
         case FIELD_NEXT_SECTOR:
            inc_sector(drive_params);
            data_set = 1;
         break;
         default:
            msg(MSG_FATAL, "Unknown field_def type %d\n",field_def[ndx].type);
            exit(1);
      }
      if (data_set) {
            // Just mark last byte written, track already updated
         field_filled = MAX(field_filled, field_def[ndx].byte_offset_bit_len + 
            field_def[ndx].len_bytes - 1);
         // If no bit list update the specified bytes
      } else if (field_def[ndx].bit_list == NULL) {
            // For the silly controller that has the bits backward reverse them
         if (field_def[ndx].op == OP_REVERSE || field_def[ndx].op == OP_REVERSE_XOR) {
            value = reverse_bits(value, field_def[ndx].len_bytes * 8);
         }

         field_filled = MAX(field_filled, field_def[ndx].byte_offset_bit_len + 
            field_def[ndx].len_bytes - 1);
         if (field_def[ndx].byte_offset_bit_len + field_def[ndx].len_bytes 
              > length) {
            msg(MSG_FATAL, "Track overflow field update %d %d %d\n", 
               field_def[ndx].byte_offset_bit_len, field_def[ndx].len_bytes, length);
            exit(1);
         }
            // Data goes in MSB first. Shift first byte to top of value
            // then pull of the bytes and update track
         value <<= (sizeof(value) - field_def[ndx].len_bytes) * 8;
         for (i = 0; i < field_def[ndx].len_bytes; i++) {
            int wbyte = (value >> (sizeof(value)*8 - 8));
            if (field_def[ndx].op == OP_XOR || field_def[ndx].op == OP_REVERSE_XOR) {
               track[field_def[ndx].byte_offset_bit_len + i] ^= wbyte;
            } else {
               track[field_def[ndx].byte_offset_bit_len + i] = wbyte;
            }
            value <<= 8;
         }
      } else {
         int ndx2 = 0;
         BIT_L *bit_list = field_def[ndx].bit_list;
         int byte_offset, bit_offset;
         uint8_t temp;
         int bit_count = 0;

            // For the silly controller that has the bits backward reverse them
         if (field_def[ndx].op == OP_REVERSE || field_def[ndx].op == OP_REVERSE_XOR) {
            value = reverse_bits(value, field_def[ndx].byte_offset_bit_len);
         }
            // Now pull off starting at the highest bit and put them into
            // the bits specified. In this encoding the bits are numbered with
            // the most significant bit of the first byte 0 counting up.
            // Multiple disjoint bit fields can be specified.
         while (bit_list[ndx2].bitl_start != -1) {
            for (i = 0; i < bit_list[ndx2].bitl_length; i++) {
                  // Find byte and bit to update
               byte_offset = (bit_list[ndx2].bitl_start + i) / 8; 
               field_filled = MAX(field_filled, byte_offset);
               bit_offset = (bit_list[ndx2].bitl_start + i) % 8; 
               if (byte_offset >= length) {
                  msg(MSG_FATAL, "Track overflow bit field\n");
                  exit(1);
               }
                  // Extract bit and update in track
               temp = ( (value >> (field_def[ndx].byte_offset_bit_len - 
                  bit_count++ - 1)) & 1) << (7 - bit_offset);
               if (field_def[ndx].op == OP_XOR || field_def[ndx].op == OP_REVERSE_XOR) {
                  track[byte_offset] ^= temp;
               } else {
                  track[byte_offset] &= ~(1 << (7 - bit_offset));
                  track[byte_offset] |= temp;
               } 
            }
            ndx2++;
         }
            // Verify field size agrees with sum of bit field lengths
         if (bit_count != field_def[ndx].byte_offset_bit_len) {
            msg(MSG_FATAL, "Bit field length mismatch %d %d\n", 
               bit_count, field_def[ndx].byte_offset_bit_len);
            exit(1);
         }
      }
      ndx++;
   }
      // Verify that last byte in field was updated
   if (field_filled != length - 1) {
      msg(MSG_FATAL, "Incorrect field length %d %d\n",field_filled, length);
      exit(1);
   }
}

// Process track definition list to update track data.
//
// drive_params: Drive parameters
// track: Track data array
// start: Offset into track to start writing data to
// length: Number of bytes in track
// track_def: The array of track definitions to process
// special_list: List of locations where special a1 code is in track
// special_list_ndx: Next free index in list
// special_list_len: Maximum number of entries in list
//
// return: Next offset into track to write to
static int process_track(DRIVE_PARAMS *drive_params,
   uint8_t track[], int start, int length, TRK_L track_def[],
   SPECIAL_LIST special_list[], int *special_list_ndx, int special_list_len)
{
   int ndx = 0;
   int new_start;
   int i;

//printf("Process track called start %d\n",start);
   while (track_def[ndx].count != -1) {
      switch (track_def[ndx].type) {
         case TRK_FILL:
            if (start + track_def[ndx].count > length) {
               msg(MSG_FATAL, "Track overflow by %d fill, %d %d %d\n", 
                   start + track_def[ndx].count - length,
                   start, track_def[ndx].count, length);
               exit(1);
            }
            memset(&track[start], track_def[ndx].value, track_def[ndx].count);
            start += track_def[ndx].count;
         break;
         case TRK_SUB:
               // Process a sub list of track definitions the specified number
               // of times
            for (i = 0; i < track_def[ndx].count; i++) {
               start = process_track(drive_params, track, start, length, 
                  (TRK_L *) track_def[ndx].list, 
                  special_list, special_list_ndx, special_list_len);
            }
         break;
         case TRK_FIELD:
            new_start = start + track_def[ndx].count;
            if (new_start >= length) {
               msg(MSG_FATAL, "Track overflow field %d,%d\n", new_start, length);
               exit(1);
            }
               // Fill the field with the specified value then
               // process the field definitions
            memset(&track[start], track_def[ndx].value, track_def[ndx].count);
            process_field(drive_params, track, start, track_def[ndx].count, 
               (FIELD_L *) track_def[ndx].list, special_list, special_list_ndx,
               special_list_len);
            start = new_start;
         break;
         default:
            msg(MSG_FATAL, "Unknown track_def type %d\n",track_def[ndx].type);
            exit(1);
      }
      ndx++;
   }
   return start;
}


// Convert data to MFM encoded data.
//
// data: Bytes to convert
// length: Number of bytes to convert
// mfm_data: Destination to write encoded data to
// special_list: List of locations special a1 mark pattern should be written
// special_list_length; Length of list
void mfm_encode(uint8_t data[], int length, uint32_t mfm_data[], int mfm_length,
   SPECIAL_LIST special_list[], int special_list_length) 
{
      // Convert a byte to 16 MFM encoded bits. First index is the MFM
      // bit immediately preceding.
   static uint16_t mfm_encode[2][256]; 
      // One on first call
   static int first_time = 1;
      // Used to index first subscript in mfm_encode
   int last_bit = 0;
      // Counters.
   int i, lbc;
   int bit;
   uint16_t value16;
   uint32_t value32 = 0;
      // extracted bit
   int ext_bit;
   int special_list_ndx = 0;

   if (length * 2 / sizeof(mfm_data[0]) > mfm_length) {
      msg(MSG_FATAL, "MFM data overflow\n");
      exit(1);
   }
      // Generate table to convert a byte to 16 MFM encoded bits
   if (first_time) {
      for (lbc = 0; lbc < 2; lbc++) {
         for (i = 0; i < 256; i++) {
            last_bit = lbc;
            value16 = 0;
            for (bit = 7; bit >= 0; bit--) {
               value16 <<= 2;
               ext_bit = (i >> bit) & 1;
               value16 |= ((!(last_bit | ext_bit)) << 1) | ext_bit;
               last_bit = ext_bit;
            }
            mfm_encode[lbc][i] = value16;
         }
      }
      first_time = 0;
   }
   last_bit = 0;
   for (i = 0; i < length; i++) {
         // If at the top location in the special list write the special.
         // pattern. List is in ascending order. Otherwise encode the byte
      if (special_list_ndx < special_list_length && i == special_list[special_list_ndx].index) {
         value16 = special_list[special_list_ndx].pattern;
         special_list_ndx++;
      } else {
         value16 = mfm_encode[last_bit][data[i]];
      }
         // Put in correct half of 32 bit word. 
      if (i & 1) {
         value32 = value32 << 16 | value16;
         mfm_data[i/2] = value32;
      } else {
         value32 = value16;
      }
      last_bit = value16 & 1;
   }
}

// Convert an extracted data file to an emulator file
// 
//TODO: handle marking bad sectors. Is interleave handling sufficient?
//   Think about handle DEC_RQDX3 format where tracks vary. Having format
//   vary between sectors on same track is really annoying.
void ext2emu(int argc, char *argv[])
{
      // Store bytes for track
   uint8_t *track;
   int track_length;
   uint32_t *track_mfm;
      // Store byte locations in track where special MFM encoding of A1
      // mark field needs to be inserted
   SPECIAL_LIST special_list[MAX_SECTORS*2];
   int special_list_ndx = 0;
      // Number of bytes written to track
   int track_filled = 0;
   DRIVE_PARAMS drive_params;
   struct stat finfo;
   int calc_size;
   CONTROLLER *controller;

   parse_cmdline(argc, argv, &drive_params, "sgjdlu3rabt", 1, 0, 0, 1);

   parse_validate_options_listed(&drive_params, "hcemf");

   if (mfm_controller_info[drive_params.controller].track_layout == NULL) {
      msg(MSG_FATAL, "Not yet able to process format %s\n",
         mfm_controller_info[drive_params.controller].name);
      exit(1);
   }

      // Pull various parameters we need for this controller
   controller = &mfm_controller_info[drive_params.controller];
   drive_params.num_sectors = controller->write_num_sectors;
   drive_params.first_sector_number = controller->write_first_sector_number;
   drive_params.sector_size = controller->write_sector_size;
   drive_params.metadata_bytes = controller->metadata_bytes;
   drive_params.data_crc = controller->write_data_crc;
   drive_params.header_crc = controller->write_header_crc;
   drive_params.emu_track_data_bytes = controller->track_words * 4;
   drive_params.start_time_ns = controller->start_time_ns;

   // Save final parameters
   drive_params.cmdline = parse_print_cmdline(&drive_params, 0, 1);

   drive_params.ext_fd = open(drive_params.extract_filename, O_RDONLY);
   if (drive_params.ext_fd < 0) {
      msg(MSG_FATAL, "Unable to open extract file: %s\n", strerror(errno));
      exit(1);
   }
   if (drive_params.metadata_bytes != 0) {
      char extention[] = ".metadata";
      char fn[strlen(drive_params.extract_filename) + strlen(extention) + 1];

      strcpy(fn, drive_params.extract_filename);
      strcat(fn, extention);

      drive_params.ext_metadata_fd = open(fn, O_RDONLY);
      if (drive_params.ext_metadata_fd < 0) {
         msg(MSG_FATAL, "Unable to open extract tag file: %s\n", strerror(errno));
         exit(1);
      }
   }
   drive_params.emu_fd = emu_file_write_header(drive_params.emulation_filename,
        drive_params.num_cyl, drive_params.num_head,
        drive_params.cmdline, drive_params.note,
        controller->clk_rate_hz,
        drive_params.start_time_ns, drive_params.emu_track_data_bytes);

   fstat(drive_params.ext_fd, &finfo);
   calc_size = drive_params.sector_size * drive_params.num_sectors * 
        drive_params.num_head * drive_params.num_cyl;
      // Warn if the extracted data file doesn't match the expected size for
      // the parameters specified
   if (calc_size != finfo.st_size) {
      msg(MSG_INFO, "Calculated extract file size %d bytes, actual size %jd\n",
        calc_size, (intmax_t) finfo.st_size); }

      // If interleave values specified set them
   if (drive_params.sector_numbers != NULL) {
      set_sector_interleave(&drive_params, drive_params.sector_numbers[0], 
         drive_params.sector_numbers[1]);
   } else {
      set_sector_interleave(&drive_params, 1, 0);
   }

   memset(special_list, 0, sizeof(special_list));

   track_length = drive_params.emu_track_data_bytes / 2;
   track = msg_malloc(track_length , "Track data");
   track_mfm = msg_malloc(drive_params.emu_track_data_bytes, "Track data bits");

   // Step through each cylinder and track and process the data
   for (cyl = 0; cyl < drive_params.num_cyl; cyl++) {
      if (cyl % 10 == 0)
         msg(MSG_PROGRESS, "At cyl %d\r", cyl);
      set_cyl(cyl);
      start_new_cyl(&drive_params);
      for (head = 0; head < drive_params.num_head; head++) {
         set_head(head);
         start_new_track(&drive_params);
         memset(track, 0, track_length);
            // Generate the byte data in track, convert to MFM and write
            // to emulator file
         track_filled = process_track(&drive_params, track, 0, track_length,
            controller->track_layout, 
            special_list, &special_list_ndx, ARRAYSIZE(special_list));
         mfm_encode(track, track_length, track_mfm, 
            drive_params.emu_track_data_bytes / sizeof(track_mfm[0]), 
            special_list, special_list_ndx);
         emu_file_write_track_bits(drive_params.emu_fd, (uint32_t *)track_mfm,
             drive_params.emu_track_data_bytes/4, cyl, head,
             drive_params.emu_track_data_bytes);

         special_list_ndx = 0;
      }
   }
      // Warn if we didn't update all of the track array
   if (track_filled != track_length) {
      msg(MSG_INFO, "Not all track filled, %d of %d bytes used\n",
        track_filled, track_length);
   }
   emu_file_close(drive_params.emu_fd, 1);
}


