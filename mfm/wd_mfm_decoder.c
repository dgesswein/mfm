// This routine decodes "Western Digital" (WD) formated disks. It's not really
// specific to WD and not all WD controllers use exactly the same format but
// WD was the first controller found with this basic format.
// The format is Headers and data are marked with an 0xa1 byte with a missing
// clock bit followed by a 0xf# byte. Several different header formats are
// supported.
//
// TODO Improve data separator and resync better after end of 
// write transition area. Also why avg_bit_sep_time is < 20 for all drives
// We probably should be able to do better than just the PLL since we can 
// look ahead.
// TODO use bytes between header marks to figure out if data or header 
// passed. Use sector_numbers to recover data if only one header lost.
//
// 11/13/15 DJG Added Seagate ST11M format
// 11/07/15 DJG Added SYMBOLICS_3640 format
// 11/01/15 DJG Renamed RUSSIAN to ELEKTRONIKA_85 and SYMBOLICS to
//    SYMBOLICS_3620 to allow support for other models.Use new drive_params 
//    field and comment changes
// 05/17/15 DJG Added new formats ADAPTEC, NEWBURYDATA, SYMBOLICS, MIGHTYFRAME,
//   and partially implemented RUSSIAN. Corrected format comments.
//   Code cleanup.
// 11/09/14 DJG Added bad block and alternate track decoding for OMTI_5510
// 09/10/14 DJG Added new Olivetti format decode. Format is best guess
//    based on one XM5220/2 drive sample. Unknown what controller wrote it.
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
//   CONTROLLER_WD_1006, Western digital 1006.
//   Closest manuals found http://bitsavers.trailing-edge.com/pdf/westernDigital/WD100x/
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. bits 5-6 sector size, bit 7 bad block flag
//         sector size is 0 256 bytes, 1 512 bytes, 2 1024 bytes, 3 128 bytes
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_OMTI_5510
//   Manual http://bitsavers.trailing-edge.com/pdf/sms/pc/
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder high
//      byte 3 cylinder low
//      byte 4 head and flags. (bit 7 bad, bit 6 assigned alternate track,
//         bit 5 is alternate track)
//      byte 5 sector
//      byte 6-9 ECC code
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size 
//         (alt cyl first 2 bytes msb first, head third)
//      ECC code (4 byte)
//
//   CONTROLLER_DEC_RQDX3
//   No manual found describing low level format
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 cylinder high in upper 4? bits
//      byte 4 sector number
//      byte 5 unknown, 2 for sample I have
//      byte 6-7 CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      ECC code (4 byte)
//
//   CONTROLLER_OLIVETTI, Olivetti is drive since controller is not known
//   5 byte header + 3 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe 
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. bits 5-7 upper 3 bits cyl, 
//         bit 7 bad block flag
//      byte 4 sector number
//      bytes 5-7 24 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//   The two starting bytes are not used in the CRC
//
//   CONTROLLER_MACBOTTOM, Old MACBOTTOM serial drive for Mac
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 Upper 8? bits of cylinder
//      byte 2 Lower 8 bits of cylinder
//      byte 3 head
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//      
//   CONTROLLER_ADAPTEC
//      6 byte header + 4 byte CRC
//      format from http://bitsavers.informatik.uni-stuttgart.de/pdf/maxtor/1014286A_XT1000_2000_OEM.pdf
//      Only tested on spare track list of XT-2190 drive
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 Upper 8 bits of logical block address (LBA)
//      byte 3 Middle 8 bits of LBA
//      byte 4 Lower 8 bits of LBA
//      byte 5 Flag bit 0x40 indicates last sector on track
//      bytes 6-9 32 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      CRC/ECC code
//      
//   CONTROLLER_NEWBURYDATA
//   4 byte header + 2 byte CRC
//      No documentation. Found on some tracks of a Maxtor XT-2910 drive
//      byte 0 0xa1
//      byte 1 0xfe combined with high bits of cylinder
//      byte 2 Low cylinder
//      byte 3 low 4 bits head upper 4 bits sector
//      bytes 4-5 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_Elektronika 85?, Russian DECpro 350 clone. 
//   From Russian documentation it probably only used cyl9 bit and doesn't
//   support larger disks. 
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. 
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0x80
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_SYMBOLICS_3620, Symbolics 3620.
//   7 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 0xfe 
//      byte 3 upper bits of cylinder
//      byte 4 low 8 bits of cylinder
//      byte 5 head number.
//      byte 6 sector number
//      bytes 7-8 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      byte 2 0xf8
//      bytes 3-10 0 (unknown if part of sector data or header)
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_SYMBOLICS_3640, Symbolics 3640.
//      Closest documentation 
//      http://bitsavers.trailing-edge.com/pdf/symbolics/3600_series/Lisp_Machine_Hardware_Memos.pdf
//
//   11 byte header
//      byte 0 0xa1
//      byte 1 0x5a
//      byte 2 0x96 
//      byte 3 0x0e 
//      byte 4 0x0e 
//      byte 5 0x9e 
//      byte 6 0x01 
//      All the following fields the data is LSB first so needs to
//      be bit reversed.
//      byte 7 sector bits 7-5 head number bits 1,0  (lsb first)
//      byte 8 head number bits 7,6 cyl bit 3,2,1,0 (lsb first)
//      byte 9 rest of cylinder
//      byte 10 odd parity of bytes 7-9.
//   Data
//      Between header and data is a bunch of 0 bits followed by a 1 bit
//      to syncronize the decoding. 
//
//      byte 0 0xa1
//      byte 1 0xf8
//      byte 2 0xf8
//      bytes 3-10 0 (unknown if part of sector data or header)
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_MIGHTYFRAME, Mightyframe, variant of WD_1006
//   No manual
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-2 head number. bits 5-6 sector size, bit 7 bad block flag
//         sector size is 0 256 bytes, 1 512 bytes, 2 1024 bytes, 3 128 bytes
//      byte 4 bits 0-4 sector number, bit 5 high bit of head number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_SEAGATE_ST11M
//   No manual
//   The first two tracks of the first cylinder is used by the controller. The
//      entire first cylinder will be written to the extract file though
//      the unused tracks are likely to be unreadable so zeros. You will need
//      to remove the first cylinder to have only the normal data area.
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 0-3 Head. 6-7 upper 2 bits of cylinder.  
//         0xff for first cylinder used by controller.
//      byte 3 Low 8 bits of cylinder
//      byte 4 Sector
//      byte 5 4 if track has been assigned spare, 8 if it is the spare.
//   The tracks are formatted such that they can contain 18 sectors but
//      only 17 used. If a single bad sector is on the track it will be
//      marked bad with 0xff in byte 2-4 and the extra sector area used.
//      More than one and an alternate track is assigned.
//   When a track is assigned a spare the header for the bad track will
//      have the cylinder and head of the replacement track.
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
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
SECTOR_DECODE_STATUS wd_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span)
{
   static int sector_size;
   // Non zero if sector is a bad block, has alternate track assigned,
   // or is an alternate track
   static int bad_block, alt_assigned, is_alternate;
   static SECTOR_STATUS sector_status;

   if (*state == PROCESS_HEADER) {
      // Clear these since not used by all formats
      alt_assigned = 0;
      is_alternate = 0;
      bad_block = 0;

      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status |= SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      if (drive_params->controller == CONTROLLER_OMTI_5510) {
         sector_status.cyl = bytes[2]<< 8;
         sector_status.cyl |= bytes[3];

         // More is in here but what is not documented in manual
         sector_status.head = bytes[4] & 0xf;
         bad_block = bytes[4] >> 7;;
         alt_assigned = (bytes[4] & 0x40) >> 6;
         is_alternate = (bytes[4] & 0x20) >> 5;
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[5];
         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_DEC_RQDX3) {
         sector_status.cyl = (bytes[3] & 0xf0) << 4;
         sector_status.cyl |= bytes[2];

         sector_status.head = bytes[3] & 0xf;
         sector_status.sector = bytes[4];
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;
         bad_block = 0;
         // Don't know what's in this byte. Print a message so it can be
         // investigated if not the 2 seen previously.
         if (bytes[5] != 0x2) {
            msg(MSG_INFO, "Header Byte 5 not 2, byte %02x on cyl %d head %d sector %d\n",
                  bytes[5], sector_status.cyl, sector_status.head, sector_status.sector);
         }

         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_WD_1006) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = bytes[3] & 0xf;
         sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4];

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ELEKTRONIKA_85) {
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = bytes[3] & 0xf;
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MIGHTYFRAME) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = (bytes[3] & 0x7) | ((bytes[4] & 0x20) >> 2);
         sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4] & 0x1f;

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_OLIVETTI) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0x70) << 4);

         sector_status.head = bytes[3] & 0xf;
         sector_size = 512;
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4];

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MACBOTTOM) {
         sector_status.cyl = bytes[2] | (bytes[1] << 8);
         sector_status.head = bytes[3];
         sector_status.sector = bytes[4];
         sector_size = 524;
      } else if (drive_params->controller == CONTROLLER_ADAPTEC) {
         uint32_t lba_addr;
         lba_addr = (bytes[2] << 16) | (bytes[3] << 8) | bytes[4];
         sector_status.sector = lba_addr % drive_params->num_sectors;
         // Prevent divide by 0
         if (drive_params->num_sectors == 0 || drive_params->num_head == 0) {
            sector_status.head = 0;
            sector_status.cyl = 0;
         } else {
            sector_status.head = (lba_addr / drive_params->num_sectors) %
              drive_params->num_head;
            sector_status.cyl = lba_addr / (drive_params->num_sectors *
              drive_params->num_head);
         }

//printf("lba %d sect %d head %d cyl %d\n", lba_addr, sector_status.sector,
//   sector_status.head, sector_status.cyl);
         sector_size = drive_params->sector_size;
         bad_block = 0;
         if (bytes[5] != 0 && bytes[5] != 0x40) {
            msg(MSG_INFO, "Unknown header flag byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
         }

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_NEWBURYDATA) {
         //int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high_lookup[16] = {0,1,-1,-1,3,-1,-1, 2,-1,-1,-1,-1,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = bytes[3] >> 4;
         bad_block = 0;
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[3] & 0xf;

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3620) {
         sector_status.cyl = (bytes[3] << 8) | bytes[4];
         sector_status.head = bytes[5];
         sector_status.sector = bytes[6];
         sector_size = drive_params->sector_size;
         if (bytes[1] != 0xfe || bytes[2] != 0xfe) {
            msg(MSG_INFO, "Invalid header id bytes %02x, %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], bytes[2], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         // Bytes that seem to always be at the start of the header. We
         // check the various unknow bits so if they change we can see what
         // they represent.
         uint8_t header_start[] = {0xa1,0x5a,0x96,0x0e,0x0e,0x9e,0x01};
         // This reverses the bit ordering in a byte. The controller writes
         // the header data LSB first not the normal MSB first.
         static unsigned char rev_lookup[16] = {
            0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
            0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
         #define REV_BYTE(n)( (rev_lookup[n&0xf] << 4) | rev_lookup[n>>4])
         // Calculate the even parity of a byte
         static unsigned char parity_lookup[16] = {
            0x0, 0x1, 0x1, 0x0, 0x1, 0x0, 0x0, 0x1,
            0x1, 0x0, 0x0, 0x1, 0x0, 0x1, 0x1, 0x0 };
         #define EPARITY(n) (parity_lookup[n&0xf] ^ parity_lookup[n>>4])

         sector_status.cyl = (REV_BYTE(bytes[8]) >> 4) | (REV_BYTE(bytes[9]) << 4);
         sector_status.head = (REV_BYTE(bytes[7]) >> 6) | ((REV_BYTE(bytes[8]) & 0x3) << 2);
         sector_status.sector = REV_BYTE(bytes[7]) & 0x7;
         if ((bytes[7] & 0x1c) != 0 || (bytes[8] & 0x30) != 0 || 
               (bytes[10] & 0xfe) != 0 ) {
            msg(MSG_INFO,"Unexpected bits set %02x %02x %02x on cyl %d,%d head %d,%d sector %d\n",
                bytes[7], bytes[8], bytes[10], exp_cyl, sector_status.cyl,
                exp_head, sector_status.head, sector_status.sector);
         }
         // Header has an odd parity check bit
         if ((EPARITY(bytes[7]) ^ EPARITY(bytes[8]) ^ EPARITY(bytes[9]) ^
                EPARITY(bytes[10])) != 1) {
            msg(MSG_INFO,"Header parity mismatch\n");
            sector_status.status |= SECT_BAD_HEADER;
         }
         // Not encoded in header so use what was provided.
         sector_size = drive_params->sector_size;

         if (memcmp(bytes, header_start, sizeof(header_start)) != 0) {
            int i;

            sector_status.status |= SECT_BAD_HEADER;
            for (i = 0; i < sizeof(header_start); i++) {
                if (bytes[i] != header_start[i]) {
                   msg(MSG_INFO, "Header byte %d differ %02x %02x on cyl %d,%d head %d,%d sector %d\n",
                      i, bytes[i], header_start[i], exp_cyl, sector_status.cyl,
                      exp_head, sector_status.head, sector_status.sector);
                }
             }
         }
      } else if (drive_params->controller == CONTROLLER_SEAGATE_ST11M) {
         if (bytes[2] == 0xff) {
            if (bytes[3] == 0xff) {
               // Bad block. Everything other than unknown byte set to 0xff
               // Set what we know and mark bad so it won't be used.
               sector_status.cyl = exp_cyl;
               sector_status.head = exp_head;
               sector_status.status |= SECT_BAD_HEADER;
               msg(MSG_INFO,"Spare sector used on cyl %d, head %d, physical sector %d\n",
                  sector_status.cyl, sector_status.head, *sector_index);
            } else {
               // Controller area only had sector and possibly cylinder
               sector_status.cyl = bytes[3];
               sector_status.head = exp_head;
               sector_status.sector = bytes[4];
            }
         } else {
            uint8_t byte5 = bytes[5];
            uint8_t byte2 = bytes[2];

            sector_status.cyl = (((bytes[2] & 0xc0) << 2) | bytes[3]) + 1;
            sector_status.head = bytes[2] & 0xf;
            if (byte5 == 0x4) {
               msg(MSG_INFO, "Cylinder %d head %d assigned alternate cyl %d head %d. Extract data not fixed\n",
                  exp_cyl, exp_head, sector_status.cyl, sector_status.head);
               byte5 = 0;
            }
            if (byte5 == 0x8) {
               is_alternate = 1;
               // Clear various bits set so check below doesn't report
               // unexpected values
               byte5 = 0;
               byte2 = byte2 & ~0x20;
            }
            if (byte5 != 0x0 || (bytes[4] & 0xe0) != 0 || 
                 (byte2 & 0x30) != 0) {
               msg(MSG_INFO, "Unexpected bytes  %02x, %02x, %02x on cyl %d,%d head %d,%d sector %d\n",
                     bytes[2], bytes[4], byte5, exp_cyl, sector_status.cyl,
                     exp_head, sector_status.head, sector_status.sector);
            }
            sector_status.sector = bytes[4];
         }
         sector_size = drive_params->sector_size;
         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else {
         msg(MSG_FATAL,"Unknown controller type %d\n",drive_params->controller);
         exit(1);
      }

      mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
            seek_difference, &sector_status, drive_params);

      msg(MSG_DEBUG,
         "Got exp %d,%d cyl %d head %d sector %d,%d size %d bad block %d\n",
            exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
            sector_status.sector, *sector_index, sector_size, bad_block);

      if (bad_block) {
         msg(MSG_INFO,"Bad block set on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
      }
      if (is_alternate) {
         msg(MSG_INFO,"Alternate cylinder set on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
      }
      // The 3640 doesn't have a 0xa1 header, search for its special sync
      if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         *state = MARK_DATA1;
      } else {
         *state = MARK_DATA;
      }
   } else { // Data
      // Value and where to look for header mark byte
      int id_byte_expected = 0xf8;
      int id_byte_index = 1;
      if (drive_params->controller == CONTROLLER_DEC_RQDX3) {
         id_byte_expected = 0xfb;
      } else if (drive_params->controller == CONTROLLER_ELEKTRONIKA_85) {
         id_byte_expected = 0x80;
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3620) {
         if (bytes[2] != 0xf8) {
            msg(MSG_INFO, "Invalid data id bytes %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], bytes[2], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         id_byte_expected = 0xe0;
         id_byte_index = 0;
      }
      if (bytes[id_byte_index] != id_byte_expected && crc == 0) {
         msg(MSG_INFO,"Invalid data id byte %02x expected %02x on cyl %d head %d sector %d\n", 
               bytes[1], id_byte_expected,
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_DATA;
      }
      if (drive_params->controller == CONTROLLER_OMTI_5510 && alt_assigned) {
         msg(MSG_INFO,"Alternate cylinder assigned cyl %d head %d (extract data not fixed)\n",
            (bytes[2] << 8) + bytes[3], bytes[4]);
      }
      if (crc != 0) {
         sector_status.status |= SECT_BAD_DATA;
      }
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }
      sector_status.ecc_span_corrected_data = ecc_span;
      if (!(sector_status.status & SECT_BAD_HEADER)) {
         // TODO: Make handling of correction data for extract cleaner
         if (mfm_write_sector(&bytes[2], drive_params, &sector_status,
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
SECTOR_DECODE_STATUS wd_decode_track(DRIVE_PARAMS *drive_params, int cyl,
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
   // Bit count of start of header for Symbolics 3640
   int header_raw_bit_count = 0;

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
         if ((state == MARK_ID || state == MARK_DATA)) {
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
#endif
            if ((raw_word & 0xffff) == 0x4489
                  && zero_count >= MARK_NUM_ZEROS) {
               header_raw_bit_count = tot_raw_bit_cntr;
               zero_count = 0;
               bytes[0] = 0xa1;
               byte_cntr = 1;
               if (state == MARK_ID) {
                  state = PROCESS_HEADER;
                  mfm_mark_header_location(all_raw_bits_count);
                  // Figure out the length of data we should look for
                  bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes + 
                        drive_params->header_crc.length / 8;
                  bytes_needed = bytes_crc_len + HEADER_IGNORE_BYTES;
               } else {
                  state = PROCESS_DATA;
                  mfm_mark_data_location(all_raw_bits_count);
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
         } else if (state == MARK_DATA1) {
            if ((tot_raw_bit_cntr - header_raw_bit_count) > 530 && 
                  ((raw_word & 0xf) == 0x9)) {
               state = PROCESS_DATA;
               mfm_mark_data_location(all_raw_bits_count);
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
               // Resync decoding to the mark
               raw_bit_cntr = 0;
               decoded_word = 0;
               decoded_bit_cntr = 0;
               byte_cntr = 0;
            }
         } else {
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
                  } else {
                     mfm_mark_end_data(all_raw_bits_count, drive_params);
                     all_sector_status |= mfm_process_bytes(drive_params, bytes,
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
   if (all_sector_status == SECT_NO_STATUS) {
      all_sector_status = SECT_BAD_HEADER;
   }
   return all_sector_status;
}
