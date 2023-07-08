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
// Code has somewhat messy implementation that should use the new data
// on format to drive processing. Also needs to be added to other decoders.
//
// 07/08/23 DJG Added Fujitsu-K-10R format
// 03/10/23 DJG Added ES7978 format
// 12/08/22 DJG Changed error message
// 10/01/22 DJG Added CTM9016 format
// 07/20/22 DJG Process sector if bytes decoded exactly matches needed
// 05/29/22 TJT Added Callan Unistar format
// 05/04/22 DJG Fixed Adaptec printing wrong sector in debug message
// 03/17/22 DJG Handle large deltas and improved error message
// 12/18/21 SWE Added David Junior_II format.
// 09/19/21 DJG Added TANDY_16B format.
// 08/28/21 DJG Fixed alternate cylinder handling for iSBC 214, iSBC 215,
//    SM_1810. Only example found in images I have was for iSBC 215. Unable
//    to validate if head pulled from correct byte. May still have errors.
// 08/27/21 DJG Added DSD_5217_512B format
// 05/27/21 DJG Added TEKTRONIX_6130 format
// 01/07/21 DJG Added RQDX2 format
// 10/25/20 DJG Fix regression in DEC_RQDX3. Make MYARC_HDFC implementation
//    match chip datasheet.
// 10/24/20 DJG Added MYARC_HFDC controller
// 10/18/20 DJG Added alternate cylinder support for SHUGART_1610
// 10/16/20 DJG Added SHUGART_SA1400 controller
// 10/08/20 DJG Added SHUGART_1610 and UNKNOWN2 controllers
// 09/21/20 DJG Added controller SM_1810_512B and fixed SYMBOLICS_3640
//   incorrectly decoding data
// 01/29/20 DJG Added support for 4th head bit for 3B1/UNIXPC
// 07/05/19 DJG Improved 3 bit head field handling
// 06/20/19 DJG Removed lines of code that were accidently left for adding 
//    alternate tracks for ISBC_214_*
// 06/19/19 DJG Added SM1040 and removed DTC_256B
// 01/20/19 DJG Added seperate format for ISBC_214. Also ISBC_214 and 215
//    renamed to have sector size as part of name for ext2emu. Make ISB_214/215
//    alternate track decoding more robust
// 12/16/18 DJG Added NIXDORF_8870
// 12/15/18 DJG Changed MACBOTTOM to not hard code sector size. Format also
//    used by Philips P3800 with 512 byte sectors.
// 09/10/18 DJG Added CONTROLLER_IBM_3174
// 09/10/18 DJG Added CONTROLLER_DILOG_DQ604
// 08/27/18 DJG Mark sector data bad if too far from header
// 08/05/18 DJG Added IBM_5288 format
// 07/02/18 DJG Added Convergent AWS SA1000 format and new data for finding
//   correct location to look for headers. Remove debug print.
// 06/10/18 DJG Added fourth DTC type. Modified header processing to allow
//    processing valid sector header found when expected data header.
// 04/22/18 DJG Added support for non 10 MHz bit rate and Altos format
// 03/31/18 DJG Allowed DTC to work with multiple sector sizes. Split DTC
//    into three versions for ext2emu
// 03/07/18 DJG Added CONTROLLER_DILOG_DQ614
// 02/04/18 DJG Made iSBC 215 alternate track handling match documentation
//    better. No example to verify with.
// 09/30/17 DJG Added Wang 2275
// 08/11/17 DJG Added Convergent AWS
// 04/21/17 DJG changed mfm_check_header_values to update sector_status_list
//    and added determining --begin_time if needed
// 03/08/17 DJG Moved Intel iSBC 215 from xebec_mfm_decoder.c
// 02/12/17 DJG Added Data Geneeral MV/2000
// 02/09/17 DJG Added AT&T 3B2
// 02/07/17 DJG Added Altos 586
// 01/06/17 DJG Change bad block message to MSG_INFO since no longer
//    causes retry
// 12/11/16 DJG Fixed handling of Adaptec bad blocks
// 10/31/16 DJG Improved Adaptec LBA support and handling sectors
//    marked bad or spare.
// 10/21/16 DJG Added unknown controller similar to MD11
// 10/16/16 DJG Indicate sector 255 is flagging bad block for RQDX3.
//    Added MOTOROLA_VME10. Renamed OLIVETTI to DTC.
// 10/04/16 DJG Added Morrow MD11
// 10/02/16 DJG Sort of handle DEC_RQDX3 which has different header format
//    on last cylinder.
// 05/21/16 DJG Parameter change to mfm_mark* functions and moved 
//    handle_alt_track_ch to mfm_decoder.c
// 01/24/16 DJG Add MVME320 controller support
// 01/13/16 DJG Changes for ext2emu related changes on how drive formats will
//     be handled.
// 01/06/16 DJG Add code to fix extracted data file when alternate tracks are
//          used. Only a few format know how to determine alternate track
// 12/31/15 DJG Fixes to Symblics 3640 and ext2emu changes
// 12/24/15 DJG Comment changes
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
//    Controller likely DTC so renamed.
//   
// Copyright 2022 David Gesswein.
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

// This reverses the bit ordering in a byte. The controller writes
// the header data LSB first not the normal MSB first.
static unsigned char rev_lookup[16] = {
   0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
   0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
#define REV_BYTE(n)( (rev_lookup[n&0xf] << 4) | rev_lookup[n>>4])

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

// Return non zero if cylinder passed in is last cylinder on drive
static int IsOutermostCylinder(DRIVE_PARAMS *drive_params, int cyl)
{
	return cyl == drive_params->num_cyl - 1;
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
//   CONTROLLER_RQDX2
//      Same as WD_1006 execpt data byte 1 is 0xfb on later cylinders
//      Last cylinder is 16 sector instead of 18 sector and has different
//      sector size value. Code changed to ignore sector size. Two missing
//      sectors will be reported as error
//
//   CONTROLLER ISBC_214_128 ISBC_214_256 ISBC_214_512 ISBC_214_1024
//   http://www.bitsavers.org/pdf/intel/iSBC/134910-001_iSBC_214_Peripheral_Controller_Subsystem_Hardware_Reference_Manual_Aug_85.pdf
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. bits 5-6 sector size, bit 7 bad block flag
//         sector size is 0 256 bytes, 1 512 bytes, 2 1024 bytes, 3 128 bytes
//      byte 4 sector number, 6-7 format type. 00 = data track, 01 = alternate
//         10 = defective track, 11 = invalid.
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER TEKTRONIX_6130
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-2 low 3 bits head number. bits 5-6 sector size, 
//            bit 7 bad block flag
//         sector size is 0 256 bytes, 1 512 bytes, 2 1024 bytes, 3 128 bytes
//      byte 4 0-5 sector number, 6 = high bit of head
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_TANDY_8MEG, Tandy 8 meg 8" drive. Same as WD_1006
//      Needed separate entry since 8" drive with different data rate.
//   CONTROLLER_TANDY_16B. Same as WD_1006. Added to enable ext2emu support.
//      Example XENIX.emu in emu.zip
//   CONTROLLER_NIXDORF_8870
//   Reference image nixdorf-1-raw_data from 8870 Quattro/7
//      Same as WD_1006 except ECC and which bytes used in ECC differ so
//      needs to be separate format
//
//      Cyl 0, head 0 has 17 sectors. Rest of disk has 16 sectors
//      TODO: Support mixed formats.
//   CONTROLLER_ES7978. Same as WD_1006 with different data CRC. Uses Intel 
//      82062 which seems to only use 16 bit CRC. Data actually has 32 bit CRC.
//
//   CONTROLLER_MOTOROLA_VME10
//   5 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder high
//      byte 3 cylinder low
//      byte 4 bits 7-5 head 4-0 sector
//      CRC/ECC code
//
//   CONTROLLER_OMTI_5510 (Also 20D)
//   Manual http://bitsavers.trailing-edge.com/pdf/sms/pc/
//   20D image omti_20d.tran/zip
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
//   CONTROLLER_Morrow_MD11
//   Manual  http://www.bitsavers.org/pdf/morrow/boards/HD-DMA_Preliminary_Technical_Manual_Jan82.pdf
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 cylinder high
//      byte 4 head and flags. (bit 7 bad, bit 6 assigned alternate track,
//         bit 5 is alternate track) The format copied from OMTI_5510.
//         Unknown if these bits are the same.
//      byte 5 sector
//      byte 6-9 ECC code
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size 
//         (alt cyl first 2 bytes msb first, head third)
//      ECC code (4 byte)
//
//   CONTROLLER_UNKNOWN1
//   From image ST-212_190385_4H_306C.td
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 cylinder high
//      byte 4 head and flags. (bit 7 bad, bit 6 assigned alternate track,
//         bit 5 is alternate track) The format copied from OMTI_5510.
//         Unknown if these bits are the same.
//      byte 5 sector
//      byte 6-9 ECC code
//   Data
//      byte 0 0xa1
//      byte 1 sector number
//      Sector data for sector size 
//         (alt cyl first 2 bytes msb first, head third)
//      ECC code (4 byte)
//      
//
//   CONTROLLER_DEC_RQDX3
//   No manual found describing low level format
//   Likely same format as MYARC_HFDC except some sectors using different
//      format. Also sector 255 seems to be used for bad blocks.
//      TODO: Should these be combined. How to handle mixed format?
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 cylinder high in upper 4? bits, low 4 bits head
//      byte 4 sector number
//      byte 5 unknown, 2 for sample I have
//      byte 6-7 CRC
//
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      ECC code (4 byte)
//
//   CONTROLLER_MYARC_HFDC
//   No manual found describing low level format though uses HDC9234 chip
//   http://www.whtech.com/ftp/datasheets%20and%20manuals/Datasheets%20-%20Misc/SMC%20HDC9234.pdf
//   Table 6 provides some format info.
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe ^ upper 2? bits of cylinder
//      byte 2 cylinder low
//      byte 3 MSb bad sector, cylinder high in upper 3 bits, low 4 bits head
//      byte 4 sector number
//      byte 5 Upper 4 bits 0 = 4 byte ECC, f = 5 bytes, e = 6 byte, d = 7 byte.
//             Lower 4 bits 0 = 128 byte sector, 1 = 256 , 2 = 512, 3 = 1024
//                rest invalid.
//      byte 6-7 CRC
//
//   Data
//      byte 0 0xa1
//      byte 1 0xfb or 0xf8
//      Sector data for sector size
//      ECC code (4 byte)
//
//   CONTROLLER_SHUGART_1610
//   Manual at http://www.pdp8online.com/mfm/status.shtml. Search 1610
//   Example BB_SA1610_ST225.emu
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 Bit 7 X, cylinder high 6-4 bits, bit 3 Y 3 bits head
//         X Y
//         0 0 Good track
//         0 1 Alternate track
//         1 0 Bad track
//         1 1 Track Alternated. Cylinder & head are of alternate track assigned
//         Alternate handled not currently handled.
//      byte 4 sector number
//      byte 5-6 CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8 (documentation says 0xfb but disk has 0xf8)
//      Sector data for sector size
//      ECC code (4 byte)
//
//   CONTROLLER_DTC, From Olivetti drive. Also matches 520 series with
//      manual on bitsavers. Olivetti was 17 512 byte sectors per track. 
//      520 manual format was shown as 33 256 byte or 18 512 byte sectors.
//   5 byte header + 3 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe 
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. bits 4-6 upper 3 bits cyl, 
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
//   CONTROLLER_CTM9016. Same as MACBOTTOM except MACBOTTOM has 524 byte 
//      sectors and CTM9016 has 1024 byte sectors. CTM9016 also has different
//      header polynomial on cyl 0 head 0.
//
//   CONTROLLER_A310_PODULE. Same as MACBOTTOM except what bytes in CRC and
//      sector length. Uses HD63463 controller chip
//      
//   CONTROLLER_FUJITSU_K-10R. Hitachi HD63463 controller chip. MACBOTTOM
//      except 0xa1 not included in CRC 
//      image mihaylov disk2.7z
//
//   CONTROLLER_ADAPTEC
//      6 byte header + 4 byte CRC
//      format from http://bitsavers.informatik.uni-stuttgart.de/pdf/maxtor/1014286A_XT1000_2000_OEM.pdf
//      Only tested on spare track list of XT-2190 drive
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 Upper 8 bits of logical block address (LBA). MSB set on bad
//         or possibly spare sectors.
//      byte 3 Middle 8 bits of LBA
//      byte 4 Lower 8 bits of LBA
//      byte 5 Flag bit 0x40 indicates last sector on track. On another image
//         probably a 4000 series controller 0x80 used to mark last sector.
//         Another uses this field to count number of bad sectors skipped.
//         FORMAT_ADAPTEC_COUNT_BAD_BLOCKS
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
//   CONTROLLER_ELEKTRONIKA_85, Russian DECpro 350 clone. 
//   From Russian documentation it probably only used cyl9 bit and doesn't
//   support larger disks. 
//   DEC documenation http://www.bitsavers.org/pdf/dec/pdp11/pro3xx/XT_Hardware_Handbook_1982.pdf
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
//      1 byte backup revision (only zero seen). 
//      15 bytes reserved (only zero seen)
//      This code doesn't to anything with these 16 bytes other than checked
//      by CRC.
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
//      byte 10 bit 0 odd parity of bytes 7-9.
//   Data
//      Between header and data is a bunch of 0 bits followed by a 1 bit
//      to syncronize the decoding. 
//
//      byte 0 0xe0
//      Sector data for sector size (1160 bytes)
//      first byte of sector data was always zero so it may be part of header.
//      CRC/ECC code
//
//   CONTROLLER_MVME320
//   MVME320B-D1.pdf table 8-1. Modify slightly to agree with what seen
//   from disk read.
//   7 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe 
//      byte 2 high bits of cylinder
//      byte 3 low 8 bits of cylinder
//      byte 4 head
//      byte 5 sector number
//      byte 6 01
//      bytes 7-8 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      CRC/ECC code
//
//
//   CONTROLLER_MIGHTYFRAME, Mightyframe, variant of WD_1006
//   No manual
//   Likely same as CONTROLLER_WD_3B1 after 4th head select was added
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
//   CONTROLLER_DG_MV2000, Data General MV/2000, WD2010 controller chip
//   No manual
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe exclusive ored with cyl11 0 cyl10 cyl9
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-2 head number. bits 5-6 sector size, bit 7 bad block flag
//         sector size is 0 256 bytes, 1 512 bytes, 2 1024 bytes, 3 128 bytes
//      byte 4 bits 0-4 sector number, bit 7 high bit of head number
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
//      byte 3 Low 8 bits of cylinder. Controller cylinder and first user
//         cylinder are both 0.
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
//   Oleksandr Kapitanenko sent me more information he determined for the
//   emulator he is working on. In this description side is what I call
//      head, track is what I call cylinder.
//      sync length = 10
//      gap_byte = 0x4E
//      gap1 = 19
//      gap2 = 0
//      gap3 = 20
//
//      CRC:
//      crc_order = 32
//      crc_poly = 0x41044185
//      crc_direct = 1
//      crc_init = 0
//      crc_xor = 0
//      crc_refin = 0
//      crc_refout = 0
//      Track:
//      (gap1 0x4E x 19 bytes)
//      repeat:
//      (sync 0x00 x 10 bytes) 0xA1 0xFE (side 1 byte)(track 1 byte)(sector 1
//      byte) 0x00 (CRC 4 bytes Big endian)(0x00 x 5  bytes)
//      (sync 0x00 x 10 bytes) 0xA1 0xF8 (512 bytes payload)(CRC 4 bytes Big
//      endian)(0x00 x 2 bytes)(gap3 0x4E x 20 bytes)
//
//      Track 0 is reserved for storing low level format information and bad
//      sector map. It has special data in sector index fields: side is always
//      0xFF for all sides
//
//      Tracks 1 an onwards are used for host computer data. Sector ID field
//      details:
//      Track #: starts from 0 , so Track 0 and Track 1 will have track field
//      =0, Track 2 =1, Track 3 =2 and so on.
//      Bits 9 and 8 of track number are stored in bits 7 and 6 of side byte.
//
//   CONTROLLER_ALTOS_586, Altos 586
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 7-4 head number. bits 2-0 upper 4 bits of cylinder
//         bit 3 back block. If bad block set no data follows header.
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_ATT_3B2, AT&T 3B2
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xff xored with upper bits of cylinder
//      byte 2 low 8 bits of cylinder
//      byte 3 head number
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_ISBC_215, Intel 310/MDS-4 with iSBC-215 controller
//      http://www.bitsavers.org/pdf/intel/iSBC/144780-002_iSBC_215_Generic_Winchester_Disk_Controller_Hardware_Reference_Manual_Dec84.pdf
//      See pages around 80 and 132. It appears unused alternate tracks aren't
//      formatted till use.
//   Image nopwd5619.emu has example of alternate cylinder assigned. Assigned
//      for head 0 so not able to verify head pulled from correct byte.
//
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0x19 or 0xd9
//      byte 2
//         3-0 high cyl
//         5-4 sector size 00 = 128, 01 = 256, 10 = 512, 11 = 1024
//         7-6 flag 00 = data track, 01 = assigned alternate
//                  10 = defective, 11 = invalid
//         No example of alternate so not tested
//      byte 3 low 8 bits of cylinder
//      byte 4 sector number
//      byte 5 head number
//      bytes 6-9 32 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xd9
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_SM-1810
//      Soviet clone. Source stated clone of iSBC 214. Much closer to
//         iSBC 215. Byte 2 fields not all verified.
//      https://github.com/dgesswein/mfm/issues/35
//      Format
//      https://user-images.githubusercontent.com/39098890/97402224-3e8daa80-1903-11eb-90c0-8c81abeccd40.jpg
//
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 (Flag not set in example images so not verified)
//         3-0 high cyl
//         5-4 sector size 00 = 128, 01 = 256, 10 = 512, 11 = 1024
//         7-6 flag 00 = data track, 01 = assigned alternate
//                  10 = defective, 11 = invalid
//         No example of alternate so not tested
//      byte 3 low 8 bits of cylinder
//      byte 4 sector number
//      byte 5 head number
//      bytes 6-9 32 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      CRC/ECC code
// 
//   CONTROLLER_DSD_5217_512B
//   http://www.bitsavers.org/pdf/dsd/5215_5217/040040-01_5215_UG_Apr84.pdf
//   iris-2400-w3.3 image
//   Used in SGI IRIS 2400. DSD/Data Systems Design changed name to Qualogy
//   Similar to SM-1810
//   5217 and 5215 use same format
//
//   6 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 (Flag not set in example images so not verified)
//         3-0 high cyl
//         5-4 sector size 00 = 128, 01 = 256, 10 = 512, 11 = 1024
//            (This seemed to match from another format but with only 512 byte
//             sample unable to verify)
//         7-6 flag 00 = data track, 01 = assigned alternate
//                  10 = defective, 11 = invalid
//         No example of alternate so not tested
//      byte 3 low 8 bits of cylinder
//      byte 4 sector number
//      byte 5 head number
//      bytes 6-9 32 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8 or 0xfb
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_CONVERGENT_AWS, CONVERGENT_AWS_SA1000
//   http://mightyframe.blogspot.com/2017/03/convergent-technologies-workstation.html
//
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2
//         7-4 head number
//         3-0 High 4 bits of cylinder
//      byte 3 low 8 bits of cylinder
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      256 bytes sector data
//      2 byte CRC/ECC code
//
//   CONTROLLER_WANG_2275
//      http://www.wang2200.org/docs.html
//      2275 Internal Memos documents have some information on spare
//      sector handling. Didn't find low level format. Seems to say uses
//      a WD2010 but format found doesn't match chip. Bad sector handling
//      is data table stored on disk. Not handled by this code.
//   5 byte header + 1 byte checksum
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. Bits 4-7 upper bits of cylinder
//      byte 4 sector number
//      bytes 5 8 bit checksum
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      24 bit CRC/ECC code
//
//   CONTROLLER_WANG_2275_B
//      Found on last 3 cylinders of WANG_2275 disk
//   5 byte header + 1 byte checksum
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-4 head number. Bits 5-7 upper bits of cylinder
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      16 bit CRC/ECC code
//      
//   CONTROLLER_CALLAN
//	aka "Callan Unistar"
//	aka "Liberty Bay Multibus model WDC-796-A"
//	Added by tjt 5-26-2022
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-3 head number. Bits 4-7 upper bits of cylinder
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      16 bit CRC/ECC code
//      
//   CONTROLLER_EDAX_PV9900
//      Quantum Q540 drive that appears to be from EDAX PV9900 EDS X-ray 
//      microanalysis system. From online informat disk controller is likely
//      from either AED Advanced Electronics Design or QBDC Q bus disk 
//      controller.
//
//   5 byte header + 2 byte crc
//      byte 0 0xa1. It looks like the upper two bits of the low 8 bits of
//         the cylinder is encoded in the sync byte so its not always the
//         normal 0xa1.
//      byte 1 cylinder low
//      byte 2 cylinder high
//      byte 3 sector number
//      byte 4 head
//      bytes 5-6 16 bit CRC 0xffff,0x1021,16
//   Data
//      byte 0 0xa1
//      Sector data for sector size
//      16 bit CRC/ECC code 0xffff,0x1021,16
//
//   CONTROLLER_DILOG_DQ614 (Distributed Logic)
//   Has label 570B1 A.
//   User manual without format info http://www.bitsavers.org/pdf/dilog/
//   ST-225-1.zip
//
//   8 byte header + 4 byte crc
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 Partition in low 5 bits? Bit 5 set on last logical sector
//          of track. Bit 6 set on last logical sector of cylinder. Bit 7
//          set on last locical sector of disk.
//      byte 3 0x33
//      byte 4 high cylinder 
//      byte 5 low cylinder
//      byte 6 head
//      byte 7 sector number. MSB set on last physcial sector of track.
//      bytes 8-11 32 bit CRC 0x58e07342,0x140a0445,32
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      32 bit CRC/ECC code 0xcf2105e0,0x140a0445,32
//   The controller supported partitioning the drive into multiple disks.
//   example has 0 and 2 in the partition field. Drive was labeled as drive 0
//   and 1. Second partition started at cyl 300. Cyl 301 head 0 had some
//   good sectors then rest of cylinder had sector headers but no data headers.
//   Unknow what this represents. Same on cyl 603. Cyl 604 on are a different
//   format and likely weren't used by the controller.
//
//   CONTROLLER_DILOG_DQ604 (Distributed Logic)
//   User manual without format info http://www.bitsavers.org/pdf/dilog/
//   emu-st412-dq604
//
//   8 byte header + 1 byte crc
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 Bit 1 set on last logical sector
//          of track. Bit 2 set on last logical sector of cylinder. Bit 3
//          set on last locical sector of disk.
//      byte 3 0x00
//      byte 4 high cylinder 
//      byte 5 low cylinder
//      byte 6 head
//      byte 7 sector number. 
//      byte 8 8 bit CRC 0x0,0x1,8
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size
//      8 bit CRC code 0x0,0x1,8
//
//   CONTROLLER_ALTOS
//      Found on Quantum Q2040 8" drive
//   4 byte header + 2 byte crc
//      byte 0 0xa1
//      byte 1 0xe0 upper 4 bits, low 4 bit upper 4 bits of cylinder
//      byte 2 low 5 bits of cylinder 7-3, 3 bit head  2-0
//      byte 3 sector number
//      bytes 4-5 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xb0
//      Sector data for sector size
//      16 bit CRC/ECC code
//
//   CONTROLLER_ROHM_PBX
//      Found on Quantum Q2040 8" drive
//   6 byte header + 2 byte crc
//      byte 0 0xa1
//      byte 1 0xa1
//      byte 2 0xa1
//      byte 3 Low 4 bits Head, bit 4 is a one.
//      byte 4 bits 0-5 Sector number. Bit 6-7 upper 2 bits of cylinder
//      byte 5 Low 8 bits of cylinder
//      bytes 6-7 16 bit CRC
//   Data (No 0xa1, marked by transitions from zeros to a one.)
//      byte 0 0xff
//      Sector data for sector size
//      24 bit CRC/ECC code
//
//   CONTROLLER_IBM_5288
//   5 byte header + 1 byte checksum
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 low 8 bits of cylinder
//      byte 3 bits 0-4 head number. Bits 5-7 upper bits of cylinder
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 0 0xa1
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      16 bit CRC/ECC code
//
//   CONTROLLER_IBM_3174
//   No manual found describing low level format. 3174.td
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 cylinder low
//      byte 3 cylinder high in upper 3 bits, low 5 bits head
//      byte 4 sector number
//      byte 5 unknown, 2 for sample I have
//      byte 6-7 CRC (0xfff,0x1021,16,0)
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      ECC code (2 byte)
//   Last 3 cylinders don't have valid data. Sector is 516 bytes. From
//      looking at the data the last 4 bytes may be extra data and not
//      part of the normal sector data.
//
//   CONTROLLER_SM1040
//   No manual found disk1tran
//   10 byte header + 4 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 flags, 
//          bits 0-1 upper 2 bits of head
//          bit 5 last physical sector on track
//          bit 6 last physical sector on cyl
//      byte 3 0x13
//      byte 4 high byte of cylinder
//      byte 5 low byte of cylinder
//      byte 6 head
//      byte 7 sector bits 0-6. Bit 7 last sector on track
//      byte 8 0x02
//      byte 9 0x64
//      byte 10-13 CRC (0xfff,0x1021,16,0)
//   Data
//      byte 0 0xa1
//      byte 1 0xfb
//      Sector data for sector size
//      ECC code (4 byte)
//
//   CONTROLLER_UNKNOWN2
//   Unknown what machine drive was used with. Drive was only low level
//   formatted.
//   Example SA1004_1.trans
//   6 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0x0b
//      byte 2 Lower 8 bits of cylinder
//      byte 3 Upper 8 bits of cylinder. Just a guess, drive was 256 cylinder
//         so field was always 0
//      byte 4 head
//      byte 5 sector number
//      bytes 6-7 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0x0d
//      Sector data for sector size
//      CRC/ECC code
//
//   CONTROLLER_SHUGART_SA1400
//   From General Processor Model T (T10). Controller is labeled Data Technology
//   without obvious part number. Board looks to match the SA1400 brochure
//   http://www.bitsavers.org/pdf/shugart/SA14xx/
//   Example sa1004_raw_data.
//   5 byte header + 2 byte CRC
//      byte 0 0xa1
//      byte 1 0xfe
//      byte 2 Lower 8 bits of cylinder. Drive only had 256 cylinders
//      byte 3 Head
//      byte 4 sector number
//      bytes 5-6 16 bit CRC
//   Data
//      byte 0 0xa1
//      byte 1 0xf8
//      Sector data for sector size. Bytes are inverted.
//      CRC/ECC code
//      
//   CONTROLLER_DJ_II
//      Format info 
//      http://bitsavers.org/pdf/konan/Konan_David_Junior_II_Reference_Aug83.pdf
//      image catnet1.6_micropolis_1304.trans
//      6 byte header standard data. See DJ_II code below
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
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status)
{
   static int sector_size;
   // Non zero if sector is a bad block, has alternate track assigned,
   // or is an alternate track
   static int bad_block, alt_assigned, is_alternate, alt_assigned_handled;
   static SECTOR_STATUS sector_status;
   // 0 after first sector marked spare/bad found. Only used for Adaptec 
   static int first_spare_bad_sector = 1;

   if (*state == PROCESS_HEADER) {
      // Clear these since not used by all formats
      alt_assigned = 0;
      alt_assigned_handled = 0;
      is_alternate = 0;
      bad_block = 0;

      memset(&sector_status, 0, sizeof(sector_status));
      sector_status.status |= init_status | SECT_HEADER_FOUND;
      sector_status.ecc_span_corrected_header = ecc_span;
      if (ecc_span != 0) {
         sector_status.status |= SECT_ECC_RECOVERED;
      }

      if (drive_params->controller == CONTROLLER_OMTI_5510) {
         sector_status.cyl = bytes[2]<< 8;
         sector_status.cyl |= bytes[3];

         // More is in here but what is not documented in manual
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4] & 0xf);
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
      } else if (drive_params->controller == CONTROLLER_MORROW_MD11 ||
           drive_params->controller == CONTROLLER_UNKNOWN1) {
         sector_status.cyl = bytes[3]<< 8;
         sector_status.cyl |= bytes[2];

         // More is in here but what is not documented in manual
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4] & 0xf);
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
      } else if (drive_params->controller == CONTROLLER_UNKNOWN2) {
         sector_status.cyl = bytes[3]<< 8;
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4]);
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[5];
         if (bytes[1] != 0x0b) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SHUGART_SA1400) {
         sector_status.cyl = bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3]);
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];
         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_DEC_RQDX3 &&
          !IsOutermostCylinder(drive_params, exp_cyl)) {
            // The last cylinder is in normal WD format. It has 256
            // byte sectors with a different polynomial so this fixes getting
            // the wrong cylinder which causes problems with reading. It still
            // won't be considered error free due to CRC error. TODO:
            // handle multiple formats for a disk so the entire disk can
            // be read without error. Other DEC controllers that use
            // same format don't seem to do this.
         sector_status.cyl = (bytes[3] & 0xf0) << 4;
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_status.sector = bytes[4];
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;
         // TODO: Figure out a better way to deal with sector number invalid
         // when bad block.
         bad_block = (sector_status.sector == 255);
         if (bad_block) {
            sector_status.status |= SECT_BAD_SECTOR_NUMBER | SECT_SPARE_BAD;
            // TODO: Print added here since count not properly updated due
            // to not knowing sector number
            msg(MSG_INFO,"Bad block set on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
         }
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
      } else if (drive_params->controller == CONTROLLER_DJ_II ) {
         // Controller David Junior II
         // Data format of a header is 4 bytes not including 0xa1, 0xfd
         // -------------------------------------------------------------------------
         // |   T7   |   T6   |   T5   |   T4   |   T3   |   T2   |   T1   |   T0   |
         // |   H2   |   H1   |   H0   |   S4   |   S3   |   S2   |   S1   |   S0   |
         // |~T7/~TB |~T6/~TA |~T5/~T9 |~T4/T4  |  ~T3   |  ~T2   |  ~T1   |  ~T0   | *
         // |  ~T8   |  ~H1   |  ~H0   |  ~S4   |  ~S3   |  ~S2   |  ~S1   |  ~S0   |
         // -------------------------------------------------------------------------

         // Data format of spare track header is 4 bytes not including 0xa1, 0xfd
         // -------------------------------------------------------------------------
         // |   T7   |   T6   |   T5   |   T4   |   T3   |   T2   |   T1   |   T0   |
         // |   T8   |    1   |    0   |    1   |    1   |   H2   |   H1   |   H0   |
         // |~T7/~TB |~T6/~TA |~T5/~T9 |~T4/T4  |  ~T3   |  ~T2   |  ~T1   |  ~T0   | *
         // |  ~T8   |    1   |    0   |    1   |    1   |  ~H2   |  ~H1   |  ~H0   |
         // -------------------------------------------------------------------------

         // Data format of a bad sector header is 8 bytes not including 0xa1, 0xfd
         // -------------------------------------------------------------------------
         // |   T7   |   T6   |   T5   |   T4   |   T3   |   T2   |   T1   |   T0   | New Track Low
         // |   T8   |    1   |    1   |    0   |    1   |   H2   |   H1   |   H0   | New T(,DH and  New Head
         // |~T7/~TB |~T7/~TA |~T6/~T9 |~T4/T4  |  ~T3   |  ~T2   |  ~T1   |  ~T0   | New Track Low Not *
         // |  ~T8   |    1   |    1   |    0   |    1   |  ~H2   |  ~H1   |  ~H0   | New ~T8,DH and New Head Not
         // |   T7   |   T6   |   T5   |   T4   |   T3   |   T2   |   T1   |   T0   | Current Track Low
         // |   H2   |   H1   |   H0   |   S4   |   S3   |   S2   |   S1   |   S0   | Current Head/Sector
         // |~T7/~TB |~T7/~TA |~T6/~T9 |~T4/T4  |  ~T3   |  ~T2   |  ~T1   |  ~T0   | Current Track Not*
         // |  ~T8   |  ~H1   |  ~H0   |  ~S4   |  ~S3   |  ~S2   |  ~S1   |  ~S0   | Cureent Track,Head,Sector Not
         // -------------------------------------------------------------------------

         // * Bit values following the slashes are for track >  d512

         int cyl_high;
         int hdr_offset, hdr_mask;

         sector_size = drive_params->sector_size; // Sectore size need from user
         bad_block = 0;
         
	 cyl_high = ((~bytes[5] ) >> 7) & 0x01; // Get ~T8 from byte 5 and invert it
         //Format change after cylinder d512. Can detect if T4 bit is not inverted in second copy.
         if (!((bytes[2] ^ bytes[4]) & 0x10)) {
	    cyl_high |= (((~bytes[4] ) >> 5) & 0x07) << 1;  //Get ~TB,~TA,~T9 from byte 4 and invert it
         }

         sector_status.cyl = cyl_high << 8;
         sector_status.cyl |= bytes[2]; // Adding low cyl

         // Bad block is 8 bytes long keeps the current track/sect head information in the last 4 bytes
         if (bytes[6] != 0x00 && bytes[7] != 0x00 && bytes[8] != 0x00){ 
            int new_cyl = sector_status.cyl;
            int new_head = bytes[3] & 7;

            bad_block=0;
            cyl_high = ((~bytes[9] ) >> 7) & 0x01;
            if (!((bytes[6] ^ bytes[8]) & 0x10)) {
               cyl_high |= (((~bytes[8] ) >> 5) & 0x07) << 1; 
            }
            sector_status.cyl = cyl_high << 8;
            sector_status.cyl |= bytes[6];
            sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[7] & 0xe0) >> 5 );
            sector_status.sector = (bytes[7] & 0x1f);

            mfm_handle_alt_track_ch(drive_params, sector_status.cyl, 
               sector_status.head, new_cyl, new_head);

            hdr_offset = 4;
            hdr_mask = 0x7f;
         } else {
            // Is it spare track header?
            if ((bytes[3] & 0x78) == 0x58 && (bytes[5] & 0x78) == 0x58) {
               sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[3] & 0x07));
               // No sector in header. Code needs sectors to be unique so we number them
               sector_status.sector = *sector_index; 
               hdr_mask = 0x7;
            } else {
               sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[3] & 0xe0) >> 5 );
               sector_status.sector = (bytes[3] & 0x1f) ; // Get sectors from byte 3
               hdr_mask = 0x7f;
            }
            hdr_offset = 0;
         }
         // Not bothering to check T4 bit since it changes at cyl >= 512
         if ((bytes[2 + hdr_offset] & 0x0f) != (~bytes[4 + hdr_offset] & 0x0f) || 
                ((bytes[3 + hdr_offset] & hdr_mask) != (~bytes[5 + hdr_offset] & hdr_mask))) {
            msg(MSG_INFO, "Header mismatch %x %x %x %x on cyl %d head %d sector %d\n",
               bytes[2], bytes[3], bytes[4], bytes[5],
               sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }

         if (bytes[1] != 0xfd) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MYARC_HFDC) {
         int ecc_size[] = { 4, -1, -1, -1,  -1, -1, -1 ,-1 ,
                           -1, -1, -1, -1,  -1,  7,  6,  5};
         sector_status.cyl = (bytes[3] & 0xf0) << 4;
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_status.sector = bytes[4];
         bad_block = bytes[5] >> 7;;
         sector_size = 128 << (bytes[5] & 0x7);
         if (drive_params->data_crc.length/8 != ecc_size[(bytes[5] & 0x70) >> 4]) {
            msg(MSG_INFO, "CRC size mismatch header byte %x on cyl %d head %d sector %d\n",
                  bytes[5], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }

         if (bytes[1] != (0xfe ^ (sector_status.cyl >> 8))) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SHUGART_1610) {
         sector_status.cyl = (bytes[3] & 0x70) << 4;
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0x7);
         sector_status.sector = bytes[4];
         sector_size = drive_params->sector_size;
         int flag = ((bytes[3] & 0x80) >> 6) | ((bytes[3] & 0x8) >> 3);
         bad_block = (flag == 2);
         alt_assigned = (flag == 3);
         if (alt_assigned) {
            // header has cyl and head of alternate track. Return cyl and
            // head of expected track to avoid error messages. Alt track
            // handling will swap the extracted data in the end.
            mfm_handle_alt_track_ch(drive_params, exp_cyl, exp_head,
              sector_status.cyl, sector_status.head);
            sector_status.cyl = exp_cyl;
            sector_status.head = exp_head;
            alt_assigned_handled = 1;
         }
         is_alternate = (flag == 1);
         if (is_alternate) {
            // header has cyl and head of track alternate of. Return cyl and
            // head of expected track.
            sector_status.cyl = exp_cyl;
            sector_status.head = exp_head;
         }

         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_IBM_3174) {
         sector_status.cyl = (bytes[3] & 0xe0) << 3;
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0x1f);
         sector_status.sector = bytes[4];
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;
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
      } else if (drive_params->controller == CONTROLLER_MVME320) {
         sector_status.cyl = (bytes[2] << 8) | bytes[3];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4]);
         sector_status.sector = bytes[5];
         // Don't know how/if these are encoded in header
         sector_size = drive_params->sector_size;
         bad_block = 0;
         // Don't know what's in this byte. Print a message so it can be
         // investigated if not the 2 seen previously.
         if (bytes[6] != 0x01) {
            msg(MSG_INFO, "Header Byte 6 not 1, byte %02x on cyl %d head %d sector %d\n",
                  bytes[6], sector_status.cyl, sector_status.head, sector_status.sector);
         }

         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d head %d sector %d\n",
                  bytes[1], sector_status.cyl, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MOTOROLA_VME10) {
         sector_status.cyl = (bytes[2] << 8) | bytes[3];
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4] >> 5);
         sector_status.sector = bytes[4] & 0x1f;
         sector_size = drive_params->sector_size;
         if (bytes[1] != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_WD_1006 ||
            drive_params->controller == CONTROLLER_RQDX2 || 
            drive_params->controller == CONTROLLER_NIXDORF_8870 || 
            drive_params->controller == CONTROLLER_TANDY_8MEG || 
            drive_params->controller == CONTROLLER_TANDY_16B || 
            drive_params->controller == CONTROLLER_ES7978 || 
            (drive_params->controller == CONTROLLER_DEC_RQDX3 && 
               IsOutermostCylinder(drive_params, exp_cyl)) ||
              drive_params->controller == CONTROLLER_WD_3B1) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         if (drive_params->controller == CONTROLLER_RQDX2) {
            sector_size = 512;
         } else {
            sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         } 
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4];
         // 3B1 with P5.1 stores 4th head bit in bit 5 of sector number field.
         if (drive_params->controller == CONTROLLER_WD_3B1) {
            sector_status.head = sector_status.head | ((sector_status.sector & 0xe0) >> 2);
            sector_status.sector &= 0x1f;
         }

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ISBC_214_128B || 
            drive_params->controller == CONTROLLER_ISBC_214_256B || 
            drive_params->controller == CONTROLLER_ISBC_214_512B || 
            drive_params->controller == CONTROLLER_ISBC_214_1024B) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4] & 0x3f;
         if ((bytes[4] & 0xc0) == 0x40) {
            is_alternate = 1;
         } else if ((bytes[4] & 0xc0) == 0x80) {
            alt_assigned = 1;
         } else if ((bytes[4] & 0xc0) != 0x00) {
            msg(MSG_INFO, "Invalid format type byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[4], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_TEKTRONIX_6130) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[3] & 0x7) | ((bytes[4] & 0x40) >> 3) );
         sector_status.sector = bytes[4] & 0x3f;

         sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         bad_block = (bytes[3] & 0x80) >> 7;

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

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MIGHTYFRAME ||
               drive_params->controller == CONTROLLER_DG_MV2000) {
         int sector_size_lookup[4] = {256, 512, 1024, 128};
         int cyl_high_lookup[16] = {0,1,2,3,-1,-1,-1,-1,4,5,6,7,-1,-1,-1,-1};
         int cyl_high;

         cyl_high = cyl_high_lookup[(bytes[1] & 0xf) ^ 0xe];
         sector_status.cyl = 0;
         if (cyl_high != -1) {
            sector_status.cyl = cyl_high << 8;
         }
         sector_status.cyl |= bytes[2];

         if (drive_params->controller == CONTROLLER_MIGHTYFRAME) {
            sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[3] & 0x7) | ((bytes[4] & 0x20) >> 2));
         } else { // DG MV/2000
            sector_status.head = mfm_fix_head(drive_params, exp_head, (bytes[3] & 0x7) | ((bytes[4] & 0x80) >> 4));
         }
         sector_size = sector_size_lookup[(bytes[3] & 0x60) >> 5];
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4] & 0x1f;

         if (cyl_high == -1) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_WANG_2275) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0xf0) << 4);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if ((drive_params->controller == CONTROLLER_WANG_2275_B) ||
        (drive_params->controller == CONTROLLER_IBM_5288)) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0xe0) << 3);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0x1f);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_CALLAN) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0xf0) << 4);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0x0f);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[4];
         bad_block = 0;

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_EDAX_PV9900) {
         sector_status.cyl = bytes[1] | (bytes[2] << 8);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[4]);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[3];

      } else if (drive_params->controller == CONTROLLER_DTC_520_256B ||
              drive_params->controller == CONTROLLER_DTC_520_512B ||
              drive_params->controller == CONTROLLER_DTC) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0x70) << 4);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_size = drive_params->sector_size;
         bad_block = (bytes[3] & 0x80) >> 7;

         sector_status.sector = bytes[4];

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ALTOS) {
         sector_status.cyl = ((bytes[1] & 0xf) << 5) | (bytes[2] >> 3);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[2] & 0x7);
         sector_size = drive_params->sector_size;

         sector_status.sector = bytes[3];

         if ((bytes[1] & 0xf0) !=  0xe0) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_MACBOTTOM ||
             drive_params->controller == CONTROLLER_CTM9016 ||
             drive_params->controller == CONTROLLER_FUJITSU_K_10R ||
             drive_params->controller == CONTROLLER_ACORN_A310_PODULE) {
         sector_status.cyl = bytes[2] | (bytes[1] << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3]);
         sector_status.sector = bytes[4];
         sector_size = drive_params->sector_size;
      } else if (drive_params->controller == CONTROLLER_ADAPTEC) {
         uint32_t lba_addr;
         lba_addr = (bytes[2] << 16) | (bytes[3] << 8) | bytes[4];
         sector_status.lba_addr = lba_addr;
         sector_status.is_lba = 1;

         // We can't determine what the actual cylinder sector and head is.
         // If we track rotation time we can take a guess at sector
         // TODO: Add this when driven by track format information
         sector_status.sector = *sector_index;
         sector_status.head = exp_head;
         sector_status.cyl = exp_cyl;

         if (lba_addr & 0x800000) {
            msg(MSG_DEBUG, "Sector marked bad/spare LBA %x on cyl %d head %d physical sector %d\n",
                  lba_addr, exp_cyl, exp_head, sector_status.sector);
            sector_status.status |= SECT_SPARE_BAD;
            sector_status.status |= SECT_BAD_LBA_NUMBER;
            if ((bytes[5] & 0xf) == 0x1 && first_spare_bad_sector) {
               drive_params->format_adjust = FORMAT_ADAPTEC_COUNT_BAD_BLOCKS;
            }
            first_spare_bad_sector = 0;
         }

//printf("lba %d sect %d head %d cyl %d\n", lba_addr, sector_status.sector,
//   sector_status.head, sector_status.cyl);
         sector_size = drive_params->sector_size;
         bad_block = 0;
         // If FORMAT_ADAPTEC_COUNT_BAD_BLOCKS all bits are used
         if (bytes[5] != 0 && bytes[5] != 0x40 && bytes[5] != 0x80 &&
             drive_params->format_adjust != FORMAT_ADAPTEC_COUNT_BAD_BLOCKS) {
            msg(MSG_INFO, "Unknown header flag byte %02x on cyl %d head %d physical sector %d\n",
                  bytes[5], exp_cyl, exp_head, sector_status.sector);
         }

         if (bytes[1] !=  0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d physical sector %d\n",
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

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] >> 4);
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
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[5]);
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

         sector_status.cyl = (REV_BYTE(bytes[8]) >> 4) | (REV_BYTE(bytes[9]) << 4);
         sector_status.head = mfm_fix_head(drive_params, exp_head, (REV_BYTE(bytes[7]) >> 6) | ((REV_BYTE(bytes[8]) & 0x3) << 2));
         sector_status.sector = REV_BYTE(bytes[7]) & 0x7;
         if ((bytes[7] & 0x1c) != 0 || (bytes[8] & 0x30) != 0 || 
               (bytes[10] & 0xfe) != 0 ) {
            msg(MSG_INFO,"Unexpected bits set %02x %02x %02x on cyl %d,%d head %d,%d sector %d\n",
                bytes[7], bytes[8], bytes[10], exp_cyl, sector_status.cyl,
                exp_head, sector_status.head, sector_status.sector);
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
               sector_status.head = mfm_fix_head(drive_params, exp_head, exp_head);
               sector_status.status |= SECT_BAD_HEADER;
               msg(MSG_INFO,"Spare sector used on cyl %d, head %d, physical sector %d\n",
                  sector_status.cyl, sector_status.head, *sector_index);
            } else {
               // Controller area only had sector and possibly cylinder
               sector_status.cyl = bytes[3];
               sector_status.head = mfm_fix_head(drive_params, exp_head, exp_head);
               sector_status.sector = bytes[4];
            }
         } else {
            uint8_t byte5 = bytes[5];
            uint8_t byte2 = bytes[2];

            sector_status.cyl = (((bytes[2] & 0xc0) << 2) | bytes[3]) + 1;
            sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[2] & 0xf);
            if (byte5 == 0x4) {
               msg(MSG_INFO, "Cylinder %d head %d assigned alternate cyl %d head %d. Extract data fixed\n",
                  exp_cyl, exp_head, sector_status.cyl, sector_status.head);
               byte5 = 0;
               mfm_handle_alt_track_ch(drive_params, exp_cyl, exp_head, 
                    sector_status.cyl, sector_status.head);
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
      } else if (drive_params->controller == CONTROLLER_ALTOS_586) {
         sector_status.cyl = bytes[2] | ((bytes[3] & 0x7) << 8);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] >> 4);
         sector_size = drive_params->sector_size;
         bad_block = (bytes[3] & 0x8) != 0;

         sector_status.sector = bytes[4];

         if (bytes[1]  != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ATT_3B2) {
         sector_status.cyl = bytes[2] | ((bytes[1] ^ 0xff) << 8);

         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3]);
         sector_size = drive_params->sector_size;
         bad_block = 0;

         sector_status.sector = bytes[4];

         if ((bytes[1] & 0xf0)  != 0xf0) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ISBC_215_128B ||
            drive_params->controller == CONTROLLER_ISBC_215_256B ||
            drive_params->controller == CONTROLLER_ISBC_215_512B ||
            drive_params->controller == CONTROLLER_ISBC_215_1024B) {
         sector_status.cyl = bytes[3] | ((bytes[2] & 0xf) << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[5]);
         sector_size = 128 << ((bytes[2] & 0x30) >> 4);
         sector_status.sector = bytes[4];
         is_alternate = (bytes[2] & 0xc0) == 0x40;
         alt_assigned = (bytes[2] & 0xc0) == 0x80;

         if (bytes[1]  != 0x19) {
            msg(MSG_INFO, "Invalid header id bytes %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SM_1810_512B) {
         sector_status.cyl = bytes[3] | ((bytes[2] & 0xf) << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[5]);
         sector_size = 128 << ((bytes[2] & 0x30) >> 4);
         sector_status.sector = bytes[4];
         is_alternate = (bytes[2] & 0xc0) == 0x40;
         alt_assigned = (bytes[2] & 0xc0) == 0x80;

         if (bytes[1]  != 0xfe) {
            msg(MSG_INFO, "Invalid header id bytes %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_DSD_5217_512B) {
         sector_status.cyl = bytes[3] | ((bytes[2] & 0xf) << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[5]);
         sector_size = 128 << ((bytes[2] & 0x30) >> 4);
         sector_status.sector = bytes[4];
         is_alternate = (bytes[2] & 0xc0) == 0x40;
         alt_assigned = (bytes[2] & 0xc0) == 0x80;
         if (bytes[1]  != 0xfe) {
            msg(MSG_INFO, "Invalid header id bytes %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_ROHM_PBX) {
         sector_status.cyl = bytes[5] | ((bytes[4] & 0xc0) << 2);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[3] & 0xf);
         sector_size = drive_params->sector_size;
         sector_status.sector = bytes[4] & 0x3f;

         if (bytes[1]  != 0xa1 || bytes[2] != 0xa1) {
            msg(MSG_INFO, "Invalid header id bytes %02x, %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], bytes[2], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_CONVERGENT_AWS ||
          drive_params->controller == CONTROLLER_CONVERGENT_AWS_SA1000) {
         sector_status.cyl = bytes[3] | ((bytes[2] & 0xf) << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[2] >> 4);
         sector_size = drive_params->sector_size;
         bad_block = 0;

         sector_status.sector = bytes[4];

         if ((bytes[1])  != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_DILOG_DQ614) {
         sector_status.cyl = bytes[5] | (bytes[4] << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[6]);
         sector_size = drive_params->sector_size;
         bad_block = 0;

         sector_status.sector = bytes[7] & 0x7f;
       
         static int last_byte2 = 0;
         if ((bytes[2] & 0x1f) != last_byte2) {
            last_byte2 = bytes[2];
            msg(MSG_INFO, "Possible start of partition %d at cyl %d head %d sector %d\n",
                  bytes[2], sector_status.cyl,
                  sector_status.head, sector_status.sector);
         }
         if ((bytes[3])  != 0x33) {
            msg(MSG_INFO, "Byte 3 %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[3], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
         if ((bytes[1])  != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_DILOG_DQ604) {
         sector_status.cyl = bytes[5] | (bytes[4] << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[6]);
         sector_size = drive_params->sector_size;
         bad_block = 0;

         sector_status.sector = bytes[7];
       
         if ((bytes[3])  != 0x00) {
            msg(MSG_INFO, "Byte 3 %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[3], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
         if ((bytes[1])  != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SM1040) {
         sector_status.cyl = bytes[5] | (bytes[4] << 8);
         sector_status.head = mfm_fix_head(drive_params, exp_head, bytes[6]);
         sector_size = drive_params->sector_size;
         bad_block = 0;

         sector_status.sector = bytes[7] & 0x7f;
       
         if ((bytes[9])  != 0x64) {
            msg(MSG_INFO, "Byte 9 %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[3], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
         if ((bytes[8])  != 0x02) {
            msg(MSG_INFO, "Byte 8 %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[3], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
         if ((bytes[3])  != 0x13) {
            msg(MSG_INFO, "Byte 3 %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[3], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
         if ((bytes[1])  != 0xfe) {
            msg(MSG_INFO, "Invalid header id byte %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else {
         msg(MSG_FATAL,"Unknown controller type %d\n",drive_params->controller);
         exit(1);
      }

      if (sector_status.is_lba) {
         msg(MSG_DEBUG,
            "Got LBA %d exp %d,%d cyl %d head %d sector %d,%d size %d bad block %d\n",
               sector_status.lba_addr, exp_cyl, exp_head, sector_status.cyl, 
               sector_status.head, sector_status.sector, *sector_index, 
               sector_size, bad_block);
      } else {
         msg(MSG_DEBUG,
            "Got exp %d,%d cyl %d head %d sector %d,%d size %d bad block %d\n",
               exp_cyl, exp_head, sector_status.cyl, sector_status.head, 
               sector_status.sector, *sector_index, sector_size, bad_block);
      }

      if (bad_block) {
         sector_status.status |= SECT_SPARE_BAD;
         msg(MSG_INFO,"Bad block set on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
      }
      if (is_alternate) {
         msg(MSG_INFO,"Alternate cylinder set on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
      }

      mfm_check_header_values(exp_cyl, exp_head, sector_index, sector_size,
            seek_difference, &sector_status, drive_params, sector_status_list);

      // The 3640 doesn't have a 0xa1 data header, search for its special sync
      if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         *state = MARK_DATA1;
      } else if (drive_params->controller == CONTROLLER_ROHM_PBX) {
         *state = MARK_DATA2;
      } else if (drive_params->controller == CONTROLLER_ALTOS_586 && bad_block) {
         // If bad block marked no data area is written
         *state = MARK_ID;
      } else {
         *state = MARK_DATA;
      }
   } else { // Data
      // Value and where to look for header mark byte
      int id_byte_expected = 0xf8;
      int id_byte_index = 1;
      int id_byte_mask = 0xff;

      sector_status.status |= init_status;

      if (drive_params->controller == CONTROLLER_MYARC_HFDC) {
         if (bytes[1] == 0xf8) {
            id_byte_expected = 0xf8;
         } else {
            id_byte_expected = 0xfb;
         }
      } else if (drive_params->controller == CONTROLLER_RQDX2) {
         if (bytes[1] == 0xf8) {
            id_byte_expected = 0xf8;
         } else {
            id_byte_expected = 0xfb;
         }
      } else if (drive_params->controller == CONTROLLER_DEC_RQDX3) {
         id_byte_expected = 0xfb;
      } else if (drive_params->controller == CONTROLLER_DJ_II) {
         id_byte_expected = 0xf8;
      } else if (drive_params->controller == CONTROLLER_IBM_3174) {
         id_byte_expected = 0xfb;
      } else if (drive_params->controller == CONTROLLER_MVME320) {
         id_byte_expected = 0xfb;
      } else if (drive_params->controller == CONTROLLER_ELEKTRONIKA_85) {
         id_byte_expected = 0x80;
      } else if (drive_params->controller == CONTROLLER_ROHM_PBX) {
         id_byte_expected = 0xff;
         id_byte_index = 0;
      } else if (drive_params->controller == CONTROLLER_ISBC_215_128B ||
            drive_params->controller == CONTROLLER_ISBC_215_256B ||
            drive_params->controller == CONTROLLER_ISBC_215_512B ||
            drive_params->controller == CONTROLLER_ISBC_215_1024B) {
         if (bytes[1] == 0x19) {
            id_byte_expected = 0x19; // Can use either 0x19 or 0xd9
            msg(MSG_INFO, "Data ID Byte 0x19. ext2emu won't generate files with this ID byte\n");
         } else {
            id_byte_expected = 0xd9;
         }
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3620) {
         if (bytes[2] != 0xf8) {
            msg(MSG_INFO, "Invalid data id bytes %02x on cyl %d,%d head %d,%d sector %d\n",
                  bytes[1], bytes[2], exp_cyl, sector_status.cyl,
                  exp_head, sector_status.head, sector_status.sector);
            sector_status.status |= SECT_BAD_HEADER;
         }
      } else if (drive_params->controller == CONTROLLER_SYMBOLICS_3640) {
         id_byte_expected = 0xf0;
         id_byte_index = 1;
         for (int i = 1; i < total_bytes; i++) {
            bytes[i] = REV_BYTE(bytes[i]);
         }
      } else if (drive_params->controller == CONTROLLER_SHUGART_SA1400) {
         for (int i = 2; i < total_bytes; i++) {
            bytes[i] = ~bytes[i];
         }
      } else if (drive_params->controller == CONTROLLER_UNKNOWN1) {
         id_byte_expected = sector_status.sector;
      } else if (drive_params->controller == CONTROLLER_UNKNOWN2) {
         id_byte_expected = 0x0d;
      } else if (drive_params->controller == CONTROLLER_WANG_2275) {
         id_byte_expected = 0xfb;
      } else if (drive_params->controller == CONTROLLER_WANG_2275_B) {
         id_byte_expected = 0xf8;
      } else if (drive_params->controller == CONTROLLER_CALLAN) {
         id_byte_expected = 0xf8;
      } else if (drive_params->controller == CONTROLLER_IBM_5288) {
         id_byte_expected = 0xfb;
         id_byte_index = 3;
      } else if (drive_params->controller == CONTROLLER_ALTOS) {
         id_byte_expected = 0xb0;
         id_byte_mask = 0xf0;
      } else if (drive_params->controller == CONTROLLER_EDAX_PV9900) {
         id_byte_expected = 0;
         id_byte_index = -1;
      } else if (drive_params->controller == CONTROLLER_DSD_5217_512B) {
         if (bytes[1] == 0xf8) {
            id_byte_expected = 0xf8;
         } else {
            id_byte_expected = 0xfb;
         }
      }
      if (id_byte_index != -1 &&
            (bytes[id_byte_index] & id_byte_mask) != id_byte_expected && 
            crc == 0) {
         msg(MSG_INFO,"Invalid data id byte %02x expected %02x on cyl %d head %d sector %d\n", 
               bytes[id_byte_index], id_byte_expected,
               sector_status.cyl, sector_status.head, sector_status.sector);
         sector_status.status |= SECT_BAD_DATA;
      }
      if ((drive_params->controller == CONTROLLER_ISBC_215_128B ||
           drive_params->controller == CONTROLLER_ISBC_215_256B ||
           drive_params->controller == CONTROLLER_ISBC_215_512B ||
           drive_params->controller == CONTROLLER_ISBC_215_1024B ||
           drive_params->controller == CONTROLLER_ISBC_214_256B ||
           drive_params->controller == CONTROLLER_ISBC_214_512B ||
           drive_params->controller == CONTROLLER_ISBC_214_1024B ||
           drive_params->controller == CONTROLLER_SM_1810_512B ||
           drive_params->controller == CONTROLLER_DSD_5217_512B)
           && alt_assigned) {
         // For defective tracks each sectors has repeating 4 byte sequence
         // Alt cyl high, alt cyl low, alt head?, 0x00. Entire track is always
         // reassigned. Only sample with alternate assigned was from iSBC 215
         // which head was 0 so couldn't verify head pulled from correct 
         // byte. Other controllers using this not verified.
         int last_acyl = -1, last_ahead = -1;
         int acyl,ahead;
         int i;
         // Find two identical values to ignore errors in data read
         // Data repeats every 4 bytes
         for (i = 2; i < 128; i += 4) {
            acyl = (bytes[i] << 8) + bytes[i+1];
            ahead = bytes[i+2];
            if (acyl == last_acyl && ahead == last_ahead) {
               mfm_handle_alt_track_ch(drive_params, sector_status.cyl, 
                 sector_status.head, acyl, ahead);
               break;
            }
            last_acyl = acyl;
            last_ahead = ahead;
         }
         if (i >= 128) {
            msg(MSG_ERR,"Unable to find alternate cylinder cyl %d head %d\n",
               sector_status.cyl, sector_status.head);
         }
         alt_assigned_handled = 1;
      }
      if (drive_params->controller == CONTROLLER_OMTI_5510 && alt_assigned) {
          mfm_handle_alt_track_ch(drive_params, sector_status.cyl, 
            sector_status.head, (bytes[2] << 8) + bytes[3], bytes[4]);
         alt_assigned_handled = 1;
      }
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
      if (alt_assigned && !alt_assigned_handled) {
         msg(MSG_INFO,"Assigned alternate cylinder not corrected on cyl %d, head %d, sector %d\n",
               sector_status.cyl, sector_status.head, sector_status.sector);
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
   float avg_bit_sep_time;     // 200 MHz clocks
   float nominal_bit_sep_time; // 200 MHz clocks
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
   // If we get too large a delta we need to process it in less than 32 bit
   // word number of bits. This holds remaining number to process
   int remaining_delta = 0;
   // Maximum delta to process in one pass
   int max_delta;
   // Intermediate value
   int tmp_raw_word;
   // Collect bytes to further process here
   uint8_t bytes[MAX_SECTOR_SIZE + 50];
   // How many we need before passing them to the next routine
   int bytes_needed = 0;
   int header_bytes_needed = 0;
   // Length to perform CRC over
   int bytes_crc_len = 0;
   int header_bytes_crc_len = 0;
   // how many we have so far
   int byte_cntr = 0;
   // Sequential counter for counting sectors
   int sector_index = 0;
   // Count all the raw bits for emulation file
   int all_raw_bits_count = 0;
   // Bit count of last of header found
   int header_raw_bit_count = 0;
   // Bit delta between last header and previous header
   int header_raw_bit_delta = 0;
   // First address mark time in ns 
   int first_addr_mark_ns = 0;

   num_deltas = deltas_get_count(0);
   raw_word = 0;
   nominal_bit_sep_time = 200e6 /
       mfm_controller_info[drive_params->controller].clk_rate_hz;
   max_delta = nominal_bit_sep_time * 22;
   avg_bit_sep_time = nominal_bit_sep_time;
   i = 1;
   while (num_deltas >= 0) {
      // We process what we have then check for more.
      for (; i < num_deltas;) {
         int delta_process;
         // If no remaining delta process next else finish remaining
         if (remaining_delta == 0) {
            delta_process = deltas[i++];
            remaining_delta = delta_process;
         }  else {
            delta_process = remaining_delta;
         }
         // Don't overflow our 32 bit word
         if (delta_process > max_delta) {
            delta_process = max_delta;
         }
         track_time += delta_process;
         // This is simulating a PLL/VCO clock sampling the data.
         clock_time += delta_process;
         remaining_delta -= delta_process;
         // Move the clock in current frequency steps and count how many bits
         // the delta time corresponds to
         for (int_bit_pos = 0; clock_time > avg_bit_sep_time / 2;
               clock_time -= avg_bit_sep_time, int_bit_pos++) {
         }
         // And then filter based on the time difference between the delta and
         // the clock. Don't update PLL if this is a long burst without
         // transitions
         if (remaining_delta == 0) {
            avg_bit_sep_time = nominal_bit_sep_time + filter(clock_time, &filter_state);
         }
#if 0
         if (cyl == 819 && head == 0) {
         printf
         ("  delta %d %d clock %.2f int bit pos %d avg_bit %.2f time %d\n",
               deltas[i], i, clock_time,
               int_bit_pos, avg_bit_sep_time, track_time);
         }
#endif
#if 0
         if (cyl == 23 && head == 3) {
            static int pass = 0;

            printf
               ("  DD %d,%d,%d,%.2f,%d,%.2f,%d\n", pass++,
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
            if ((raw_word & 0xffff) == 0x4489) {
   printf("mark at %d zero %d\n", tot_raw_bit_cntr, zero_count);
}
#endif
//printf("Raw %08x tot %d\n",raw_word, tot_raw_bit_cntr);
            int use_new_count = 0;  // Use new method
            int good_mark = 0;      // Non zero if proper location to look
            int delta = tot_raw_bit_cntr - header_raw_bit_count;

            // Only look at headers at the expected location from start of
            // track or from last header.
            if ((raw_word & 0xffff) == 0x4489) {
               CONTROLLER *cont = &mfm_controller_info[drive_params->controller];
               if (header_raw_bit_count == 0 && cont->first_header_min_bits != 0) {
                  use_new_count = 1;
                  if (tot_raw_bit_cntr >= cont->first_header_min_bits) {
                     good_mark = 1;
                  }
               }
               if (header_raw_bit_count != 0 && cont->header_min_delta_bits != 0) {
                  use_new_count = 1;
                  if (state == MARK_ID && delta >= cont->header_min_delta_bits) {
                     good_mark = 1; 
                  }
                  if (state == MARK_DATA && delta >= cont->data_min_delta_bits) {
                     good_mark = 1; 
                  }
               }
               // If not using new bit count method use number of zeros to 
               // determine if good mark
               if (!use_new_count && zero_count >= MARK_NUM_ZEROS) {
                  good_mark = 1;
               }
               //printf("Delta %d header %d tot %d use %d good %d\n", delta, header_raw_bit_count, tot_raw_bit_cntr, use_new_count, good_mark);
            }
            if ((drive_params->controller == CONTROLLER_EDAX_PV9900 &&
                    (((raw_word & 0xffff) == 0x4489 && state != MARK_ID) || 
                     ((raw_word & 0xfffff) == 0xa4891 && state == MARK_ID)) )  ||
               (drive_params->controller != CONTROLLER_EDAX_PV9900 && 
               (raw_word & 0xffff) == 0x4489 && good_mark) ) {
               if (first_addr_mark_ns == 0) {
                  first_addr_mark_ns = track_time * CLOCKS_TO_NS;
               }
               if (header_raw_bit_count != 0) {
                  header_raw_bit_delta = tot_raw_bit_cntr - header_raw_bit_count;
               }
               header_raw_bit_count = tot_raw_bit_cntr;
               zero_count = 0;
               bytes[0] = 0xa1;
               byte_cntr = 1;
              
               header_bytes_crc_len = mfm_controller_info[drive_params->controller].header_bytes + 
                        drive_params->header_crc.length / 8;
               if (drive_params->controller == CONTROLLER_DSD_5217_512B) {
                  header_bytes_needed = header_bytes_crc_len + 5;
               } else {
                  header_bytes_needed = header_bytes_crc_len + HEADER_IGNORE_BYTES;
               }
               if (state == MARK_ID) {
                  state = PROCESS_HEADER;
                  mfm_mark_header_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
                  // Figure out the length of data we should look for
                  bytes_crc_len = header_bytes_crc_len;
                  bytes_needed = header_bytes_needed;
               } else {
                  state = PROCESS_DATA;
                  mfm_mark_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
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
         } else if (state == MARK_DATA1) { // SYMBOLICS_3640
            if ((tot_raw_bit_cntr - header_raw_bit_count) > 530 && 
                  ((raw_word & 0xf) == 0x9)) {
               state = PROCESS_DATA;
               mfm_mark_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
               // Write sector assumes one sync byte at the start of the data
               // so we store the 0x01 sync byte.
               bytes[0] = 0x01;
               byte_cntr = 1;
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
               raw_bit_cntr = 10;
               decoded_word = 0;
               decoded_bit_cntr = 0;
               if (header_raw_bit_count != 0) {
                  header_raw_bit_delta = tot_raw_bit_cntr - header_raw_bit_count;
               }
               header_raw_bit_count = tot_raw_bit_cntr;
            }
         } else if (state == MARK_DATA2) {
//printf("DATA2 %x\n", raw_word);
            if ((raw_word & 0xf) == 0x9) {
               state = PROCESS_DATA;
               mfm_mark_location(all_raw_bits_count, 0, tot_raw_bit_cntr);
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
               byte_cntr = 0;
               raw_bit_cntr = 2;
               decoded_word = 0;
               decoded_bit_cntr = 0;
               if (header_raw_bit_count != 0) {
                  header_raw_bit_delta = tot_raw_bit_cntr - header_raw_bit_count;
               }
               header_raw_bit_count = tot_raw_bit_cntr;
            }
         } else { // PROCESS_DATA
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
                     // If sufficent bits we may have missed a header
                     // 7 is 2 MFM encoded bits and extra for fill fields.
                     // 8 is byte to bits, dup code below
                     if (byte_cntr == header_bytes_needed &&
                           header_raw_bit_delta > header_bytes_needed * 7 * 8) {
                        uint64_t crc;
                        int ecc_span;
                        SECTOR_DECODE_STATUS init_status = 0;

                        // Don't perform ECC corrections. They can be
                        // false corrections when not actually sector header.
                        mfm_crc_bytes(drive_params, bytes, 
                           header_bytes_crc_len, PROCESS_HEADER, &crc, 
                           &ecc_span, &init_status, 0);

                        // We will only get here if processing as data. If
                        // we found a header with good CRC switch to processing
                        // it as a header. Poly != 0 for my testing
                        if (crc == 0 && !(init_status & SECT_AMBIGUOUS_CRC) && drive_params->header_crc.poly != 0) {
//printf("Switched header %x\n", init_status);
                           mfm_mark_header_location(MARK_STORED, 0, 0);
                           mfm_mark_end_data(all_raw_bits_count, drive_params, cyl, head);
                           state = PROCESS_HEADER;
                           bytes_crc_len = header_bytes_crc_len;
                           bytes_needed = header_bytes_needed;
                           all_sector_status |= mfm_process_bytes(drive_params, bytes,
                              bytes_crc_len, bytes_needed, &state, cyl, head, 
                              &sector_index, seek_difference, sector_status_list, 0);
                              // Don't allow these bytes to be reprocessed below
                              byte_cntr = 0;
                        }
                     }
                     bytes[byte_cntr++] = decoded_word;
                     if (byte_cntr == header_bytes_needed && 
                           state == PROCESS_DATA) {
                        // Didn't find header so mark location previously found
                        // as data
                        mfm_mark_data_location(MARK_STORED, 0, 0);
                     }
                  } 
                  if (byte_cntr == bytes_needed) {
                     int force_bad = SECT_NO_STATUS;

                     // If data header too far from sector header mark bad.
                     // 7 is 2 MFM encoded bits and extra for fill fields.
                     // 8 is byte to bits, dup code above
                     if (state == PROCESS_DATA &&
                        header_raw_bit_delta > header_bytes_needed * 7 * 8) {
                        force_bad = SECT_BAD_DATA;
                        msg(MSG_DEBUG, "Ignored data too far from header %d, %d on cyl %d head %d sector index %d\n",
                          header_raw_bit_delta, header_bytes_needed * 7 * 8,
                          cyl, head, sector_index);
                     }
                     mfm_mark_end_data(all_raw_bits_count, drive_params, cyl, head);
                     all_sector_status |= mfm_process_bytes(drive_params, bytes,
                        bytes_crc_len, bytes_needed, &state, cyl, head, 
                        &sector_index, seek_difference, sector_status_list, 
                        force_bad);
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
   int bits = tot_raw_bit_cntr - 
           mfm_controller_info[drive_params->controller].track_words * 32;
   if (bits < -2000) { 
      msg(MSG_ERR, "Ran out of data on sector index %d.\n Track short %d bits from expected length.\n Either deltas lost or index pulse early\n",
         sector_index, bits);
   } else
   if (state == PROCESS_DATA && sector_index <= drive_params->num_sectors) {
      float begin_time = 
         ((bytes_needed - byte_cntr) * 16.0 *
             1e9/mfm_controller_info[drive_params->controller].clk_rate_hz
             + first_addr_mark_ns) / 2 + drive_params->start_time_ns;
      msg(MSG_ERR, "Ran out of data on sector index %d, try adding --begin_time %.0f to mfm_read command line\n",
         sector_index, round(begin_time / 1000.0) * 1000.0);
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
