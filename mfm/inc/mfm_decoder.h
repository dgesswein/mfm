#ifndef MFM_DECODER_H_
#define MFM_DECODER_H_
//
// 12/17/17 DJG Aded EDAX_PV9900
// 11/23/17 DJG Changed Wang 2275_B to CONT_MODEL so it won't get confused
//    with WD_1006
// 09/30/17 DJG Added support for Wang 2275
// 08/11/17 DJG Added support for Convergent AWS
// 05/19/17 DJG New sector state to indicate it hasn't been written.
// 04/21/17 DJG Allow --begin_time to override default values from analyze
// 03/08/17 DJG Fixed Intel iSBC 215 and added support for all sector lengths
// 02/12/17 DJG Added support for Data General MV/2000. Fix mfm_util
//    for Mightframe
// 02/09/17 DJG Added support for AT&T 3B2
// 02/07/17 DJG Added support for Altos 586 and adjusted start time for
//    Cromemco to prevent trying to read past end of track.
// 01/18/17 DJG Added 532 sector length for Sun Remarketing OMTI controller
//    for Lisa computer and --ignore_seek_errors option
// 01/06/17 DJG Don't consider SECT_SPARE_BAD unrecoverable error
// 12/11/16 DJG Added Intel iSBC_215 controller. Fix for Adaptec format
//    bad block handling. Handle sector contents which make CRC detection
//    ambiguous. 
// 11/20/16 DJG Add logic to allow emulator track length to be increased if
//    data won't fit. Found with Vector4. Most likly drive rotation speed
//    and pad bits distributed between sectors instead of grouped at end.
//    Change to allow copying extra data before the begining of the
//    sector needed for syncronization to make fixing of emu data work better. 
//    Changes to pass length of data for correcting emu file data
//    instead of trying to recalculate.
// 11/14/16 DJG Added Telenex Autoscope, Xebec S1420 and Vector4 formats
// 11/02/16 DJG Added metadata length field. Only Xerox 6085 uses
// 10/31/16 DJG Added extra header needed by Cromemco to make ext2emu
//    files work. Changes to handle Adaptec and sectors marked bad better
// 10/22/16 DJG Added unknown format found on ST-212 disk
// 10/16/16 DJG Renamed OLIVETTI to DTC. Added MOTOROLA_VME10 and SOLOSYSTEMS
//
//
// MFM is up to 32 for 256 byte sectors. This allows growth for RLL/ESDI
#define MAX_SECTORS 50
#define MAX_HEAD 16
#define MAX_CYL 4096
#define MAX_SECTOR_SIZE 10240 // Data size in bytes
// Max size of raw words for a track. This is big enough to
// hold future growth up to 30 Mbit/sec at 3600 RPM
#define MAX_TRACK_WORDS 16000

// Various convenience macros
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))
#define MAX(x,y) (x > y ? x : y)
#define MIN(x,y) (x < y ? x : y)
#define BIT_MASK(x) (1 << (x))


// Number of nanoseconds of each PRU clock
#define CLOCKS_TO_NS 5

// These are various statistics from reading the drive that are used to
// print a summary when finished.
typedef struct {
   int max_sect;
   int min_sect;
   int max_head;
   int min_head;
   int max_cyl;
   int min_cyl;
   int num_good_sectors;
   int num_bad_header;
   int num_bad_data;
   int num_spare_bad;
   int num_ecc_recovered;
   int num_retries;
   int max_ecc_span;
   int max_track_words;
   int emu_data_truncated;
} STATS;

typedef struct {
      // Address of bad sector
   int cyl;
   int head;
   int sector;
      // Non zero if last entry in list
   int last;
} MARK_BAD_INFO;

typedef struct alt_struct ALT_INFO;
struct alt_struct {
   ALT_INFO *next;
   int bad_offset;
   int good_offset;
   int length;
};

// This is the main structure defining the drive characteristics
typedef struct {
   // The number of cylinders, heads, and sectors per track
   int num_cyl;
   int num_head;
   int num_sectors;
   // The number of the first sector. Some disks start at 0 others 1
   int first_sector_number;
   // Size of data area of sector in bytes
   int sector_size;
   // CRC/ECC used for header and data area
   CRC_INFO header_crc, data_crc;
   // Track format
   // Update mfm_controller_info list below if enum changed.
   // ORDER IN THE TWO LISTS MUST MATCH
   // TODO, replace this with pointer to CONTROLLER entry
   enum {CONTROLLER_NONE, CONTROLLER_NEWBURYDATA,
      CONTROLLER_WD_1006, 
      CONTROLLER_WD_3B1,
      CONTROLLER_MOTOROLA_VME10, 
      CONTROLLER_DTC, CONTROLLER_MACBOTTOM, 
      CONTROLLER_ELEKTRONIKA_85,
      CONTROLLER_ALTOS_586,
      CONTROLLER_ATT_3B2,
      CONTROLLER_CONVERGENT_AWS,
      CONTROLLER_WANG_2275,
      CONTROLLER_WANG_2275_B,
      CONTROLLER_EDAX_PV9900,
      CONTROLLER_OMTI_5510, 
      CONTROLLER_XEROX_6085, 
      CONTROLLER_TELENEX_AUTOSCOPE, 
      CONTROLLER_MORROW_MD11,
      CONTROLLER_UNKNOWN1,
      CONTROLLER_DEC_RQDX3, 
      CONTROLLER_SEAGATE_ST11M,
      CONTROLLER_ISBC_215,
      CONTROLLER_ADAPTEC, 
      CONTROLLER_MVME320,
      CONTROLLER_SYMBOLICS_3620, CONTROLLER_SYMBOLICS_3640, 
      CONTROLLER_MIGHTYFRAME, 
      CONTROLLER_DG_MV2000, 
      CONTROLLER_SOLOSYSTEMS, 
      CONTROLLER_XEBEC_104786, 
      CONTROLLER_XEBEC_S1420, 
      CONTROLLER_EC1841, 
      CONTROLLER_CORVUS_H, CONTROLLER_NORTHSTAR_ADVANTAGE,
      CONTROLLER_CROMEMCO,
      CONTROLLER_VECTOR4,
      CONTROLLER_VECTOR4_ST506
   } controller;
   // The sector numbering used. This will vary from the physical order if
   // interleave is used. Only handles all sectors the same.
   uint8_t *sector_numbers;
   // DRIVE_STEP_FAST or DRIVE_STEP_SLOW from drive.h
   int step_speed;
   // Stats structure above
   STATS stats;
   // Files to write extracted sector data, emulator data file, and 
   // raw transitions data to. Null if file should not be written.
   char *extract_filename;
   char *emulation_filename;
   char *transitions_filename;
   // 1 if emulation file is output
   int emulation_output;
   // One WD controller truncated the head number to 3 bits in the header. This
   // enables processing for that.
   int head_3bit;
   // Number of retries to do when an error is found reading the disk
   int retries;
   // Number of retries to do without seeking
   int no_seek_retries;
   // Disables some header checks. Not currently used.
   int ignore_header_mismatch;
   // Drive number for setting select lines
   int drive;
   // Command line for driver parameters. Null if not set
   char *cmdline;
   // Input/output files
   int tran_fd;
   int emu_fd;
   int ext_fd;
   int metadata_fd;
   TRAN_FILE_INFO *tran_file_info;
   EMU_FILE_INFO *emu_file_info;
   // Size of data for each emulator track. Only valid when writing
   // emulator file. TODO, use emu_file_info field instead
   int emu_track_data_bytes;
   // non zero if analyze option set
   int analyze;
   // non zero if in process of performing format analyze.
   int analyze_in_progress;
   // What cylinder and head to analyze
   int analyze_cyl;
   int analyze_head;
   // What options have been set. Used by command line parsing and validation
   uint32_t opt_mask;
   // Command line note parameter
   char *note;
   // Time after index to start read in nanoseconds
   uint32_t start_time_ns;
   // Non zero if begin_time option set on command line. Don't override
   int start_time_set_cmd_line;
   // List of sector to mark bad in ext2emu. Sorted ascending
   MARK_BAD_INFO *mark_bad_list;
   // Index for next entry in array above
   int next_mark_bad;
   // Linked list of alternate tracks for fixing extracted data file
   ALT_INFO *alt_llist;
   // Cylinder to start write precompensation at
   int write_precomp_cyl;
   // Precompensation time in nanoseconds
   int early_precomp_ns;
   int late_precomp_ns;
   // If we detect special cases of the format durring running we
   // set them here. ADAPTEC_COUNT_BAD_BLOCKS is when bits 0 to at least
   // 5
   enum {FORMAT_NONE, FORMAT_ADAPTEC_COUNT_BAD_BLOCKS} format_adjust;
   // Non zero if seek errors should be ignored
   int ignore_seek_errors;
} DRIVE_PARAMS;

// This isn't clean programming but keeps it together with structure above so
// hopefully they stay in sync.
// Update controller enum above if list changed.

#ifndef DEF_DATA
#define DEF_EXTERN extern
#else
#define DEF_EXTERN
#endif

// These are the formats we will search through.
DEF_EXTERN struct {
   uint64_t poly;
   int length;
   int ecc_span;
} mfm_all_poly[]
#ifdef DEF_DATA
   = {
  // Length 1 for parity (Symbolics 3640. Doesn't really use length
  {0, 1, 0},
  // Length 16 for Northstar header checksum
  {0, 16, 0},
  // Length 32 for Northstar data checksum
  {0, 32, 0},
  // Length 8 for Wang header checksum
  {0, 8, 0},
  {0x00a00805, 32, 5},
  // Don't move this without fixing the Northstar reference
  {0x1021, 16, 0},
  {0x8005, 16, 0},
  {0x140a0445, 32, 5},
  {0x140a0445000101, 56, 16}, // From WD42C22C datasheet, not tested
  {0x0104c981, 32, 5},
  {0x24409, 24, 0},
  {0x3e4012, 24, 0}, // WANG 2275
  // Adaptec bad block on Maxtor XT-2190
  {0x41044185, 32, 5},
  // MVME320 controller
  {0x10210191, 32, 5}
  // From uPD7261 datasheet. Also has better polynomials so commented out
  //{0x1, 16, 0} 
  // From 9410 CRC checker. Not seen on any drive so far
  //{0x4003, 16, 0} 
  //{0xa057, 16, 0} 
  //{0x0811, 16, 0} 
}
#endif
;

DEF_EXTERN struct {
   int length; // -1 indicates valid for all polynomial size  
   uint64_t value;
}  mfm_all_init[]
#ifdef DEF_DATA
 = 
   {{-1, 0}, {-1, 0xffffffffffffffff}, {32, 0x2605fb9c}, {32, 0xd4d7ca20},
     {32, 0x409e10aa},
     // This is 532 byte sector OMTI. Above are other OMTI. They likely are
     // compensating for something OMTI is doing to the CRC like below
     // TODO: Would be good to find out what. File sun_remarketing/kalok*
     {32, 0x84a36c27},

     // These are for iSBC_215. The final CRC is inverted but special
     // init value will also make it match
     // TODO Add xor to CRC to allow these to be removed
     // header
     {32, 0xed800493},
     // 128 byte sector
     {32, 0xec1f077f},
     // 256 byte sector
     {32, 0xde60050c},
     // 512 byte sector
     {32, 0x03affc1d},
     // 1024 byte sector
     {32, 0xbe87fbf4},
     // This is data area for Altos 586. Unknown why this initial value needed.
     {16, 0xe60c},
     // WANG 2275 with all header bytes in CRC
     {24, 0x223808}
  }
#endif
;
// Smallest sector size should be first in list
DEF_EXTERN int mfm_all_sector_size[]
#ifdef DEF_DATA
 = {128, 256, 512, 524, 532, 1024, 1160, 1164, 2048, 4096, 10240, -1}
  // -1 marks end of array
#endif
;

// Number of sectors to search for LBA format
DEF_EXTERN int mfm_lba_num_sectors[]
#ifdef DEF_DATA
 = {17, 18, 32, 33, -1}
  // -1 marks end of array
#endif
;

// This defines fields that aren't whole multiples of bytes.
typedef struct bit_l {
      // Start bit, bits are numbered with most significant bit of the
      // first byte 0
   int bitl_start;
      // Field length in bits
   int bitl_length;
} BIT_L;

// This defines the fields in sector header and data areas
typedef struct field_l {
      // Length of field in bytes. If defined by bit fields set to 0
   int len_bytes;
      // Type of field,
      // FIELD_FILL fills the specified number of bytes with value
      // FIELD_A1 writes the A1 header/data mark code. Only 1 byte valid.
      // FIELD_CYL, HEAD, SECTOR, LBA write the current cylinder, head, sector,
      //   or logical block address. Valid for byte of bit fields.
      // FIELD_HDR_CRC and FIELD_DATA_CRC write the check word. The
      //    CONTROLLER data defines the type of check word.
      // FIELD_MARK_CRC_START and END are used to mark the start and end
      // byte for CRC (check data) calculation. The default
      // includes the all the data from sector start flag byte (a1 etc) to 
      // the CRC.
   enum {FIELD_FILL, FIELD_A1, FIELD_CYL, FIELD_HEAD, FIELD_SECTOR,
      FIELD_LBA, FIELD_HDR_CRC, FIELD_DATA_CRC, FIELD_SECTOR_DATA, 
      FIELD_MARK_CRC_START, FIELD_MARK_CRC_END,
      FIELD_BAD_SECTOR,
         // These are for controllers that need special handling
      FIELD_HEAD_SEAGATE_ST11M, FIELD_CYL_SEAGATE_ST11M,
         // Mark end of sector to increment sector counter
      FIELD_NEXT_SECTOR
      } type;
      // Value for field. 
   uint8_t value;
      // OP_SET writes the data to the field, OP_XOR exclusive or's it
      // with the current contents, OP_REVERSE reverses the bits then does
      // an OP_SET.
   enum {OP_SET, OP_XOR, OP_REVERSE} op;
      // If bit_list is null is is the byte from start of field.
      // If bit_list is not null this is the length of the field in bits.
   int byte_offset_bit_len;
      // The list of bits in field if not null.
   BIT_L *bit_list;
} FIELD_L;

// This defines all the data in the track. Each operation starts at the
// end of the previous one.
typedef struct trk_l {
      // The count for the field. For TRK_FIll and TRK_FIELD its the number 
      // of bytes,
      // for TRK_SUB its the number of times the specificied list should be
      // repeated.
   int count;
      // TRK_FILL fills the specified length of bytes. 
   enum {TRK_FILL, TRK_SUB, TRK_FIELD} type;
      // Only used for TRK_FILL.
   uint8_t value;
      // Pointer to a TRK_L for TRK_SUB or FIELD_L for TRK_FIELD
   void *list;
} TRK_L;

// *** NOTE: These define the track starting from start_time_ns ****
//
// For more information on the track formats see the *decoder.c files.
//
// TODO, use these tables to drive reading the data also instead of the
// current hard coded data.
// Will likely need a separate op for reading.
//
// Format for AT&T 3B1 computer
DEF_EXTERN TRK_L trk_3B1[] 
#ifdef DEF_DATA
 = 
{ { 45, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {15, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              // This adds upper 3 bits of cylinder to bits 3,1,0 of
              // the 0xfe byte and the rest in the next bit. The cylinder
              // bits are xored with the 0xfe. Xor with 0 just sets the bits
              {0, FIELD_CYL, 0x00, OP_XOR, 11, 
                 (BIT_L []) {
                    { 12, 1},
                    { 14, 10},
                    { -1, -1},
                 }
              },
              // Sector size 512
              {1, FIELD_FILL, 0x20, OP_SET, 3, NULL},
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {516, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {38, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {275, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// From http://www.mirrorservice.org/sites/www.bitsavers.org/pdf/sms/asic/OMTI_5050_Programmable_Data_Sequencer_Jun86.pdf
// Appendix A
DEF_EXTERN TRK_L trk_omti_5510[] 
#ifdef DEF_DATA
 = 
{ { 11, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {12, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {2, FIELD_CYL, 0x00, OP_SET, 2, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 4, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {14, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {14, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {715, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// From http://www.mirrorservice.org/sites/www.bitsavers.org/pdf/sms/asic/OMTI_5050_Programmable_Data_Sequencer_Jun86.pdf
// Appendix A
DEF_EXTERN TRK_L trk_mvme320[] 
#ifdef DEF_DATA
 = 
{ { 20, TRK_FILL, 0x4e, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {12, TRK_FILL, 0x00, NULL},
        {9, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {2, FIELD_CYL, 0x00, OP_SET, 2, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 4, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 5, NULL},
              {1, FIELD_FILL, 0x01, OP_SET, 6, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 7, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {4, TRK_FILL, 0x4e, NULL},
        {12, TRK_FILL, 0x00, NULL},
        {262, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfb, OP_SET, 1, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {350, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// From looking at data read from a drive
DEF_EXTERN TRK_L trk_symbolics_3640[] 
#ifdef DEF_DATA
 = 
{ { 8, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {47, TRK_FILL, 0x00, NULL},
        {11, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x5a, OP_SET, 1, NULL},
              {1, FIELD_FILL, 0x96, OP_SET, 2, NULL},
              {1, FIELD_FILL, 0x0e, OP_SET, 3, NULL},
              {1, FIELD_FILL, 0x0e, OP_SET, 4, NULL},
              {1, FIELD_FILL, 0x9e, OP_SET, 5, NULL},
              {1, FIELD_FILL, 0x01, OP_SET, 6, NULL},
              {0, FIELD_SECTOR, 0x00, OP_REVERSE, 3, 
                 (BIT_L []) {
                    { 56, 3},
                    { -1, -1},
                 }
              },
              {0, FIELD_HEAD, 0x00, OP_REVERSE, 4, 
                 (BIT_L []) {
                    { 62, 4},
                    { -1, -1},
                 }
              },
              {0, FIELD_CYL, 0x00, OP_REVERSE, 12, 
                 (BIT_L []) {
                    { 68, 12},
                    { -1, -1},
                 }
              },
              {1, FIELD_HDR_CRC, 0x00, OP_SET, 10, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {25, TRK_FILL, 0x00, NULL},
        {1166, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_FILL, 0x01, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xe0, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {1160, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 1162, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {49, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {32, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;


// From looking at data read from a drive and manual. Commented out values
// are what the manual specified which didn't work and didn't match captured
// data. The controller could read data written with this format but got
// errors after it wrote to a track.
DEF_EXTERN TRK_L trk_northstar[] 
#ifdef DEF_DATA
 = 
//{ { 44, TRK_FILL, 0xff, NULL },
{ { 69, TRK_FILL, 0xff, NULL },
  { 3, TRK_FILL, 0x55, NULL },
  //{ 40, TRK_FILL, 0xff, NULL },
  { 8, TRK_FILL, 0xff, NULL },
  { 16, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {67, TRK_FILL, 0x00, NULL},
// This is somewhat inconsistant. The Symbolics 3640 needs the 1 as part of the
// header, Northstar code assumes it is not.
        { 1, TRK_FILL, 0x01, NULL},
        { 525, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_SECTOR, 0x00, OP_SET, 0, NULL},
              {0, FIELD_CYL, 0x00, OP_SET, 12, 
                 (BIT_L []) {
                    { 0, 4},
                    { 8, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_SET, 2, NULL},
              {4, FIELD_FILL, 0x00, OP_SET, 3, NULL},
              {1, FIELD_FILL, 0xff, OP_SET, 8, NULL},
              {1, FIELD_HDR_CRC, 0x00, OP_SET, 7, NULL},
              {1, FIELD_HDR_CRC, 0x00, OP_XOR, 8, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 9, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 9, NULL},
              {0, FIELD_MARK_CRC_END, 0, OP_SET, 520, NULL},
              {2, FIELD_FILL, 0xff, OP_SET, 523, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 521, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_XOR, 523, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        //{45, TRK_FILL, 0x00, NULL},
        {49, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   //{121, TRK_FILL, 0xff, NULL},
   {64, TRK_FILL, 0xff, NULL},
   {-1, 0, 0, NULL}
}
#endif
;

DEF_EXTERN TRK_L trk_seagate_ST11M[] 
#ifdef DEF_DATA
 = 
{ { 19, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {10, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {1, FIELD_HEAD_SEAGATE_ST11M, 0x00, OP_SET, 2, NULL},
                 // On first cylinder byte 2 is 0xff. This is set by 
                 // FIELD_HEAD_SEAGATE_ST11M. The XOR prevents clearing
                 // the upper 2 bits
              {0, FIELD_CYL_SEAGATE_ST11M, 0x00, OP_XOR, 10, 
                 (BIT_L []) {
                    { 16, 2},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 5, NULL}, // Spare flags
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {20, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {622, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_cromemco_stdc[] 
#ifdef DEF_DATA
 = 
{ { 40, TRK_FILL, 0x00, NULL },
  { 1, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {5, TRK_FIELD, 0x00,  // Track header
           (FIELD_L []) {
              {1, FIELD_FILL, 0x04, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xaa, OP_SET, 1, NULL},
              {0, FIELD_CYL, 0x00, OP_SET, 16,  //6
                 (BIT_L []) {
                    { 24, 8}, // High byte
                    { 16, 8}, // Low byte
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_SET, 4, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {75, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   { 1, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {10258, TRK_FIELD, 0x00,  // All bytes in CRC must be in TRK_FIELD
           (FIELD_L []) {
              {1, FIELD_FILL, 0x04, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 1, NULL},
              {3, FIELD_FILL, 0xaa, OP_SET, 2, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 5, NULL},
              {0, FIELD_CYL, 0x00, OP_SET, 16,  //6
                 (BIT_L []) {
                    { 56, 8}, // High byte
                    { 48, 8}, // Low byte
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_SET, 8, NULL},
              {10240, FIELD_SECTOR_DATA, 0x00, OP_SET, 9, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 10249, NULL},
              {2, FIELD_FILL, 0xaa, OP_SET, 10250, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 10252, NULL},
              {0, FIELD_CYL, 0x00, OP_SET, 16,  //10253
                 (BIT_L []) {
                    { 82032, 8},
                    { 82024, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_SET, 10255, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 10256, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {4, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {36, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;
typedef enum {CHECK_CRC, CHECK_CHKSUM, CHECK_PARITY} CHECK_TYPE;

typedef struct {
   char *name;
      // Sector size needs to be smallest value to prevent missing next header
      // for most formats. Some controller formats need the correct value.
   int analyze_sector_size;
      // Rate of MFM clock & data bit cells
   uint32_t clk_rate_hz;
      // Delay from index pulse to we should start capturing data in
      // nanoseconds. Needed to ensure we start reading with the first
      // physical sector
   uint32_t start_time_ns;

   int header_start_poly, header_end_poly;
   int data_start_poly, data_end_poly;

   int start_init, end_init;
   enum {CINFO_NONE, CINFO_CHS, CINFO_LBA} analyze_type;

      // Size of headers not including checksum
   int header_bytes, data_header_bytes; 
      // These bytes at start of header and data header ignored
   int header_crc_ignore, data_crc_ignore;
   CHECK_TYPE header_check, data_check;

      // These bytes at the end of the data area are included in the CRC
      // but should not be written to the extract file.
   int data_trailer_bytes;
      // 1 if data area is separate from header. 0 if one CRC covers both
   int separate_data;
      // Layout of track.
   TRK_L *track_layout;
      // Sector size to use for converting extract to emulator file
   int write_sector_size;
      // And number of sectors per track
   int write_num_sectors;
      // And number of sectors per track
   int write_first_sector_number;
      // Number of 32 bit words in track MFM data
   int track_words;

      // Non zero of drive has metadata we which to extract.
   int metadata_bytes;
      // Number of extra MFM 32 bit words to copy when moving data around
      // to fix read errors. Formats that need a bunch of zero before
      // a one should use this to copy the zeros.
   int copy_extra;

      // Check information
   CRC_INFO write_header_crc, write_data_crc;

      // Analize is use full search on this format. Model is use
      // the specific data only.
      // TODO: Analize should search model for specific models before
      // doing generic first. We may want to switch this to bit mask if
      // some we will use as specific model then try search with different
      // polynomials.
   enum {CONT_ANALIZE, CONT_MODEL} analyze_search;
} CONTROLLER;

DEF_EXTERN CONTROLLER mfm_controller_info[]
// Keep sorted by header length. MUST MATCH order of controller enum
#ifdef DEF_DATA
   = {
      {"CONTROLLER_NONE",        0, 10000000,      0, 
         0,0, 0,0,
         0,0, CINFO_NONE,
         0, 0, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, 0 },
      {"NewburyData",          256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE},
      {"WD_1006",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"WD_3B1",          512, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_3B1, 512, 17, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL },
      {"Motorola_VME10",  256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0,0xa00805,32,0},{0,0xa00805,32,0}, CONT_ANALIZE },
      {"DTC",             256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"MacBottom",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Elektronika_85",      256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         16, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Altos_586",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"ATT_3B2",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"CONVERGENT_AWS",       256, 10000000, 460000, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"WANG_2275",              256, 10000000,      0, 
         3, 4, 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 0, CHECK_CHKSUM, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"WANG_2275_B",            256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_MODEL },
      {"EDAX_PV9900",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 1, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"OMTI_5510",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_omti_5510, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,5},{0xd4d7ca20,0x104c981,32,5}, CONT_ANALIZE },
      {"Xerox_6085",           256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         20, 0,
         { 0x2605fb9c,0x104c981,32,5},{0xd4d7ca20,0x104c981,32,5}, CONT_ANALIZE },
      {"Telenex_Autoscope",           256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,5},{0xd4d7ca20,0x104c981,32,5}, CONT_ANALIZE },
      {"Morrow_MD11",            1024, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 1024, 9, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,5},{0xd4d7ca20,0x104c981,32,5}, CONT_ANALIZE },
      {"Unknown1",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,5},{0xd4d7ca20,0x104c981,32,5}, CONT_ANALIZE },
// OMTI_5200 uses initial value 0x409e10aa for data
      {"DEC_RQDX3",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Seagate_ST11M",        256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_seagate_ST11M, 512, 17, 0, 5209,
         0, 0,
         {0x0,0x41044185,32,5},{0x0,0x41044185,32,5}, CONT_ANALIZE },
      {"Intel_iSBC_215",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
//TODO, this won't analyze properly
      {"Adaptec",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_LBA,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"MVME320",        256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_mvme320, 256, 32, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x10210191,32,5}, CONT_ANALIZE },
      {"Symbolics_3620",       256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 3, 3, 3, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
// Should be model after data filled in
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Symbolics_3640",       256, 10000000,      0, 
         0, 1, 4, ARRAYSIZE(mfm_all_poly), 
         0, 1, CINFO_CHS,
         11, 2, 0, 2, CHECK_PARITY, CHECK_CRC,
         0, 1, trk_symbolics_3640, 1160, 8, 0, 5209,
         0, 0,
         {0x0,0x0,1,0},{0x0,0xa00805,32,5}, CONT_MODEL },
// This format is detected by special case code so it doesn't need to
// be sorted by number. It should not be part of a normal search
// since it will match wd_1006 for drives less than 8 heads
// CONT_MODEL is currently doing TODO: when model support
// added revisit this
      {"Mightyframe",          256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_MODEL },
// This format is detected by special case code so it doesn't need to
// be sorted by number. It should not be part of a normal search
// since it will match wd_1006 for drives less than 8 heads
// CONT_MODEL is currently doing TODO: when model support
// added revisit this
      {"DG_MV2000",          256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_MODEL },
// END of WD type controllers
//    Changed begin time from 0 to 100500 to work with 1410A. The sample
//    I have of the 104786 says it should work with it also so changing default.
//    Its possible this will cause problems with other variants.
      {"SOLOsystems",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Xebec_104786",         256, 10000000,      100500,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Xebec_S1420",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"EC1841",         256, 10000000,      220000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"Corvus_H",             512, 11000000,  312000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         3, 3, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 0, 0, 0, 5209,
// Should be model after data filled in
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALIZE },
      {"NorthStar_Advantage",  256, 10000000, 230000,
         1, 2, 2, 3,
         0, 1, CINFO_CHS,
         7, 0, 0, 0, CHECK_CHKSUM, CHECK_CHKSUM,
         0, 1, trk_northstar, 512, 16, 0, 5209,
// Should be model after data filled in
         0, 33,
         {0,0,16,0},{0,0,32,0}, CONT_ANALIZE },
      {"Cromemco",             10240, 10000000,  6000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 9, 0, 0, CHECK_CRC, CHECK_CRC,
         7, 0, trk_cromemco_stdc, 10240, 1, 0, 5209,
// Should be model after data filled in
         0, 0,
         {0,0x8005,16,0},{0,0x8005,16,0}, CONT_ANALIZE },
      {"Vector4",             256, 10000000,  300000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 4, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 256, 32, 0, 5209,
// Should be model after data filled in
         0, 20,
         {0x0,0x104c981,32,5},{0x0,0x104c981,32,5}, CONT_ANALIZE },
      {"Vector4_ST506",             256, 10000000,  300000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 4, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 256, 32, 0, 5209,
// Should be model after data filled in
         0, 20,
         {0x0,0x104c981,32,5},{0x0,0x104c981,32,5}, CONT_ANALIZE },
      {NULL, 0, 0, 0,
         0, 0, 0, 0,
         0,0, CINFO_NONE,
         0, 0, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 0, 0, 0, 0,
         0, 0,
         {0,0,0,0},{0,0,0,0}, 0 }
   }
#endif
;

// The possible states from reading each sector.

// These are ORed into the state
// If this is set the data being CRC'd is zero so the zero CRC
// result is ambiguous since any polynomial will match
#define SECT_AMBIGUOUS_CRC 0x800
// If set treat as error for analyze but otherwise ignore it
#define SECT_ANALYZE_ERROR 0x400
// Set if the sector number is bad when BAD_HEADER is not set.
// Some formats use bad sector numbers to flag bad blocks
#define SECT_BAD_LBA_NUMBER 0x200
#define SECT_BAD_SECTOR_NUMBER 0x100
// This is used to mark sectors that are spare sectors or are marked
// bad and don't contain user data. 
// It suppresses counting as errors other errors seen.
#define SECT_SPARE_BAD      0x100
#define SECT_ZERO_HEADER_CRC 0x80
#define SECT_ZERO_DATA_CRC  0x40
#define SECT_HEADER_FOUND   0x20
#define SECT_ECC_RECOVERED  0x10
#define SECT_WRONG_CYL      0x08
// Sector hasn't been written yet
#define SECT_NOT_WRITTEN    0x04
// Only one of these three will be set. BAD_HEADER is initially set
// until we find a good header, then BAD_DATA is set until we find good data
#define SECT_BAD_HEADER     0x02
#define SECT_BAD_DATA       0x01
#define SECT_NO_STATUS      0x00
typedef uint32_t SECTOR_DECODE_STATUS;
#define UNRECOVERED_ERROR(x) ((x & (SECT_BAD_HEADER | SECT_BAD_DATA)) && !(x & SECT_SPARE_BAD))

// The state of a sector
typedef struct {
   // The span of any ECC correction. 0 if no correction
   int ecc_span_corrected_data;
   int ecc_span_corrected_header;
   // The difference between the expected and actual cylinder found
   int cyl_difference;
   // The values from the sector header
   int cyl, head, sector;
   // Non zero if drive is LBA
   int is_lba;
   // The logical block address for LBA drives.
   int lba_addr;
   // A sequential count of sectors starting from 0
   // Logical sector will only be accurate if no header read errors
   // on preceding sectors
   int logical_sector; 
   // The sector state
   SECTOR_DECODE_STATUS status;
   SECTOR_DECODE_STATUS last_status;
} SECTOR_STATUS;

SECTOR_DECODE_STATUS mfm_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);
SECTOR_DECODE_STATUS wd_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);
SECTOR_DECODE_STATUS tagged_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);
SECTOR_DECODE_STATUS xebec_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);
SECTOR_DECODE_STATUS corvus_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);
SECTOR_DECODE_STATUS northstar_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
   int head, uint16_t deltas[], int *seek_difference, 
   SECTOR_STATUS bad_sector_list[]);

void mfm_check_header_values(int exp_cyl, int exp_head, int *sector_index, 
   int sector_size, int *seek_difference, SECTOR_STATUS *sector_status, 
   DRIVE_PARAMS *drive_params, SECTOR_STATUS sector_status_list[]);
void mfm_decode_setup(DRIVE_PARAMS *drive_params, int write);
void mfm_decode_done(DRIVE_PARAMS *drive_params);
int mfm_write_sector(uint8_t bytes[], DRIVE_PARAMS *drive_params,
   SECTOR_STATUS *sector_status, SECTOR_STATUS bad_sector_list[],
   uint8_t all_bytes[], int all_bytes_len);
int mfm_write_metadata(uint8_t bytes[], DRIVE_PARAMS *drive_params,
   SECTOR_STATUS *sector_status);
void mfm_init_sector_status_list(SECTOR_STATUS *sector_status_list, int num_sectors); 
void mfm_dump_bytes(uint8_t bytes[], int len, int cyl, int head,
      int sector_index, int msg_level);

// Define states for processing the data. MARK_ID is looking for the 0xa1 byte
// before a header and MARK_DATA is same for the data portion of the sector.
// MARK_DATA1 is looking for special Symbolics 3640 mark code.
// PROCESS_HEADER is processing the header bytes and PROCESS_DATA processing
// the data bytes. HEADER_SYNC and DATA_SYNC are looking for the one bit to sync to
// in CONTROLLER_XEBEC_104786. Not all decoders use all states.
typedef enum { MARK_ID, MARK_DATA, MARK_DATA1, HEADER_SYNC, DATA_SYNC, PROCESS_HEADER, PROCESS_HEADER2, PROCESS_DATA
} STATE_TYPE;

SECTOR_DECODE_STATUS mfm_process_bytes(DRIVE_PARAMS *drive_params, uint8_t bytes[],
      int bytes_crc_len, int total_bytes, STATE_TYPE *state, int cyl, 
      int head, int *sector_index,
      int *seek_difference, SECTOR_STATUS sector_status_list[],
      SECTOR_DECODE_STATUS init_status);

SECTOR_DECODE_STATUS wd_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);
SECTOR_DECODE_STATUS tagged_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);
SECTOR_DECODE_STATUS xebec_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);
SECTOR_DECODE_STATUS corvus_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);
SECTOR_DECODE_STATUS northstar_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);

int mfm_save_raw_word(DRIVE_PARAMS *drive_params, int all_raw_bits_count, 
   int int_bit_pos, int raw_word);
void mfm_mark_header_location(int bit_count, int bit_offset, int tot_bit_count);
void mfm_mark_data_location(int bit_count, int bit_offset, int tot_bit_count);
void mfm_mark_end_data(int bit_count, DRIVE_PARAMS *drive_params);
void mfm_handle_alt_track_ch(DRIVE_PARAMS *drive_params, unsigned int bad_cyl, 
      unsigned int bad_head, unsigned int good_cyl, unsigned int good_head);

#undef DEF_EXTERN
#endif /* MFM_DECODER_H_ */
