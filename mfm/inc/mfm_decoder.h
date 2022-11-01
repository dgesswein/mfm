#ifndef MFM_DECODER_H_
#define MFM_DECODER_H_
//
// 10/31/22 DJG Added ext2emu Corvus_H support 
// 10/01/22 DJG Added CTM9016 format
// 06/01/22 TJT Add CALLAN with proper CRC info
// 03/17/22 DJG Update function prototype
// 12/18/21 DJG Fix Symbolics 3640 ext2emu creation
// 12/18/21 SWE Added David Junior II
// 10/29/21 DJG Added STRIDE_440
// 09/20/21 DJG Added TANDY_16B to give ext2emu support
// 09/03/21 DJG Added SUPERBRAIN
// 08/27/21 DJG Added DSD_5217_512B
// 05/27/21 DJG Added TEKTRONIX_6130
// 03/21/21 DJG Added initial value for OMTI 20D controller 256 byte sectors
// 02/15/21 DJG Added ext2emu support for CONVERGENT_AWS
// 02/01/21 DJG Adjusted trk_ELEKTROKIKA_85 to match documentation on format
//    found.
// 01/18/21 DJG Add ext2emu support for Elektronika_85
// 01/07/21 DJG Added RQDX2 format
// 12/11/20 DJG Found false ECC correction so reduced ECC correction length
// 11/13/20 DJG Added CONTROLLER_ACORN_A310_PODULE
// 10/26/20 DJG ext2emu support for MYARC_HFDC controller
// 10/24/20 DJG Added MYARC_HFDC controller and ext2emu support for SM_1810_512B
// 10/17/20 DJG increased maximum ECC correction length to 2 for 24 bit
//    polynomial and 7 or 8 for 32 bit polynomial.
// 10/16/20 DJG Added SHUGART_SA1400 controller. 
// 10/08/20 DJG Added SHUGART_1610 and UNKNOWN2 controllers
// 09/21/21 DJG Added controller SM_1810_512B
// 12/31/19 DJG Added PERQ T2 ext2emu support. Not tested.
// 11/29/19 DJG Fix PERQ T2 format to ignore sector data trailing byte
// 10/25/19 DJG Added PERQ T2 format
// 10/05/19 DJG Fixes to detect when CONT_MODEL controller doesn't really
//    match format
// 07/19/19 DJG Added ext2emu support for Xerox 8010. Fixed data for 
//    trk_omti_5510
// 06/19/19 DJG Removed DTC_256B since only difference from DEC_520_256B was
//    error. Added SM1040 format. Fixed Xerox_8010 bitrate. Added recovery 
//    mode flag
// 02/09/19 DJG Added CONTROLLER_SAGA_FOX, adjusted trk_ISBC215_1024b to match
//    example file
// 01/20/18 DJG Increased maximum sector to support iSBC 214/215 128B 54 sectors/track
//    Added support for ext2emu for iSBC 214/215. Controller names changed.
// 12/16/18 DJG Added NIXDORF_8870
// 11/03/18 DJG Renamed variable
// 10/12/18 DJG Added CONTROLLER_IBM_3174
// 09/10/18 DJG Added CONTROLLER_DILOG_DQ604
// 08/05/18 DJG Added IBM_5288. Fixed Convergent AWS SA1000 format
// 07/02/18 DJG Added Convergent AWS SA1000 format and new data for finding
//   correct location to look for headers
// 06/03/18 DJG Added Tandy 8 Meg SA1004, fourth DTC variant, and ROHM_PBX
// 04/25/18 DJG Added Xerox 8010 and Altos support
// 03/31/18 DJG Added ext2emu support for DTC.
// 03/09/18 DJG Added CONTROLLER_DILOG_DQ614 and fields for reading more
//    cylinders and heads than analyze determines.
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
// ISBC_214 can have 54 128 byte sectors
#define MAX_SECTORS 70
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
   // Don't do retry when >= than specified head or cylinder. This is used
   // when drive mixes formats. Code currently can only decode one so
   // retries don't help
   int noretry_cyl;
   int noretry_head;
   // The number of the first sector. Some disks start at 0 others 1
   int first_sector_number;
   // Size of data area of sector in bytes
   int sector_size;
   // Size of metadata area of sector in bytes
   int metadata_bytes;
   // CRC/ECC used for header and data area
   CRC_INFO header_crc, data_crc;
   // Track format
   // Update mfm_controller_info list below if enum changed.
   // ORDER IN THE TWO LISTS MUST MATCH
   // TODO, replace this with pointer to CONTROLLER entry
   enum {CONTROLLER_NONE,
      CONTROLLER_NEWBURYDATA,
      CONTROLLER_ALTOS,
      CONTROLLER_SUPERBRAIN,
      CONTROLLER_WD_1006, 
      CONTROLLER_RQDX2, 
      CONTROLLER_ISBC_214_128B,
      CONTROLLER_ISBC_214_256B,
      CONTROLLER_ISBC_214_512B,
      CONTROLLER_ISBC_214_1024B,
      CONTROLLER_TEKTRONIX_6130,
      CONTROLLER_NIXDORF_8870, 
      CONTROLLER_TANDY_8MEG, 
      CONTROLLER_WD_3B1,
      CONTROLLER_TANDY_16B,
      CONTROLLER_MOTOROLA_VME10, 
      CONTROLLER_DTC, 
      CONTROLLER_DTC_520_256B, 
      CONTROLLER_DTC_520_512B, 
      CONTROLLER_MACBOTTOM, 
      CONTROLLER_CTM9016, 
      CONTROLLER_ACORN_A310_PODULE, 
      CONTROLLER_ELEKTRONIKA_85,
      CONTROLLER_ALTOS_586,
      CONTROLLER_ATT_3B2,
      CONTROLLER_CONVERGENT_AWS,
      CONTROLLER_CONVERGENT_AWS_SA1000,
      CONTROLLER_WANG_2275,
      CONTROLLER_WANG_2275_B,
      CONTROLLER_CALLAN,
      CONTROLLER_IBM_5288,
      CONTROLLER_EDAX_PV9900,
      CONTROLLER_SHUGART_1610,
      CONTROLLER_SHUGART_SA1400,
      CONTROLLER_SM_1810_512B, 
      CONTROLLER_DSD_5217_512B, 
      CONTROLLER_OMTI_5510, 
      CONTROLLER_XEROX_6085, 
      CONTROLLER_TELENEX_AUTOSCOPE, 
      CONTROLLER_MORROW_MD11,
      CONTROLLER_UNKNOWN1,
      CONTROLLER_UNKNOWN2,
      CONTROLLER_DEC_RQDX3, 
      CONTROLLER_MYARC_HFDC, 
      CONTROLLER_IBM_3174,
      CONTROLLER_SEAGATE_ST11M,
      CONTROLLER_ISBC_215_128B,
      CONTROLLER_ISBC_215_256B,
      CONTROLLER_ISBC_215_512B,
      CONTROLLER_ISBC_215_1024B,
      CONTROLLER_XEROX_8010,
      CONTROLLER_ROHM_PBX,
      CONTROLLER_ADAPTEC, 
      CONTROLLER_MVME320,
      CONTROLLER_SYMBOLICS_3620, 
      CONTROLLER_DJ_II,
      CONTROLLER_SM1040,
      CONTROLLER_SYMBOLICS_3640, 
      CONTROLLER_MIGHTYFRAME, 
      CONTROLLER_DG_MV2000, 
      CONTROLLER_SOLOSYSTEMS, 
      CONTROLLER_DILOG_DQ614,
      CONTROLLER_DILOG_DQ604,
      CONTROLLER_XEBEC_104786, 
      CONTROLLER_XEBEC_104527_256B, 
      CONTROLLER_XEBEC_104527_512B, 
      CONTROLLER_XEBEC_S1420, 
      CONTROLLER_EC1841, 
      CONTROLLER_CORVUS_H, 
      CONTROLLER_NORTHSTAR_ADVANTAGE,
      CONTROLLER_CROMEMCO,
      CONTROLLER_VECTOR4,
      CONTROLLER_VECTOR4_ST506,
      CONTROLLER_STRIDE_440,
      CONTROLLER_SAGA_FOX,
      CONTROLLER_PERQ_T2
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
   // Use drive recovery line.
   int recovery;
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
   int ext_metadata_fd;
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
   // Non zero if start_time_ns has been set either from input file or command
   // line and shouldn't be overridden.
   int dont_change_start_time;
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
  // Length 0 for parity (Symbolics 3640). Doesn't really use length
  // also used for CHECK_NONE
  {0, 0, 0},
  // Length 16 for Northstar header checksum
  {0, 16, 0},
  // Length 32 for Northstar data checksum
  {0, 32, 0},
  // Length 8 for Wang header checksum
  {0, 8, 0},
  // This seemed to have more false corrects than other 32 bit polynomials with
  // more errors than can be corrected. Had false correction at length 5 on disk
  // read so dropped back to 4. My attempt to test showed 7 gives 3-42 false 
  // corrections per 100000. Some controllers do 11 bit correct with 32 bit
  // polynomial
  {0x00a00805, 32, 4}, 
  // Don't move this without fixing the Northstar reference
  {0x1021, 16, 0},
  {0x8005, 16, 0},
  // CTM9016
  {0x0001, 16, 0},
  // The rest of the 32 bit polynomials with 8 bit correct get 5-19 false 
  // corrects per 100000 when more errors than can be corrected. Reduced due
  // to false correct seen with 0x00a00805
  {0x140a0445, 32, 6},
  {0x140a0445000101ll, 56, 22}, // From WD42C22C datasheet, not tested
  {0x0104c981, 32, 6},
  // The Shugart SA1400 that uses this polynomial says it does 4 bit correct.
  // That seems to have excessive false corrects when more errors that can be
  // corrected so went with 2 bit correct which has 43-141 miscorrects 
  // per 100000 for data with more errors than can be corrected.
  {0x24409, 24, 2},
  {0x3e4012, 24, 0}, // WANG 2275. Not a valid ECC code so max correct 0
  {0x88211, 24, 2}, // ROHM_PBX
  // Adaptec bad block on Maxtor XT-2190
  {0x41044185, 32, 6},
  // MVME320 controller
  {0x10210191, 32, 6},
  // Shugart 1610
  {0x10183031, 32, 6},
  // DSD 5217
  {0x00105187, 32, 6},
  // David Junior II DJ_II
  {0x5140c101, 32, 6},
  // Nixdorf 
  {0x8222f0804bda23ll, 56, 22}
  // DQ604 Not added to search since more likely to cause false
  // positives that find real matches
  //{0x1, 8, 0}
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
   {{-1, 0}, {-1, 0xffffffffffffffffll}, {32, 0x2605fb9c}, {32, 0xd4d7ca20},
     {32, 0x409e10aa},
     // 256 byte OMTI
     {32, 0xe2277da8},
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
     {24, 0x223808},
     // This is for DILOG_DQ614, header and data
     {32, 0x58e07342},
     {32, 0xcf2105e0},
     // This is for Convergent AWS on Quantum Q2040 header and data
     {32, 0x920d65c0},
     {32, 0xef26129d},
     {16, 0x8026}, // IBM 3174
     {16, 0x551a} // Altos
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
      // FIELD_C0 writes the C0 header/data mark code. Only 1 byte valid.
      // FIELD_CYL, HEAD, SECTOR, LBA write the current cylinder, head, sector,
      //   or logical block address. Valid for byte of bit fields.
      // FIELD_HDR_CRC and FIELD_DATA_CRC write the check word. The
      //    CONTROLLER data defines the type of check word.
      // FIELD_MARK_CRC_START and END are used to mark the start and end
      // byte for CRC (check data) calculation. The default
      // includes the all the data from sector start flag byte (a1 etc) to 
      // the CRC.
   enum {FIELD_FILL, FIELD_A1, FIELD_C0, FIELD_CYL, FIELD_HEAD, FIELD_SECTOR,
      FIELD_LBA, FIELD_HDR_CRC, FIELD_DATA_CRC, FIELD_SECTOR_DATA, 
      FIELD_MARK_CRC_START, FIELD_MARK_CRC_END,
      FIELD_BAD_SECTOR,
         // These are for controllers that need special handling
      FIELD_HEAD_SEAGATE_ST11M, FIELD_CYL_SEAGATE_ST11M,
         // Mark end of sector to increment sector counter
      FIELD_NEXT_SECTOR, FIELD_SECTOR_METADATA,
      } type;
      // Value for field. 
   uint8_t value;
      // OP_SET writes the data to the field, OP_XOR exclusive or's it
      // with the current contents, OP_REVERSE reverses the bits then does
      // an OP_SET.
   enum {OP_SET, OP_XOR, OP_REVERSE, OP_REVERSE_XOR} op;
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
// TANDY 16B. From example XENIX.emu in emu.zip. Minor change to 3b1 format. Probably minor padding changes don't matter.
DEF_EXTERN TRK_L trk_tandy_16b[] 
#ifdef DEF_DATA
 = 
{ { 34, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
        {33, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {388, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// Format from http://www.bitsavers.org/pdf/intel/iSBC/134910-001_iSBC_214_Peripheral_Controller_Subsystem_Hardware_Reference_Manual_Aug_85.pdf

// Format from http://www.bitsavers.org/pdf/intel/iSBC/134910-001_iSBC_214_Peripheral_Controller_Subsystem_Hardware_Reference_Manual_Aug_85.pdf
// Four sectors sizes for Intel iSBC214 controller
DEF_EXTERN TRK_L trk_ISBC214_128b[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 54, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
              // Sector size 128
              {1, FIELD_FILL, 0x60, OP_SET, 3, NULL},
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {134, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {128, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 130, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {15, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {251, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_ISBC214_256b[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
              // Sector size 256
              {1, FIELD_FILL, 0x00, OP_SET, 3, NULL},
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {262, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {15, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {291, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;
DEF_EXTERN TRK_L trk_ISBC214_512b[] 
#ifdef DEF_DATA
 = 
{ { 38, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
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
        {3, TRK_FILL, 0x00, NULL},
        {38, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {265, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_ISBC214_1024b[] 
#ifdef DEF_DATA
 = 
{ { 54, TRK_FILL, 0x4e, NULL },
  { 9, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
              // Sector size 1024
              {1, FIELD_FILL, 0x40, OP_SET, 3, NULL},
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {1030, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {1024, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 1026, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {54, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {257, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// Format from http://www.bitsavers.org/pdf/intel/iSBC/144780-002_iSBC_215_Generic_Winchester_Disk_Controller_Hardware_Reference_Manual_Dec84.pdf. Gaps not
// specified in manual so iSBC_214 values used
// Four sectors sizes for Intel iSBC215 controller
// **** NOTE, these don't currently generate image readable by the controller.
// **** More investigation needed if sufficient interest
DEF_EXTERN TRK_L trk_ISBC215_128b[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 54, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x19, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Sector size 128. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x00, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {134, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xd9, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {128, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 130, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {13, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {251, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_ISBC215_256b[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 31, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x19, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Sector size 256. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x10, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {262, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xd9, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {17, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {452, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;
DEF_EXTERN TRK_L trk_ISBC215_512b[] 
#ifdef DEF_DATA
 = 
{ { 38, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x19, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Sector size 512. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x20, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xd9, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {36, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {265, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_ISBC215_1024b[] 
#ifdef DEF_DATA
 = 
// This is from inspecting data from drive. First data after index
// does not seem to be deterministic
{ { 15, TRK_FILL, 0x00, NULL },
  { 9, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {15, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x19, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Sector size 1024. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x30, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {5, TRK_FILL, 0x4e, NULL},
        {12, TRK_FILL, 0x00, NULL},
        {1030, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xd9, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {1024, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 1026, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {65, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {143, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_tektronix_6130[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 54, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
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
              // Sector size 128
              {1, FIELD_FILL, 0x60, OP_SET, 3, NULL},
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {15, TRK_FILL, 0x00, NULL},
        {134, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {128, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 130, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {15, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {251, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;


DEF_EXTERN TRK_L trk_convergent_aws[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x00, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 2, NULL},
              // Put head in bits 7-4 of byte 2
              {1, FIELD_HEAD, 0x00, OP_XOR, 4,
                 (BIT_L []) {
                    { 16, 4},
                    { -1, -1},
                 }
              },
              // Cyl in upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {19, TRK_FILL, 0x00, NULL},
        {260, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {19, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {195, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_ELEKTROKIKA_85[] 
#ifdef DEF_DATA
 = 
{ { 2, TRK_FILL, 0x00, NULL },
  { 16, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
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
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_SET, 3, NULL},
              // Don't support alternate tracks
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {532, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x80, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {16, FIELD_FILL, 0x00, OP_SET, 514, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 530, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {40, TRK_FILL, 0x55, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {672, TRK_FILL, 0x55, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// From hand written notes 20201020_225628.jpg slightly adjusted to
// match disk image
DEF_EXTERN TRK_L trk_sm_1810[] 
#ifdef DEF_DATA
 = 
{ { 15, TRK_FILL, 0x4e, NULL },
  { 16, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {12, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Sector size 512. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x20, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {16, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {4, TRK_FILL, 0x00, NULL},
        {24, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {1059, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// From http://www.bitsavers.org/pdf/dsd/5215_5217/040040-01_5215_UG_Apr84.pdf
DEF_EXTERN TRK_L trk_DSD_5217_512B[] 
#ifdef DEF_DATA
 = 
{ { 7, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {12, TRK_FILL, 0x00, NULL},
        {10, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              // Sector size 512. All tracks data, alternate not supported
              {1, FIELD_FILL, 0x20, OP_SET, 2, NULL},
              // Upper 4 bits in low 4 bits of byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 12, 
                 (BIT_L []) {
                    { 20, 4},
                    { 24, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 5, NULL},
              {4, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        // 3 after header and 12 before data field
        {3, TRK_FILL, 0x4e, NULL},
        {6, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfb, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {44, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {330, TRK_FILL, 0x4e, NULL},
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
   {717, TRK_FILL, 0x4e, NULL},
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
        {1165, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_FILL, 0x0f, OP_SET, 0, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 1, NULL},
              {1160, FIELD_SECTOR_DATA, 0x00, OP_REVERSE, 1, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 1161, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {49, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {42, TRK_FILL, 0x4e, NULL},
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

// 512B 17 sectors per track from unknown DTC controller
DEF_EXTERN TRK_L trk_dtc_pc_512b[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // This adds upper 3 bits of cylinder to bits 4-6 of
              // byte 3 and the rest in byte 2. 
              {0, FIELD_CYL, 0x00, OP_SET, 11, 
                 (BIT_L []) {
                    { 25, 3}, // Byte 3 bits 6-4 gets upper 3 bits
                    { 16, 8}, // byte 2 gets lower 8 bits
                    { -1, -1},
                 }
              },
              // Add head to lower bits, Need XOR since SET clears all bits
              // in byte.
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {3, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {17, TRK_FILL, 0x00, NULL},
        {517, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {3, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {37, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {304, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// 512B 18 sectors per track from DTC 520 controller manual
DEF_EXTERN TRK_L trk_dtc_520_512b[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 18, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // This adds upper 3 bits of cylinder to bits 4-6 of
              // byte 3 and the rest in byte 2. 
              {0, FIELD_CYL, 0x00, OP_SET, 11, 
                 (BIT_L []) {
                    { 25, 3}, // Byte 3 bits 6-4 gets upper 3 bits
                    { 16, 8}, // byte 2 gets lower 8 bits
                    { -1, -1},
                 }
              },
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {3, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {517, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {3, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {14, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {158, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_dtc_520_256b[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 33, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // This adds upper 3 bits of cylinder to bits 4-6 of
              // byte 3 and the rest in byte 2. 
              {0, FIELD_CYL, 0x00, OP_SET, 11, 
                 (BIT_L []) {
                    { 25, 3}, // Byte 3 bits 6-4 gets upper 3 bits
                    { 16, 8}, // byte 2 gets lower 8 bits
                    { -1, -1},
                 }
              },
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {3, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {261, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {3, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {10, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {205, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// **** NOTE, these don't currently generate image readable by the controller.
// **** More investigation needed if sufficient interest
// No information on format is available
DEF_EXTERN TRK_L trk_saga_fox[] 
#ifdef DEF_DATA
 = 
{ { 87, TRK_FILL, 0x00, NULL },
  { 34, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {17, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x0f, OP_SET, 0, NULL},
              // This adds upper 3 bits of cylinder to bits 4-6 of
              // byte 3 and the rest in byte 2. 
              {0, FIELD_CYL, 0x00, OP_REVERSE, 16,
                 (BIT_L []) {
                    { 8, 8}, // High byte get written to low after reverse
                    { 16, 8},
                    { -1, -1},
                 }
              },
              // Add head to lower bits
              {1, FIELD_HEAD, 0x00, OP_REVERSE, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_REVERSE, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {19, TRK_FILL, 0x00, NULL},
        {259, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x0f, OP_SET, 0, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_REVERSE, 1, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 257, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {-1, 0, 0, NULL},
     }
   },
   {63, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// Some info in
// http://bitsavers.org/pdf/xerox/8010_dandelion/DandelionHardwareRefRev2.2.pdf
// Starting on page 75 and 88
// The number of fill bytes should be reasonably close. The actual fill byte
// values may not match the real format since the values seen were not
// consistant.
DEF_EXTERN TRK_L trk_xerox_8010[] 
#ifdef DEF_DATA
 = 
{ { 60, TRK_FILL, 0x4e, NULL },
  { 16, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {14, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x41, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {2, FIELD_CYL, 0x00, OP_SET, 2, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 4, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 5, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {21, TRK_FILL, 0x00, NULL},
        {28, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x43, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {24, FIELD_SECTOR_METADATA, 0x00, OP_SET, 2, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 26, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {23, TRK_FILL, 0x00, NULL},
        {516, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0x43, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {790, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_perq_t2[] 
#ifdef DEF_DATA
 = 
{ { 158, TRK_FILL, 0x00, NULL },
  { 16, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {22, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              // sector mark
              {1, FIELD_C0, 0xC0, OP_SET, 0, NULL},
              {13, FIELD_FILL, 0x00, OP_SET, 1, NULL},
              // Sync
              {1, FIELD_FILL, 0x0f, OP_SET, 14, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 15, NULL},
              // Upper 4 bits in upper 4 bits of byte 1, lower 8 bits in
              // byte 0
              {0, FIELD_CYL, 0x00, OP_REVERSE_XOR, 12, 
                 // This is reversed to match the bit reversing
                 (BIT_L []) {
                    {120, 8},
                    {132, 4},
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_REVERSE_XOR, 16, NULL},
              {1, FIELD_SECTOR, 0x00, OP_REVERSE, 17, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 18, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 19, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 21, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {35, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {14, FIELD_FILL, 0x00, OP_SET, 0, NULL},
              // Sync
              {1, FIELD_FILL, 0x0f, OP_SET, 14, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 15, NULL},
              {16, FIELD_SECTOR_METADATA, 0x00, OP_REVERSE, 15, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 31, NULL},
              {2, FIELD_FILL, 0x00, OP_SET, 33, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {575, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {14, FIELD_FILL, 0x00, OP_SET, 0, NULL},
              // Sync
              {1, FIELD_FILL, 0x0f, OP_SET, 14, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 15, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_REVERSE, 15, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 527, NULL},
              {46, FIELD_FILL, 0x00, OP_SET, 529, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {-1, 0, 0, NULL},
     }
   },
   {148, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_shugart_1610[] 
#ifdef DEF_DATA
 = 
{ { 26, TRK_FILL, 0x4e, NULL },
  { 17, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {12, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 0, NULL},
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              // Upper 3 bits in bits 6-4 of byte 3, lower 8 bits in
              // byte 2. All tracks marked good track
              {0, FIELD_CYL, 0x00, OP_XOR, 11, 
                 // List is starting from the high bit. Low bit to write to, length
                 (BIT_L []) {
                    { 28, 3},
                    { 16, 8},
                    { -1, -1},
                 }
              },
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 0, NULL},
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 514, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x00, NULL},
        {45, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {209, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_shugart_1400[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xfe, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              // Upper 3 bits in bits 6-4 of byte 3, lower 8 bits in
              // byte 2. All tracks marked good track
              {1, FIELD_CYL, 0x00, OP_SET, 2, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {3, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {15, TRK_FILL, 0x00, NULL},
        {261, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 2, NULL},
              {256, FIELD_FILL, 0xff, OP_SET, 2, NULL},
              // Data is inverted. Fill with 1's then XOR in data
              {256, FIELD_SECTOR_DATA, 0x00, OP_XOR, 2, NULL},
              {3, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {17, TRK_FILL, 0x00, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {354, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// Mixture of datasheet and formatter gap lengths adjusted with actual
// file. Not sure if really correct. The pad at the end seems a little
// low.
DEF_EXTERN TRK_L trk_myarc_hfdc[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {13, TRK_FILL, 0x00, NULL},
        {8, TRK_FIELD, 0x00, 
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
              // Put head in lower bits
              {1, FIELD_HEAD, 0x00, OP_XOR, 3, NULL},
              // Put upper 3 bits of cyl in bits 6-4
              {0, FIELD_CYL, 0x00, OP_XOR, 11, 
                 (BIT_L []) {
                    { 25, 3},
                    { 32, 8}, // Dump the rest of the bits where it will be overwritten
                    { -1, -1},
                 }
              },
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              // 4 byte ECC, Sector size 256
              {1, FIELD_FILL, 0x01, OP_SET, 5, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 6, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x4e, NULL},
        {13, TRK_FILL, 0x00, NULL},
        {262, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {3, TRK_FILL, 0x00, NULL},
        {15, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {258, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_acorn_a310_podule[] 
#ifdef DEF_DATA
 = 
{ { 16, TRK_FILL, 0x4e, NULL },
  { 32, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {16, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 1, NULL},
              {2, FIELD_CYL, 0x00, OP_SET, 1, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {2, TRK_FILL, 0x4e, NULL},
        {16, TRK_FILL, 0x00, NULL},
        {262, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 1, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {256, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 258, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {17, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {162, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_CTM9016[] 
#ifdef DEF_DATA
 = 
{ { 257, TRK_FILL, 0x4e, NULL },
  { 8, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        {24, TRK_FILL, 0x00, NULL},
        {7, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {2, FIELD_CYL, 0x00, OP_SET, 1, NULL},
              {1, FIELD_HEAD, 0x00, OP_SET, 3, NULL},
              {1, FIELD_SECTOR, 0x00, OP_SET, 4, NULL},
              {2, FIELD_HDR_CRC, 0x00, OP_SET, 5, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {27, TRK_FILL, 0x00, NULL},
        {1030, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_A1, 0xa1, OP_SET, 0, NULL},
              {1, FIELD_FILL, 0xf8, OP_SET, 1, NULL},
              {1024, FIELD_SECTOR_DATA, 0x00, OP_SET, 2, NULL},
              {4, FIELD_DATA_CRC, 0x00, OP_SET, 1026, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {4, TRK_FILL, 0x00, NULL},
        {98, TRK_FILL, 0x4e, NULL},
        {-1, 0, 0, NULL},
     }
   },
   {641, TRK_FILL, 0x4e, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

DEF_EXTERN TRK_L trk_corvus_h[] 
#ifdef DEF_DATA
 = 
{ { 7, TRK_FILL, 0x00, NULL },
  { 20, TRK_SUB, 0x00, 
     (TRK_L []) 
     {
        // Real disk seems to have some non zero bytes here but they aren't
        // always the same or a simple pattern so I ignored them
        {49, TRK_FILL, 0x00, NULL},
        {518, TRK_FIELD, 0x00, 
           (FIELD_L []) {
              {1, FIELD_FILL, 0x02, OP_SET, 0, NULL},
              {0, FIELD_MARK_CRC_START, 0, OP_SET, 1, NULL},
              {1, FIELD_FILL, 0x00, OP_SET, 1, NULL},
              // Put head in bits 7-5 of byte 1
              {1, FIELD_HEAD, 0x00, OP_XOR, 3,
                 (BIT_L []) {
                    { 8, 3},
                    { -1, -1},
                 }
              },
              // Put sector in bits 4-0 of byte 1
              {0, FIELD_SECTOR, 0x00, OP_XOR, 5,
                 (BIT_L []) {
                    { 11, 5},
                    { -1, -1},
                 }
              },
              {2, FIELD_FILL, 0x00, OP_SET, 2, NULL},
              // Cyl in upper 8 bits in byte 2, lower 8 bits in
              // byte 3
              {0, FIELD_CYL, 0x00, OP_XOR, 16, 
                 (BIT_L []) {
                    { 24, 8},
                    { 16, 8},
                    { -1, -1},
                 }
              },
              {512, FIELD_SECTOR_DATA, 0x00, OP_SET, 4, NULL},
              {2, FIELD_DATA_CRC, 0x00, OP_SET, 516, NULL},
              {0, FIELD_NEXT_SECTOR, 0x00, OP_SET, 0, NULL},
              {-1, 0, 0, 0, 0, NULL}
           }
        },
        {-1, 0, 0, NULL},
     }
   },
   {113, TRK_FILL, 0x00, NULL},
   {-1, 0, 0, NULL},
}
#endif
;

// CHECK_NONE is used for header formats where some check that is specific
// to the format is used so can't be generalized. If so the check will need
// to be done in the header decode and ext2emu as special cases.
typedef enum {CHECK_CRC, CHECK_CHKSUM, CHECK_PARITY, CHECK_XOR16,
              CHECK_NONE} CHECK_TYPE;

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
      // These bytes at start of header and data header ignored in CRC calc
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
      // And first sector number
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
   enum {CONT_ANALYZE, CONT_MODEL} analyze_search;

      // Minimum number of bits from last good header. Zero if not used
      // Both must be zero or non zero
   int header_min_delta_bits, data_min_delta_bits;
     // Minimum number of bits from index for first header
   int first_header_min_bits;
} CONTROLLER;

DEF_EXTERN CONTROLLER mfm_controller_info[]
// Keep sorted by header length for same controller family. 
// MUST MATCH order of controller enum
#ifdef DEF_DATA
   = {
      {"CONTROLLER_NONE",        0, 10000000,      0, 
         0,0, 0,0,
         0,0, CINFO_NONE,
         0, 0, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, 0,
         0, 0, 0
      },
      {"NewburyData",          256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Altos",              256, 8680000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0x551a,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Superbrain",  256, 10000000, 230000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly),
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 1, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
// Should be model after data filled in
         0, 33,
         {0,0x1021,16,0},{0,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"WD_1006",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"RQDX2",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 18, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_214_128B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC214_128b, 128, 54, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_214_256B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC214_256b, 256, 32, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_214_512B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC214_512b, 512, 17, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_214_1024B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC214_1024b, 1024, 9, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      // TODO: Analyize currently can't separate this from Intel_iSBC_214_512B
      // since only different for heads >= 8
      {"Tektronix_6130",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_tektronix_6130, 512, 17, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"NIXDORF_8870",         256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 2, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 16, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0,0x8222f0804bda23,56,22}, CONT_MODEL,
         0, 0, 0
      },
      {"TANDY_8MEG",              512, 8680000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"WD_3B1",          512, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_3B1, 512, 17, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"TANDY_16B",          512, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_tandy_16b, 512, 17, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Motorola_VME10",  256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0,0xa00805,32,4},{0,0xa00805,32,4}, CONT_ANALYZE,
         0, 0, 0
      },
      {"DTC",             256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_dtc_pc_512b, 512, 17, 0, 5209,
         0, 0,
         {0,0x24409,24,2},{0,0x24409,24,2}, CONT_MODEL,
         0, 0, 0
      },
      {"DTC_520_256B",             256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_dtc_520_256b, 256, 33, 0, 5209,
         0, 0,
         {0,0x24409,24,2},{0,0x24409,24,2}, CONT_MODEL,
         0, 0, 0
      },
      {"DTC_520_512B",             512, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_dtc_520_512b, 512, 18, 0, 5209,
         0, 0,
         {0,0x24409,24,2},{0,0x24409,24,2}, CONT_MODEL,
         0, 0, 0
      },
      {"MacBottom",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"CTM9016",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_CTM9016, 1024, 8, 0, 5209,
         0, 0,
         {0x0,0x1021,16,0},{0x0,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Acorn_A310_podule",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, trk_acorn_a310_podule, 256, 32, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0x0,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      // Also DEC professional 350 & 380
      {"Elektronika_85",      256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         16, 1, trk_ELEKTROKIKA_85, 512, 16, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Altos_586",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"ATT_3B2",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"CONVERGENT_AWS",       256, 10000000, 0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_convergent_aws, 256, 32, 1, 5209,
         0, 0,
         {0x0,0x1021,16,0},{0x0,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"CONVERGENT_AWS_SA1000",       512, 8680000, 0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 16, 0, 5209,
         0, 0,
         {0x920d65c0,0x140a0445,32,6},{0xef26129d,0x140a0445,32,6}, CONT_MODEL,
         8700, 770, 350
      },
      {"WANG_2275",              256, 10000000,      0, 
         3, 4, 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 0, CHECK_CHKSUM, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"WANG_2275_B",            256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_MODEL,
         0, 0, 0
      },
      {"CALLAN",            256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
	 {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"IBM_5288",            256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 4, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"EDAX_PV9900",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 1, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"SHUGART_1610",          512, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_shugart_1610, 512, 17, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x10183031,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"SHUGART_SA1400",            256, 8680000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_shugart_1400, 256, 32, 0, 5209,
         0, 0,
         {0x0,0x24409,24,2},{0x0,0x24409,24,2}, CONT_MODEL,
         0, 0, 0
      },
      // OMTI_5200 uses initial value 0x409e10aa for data
      // Data CRC is really initial value 0 xor of final value of 0xffffffff.
      // Code doesn't do final xor so initial value is equivalent.
      {"SM_1810_512B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_sm_1810, 512, 16, 0, 5209,
         0, 0,
         {0xed800493,0xa00805,32,4},{0x03affc1d,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"DSD_5217_512B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_DSD_5217_512B, 512, 17, 0, 5209,
         0, 0,
         {0xffffffff,0x105187,32,6},{0xffffffff,0x105187,32,6}, CONT_MODEL,
         0, 0, 0
      },
      // For 20D controller 256 byte sectors polynomial is 0xe2277da8,0x104c981,32,6
      {"OMTI_5510",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_omti_5510, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,6},{0xd4d7ca20,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Xerox_6085",           256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         20, 0,
         { 0x2605fb9c,0x104c981,32,6},{0xd4d7ca20,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Telenex_Autoscope",           256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,6},{0xd4d7ca20,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Morrow_MD11",            1024, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 1024, 9, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,6},{0xd4d7ca20,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Unknown1",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 1, 1, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         { 0x2605fb9c,0x104c981,32,6},{0xd4d7ca20,0x104c981,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"Unknown2",            256, 8680000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffff,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"DEC_RQDX3",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      // ext2emu not supported since it used deleted data 0xf8 on some
      // sectors which we currently don't have a good way to handle
      {"MYARC_HFDC",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_myarc_hfdc, 256, 32, 0, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"IBM_3174",            256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 516, 17, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0x8026,0x1021,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Seagate_ST11M",        256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_seagate_ST11M, 512, 17, 0, 5209,
         0, 0,
         {0x0,0x41044185,32,6},{0x0,0x41044185,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Intel_iSBC_215_128B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC215_128b, 128, 54, 0, 5209,
         0, 0,
         {0xed800493,0xa00805,32,4},{0xbe87fbf4,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_215_256B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC215_256b, 256, 31, 0, 5209,
         0, 0,
         {0xed800493,0xa00805,32,4},{0xbe87fbf4,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_215_512B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC215_512b, 512, 17, 0, 5209,
         0, 0,
         {0xed800493,0xa00805,32,4},{0xbe87fbf4,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Intel_iSBC_215_1024B",      128, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_ISBC215_1024b, 1024, 9, 0, 5209,
         0, 0,
         {0xed800493,0xa00805,32,4},{0xbe87fbf4,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
// Quantum Q20#0 Xerox Star drive
// SA100# 8" drives with similar format won't work properly with this
// format and ext2emu due to different rotation speed.
      {"Xerox_8010",      128, 8500000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 2, 2, CHECK_CRC, CHECK_CRC,
         0, 1, trk_xerox_8010, 512, 16, 0, 5425,
         24, 0,
         {0xffff,0x8005,16,0},{0xffff,0x8005,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"ROHM_PBX",      256, 8680000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 1, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0}, {0, 0x88211, 24, 2}, CONT_MODEL,
         0, 0, 0
      },
//TODO, this won't analyze properly
      {"Adaptec",              256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_LBA,
         6, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"MVME320",        256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, trk_mvme320, 256, 32, 1, 5209,
         0, 0,
         {0xffff,0x1021,16,0},{0xffffffff,0x10210191,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Symbolics_3620",       256, 10000000,      0, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 3, 3, 3, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
// Should be model after data filled in
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      // Header is either 6 or 10 bytes
      {"DJ_II",            256, 10000000,      0,
         0, 0, 4, ARRAYSIZE(mfm_all_poly),
         0, 0, CINFO_CHS,
         6, 2, 0, 2, CHECK_NONE, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0x5140c101,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"SM1040",       256, 10000000,      0, 
         0, 1, 4, ARRAYSIZE(mfm_all_poly), 
         0, 1, CINFO_CHS,
         10, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         {0x6e958e56,0x140a0445,32,6},{0xcf2105e0,0x140a0445,32,6}, CONT_MODEL,
         0, 0, 0
      },
      {"Symbolics_3640",       256, 10000000,      0, 
         0, 1, 4, ARRAYSIZE(mfm_all_poly), 
         0, 1, CINFO_CHS,
         11, 2, 7, 2, CHECK_PARITY, CHECK_CRC,
         0, 1, trk_symbolics_3640, 1160, 8, 0, 5209,
         0, 0,
         {0x0,0x0,0,0},{0x0,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
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
         {0,0,0,0},{0,0,0,0}, CONT_MODEL,
         0, 0, 0
      },
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
         {0,0,0,0},{0,0,0,0}, CONT_MODEL,
         0, 0, 0
      },
// END of WD type controllers
      {"SOLOsystems",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"DILOG_DQ614",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         8, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"DILOG_DQ604",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         8, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         {0,0x1,8,0},{0,0x1,8,0}, CONT_MODEL,
         0, 0, 0
      },
//    Changed begin time from 0 to 100500 to work with 1410A. The sample
//    I have of the 104786 says it should work with it also so changing default.
//    Its possible this will cause problems with other variants.
      {"Xebec_104786",         256, 10000000,      100500,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Xebec_104527_256B",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 256, 32, 0, 5209,
         0, 0,
         {0x0,0xa00805,32,4},{0x0,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Xebec_104527_512B",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 512, 17, 0, 5209,
         0, 0,
         {0x0,0xa00805,32,4},{0x0,0xa00805,32,4}, CONT_MODEL,
         0, 0, 0
      },
      {"Xebec_S1420",         256, 10000000,      0,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 0,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"EC1841",         256, 10000000,      220000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 1, NULL, 0, 0, 0, 5209,
         0, 10,
         {0,0,0,0},{0,0,0,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Corvus_H",             512, 11000000,  312000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         3, 3, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, trk_corvus_h, 512, 20, 0, 5730,
         0, 0,
         // Only have one CRC. DATA_CRC needs to be non zero for analyze_model
         // to ignore this format. Also needed by ext2emu for mark_bad to work
         {0xffff,0x8005,16,0},{0xffff,0x8005,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"NorthStar_Advantage",  256, 10000000, 230000,
         1, 2, 2, 3,
         0, 1, CINFO_CHS,
         7, 0, 0, 0, CHECK_CHKSUM, CHECK_CHKSUM,
         0, 1, trk_northstar, 512, 16, 0, 5209,
// Should be model after data filled in
         0, 33,
         {0,0,16,0},{0,0,32,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Cromemco",             10240, 10000000,  6000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 9, 0, 0, CHECK_CRC, CHECK_CRC,
         7, 0, trk_cromemco_stdc, 10240, 1, 0, 5209,
// Should be model after data filled in
         0, 0,
         {0,0x8005,16,0},{0,0x8005,16,0}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Vector4",             256, 10000000,  300000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 4, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 256, 32, 0, 5209,
// Should be model after data filled in
         0, 20,
         {0x0,0x104c981,32,6},{0x0,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Vector4_ST506",             256, 10000000,  300000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 4, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 256, 32, 0, 5209,
// Should be model after data filled in
         0, 20,
         {0x0,0x104c981,32,6},{0x0,0x104c981,32,6}, CONT_ANALYZE,
         0, 0, 0
      },
      {"Stride_440",             256, 10000000,  300000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         28, 0, 6, 0, CHECK_CRC, CHECK_CRC,
         1, 0, NULL, 8192, 1, 0, 5209,
         0, 20,
         {0x0,0x8005,16,0},{0x0,0x8005,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"Saga_Fox",             256, 10000000,  330000,
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 1, 0, 0, CHECK_XOR16, CHECK_XOR16,
         0, 1, trk_saga_fox, 256, 33, 0, 5209,
// Should be model after data filled in
         0, 20,
         {0x0,0,16,0},{0x0,0x0,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {"PERQ_T2",              256, 10000000,    754000, 
         4, ARRAYSIZE(mfm_all_poly), 4, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 0, 0, 0, CHECK_CRC, CHECK_CRC,
         1, 1, trk_perq_t2, 512, 16, 0, 5209,
         16, 0,
         {0x0,0x8005,16,0},{0x0,0x8005,16,0}, CONT_MODEL,
         0, 0, 0
      },
      {NULL, 0, 0, 0,
         0, 0, 0, 0,
         0,0, CINFO_NONE,
         0, 0, 0, 0, CHECK_CRC, CHECK_CRC,
         0, 0, NULL, 0, 0, 0, 0,
         0, 0,
         {0,0,0,0},{0,0,0,0}, 0,
         0, 0, 0
      }
   }
#endif
;

// The possible states from reading each sector.

// These are ORed into the state
// This is set if information is found such as sector our of the expected
// range. If MODEL controller mostly matches but say has one less sector than
// the disk has it was being selected instead of going on to ANALYZE 
#define ANALYZE_WRONG_FORMAT 0x1000
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
SECTOR_DECODE_STATUS perq_decode_track(DRIVE_PARAMS *drive_parms, int cyl, 
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
// MARK_DATA2 is looking for special ROHM PBX mark code.
// DATA_SYNC2 is looking for SUPERBRAIN
// PROCESS_HEADER is processing the header bytes and PROCESS_DATA processing
// the data bytes. HEADER_SYNC and DATA_SYNC are looking for the one bit to sync to
// in CONTROLLER_XEBEC_104786. Not all decoders use all states.
typedef enum { MARK_ID, MARK_DATA, MARK_DATA1, MARK_DATA2, HEADER_SYNC, HEADER_SYNC2, DATA_SYNC, DATA_SYNC2, PROCESS_HEADER, PROCESS_HEADER2, PROCESS_DATA
} STATE_TYPE;

SECTOR_DECODE_STATUS mfm_crc_bytes(DRIVE_PARAMS *drive_params, uint8_t bytes[],
    int bytes_crc_len, int state, uint64_t *crc_ret, int *ecc_span, 
    SECTOR_DECODE_STATUS *init_status, int perform_ecc);

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
SECTOR_DECODE_STATUS perq_process_data(STATE_TYPE *state, uint8_t bytes[],
      int total_bytes,
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span,
      SECTOR_DECODE_STATUS init_status);

int mfm_save_raw_word(DRIVE_PARAMS *drive_params, int all_raw_bits_count, 
   int int_bit_pos, int raw_word);
// Use data stored by mfm_mark_location instead of data passed in to
//   mfm_mark_header_location or mfm_mark_data_location
#define MARK_STORED -999
void mfm_mark_header_location(int bit_count, int bit_offset, int tot_bit_count);
void mfm_mark_data_location(int bit_count, int bit_offset, int tot_bit_count);
void mfm_mark_location(int bit_count, int bit_offset, int tot_bit_count);
void mfm_mark_end_data(int bit_count, DRIVE_PARAMS *drive_params, int cyl, int head);
void mfm_handle_alt_track_ch(DRIVE_PARAMS *drive_params, unsigned int bad_cyl, 
      unsigned int bad_head, unsigned int good_cyl, unsigned int good_head);
int mfm_fix_head(DRIVE_PARAMS *drive_params, int exp_head, int head);

#undef DEF_EXTERN
#endif /* MFM_DECODER_H_ */
