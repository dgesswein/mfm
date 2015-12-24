#ifndef MFM_DECODER_H_
#define MFM_DECODER_H_

//
// MFM is 17 or 18 sectors. This allows growth for RLL/ESDI
#define MAX_SECTORS 50
#define MAX_HEAD 16
#define MAX_CYL 4096
#define MAX_SECTOR_SIZE 4096 // Data size in bytes
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
   int num_ecc_recovered;
   int num_retries;
   int max_ecc_span;
} STATS;

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
   enum {CONTROLLER_NONE, CONTROLLER_NEWBURYDATA,
      CONTROLLER_WD_1006, CONTROLLER_OLIVETTI, CONTROLLER_MACBOTTOM, 
      CONTROLLER_ELEKTRONIKA_85,
      CONTROLLER_OMTI_5510, CONTROLLER_DEC_RQDX3, 
      CONTROLLER_SEAGATE_ST11M,
      CONTROLLER_ADAPTEC, 
      CONTROLLER_SYMBOLICS_3620, CONTROLLER_SYMBOLICS_3640, 
      CONTROLLER_MIGHTYFRAME, 
      CONTROLLER_XEBEC_104786, 
      CONTROLLER_CORVUS_H, CONTROLLER_NORTHSTAR_ADVANTAGE
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
   // Disables some header checks. Not currently used.
   int ignore_header_mismatch;
   // Drive number for setting select lines
   int drive;
   // Command line for driver parameters. Null if not set
   char *cmdline;
   // Input/output files
   int tran_fd;
   int emu_fd;
   TRAN_FILE_INFO *tran_file_info;
   EMU_FILE_INFO *emu_file_info;
   // Size of data for each emulator track. Only valid when writing
   // emulator file. TODO, use emu_file_info field instead
   int emu_track_data_bytes;
   // non zero if analyze option set
   int analyze;
   // What cylinder and head to analyze
   int analyze_cyl;
   int analyze_head;
   // What options have been set. Used by command line parsing and validation
   uint32_t opt_mask;
   // Command line note parameter
   char *note;
   // Time after index to start read in nanoseconds
   uint32_t start_time_ns;
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
  {0x00a00805, 32, 5},
  // Don't move this without fixing the Northstar reference
  {0x1021, 16, 0},
  {0x8005, 16, 0},
  {0x140a0445, 32, 5},
  {0x140a0444000101, 56, 16}, // From datasheet, not tested
  {0x0104c981, 32, 5},
  {0x24409, 24, 0},
  // Adaptec bad block on Maxtor XT-2190
  {0x41044185, 32, 5}
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
   {{-1, 0}, {-1, 0xffffffffffffffff}, {32, 0x2605fb9c}, {32, 0xd4d7ca20}}
#endif
;
// Smallest sector size should be first in list
DEF_EXTERN int mfm_all_sector_size[]
#ifdef DEF_DATA
 = {256, 512, 524, 1024, 1160, 1164, 2048, 4096, -1}
  // -1 marks end of array
#endif
;

// Number of sectors to search for LBA format
DEF_EXTERN int mfm_lba_num_sectors[]
#ifdef DEF_DATA
 = {17, 32, 33, -1}
  // -1 marks end of array
#endif
;

DEF_EXTERN struct {
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
      // These bytes at the end of the data area are included in the CRC
      // but should not be written to the extract file.
   int data_trailer_bytes;
      // 1 if data area is separate from header. 0 if one CRC covers both
   int separate_data;
} mfm_controller_info[]

// Keep sorted by header length. MUST MATCH order of controller enum
#ifdef DEF_DATA
   = {
      {"CONTROLLER_NONE",        0, 10000000,      0, 
         0,0, 0,0,
         0,0, CINFO_NONE,
         0, 0, 0, 0, 
         0, 0},
      {"NewburyData",          256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         4, 2, 0, 0, 
         0, 1},
      {"WD_1006",              256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0,
         0, 1},
      {"Olivetti",             256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 2, 2, 
         0, 1},
      {"MacBottom",            256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0,
         0, 1},
      {"Elektronika_85",      256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         5, 2, 0, 0, 
         16, 1},
      {"OMTI_5510",            256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0,
         0, 1},
      {"DEC_RQDX3",            256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0,
         0, 1},
      {"Seagate_ST11M",        256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         6, 2, 0, 0,
         0, 1},
//TODO, this won't analyze properly
      {"Adaptec",              256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_LBA,
         6, 2, 0, 0,
         0, 1},
      {"Symbolics_3620",       256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         7, 3, 3, 3,
         0, 1},
      {"Symbolics_3640",       256, 10000000,      0, 
         0, 1, 3, ARRAYSIZE(mfm_all_poly), 
         0, 1, CINFO_CHS,
         11, 1, 0, 1,
         0, 1},
// This format is detected by special case code so it doesn't need to
// be sorted by number
      {"Mightyframe",          256, 10000000,      0, 
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_NONE,
         5, 2, 0, 0, 
         0, 1},
// END of WD type controllers
      {"Xebec_104786",         256, 10000000,      0,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         9, 2, 0, 0, 
         0, 1},
      {"Corvus_H",             512, 11000000,  312000,
         3, ARRAYSIZE(mfm_all_poly), 3, ARRAYSIZE(mfm_all_poly), 
         0, ARRAYSIZE(mfm_all_init), CINFO_CHS,
         3, 0, 0, 0, 
         0, 0},
      {"NorthStar_Advantage",  256, 10000000, 230000,
         1, 2, 2,3,
         0, 1, CINFO_CHS,
         7, 0, 0, 0, 
         0, 1},
      {NULL, 0, 0, 0,
         0,0, 0,0, CINFO_NONE,
         0, 0, 0, 0, 
         0, 0},
   }
#endif
;

// The possible states from reading each sector.

// These are ORed into the state
#define SECT_ZERO_HEADER_CRC 0x40
#define SECT_ZERO_DATA_CRC  0x20
#define SECT_HEADER_FOUND   0x10
#define SECT_ECC_RECOVERED  0x08
#define SECT_WRONG_CYL      0x04
// Only one of two will be set
#define SECT_BAD_HEADER     0x02
#define SECT_BAD_DATA       0x01
#define SECT_NO_STATUS      0x00
typedef uint32_t SECTOR_DECODE_STATUS;
#define UNRECOVERED_ERROR(x) (x & (SECT_BAD_HEADER | SECT_BAD_DATA))

// The state of a sector
typedef struct {
   // The span of any ECC correction. 0 if no correction
   int ecc_span_corrected_data;
   int ecc_span_corrected_header;
   // The difference between the expected and actual cylinder found
   int cyl_difference;
   // The values from the sector header
   int cyl, head, sector;
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
   DRIVE_PARAMS *drive_params);
void mfm_decode_setup(DRIVE_PARAMS *drive_params, int write);
void mfm_decode_done(DRIVE_PARAMS *drive_params);
int mfm_write_sector(uint8_t bytes[], DRIVE_PARAMS *drive_params,
   SECTOR_STATUS *sector_status, SECTOR_STATUS bad_sector_list[],
   uint8_t all_bytes[], int all_bytes_len);
void mfm_init_sector_status_list(SECTOR_STATUS *sector_status_list, int num_sectors); 
void mfm_dump_bytes(uint8_t bytes[], int len, int cyl, int head,
      int sector_index, int msg_level);

// Define states for processing the data. MARK_ID is looking for the 0xa1 byte
// before a header and MARK_DATA is same for the data portion of the sector.
// MARK_DATA1 is looking for special Symbolics 3640 mark code.
// PROCESS_HEADER is processing the header bytes and PROCESS_DATA processing
// the data bytes. HEADER_SYNC and DATA_SYNC are looking for the one bit to sync to
// in CONTROLLER_XEBEC_104786. Not all decoders use all states.
typedef enum { MARK_ID, MARK_DATA, MARK_DATA1, HEADER_SYNC, DATA_SYNC, PROCESS_HEADER, PROCESS_DATA
} STATE_TYPE;

SECTOR_DECODE_STATUS mfm_process_bytes(DRIVE_PARAMS *drive_params, uint8_t bytes[],
      int bytes_crc_len, STATE_TYPE *state, int cyl, int head, int *sector_index,
      int *seek_difference, SECTOR_STATUS sector_status_list[]);

SECTOR_DECODE_STATUS wd_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span);
SECTOR_DECODE_STATUS xebec_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span);
SECTOR_DECODE_STATUS corvus_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span);
SECTOR_DECODE_STATUS northstar_process_data(STATE_TYPE *state, uint8_t bytes[],
      uint64_t crc, int exp_cyl, int exp_head, int *sector_index,
      DRIVE_PARAMS *drive_params, int *seek_difference,
      SECTOR_STATUS sector_status_list[], int ecc_span);

int mfm_save_raw_word(DRIVE_PARAMS *drive_params, int all_raw_bits_count, 
   int int_bit_pos, int raw_word);
void mfm_mark_header_location(int bit_count);
void mfm_mark_data_location(int bit_count);
void mfm_mark_end_data(int bit_count, DRIVE_PARAMS *drive_params);
#undef DEF_EXTERN
#endif /* MFM_DECODER_H_ */
