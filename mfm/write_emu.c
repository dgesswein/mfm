#define VERSION "1.0pre11"
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "crc_ecc.h"
#include "msg.h"
#include "emu_tran_file.h"
#define DEF_DATA
#include "mfm_decoder.h"

uint32_t reverse_bits(uint32_t value, int len_bits) {
   uint32_t new_value = 0;
   int i;

   for (i = 0; i < len_bits; i++) {
      new_value = (new_value << 1) | (value & 1);
      value >>= 1;
   } 
   return new_value;
}

static int list_size_bytes;
static int *sector_used_list;
static int sector_used_count;
static int sector;
static int track_start_sector;
static int sector_interleave;

static int track_interleave;

static int head, cyl;

void set_head(int head_i) {
   head = head_i;
}

int get_head() {
   return head;
}

void set_cyl(int cyl_i) {
   cyl = cyl_i;
}

int get_cyl() {
   return cyl;
}

void set_sector_interleave(DRIVE_PARAMS *drive_params,
     int sector_interleave_i, int track_interleave_i) {

   list_size_bytes = sizeof(*sector_used_list) * drive_params->num_sectors; 
   sector_interleave = sector_interleave_i;
   sector_used_list = msg_malloc(list_size_bytes,"sector_used_list");
   track_interleave = track_interleave_i;

   memset(sector_used_list, 0, list_size_bytes);
}

void start_new_track(DRIVE_PARAMS *drive_params) {
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

void start_new_cyl(DRIVE_PARAMS *drive_params) {
   track_start_sector = 0;
}

int get_sector() {
  return sector; 
}

void inc_sector(DRIVE_PARAMS *drive_params) 
{
   sector_used_list[sector] = 1;
   sector_used_count++;
   if (sector_used_count < drive_params->num_sectors) {
      sector += sector_interleave;
      while (sector_used_list[sector]) {
         sector = (sector + 1) % drive_params->num_sectors;
      }
   }
}

int get_lba(DRIVE_PARAMS *drive_params) {
   return (get_cyl() * drive_params->num_head + get_head()) *
       drive_params->num_sectors + sector;
}

uint64_t get_check_value(uint8_t track[], int length, CRC_INFO *crc_info,
   CHECK_TYPE check_type) {
   uint64_t value;

   if (check_type == CHECK_CRC) {
      value = crc64(track, length, crc_info);
   } else if (check_type == CHECK_CHKSUM) {
      value = checksum64(track, length, crc_info);
      if (crc_info->length == 16) {
         value = value & 0xff;
      } else if (crc_info->length == 32) {
         value = value & 0xffff;
      } else {
         msg(MSG_FATAL, "Unsupported checksum length %d\n",crc_info->length);
         exit(1);
      }
   } else if (check_type == CHECK_PARITY) {
      value = parity64(track, length, crc_info);
   } else {
      msg(MSG_FATAL, "Unknown check_type %d\n",check_type);
      exit(1);
   }
   return value;
}

void get_data(DRIVE_PARAMS *drive_params, uint8_t track[]) {
   int block;
   int rc;

   block = (get_cyl() * drive_params->num_head + get_head()) *
       drive_params->num_sectors + get_sector();

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
void process_field(DRIVE_PARAMS *drive_params, 
   uint8_t track[], int trk_offset, int length, 
   FIELD_L field_def[], int a1_list[], int *a1_list_ndx, int a1_list_len)
{
   int ndx = 0;
   uint64_t value;
   int i;
   int data_set;
   int crc_start = 0;
   int crc_end = -1;
   int field_filled = 0;

//printf("Process field called start %d\n",trk_offset);

   while (field_def[ndx].len_bytes != -1) {
      data_set = 0;
      switch (field_def[ndx].type) {
         case FIELD_FILL:
            if (field_def[ndx].byte_offset_bit_len +
                field_def[ndx].len_bytes > length) {
               msg(MSG_FATAL, "Track overflow field fill %d, %d, %d\n",
                 field_def[ndx].byte_offset_bit_len, field_def[ndx].len_bytes,
                 length);
               exit(1);
            }
            memset(&track[field_def[ndx].byte_offset_bit_len], 
               field_def[ndx].value, field_def[ndx].len_bytes);
            data_set = 1;
            if (field_def[ndx].op != OP_SET) {
               msg(MSG_FATAL, "Only OP_SET currently supported for FIELD_FILL\n");
               exit(1);
            }
         break;
         case FIELD_CYL:
            value = get_cyl();
         break;
         case FIELD_HEAD:
            value = get_head();
         break;
         case FIELD_SECTOR:
            value = get_sector();
         break;
         case FIELD_LBA:
            value = get_lba(drive_params);
         break;
         case FIELD_HDR_CRC:
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
         break;
         case FIELD_MARK_CRC_START:
            crc_start = field_def[ndx].byte_offset_bit_len;
            data_set = 1;
         break;
         case FIELD_MARK_CRC_END:
            crc_end = field_def[ndx].byte_offset_bit_len;
            data_set = 1;
         break;
         case FIELD_TRK_DATA:
            get_data(drive_params, &track[field_def[ndx].byte_offset_bit_len]);
            data_set = 1;
            inc_sector(drive_params);
         break;
         case FIELD_BAD_BLOCK:
            value = 0;
         break;
         case FIELD_A1:
            if (*a1_list_ndx >= a1_list_len) {
               msg(MSG_FATAL, "A1 list overflow\n");
               exit(1);
            }      
            a1_list[(*a1_list_ndx)++] = trk_offset + 
                field_def[ndx].byte_offset_bit_len;
            value = 0xa1;
         break;
         default:
            msg(MSG_FATAL, "Unknown field_def type %d\n",field_def[ndx].type);
            exit(1);
      }
      if (data_set) {
         field_filled = MAX(field_filled, field_def[ndx].byte_offset_bit_len + field_def[ndx].len_bytes - 1);
      } else if (!data_set && field_def[ndx].bit_list == NULL) {
         field_filled = MAX(field_filled, field_def[ndx].byte_offset_bit_len + field_def[ndx].len_bytes - 1);
//printf("value %lld len %d %d %d\n",value, field_def[ndx].byte_offset_bit_len , field_def[ndx].len_bytes, length);
         if (field_def[ndx].byte_offset_bit_len + field_def[ndx].len_bytes 
              > length) {
            msg(MSG_FATAL, "Track overflow field update %d %d %d\n", 
               field_def[ndx].byte_offset_bit_len, field_def[ndx].len_bytes, length);
            exit(1);
         }
         value <<= (sizeof(value) - field_def[ndx].len_bytes) * 8;
         for (i = 0; i < field_def[ndx].len_bytes; i++) {
            int wbyte = (value >> (sizeof(value)*8 - 8));
            if (field_def[ndx].op == OP_XOR) {
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

         if (field_def[ndx].op == OP_REVERSE) {
            value = reverse_bits(value, field_def[ndx].byte_offset_bit_len);
         }
         while (bit_list[ndx2].bitl_start != -1) {
            for (i = 0; i < bit_list[ndx2].bitl_length; i++) {
               byte_offset = (bit_list[ndx2].bitl_start + i) / 8; 
               field_filled = MAX(field_filled, byte_offset);
               bit_offset = (bit_list[ndx2].bitl_start + i) % 8; 
               if (byte_offset >= length) {
                  msg(MSG_FATAL, "Track overflow bit field\n");
                  exit(1);
               }
               temp = (value >> (field_def[ndx].byte_offset_bit_len - 
                  bit_count++ - 1) & 1) << (7 - bit_offset);
               if (field_def[ndx].op == OP_XOR) {
                  track[byte_offset] ^= temp;
               } else {
                  track[byte_offset] &= ~(1 << (7 - bit_offset));
                  track[byte_offset] |= temp;
               } 
            }
            ndx2++;
         }
         if (bit_count != field_def[ndx].byte_offset_bit_len) {
            msg(MSG_FATAL, "Bit field length mismatch %d %d\n", 
               bit_count, field_def[ndx].byte_offset_bit_len);
            exit(1);
         }
      }
      ndx++;
   }
   if (field_filled != length - 1) {
      msg(MSG_FATAL, "Incorrect field length %d %d\n",field_filled, length);
      exit(1);
   }
}

int process_track(DRIVE_PARAMS *drive_params,
   uint8_t track[], int start, int length, TRK_L track_def[],
   int a1_list[], int *a1_list_ndx, int a1_list_len)
{
   int ndx = 0;
   int new_start;
   int i;

//printf("Process track called start %d\n",start);
   while (track_def[ndx].count != -1) {
      switch (track_def[ndx].type) {
         case TRK_FILL:
            for (i = 0; i < track_def[ndx].count; i++) {
               if (start >= length) {
                  msg(MSG_FATAL, "Track overflow fill, %d %d %d\n", start, length, i);
                  exit(1);
               }
               track[start++] = track_def[ndx].value;
            } 
         break;
         case TRK_SUB:
            for (i = 0; i < track_def[ndx].count; i++) {
               start = process_track(drive_params, track, start, length, 
                  (TRK_L *) track_def[ndx].list, 
                  a1_list, a1_list_ndx, a1_list_len);
            }
         break;
         case TRK_FIELD:
            new_start = start + track_def[ndx].count;
            if (new_start >= length) {
               msg(MSG_FATAL, "Track overflow field\n");
               exit(1);
            }
            memset(&track[start], track_def[ndx].value, track_def[ndx].count);
            process_field(drive_params, &track[start], start, 
               track_def[ndx].count, 
               (FIELD_L *) track_def[ndx].list, a1_list, a1_list_ndx,
               a1_list_len);
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


// Parse controller value (track/sector header format). The formats are named
// after the controller that wrote the format. Multiple controllers may use
// the same format.
//
// arg: Controller string
// return: Controller number
static int parse_controller(char *arg) {
   int i;
   int controller = -1;

   for (i = 0; mfm_controller_info[i].name != NULL; i++) {
      if (strcasecmp(mfm_controller_info[i].name, arg) == 0) {
         controller = i;
      }
   }
   if (controller == -1) {
      msg(MSG_FATAL, "Unknown controller %s. Choices are\n",arg);
      for (i = 0; mfm_controller_info[i].name != NULL; i++) {
         msg(MSG_FATAL,"%s\n",mfm_controller_info[i].name);
      }
      exit(1);
   }
   return controller;
}

// Minimum to generate extract file (sector data)
static int min_opts = 0x1f;
static struct option long_options[] = {
         {"heads", 1, NULL, 'h'},
         {"cylinders", 1, NULL, 'c'},
      //   {"header_crc", 1, NULL, 'g'},
      //   {"data_crc", 1, NULL, 'j'},
         {"format", 1, NULL, 'f'},
      //   {"sector_length", 1, NULL, 'l'},
      //   {"interleave", 1, NULL, 'i'},
         {"extracted_data_file", 1, NULL, 'e'},
         {"emulation_file", 1, NULL, 'm'},
         {"quiet", 1, NULL, 'q'},
         {"version", 0, NULL, 'v'},
         {NULL, 0, NULL, 0}
};
//static char short_options[] = "h:c:g:d:f:j:l:ui:3r:a::q:b:t:e:m:vn:";
static char short_options[] = "h:c:f:q:e:m:v";

// Main routine for parsing command lines
//
// argc, argv: Main argc, argv
// drive_parameters: Drive parameters where most of the parsed values are stored
// delete_options: Options to delete from list of valid options (short option)
// initialize: 1 if drive_params should be initialized with defaults
// only_deleted: 1 if we only want to process options specified in 
//    delete_options. Other options are ignored, not error
// ignore_invalid_options: Don't exit if option is not known
void parse_cmdline(int argc, char *argv[], DRIVE_PARAMS *drive_params)
{
   int rc;
   // Loop counters
   int i;
   int options_index;

   // Options above are superset for all the programs to ensure options stay consistent
   // Enable all errors other than debug
   msg_set_err_mask(~0 ^ (MSG_DEBUG | MSG_DEBUG_DATA));
   memset(drive_params, 0, sizeof(*drive_params));
   // Set defaults
   drive_params->emu_fd = -1;
   drive_params->tran_fd = -1;
   drive_params->sector_size = 512;

   // Handle the options. The long options are converted to the short
   // option name for the switch by getopt_long.
   options_index = -1;
   optind = 1; // Start with first element. We may call again with same argv
   while ((rc = getopt_long(argc, argv, short_options, long_options,
         &options_index)) != -1) {
      // Short options don't set options_index so look it up
      if (options_index == -1) {
         for (i = 0; i < ARRAYSIZE(long_options); i++) {
            if (rc == long_options[i].val) {
               options_index = i;
               break;
            }
         }
         if (options_index == -1) {
            // If only deleted specified don't print error. Option will
            // be ignored below. The valid options would be printed with
            // only the few options selected which would confuse the user
            msg(MSG_FATAL, "Error parsing option %c\n",rc);
            msg(MSG_FATAL,"Valid options:\n");
            for (i = 0; long_options[i].name != NULL; i++) {
               msg(MSG_FATAL, "%c %s\n", long_options[i].val, 
                  long_options[i].name);
            }
            exit(1);
         }
      }
      // If option is not found error
      if (options_index == -1) {
         msg(MSG_FATAL,"Option '%c' not valid for this program\n",
            rc);
         exit(1);
      } else {
         drive_params->opt_mask |= 1 << options_index;
         switch(rc) {
         case 'h':
            drive_params->num_head = atoi(optarg);
            if (drive_params->num_head <= 0 || 
                  drive_params->num_head > MAX_HEAD) {
               msg(MSG_FATAL,"Heads must be 1 to %d\n", MAX_HEAD);
               exit(1);
            }
            break;
         case 'c':
            drive_params->num_cyl = atoi(optarg);
            if (drive_params->num_cyl <= 0) {
               msg(MSG_FATAL,"Cylinders must be greater than 0\n");
               exit(1);
            }
            break;
         case 'g':
          //  drive_params->header_crc = parse_crc(optarg);
            break;
         case 'j':
          //  drive_params->data_crc = parse_crc(optarg);
            break;
         case 'i':
         //   drive_params->sector_numbers = parse_interleave(optarg, drive_params);
            break;
         case 'f':
            drive_params->controller = parse_controller(optarg);
            break;
         case 'e':
            drive_params->extract_filename = optarg;
            break;
         case 'm':
            drive_params->emulation_filename = optarg;
            break;
         case 'q':
            msg_set_err_mask(~strtoul(optarg, NULL, 0));
            break;
         case 'v':
            msg(MSG_INFO_SUMMARY,"Version %s\n",VERSION);
            break;
         case '?':
            msg(MSG_FATAL, "Didn't process argument %c\n", rc);
            exit(1);
            break;
         default:
            msg(MSG_FATAL, "Didn't process argument %c\n", rc);
            exit(1);
         }
      }
      options_index = -1;
   }
   if (optind < argc) {
      msg(MSG_FATAL, "Uknown option %s specified\n",argv[optind]);
      exit(1);
   }
}

// This validates options where we need the options list for messages
//
// drive_params: Drive parameters
// mfm_read: 1 if mfm_read, 0 if mfm_util
void parse_validate_options(DRIVE_PARAMS *drive_params) {
   int i;

   if ((drive_params->opt_mask & min_opts) !=
               min_opts) {
      msg(MSG_FATAL, "Requires options:\n");
      for (i = 0; i < 32; i++) {
         int bit = (1 << i);
         if (!(drive_params->opt_mask & bit) && 
               (min_opts & bit)) {
            msg(MSG_FATAL, " %s", long_options[i].name);
         }
      }
      msg(MSG_FATAL, "\n");
      exit(1);
   }
}

void mfm_encode(uint8_t data[], int length, uint32_t mfm_data[], int a1_list[],
   int a1_list_length) 
{
   static uint16_t mfm_encode[2][256]; 
   static int first_time = 1;
   int last_bit = 0;
   int i, lbc;
   int bit;
   uint16_t value16;
   uint32_t value32 = 0;
   int ext_bit;
   int a1_list_ndx = 0;

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
      if (a1_list_ndx < a1_list_length && i == a1_list[a1_list_ndx]) {
         value16 = 0x4489;
         a1_list_ndx++;
      } else {
         value16 = mfm_encode[last_bit][data[i]];
      }
      if (i & 1) {
         value32 = value32 << 16 | value16;
         mfm_data[i/2] = value32;
      } else {
         value32 = value16;
      }
      last_bit = value16 & 1;
   }
}
int main(int argc, char *argv[])
{
   uint8_t *track;
   int track_length;
   uint32_t *track_mfm;
   int a1_list[100];
   int a1_list_ndx = 0;
   int track_filled = 0;
   DRIVE_PARAMS drive_params;
   struct stat finfo;
   int calc_size;

   parse_cmdline(argc, argv, &drive_params);
   parse_validate_options(&drive_params);

   if (mfm_controller_info[drive_params.controller].track_layout == NULL) {
      msg(MSG_FATAL, "Not yet able to process format %s\n",
         mfm_controller_info[drive_params.controller].name);
      exit(1);
   }

   drive_params.num_sectors = mfm_controller_info[drive_params.controller].write_num_sectors;
   drive_params.sector_size = mfm_controller_info[drive_params.controller].write_sector_size;
   drive_params.cmdline = "";
   drive_params.data_crc = mfm_controller_info[drive_params.controller].write_data_crc;
   drive_params.header_crc = mfm_controller_info[drive_params.controller].write_header_crc;
   drive_params.emu_track_data_bytes = mfm_controller_info[drive_params.controller].track_words * 4;
   drive_params.start_time_ns = mfm_controller_info[drive_params.controller].start_time_ns;

   drive_params.ext_fd = open(drive_params.extract_filename, O_RDONLY);
   if (drive_params.ext_fd < 0) {
      msg(MSG_FATAL, "Unable to open extract file: %s\n", strerror(errno));
      exit(1);
   }
   drive_params.emu_fd = emu_file_write_header(drive_params.emulation_filename,
        drive_params.num_cyl, drive_params.num_head,
        drive_params.cmdline, drive_params.note,
        mfm_controller_info[drive_params.controller].clk_rate_hz,
        drive_params.start_time_ns, drive_params.emu_track_data_bytes);

   fstat(drive_params.ext_fd, &finfo);
   calc_size = drive_params.sector_size * drive_params.num_sectors * 
        drive_params.num_head * drive_params.num_cyl;
   if (calc_size != finfo.st_size) {
      msg(MSG_INFO, "Calculated extract file size %d bytes, actual size %d\n",
        calc_size, finfo.st_size);
   }

   set_sector_interleave(&drive_params, 1, 0);

   memset(a1_list, 0, sizeof(a1_list));

   track_length = drive_params.emu_track_data_bytes / 2;
   track = msg_malloc(track_length , "Track data");
   track_mfm = msg_malloc(drive_params.emu_track_data_bytes, "Track data bits");

   for (cyl = 0; cyl < drive_params.num_cyl; cyl++) {
      set_cyl(cyl);
      start_new_cyl(&drive_params);
      for (head = 0; head < drive_params.num_head; head++) {
         set_head(head);
         start_new_track(&drive_params);
         memset(track, 0, track_length);
         track_filled = process_track(&drive_params, track, 0, track_length,
            mfm_controller_info[drive_params.controller].track_layout, 
            a1_list, &a1_list_ndx, ARRAYSIZE(a1_list));
         mfm_encode(track, track_length, track_mfm, a1_list, a1_list_ndx);
         emu_file_write_track_bits(drive_params.emu_fd, (uint32_t *)track_mfm,
               drive_params.emu_track_data_bytes/4, cyl, head,
               drive_params.emu_track_data_bytes);

         a1_list_ndx = 0;
      }
   }
   if (track_filled != track_length) {
      msg(MSG_INFO, "Not all track filled, %d of %d bytes uses\n",
        track_filled, track_length);
   }
   emu_file_close(drive_params.emu_fd, 1);
}


//TODO handle mfm_controller_info[drive_params->controller].data_trailer_bytes
//handle marking bad sectors
