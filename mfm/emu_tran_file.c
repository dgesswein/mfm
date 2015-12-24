// These routines are used to read and write the emulator and transition
// file formats for the MFM disk programs.
//
// emu routines are for emulator file format and tran routines for transition/
//    delta file format.
// internal delta format is an array of 16-bit transition delta times for
//    easy handling, while the transition file format is a packed version
//    defined below.  (generally "delta" is internal and "transition"
//    external, but this is not guaranteed consistent.)
//
// Call emu_file_read_track_deltas to read next track of data and convert to 
//    delta format.
// Call emu_file_write_header to open file for writing and write out
//    header data. File will be created or truncated.
// Call emu_file_read_header to open file for reading or read/write.
// Call emu_file_write_track_bits to write next track of emulation file data.
// Call emu_file_rewrite_track to update a track in emulation file.
// Call emu_file_read_track_bits to read next track of emulation file data.
// Call emu_file_read_cyl to read a cylinder of emulation file data.
// Call emu_file_write_cyl to write a cylinder of emulation file data.
// Call emu_file_close to close emulation file.
// Call emu_file_seek_track to seek to desired cylinder and head
//
// Call tran_file_close to close transition file.
// Call tran_file_read_header to open file for reading.
// Call tran_file_write_header to open file for writing and write out
//    header data. File will be created or truncated.
// Call tran_file_read_track_deltas to read next track of data and convert to 
//    delta format.
// Call tran_file_write_track_deltas to write next track of data, converting  
//    from delta format.
// Call tran_file_seek_track to seek to desired cylinder and head
//
// Delta format is count of clocks between ones in 200 MHz clocks.
//
// All files are read and written little endian. 
// File version word 
//    Upper 8 bits the file type. 
//       1 = transition, 2 = emulation.
//    Next 8 bits major version. This field will be increased if file format
//       is changed incompatible with old programs.
//    Next 8 bits is minor version. This field will be increased if file format
//       changed but older programs should be able to use. Programs should
//       handle increases in header size. New fields will be added after
//       existing fields.
//    Next 8 bits are unused and should be zero.
//
// Emulator file header format. See code for which fields are present
//    in which file version.
//    uint8_t[8] File id string 0xee 0x4d 0x46 0x4d 0x0d 0x0a 0x1a 0x00
//    uint32_t File type and version. Current 0x02020200. 
//    uint32_t Offset to start of first track header in bytes.
//    uint32_t Size of the data portion of each track in bytes.
//    uint32_t Size of the header portion of each track in bytes.
//    uint32_t Number of cylinders of track data in file.
//    uint32_t Number of heads/tracks of data per cylinder.
//    uint32_t Bit rate in Hz.
//    uint32_t command line length in bytes
//    uint8_t[n] Command line used to create emulator file. 0 terminated.
//    uint32_t Note length in bytes
//    uint8_t[n] Note string. 0 terminated.
//    uint32_t Start of track time from index in nanoseconds
// Emulator file track header format
//    uint32_t 0x12345678, Value to mark header.
//    int32_t Cylinder number of track
//    int32_t Head number of track
//    Cylinder and head of -1 indicate no more data. No track data will
//       follow end of file marker.
// Emulator file track data format
//    uint32_t of MFM clock and data bits for length specified in file header.
//        Bit 31 is the first bit.
//
// Transition file header format. See code for which fields are present
//    in which file version.
//    uint8_t[8] File id string 0xee 0x4d 0x46 0x4d 0x0a 0x1a 0x0a 0x0d
//    uint32_t File type and version. Current 0x01020200
//    uint32_t Offset to start of first track header in bytes.
//    uint32_t Size of the header portion of each track in bytes.
//    uint32_t Number of cylinders of track data in file.
//    uint32_t Number of heads/tracks of data per cylinder.
//    uint32_t Transition count rate in Hz. Only 200 MHz currently supported.
//    uint32_t Length of command line used when capturing.
//    uint8_t  Command line used when capturing. Zero terminated.
//    uint32_t Length of note option.
//    uint8_t  Note option text. Zero terminated.
//    uint32_t Start time of data from index in nanoseconds.
//    uint32_t Checksum of header. Calculated over bytes using crc64 in
//       this program suite. Polynomial 0x140a0445 length 32 initial value
//       0xffffffff.
//
// Transition file track header format 
//    int32_t Cylinder number of track
//    int32_t Head number of track
//    uint32_t Number of bytes of transition data
//    uint32_t Checksum of header and track data. Calculated over bytes 
//    using crc64 in this program suite. Polynomial 0x140a0445 length 
//       32 initial value 0xffffffff.
//    Cylinder and head of -1 indicate no more data.
//
// Transitions are count of clocks between one bits stored as bytes.
//    A transition of 255 indicates the next 3 bytes are a 24 bit transition
//    value. A transition value of 254 indicates the next 2 bytes are a 16
//    bit transition value. Otherwise the transition count is a single byte.
//    Clock transition count clock frequency is in file header. For 200 MHz
//    a count of 40 indicates 5 MHz pulse spacing.
//
// 12/24/15 DJG Cleanup and output valid MFM data if needing to pad track
// 05/16/15 DJG Added routines needed for analyze to work on transition and
//    emulation files.
// 01/04/15 Added using sample rate field to emulation file. Added
//    start_time_ns to emu file header and incremented file version. Allowed
//    emulator track file length to be set.
//    Added start_time_ns to transition file header and incremented file
//    version.
// 10/19/14 DJG Added and modified functions to allow emulator file writes to 
// 	be buffered
//
// Copyright 2013 David Gesswein.
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
//
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

#include "msg.h"
#include "emu_tran_file.h"
#include "crc_ecc.h"
// Only for CLOCKS_TO_NS
#include "mfm_decoder.h"

// Maximum number of delta bytes in a track we support. For 60 RPM and
// 10 MHz rate should have 166666 deltas. For future RLL 50 RPM and 15 MHz
// should have 300000. Padded more.
#define MAX_BYTE_DELTAS 400000
// EMU file header marker
#define TRACK_ID_VALUE 0x12345678
// File header marker
uint8_t expected_header_id[] = 
   { 0xee, 0x4d, 0x46, 0x4d, 0x0d, 0x0a, 0x1a, 0x00};
// File type 2, major version 1, minor version 2
#define EMU_FILE_VERSION 0x02020200
// File type 1, major version 2, minor version 2
#define TRAN_FILE_VERSION 0x01020200

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

static CRC_INFO trans_initial_poly =
{   .poly = 0x140a0445,
    .length = 32,
    .init_value = 0xffffffff,
    .ecc_max_span = 0
};


// Internal routines for reading and writing file.
// fd: File descriptor to read/write
// bytes: Data to read/write
// len: length of data in bytes
static void emu_file_write(int fd, void *bytes, int len) {
   int rc;
   if ((rc = write(fd, bytes, len)) != len) {
      msg(MSG_FATAL, "Failed to write bytes to emulation file %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   }
}
static void emu_file_read(int fd, void *bytes, int len) {
   int rc;
   if ((rc = read(fd, bytes, len)) != len) {
      msg(MSG_FATAL, "Failed to read bytes from emulation file %d %s\n", rc,
            rc == -1 ? strerror(errno) : "");
      exit(1);
   }
}

// Find the desired cylinder and head in the transition file
// fd: File descriptor to read from
// seek_cyl: Cylinder number to find
// seek_head: head number to find
int emu_file_seek_track(int fd, int seek_cyl, int seek_head, 
      EMU_FILE_INFO *emu_file_info) {
   off_t offset;
   int track_size = (emu_file_info->track_data_size_bytes + 
          emu_file_info->track_header_size_bytes);

   // If called with invalid head or cylinder return error
   if (seek_head >= emu_file_info->num_head || seek_cyl >=
      emu_file_info->num_cyl) {
      return 1;
   }

   offset = (off_t) seek_cyl * track_size * emu_file_info->num_head + 
      seek_head * track_size + emu_file_info->file_header_size_bytes;

   lseek(fd, offset , SEEK_SET);

   return 0;
}

// This calls bit read routine and then converts the bits into deltas.
// The deltas are returned assuming 200 MHz PRU sample clock for delta time. 
//
// fd: File descriptor to read from
// emu_file_info: Information on emulator file format
// deltas: Output delta times between ones
// max_deltas: Size of deltas buffer in words
// cyl: Cylinder number of track read
// head: head number of track read
// start_time_ns: Start this number of nanoseconds into the emu file data
//    wrapping around at end
// return: Number of deltas read in words. -1 if end of file found.
//TODO: mfm_read/mfm_emu don't set the PRU clock to the frequency to
//   make deltas integer for bit rate like mfm_emu does. This should be
//   fixed. This code takes into account the actual delta times vs the
//   emu file clock rate such that the deltas match what would be
//   captured with the 200 MHz PRU clock
int emu_file_read_track_deltas(int fd, EMU_FILE_INFO *emu_file_info,
      uint16_t deltas[], int max_deltas, int *cyl, int *head)
{
   uint32_t bits[10000];   // Track bits read
   int num_words;          // Size of buffer and number of bytes read
   int num_deltas;         // Index of deltas building
   int delta_time;
   int wc, bc;             // Word and bit counter
   double bit_time = 0;
   int delta;

   if ((num_words = emu_file_read_track_bits(fd, emu_file_info, bits,
         ARRAYSIZE(bits), cyl, head)) == -1) {
      num_deltas = -1;
   } else {
      num_deltas = 0;
      delta_time = 0;
      for (wc = 0; wc < num_words; wc++) {
         for (bc = 0; bc < 32; bc++) {
            delta = rint(bit_time / CLOCKS_TO_NS);
            delta_time += delta;
            bit_time += 1e9 / emu_file_info->sample_rate_hz - delta * CLOCKS_TO_NS;
            if (bits[wc] & 0x80000000) {
               deltas[num_deltas++] = delta_time;
               if (num_deltas >= max_deltas) {
                  msg(MSG_FATAL, "Emulation file delta overflow\n");
                  exit(1);
               }
               delta_time = 0;
            }
            bits[wc] <<= 1;
         }
      }
   }
   return num_deltas;
}

// Create or truncate file and write emulator file header.
//
// fn: File name to write to
// num_cyl: Number of cylinder file will contain
// num_head: Number of tracks per cylinder
// cmdline: Command line string to store in header
// note: Note to store in header
// sample_rate_hz: The MFM clock and data bit rate in Hertz
// start_time_ns: The time from index the data capture was started in nanosecond
// track_bytes: The size of each track data in bytes
// return: file descriptor of file opened
int  emu_file_write_header(char *fn, int num_cyl, int num_head, char *cmdline,
      char *note, uint32_t sample_rate_hz, uint32_t start_time_ns, 
      uint32_t track_bytes) {
   uint32_t value;
   int fd;
   char *lcl_note = "";
   int track_start;

   // If note null store empty string
   if (note != NULL) {
      lcl_note = note;
   }


   fd = open(fn, O_WRONLY | O_CREAT | O_TRUNC, 0664);
   if (fd < 0) {
      msg(MSG_FATAL, "Failed to open emulation file %s: %s\n",
            fn, strerror(errno));
      exit(1);
   }

   emu_file_write(fd, expected_header_id, sizeof(expected_header_id));
   // File type 2, major version 1, minor version 1
   value = EMU_FILE_VERSION;
   emu_file_write(fd, &value, sizeof(value));
   // Offset of first track header
   value = sizeof(expected_header_id) + 4*10 + strlen(cmdline)+1 + 
       strlen(lcl_note)+1;
   track_start = value;
   emu_file_write(fd, &value, sizeof(value));
   value = track_bytes;
   emu_file_write(fd, &value, sizeof(value));
   // Offset of track data from start of track
   value = 4*3;
   emu_file_write(fd, &value, sizeof(value));
   value = num_cyl;
   emu_file_write(fd, &value, sizeof(value));
   value = num_head;
   emu_file_write(fd, &value, sizeof(value));
   value = sample_rate_hz;
   emu_file_write(fd, &value, sizeof(value));

   if (cmdline == NULL) {
      msg(MSG_FATAL,"emu_file_write_header cmdline not specified\n");
      exit(1);
   }
   // Write length including terminating null
   value = strlen(cmdline)+1;
   emu_file_write(fd, &value, sizeof(value));
   emu_file_write(fd, cmdline, value);

   value = strlen(lcl_note)+1;
   emu_file_write(fd, &value, sizeof(value));
   emu_file_write(fd, lcl_note, value);

   value = start_time_ns;
   emu_file_write(fd, &value, sizeof(value));

   if (track_start != lseek(fd, 0, SEEK_CUR)) {
      msg(MSG_FATAL, "Header size wrong, update code\n");
      exit(1);
   }

   return fd;
}


// Read emulator file header.
//
// fn: File name to read header from
// emu_file_info: Information on emulator file format, set by this routine
// rewrite: Open file for reading and writing
// return: file descriptor of file opened
int emu_file_read_header(char *fn, EMU_FILE_INFO *emu_file_info,
      int rewrite)
{
   uint8_t header_id[sizeof(expected_header_id)];
   uint32_t value;
   int fd;
   int header_left;

   if (rewrite) {
      fd = open(fn, O_RDWR);
   } else {
      fd = open(fn, O_RDONLY);
   }
   if (fd < 0) {
      msg(MSG_FATAL, "Failed to open emulation file %s: %s\n", 
            fn, strerror(errno));
      exit(1);
   }

   emu_file_read(fd, header_id, sizeof(header_id));
   if (memcmp(header_id, expected_header_id, sizeof(header_id)) != 0) {
      msg(MSG_FATAL, "Emulation file doesn't have expected id value\n");
      exit(1);
   }
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->version = value;
   if ((value & 0xff000000) != (EMU_FILE_VERSION & 0xff000000) ||
         (value & 0xff0000) > (EMU_FILE_VERSION & 0xff0000)) {
      msg(MSG_FATAL, "Emulation file incorrect type or higher revision than supported %x %x",
            value, EMU_FILE_VERSION);
      exit(1);
   }
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->file_header_size_bytes = value;
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->track_data_size_bytes = value;
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->track_header_size_bytes = value;
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->num_cyl = value;
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->num_head = value;
   emu_file_read(fd, &value, sizeof(value));
   emu_file_info->sample_rate_hz = value;

   // These fields only in later versions
   emu_file_info->decode_cmdline = NULL;
   emu_file_info->note = NULL;
   if ((emu_file_info->version & 0xffff0000) >= 0x02020000) {
      emu_file_read(fd, &value, sizeof(value));
      emu_file_info->decode_cmdline = msg_malloc(value, 
         "emu_file_read decode_cmdline");
      emu_file_read(fd, emu_file_info->decode_cmdline, value);

      emu_file_read(fd, &value, sizeof(value));
      emu_file_info->note = msg_malloc(value, "emu_file_read note");
      emu_file_read(fd, emu_file_info->note, value);
   }
   emu_file_info->start_time_ns = 0;
   if ((emu_file_info->version & 0xffffff00) >= 0x02020200) {
      emu_file_read(fd, &value, sizeof(value));
      emu_file_info->start_time_ns = value;
   }
   // If more data in header ignore it. This allows 
   // minor revisions to add additional fields and old programs still 
   // can process
   header_left = emu_file_info->file_header_size_bytes - lseek(fd, 0, SEEK_CUR);
   if (header_left > 0) {
      char *ignore = msg_malloc(value, "emu_file_read ignore");
      emu_file_read(fd, ignore, header_left);
      free(ignore);
   }

   return fd;
}

// Write track header and bits
//
// fd: File descriptor to write to
// words: Words to write
// num_words: Number of words to write
// cyl: Cylinder of track. Pass -1 to write end of file marker
// head: Head/track number
// track_bytes: The size of each track data in bytes.
void emu_file_write_track_bits(int fd, uint32_t *words, int num_words, 
   int cyl, int head, uint32_t track_bytes) 
{
   uint32_t value;
   int i;

printf("Words %d\n",num_words);
   if (fd != -1) {
      value = TRACK_ID_VALUE;
      emu_file_write(fd, &value, sizeof(value));
      value = cyl;
      emu_file_write(fd, &value, sizeof(value));
      value = head;
      emu_file_write(fd, &value, sizeof(value));
      // Cylinder -1 is end of file marker so don't write data. Otherwise
      // pad with zero or truncate if longer than track_bytes.
      if (cyl != -1) {
         uint32_t fill;
         // Fill with valid MFM pattern
         // Pick word that won't put two ones in a row
         if (num_words == 0 || words[num_words-1] & 1) {
            fill = 0x55555555;
         } else {
            fill = 0xaaaaaaaa;
         }
         for (i = num_words; i < track_bytes/4; i++) {
            words[i] = fill;
         }
         emu_file_write(fd, words, track_bytes);
      }
      // Free pages written to disk. Improves write speed on fast USB flash
      // but worse with internal flash
      // posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
   }
}

// Overwrite a track including header with new data
//
// fd: File descriptor to write to
// emu_file_info: Information on emulator file format
// cyl: Cylinder of track. Pass -1 to write end of file marker
// head: Head/track number
// buf: Buffer to write
// buf_size: Buffer size in bytes
void emu_file_rewrite_track(int fd, EMU_FILE_INFO *emu_file_info,
      int cyl, int head, void *buf, int buf_size)
{
   off_t offset;
   int rc;
   int track_size = emu_file_info->track_data_size_bytes +
         emu_file_info->track_header_size_bytes;
   int cyl_size = track_size * emu_file_info->num_head;

   if (buf_size != track_size) {
      msg(MSG_FATAL, "Emulation track buffer size mismatch %d %d\n",
            buf_size, track_size);
      exit(1);
   }

   offset = (off_t) cyl * cyl_size + head * track_size +
         emu_file_info->file_header_size_bytes;

   if ((rc = pwrite(fd, buf, track_size, offset)) != track_size) {
      msg(MSG_FATAL, "Failed to write emulation track rc %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   }
}

// Read track header and bits
//
// fd: File descriptor to read from
// emu_file_info: Information on emulator file format
// words: Words to write
// num_words: Size of words buffer in words
// cyl: Cylinder of track. Pass -1 to write end of file marker
// head: Head/track number
// return: number of words of track data read if OK, -1 if end of file found
int emu_file_read_track_bits(int fd, EMU_FILE_INFO *emu_file_info,
      uint32_t *words, int num_words, int *cyl, int *head)
{
   uint32_t value;

   emu_file_read(fd, &value, sizeof(value));
   if (value != TRACK_ID_VALUE) {
      msg(MSG_FATAL, "Emulation file track id value mismatch %x %x\n", 
         value, TRACK_ID_VALUE);
      exit(1);
   }
   emu_file_read(fd, &value, sizeof(value));
   *cyl = value;
   emu_file_read(fd, &value, sizeof(value));
   *head = value;
   if (*cyl == -1 && *head == -1) {
      num_words = -1;
   } else {
      if (emu_file_info->track_data_size_bytes > num_words*4) {
         msg(MSG_FATAL, "Emulation file track larger than buffer %d %d\n",
               emu_file_info->track_data_size_bytes, num_words);
         exit(1);
      }
      emu_file_read(fd, words, emu_file_info->track_data_size_bytes);
      num_words = emu_file_info->track_data_size_bytes/4;
   }
   return num_words;
}

// Read cylinder of data including headers
//
// fd: File descriptor to read from
// emu_file_info: Information on emulator file format
// cyl: Cylinder to read
// buf: Buffer to read into
// buf_size: Buffer size in bytes
void emu_file_read_cyl(int fd, EMU_FILE_INFO *emu_file_info, int cyl,
      void *buf, int buf_size) {
   off_t offset;
   int rc;
   int cyl_size = emu_file_info->num_head *
         (emu_file_info->track_data_size_bytes + emu_file_info->track_header_size_bytes);

   if (buf_size < cyl_size) {
      msg(MSG_FATAL, "Emulation cylinder buffer too small\n");
      exit(1);
   }

   offset = (off_t) cyl * cyl_size + emu_file_info->file_header_size_bytes;

   if ((rc = pread(fd, buf, cyl_size, offset)) != cyl_size) {
      msg(MSG_FATAL, "Failed to read emulation cylinder rc %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   }
}

// Write cylinder of data including headers
//
// fd: File descriptor to write to
// emu_file_info: Information on emulator file format
// cyl: Cylinder to read
// buf: Buffer to write
// buf_size: Buffer size in bytes
void emu_file_write_cyl(int fd, EMU_FILE_INFO *emu_file_info, int cyl,
      void *buf, int buf_size) {
   off_t offset;
   int rc;
   int cyl_size = emu_file_info->num_head *
         (emu_file_info->track_data_size_bytes + emu_file_info->track_header_size_bytes);

   if (buf_size != cyl_size) {
      msg(MSG_FATAL, "Emulation cylinder buffer size mismatch\n");
      exit(1);
   }

   offset = (off_t) cyl * cyl_size + emu_file_info->file_header_size_bytes;

   if ((rc = pwrite(fd, buf, cyl_size, offset)) != cyl_size) {
      msg(MSG_FATAL, "Failed to write emulation cylinder rc %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   }
}

// Close emulator files
// fd: File descriptor to close
// write_eof: Write end of file mark before closing file
void emu_file_close(int fd, int write_eof)
{
   if (fd != -1) {
      if (write_eof) {
         emu_file_write_track_bits(fd, NULL, 0, -1, -1, 0);
      }
      fsync(fd);
      close(fd);
   }
}

////// Transition file routines

// Internal routines for reading and writing file.
// fd: File descriptor to read/write
// bytes: Data to read/write
// len: length of data in bytes
// poly: Polynomial data for calculating checksum. CRC value is
//    input and output in init_value field
static void tran_file_write(int fd, void *bytes, int len, CRC_INFO *poly) {
   int rc;
   if ((rc = write(fd, bytes, len)) != len) {
      msg(MSG_FATAL, "Failed to write word to transition file %d %s\n", rc,
            rc == -1 ? strerror(errno): "");
      exit(1);
   }
   poly->init_value = crc64(bytes, len, poly);
}

static void tran_file_read(int fd,void *bytes, int len, CRC_INFO *poly) {
   int rc;
   if ((rc = read(fd, bytes, len)) != len) {
      msg(MSG_FATAL, "Failed to read word from transition file %d %d %s\n", rc,
            len, rc == -1 ? strerror(errno) : "");
      exit(1);
   }
   poly->init_value = crc64(bytes, len, poly);
}

// Close transitions files
// fd: File descriptor to  close
// write_eof: Write end of file mark before closing file
void tran_file_close(int fd, int write_eof)
{
   if (fd != -1) {
      if (write_eof) {
         tran_file_write_track_deltas(fd, NULL, 0, -1, -1);
         fsync(fd);
      }
      close(fd);
   }

}
// Read emulator file header.
//
// fn: File name to read header from
// tran_file_info: Information from header
// return: file descriptor for file opened
int tran_file_read_header(char *fn, TRAN_FILE_INFO *tran_file_info)
{
   uint8_t header_id[sizeof(expected_header_id)];
   uint32_t value;
   CRC_INFO poly = trans_initial_poly;
   uint32_t crc;
   int fd;
   int header_left;

   fd = open(fn, O_RDONLY);
   if (fd < 0) {
      msg(MSG_FATAL, "Failed to open transition file %s: %s\n", 
            fn, strerror(errno));
      exit(1);
   }

   tran_file_read(fd, header_id, sizeof(header_id), &poly);
   if (memcmp(header_id, expected_header_id, sizeof(header_id)) != 0) {
      msg(MSG_FATAL, "Transition file doesn't have expected header id value\n");
      exit(1);
   }
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->version = value;
   if ((value & 0xff000000) != (TRAN_FILE_VERSION & 0xff000000) ||
         (value & 0xff0000) > (TRAN_FILE_VERSION & 0xff0000)) {
      msg(MSG_FATAL, "Transition file incorrect type or higher revision than supported %x %x",
            value, TRAN_FILE_VERSION);
      exit(1);
   }
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->file_header_size_bytes = value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->track_header_size_bytes = value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->num_cyl = value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->num_head = value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->sample_rate_hz = value;
   if (tran_file_info->sample_rate_hz != 200000000) {
      msg(MSG_FATAL, "Only transitions with sample rate of 200 MHz supported, got %d\n",
            tran_file_info->sample_rate_hz);
      exit(1);
   }
   tran_file_read(fd, &value, sizeof(value), &poly);
   tran_file_info->decode_cmdline = msg_malloc(value, 
      "tran_file_read decode_cmdline");
   tran_file_read(fd, tran_file_info->decode_cmdline, value, &poly);

   // This field only in later versions
   tran_file_info->note = NULL;
   if ((tran_file_info->version & 0xffff0000) >= 0x01020000) {
      tran_file_read(fd, &value, sizeof(value), &poly);
      tran_file_info->note = msg_malloc(value, "tran_file_read note");
      tran_file_read(fd, tran_file_info->note, value, &poly);
   }
   tran_file_info->start_time_ns = 0;
   if ((tran_file_info->version & 0xffffff00) >= 0x01020200) {
      tran_file_read(fd, &value, sizeof(value), &poly);
      tran_file_info->start_time_ns = value;
   }
   // If more data than 4 byte checksum in header ignore it. This allows 
   // minor revisions to add additional fields and old programs still 
   // can process
   header_left = tran_file_info->file_header_size_bytes - lseek(fd, 0, SEEK_CUR);
   if (header_left > 4) {
      char *ignore = msg_malloc(value, "tran_file_read ignore");
      tran_file_read(fd, ignore, header_left, &poly);
      free(ignore);
   }

   crc = poly.init_value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   // CRC is calculated big endian so final CRC won't be zero so we compare
   if (value != crc) {
      msg(MSG_FATAL, "Transition file header CRC error %x %x\n", crc, value);
      exit(1);
   }

   return fd;
}

// Create or truncate file and write transition file header.
//
// fn: File name to write to
// num_cyl: Number of cylinder file will contain
// num_head: Number of tracks per cylinder
// cmdline: Command line used for generation to store in file
// return: file descriptor for file opened
int tran_file_write_header(char *fn, int num_cyl, int num_head,
      char *cmdline, char *note, uint32_t start_time_ns) {
   uint32_t value;
   CRC_INFO poly = trans_initial_poly;
   int fd;
   char *lcl_note = "";
   int track_start;

   // If note null store empty string
   if (note != NULL) {
      lcl_note = note;
   }

   fd = open(fn, O_WRONLY | O_CREAT | O_TRUNC, 0664);
   if (fd < 0) {
      msg(MSG_FATAL, "Failed to open transition file %s: %s\n",
            fn, strerror(errno));
      exit(1);
   }

   tran_file_write(fd, expected_header_id, sizeof(expected_header_id), &poly);
   value = TRAN_FILE_VERSION;
   tran_file_write(fd, &value, sizeof(value), &poly);
   // Offset of first track header
   value = sizeof(expected_header_id) + 4*10 + strlen(cmdline)+1 + 
      strlen(lcl_note)+1;
   track_start = value;
   tran_file_write(fd, &value, sizeof(value), &poly);
   // Offset of track data from start of track
   value = 4*3;
   tran_file_write(fd, &value, sizeof(value), &poly);
   value = num_cyl;
   tran_file_write(fd, &value, sizeof(value), &poly);
   value = num_head;
   tran_file_write(fd, &value, sizeof(value), &poly);
   value = 200000000; // Sample rate in Hz
   tran_file_write(fd, &value, sizeof(value), &poly);

   if (cmdline == NULL) {
      msg(MSG_FATAL,"tran_file_write_header cmdline not specified\n");
      exit(1);
   }
   // write length including terminating null
   value = strlen(cmdline)+1;
   tran_file_write(fd, &value, sizeof(value), &poly);
   tran_file_write(fd, cmdline, value, &poly);

   value = strlen(lcl_note)+1;
   tran_file_write(fd, &value, sizeof(value), &poly);
   tran_file_write(fd, lcl_note, value, &poly);

   value = start_time_ns; // Time offset in nanoseconds
   tran_file_write(fd, &value, sizeof(value), &poly);
   // Checksum
   value = poly.init_value;
   tran_file_write(fd, &value, sizeof(value), &poly);

   if (track_start != lseek(fd, 0, SEEK_CUR)) {
      msg(MSG_FATAL, "Header size wrong, update code\n");
      exit(1);
   }

   return fd;
}

// Find the desired cylinder and head in the transition file
// fd: File descriptor to read from
// seek_cyl: Cylinder number to find
// seek_head: head number to find
// start_first_track: Byte location of first track in file
// return: 0 if track found else 1
int tran_file_seek_track(int fd, int seek_cyl, int seek_head, 
      int start_first_track) {
   int done = 0;
   uint32_t cyl, head;
   uint32_t num_bytes;
   CRC_INFO poly = trans_initial_poly;
   int rc = 0;

   lseek(fd, start_first_track, SEEK_SET);
   while (!done) {
      tran_file_read(fd, &cyl, sizeof(cyl), &poly);
      tran_file_read(fd, &head, sizeof(head), &poly);
      tran_file_read(fd, &num_bytes, sizeof(num_bytes), &poly);

      if (cyl == -1 && head == -1) {
         msg(MSG_DEBUG, "Unable to find cylinder %d head %d\n",
               seek_cyl, seek_head);
         rc = 1;
         done = 1;
      }
      if (cyl == seek_cyl && head == seek_head) {
         done = 1;
         lseek(fd, -4 * 3, SEEK_CUR);
      } else {
         if (lseek(fd, num_bytes + 4, SEEK_CUR) == -1) {
            msg(MSG_FATAL, "tran_file_seek_track seek failed\n");
            exit(1);
         }
      }
   }
   return rc;
}

// Read transition track and return as deltas
//
// fd: File descriptor to read from
// deltas: Deltas to return
// max_deltas: Size of deltas buffer in words
// cyl: Cylinder number of track read
// head: head number of track read
// return: Number of deltas read in words. -1 if end of file found.
int tran_file_read_track_deltas(int fd, uint16_t deltas[], int max_deltas, 
    int *cyl, int *head)
{
   uint8_t deltas_in[MAX_BYTE_DELTAS];
   uint32_t value;
   CRC_INFO poly = trans_initial_poly;
   int32_t num_bytes;
   int rc;
   int deltas_ndx = 0;
   int i;
   uint32_t crc;

   tran_file_read(fd, &value, sizeof(value), &poly);
   *cyl = value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   *head = value;
   tran_file_read(fd, &num_bytes, sizeof(num_bytes), &poly);

   if (*cyl == -1 && *head == -1) {
      rc = -1;
   } else {
      if (num_bytes > sizeof(deltas_in)) {
         msg(MSG_FATAL, "Transition file track larger than buffer %d %d\n",
               num_bytes, sizeof(deltas_in));
         exit(1);
      }
      tran_file_read(fd, deltas_in, num_bytes, &poly);
      i = 0;
      while (i < num_bytes) {
         if (deltas_in[i] == 255) {
            value = deltas_in[i+1] | ((uint32_t) deltas_in[i+2] << 8) | 
               ((uint32_t) deltas_in[i+3] << 16);
            i += 4;
         } else if (deltas_in[i] == 254) {
            value = deltas_in[i+1] | ((uint32_t) deltas_in[i+2] << 8);
            i += 3;
         } else {
            value = deltas_in[i++];
         }
         if (deltas_ndx >= max_deltas) {
            msg(MSG_FATAL, "Transition file deltas overflow %d %d\n", 
                deltas_ndx, max_deltas);
            exit(1);
         }
         deltas[deltas_ndx++] = value;
      }
      rc = deltas_ndx;
   }
   crc = poly.init_value;
   tran_file_read(fd, &value, sizeof(value), &poly);
   // CRC is calculated big endian so final CRC won't be zero so we compare
   if (value != crc) {
      msg(MSG_FATAL, "Transition file track CRC error %x %x\n", crc, value);
      exit(1);
   }
   return rc;
}

// Write transition track from deltas
//
// fd: File descriptor to write to
// deltas: Deltas to write
// num_deltas: Number of deltas to write in words
// cyl: Cylinder number of track
// head: head number of track
void tran_file_write_track_deltas(int fd, uint16_t *deltas, int num_deltas, int cyl, int head)
{
   uint8_t deltas_out[MAX_BYTE_DELTAS];
   int deltas_ndx = 0;
   uint32_t value;
   int i;
   CRC_INFO poly = trans_initial_poly;

   if (fd == -1) {
      return;
   }
   value = cyl;
   tran_file_write(fd, &value, sizeof(value), &poly);
   value = head;
   tran_file_write(fd, &value, sizeof(value), &poly);
   // Cylinder -1 is end of file marker
   if (cyl == -1) {
      value = 0; // Data length is zero
      tran_file_write(fd, &value, sizeof(value), &poly);
   } else {
      for (i = 0; i < num_deltas; i++) {
if (deltas[i] == 0) {
   printf("*** WRITING 0 delta at %d of %d\n", i, num_deltas);
}
         // If greater than a byte or matches one of the size indicator
         // values write it as multiple byte values with size indicator.
         if (deltas[i] >= 254) {
            if (deltas_ndx + 3 >= sizeof(deltas_out)) {
               msg(MSG_FATAL, "Too many transitions %d\n",deltas_ndx);
               exit(1);
            }
            // With 16 bit deltas this isn't possible. For future.
            if (deltas[i] > 0xffff) {
               deltas_out[deltas_ndx++] = 255;
               value = deltas[i];
               deltas_out[deltas_ndx++] = value;
               deltas_out[deltas_ndx++] = value >> 8;
               deltas_out[deltas_ndx++] = value >> 16;
            } else {
               deltas_out[deltas_ndx++] = 254;
               value = deltas[i];
               deltas_out[deltas_ndx++] = value;
               deltas_out[deltas_ndx++] = value >> 8;
            }
         } else {
            if (deltas_ndx >= sizeof(deltas_out)) {
               msg(MSG_FATAL, "Too many transitions %d\n",deltas_ndx);
               exit(1);
            }
            deltas_out[deltas_ndx++] = deltas[i];
         }
      }
      // Write length and data
      value = deltas_ndx;
      tran_file_write(fd, &value, sizeof(value), &poly);
      tran_file_write(fd, deltas_out, deltas_ndx, &poly);
   }
   value = poly.init_value;
   tran_file_write(fd, &value, sizeof(value), &poly);
   // Free pages written to disk. Improves write speed on fast USB flash
   // but worse with internal flash
   // posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
}
