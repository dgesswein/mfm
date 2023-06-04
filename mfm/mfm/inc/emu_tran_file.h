/*
 * emu_tran_file.h
 *
 * 11/09/14 DJG Added new function prototypes for emulator file
 * 	buffering and structure changes for buffering and other new
 *     command line options	
 *  Created on: Jan 25, 2014
 *      Author: djg
 */

#ifndef EMU_TRAN_FILE_H_
#define EMU_TRAN_FILE_H_

// Information on disk image file for emulator
typedef struct emu_file_info {
      // File version information
   uint32_t version;
      // The size of this data in the file header
   int file_header_size_bytes;
      // Header and data size of track. All tracks are the same size
   int track_header_size_bytes;
   int track_data_size_bytes;
      // File contains num_cyl*num_head tracks
   int num_cyl;
   int num_head;
      // Rate in Hz
   int sample_rate_hz;
      // Delay from index to first data in nanoseconds
   uint32_t start_time_ns;
      // The command line used to generate the file
   char *decode_cmdline;
      // And description of file
   char *note;
} EMU_FILE_INFO;

// Information on transition data file
typedef struct tran_file_info {
      // File version information
   uint32_t version;
   int file_header_size_bytes;
   int track_header_size_bytes;
      // File contains num_cyl*num_head tracks
   int num_cyl;
   int num_head;
      // Rate in Hz
   int sample_rate_hz;
      // Delay from index to first data in nanoseconds
   uint32_t start_time_ns;
      // The command line used to generate the file
   char *decode_cmdline;
      // And description of file
   char *note;
} TRAN_FILE_INFO;

int emu_file_write_header(char *fn, int num_cyl, int num_head, char *cmdline,
      char *note, uint32_t sample_rate, uint32_t start_time_ns, 
      uint32_t track_bytes);
int emu_file_read_header(char *fn, EMU_FILE_INFO *emu_file_info_out,
      int rewrite);
void emu_file_write_track_bits(int fd, uint32_t *words, int num_words, int cyl,
      int head, uint32_t track_bytes);
int emu_file_read_track_bits(int fd, EMU_FILE_INFO *emu_file_info,
      uint32_t *words, int num_bytes, int *cyl, int *head);
void emu_file_close(int fd, int write_eof);
int emu_file_seek_track(int fd, int seek_cyl, int seek_head, EMU_FILE_INFO *emu_file_info);
int emu_file_read_track_deltas(int fd, EMU_FILE_INFO *emu_file_info,
      uint16_t deltas[], int max_deltas, int *cyl, int *head);
void emu_file_read_cyl(int fd, EMU_FILE_INFO *emu_file_info, int cyl,
      void *buf, int buf_size);
void emu_file_write_cyl(int fd, EMU_FILE_INFO *emu_file_info, int cyl,
      void *buf, int buf_size);
void emu_file_rewrite_track(int fd, EMU_FILE_INFO *emu_file_info,
      int cyl, int head, void *buf, int buf_size);

int tran_file_write_header(char *fn, int num_cyl, int num_head, char *cmdline,
      char *note, uint32_t start_time_ns);
int tran_file_read_header(char *fn, TRAN_FILE_INFO *tran_file_info);
int tran_file_seek_track(int fd, int seek_cyl, int seek_head, TRAN_FILE_INFO *tran_file_info);
int tran_file_read_track_deltas(int fd,uint16_t deltas[], int max_deltas, int *cyl,
      int *head);
void tran_file_write_track_deltas(int fd,uint16_t *deltas, int num_words, int cyl, int head);
void tran_file_close(int fd, int write_eof);

float emu_rps(int sample_rate_hz);

#endif /* EMU_TRAN_FILE_H_ */
