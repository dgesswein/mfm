// Needs fixed prulib 
// http://e2e.ti.com/support/arm/sitara_arm/f/791/t/239735.aspx
// https://github.com/beagleboard/am335x_pru_package/issues/3

// This module is the routines needed to emulate a MFM disk. The emulated
// disk data is the raw clock and data MFM bit stream for each track.
// The ARM handles reading and writing a cylinder of track data to DDR 
// memory buffer. PRU 1 converts the MFM bits to Pulse Width Modulation (PWM)
// words PRU 0 uses to generate the MFM bitstream. PRU 1 also converts the
// MFM delta transitions times from PRU 0 to the MFM bitstream and stores
// it back in DDR memory. PRU 0 handles outputting the PWM words to the PWM
// generator, reading the delta words from the hardware and handling the
// various drive control lines. If the host seeks PRU 0 interrupts the ARM
// to let it fetch the next cylinder.
//

// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
//
// 05/19/17 DJG Dummped more memory on error.
// 02/20/16 DJG Reduced amount of delay when writing multiple tracks with
//    some track buffers full. Delays seemed longer than needed.
// 01/06/16 DJG Detect reversed J2 cable, don't allow different track lenght
//    when emulating two drives and other error messages fixes
// 11/22/15 DJG Added 15 MHz bitrate and fixed PRU0_DEFAULT_PULSE_WIDTH.
//    Set bit pattern to 1010... when initializing new image. Some
//    controllers don't like all zeros.
// 11/01/15 DJG Fixed incorrect printing of select and head for rev B board.
// 07/30/15 DJG Modified to support revision B board.
// 05/17/15 DJG Added DMA setup, printing/logging errors if PRU halts due
//    to internal checks.
// 01/04/15 DJG and Steven Hirsch 
//    Added support to set PRU clock to rate needed to
//    generate proper MFM clock and data bit rate. Moved PRU1 PWM table
//    generation and other values to this routine so it can be built for 
//    desired bit rate.
//    Added support for start_time_ns and sample_rate_hz to deal with
//    drives that don't start sectors immediately after index and use different
//    bit rates.
// 11/09/14 DJG Added write buffer and delays to handle slow flash memory
// 09/06/14 DJG Fixed deadlock between shutting down PRU 0 and seeking
//    to next cylinder
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
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fenv.h>
#include <inttypes.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>

// MFM program include files
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "msg.h"
#include "pru_setup.h"
#include "emu_tran_file.h"
#include "parse_cmdline.h"
#include "drive.h"
#include "board.h"

#include "cmd.h"

// TI 335 has 200Mhz. clock by default. We may change
uint32_t pru_clock_hz = 200000000UL;

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// Pick best timer clock
#ifdef CLOCK_MONOTONIC_RAW
#define CLOCK CLOCK_MONOTONIC_RAW
#else
#define CLOCK CLOCK_MONOTONIC
#endif

// File syste read/write threads
pthread_t read_thread;
pthread_t write_thread;

// This is a circular buffer holding tracks to write to the file. The flash
// can't keep up with random writes so this is needed to prevent timeouts when
// the writes block. Buffer is empty when get==put
struct {
   int drive;        // Drive, cylinder and head data is for, drive -1 signals
   int cyl;          // write thread to exit
   int head;
   int size;         // Size of track data
   char *buf;        // Track data
} *track_buffer;
// Circular buffer indexes
volatile int track_buffer_get;
volatile int track_buffer_put;

// Semaphore to let write thread know more data is available
sem_t write_sem;

// Log for run information
FILE *log_file;
// time() this program was started at
time_t start_time;

// Make a table which we index with the four MSB of the MFM bitstream
// to determine the next PWM word. The value of four guarantees we'll
// see at least a single 1 bit.

// The MFM bitstream is the MFM data and clock bits referred to as
// mfm encoded here:
// http://en.wikipedia.org/wiki/Modified_Frequency_Modulation 

// All the words generated must have a minimum duration of 2 bit cells,
// nominally 40 clocks, 200 ns.
// For example bit pattern 10010[1] is sent as a pulse with period 60
// counts and a pulse with period 40 for 10 MHz data rate.

// If we find a 11 we drop the second bit since we can't generate it.
// It's bad MFM data so shouldn't cause problems. 01 we flag as an
// error since we should never see it unless something has gone
// wrong. Without changing the PWM polarity we can't generate a zero
// followed by a 1. We pick the bits we process to ensure that valid
// MFM data won't generate the patterns we can't encode.  For example
// bit pattern 1000 we only process the first two (10) and leave the
// last two bits (00) for the next lookup.  For valid MFM code we
// could generate the waveform for all four bits instead since the
// next bit should be a 1. I don't in case it is a zero.
// 
// Bits 31-28 are how many bits to remove from data. This is the MFM
//   clock and data bits we will shift off, not anything to do with the
//   data bits encoded by the MFM encoding.
//   The bits we remove were choosen to make sure that the left over
//   bits aren't a pattern we can't encode.
// Bits 27-25 unused
// bit  24 is flag for illegal bit pattern found
// Bits 23-16 are duration of 1. 0 is no one. (PWM ACMP)
// Bits 15-0 is period to next bit time. (PWM APRD)
// Period values are one less than actual period generated.

// Drives construction of PRU1 bit lookup table
typedef struct {
   // Number of bits we can represent in a single PWM word (2 or 3)
   uint32_t bitcount;
   // Value of first bit (0 or 1)
   uint32_t leading_bit;
   // Error flag (see below)
   uint32_t error_flag;
} table_rec_t;

table_rec_t bit_table[] = {
   { 2, 0, 0 }, // 0000
   { 3, 0, 0 }, // 0001
   { 2, 0, 0 }, // 0010
   { 2, 0, 0 }, // 0011

   // If the bit shifting algorithm is working correctly, we should
   // never see a pattern starting with '01'.  If we do, then
   // something is wrong and and error is flagged.

   { 2, 0, 1 }, // 0100 treated as 0000
   { 3, 0, 1 }, // 0101 treated as 0001
   { 2, 0, 1 }, // 0110 treated as 0010
   { 2, 0, 1 }, // 0111 treated as 0011

   { 2, 1, 0 }, // 1000
   { 3, 1, 0 }, // 1001
   { 2, 1, 0 }, // 1010
   { 2, 1, 0 }, // 1011

   // The next four patterns are not valid MFM data and cannot be
   // generated.  We just drop the second 1.

   { 2, 1, 0 }, // 1100 treated as 1000
   { 3, 1, 0 }, // 1101 treated as 1001
   { 2, 1, 0 }, // 1110 treated as 1010
   { 2, 1, 0 }, // 1111 treated as 1011

   { 0, 0, 0 }
};

// TODO Find a new home for this
//
// This routine will either set the CPU speed to maximum or restore it
// back to the previous setting. The normal governor didn't boost the
// processor sufficiently to prevent getting behind in processing.
//
// restore: 0 to set speed to maximum, 1 to restore value from previous
//    call
int set_restore_max_cpu_speed(int restore) {
   FILE *file;
   static char governor[100];
   static int freq_changed = 0;
   char maxfreq[100];

   if (!restore) {
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "r");
      if (file == NULL) {
         return -1;
      }
      if (fscanf(file, "%100s", governor) < 1) {
         return -1;
      }
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "w");
      if (file == NULL) {
         return -1;
      }
      if (fprintf(file,"userspace") < 1) {
         return -1;
      }
      fclose(file);
      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq", "r");
      if (file == NULL) {
         return -1;
      }
      if (fscanf(file, "%100s", maxfreq) < 1) {
         return -1;
      }
      fclose(file);

      file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed", "w");
      if (file == NULL) {
         return -1;
      }
      if (fprintf(file,maxfreq) < 1) {
         return -1;
      }
      fclose(file);
      freq_changed = 1;
   } else {
      if (freq_changed) {
         file = fopen("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", "w");
         if (file == NULL) {
            return -1;
         }
         if (fprintf(file,governor) < 1) {
            return -1;
         }
         fclose(file);
      }
      freq_changed = 0;
   }
   return 0;
}


// This routine is for cleaning up when shutting down. Its installed as an
// atexit routine and called by SIGINT handler.
void shutdown(void)
{
   static int called = 0;
   time_t stop_time;
   struct timespec tv_start;
   double shutdown_start, shutdown_stop;

   if (called)
      return;
   called = 1;

   // Time how long it takes from shutdown request to file flushed so users
   // can see how its doing versus the capacitor run time. 
   clock_gettime(CLOCK, &tv_start);
   shutdown_start = tv_start.tv_sec + tv_start.tv_nsec / 1e9;

   set_restore_max_cpu_speed(1);

   // Command PRU code to termiate
   pru_write_word(MEM_PRU0_DATA,PRU0_EXIT, 1);

   // Wait for read and write threads to complete. File flushed by
   // write thread
   pthread_join(write_thread, NULL);
   pthread_join(read_thread, NULL);

   // Wait for PRU to signal its done
   pru_shutdown();

   clock_gettime(CLOCK, &tv_start);
   shutdown_stop = tv_start.tv_sec + tv_start.tv_nsec / 1e9;
   fprintf(log_file,"   Shutdown time %.1f seconds\n",
      shutdown_stop - shutdown_start);

   stop_time = time(NULL);
   fprintf(log_file,"Emulation stopped %.24s, duration %.3f days\n",ctime(&stop_time),
      (stop_time - start_time)/60.0/60.0/24.0);
}

// SIGINT/ control-c handler. Call our shutdown and exit
void shutdown_signal(void)
{
   shutdown();
   exit(1);
}

// Get number of entries used in track buffer
// size: Total number of entries in buffer
// return: Number of entries used. Last entry can't
//         be used to maximum return is size-1
int track_buffers_used(int size) {
   int used = track_buffer_put - track_buffer_get;

   if (used < 0) {
      used += size;
   }

   return used;
}

// This routine update the buffer parameters and put pointer and posts
// write thread semaphore to write data in buffer
//
// buffer_count: Number of buffers
// drive: drive number data is for
// cyl: cylinder to write to
// head: head/track to write to
// size: size of track data
void update_buffer(int buffer_count, int drive, int cyl, int head, int size) {
   int next_put;

   next_put = (track_buffer_put + 1) % buffer_count;
   if (next_put == track_buffer_get) {
      msg(MSG_INFO, "Track buffer full\n");
   }
   // If no free buffers wait until one is free
   while (next_put == track_buffer_get) {
      usleep(10000);
   }
   track_buffer[track_buffer_put].drive = drive;
   track_buffer[track_buffer_put].cyl = cyl;
   track_buffer[track_buffer_put].head = head;
   track_buffer[track_buffer_put].size = size;
   track_buffer_put = next_put;

   sem_post(&write_sem);
}

// Get the cylinder data and send to PRU. The cylinder is read from the file
// and if the track buffer has later data it is copied to the cylinder data.
// The data is then sent to the PRU
//
// drive_params: Drive parameters
// drive: Drive number
// cyl: Cylinder
// cyl_size: Cylinder size in bytes including headers
// track_size: Track size in bytes including header
// data: Cylinder data
void send_PRU_cyl_data(DRIVE_PARAMS *drive_params, int drive, int cyl,
      int cyl_size, int track_size, uint8_t *data) {
   int get_hold, index;

   // We need to have get index from before we read from file to ensure we
   // check all data that hasn't been written before the read. Put can't
   // change during this routine.
   get_hold = track_buffer_get;
   emu_file_read_cyl(drive_params->fd[drive],
         &drive_params->emu_file_info[drive], cyl, data,  cyl_size);
   // We need to go from get pointer to put pointer to ensure the last
   // data we move is the latest. We may move data we overwrite again.
   index = get_hold;
   while (index != track_buffer_put) {
      // If our drive and cylinder then move the track data to the correct
      // location in cylinder buffer
      if (track_buffer[index].drive == drive && track_buffer[index].cyl == cyl) {
         memcpy(&data[track_size * track_buffer[index].head],
               track_buffer[index].buf, track_buffer[index].size);
      }
      index = (index + 1) % drive_params->buffer_count;
   }

   pru_write_mem(MEM_DDR, data, cyl_size, drive*DDR_DRIVE_BUFFER_MAX_SIZE);
}

// This thread writes a cylinder of data to the shared DDR memory for the
// PRU's. It waits for an interrupt, writes the buffer back to disk if dirty,
// then reads the next cylinder of data. The data to write is sent through
// track buffers to another thread to actually write to prevent host computer
// timeouts when the write blocks. We delay between writes a linear
// increasing delay as the buffers get full. The flash memory can't keep up
// with random writes.
// TODO: Speed up memory copy.
//
// arg: drive_params pointer
static void *emu_proc(void *arg)
{
   DRIVE_PARAMS *drive_params = arg;
   // Buffer sizes for cylinder and track for each emulated drive
   int cyl_size[MAX_DRIVES];
   int track_size[MAX_DRIVES];
   // Current and new cylinder for each drive. Initial values force transfer
   // of cylinder 0 at startup
   int cyl[MAX_DRIVES] = {-1,-1};
   int new_cyl[MAX_DRIVES] = {0,0};
   // Buffer for the drive data
   uint8_t *data[MAX_DRIVES];
   // PRU register to decode for debug
   uint32_t b;
   // How long we took to respond to PRU request for next cylinder
   double seek_time, max_seek_time = 0;
   // How long we should delay
   float delay_time;
   // Bit flag for which tracks need to be written
   int dirty;
   int i;
   int done = 0;
   int trk;
   // Minimum free buffers
   int min_free_buf;
   // Buffer variables
   int num_used_buf, num_free_buf, init_used_buf;
   uint32_t total_seeks = 0, total_writes = 0;

   min_free_buf = drive_params->buffer_count;
   // Setup our variables
   for (i = 0; i < drive_params->num_drives; i++) {
      track_size[i] = drive_params->emu_file_info[i].track_data_size_bytes +
         drive_params->emu_file_info[i].track_header_size_bytes;
      cyl_size[i] = track_size[i] * drive_params->emu_file_info[i].num_head;
      data[i] = malloc(cyl_size[i]);
      if (data[i] == NULL) {
         msg(MSG_FATAL, "Cylinder buffer malloc failed, size %d\n",
            cyl_size[i]);
         exit(1);
      }
   }

   while (!done) {
      for (i = 0; i < drive_params->num_drives; i++) {
         if (new_cyl[i] != cyl[i]) {
            total_seeks++;
            cyl[i] = new_cyl[i];
            // Get new cylinder data and send to PRU
            send_PRU_cyl_data(drive_params, i, cyl[i], cyl_size[i],
                  track_size[i], data[i]);
         }
      }
      pru_exec_cmd(CMD_START, 0);
      seek_time = pru_read_word(MEM_PRU0_DATA,PRU0_SEEK_TIME) / 200e6 * 1e3;
      if (seek_time > max_seek_time) {
         max_seek_time = seek_time;
      }
      msg(MSG_INFO,"  Waiting, seek time %.1f ms max %.1f min free buffers %d\n",
         seek_time, max_seek_time, min_free_buf);
      // Wait for request from PRU
      prussdrv_pru_wait_event (PRU_EVTOUT_0);
      prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
      // If no data to write we don't delay
      delay_time = 0;
      num_used_buf = 0;
      num_free_buf = -1;
      init_used_buf = track_buffers_used(drive_params->buffer_count);

      for (i = 0; i < drive_params->num_drives; i++) {
         dirty = pru_read_word(MEM_PRU1_DATA, PRU1_DRIVE0_TRK_DIRTY +
            i*PRU_WORD_SIZE_BYTES);
         new_cyl[i] = pru_read_word(MEM_PRU0_DATA,PRU0_DRIVE0_CUR_CYL +
            i*PRU_WORD_SIZE_BYTES);
         if (new_cyl[i] == -1) {
            done = 1;
         }
         b = pru_read_word(MEM_PRU0_DATA,PRU0_CUR_SELECT_HEAD);
         if (cyl[i] != new_cyl[i]) {
            int sel, head;
            if (board_get_revision() == 0) {
               sel = (((b >> 22) & 0x3) | ((b >> 24) & 0xc)) ^ 0xf;
               head = ((b >> 2) & 0xf) ^ 0xf;
            } else {
               sel = ((b >> 22) & 0x3)  ^ 0x3;
               head = ((b >> 8) & 0xf) ^ 0xf;
            }

            msg(MSG_INFO,"  Drive %d Cyl %d->%d select %d, head %d dirty %x\n",
               i, cyl[i], new_cyl[i], sel, head, dirty);
         }
         if (dirty) {
            total_writes++;
            pru_write_word(MEM_PRU1_DATA,PRU1_DRIVE0_TRK_DIRTY +
               i*PRU_WORD_SIZE_BYTES, 0);
               // Do delay based on used buffers on enter
            for (trk = 0; trk < drive_params->emu_file_info[i].num_head; trk++) {
               // For each dirty track read the data from PRU, put in
               // buffer and tell writer it has more data.
               if (dirty & (1 << trk)) {
                  pru_read_mem(MEM_DDR, track_buffer[track_buffer_put].buf,
                     track_size[i], track_size[i]*trk +
                     i*DDR_DRIVE_BUFFER_MAX_SIZE);

                  num_used_buf = track_buffers_used(drive_params->buffer_count);
                  // -1 is because last buffer can't be used
                  num_free_buf = drive_params->buffer_count - num_used_buf - 1;

                  if (num_free_buf < min_free_buf) {
                     min_free_buf = num_free_buf;
                  }
                  update_buffer(drive_params->buffer_count, i, cyl[i], trk, track_size[i]);

                  // Do linear delay based on number of buffers full
                  // We will do one delay after all data transfered
                  delay_time = num_used_buf * drive_params->buffer_time;
               }
            }
         }
      }
      if (num_free_buf != -1) {
         msg(MSG_INFO, "Free buffers %d,%d delay %.3f\n", num_free_buf, 
            init_used_buf, delay_time);
      }
      // If delay small or only couple buffers full before last transfer then
      // don't delay.
      if (delay_time > 2e-3 && init_used_buf > 2) {
         // If sum of delays greater than maximum only delay maximum
         // Assume maximum is when controller timeouts will happen
         if (delay_time > drive_params->buffer_max_time) {
            delay_time = drive_params->buffer_max_time;
         }

         msg(MSG_INFO, "Actual delay %.3f\n", delay_time);
         usleep((int) (delay_time * 1e6));
      }
   }
   // Tell PRU we saw the exiting flag
   pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_CUR_CYL, 0);
   update_buffer(drive_params->buffer_count, -1, 0, 0, 0);

   fprintf(log_file,"   Max seek time %.1f ms min free buffers %d, %u seeks %u writes\n",
      max_seek_time, min_free_buf, total_seeks, total_writes);

   return NULL;
}

// This thread writes data from the buffers to disk
//
// arg: drive_params pointer
static void *emu_proc_write(void *arg)
{
   DRIVE_PARAMS *drive_params = arg;
   int i;

   while (1) {
      // wait until data available
      sem_wait(&write_sem);

      // drive -1 signals we are done
      if (track_buffer[track_buffer_get].drive == -1) {
         break;
      }

      emu_file_rewrite_track(
         drive_params->fd[track_buffer[track_buffer_get].drive],
         &drive_params->emu_file_info[track_buffer[track_buffer_get].drive],
         track_buffer[track_buffer_get].cyl,
         track_buffer[track_buffer_get].head,
         track_buffer[track_buffer_get].buf,
         track_buffer[track_buffer_get].size);
      track_buffer_get = (track_buffer_get + 1) % drive_params->buffer_count;
   }

   // Done with files
   for (i = 0; i < MAX_DRIVES; i++) {
      emu_file_close(drive_params->fd[i], 0);
   }

   return NULL;
}

// Main routine. 
int main(int argc, char *argv[])
{
   // Size of shared memory in bytes
   int ddr_mem_size;
   int i;
   DRIVE_PARAMS drive_params;
   int cyl,head;
   int select_map[2][5] = 
      {
         {0, GPIO_SELECT1, GPIO_SELECT2, GPIO_SELECT3, GPIO_SELECT4},
         {0, GPIO_SELECT1, GPIO_SELECT2, 0, 0}
      };
   char *pru_files[2][2] =
      { 
         {"prucode0_reva.bin","prucode1_reva.bin"},
         {"prucode0_revb.bin","prucode1_revb.bin"}
      };
   int max_buffer = 0;
   struct sched_param params;
   uint32_t sample_rate_hz = 0;
   uint32_t index_start, index_end, last_index_start = 0xffffffff;
   uint32_t last_track_time = 0xffffffff;
   uint32_t bit_period = 0;
   int track_size;
   uint32_t *data;

   board_initialize();

   // Find out what we should do
   parse_cmdline(argc, argv, &drive_params);

   if (drive_params.initialize) {
      if (drive_params.num_drives != 1) {
         msg(MSG_FATAL, "Only one filename may be specified when initializing\n");
         exit(1);
      }
      drive_params.cmdline = parse_print_cmdline(&drive_params, 0);
      // Assume 3600 RPM, 60 RPS. Make round number of words
      track_size = ceil(1/60.0 * drive_params.sample_rate_hz / 8 / 4)*4;
      data = calloc(1,track_size);
      for (i = 0; i < ARRAYSIZE(data); i++) {
         data[i] = 0xaaaaaaaa;
      }
      drive_params.fd[0] = emu_file_write_header(drive_params.filename[0],
         drive_params.num_cyl, drive_params.num_head, drive_params.cmdline,
         drive_params.note, drive_params.sample_rate_hz, 
         drive_params.start_time_ns, track_size);
      for (cyl = 0; cyl < drive_params.num_cyl; cyl++) {
         for (head = 0; head < drive_params.num_head; head++) {
            emu_file_write_track_bits(drive_params.fd[0], data, track_size / 4, 
               cyl, head, track_size);
         }
      }
      free(data);
      emu_file_close(drive_params.fd[0], 1);
      sample_rate_hz = drive_params.sample_rate_hz;
   }
   // Initialize PRU
   ddr_mem_size = pru_setup(2);
   if (ddr_mem_size == -1) {
      exit(1);
   }

   for (i = 0; i < MAX_DRIVES; i++) {
      // 0 for drive always selected, 1-4 for select 1-4
      if (i < drive_params.num_drives) {
         if (drive_params.drive[i] < ARRAYSIZE(select_map[0])) {
            pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_SELECT +
               i*PRU_WORD_SIZE_BYTES,
               select_map[board_get_revision()][drive_params.drive[i]]);
         } else {
            msg(MSG_FATAL, "Illegal drive select %d\n", drive_params.drive);
            exit(1);
         }
      } else {
         pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_SELECT + i*
            (PRU0_DRIVE1_SELECT - PRU0_DRIVE0_SELECT), 0);
      }
   }
   log_file = fopen("logfile.txt","a");
   msg_set_logfile(log_file, MSG_FATAL);
   start_time = time(NULL);
   fprintf(log_file,"Emulation started %.24s\n",ctime(&start_time));
   fflush(log_file);

   // Heads must be zero for unused drives
   for (i = 0; i < MAX_DRIVES; i++) {
      pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_NUM_HEAD + i*PRU_WORD_SIZE_BYTES,
         0);
   }

   // Send parameters for drives emulating to PRU
   for (i = 0; i < drive_params.num_drives; i++) {
      EMU_FILE_INFO *curr_info = &drive_params.emu_file_info[i];

      drive_params.fd[i] = emu_file_read_header(drive_params.filename[i],
         curr_info, 1);
      if (curr_info->note != NULL &&
         strlen(curr_info->note) != 0) {
         msg(MSG_INFO, "Drive %d note: %s\n", i, 
            curr_info->note);
      }

      msg(MSG_INFO,"Drive %d num cyl %d num head %d track len %d\n", i,
         curr_info->num_cyl, 
         curr_info->num_head,
         curr_info->track_data_size_bytes);

      if (sample_rate_hz == 0) {
         sample_rate_hz = curr_info->sample_rate_hz;
      }
      else if (curr_info->sample_rate_hz != sample_rate_hz) {
         msg(MSG_FATAL, "Emulation files must all agree on sample rate\n");
         exit(1);
      }
      pru_clock_hz = pru_set_clock(sample_rate_hz, PRU_SET_CLOCK_NO_HALT);
      printf("PRU clock %d\n",pru_clock_hz);

      // Setup PRU0 rate-specific values
      bit_period = lround((double) pru_clock_hz / sample_rate_hz);

      pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_NUM_HEAD + i*PRU_WORD_SIZE_BYTES,
         curr_info->num_head);
      pru_write_word(MEM_PRU0_DATA,PRU0_DRIVE0_NUM_CYL + i*PRU_WORD_SIZE_BYTES,
         curr_info->num_cyl);
      pru_write_word(MEM_PRU1_DATA,PRU1_DRIVE0_TRACK_HEADER_BYTES +
         i*PRU_WORD_SIZE_BYTES, curr_info->track_header_size_bytes);
      pru_write_word(MEM_PRU1_DATA,PRU1_DRIVE0_TRACK_DATA_BYTES +
         i*PRU_WORD_SIZE_BYTES, curr_info->track_data_size_bytes);
      if (curr_info->track_data_size_bytes > max_buffer) {
         max_buffer = curr_info->track_data_size_bytes;
      }

      int track_time = lround(curr_info->track_data_size_bytes * 8.0 * 
         bit_period);
      if (curr_info->start_time_ns != 0) {
         index_start = lround(track_time - curr_info->start_time_ns / 1e9 * 
            pru_clock_hz);
         index_end = index_start + lround(200e-6 * pru_clock_hz);
         if (index_end > track_time) {
            index_end = index_end - track_time;
         }
      } else {
         index_start = 0;
         index_end = lround(200e-6 * pru_clock_hz);
      }
      if (last_index_start != 0xffffffff && index_start != last_index_start) {
         msg(MSG_FATAL, "Emulation files must all agree on start time\n");
         exit(1);
      }
      last_index_start = index_start;
      if (last_track_time != 0xffffffff && track_time != last_track_time) {
         msg(MSG_FATAL, "Emulation files must all have same track time/track length\n");
         exit(1);
      }
      last_track_time = track_time;
      pru_write_word(MEM_PRU0_DATA, PRU0_START_INDEX_TIME, index_start);
      pru_write_word(MEM_PRU0_DATA, PRU0_END_INDEX_TIME, index_end);
      pru_write_word(MEM_PRU0_DATA, PRU0_ROTATION_TIME, track_time);
   }

   pru_write_word(MEM_PRU0_DATA, PRU0_BIT_PRU_CLOCKS, bit_period);
   // Actual pulse width is 1 more than specified
   pru_write_word(MEM_PRU0_DATA, PRU0_DEFAULT_PULSE_WIDTH, bit_period*2-1);

   // Setup PRU1 rate-specific values
   unsigned idx = 0;
   int ram_offset = PRU1_BIT_TABLE;
   table_rec_t rec = bit_table[idx++];
   
   while (rec.bitcount > 0) {
      uint32_t bits = (rec.bitcount<<28) | (rec.error_flag<<24) | ((rec.leading_bit*bit_period)<<16) | ((rec.bitcount*bit_period)-1);
      pru_write_word(MEM_PRU1_DATA, ram_offset, bits);
      ram_offset += PRU_WORD_SIZE_BYTES;
      rec = bit_table[idx++];
   }
   uint32_t inv_bit_period_s32 = lround((double) (1ll << 32) / bit_period);
   pru_write_word(MEM_PRU1_DATA, PRU1_INV_BIT_PERIOD_S32, inv_bit_period_s32);

   uint32_t zero_threshold = lround(bit_period * 1.5);
   pru_write_word(MEM_PRU1_DATA, PRU1_ZERO_BIT_THRESHOLD, zero_threshold);
   pru_write_word(MEM_PRU1_DATA, PRU1_BIT_PRU_CLOCKS, bit_period);

   track_buffer = malloc(drive_params.buffer_count * sizeof(*track_buffer));
   if (track_buffer == NULL) {
      msg(MSG_FATAL, "Track buffer malloc failed\n");
      exit(1);
   }
   memset(track_buffer, 0, drive_params.buffer_count * sizeof(*track_buffer));
   for (i = 0; i < drive_params.buffer_count; i++) {
      track_buffer[i].buf = malloc(max_buffer);
      if (track_buffer[i].buf == NULL) {
         msg(MSG_FATAL, "Track buffer buf malloc failed\n");
         exit(1);
      }
   }

   // TODO: Get driver working to find unused channel
   pru_write_word(MEM_PRU1_DATA,PRU1_DMA_CHANNEL, 7);

   track_buffer_put = 0;
   track_buffer_get = 0;

   // And start our code
   for (i = 0; i < ARRAYSIZE(pru_files[0]); i++) {
      char *fn = pru_files[board_get_revision()][i];
      if (prussdrv_exec_program(i, fn) != 0) {
         msg(MSG_FATAL, "Unable to execute %s\n", fn);
         exit(1);
      }
   }

   // Cleanup when we exit
   signal(SIGINT,(__sighandler_t) shutdown_signal);
   atexit(shutdown);

   sem_init(&write_sem, 0, 0);

   if (pthread_create(&read_thread, NULL, &emu_proc, &drive_params)
      != 0) {
      msg(MSG_FATAL, "Unable to create read thread\n");
      exit(1);
   }

   if (pthread_create(&write_thread, NULL, &emu_proc_write, &drive_params)
      != 0) {
      msg(MSG_FATAL, "Unable to create read thread\n");
      exit(1);
   }

   params.sched_priority = sched_get_priority_max(SCHED_FIFO);
   if (pthread_setschedparam(read_thread, SCHED_FIFO, &params) != 0) {
      msg(MSG_ERR, "Unable to set thread priority\n");
   }
#if 1
   // TODO: Should this drop speed when idle?
   // This is needed to keep up with the data. Without it we will take
   // more than 2 revolutions per track due to the default governor not
   // increasing the CPU speed enough. We switch frequently between busy and
   // sleeping.
   if (set_restore_max_cpu_speed(0)) {
      msg(MSG_ERR, "Unable to set CPU to maximum speed\n");
   }
#endif
   sleep(1);
   // This is debugging stuff to print state of data from PRU
   struct {
      int loc;
      MEM_TYPE mem_type;
      char *txt;
      uint32_t last;
   } locs[] = {
      {PRU1_BAD_PATTERN_COUNT, MEM_PRU1_DATA, "bad pattern count %d\n", 123},
      {PRU0_RQUEUE_UNDERRUN, MEM_PRU0_DATA, "Read queue underrun %d\n", 123},
      {PRU0_WQUEUE_OVERRUN, MEM_PRU0_DATA, "Write queue overrun %d\n", 123},
      {PRU0_ECAP_OVERRUN, MEM_PRU0_DATA, "Ecapture overrun %d\n", 123},
      {PRU0_HEAD_SELECT_GLITCH_COUNT, MEM_PRU0_DATA, "glitch count %d\n", 123},
      {PRU0_HEAD_SELECT_GLITCH_VALUE, MEM_PRU0_DATA, "glitch value %x\n", 123},
      {PRU_TEST0, MEM_PRU0_DATA, "0:test 0 %x\n", 123},
      {PRU_TEST1, MEM_PRU0_DATA, "0:test 1 %x\n", 123},
      {PRU_TEST2, MEM_PRU0_DATA, "0:test 2 %x\n", 123},
      {PRU_TEST3, MEM_PRU0_DATA, "0:test 3 %x\n", 123},
      {PRU_TEST4, MEM_PRU0_DATA, "0:test 4 %x\n", 123},
      {PRU_TEST0, MEM_PRU1_DATA, "1:test 0 %x\n", 123},
      {PRU_TEST1, MEM_PRU1_DATA, "1:test 1 %x\n", 123},
      {PRU_TEST2, MEM_PRU1_DATA, "1:test 2 %x\n", 123},
      {PRU_TEST3, MEM_PRU1_DATA, "1:test 3 %x\n", 123},
      {PRU_TEST4, MEM_PRU1_DATA, "1:test 4 %x\n", 123}
   };
   while (1) {
      uint32_t a;
#if 1
      for (i = 0; i < ARRAYSIZE(locs); i++) {
         a = pru_read_word(locs[i].mem_type,locs[i].loc);
         if (a != locs[i].last) {
            printf(locs[i].txt,a);
            locs[i].last = a;
         }
      } 
#endif
#if 1
      static uint32_t b, sel, head, last_b = 0xffffffff;
      static int first_time = 1;
      b = pru_read_word(MEM_PRU0_DATA,PRU0_CUR_SELECT_HEAD);
      if (board_get_revision() == 0) {
         sel = (((b >> 22) & 0x3) | ((b >> 24) & 0xc)) ^ 0xf;
         head = ((b >> 2) & 0xf) ^ 0xf;
      } else {
         sel = ((b >> 22) & 0x3)  ^ 0x3;
         head = ((b >> 8) & 0xf) ^ 0xf;
      }
      if (b != last_b) {
         msg(MSG_INFO, "select %d head %d\n", sel, head);
         last_b = b;
         if (first_time) {
            first_time = 0;
            if (b & (1 << CUR_SELECT_HEAD_WRITE_ERR)) {
               msg(MSG_ERR, "**Write and step are active, is J2 cable reversed?**\n");
            }
         }
      } 
#endif
      if (pru_get_halt(0) || pru_get_halt(1)) {
         for (i = 0; i < 2; i++) {
            MEM_TYPE mem_type;
            if (i == 0) {
               mem_type = MEM_PRU0_DATA;
            } else {
               mem_type = MEM_PRU1_DATA;
            } 
            printf("PRU %d pc %04x\n",i, pru_get_pc(i));
            pru_print_registers(i);
            pru_print_memory(mem_type, 0, 256);
            pru_print_memory(mem_type, 0x400, 128);
         }
         pru_print_memory(MEM_PRU_SHARED, 0x0, 128);
         // With halted PRU we can't cleanly exit
         fclose(log_file);
         _exit(1);
      }
      //printf("PC %x %x\n", pru_get_pc(0), pru_get_pc(1));
      usleep(1000000);
   }

   // Not reached, exit called to terminate
   return (0);
}

