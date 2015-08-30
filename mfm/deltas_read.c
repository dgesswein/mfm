// This module is for reading deltas (MFM bit transition delta times) from the 
// PRU and optionally writing them to a file. The deltas are also available 
// through the pointer returned by deltas setup. The deltas are 16 bit unsigned
// values with each word the time difference between the current rising edge and
// the previous rising edge of the MFM data signal in 200 MHz clock
// counts.
//
// The PRU puts the deltas in the shared DDR memory. These
// routines copy them to normal DDR memory. The shared DDR memory is uncached
// so slow to access. Copying is a little faster.
// A thread is used to read the data from the PRU and update the available delta
// count
//
// Call deltas_setup once to setup the process.
// Call deltas_start_thread to start the reader thread and optionally start
//   writing delta data to filedeltas_get_count
// Call deltas_start_read after each read command is sent to the PRU.
// Call deltas_get_count to get the number of deltas available
// Call deltas_wait_read_finished to wait until all deltas are received
// Call deltas_stop_thread when done with the delta thread
//
// 06/27/2015 DJG Made CMD_STATUS_READ_OVERRUN a warning instead of fatal error
// 05/16/2015 DJG Changes for deltas_read_file.c
// 01/04/2015 DJG Changes for start_time_ns
// 11/09/2014 DJG Changes for new note option
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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "pru_setup.h"
#include "deltas_read.h"
#include "drive.h"

static void *delta_proc(void *arg);

// Semaphore to control when the delta reader starts looking for more deltas
static sem_t deltas_sem;
// Pointer to delta memory
static uint16_t *deltas;
// The current cylinder and head processing. Used when writing transitions to file
static int deltas_cyl, deltas_head;
// The thread to process deltas
pthread_t delta_thread;
// State of thread
static volatile enum {
   THREAD_NOT_CREATED,
   THREAD_SHUTDOWN,
   THREAD_RUNNING
} thread_state = THREAD_NOT_CREATED;

// Number of deltas in buffer
static int num_deltas;
// Deltas are being received from read thread
static int streaming;

// Allocate the memory to hold the deltas and initialize the semaphore.
// Call once before calling other routines.
//
// ddr_mem_size: Size of ddr shared memory in bytes
void *deltas_setup(int ddr_mem_size) {
   // Create storage for delta data read from PRU
   deltas = msg_malloc(ddr_mem_size, "deltas_setup deltas");

   if (sem_init(&deltas_sem, 0, 0) == -1) {
      msg(MSG_FATAL, "Sem creation failed\n");
      exit(1);
   }
   return deltas;
}

// Start the delta thread.
//
// drive_params: NULL if no transition data should be written
void deltas_start_thread(DRIVE_PARAMS *drive_params)
{
   if (pthread_create(&delta_thread, NULL, &delta_proc, drive_params) == 0) {
      thread_state = THREAD_RUNNING;
   } else {
      msg(MSG_FATAL, "Unable to create delta thread\n");
      exit(1);
   }
}

// Routine to stop the delta thread
void deltas_stop_thread()
{
   if (thread_state == THREAD_RUNNING) {
      thread_state = THREAD_SHUTDOWN;
      // Let it run to see the shutdown
      sem_post(&deltas_sem);
      // And wait for it to exit
      pthread_join(delta_thread, NULL);
      thread_state = THREAD_NOT_CREATED;
   }
}

// Call to start reading the deltas after starting the PRU CMD_READ_TRACK.
// It may also be called again if it is desired to reprocess the same deltas.
//
// cyl, head: Track being read
void deltas_start_read(int cyl, int head) {
   deltas_cyl = cyl;
   deltas_head = head;
   // Init state before releasing thread
   deltas_update_count(0, 1);
   sem_post(&deltas_sem);
}

// Write deltas to file. Don't call if transitions_file is -1.
//
// deltas: delta data to write
// num_deltas: number of deltas to write in words
static void write_deltas(int fd, uint16_t deltas[], int num_deltas) {

   tran_file_write_track_deltas(fd, deltas, num_deltas, deltas_cyl, deltas_head);
}

// This is the thread for processing deltas. 
// We read a tracks worth of delta from PRU into global deltas then
// wait for semaphore to repeat. If global thread_state is
// THREAD_SHUTDOWN then we exit.
//
// arg: pointer to drive_params if output files should be created or NULL
//    to not create a file
static void *delta_proc(void *arg)
{
   // Total number of deltas we have gotten on this track
   int track_deltas = 0;
   // Bytes of deltas to read
   int num_bytes;
   // set to 1 when PRU has indicated read is done
   int pru_finished_read = 0;
   // When 1 wait for read to be started
   int wait_read = 1;
   DRIVE_PARAMS *drive_params = (DRIVE_PARAMS *) arg;

   // Open file if requested
   if (drive_params != NULL && drive_params->transitions_filename != NULL) {
      drive_params->tran_fd = tran_file_write_header(
            drive_params->transitions_filename,
            drive_params->num_cyl, drive_params->num_head,
            drive_params->cmdline, drive_params->note, 
            drive_params->start_time_ns);
   }

   // And loop reading delta transitions
   while (1) {
      // Wait till read started
      if (wait_read) {
         track_deltas = 0;
         sem_wait(&deltas_sem);
         wait_read = 0;
         if (thread_state == THREAD_SHUTDOWN)
            break;
      }
      num_bytes = pru_read_word(MEM_PRU0_DATA, PRU0_WRITE_PTR) -
         track_deltas * sizeof(deltas[0]); 
      pru_read_mem(MEM_DDR, &deltas[track_deltas],
            num_bytes, track_deltas * sizeof(deltas[0]));
      track_deltas += num_bytes / sizeof(deltas[0]);
      deltas_update_count(track_deltas, 1);
      // If we didn't get very many deltas sleep to reduce overhead
      if (num_bytes < 300) {
         // CMD_STATUS_READ_STARTED indicates read is in progress,
         // CMD_STATUS_OK indicates PRU has finished read
         if (pru_get_cmd_status() != CMD_STATUS_READ_STARTED) {
            if (pru_get_cmd_status() != CMD_STATUS_OK) {
               if (pru_get_cmd_status() == CMD_STATUS_DELTA_OVERFLOW) {
                  msg(MSG_ERR_SERIOUS, "Delta transition time overflow, raw transitions will not accuractly represent cyl %d head %d\n", 
                     deltas_cyl, deltas_head);
               } else if (pru_get_cmd_status() == CMD_STATUS_READ_OVERRUN) {
                  msg(MSG_ERR_SERIOUS, "Delta transitions lost, raw transitions will not accuractly represent cyl %d head %d\n",
                     deltas_cyl, deltas_head);
               } else {
                  msg(MSG_FATAL, "Fault reading deltas cmd %x status %x cyl %d head %d\n",
                     pru_get_cmd_status(), drive_get_drive_status(),
                     deltas_cyl, deltas_head);
                  drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
                  msg(MSG_FATAL, "CMD_DATA %x delta count %x\n", pru_get_cmd_data(),
                     pru_read_word(MEM_PRU0_DATA, PRU0_WRITE_PTR));
                  exit(1);
               }
            }
            // PRU says its done. Use pru_finished_read variable to loop one more time
            // to ensure we have read all the deltas
            if (pru_finished_read) {
               // This must be before update_mfm_deltas to prevent
               // next read starting while we are still writing deltas
               if (drive_params != NULL && 
                     drive_params->transitions_filename != NULL) {
                  write_deltas(drive_params->tran_fd, deltas, track_deltas);
               }

               // All are transferred, tell MFM decoder all deltas available
               deltas_update_count(track_deltas, 0);
               pru_finished_read = 0;
               // Indicate we should wait for next read
               wait_read = 1;
            } else {
               // check one more time to avoid race conditions
               pru_finished_read = 1;
            }
         }
         usleep(500);
      }
   }
   if (drive_params != NULL && drive_params->transitions_filename != NULL) {
      tran_file_close(drive_params->tran_fd, 1);
   }
   return NULL;
}

// Update our count of deltas. Streaming indicates we are reading data from
// PRU as it comes in. Streaming is set to zero after all data is read from
// the PRU.
//
// num_deltas_in: Total number of deltas read so far
// streaming_in: 1 if data is being read from PRU. 0 when all data read.
void deltas_update_count(int num_deltas_in, int streaming_in)
{
   num_deltas = num_deltas_in;
   streaming = streaming_in;
}

// Get the delta count. While streaming we return the number of deltas. When
// we are done streaming and cur_deltas is >= num_deltas we return -1 to
// indicate to caller it has processed all of the deltas.
//
// cur_delta: Number of deltas processed by caller
// return: Number of deltas read or -1 if no more deltas
int deltas_get_count(int deltas_processed)
{
   if (streaming) {
      return num_deltas;
   } else {
      if (deltas_processed >= num_deltas) {
         return -1;
      } else {
         return num_deltas;
      }
   }
}

// Wait until all deltas received. This is used when writing the deltas to a
// file but not decoding. The wait ensures the deltas are written before
// staring the next read.
//
// return: number of deltas read
int deltas_wait_read_finished()
{
   int num_deltas, ret_deltas = 0;

   num_deltas = deltas_get_count(0);
   while ((num_deltas = deltas_get_count(num_deltas)) >= 0) {
      ret_deltas = num_deltas;
      usleep(500);
   }
   return ret_deltas;
}
