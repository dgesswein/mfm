// Message handling routines
//
// Call msg to print an error message
// Call msg_set_err_mask to set which errors should be printed
// Call msg_get_err_mask to get current error mask
// Call msg_malloc to malloc with error message if fail
// Call msg_set_logfile to log fatal errors to the log file
//
// 05/17/15 DJG Added ability to log errors to a file
// 11/09/14 DJG added new msg_malloc so I don't have to keep checking return
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
#include <stdarg.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>

#ifdef CLOCK_MONOTONIC_RAW
#define CLOCK CLOCK_MONOTONIC_RAW
#else
#define CLOCK CLOCK_MONOTONIC
#endif

#include "msg.h"

// Current error mask. Not thread safe for changes
static uint64_t err_mask = 0xffffffff;

// If not null log fatal errors to file
static FILE *logfile = NULL;
uint32_t logfile_err_mask;

// Print an error message.
//
// level: Error level used to determine if message should be printed
// format: Format string for message
//
void msg(uint32_t level, char *format, ...) {
   static int last_progress = 0;
   va_list va;


   va_start(va, format);
   if (err_mask & level) {
      // Overwrite progress message with spaces in case new message is shorter
      if (last_progress && !(level & MSG_PROGRESS)) {
         printf("%79s\r","");
      }
      vprintf(format, va);
      if (level & MSG_PROGRESS) {
         fflush(stdout);
      }
      last_progress = level & MSG_PROGRESS;
   }
   if (logfile != NULL && (level & logfile_err_mask)) {
      vfprintf(logfile, format, va);
   }
   va_end(va);
}

// Set the error mask and get previous value
//
// mask: New error mask
// return: Previous error mask
uint32_t msg_set_err_mask(uint32_t mask) {
   uint32_t last_mask;

   last_mask = err_mask;
   err_mask = mask;
   return last_mask;
}

// Get the error mask
// return: Current error mask
uint32_t msg_get_err_mask(void) {
   return err_mask;
}

// Malloc and print error if fail
// 
void *msg_malloc(size_t size, char *msgstr) {
   void *ptr;

   ptr = malloc(size);
   if (ptr == NULL) {
      msg(MSG_FATAL,"Malloc failed %s size %u\n", msgstr, size);
      exit(1);
   }
   return ptr;
}

// Set logfile to log selected errors to
// file: File descriptor to write messages to
// mask: Error mask for errors to log
void msg_set_logfile(FILE *file, uint32_t mask) {
   logfile = file;
   logfile_err_mask = mask;
}
