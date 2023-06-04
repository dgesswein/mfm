// This is a replacement for deltas_read.c for use when reading data from a file
// instead of a real drive. See deltas_read for more information.
//
// Copyright 2015 David Gesswein.
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
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "msg.h"

// The number of deltas we have to process
uint32_t num_deltas;

// Update the count of deltas. Always not streaming for reading files.
//
// num_deltas_in: Number of deltas read
void deltas_update_count(int num_deltas_in, int streaming_in)
{
   num_deltas = num_deltas_in;
}

// Return deltas available or -1 if caller has processed all the deltas.
//
// cur_delta: Number of deltas processed so far
// return: Number of deltas available or -1 if all processed.
int deltas_get_count(int deltas_processed)
{
   if (deltas_processed >= num_deltas) {
      return -1;
   } else {
      return num_deltas;
   }
}

void deltas_start_read(void) {
   msg(MSG_FATAL, "deltas_start_read called\n");
   exit(1);
}
void deltas_start_thread(void) {
   msg(MSG_FATAL, "deltas_start_thread called\n");
   exit(1);
}
void deltas_stop_thread(void) {
   msg(MSG_FATAL, "deltas_stop_thread called\n");
   exit(1);
}

void deltas_wait_read_finished(void) {
}
