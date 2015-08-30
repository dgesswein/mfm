// This module is various routines for doing Cyclic Redundancy Check (CRC)
// and Error Correction Code (ECC) calculations
// crc_revbits is used to reverse bit order in a word
// crc64 calculates CRC up to 64 bits long crc of data
// ecc64 corrects single burst errors
// checksum64 calculates checksums up to 64 bits long
//
// 01/04/15 DJG Added checksum64 function
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
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "crc_ecc.h"

// Find last set (leftmost 1). Not defined in Linux so we use a GCC specific call
// count leftmost zeros.
#define fls(x) (32 - __builtin_clz(x))

// Reverse the bit order of v. 00100001b will become 10000100b
// Not that efficient but we don't use it that often.
//
// v: value to reverse
// length: length of v in bits
// return: reverse of v
uint64_t crc_revbits(uint64_t v, int length)
{
   uint64_t i;
   uint64_t ov;
   uint64_t stop;

   if (length == 64) 
      stop = 0;
   else 
      stop = (uint64_t) 1 << length;

   ov = 0;
   for (i = 1; i != stop; i <<= 1) {
      ov <<= 1;
      if (v & i) {
         ov |= 1;
      }
   }
   return ov;
}

// Calculate a CRC up to 64 bits long over the specified data.
// CRC_INFO specifies the CRC length, polynomial, and initial value.
// The CRC is calculated most significant bit first.
// The CRC result is returned. Zero is normally defined as no error
// though other values can be used.
// There are two forms the CRC is normally implemented in. The polynomial can
// be converted between the two forms by
// poly = revbits(poly, length) << 1) | 1
//
// bytes: bytes to calculate CRC over
// num_bytes: length of bytes
// crc_info: CRC parameters to use
// return: CRC of bytes
uint64_t crc64(uint8_t bytes[], int num_bytes, CRC_INFO *crc_info)
{
   int64_t crc;
   // This improves performance vs. accessing directly in loop for the
   // GCC versions tested. The CRC polynomial
   uint64_t poly = crc_info->poly;
   // Loop counters
   int index, bit;

   crc = crc_info->init_value;
   for (index = 0; index < num_bytes; index++) {
      crc = crc ^ ((uint64_t) bytes[index] << (crc_info->length-8));
      for (bit = 1; bit <= 8; bit++) {
         // if leftmost (most significant) bit is set xor in the polynomial
         if (crc & ((uint64_t) 1 << (crc_info->length-1))) {
            crc = (crc << 1) ^ poly;
         } else {
            crc = crc << 1;
         }
      }
   }
   // Trim the result to the requested length
   if (crc_info->length == 64)
      return crc;
   else
      return crc & (((uint64_t) 1 << crc_info->length)-1);
}

// Correct the specified data given the syndrome (incorrect CRC value) and
// the CRC parameters. Return is non zero if a correction has been applied.
// It is possible the correction is wrong if sufficient bits are in error.
// Miscorrection probability depends on span length vs. polynomial length and
// quality of the polynomial chosen.
//
// bytes: bytes to calculate CRC over
// num_bytes: length of bytes
// syndrome: Value return by crc64
// crc_info: CRC/ECC parameters to use
// return: Length of correction in bits. Zero if no correction possible
//   also bytes modified if return value is non zero.
int ecc64(uint8_t bytes[], int num_bytes, uint64_t syndrome, CRC_INFO *crc_info)
{
   // Number of bits corrected
   int span;
   // Loop counters
   int index, bit;
   int bits_left;
   uint64_t poly;
   uint64_t crc_mask = (((uint64_t) 1 << 
         (crc_info->length - crc_info->ecc_max_span)) - 1) <<
         (crc_info->ecc_max_span);

   span = 0;
   syndrome = crc_revbits(syndrome, crc_info->length); 
   poly = ((crc_revbits(crc_info->poly, crc_info->length) << 1) | 1);
   // Stop when span is non zero (correction found)
   for (index = num_bytes; index > 0 && span == 0; index--) {
      // We continue after correction found to align the correction data to bytes
      for (bit = 1; bit <= 8; bit++) {
         // if leftmost (most significant) bit is set
         if (syndrome & ((uint64_t) 1 << (crc_info->length-1))) {
            syndrome = (syndrome << 1) ^ poly;
         } else {
            syndrome = syndrome << 1;
         }
         // If the upper ecc_max_span bits are zero we have found our correction
         if ((syndrome & crc_mask) == 0 && span == 0) {
            // Store the length of the correction
            span = fls(syndrome) - ffs(syndrome) + 1;
         }
      }
   }
   // If we found a correction fix the data
   if (span != 0) {
      // Round up span to handle worst case split across bytes
      bits_left = crc_info->ecc_max_span + 7;
      // Apply the correction to all possibly affected bytes
      while (bits_left > 0 && index < num_bytes) {
         bytes[index] ^= crc_revbits((syndrome & 0xff),8);
         index++;
         syndrome >>= 8;
         bits_left -= 8;
      }
   }
   return span;
}

// This is a checksum for formats that don't use a CRC
// bytes: bytes to calculate checksum over
// num_bytes: length of bytes
// crc_info: CRC parameters to use. We only use the initial value and length
// return: Checksum of bytes
uint64_t checksum64(uint8_t *bytes, int num_bytes, CRC_INFO *crc_info)
{
   int i;
   uint64_t sum;

   sum = crc_info->init_value;

   for (i = 0; i < num_bytes; i++) {
      sum += bytes[i];
   }  
   // Trim the result to the requested num_bytes
   if (crc_info->length == 64)
      return sum;
   else
      return sum & (((uint64_t) 1 << crc_info->length)-1);
}


