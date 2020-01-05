#define CYCLE_CNTR r5
// This code handles for disk reads getting the MFM bitstream from DDR 
// memory and converting it to pulse width modulation (PWM) words for PRU 0.
// For write it takes the delta time between MFM transitions, decodes it into
// clock and data bits and updates the bitstream in the DDR memory.
// 
// See comments in prucode0.p for more information.
//
// The read and write data goes through a circular buffer in shared memory
// which allows time for the DMA start code to run.
// Tests show reads can be delayed up to 5 microseconds (1000 cycles). To
// deal with that the data if fetch using DMA. Two buffers are used. When one
// buffer is empty and the second is 1/2 empty we start the next DMA. We need
// to back up when a write starts since we have processed some words
// that were queued for PRU0. At the end of the write we will need to skip
// forward some to stay in sync with rotation.
//
// Writes seem much better with delays around 200 cycles. Writes are
// posted which reduces the delay. If we get behind in writes
// we will corrupt the track. Words need to be written faster than 40
// cycles (200 ns) on average.
// Clock is about 200 MHz, 5 nanoseconds. It will be adjusted to generate
// the desired bit rate.

// An entire cylinder is stored in the DDR memory. If the head select
// lines are changed PRU 0 will tell us to restart sending data and we
// will calculate the correct location to send the data from. PRU 0
// will tell us the starting bit of the write data. We mark the cylinder
// dirty so the ARM can write it back to disk.
// PRU 0 handles getting the data updated if the controller selects another
// cylinder.
// 
// TODO: Comments haven't all been fixed for the DMA changes.
//
// See prucode0.p for information on the transfer memory registers and
// communication between the two PRU's.

// See mfm_emu.c for PRU1_BIT_TABLE format.
//
// 01/05/20 DJG Removed debug to prevent rare emu shutdown when data
//              not ready in time to pru0.
// 05/19/17 DJG Fixed comment.
// 04/30/16 DJG Increased WRITE_READ_TIME and NEW_READ_TIME to prevent abort
//    during OS install on 3b2. TODO: Need to investigate why these
//    need to be longer than expected.
// 02/20/16 DJG Fixed handling of time past track length in convert_track_time
// 11/20/15 DJG Fixed occasional abort due to data not being ready by
//    increasing WRITE_READ_TIME
// 05/20/15 DJG Significant change to switch to DMA (on PRU1) for reading
//    from DRAM and how data is syncronized to simulated rotation time.
// 01/04/15 DJG Added logic to allow bit rate to be varied and the
//   index pulse shifted relative to the start of the data. When end of
//   track is reached we wrap without needing a handshake between the PRU
//   and restarting read.
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

.setcallreg r19.w0
#define RETREG r19

.origin 0
.entrypoint START

#include "prucode.hp"

#include "inc/cmd.h"

// These are registers that are used globally. Check registers defined in 
// cmd.h if looking for free registers.

   // Value 32
#define CONST_32        r7.b3
#define NEXT_DMA_WCOUNT r7.w0 
#define NEXT_DMA_WCOUNT_MSB 15
   // r8 PRU1_BUF_STATE
   // r9 PRU0_BUF_STATE
   // Current offset into DDR memory. Not updated during read
#define DDR_OFFSET    r10
   // Words in DDR memory left to process in track
#define WORDS_LEFT    r11
// r12 unused
   // Address in PRU RAM DMA buffer
#define BUF_ADDR      r13
   // Current data word processing from DDR
#define WORD1         r14
   // Next data word processing
#define WORD2         r15
// r16 unused
   // Total number of bytes in track (need to change LBCO if changed to w0)
#define TRACK_BYTES   r17
// r18 TRACK_BIT
// r19.w0 call return addr, r19.w2 save call return
// r20 is DRIVE_DATA
   // Count of bits in track. Must be in byte field
#define BIT_COUNT     r21.b2
#define BIT_COUNT_MSB 7
   // Clocks in a MFM clock/data bit cell
#define BIT_PRU_CLOCKS r21.b1
   // Threshold for detecting zero bit cell
#define ZERO_THRESHOLD r21.b0
   // Offset from start of DDR memory of track data
#define TRACK_OFFSET  r22
   // Physical address of start of DDR memory
#define DDR_ADDR      r23
// r24-r29 subroutine usage

// Overwritten when multiply used
   // Base address of EDMA registers
#define EDMA_BASE     r26
   // EDMA channel to use
#define EDMA_PARAM_BASE r27
   // EDMA channel to use
#define EDMA_CHANNEL  r28


   // DMA buffers are 64 bytes/16 words. Must be power of 2 and <= 128
#define DMA_SIZE      64
   // Must be on 256 byte boundry
#define READ_BUF_1_ADDR               0x400
#define READ_BUF_2_ADDR               (READ_BUF_1_ADDR + DMA_SIZE)

   // Time in PRU clocks from write end to output read data
#define WRITE_READ_TIME 3100/5
   // Time from read time requested to bits we generate
   // 8.7 microseconds in PRU clocks. This allows for completing DMA
   // If this is increased MAX_TIME_OFFSET IN prucode0.p will need to
   // be increased.
#define NEW_READ_TIME 8700/5


START:
      // Enable OCP master port
   LBCO     r0, CONST_PRUCFG, 4, 4
      // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
   CLR      r0, r0, 4         
   SBCO     r0, CONST_PRUCFG, 4, 4

      // Configure the programmable pointer register by setting c25_pointer[15:0]
      // field to 0x0020.  This will make C25 point to 0x00002000 (Other PRU RAM).
   MOV      r0, 0x00000020
   MOV      r4, PRU1_CONTROL | CTBIR
   SBBO     r0, r4, 0, 4
      // Configure the programmable pointer register for PRU0 by setting 
      // c31_pointer[15:0] field to 0x0010.  This will make C31 point to 
      // 0x80001000 (DDR memory). Configure the programmable pointer register
      // for PRU0 by setting c28_pointer[15:0] field to 0x0100.  This will
      //  make C28 point to 0x00010000 (PRU shared RAM).
   MOV      r0, 0x00000100
   MOV      r4, PRU1_CONTROL | CTPPR_0
   SBBO     r0, r4, 0, 4

      // Configure the programmable pointer register for PRU0 by setting 
      // c31_pointer[15:0] field to 0x0010.  This will make C31 point to
      //  0x80001000 (DDR memory).
   MOV      r0, 0x00100000
   MOV      r4, PRU1_CONTROL | CTPPR_1
   SBBO     r0, r4, 0, 4

      // Get address and size of DDR buffer. Currently we don't use size
   LBCO     DDR_ADDR, CONST_PRURAM, PRU_DDR_ADDR, 4
   //LBCO     DDR_SIZE, CONST_PRURAM, PRU_DDR_SIZE, 4

   MOV      r2, 0
   SBCO     r2, CONST_PRURAM, PRU1_BAD_PATTERN_COUNT, 4
   SBCO     r2, CONST_PRURAM, PRU_TEST0, 4
   SBCO     r2, CONST_PRURAM, PRU_TEST1, 4
   SBCO     r2, CONST_PRURAM, PRU_TEST2, 4
   SBCO     r2, CONST_PRURAM, PRU_TEST3, 4
   SBCO     r2, CONST_PRURAM, PRU_TEST4, 4

   SBCO     r2, CONST_PRURAM, PRU1_DRIVE0_TRK_DIRTY, 4
   SBCO     r2, CONST_PRURAM, PRU1_DRIVE1_TRK_DIRTY, 4

   // Configure DMA

    // Load base addresses into registers
    MOV  EDMA_BASE, EDMA0_CC_BASE
    LBCO EDMA_CHANNEL, CONST_PRURAM, PRU1_DMA_CHANNEL, 4  // Read channel

    // Setup EDMA region access for Shadow Region 1
    MOV  r4, 0
    // Bit mask for channel. Only works for channels < 32
    SET  r4, EDMA_CHANNEL 
    MOV  R2, DRAE1
    SBBO r4, EDMA_BASE,  R2, 4 // Region 1
   
    // DCHMAP_# is set by system

    // Clear channel event from EDMA event registers
    MOV   r3, GLOBAL_SECR
    SBBO  r4, EDMA_BASE,  r3, 4 
    MOV   r3, GLOBAL_ICR
    SBBO  r4, EDMA_BASE,  r3, 4 
    MOV   r3, GLOBAL_IER
    LBBO  r0, EDMA_BASE,  r3, 4 
    // Disable channel interrupt
    MOV   r3, GLOBAL_IECR
    SBBO  r4, EDMA_BASE,  r3, 4 
    // Enable shadow 1 interrupt. Interrupts don't save time since
    // Syncronizing write to clear interrupt takes as much time as
    // just polling.
//    MOV   r3, IESRL
//    SBBO  r4, EDMA_BASE,  r3, 4 

    // Clear event missed register
    MOV   r3, EMCR
    SBBO  r4, EDMA_BASE,  r3, 4 


    // Setup and store PaRAM data for transfer
    LSL  EDMA_PARAM_BASE, EDMA_CHANNEL, 5 // channel*32
    MOV  r4, PARAM_OFFSET
    ADD  EDMA_PARAM_BASE, EDMA_PARAM_BASE, r4
        // Address of our channel PaRAM
    ADD  EDMA_PARAM_BASE, EDMA_PARAM_BASE, EDMA_BASE  

    // Build initial DMA descriptors. They will be modifed by dma routines
    // First entry OPT TCC = channel, enable transfer complete interrupt
    // SRC will be set later
    // ACNT = DMA_SIZE, BCNT = 1,
    // DST = buffer 1
    // SRC, DST BIDX = 0
    // LINK = next PaRAM set, BCNTRLD = 0
    // CCNT = 1
    LSL   r0, EDMA_CHANNEL, 12
    OR    r0.w2, r0.w2, 0x10
    SBBO  r0, EDMA_PARAM_BASE, OPT, 4
       // This will be updated when submitted
    SBBO  DDR_ADDR, EDMA_PARAM_BASE,  SRC, 4
    MOV   r0, DMA_SIZE
    SBBO  r0, EDMA_PARAM_BASE, A_B_CNT, 2  	
    MOV   r0, 1
    SBBO  r0, EDMA_PARAM_BASE, A_B_CNT+2, 2	 		
    MOV   r0, READ_BUF_1_ADDR
    LBCO  r1, CONST_PRURAM, PRU_DATARAM_ADDR, 4
    ADD   r0, r0, r1
    SBBO  r0, EDMA_PARAM_BASE, DST, 4
    MOV   r0, 0
    SBBO  r0, EDMA_PARAM_BASE, SRC_DST_BIDX, 4
    ADD   r0, EDMA_PARAM_BASE, 32
    MOV   r0.w2, 0
    SBBO  r0, EDMA_PARAM_BASE, LINK_BCNTRLD, 4
    MOV   r0, 0
    SBBO  r0, EDMA_PARAM_BASE, SRC_DST_CIDX, 4
    MOV   r0, 1
    SBBO  r0, EDMA_PARAM_BASE, CCNT, 2

    ADD   EDMA_PARAM_BASE, EDMA_PARAM_BASE, 32     // Go to next entry

    // Second entry OPT TCC = channel, enable transfer complete interrupt
    // SRC will be set later
    // ACNT = DMA_SIZE, BCNT = 1,
    // DST = buffer 2
    // SRC, DST BIDX = 0
    // LINK = next PaRAM set, BCNTRLD = 0
    // CCNT = 1
    LSL   r0, EDMA_CHANNEL, 12
    OR    r0.w2, r0.w2, 0x10		//TCINTEN and TCC = PruDmaChannel
    SBBO  r0, EDMA_PARAM_BASE, OPT, 4
       // This will be updated when submitted
    SBBO  DDR_ADDR, EDMA_PARAM_BASE,  SRC, 4
    MOV   r0, DMA_SIZE
    SBBO  r0, EDMA_PARAM_BASE, A_B_CNT, 2  	
    MOV   r0, 1
    SBBO  r0, EDMA_PARAM_BASE, A_B_CNT+2, 2	 		
    MOV   r0, READ_BUF_1_ADDR
    LBCO  r1, CONST_PRURAM, PRU_DATARAM_ADDR, 4
    ADD   r0, r0, r1
    SBBO  r0, EDMA_PARAM_BASE, DST, 4
    MOV   r0, 0
    SBBO  r0, EDMA_PARAM_BASE, SRC_DST_BIDX, 4
    // Reload from this entry
    MOV   r0, EDMA_PARAM_BASE
    MOV   r0.w2, 0
    SBBO  r0, EDMA_PARAM_BASE, LINK_BCNTRLD, 4
    MOV   r0, 0
    SBBO  r0, EDMA_PARAM_BASE, SRC_DST_CIDX, 4
    MOV   r0, 1
    SBBO  r0, EDMA_PARAM_BASE, CCNT, 2

    ADD   EDMA_PARAM_BASE, EDMA_PARAM_BASE, 32

      // Set multiply only mode. Only needs to be done once
   MOV      r25, 0                
   XOUT     0, r25, 1

   LBCO     ZERO_THRESHOLD, CONST_PRURAM, PRU1_ZERO_BIT_THRESHOLD, 4
   LBCO     BIT_PRU_CLOCKS, CONST_PRURAM, PRU1_BIT_PRU_CLOCKS, 4
        
      // Local memory starts at address 0 in memory map
   MOV      CONST_32, 32

   MOV      PRU1_BUF_STATE, 0
      // Default to first drive
   MOV      DRIVE_DATA, 0

      // Track timer
   MOV      CYCLE_CNTR, PRU0_CONTROL | CYCLE

wait_read:
//SET r30, 0
   // Wait for PRU0, exit if requested. 
   MOV      PRU1_STATE, STATE_READ_DONE
   MOV      PRU1_BUF_OFFSET, 0
   XOUT     10, PRU1_BUF_STATE, 4 
wait_read_lp:
   XIN      10, PRU0_BUF_STATE, 4
   QBEQ     DONE, PRU0_STATE, STATE_EXIT
   QBNE     wait_read_lp, PRU0_STATE, STATE_READ 

      // Get offset to drive data for selected drive
   XIN      10, DRIVE_DATA, 4         
   // Get track byte count and convert to words, all must be same
   LBBO     TRACK_BYTES, DRIVE_DATA, PRU1_DRIVE0_TRACK_DATA_BYTES, 4 

   MOV      DDR_OFFSET, 0
   QBEQ     drive_0, DRIVE_DATA, 0
      // Address of second drive data
   MOV      DDR_OFFSET, DDR_DRIVE_BUFFER_MAX_SIZE
drive_0:
   LSR      WORDS_LEFT, TRACK_BYTES, 2 // divide by 4 to get words

      // Calculate start of track by multiply head by track size in bytes
      // and add in header size to get start address of data
      // Get current head from PRU 0
   LBCO     r29, CONST_PRURAM_OTHER, PRU0_CUR_HEAD, 4
   MOV      r28, TRACK_BYTES
   LBBO     r0, DRIVE_DATA, PRU1_DRIVE0_TRACK_HEADER_BYTES, 4 
   ADD      DDR_OFFSET, DDR_OFFSET,  r0  // Point after header
   ADD      r28, r28, r0
   MOV      r28, r28              // NOP, multiply has one cycle latency
   XIN      0, r26, 8             // Get multiply results (27,26=28*29)
   ADD      DDR_OFFSET, DDR_OFFSET, r26  // Point to our head track 
   MOV      TRACK_OFFSET, DDR_OFFSET

   
      // Fetch track time and convert to word offset in r4 and
      // track bit in r27
   LBBO     r0, CYCLE_CNTR, 0, 4  // get track rotation time
SBCO    r0, CONST_PRURAM, 0xfc, 4

      // Give us enough time to get the bits out. PRU0 will start
      // outputting at this time.
   MOV      r1, NEW_READ_TIME
   ADD      r0, r0, r1

   CALL     convert_track_time    // r4 word offset, r27 bit count
   LSL      r0, r4, 2             // Convert word offset to byte
   ADD      DDR_OFFSET, TRACK_OFFSET, r0    // Add offset into track
   SUB      WORDS_LEFT, WORDS_LEFT, r4      // Get how many words left
      // Convert byte offset from start of track to bits to determine where
      // we start sending data.
   SUB      TRACK_BIT, DDR_OFFSET, TRACK_OFFSET   
   LSL      TRACK_BIT, TRACK_BIT, 3        // In bytes, convert to bits
   AND      r6, r27, 0x1f                  // Get how many bits into first word
   ADD      TRACK_BIT, TRACK_BIT, r6
   XOUT     10, TRACK_BIT, 4               // And tell PRU 0 start bit of data
   MOV      PRU1_STATE, STATE_READ_BIT_SET
   XOUT     10, PRU1_BUF_STATE, 4                    
   MOV      PRU1_BUF_OFFSET, 0             // Start filling at first location

   MOV      r2, DMA_SIZE*2
   MOV      r3, READ_BUF_1_ADDR
   MOV      r4, DDR_OFFSET
   CALL     start_dma
   CALL     wait_dma

   MOV      BUF_ADDR, READ_BUF_1_ADDR
      // DMA half way through second buffer. DMA_SIZE is in bytes but we
      // count words.
   MOV      NEXT_DMA_WCOUNT, DMA_SIZE/4 + DMA_SIZE/8
restart_read:
   MOV      BIT_COUNT, 31                  // 32 bits to process, negative stop
   LBCO     WORD1, CONST_PRURAM, BUF_ADDR, 8 // Get WORD1,WORD2
      // Increment by two words we read
   CALL     update_dma
   CALL     update_dma

LBBO     r0, CYCLE_CNTR, 0, 4   // get time
SBCO    r0, CONST_PRURAM, 0xf4, 4

      // Do shift to get proper bit to start of word.
   QBEQ     noshift, r6, 0
   LSL      WORD1, WORD1, r6
   SUB      r0, CONST_32, r6
   LSR      r2, WORD2, r0                // High bits to move to WORD1
   OR       WORD1, WORD1, r2
   LSL      WORD2, WORD2, r6             // And move next bits to top
   SUB      BIT_COUNT, BIT_COUNT, r6
noshift:
      // Clear the next to top most bit so we can't get a decoding
      // error. Bit isn't critical so changing it won't mess up
      // emulation.
   CLR      WORD1, 30

// TODO This is a little slow, ~150 ns/output word 200ns is limit
bitloop:
   LSR      r0, WORD1, 28                  // Get upper 4 bits of MFM data
   LSL      r0, r0, 2                      // Make byte address of word
   LBBO     r0, r0, PRU1_BIT_TABLE, 4      // Lookup next PWM word
   SBCO     r0, CONST_PRUSHAREDRAM, PRU1_BUF_OFFSET, 4  // Store it
   ADD      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, 4         // Move to next location
      // Wrap if needed
   AND      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, SHARED_PWM_READ_MASK   
   XOUT     10, PRU1_BUF_STATE, 4          // Send our offset
// PRU0 timeded out vary rarely at halt before filled: Instead of increasing
// start delay this code which isn't really needed removed. Could also set data
// available when 2/3 full to avoid increasing it at expense of more code.
//   QBBC     noerr, r0, 24
//   LBCO     r2, CONST_PRURAM, PRU1_BAD_PATTERN_COUNT, 4
//   ADD      r2, r2, 1
//   SBCO     r2, CONST_PRURAM, PRU1_BAD_PATTERN_COUNT, 4
//noerr:
   LSR      r4, r0, 28           // Get shift count

      // Here we shift WORD1, WORD2 by r4. If we use up all bits in WORD2
      // we read another word into WORD2 and possibly shift it to correct
      // position.
   LSL      WORD1, WORD1, r4
      // This entry is used to move bits from WORD2 after read into WORD1
shift_word2:
   SUB      r0, CONST_32, r4
   LSR      r2, WORD2, r0                // High bits to move to WORD1
   OR       WORD1, WORD1, r2
   LSL      WORD2, WORD2, r4             // And move next bits to top
   SUB      BIT_COUNT, BIT_COUNT, r4
   QBBC     waitfree, BIT_COUNT, BIT_COUNT_MSB // Branch if bits left in WORD1
   SUB      WORDS_LEFT, WORDS_LEFT, 1
   QBEQ     finished, WORDS_LEFT, 0      // Branch if no words left

read_next_word:
      // Get next word and update pointer
   LBCO     WORD2, CONST_PRURAM, BUF_ADDR, 4 
   CALL     update_dma

      // If bitcount is -1 then shift done. Otherwise we need to
      // fix the bits that didn't get shifted into WORD1
   QBEQ     waitfree_setbit_count, BIT_COUNT, 255
      // Number of bits we need to shift into WORD1
   XOR      r4, BIT_COUNT, 255
   MOV      BIT_COUNT, 31                // 32 bits to process
   JMP      shift_word2
waitfree_setbit_count:
   MOV      BIT_COUNT, 31                // 32 bits to process
      // Check to see if space is available before doing next word
waitfree:
      // Get current read location and state of pru 0. If no longer
      // read go handle. Wait for room
   XIN      10, PRU0_BUF_STATE, 4        
   QBNE     check_write, PRU0_STATE, STATE_READ 
   QBNE     bitloop, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET 
      // Indicate buffer has data and PRU 0 can start PWM generation
   MOV      PRU1_STATE, STATE_READ_FILLED
   XOUT     10, PRU1_BUF_STATE, 4                    
LBBO     r0, CYCLE_CNTR, 0, 4   // get time
SBCO    r0, CONST_PRURAM, 0xf8, 4
   JMP      waitfree

      // Write a zero word to buffer to indicate end of data for track
finished:
SET r30, 0
   XIN      10, PRU0_BUF_STATE, 4          // Get current read location pru 0
   QBNE     check_write, PRU0_STATE, STATE_READ   // Handle state change
   QBEQ     finished_full, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET // no room wait
   MOV      r2, 0                          // Zero marks end of PWM words
   SBCO     r2, CONST_PRUSHAREDRAM, PRU1_BUF_OFFSET, 4  // Store it
   ADD      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, 4         // Move to next location
      // Wrap if needed
   AND      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, SHARED_PWM_READ_MASK
   XOUT     10, PRU1_BUF_STATE, 4
      // Repeat track, set word count to number of words in track
   LSR      WORDS_LEFT, TRACK_BYTES, 2 
CLR r30, 0
   JMP      read_next_word                 // Start from beginning of track
finished_full:
      // Indicate buffer has data and PRU 0 can start PWM generation
   MOV      PRU1_STATE, STATE_READ_FILLED
   XOUT     10, PRU1_BUF_STATE, 4                    
   JMP      finished

   // Convert time to track bit and offset from start of track
   // r0: time to convert
   // Out:
   // r4 = offset from start of track in words
   // r27 = track bit count
   // r26-r29 modified
convert_track_time:
      // Divide by r0 bit period. We multiply by 1/bit_period
      // scaled by 2^32 then divide the result by 2^32.
   LBCO     r29, CONST_PRURAM, PRU1_INV_BIT_PERIOD_S32, 4
   MOV      r28, r0
   MOV      r28, r28              // NOP, multiply has one cycle latency
   XIN      0, r26, 8             // Get multiply results (27,26=28*29)
      // Get bit count of track
   LSL      r29, TRACK_BYTES, 3
      // If more bits than in track wrap
   QBLT     convert_ok, r29, r27        
   SUB      r27, r27, r29
convert_ok:
      // Convert to word offset, divide by 2^5 (32), Also doing divide 2^32
      // by using upper 32 bits of multiply result
   LSR      r4, r27, 5
   RET

   // See if switch to write or terminated was requested. If not restart read.
check_write:
   QBEQ    DONE, PRU0_STATE, STATE_EXIT
      // If PRU 0 stopped read restart, wait_read sets state to done
   QBEQ    wait_read, PRU0_STATE, STATE_READ_DONE
   QBEQ    do_write, PRU0_STATE, STATE_WRITE_WAIT
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
   HALT
              
   // Update memory pointers for word read and start DMA if needed
   // r26-29 modified from check_dma and start_dma
update_dma:
   XOUT     11, r0, 20   // Save r0-r4
      // Update pointer
   ADD      BUF_ADDR.b0, BUF_ADDR.b0, 4
   AND      BUF_ADDR.b0, BUF_ADDR.b0, (DMA_SIZE*2-1)
   SUB      NEXT_DMA_WCOUNT, NEXT_DMA_WCOUNT, 1
   QBBS     need_dma, NEXT_DMA_WCOUNT, NEXT_DMA_WCOUNT_MSB  // Negative
   QBNE     no_dma, NEXT_DMA_WCOUNT, 0
need_dma:
   // Save return address before we do another call
   XOUT     11, RETREG, 4

   CALL     check_dma
      // DMA next buffer
   MOV      r2, DMA_SIZE
   LBCO     r3, CONST_PRURAM, PRU1_NEXT_DMA_RAM_OFFSET, 4
   LBCO     r4, CONST_PRURAM, PRU1_NEXT_DMA_DDR_OFFSET, 4
   CALL     start_dma
   XIN      11, RETREG, 4
      // Schedule next dma
   ADD      NEXT_DMA_WCOUNT, NEXT_DMA_WCOUNT, DMA_SIZE/4  // 4 to get words
no_dma:
   XIN      11, r0, 20   // restore r0-r4
   RET

do_write:
      // Mark data dirty so ARM can write it to disk.
   LBBO    r0, DRIVE_DATA, PRU1_DRIVE0_TRK_DIRTY, 4
   LBCO    r1, CONST_PRURAM_OTHER, PRU0_CUR_HEAD, 4
   SET     r0, r0, r1
   SBBO    r0, DRIVE_DATA, PRU1_DRIVE0_TRK_DIRTY, 4
      // Get bit location write starts from and tell PRU 0 we are ready
   XIN     10, TRACK_BIT, 4
   MOV     PRU1_STATE, STATE_WRITE_WAIT
   MOV     PRU1_BUF_OFFSET, 0         // Set to start of buffer
   XOUT    10, PRU1_BUF_STATE, 4
      // Wait for PRU 0 to go to write state
write_wait_lp:
   XIN     10, PRU0_BUF_STATE, 4         
   QBNE    write_wait_lp, PRU0_STATE, STATE_WRITE
   MOV     PRU1_STATE, STATE_WRITE
   XOUT    10, PRU1_BUF_STATE, 4

   LSR     DDR_OFFSET, TRACK_BIT, 5  // Convert bit count to word count

   MOV     r2, DDR_OFFSET
   CALL    find_buf_loc

      // Now convert to bit in word. We count from msb bit = 31 - remainder
   AND     BIT_COUNT, TRACK_BIT, 0x1f           // Remainder
   MOV     r0, 31
   SUB     BIT_COUNT, r0, BIT_COUNT
      // Convert DDR_OFFSET in words to bytes and add to start of track to
      // get offset of words we're writing.
   LSL     DDR_OFFSET, DDR_OFFSET, 2 
   ADD     DDR_OFFSET, DDR_OFFSET, TRACK_OFFSET
      // Clear WORD1 & 2. WORD1 will be a mask of the bits write didn't change
      // WORD2 the changed bits. We need this to combine the modified
      // bits with the unmodified bits in the word at the start and end of
      // the write.
   FILL    &WORD1, 4
   ZERO    &WORD2, 4
LBCO    r0, CONST_PRURAM, BUF_ADDR, 4   // Get word
LBBO    r1, DDR_ADDR, DDR_OFFSET, 4
QBEQ    delta_loop, r0, r1
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
HALT // Pointers mismatch
delta_loop:
   XIN     10, PRU0_BUF_STATE, 4                
   QBEQ    delta_loop, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET  // Wait for data
      // Get word, stop if zero. Update offset in buffer
   LBCO    r4, CONST_PRUSHAREDRAM, PRU1_BUF_OFFSET, 4
   QBEQ    write_done, r4, 0
   ADD     PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, 4
   AND     PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, SHARED_DELTAS_WRITE_MASK
   XOUT    10, PRU1_BUF_STATE, 4
      // If time is less than bit-period * 1.5 we will call it just a
      // one otherwise some zeros precede the one.
   QBGT    done_zeros, r4, ZERO_THRESHOLD
      // Write a zero bit, decrement time and check if more zero
wzero:
   CLR     WORD1, WORD1, BIT_COUNT
   SUB     BIT_COUNT, BIT_COUNT, 1
      // If count went negative write the word back to DDR
   QBBC    wnext, BIT_COUNT, BIT_COUNT_MSB
   CALL    write_word
wnext:
   SUB     r4, r4, BIT_PRU_CLOCKS
   QBLE    wzero, r4, ZERO_THRESHOLD
      // Write a one
done_zeros:
   CLR     WORD1, WORD1, BIT_COUNT
   SET     WORD2, WORD2, BIT_COUNT
   SUB     BIT_COUNT, BIT_COUNT, 1
   QBBC    delta_loop, BIT_COUNT, BIT_COUNT_MSB
   CALL    write_word
   JMP     delta_loop

      // Write the word back to DDR. If the mask is all ones we don't
      // need to read the existing word and combine.
      // r0 modified and r26-29 modified from update_dma
write_word:
   QBEQ    just_write, WORD1, 0
   LBCO    r0, CONST_PRURAM, BUF_ADDR, 4   // Get word
   AND     r0, r0, WORD1                   // Clear bits we wish to change
   OR      WORD2, WORD2, r0                // And put them in
just_write:
   SBBO    WORD2, DDR_ADDR, DDR_OFFSET, 4  // Store word
   ADD     DDR_OFFSET, DDR_OFFSET, 4       // Point to next
   SUB     WORDS_LEFT, WORDS_LEFT, 1
   // Save return address before we do another call
   MOV     RETREG.w2, RETREG.w0
   CALL    update_dma
   MOV     RETREG.w0, RETREG.w2
   QBNE    write_ret, WORDS_LEFT, 0        // Branch if more words in track
   MOV     DDR_OFFSET, TRACK_OFFSET        // Wrap to beginning of track
   LSR     WORDS_LEFT, TRACK_BYTES, 2      // divide by 4 to get words
write_ret:
   MOV     BIT_COUNT, 31
   FILL    &WORD1, 4
   ZERO    &WORD2, 4
   RET

write_done:
      // Write word in progress
   CALL     write_word
      // Let PRU 0 continue. 
   MOV      PRU1_STATE, STATE_WRITE_DONE
   XOUT     10, PRU1_BUF_STATE, 4
write_done_wait:
   XIN      10, PRU0_BUF_STATE, 4          // Get pru0 status
   QBNE     write_done_wait, PRU0_STATE, STATE_READ
//set r0 for proper bit
   LBBO     r0, CYCLE_CNTR, 0, 4           // get track rotation time
   MOV      r1, WRITE_READ_TIME
   ADD      r0, r0, r1
   CALL     convert_track_time             // r4 word offset, r27 bit count
   MOV      r2, r4
   CALL     find_buf_loc
   AND      r6, r27, 0x1f                  // Get how many bits into first word
   MOV      TRACK_BIT, r27
   XOUT     10, TRACK_BIT, 4               // And tell PRU 0 start bit of data
   MOV      PRU1_STATE, STATE_READ_BIT_SET
   XOUT     10, PRU1_BUF_STATE, 4                    


   LBBO     r0, CYCLE_CNTR, 0, 4   // get time
   SBCO    r0, CONST_PRURAM, 0xf0, 4


      // Restart read at BUF_LOC at r6 bit. NEXT_DMA_WCOUNT and WORDS_LEFT
      // set by find_buf_loc
   JMP      restart_read

   // Given bit count find location in buffer for word and count to
   // next DMA
   // in r2 = words into track buffer to convert, modified
   // out BUF_ADDR, NEXT_DMA_WCOUNT modified
   // r0 modified
find_buf_loc:
   LSR     WORDS_LEFT, TRACK_BYTES, 2       // divide by 4 to get words
   SUB     WORDS_LEFT, WORDS_LEFT, r2       // Words left
   LSL     r2, r2, 2                        // Convert to byte
   ADD     r2, r2, TRACK_OFFSET             // Convert to DDR address
      // Find number of bytes we need to back up. Handle if DMA address
      // wrapped to beginning of track
   LBCO    r24, CONST_PRURAM, PRU1_NEXT_DMA_DDR_OFFSET, 4
   QBGE    no_wrap2, r2, r24
   ADD     r24, r24, TRACK_BYTES
no_wrap2:
   SUB     r24, r24, r2
      // At least 3 words from end and no more than full buffer
      // one word to notice we need to start DMA and two more to
      // cover time DMA takes.
   QBGE    err2, r24, 3*4 
   QBGE    noerr2, r24, 2*DMA_SIZE
err2:
   MOV      r26, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r26, r25, 0, 4
   HALT
      // r24 is now number of words we need to back up from end
      // of last DMA (next DMA addr). We need to do next dma at that
      // many bytes minus half a buffer.
noerr2:
   LSR     NEXT_DMA_WCOUNT, r24, 2              // Words to end of buffer
      // We want to DMA half way in
   SUB     NEXT_DMA_WCOUNT, NEXT_DMA_WCOUNT, DMA_SIZE/2/4   
      // Back up by BUF_ADDR from end of last ram buffer address
      // if we go below first address wrap to end of buffer
   LBCO    r2, CONST_PRURAM, PRU1_NEXT_DMA_RAM_OFFSET, 4
   SUB     BUF_ADDR.b0, r2.b0, r24
   AND     BUF_ADDR.b0, BUF_ADDR.b0, (DMA_SIZE*2-1)
   RET

   // Start DMA from DDR to local memory
   // r2 = size in bytes
   // r3 = destination local address
   // r4 = ddr offset
   // r2, r24-r29 modified
start_dma:
LBCO    r24, CONST_PRURAM, 0xe0, 4
SBCO    r24, CONST_PRURAM, 0xe4, 4
LBBO    r24, CYCLE_CNTR, 0, 4   // get time
SBCO    r24, CONST_PRURAM, 0xe0, 4

    ADD     r24, r3, r2
    AND     r24.b0, r24.b0, (DMA_SIZE*2-1)
    SBCO    r24, CONST_PRURAM, PRU1_NEXT_DMA_RAM_OFFSET, 4
    ADD     r24, r4, r2
    ADD     r25, TRACK_BYTES, TRACK_OFFSET
      // If past end of track wrap
    QBLT    addr_ok, r25, r24
    SUB     r24, r24, r25
    ADD     r24, r24, TRACK_OFFSET
addr_ok:
    SBCO    r24, CONST_PRURAM, PRU1_NEXT_DMA_DDR_OFFSET, 4
    // Load base addresses into registers
    MOV     EDMA_BASE, EDMA0_CC_BASE
    LBCO    EDMA_CHANNEL, CONST_PRURAM, PRU1_DMA_CHANNEL, 4
    // Setup and store PaRAM data for transfer
    LSL     EDMA_PARAM_BASE, EDMA_CHANNEL, 5 // channel*32
    MOV     r24, PARAM_OFFSET
    ADD     EDMA_PARAM_BASE, EDMA_PARAM_BASE, r24
    ADD     EDMA_PARAM_BASE, EDMA_PARAM_BASE, EDMA_BASE  

    ADD     r24, r4, DDR_ADDR
    SBBO    r24, EDMA_PARAM_BASE,  SRC, 4
sbco r24, CONST_PRURAM, 0xd4, 4
    LBCO    r24, CONST_PRURAM, PRU_DATARAM_ADDR, 4
    ADD     r24, r24, r3
    SBBO    r24, EDMA_PARAM_BASE, DST, 4

    // Find bytes left in track
    ADD     r29, TRACK_BYTES, TRACK_OFFSET
    SUB     r29, r29, r4
    // First entry OPT TCC = channel, enable transfer complete interrupt
    // SRC will be set later
    // ACNT = DMA_SIZE, BCNT = 1,
    // DST = buffer 1
    // SRC, DST BIDX = 0
    // LINK = next PaRAM set, BCNTRLD = 0
    // CCNT = 1
    LSL     r24, EDMA_CHANNEL, 12
    QBGE    nowrap, r2, r29
    // We need to wrap, chain to next
    OR      r24.w2, r24.w2, 0x40
    SBBO    r24, EDMA_PARAM_BASE, OPT, 4
    MOV     r24, r29
    MOV     r24.w2, 1
sbco r24, CONST_PRURAM, 0xd0, 4
    SBBO    r24, EDMA_PARAM_BASE, A_B_CNT, 4  

    ADD     EDMA_PARAM_BASE, EDMA_PARAM_BASE, 32     // Go to next entry

    // Remaining bytes
    SUB     r2, r2, r29
    // Start at beginning of track
    ADD     r24, TRACK_OFFSET, DDR_ADDR
    SBBO    r24, EDMA_PARAM_BASE,  SRC, 4
    LBCO    r24, CONST_PRURAM, PRU_DATARAM_ADDR, 4
    // And after last data transferd for dest
    ADD     r24, r24, r3
    ADD     r24, r24, r29
    SBBO    r24, EDMA_PARAM_BASE, DST, 4

    // This will set the required last few entries for the first when jumped
    // here or for the second on fall through
    LSL     r24, EDMA_CHANNEL, 12
nowrap:
    OR      r24.w2, r24.w2, 0x10
    SBBO    r24, EDMA_PARAM_BASE, OPT, 4

    MOV     r24, r2
    MOV     r24.w2, 1
    SBBO    r24, EDMA_PARAM_BASE, A_B_CNT, 4  	

    // Clear flag
    MOV     r24, 0
    SET     r24, EDMA_CHANNEL
    MOV     r25, ICR
    SBBO    r24, EDMA_BASE, r25, 4
    MOV     r25, ESR
    // Triggering the transfer
    SBBO    r24, EDMA_BASE, r25, 4
LBBO    r24, CYCLE_CNTR, 0, 4   // get time
SBCO    r24, CONST_PRURAM, 0xe8, 4
    RET

	 		
   // r24-r26 modified
wait_dma:
    // Load base addresses into registers
    MOV    EDMA_BASE, EDMA0_CC_BASE
    MOV    r25, IPR
    LBCO   EDMA_CHANNEL, CONST_PRURAM, PRU1_DMA_CHANNEL, 4
wait_dma_loop:
   // TODO: This is slow, should we use interrupt?
    LBBO   r24, EDMA_BASE, r25, 4
    QBBC   wait_dma_loop, r24, EDMA_CHANNEL
LBBO    r24, CYCLE_CNTR, 0, 4   // get time
SBCO    r24, CONST_PRURAM, 0xec, 4
    RET

   // r24-r26 modified
check_dma:
    // Load base addresses into registers
    MOV    EDMA_BASE, EDMA0_CC_BASE
    MOV    r25, IPR
    LBCO   EDMA_CHANNEL, CONST_PRURAM, PRU1_DMA_CHANNEL, 4
   // TODO: This is slow, should we use interrupt?
    LBBO   r24, EDMA_BASE, r25, 4
    QBBC   wait_dma_halt, r24, EDMA_CHANNEL
    RET
wait_dma_halt:
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
    HALT

DONE:
      // Send notification to Host for program completion
   MOV       r31.b0, PRU1_ARM_INTERRUPT+16

      // Halt the processor
   HALT

      // Restarted here if the ARM wishes to abort an operation. Any needed
      // cleanup should be here to restore to a good state. Currently none
   .ORIGIN RESTART_ADDR
      // Send notification to Host for program completion
   MOV      r31.b0, PRU1_ARM_INTERRUPT+16

      // Halt the processor
   HALT

