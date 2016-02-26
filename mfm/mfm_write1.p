#define CYCLE_CNTR r5
// This code handles disk write getting the MFM bitstream from DDR 
// memory and converting it to pulse width modulation (PWM) words for PRU 0.
// 
// See comments in mfm_write0.p for more information.
//
// The write data goes through a circular buffer in shared memory
// which allows time for the DMA start code to run.
// Tests show reads can be delayed up to 5 microseconds (1000 cycles). To
// deal with that the data if fetch using DMA. Two buffers are used. When one
// buffer is empty and the second is 1/2 empty we start the next DMA. We need
// to back up when a write starts since we have processed some words
// that were queued for PRU0. At the end of the write we will need to skip
// forward some to stay in sync with rotation.
//
// An entire cylinder is stored in the DDR memory. If the head select
// lines are changed PRU 0 will tell us to restart sending data and we
// will calculate the correct location to send the data from.
// 
// TODO: Comments haven't all been fixed for the DMA changes.
//
// See mfm_write0.p for information on the transfer memory registers and
// communication between the two PRU's.

// See drive_write.c for PRU1_BIT_TABLE format.

//
// Copyright 2016 David Gesswein.
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
#include "inc/cmd_write.h"

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
   // Last write precompensation write shift
#define LAST_PRECOMP  r12
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
   // Count of bits in track. Must be in byte field
#define BIT_COUNT     r21.b2
#define BIT_COUNT_MSB 7
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

      // Local memory starts at address 0 in memory map
   MOV      CONST_32, 32

   MOV      PRU1_BUF_STATE, 0

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

   // Get track byte count and convert to words, all must be same
   LBCO     TRACK_BYTES, CONST_PRURAM, PRU1_DRIVE0_TRACK_DATA_BYTES, 4 

   MOV      DDR_OFFSET, 0
   LSR      WORDS_LEFT, TRACK_BYTES, 2 // divide by 4 to get words

      // Calculate start of track by multiply head by track size in bytes
      // and add in header size to get start address of data
      // Get current head from PRU 0
   LBCO     r29, CONST_PRURAM, PRU1_CUR_HEAD, 4
   MOV      r28, TRACK_BYTES
   LBCO     r0, CONST_PRURAM, PRU1_DRIVE0_TRACK_HEADER_BYTES, 4 
   ADD      DDR_OFFSET, DDR_OFFSET,  r0  // Point after header
   ADD      r28, r28, r0
   MOV      r28, r28              // NOP, multiply has one cycle latency
   XIN      0, r26, 8             // Get multiply results (27,26=28*29)
   ADD      DDR_OFFSET, DDR_OFFSET, r26  // Point to our head track 
   MOV      TRACK_OFFSET, DDR_OFFSET
   MOV      TRACK_BIT, 0

   
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

      // Clear the next to top most bit so we can't get a decoding
      // error. Bit isn't critical so changing it won't mess up
      // emulation.
   CLR      WORD1, 30
   MOV     LAST_PRECOMP, 0

// TODO This is a little slow, ~150 ns/output word 200ns is limit
bitloop:
   LSR      r0, WORD1, 26                  // Get upper 6 bits of MFM data
   LSL      r0, r0, 2                      // Make byte address of word
   LBBO     r0, r0, PRU1_BIT_TABLE, 4      // Lookup next PWM word
   // Adjust timing based on previous shift
   ADD      r0, r0, LAST_PRECOMP
   QBBS     negval, r0, 15
   MOV      LAST_PRECOMP, 0
   JMP      storeval
negval:
   MOV      LAST_PRECOMP, -1 
storeval:
   MOV      LAST_PRECOMP.b0, r0.b1
   MOV      r0.b1, 0
   SBCO     r0, CONST_PRUSHAREDRAM, PRU1_BUF_OFFSET, 4  // Store it
   ADD      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, 4         // Move to next location
      // Wrap if needed
   AND      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, SHARED_PWM_READ_MASK   
   XOUT     10, PRU1_BUF_STATE, 4          // Send our offset
   QBBC     noerr, r0, 24
   LBCO     r2, CONST_PRURAM, PRU1_BAD_PATTERN_COUNT, 4
   ADD      r2, r2, 1
   SBCO     r2, CONST_PRURAM, PRU1_BAD_PATTERN_COUNT, 4
noerr:
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
   QBNE     check_exit, PRU0_STATE, STATE_READ 
   QBNE     bitloop, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET 
      // Indicate buffer has data and PRU 0 can start PWM generation
   MOV      PRU1_STATE, STATE_READ_FILLED
   XOUT     10, PRU1_BUF_STATE, 4                    
   JMP      waitfree

      // Write a zero word to buffer to indicate end of data for track
finished:
   XIN      10, PRU0_BUF_STATE, 4          // Get current read location pru 0
   QBNE     check_exit, PRU0_STATE, STATE_READ   // Handle state change
   QBEQ     finished_full, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET // no room wait
   MOV      r2, 0                          // Zero marks end of PWM words
   SBCO     r2, CONST_PRUSHAREDRAM, PRU1_BUF_OFFSET, 4  // Store it
   ADD      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, 4         // Move to next location
      // Wrap if needed
   AND      PRU1_BUF_OFFSET, PRU1_BUF_OFFSET, SHARED_PWM_READ_MASK
   XOUT     10, PRU1_BUF_STATE, 4

      // Wait for PRU0 to enter read done state
finished_wait:
   XIN      10, PRU0_BUF_STATE, 4
   QBNE     check_exit, PRU0_STATE, STATE_READ   // Handle state change
   JMP      finished_wait

finished_full:
      // Indicate buffer has data and PRU 0 can start PWM generation
   MOV      PRU1_STATE, STATE_READ_FILLED
   XOUT     10, PRU1_BUF_STATE, 4                    
   JMP      finished

   // See if switch to write or terminated was requested. If not restart read.
check_exit:
   QBEQ    DONE, PRU0_STATE, STATE_EXIT
      // If PRU 0 stopped read restart, wait_read sets state to done
   QBEQ    wait_read, PRU0_STATE, STATE_READ_DONE
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

   // Start DMA from DDR to local memory
   // r2 = size in bytes
   // r3 = destination local address
   // r4 = ddr offset
   // r2, r24-r29 modified
start_dma:
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

