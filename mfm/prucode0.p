// This code is for reading from MFM disks. It provides the
// following functions
// CMD_EXIT: Exit
// CMD_READ_TRACK:
//    Use the enhanced capture modules (eCAP) to measure the time
//    between MFM transitions. The data is put in the shared DDR memory. The
//    data 16 bit count of 200 MHz clocks between transitions.
// CMD_RPM:
//    Measure drive RPM
// CMD_SEEK_FAST:
//    Perform a buffered seek
// CMD_SEEK_SLOW:
//    Perform an unbuffered seek
// CMD_SEEK_SLOW_TRACK0
//    Return the head to track 0
// CMD_CHECK_READY
//    Wait for drive ready
//
// Commands are read from PRU0_CMD and we then write to the same
// location the status of the command.
// If the command needs a single word of data or returns one it is in
// PRU0_CMD_DATA
//
// Time is in 200 MHz clocks
// 05/17/15 DJG Fix times for slow seeks
// 01/04/15 DJG Added logic to delay start and end read by the set number
//   of clocks from the index pulse. Fixed initialization of ecap registers
//   to make it always start with capture register 1. Fix bug that
//   could cause a transition time read to be overwritten since flag
//   register wasn't masked by what we had read. Avoid possible loss of
//   transition value by only clearing the flag for value being read.
//   Made sure 0 written when clearing timer.
// 09/06/14 DJG Increased seek timeout to 4 seconds so if the drive
//    recalibrates we may not time out.
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

.setcallreg r29.w0

.origin 0
.entrypoint START

#include "prucode.hp"

#include "inc/cmd.h"

// 4 seconds, guess at slowest time to complete a seek including possible
// recalibrate.
#define SEEK_COMPLETE_TIMEOUT 800000000
// Pulse width for buffered seek pulses
#define SEEK_PW 1000    //  5 microseconds
#define SEEK_PW_SLOW 8000    //  40 microseconds
// Delay between buffered seek pulses
#define SEEK_WAIT 6000  // 30 microseconds
// Delay between unbuffered seek pulses
#define SEEK_WAIT_SLOW 800000  // 4 milliseconds
// Maximum time to wait for an index pulse
#define INDEX_TIMEOUT 5000000 // 25 milliseonds
// Time to wait for the head step direction line to settle
#define DIR_SETTLE_TIME 40 // 200 nanoseconds

START:
   // Enable OCP master port
   LBCO     r0, CONST_PRUCFG, 4, 4
   CLR      r0, r0, 4         // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
   SBCO     r0, CONST_PRUCFG, 4, 4

   // Configure the programmable pointer register for PRU0 by setting c28_pointer[15:0]
   // field to 0x0120.  This will make C28 point to 0x00012000 (PRU shared RAM).
   MOV      r0, 0x00000120
   MOV      r1, CTPPR_0
   SBBO     r0, r1, 0, 4

   // Configure the programmable pointer register for PRU0 by setting c31_pointer[15:0]
   // field to 0x0010.  This will make C31 point to 0x80001000 (DDR memory).
   MOV      r0, 0x00100000
   MOV      r1, CTPPR_1
   SBBO     r0, r1, 0, 4

   // Enable cycle counter. We use this for delays
   MOV      r3, PRU0_CONTROL_REG    
   LBBO     r2, r3, 0, 4            // Read reg
   SET      r2, r2, 3
   SBBO     r2, r3, 0, 4            // Write reg
   // R28 will be address of the cycle counter timer
   MOV      r28, PRU0_CYCLE_COUNTER_REG

   LBCO     r23, CONST_PRURAM, PRU_DDR_ADDR, 4  // Address of DDR buffer
   LBCO     r22, CONST_PRURAM, PRU_DDR_SIZE, 4  // Size in bytes of DDR buffer

   MOV      r0, 0x1e01aa            // Make sure we are in capture mode
   SBCO     r0, CONST_ECAP, 0x28, 4 // Load ECCTL1 and ECCTL2
   CLR      r30, R30_MFM0_IN_ENABLE // Turn on receiving read data

   CLR      r30, R30_STEP_BIT       // Make sure seek is off

   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
wait_cmd:
   // Clear timer to prevent it from from overflowing and stopping
   MOV      r2,0              
   SBBO     r2, r28, 0, 4
   // Store drive status for ARM
   SBCO     r31, CONST_PRURAM, PRU0_STATUS, 4
   // Get command and branch to correct routine
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     read_track, r1, CMD_READ_TRACK
   QBEQ     seek, r1, CMD_SEEK_FAST
   QBEQ     slow_seek, r1, CMD_SEEK_SLOW
   QBEQ     slow_seek_track0, r1, CMD_SEEK_SLOW_TRACK0
   QBEQ     rpm, r1, CMD_RPM
   QBEQ     check_ready_cmd, r1, CMD_CHECK_READY
   QBEQ     EXIT, r1, CMD_EXIT
   JMP      wait_cmd

   // We wait for the seek complete timeout for drive to be ready
check_ready_cmd:
   MOV      r3, SEEK_COMPLETE_TIMEOUT
   CALL     wait_ready
   // Not ready. Wait_ready already set error status
   QBNE     wait_cmd, r3,0             
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 
   JMP      wait_cmd

   // This reads a track worth of MFM delta transitions times
read_track:
   MOV      r3, 10                  // Make sure we are ready
   CALL     wait_ready
   QBNE     wait_cmd, r3, 0

   // R21 will be Offset in DDR buffer to write to. PRU0_WRITE_PTR
   // is the word the ARM reads to find how many deltas have been written
   MOV      r21, 0                   
   SBCO     r21, CONST_PRURAM, PRU0_WRITE_PTR, 4   // write ptr 

   // Pointer updated, let ARM know it can read it now
   MOV      r1, CMD_STATUS_READ_STARTED
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
 
   // Now find start of track. Start is falling edge of index signal
   MOV      r3, INDEX_TIMEOUT
   MOV      r2, 0
   SBBO     r2, r28, 0, 4    // Clear timer
wait_high:
   QBBS     wait_low, r31, R31_INDEX_BIT   // Wait for signal high
   LBBO     r0, r28, 0, 4
   QBLT     wait_high, r3, r0          // Try again if we haven't timed out
wait_timeout:
   MOV      r1, CMD_STATUS_INDEX_TIMEOUT
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command failed
   JMP      wait_cmd
wait_low:
   QBBC     wait_start_time, r31, R31_INDEX_BIT   // Wait for falling edge
   LBBO     r0, r28, 0, 4
   QBLT     wait_low, r3, r0
   JMP      wait_timeout

   // For some disk formats we need to delay start of data capture so we
   // get the sector the crosses the failling index pulse. Wait for the
   // time specified.
wait_start_time:
   MOV      r2, 0
   SBBO     r2, r28, 0, 4    // Clear timer
   LBCO     r3, CONST_PRURAM, PRU0_START_TIME_CLOCKS, 4
wait_start_loop:
   LBBO     r0, r28, 0, 4
   QBLT     wait_start_loop, r3, r0   // Try again if we haven't timed out

     // Trigger cap1 on rising edges. Reset count on trigger to get delta time
     //Enable cap1-4 buffer registers. Divide by 1, stop on emulation suspend
     // continuous mode, wrap after 4, no rearm, counter free run, no sync
     // capture mode. Reset sequencer
start_read:
   MOV      r7, CMD_STATUS_OK    // Command status in r7
   MOV      r0, 0                // Stop capture
   SBCO     r0, CONST_ECAP, 0x28, 4
   MOV      r0, 0x0              // Clear count
   SBCO     r0, CONST_ECAP, 0, 4
   // This will trigger on both edges. Might be possible to use the inforation
   // to help decode poor bits
   //MOV      r0, 0x1e01ee
   //MOV      r0, 0x1e01ff       // Negative edge
   // Trigger positive edges only, reset ECAP CAP counter
   // This sequence of resetting ECAP and flags seems necessary to
   // get capture to start with CAP1 register which the code needs.
   MOV      r0, 0x1e01aa         // ECCTL1 = 0x01aa & ECCTL2 = 0x1e
   MOV      r1, 0                // ECEINT and ECFLG
   MOV      r2, 0x1e             // ECCLR = 0x1e, ECFRC
   SBCO     r0, CONST_ECAP, 0x28, 12 // Load above registers

   // 0 if index has gone back high, one if index hasn't gone back high
   // 2 or more is extra time after index we continue capturing data
   MOV      r5, 1 

   // We store the four capture register values in R10-R13 then
   // write them all at once to DDR. This improved transfer rate.
   // for MFM not really needed. The buffering and prevents overruns
   // when the DDR writes stall. They can for numerous cycles if the
   // ARM processor is busy accessing the DDR.
   MOV      r1, 0                // Clear which counts read flag
caploop:
   LBCO     r0, CONST_ECAP, 0x2e, 2  // Read flags
   // If the flags are 0x1e then we took too long and lost transition(s)
   QBNE     no_overrun, r0, 0x1e
   MOV      r7, CMD_STATUS_READ_OVERRUN
no_overrun:
   // Clear flag for any CAP register we have processed (r1). We
   // must process them in order
   XOR      r2, r1, 0x1e             
   AND      r0, r0, r2
   QBBC     nxt1, r0, 1              // cap1 doesn't have data?
   LBCO     r10, CONST_ECAP, 0x8, 4  // Get count
   ADD      r10, r10, 1              // It's one less than actual time
   QBEQ     nxt0, r10.w2, 0          // Did we overflow?
   MOV      r10, 0xffff              // Set to maximum
   MOV      r7, CMD_STATUS_DELTA_OVERFLOW
nxt0:
      // Clear overflow so test for flag 0x1e will work
   MOV      r2, 0x22
   OR       r1, r1, r2               // Mark which word received
   SBCO     r2, CONST_ECAP, 0x30, 2  // Clear flag
nxt1:
   QBBC     nxt2, r0, 2              // cap2 doesn't have data?
   LBCO     r11, CONST_ECAP, 0xc, 4  // Get count
   ADD      r11, r11, 1              // It's one less than actual time
   QBEQ     nxt1b, r11.w2, 0     // Did we overflow?
   MOV      r11, 0xffff              // Set to maximum
   MOV      r7, CMD_STATUS_DELTA_OVERFLOW
nxt1b:
   MOV      r10.w2, r11.w0           // And store in upper 16 bits of R10
   MOV      r2, 0x24
   OR       r1, r1, r2               // Mark which word received
   SBCO     r2, CONST_ECAP, 0x30, 2  // Clear flag
nxt2:
   QBBC     nxt3, r0, 3              // cap3 doesn't have data?
   LBCO     r11, CONST_ECAP, 0x10, 4 // Get count
   ADD      r11, r11, 1              // It's one less than actual time
   QBEQ     nxt2b, r11.w2, 0         // Did we overflow?
   MOV      r11, 0xffff              // Set to maximum
   MOV      r7, CMD_STATUS_DELTA_OVERFLOW
nxt2b:
   MOV      r2, 0x28
   OR       r1, r1, r2               // Mark which word received
   SBCO     r2, CONST_ECAP, 0x30, 2  // Clear flag
nxt3:
   QBBC     nxt4, r0, 4              // cap4 doesn't have data?
   LBCO     r12, CONST_ECAP, 0x14, 4 // Get count
   ADD      r12, r12, 1              // It's one less than actual time
   QBEQ     nxt3b, r12.w2, 0         // Did we overflow?
   MOV      r12, 0xffff              // Set to maximum
   MOV      r7, CMD_STATUS_DELTA_OVERFLOW
nxt3b:
   MOV      r11.w2, r12.w0           // And store in upper 16 bits of R11
   MOV      r2, 0x30
   OR       r1, r1, r2               // Mark which word received
   SBCO     r2, CONST_ECAP, 0x30, 2  // Clear flag
   SBBO     r10, r23, r21, 8         // write times to ddr
   ADD      r21, r21, 8              // inc ptr
   // Stop if we will go past end of memory segment
   QBGT     overflow, r22, r21
   SBCO     r21, CONST_PRURAM, PRU0_WRITE_PTR, 4   // write ptr 
   MOV      r1, 0                    // No data stored
   JMP      caploop
   // Check if we are done
nxt4:
   QBLE     chk_time, r5, 2          // Need end capture delay?
   QBBC     chk_ignore, r31, R31_INDEX_BIT   // Index low?
   MOV      r5, 0       // Index has gone back high
   JMP      caploop 
chk_ignore:
      // Don't check for index low if we haven't seen high
   QBEQ     caploop, r5, 1      
   MOV      r2, 0
   SBBO     r2, r28, 0, 4           // Clear timer
   LBCO     r5, CONST_PRURAM, PRU0_START_TIME_CLOCKS, 4   // Get delay time
   QBLE     caploop, r5, 2          // Capture more if delay > 1
   JMP      read_done
chk_time:
   LBBO     r0, r28, 0, 4           // Read timer
   QBLT     caploop, r5, r0         // Try again if we haven't timed out
read_done:
      // Now we need to write the registers that have data. Probably
      // don't really need to do this since the data at the index is
      // filler.
   QBEQ     no_queued, r1, 0        // No data queued if zero
   MOV      r2, 2                   // r10.w0 must be used
   QBBC     store, r1, 2            // r10.w2 used?
   ADD      r2,r2, 2
   QBBC     store, r1, 3            // r11.w0 used?
   ADD      r2,r2, 2               
     // Don't need to check r11.w2, it can't be dirty by logic
store:
     // write times to ddr. We may write more data than needed but count will
     // be updated properly
   SBBO     r10, r23, r21, 8        
   ADD      r21, r21, r2            // Inc pointer
   SBCO     r21, CONST_PRURAM, PRU0_WRITE_PTR, 4   // write ptr 
no_queued:
   SBCO     r7, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   JMP      wait_cmd   
overflow:
   MOV      r1, CMD_STATUS_READ_OVERFLOW
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Error
   JMP      wait_cmd   

// To find track zero we see if we are at track zero. If not we
// step in, wait for seek complete, then repeat
slow_seek_track0:
   // Seek in, turn off active low seek in signal driven by inverter
   CLR      r30, R30_SEEK_DIR_BIT   
   MOV      r3, DIR_SETTLE_TIME
   MOV      r2, 0
   SBBO     r2, r28, 0, 4           // Clear timer
settle0_lp:
   LBBO     r0, r28, 0, 4
   QBLT     settle0_lp, r3, r0

seek0_lp:
   MOV      r0, GPIO0 | GPIO_DATIN          // Read bits
   LBBO     r0, r0, 0, 4
   QBBC     seek0_done, r0, GPIO0_TRACK_0   // Already at track0, stop

   // Pulse the step bit for the correct duration
   MOV      r3, SEEK_PW_SLOW
   SET      r30, R30_STEP_BIT       // Start pulse
   MOV      r2, 0x0                 // Keep zero around
   SBBO     r2, r28, 0, 4           // Clear timer
seek0_w:
   LBBO     r0, r28, 0, 4           
   QBLT     seek0_w, r3, r0
   CLR      r30, R30_STEP_BIT       

   // Wait for seek complete
   MOV      r3, SEEK_COMPLETE_TIMEOUT
   SBBO     r2, r28, 0, 4           // Clear timer
seek0_complete_lp:
   QBBC     seek0_lp, r31, R31_SEEK_COMPLETE_BIT  // Complete, check again
   LBBO     r0, r28, 0, 4
   QBLT     seek0_complete_lp, r3, r0
   MOV      r0, CMD_STATUS_SEEK_COMPLETE_ERR
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   JMP      wait_cmd   
seek0_done:
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   JMP      wait_cmd   

   // Set up time between pulses for type of seek
slow_seek:
   MOV      r6, SEEK_WAIT_SLOW
   MOV      r8, SEEK_PW_SLOW
   JMP      do_seek
seek:
   MOV      r6, SEEK_WAIT
   MOV      r8, SEEK_PW
do_seek:
   LBCO     r1, CONST_PRURAM, PRU0_CMD_DATA, 4 // Get tracks to seek
   MOV      r2, 0x0                 // Keep zero around
   QBBS     neg, r1, 31             // Is seek value negative?
   SET      r30, R30_SEEK_DIR_BIT
   JMP      dir_settle
neg:
   CLR      r30, R30_SEEK_DIR_BIT   // Seek in, set active low in signal
   RSB      r1, r1, 0               // Make count positive
dir_settle:
   MOV      r3, DIR_SETTLE_TIME
   SBBO     r2, r28, 0, 4           // Clear timer
settle_lp:
   LBBO     r0, r28, 0, 4
   QBLT     settle_lp, r3, r0

   // And generate the pulses at the specified rate
seek_lp:
   SET      r30, R30_STEP_BIT       // Start acive low pulse
   SBBO     r2, r28, 0, 4           // Clear timer
seek_w1:
   LBBO     r0, r28, 0, 4           // Wait for pulse width
   QBLT     seek_w1, r8, r0
   CLR      r30, R30_STEP_BIT       

   MOV      r3, r6                  // Get Idle time between pulses
   SBBO     r2, r28, 0, 4
seek_w2:
   LBBO     r0, r28, 0, 4
   QBLT     seek_w2, r3, r0

   SUB      r1, r1, 1
   QBLE     seek_lp, r1, 1          // And repeat if needed

check_ready:
   MOV      r3, SEEK_COMPLETE_TIMEOUT
   CALL     wait_ready
      // Didn't go ready, return to cmd loop. Error set in wait_ready
   QBNE     wait_cmd, r3, 0    
   MOV      r1, CMD_STATUS_OK
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   JMP      wait_cmd   

// Check for drive ready. r3 = timeout value on entry, 0 on return if ready
wait_ready:
   MOV      r2,0
   SBBO     r2, r28, 0, 4    // Clear timer
ready_loop:
   SBCO     r31, CONST_PRURAM, PRU0_STATUS, 4
   // Ready when seek complete, ready, and drive selected are low
   QBBS     ready_timeout, r31, R31_SEEK_COMPLETE_BIT
   QBBS     ready_timeout, r31, R31_READY_BIT
   QBBS     ready_timeout, r31, R31_DRIVE_SEL
   MOV      r3, 0    // We are ready, return good
   RET
ready_timeout:
   LBBO     r0, r28, 0, 4
   QBLT     ready_loop, r3, r0
   MOV      r0, CMD_STATUS_READY_ERR
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   SBCO     r31, CONST_PRURAM, PRU0_STATUS, 4
   RET
   
   // Measure drive RPM, time between index pulses
rpm:
   MOV      r3, INDEX_TIMEOUT
   MOV      r2, 0
   SBBO     r2, r28, 0, 4    // Clear timer
rpm_wait:
   QBBS     rpm_wait2, r31, R31_INDEX_BIT  // Wait for signal high
   LBBO     r0, r28, 0, 4
   QBLT     rpm_wait, r3, r0               // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_wait2:
   QBBC     rpm_clr, r31, R31_INDEX_BIT   // Wait for falling edge
   LBBO     r0, r28, 0, 4
   QBLT     rpm_wait2, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_clr:
   SBBO     r2, r28, 0, 4                 // Clear timer
rpm_wait3:
   QBBS     rpm_wait4, r31, R31_INDEX_BIT // Wait for signal high
   LBBO     r0, r28, 0, 4
   QBLT     rpm_wait3, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_wait4:
   QBBC     rpm_done, r31, R31_INDEX_BIT  // Wait for falling edge
   LBBO     r0, r28, 0, 4
   QBLT     rpm_wait4, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_done:
   LBBO     r2, r28, 0, 4  
   SBCO     r2, CONST_PRURAM, PRU0_CMD_DATA, 4 // Store rotation time
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   JMP      wait_cmd
rpm_timeout:
   MOV      r0, CMD_STATUS_INDEX_TIMEOUT
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 // Error
   JMP      wait_cmd
   

EXIT:
   MOV      r1, CMD_STATUS_OK
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   // Send notification to Host for program completion
   MOV      r31.b0, PRU0_ARM_INTERRUPT+16

   // Halt the processor
   HALT

   // Restarted here if the ARM wishes to abort an operation. Any needed
   // cleanup should be here to restore to a good state. Currently none
   .ORIGIN RESTART_ADDR
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   JMP      wait_cmd
