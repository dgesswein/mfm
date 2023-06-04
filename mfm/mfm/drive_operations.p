// Common code for talking to drives. Included by other .p files
//
// 03/22/19 DJG Added REV C support
// 08/05/18 DJG Further increased seek complete timeout for Quantum Q2040 drive recal
// 07/19/18 DJG Increased seek complete timeout for Quantum Q2040 drive recal
// 04/21/18 DJG Handle drives that go not ready during seek to track 0
//
// Copyright 2019 David Gesswein.
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


// 8 seconds, guess at slowest time to complete a seek including possible
// recalibrate.
#define SEEK_COMPLETE_TIMEOUT 1600000000
// Pulse width for buffered seek pulses
#define SEEK_PW 1000    //  5 microseconds
#define SEEK_PW_SLOW 8000    //  40 microseconds
// Delay between buffered seek pulses
#define SEEK_WAIT 6000  // 30 microseconds
// Delay between unbuffered seek pulses
#define SEEK_WAIT_SLOW 800000  // 4 milliseconds
// Time to wait for the head step direction line to settle
#define DIR_SETTLE_TIME 40 // 200 nanoseconds


// To find track zero we see if we are at track zero. If not we
// step in, wait for seek complete, then repeat
slow_seek_track0:
   // Seek in, turn off active low seek in signal driven by inverter
   CLR      r30, R30_SEEK_DIR_BIT   
   MOV      r3, DIR_SETTLE_TIME
   MOV      r2, 0
   SBBO     r2, CYCLE_CNTR, 0, 4           // Clear timer
settle0_lp:
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     settle0_lp, r3, r0

seek0_lp:
   // Signal on different pin for revision C (2) board
   LBCO     r0, CONST_PRURAM, PRU0_BOARD_REVISION, 4
   QBEQ     track_0_revc, r0, 2
   MOV      r0, GPIO0 | GPIO_DATIN          // Read bits
   LBBO     r0, r0, 0, 4
   QBBC     seek0_done, r0, GPIO0_TRACK_0_BIT   // Already at track0, stop
   JMP      seek0_next
track_0_revc:
   MOV      r0, GPIO3 | GPIO_DATIN          // Read bits
   LBBO     r0, r0, 0, 4
   QBBC     seek0_done, r0, GPIO3_TRACK_0_BIT_REVC  // Already at track0, stop

seek0_next:
   // Pulse the step bit for the correct duration
   MOV      r3, SEEK_PW_SLOW
   SET      r30, R30_STEP_BIT       // Start pulse
   MOV      r2, 0x0                 // Keep zero around
   SBBO     r2, CYCLE_CNTR, 0, 4           // Clear timer
seek0_w:
   LBBO     r0, CYCLE_CNTR, 0, 4           
   QBLT     seek0_w, r3, r0
   CLR      r30, R30_STEP_BIT       

   // Wait for seek complete
   MOV      r3, SEEK_COMPLETE_TIMEOUT
   CALL     wait_ready
      // Didn't go ready, return to cmd loop. Error set in wait_ready
   QBNE     wait_cmd, r3, 0    
   JMP      seek0_lp

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
   SBBO     r2, CYCLE_CNTR, 0, 4           // Clear timer
settle_lp:
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     settle_lp, r3, r0

   // And generate the pulses at the specified rate
seek_lp:
   SET      r30, R30_STEP_BIT       // Start acive low pulse
   SBBO     r2, CYCLE_CNTR, 0, 4           // Clear timer
seek_w1:
   LBBO     r0, CYCLE_CNTR, 0, 4           // Wait for pulse width
   QBLT     seek_w1, r8, r0
   CLR      r30, R30_STEP_BIT       

   MOV      r3, r6                  // Get Idle time between pulses
   SBBO     r2, CYCLE_CNTR, 0, 4
seek_w2:
   LBBO     r0, CYCLE_CNTR, 0, 4
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

   // We wait for the seek complete timeout for drive to be ready
check_ready_cmd:
   MOV      r3, SEEK_COMPLETE_TIMEOUT
   CALL     wait_ready
   // Not ready. Wait_ready already set error status
   QBNE     wait_cmd, r3,0             
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 
   JMP      wait_cmd

// Check for drive ready. r3 = timeout value on entry, 0 on return if ready
wait_ready:
   MOV      r2,0
   SBBO     r2, CYCLE_CNTR, 0, 4    // Clear timer
ready_loop:
   MOV      CALL_HOLD, RET_REG
   CALL     get_status
   MOV      RET_REG, CALL_HOLD
   // Ready when seek complete, ready, and drive selected are low
   QBBS     ready_timeout, r31, R31_SEEK_COMPLETE_BIT
   QBBS     ready_timeout, r31, R31_READY_BIT
   QBBS     ready_timeout, r31, R31_DRIVE_SEL
   // Kill time, NOP
   MOV      r0, r0
   MOV      r0, r0
   MOV      r0, r0
   MOV      r0, r0
   MOV      r0, r0
   MOV      r0, r0
   MOV      r0, r0
   // Verify still good. Signals don't all transition at the same time
   QBBS     ready_timeout, r31, R31_SEEK_COMPLETE_BIT
   QBBS     ready_timeout, r31, R31_READY_BIT
   QBBS     ready_timeout, r31, R31_DRIVE_SEL
   MOV      r3, 0    // We are ready, return good
   RET
ready_timeout:
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     ready_loop, r3, r0
   MOV      r0, CMD_STATUS_READY_ERR
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   MOV      CALL_HOLD, RET_REG
   CALL     get_status
   MOV      RET_REG, CALL_HOLD
   RET
   
// r0 modified
// r1 is updated status register.
get_status:
   LBCO     r0, CONST_PRURAM, PRU0_BOARD_REVISION, 4
   MOV      r1, r31
   QBNE     store_status, r0, 2
   MOV      r0, GPIO1 | GPIO_DATIN          // Read bits
   LBBO     r0, r0, 0, 4
   // If revision C board put write fault in same bit a A & B. R31 bit should
   // be 0 when read for revision C.
   QBBC     store_status, r0, GPIO1_WRITE_FAULT_BIT
   SET      r1, R31_WRITE_FAULT_BIT
store_status:
   SBCO     r1, CONST_PRURAM, PRU0_STATUS, 4
   RET
   
   // Measure drive RPM, time between index pulses
rpm:
   MOV      r3, INDEX_TIMEOUT
   MOV      r2, 0
   SBBO     r2, CYCLE_CNTR, 0, 4    // Clear timer
rpm_wait:
   QBBS     rpm_wait2, r31, R31_INDEX_BIT  // Wait for signal high
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     rpm_wait, r3, r0               // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_wait2:
   QBBC     rpm_clr, r31, R31_INDEX_BIT   // Wait for falling edge
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     rpm_wait2, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_clr:
   SBBO     r2, CYCLE_CNTR, 0, 4                 // Clear timer
rpm_wait3:
   QBBS     rpm_wait4, r31, R31_INDEX_BIT // Wait for signal high
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     rpm_wait3, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_wait4:
   QBBC     rpm_done, r31, R31_INDEX_BIT  // Wait for falling edge
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     rpm_wait4, r3, r0             // Try again if we haven't timed out
   JMP      rpm_timeout
rpm_done:
   LBBO     r2, CYCLE_CNTR, 0, 4  
   SBCO     r2, CONST_PRURAM, PRU0_CMD_DATA, 4 // Store rotation time
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   JMP      wait_cmd
rpm_timeout:
   MOV      r0, CMD_STATUS_INDEX_TIMEOUT
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4 // Error
   JMP      wait_cmd
