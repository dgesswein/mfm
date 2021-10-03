#define MEASURE_QUEUE_FULL_WRITEx
#define MEASURE_QUEUE_FULL_READx
//
// This code is for writing a MFM disk. 
// Comments below are for emulator code where this was lifed from.
// Comments are not accurate anymore.
//
// The ARM puts into the
// shared DRAM a cylinder of MFM clock and data bits for each drive.
// PRU1 converts that data to PWM control values which this PRU writes
// to the ECAP peripheral to generate the MFM waveform. This PRU also handles
// the control signals from the disk controller as discussed below.
// 
// The MFM bits the emulator processes are the bits labled MFM encoded idn
// http://en.wikipedia.org/wiki/Modified_Frequency_Modulation
// The emulator does not know the meaning of those bits, it just generates
// the waveform they correspond to. The most general format would be just
// storing the bit timing (transition data) but that would be much larger
// and the number of transitions varies with the data written. Just the
// data bits could be stored but disk formats use intentional encoding "error"
// to mark the start of sectors which would require special handling. This
// format was choosen since it was easy to implement and should support most
// disk formats though at the expense of larger files.
//
// Writes are handled by converting the transition timing into the clock and
// data bits and overwriting the proper spot in the track data.
//
// This code` processes the following comamnds:
// CMD_WRITE_TRACK: Write current track
//
// Commands are read from PRU0_CMD and we then write to the same
// location the status of the command.
// If the command needs a single word of data or returns one it is in
// PRU0_CMD_DATA
//
// Time is in nominally 200 MHz clocks. The clock rate is adjusted by ARM to
// generate desired mfm clock and data bit rate.
//
// Head and select lines are on external GPIO0 module. We set up an interrupt
// so we can tell when they change without polling slow read of the GPIO0 pin
// register
//
// This PRU handles the Pulse Width Modulation (PWM) generation of 
// read data to the disk controller using PWM data words from PRU 1. It also
// handles capturing the controller write data as delta time between pulses and
// sends as delta transition time words to PRU 1. 
// This PRU also handles the head and drive select lines. Head select is 
// handled by stopping the MFM output data, and telling PRU 1 to start 
// generating data from the new head. Select is similar, when we are 
// selected we tell PRU 1 to start generating data and we turn off all 
// control signals when select goes away.
// 
// We also handle seeks. We count the number of step pulses and update
// the cylinder number. After the steps are done we interrupt the ARM so
// it can write out the current buffer if it's dirty and fetch the new
// cylinder. It sends START command after the data is updated.
// 
// TODO: The code has been sigificantly changed to use DMA to allow more
// accurate timing of the data. Some of the comments are out of date.
//
//
// Don't trust the following comments.
//
// Synchronization between the two PRU's is via XFER bank 10.
// r8.w0 PRU1_BUF_OFFSET
//    Transfers from PRU 1 the offset in the shared memory of the word it
//    has last written or read. Set to 0 at the start of a track. 
//    Entire register is PRU1_BUF_STATE
// r8.b3 PRU1_STATE
//    Transfers from PRU 1 the current operation state machine state. Used
//    to synchronize operation of the two PRU's.
// r9.w0 PRU0_BUF_OFFSET
//    Transfers from PRU 0 the offset in the shared memory of the word it
//    has last written or read. Set to 0 at the start of a track. 
//    Entire register is PRU0_BUF_STATE
// r9.b3 PRU0_STATE
//    Transfers from PRU 0 the current operation state machine state. Used
//    to synchronize operation of the two PRU's.
// r18 TRACK_BIT
//    Transfers from PRU 1 the bit count of the first PWM word written to
//    the shared memory. Transfers from PRU 0 at start of write the bit time
//    write went active to determine where PRU 1 updates track bits.
//
// Summary of state/operation transition. See code for full details of
// operation
//
// TODO: Redo this as a diagram. This isn't understandable.
// 0: Initialize PRU0_STATE(STATE_IDLE)
// 0: Wait for go command
// 0: 0track_loop:
// 0: Write PRU0_BUF_OFFSET, PRU0_STATE(STATE_READ)
// 1: 1track_loop:
// 1: Write PRU1_STATE(STATE_READ_DONE)
// 1: Wait PRU0_STATE(STATE_READ)
// 0: Wait PRU1_STATE(STATE_READ_FILLED)
// 1: Write TRACK_BIT, PRU1_STATE(STATE_READ)
// 1: 1PWM_word_loop:
// 1: Convert track word and write to shared memory and update PRU1_BUF_OFFSET
// 1: If buffer full send PRU1_STATE(STATE_READ_FILLED)
// 0: 0PWM_word_loop:
// 0: Read PWM words, update PRU0_BUF_OFFSET and send to PWM hardware
// 0: If write_gate goes active goto 0write
// 0: If seek or head change goto 0state_change
// 1: If PRU0_STATE != STATE_READ goto 1handle_state_change
// 1: If not end of data goto 1PWM_word_loop:
// 1: Write PRU word of zero (setting STATE_READ_FILLED)
// 0: If PWM word 0 not zero goto 0PWM_word_loop
// 0: Write PRU0_STATE(STATE_READ_DONE)
// 1: Wait PRU0_STATE(STATE_READ_DONE)
// 1: Write PRU1_STATE(STATE_READ_DONE)
// 1: goto 1track_loop:
// 0: Wait PRU1_STATE(STATE_READ_DONE)
// 0: Goto 0track_loop
//
// 0: 0write:
// 0: Write TRACK_BIT, PRU0_STATE(STATE_WRITE_WAIT)
// 1: 1handle_state_change:
// 1: If PRU0_STATE(STATE_READ_DONE) goto 1restart_read
// 1: IF PRU0_STATE not STATE_WRITE_WAIT halt
// 1: Read TRACK_BIT, set PRU1_STATE(STATE_WRITE_WAIT)
// 0: Wait PRU1_STATE(STATE_WRITE_WAIT)
// 0: 0write_word_loop:
// 0: Read ecap DELTA time, write to shared memory & update PRU0_BUF_OFFSET
// 0: If write not inactive goto 0write_word_loop:
// 0: goto 0write_done
// 1: 1write_word_loop:
// 1: If word available read it and update PRU1_BUF_OFFSET
// 1: If word zero goto 1write_done:
// 1: Decode and update track bits
// 1: goto 1write_word_loop:
// 0: 0write_done:
// 0: write zero delta word
// 0: wait for PRU1_STATE(STATE_WRITE_DONE)
// 0: goto 0track_loop
// 1: 1write_done:
// 1: Write PRU1_STATE(STATE_WRITE_DONE)
// 1: Wait PRU0_STATE(STATE_READ)
// 1: Goto special start track from end location of write
//
// 0: 0state_change: (handling seek/head change not show)
// 0: Write PRU0_STATE(STATE_READ_DONE)
// 0: Wait PRU1_STATE(STATE_READ_DONE)
// 0: goto 0track_loop
// 1: 1restart_read:
// 1: Write PRU1_STATE(STATE_READ_DONE)
// 1: Wait PRU0_STATE(STATE_READ_DONE)
// 1: goto 1track_loop
//
// 09/08/21 DJG Changed to using command status to indicate track write done
//     instead of interrupt. Error was ignored.
// 03/20/21 DJG Fix race condition on sampling index.
// 03/07/21 DJG Continue writing to end of track if end of data seen before 
//     index. Stop write data when turning off write.
// 02/22/21 DJG Stop write when index seen to prevent overwriting beginning of
//     track on drives with > 3600 RPM
// 03/22/19 DJG Added REV C support
// 01/22/16 DJG Make sure write going inactive is checked when all
//     capture registers used. High frequency noise? casued problem.
// 01/17/16 DJG Fix select speedup state tracking
// 01/13/16 DJG Speed up select to try to make work with Symbolics 3640.
//     Speedup was insufficient but may be useful for other systems.
// 01/07/16 DJG Move picking up initial select and head until after head mask
//    calculated.
//    initialized, detect reversed J2.
// 01/06/16 DJG Fix determining head mask, make sure head and select are
//    initialized, detect reversed J2.
// 12/31/15 DJG Add additional step pulse checks to work with Symbolics
//    1 microsecond step pulse width.
// 07/12/15 DJG Added additional check for step pulse to prevent missing
//    step when it occurs right after change in head select.
// 06/29/15 DJG Handle unbuffered/ST506 step pulses while waiting for
//    ARM to send data for previous cylinder requested.
// 05/20/15 DJG Significant change to switch to DMA (on PRU1) for reading
//    from DRAM and how data is syncronized to simulated rotation time.
//    also some fixes for head select and MFM 422 chip control.
//    The code as a number of internal consistency checks where it will halt 
//    if it detects problems. Added igoring upper head selelect lines if
//    they aren't needed to address heads (Fixed Northstar). Assert seek
//    complete if step is held active too long (Fixed Northstar). Allow
//    drive to be deselected immediatly after step pulse done.
// 01/04/15 DJG Added logic to allow bit rate to be varied and the
//   index pulse shifted relative to the start of the data. The index pulse
//   shift compensates for the start_time_ns delay in starting read in mfm_read.
//   Fixed not turning track0 off when not selected. Cleared wrap flag in ECLFG
//   so transition loss will be deteced.
//   
// 09/06/14 DJG Fixed deadlock between shutting down and seeking to
//   next cylinder.
//
// Copyright 2021 David Gesswein.
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
// Code will need to change if this is changed

.setcallreg r29.w0
#define CALL_HOLD r29.w2
#define RET_REG r29.w0

.origin 0
.entrypoint START

#include "prucode.hp"

#include "inc/cmd.h"
#include "inc/cmd_write.h"

// These are registers that are used globally. Check registers defined in 
// cmd.h if looking for free registers.

   // PWM word from PRU 1 in various forms
#define PWM_WORD     r5
  
   // Cycle counter time to delay after index before starting write
#define START_TIME_CLOCKS r11

   // Address of cycle counter
#define CYCLE_CNTR   r17

   // r18 is TRACK_BIT defined in cmd.h

   // Register value 0
#define RZERO        r19

   // r20 hold for last r31 value

   // Minimum space left in queue from/to PRU 1. Only set if 
   // MEASURE_QUEUE_FULL read/write defined
#define MIN_QUEUE_LEFT r21

   // Address and size of DDR memory region. Both not currently used
#define DDR_SIZE     r22 
#define DDR_ADDR     r23
// r24 and r25 used by subroutines and not restored

// Maximum time to wait for an index pulse
#define INDEX_TIMEOUT 5000000 // 25 milliseonds

START:
   MOV      RZERO, 0
      // Enable OCP master port
   LBCO     r0, CONST_PRUCFG, 4, 4
      // Clear SYSCFG[STANDBY_INIT] to enable OCP master port
   CLR      r0, r0, 4         
   SBCO     r0, CONST_PRUCFG, 4, 4

      // Configure the programmable pointer register for PRU0 by setting 
      // c28_pointer[15:0]field to 0x0100.  This will make C28 point 
      // to 0x00010000 (PRU shared RAM).
   MOV      r0, 0x00000100
   MOV      r1, PRU0_CONTROL | CTPPR_0
   SBBO     r0, r1, 0, 4

      // Configure the programmable pointer register for PRU0 by 
      // setting c31_pointer[15:0] field to 0x0010.  This will make C31 point
      // to 0x80001000 (DDR memory).
   MOV      r0, 0x00100000
   MOV      r1, PRU0_CONTROL | CTPPR_1
   SBBO     r0, r1, 0, 4

      // Enable cycle counter. We use this for timing the track/index
      // 4 cycles to read
   MOV      CYCLE_CNTR, PRU0_CONTROL | CYCLE
   MOV      r3, PRU0_CONTROL | CONTROL    
   LBBO     r2, r3, 0, 4            // Read reg
   SET      r2, r2, 3
   SBBO     r2, r3, 0, 4            // Write reg
   SBBO     RZERO, CYCLE_CNTR, 0, 4          // Clear timer

      // Enable Industrial ethernet counter. We use this for delays etc
      // It is slow (12 cycles) to read. TODO Might be able to use compare regs
   MOV      r0, 0x011 // Enable and set increment to 1
   MOV      r1, 1     // Clear overflow
   MOV      r2, 0     // Compensation disabled
   MOV      r3, 0     // Clear count
   SBCO     r0, CONST_IEP, 0, 16

      // Turn off write
   CLR      r30, R30_WRITE_GATE
      // Change write gate signal to PRU control
   MOV      r1, AM335X_CTRL_BASE | CONF_MCASP0_ACLKX
   LBBO     r2, r1, 0, 4
   SBCO     r2, CONST_PRURAM, PRU0_MCASP0_ACLKX, 4
   MOV      r2, 0x2d   // pru output, no pullup/down, slow
//TODO, this doesn't work. It appears the register is read only
   SBBO     r2, r1, 0, 4
      // Set MFM data to be output, in enable is active low
   SET      r30, R30_MFM0_IN_ENABLE

      // Get DDR address and size.
   LBCO     DDR_ADDR, CONST_PRURAM, PRU_DDR_ADDR, 4
   LBCO     DDR_SIZE, CONST_PRURAM, PRU_DDR_SIZE, 4


      // Clear various variable
   SBCO     RZERO, CONST_PRURAM, PRU0_RQUEUE_UNDERRUN, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_WQUEUE_OVERRUN, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST0, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST1, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST2, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST3, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST4, 4

      // Set multiply only mode. Only needs to be done once
   MOV      r25, 0                 
   XOUT     0, r25, 1


   MOV      PRU0_BUF_STATE, 0
   MOV      PRU0_STATE, STATE_IDLE
   XOUT     10, PRU0_BUF_STATE, 4

   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_CLEARDATAOUT
   SBBO     r24, r25, 0, 4

#ifdef MEASURE_QUEUE_FULL_READ
   MOV      MIN_QUEUE_LEFT, (SHARED_PWM_READ_MASK+1)
   SBCO     MIN_QUEUE_LEFT, CONST_PRURAM, PRU_TEST3, 4
#endif
#ifdef MEASURE_QUEUE_FULL_WRITE
   MOV      MIN_QUEUE_LEFT, (SHARED_DELTAS_WRITE_MASK+1)
   SBCO     MIN_QUEUE_LEFT, CONST_PRURAM, PRU_TEST3, 4
#endif

   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_STATUS, 4

// Wait for initial go from the ARM before we start acting like a drive
// We need various data set up by it before we can start
wait_cmd:
      // Keep our time cleared. Cycle counter stops if it overflows
   SBBO     RZERO, CYCLE_CNTR, 0, 4
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     EXIT, r1, CMD_EXIT
   QBEQ     mfm_setup, r1, CMD_WRITE_TRACK
   QBEQ     seek, r1, CMD_SEEK_FAST
   QBEQ     slow_seek, r1, CMD_SEEK_SLOW
   QBEQ     slow_seek_track0, r1, CMD_SEEK_SLOW_TRACK0
   QBEQ     rpm, r1, CMD_RPM

   QBEQ     check_ready_cmd, r1, CMD_CHECK_READY
   JMP      wait_cmd

#include "drive_operations.p"

mfm_setup:
   LBCO     START_TIME_CLOCKS, CONST_PRURAM, PRU0_START_TIME_CLOCKS, 4
next_mfm:
      // Switch to PWM mode
   SBCO     RZERO, CONST_ECAP, ECCTL1, 4
      // Default width for a 1 at current data rate
   LBCO     r0, CONST_PRURAM, PRU0_DEFAULT_PULSE_WIDTH, 4
   SBCO     r0, CONST_ECAP, CAP1, 4
   SBCO     r0, CONST_ECAP, CAP3, 4
   SBCO     RZERO, CONST_ECAP, CAP2, 4
   SBCO     RZERO, CONST_ECAP, CAP4, 4
   SBCO     RZERO, CONST_ECAP, TSCTR, 4
      // Set APWM mode output high
   MOV      r0, 0x02100000          // low 16 is ECCTL1, upper ECCTL2
   SBCO     r0, CONST_ECAP, ECCTL1, 4

sendtrack:   
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4 // Prevent timer overflow
LBBO     r0, CYCLE_CNTR, 0, 4   // get time
SBCO r0, CONST_PRURAM, 0xf4, 4
   MOV      PRU0_STATE, STATE_READ
   MOV      PRU0_BUF_OFFSET, 0
   XOUT     10, PRU0_BUF_STATE, 4

   MOV      TRACK_BIT, 0

      // Verify queue filled
wait_fill:
   XIN      10, PRU1_BUF_STATE, 4              
   QBNE     wait_fill, PRU1_STATE, STATE_READ_FILLED

   // Now find start of track. Start is falling edge of index signal
   MOV      r3, INDEX_TIMEOUT
   MOV      r2, 0
   SBBO     r2, CYCLE_CNTR, 0, 4    // Clear timer
wait_high:
   QBBS     wait_low, r31, R31_INDEX_BIT   // Wait for signal high
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     wait_high, r3, r0          // Try again if we haven't timed out
wait_timeout:
   MOV      r1, CMD_STATUS_INDEX_TIMEOUT
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command failed
   JMP      wait_cmd
wait_low:
   QBBC     wait_start_time, r31, R31_INDEX_BIT   // Wait for falling edge
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     wait_low, r3, r0
   JMP      wait_timeout

   // For some disk formats we need to delay start of data capture so we
   // get the sector the crosses the failling index pulse. Wait for the
   // time specified.
wait_start_time:
   MOV      r2, 0
   SBBO     r2, CYCLE_CNTR, 0, 4    // Clear timer
   LBCO     r3, CONST_PRURAM, PRU0_START_TIME_CLOCKS, 4
wait_start_loop:
   LBBO     r0, CYCLE_CNTR, 0, 4
   QBLT     wait_start_loop, r3, r0 // Try again if we haven't timed out
   MOV      r20, r31                // Initialize last index state
      // Turn on write
   SET      r30, R30_WRITE_GATE

      // Send read MFM data to the controller.
      // This reads the PWM words representing the bits from PRU 1 and 
      // writes them to the PWM controller. Bits 31-28 are the number
      // of MFM bit times the word generates. Bits 27-24 should be ignored.
      // Needs to be less than 40. currently around 27 cycles
read:
   LBCO     PWM_WORD, CONST_PRUSHAREDRAM, PRU0_BUF_OFFSET, 4 // 3 cycles
      // Increment and wrap if needed 
   ADD      PRU0_BUF_OFFSET, PRU0_BUF_OFFSET, 4 
   AND      PRU0_BUF_OFFSET, PRU0_BUF_OFFSET, SHARED_PWM_READ_MASK   
   QBEQ     end_data, PWM_WORD, 0         // Time 0 marks end of data
   // If we see falling edge of index stop write
   MOV      r0, r31     // sample R31 to get consistent data for checks
   QBBS     indexhigh, r0, R31_INDEX_BIT     // Continue if high
   QBBS     end_track, r20, R31_INDEX_BIT    // Stop if last high
indexhigh:
   MOV      r20, r0
   LSR      r3, PWM_WORD, 28               // Save bit count for loadit:
   MOV      PWM_WORD.b3, 0                 // Clear count
      // Send our read offset into shared memory to PRU 1 and get its write offset
   XOUT     10, PRU0_BUF_STATE, 4
   XIN      10, PRU1_BUF_STATE, 4
      // Either calculate how much space is in queue or simple did it wrap check
#ifdef MEASURE_QUEUE_FULL_READ
      // MSB set indicates writer done so don't check
   QBBS     chke, PRU1_BUF_STATE, 16                  
      // Calculate words in queue handling queue wrap
   SUB      r0, PRU1_BUF_OFFSET, PRU0_BUF_OFFSET
   QBBC     notneg, r0, 31
   ADD      r0, r0, (SHARED_PWM_READ_MASK+1)
notneg:
   QBGE     chke, MIN_QUEUE_LEFT, r0
   MOV      MIN_QUEUE_LEFT, r0
   SBCO     MIN_QUEUE_LEFT, CONST_PRURAM, PRU_TEST3, 4
   SBCO     PRU0_BUF_STATE, CONST_PRURAM, PRU_TEST1, 4
   SBCO     PRU1_BUF_STATE, CONST_PRURAM, PRU_TEST2, 4
#else
   QBNE     notfullr, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET
   LBCO     r0, CONST_PRURAM, PRU0_RQUEUE_UNDERRUN, 4
   ADD      r0, r0, 1
   SBCO     r0, CONST_PRURAM, PRU0_RQUEUE_UNDERRUN, 4
   SBCO     PRU0_BUF_STATE, CONST_PRURAM, PRU_TEST1, 4
   SBCO     PRU1_BUF_STATE, CONST_PRURAM, PRU_TEST2, 4
notfullr:
#endif

chke:
   LBCO     r0, CONST_ECAP, ECFLG, 2    // did PWM read shadow regs? 4 cycles
   QBBS     loadit, r0, 6               // Yes, load a new one
   JMP      chke                        // Check PWM again
loadit:
   SBCO     PWM_WORD.w0, CONST_ECAP, CAP3, 2  // Update shadow period, 2 cycles
   SBCO     PWM_WORD.w2, CONST_ECAP, CAP4, 2  // Update shadow width, 2 cycles
   SBCO     r0, CONST_ECAP, ECCLR, 2          // Clear flag, 2 cycles
      // Moved here since this is when the word actually loaded into
      // hardware and to prevent write seeing an updated count that is
      // past end of data. If this increments past last bit the next
      // word we see in read should be a zero which will reset
   ADD      TRACK_BIT, TRACK_BIT, r3          // Update bit count
   JMP      read
      // Write last pattern till end of track
end_data:
   QBBC     end_track, r31, R31_INDEX_BIT     // Low, stop
   JMP      end_data
end_track:
      // Stop write
   SBCO     RZERO, CONST_ECAP, CAP2, 4  // Turn off data
   CLR      r30, R30_WRITE_GATE  
   MOV      PRU0_STATE, STATE_READ_DONE
   XOUT     10, PRU0_BUF_STATE, 4
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
wait_done:
   XIN      10, PRU1_BUF_STATE, 4              
   QBNE     wait_done, PRU1_STATE, STATE_READ_DONE
      // Starting a new track at the beginning. Reset time and counters to
      // beginning.
   MOV      TRACK_BIT, 0
   SBBO     RZERO, CYCLE_CNTR, 0, 4     // Set time to beginning of track
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4  // Prevent overflow and halt
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     EXIT, r1, CMD_EXIT
   JMP      wait_cmd
   
      // ARM has requested us to exit
EXIT:
   CLR      r30, R30_WRITE_GATE  
   SBCO     RZERO, CONST_ECAP, CAP2, 4  // Turn off data
      // Change write gate signal to GPIO control
   MOV      r1, AM335X_CTRL_BASE | CONF_MCASP0_ACLKX
   LBCO     r2, CONST_PRURAM, PRU0_MCASP0_ACLKX, 4
   SBBO     r2, r1, 0, 4

   MOV      r1, CMD_STATUS_OK
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   // Tell PRU 1 to exit. 
   MOV      PRU0_STATE, STATE_EXIT
   XOUT     10, PRU0_BUF_STATE, 4
      // Tell ARM to read CYL
   MOV      r31.b0, PRU0_ARM_INTERRUPT+16
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
