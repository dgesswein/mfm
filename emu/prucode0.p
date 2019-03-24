#define MEASURE_QUEUE_FULL_WRITEx
#define MEASURE_QUEUE_FULL_READx
// TODO: Allow operation of one drive in parallel with seek on other.
//
// This code is for emulating a MFM disk. The ARM puts into the
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
// CMD_START: Start emulation
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
// The write data is inserted 
// within a couple bits of the "proper" position using bit counts, not the
// cycle counter. The cycle counter is used so we can track rotation while
// not selected when we aren't generating bits.
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
// 03/22/19 DJG Added Board REV C support
// 05/19/17 DJG Changed seek so it will set cylinder to zero if greater than
//     limit at end of seek. Previous code did it on each step so would go
//     n n+1 0 1 if step pulses kept coming in the same seek. 
// 09/02/16 DJG Put in Rob Jarratt code change so when stepped past last
//     track it will go back to track 0
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
// Code will need to change if this is changed

.setcallreg r29.w0
#define RETREG r29

.origin 0
.entrypoint START

#include "prucode.hp"

#include "inc/cmd.h"

// These are registers that are used globally. Check registers defined in 
// cmd.h if looking for free registers.

   // PWM word from PRU 1 in various forms
#define PWM_WORD     r5
  
   // Cycle counter time to turn on index pulse
#define START_INDEX_TIME r11
   // Cycle counter time to turn off index pulse
#define END_INDEX_TIME r12

   // Address of cycle counter
#define CYCLE_CNTR   r17

   // r18 is TRACK_BIT defined in cmd.h

   // Register value 0
#define RZERO        r19

   // r20 is currently selected drive data memory pointer, Defined
   // as DRIVE_DATA in cmd.h
   // Value is 0 for drive 0, 4 for drive 1, and 1 if no drive is selected

   // Minimum space left in queue from/to PRU 1. Only set if 
   // MEASURE_QUEUE_FULL read/write defined
#define MIN_QUEUE_LEFT r21

   // Address and size of DDR memory region. Both not currently used
//#define DDR_SIZE     r22 
   // Bit in GPIO or R31 for currently selected drive
#define CUR_DRIVE_SEL r22
#define DDR_ADDR     r23
// r24 and r25 used by subroutines and not restored

   // 100 nanoseconds, Settle time for head/select lines after change
#define SETTLE_TIME 20
   // 250 microsecond max interval between step pulses for buffered seek
   // Pulses should be within 200 us
#define SEEK_FINISH_TIME 50000
   // Maximum low time for step is 500 us. We allow 550.
#define STEP_MAX_LOW_TIME 110000

   // Maximum time offset between bit time from PRU 1 and time counter
   //  8 microseconds in PRU clocks
#define MAX_TIME_OFFSET 8500/5

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

      // Global enable of all host interrupts
   LDI       r0, 1
   SBCO      r0, CONST_PRUSSINTC, GER_OFFSET, 2

      // Enable host interrupt 0
   MOV       r0, HOST_NUM
   SBCO      r0, CONST_PRUSSINTC, HIESR_OFFSET, 4

      // Map channel 0 to host 0
   MOV       r1, (INTC_HOSTMAP_REGS_OFFSET + HOST_NUM)
   MOV       r0, CHN_NUM
   SBCO      r0, CONST_PRUSSINTC, r1, 1

      // Map GPIO 0 interrupt to channel 0
   MOV       r1, (INTC_CHNMAP_REGS_OFFSET + GPIO0_EVT)
   MOV       r0, CHN_NUM
   SBCO      r0, CONST_PRUSSINTC, r1, 1

      // Set to positive polarity
   MOV       r1, (INTC_REGS_BASE + INTC_SYS_INT_REGS_OFFSET)
   MOV       r0, 0xFFFFFFFF
   SBBO      r0, r1, 0, 4
   SBBO      r0, r1, 4, 4

      // Set to positive edge trigger
   SBBO      RZERO, r1, 0x80, 4
   SBBO      RZERO, r1, 0x84, 4

      // Make sure the GPIO 0 system interrupt is cleared
   MOV       r0, GPIO0_EVT
   SBCO      r0, CONST_PRUSSINTC, SICR_OFFSET, 4

      // Enable GPIO 0 system interrupt
   SBCO      r0, CONST_PRUSSINTC, EISR_OFFSET, 4

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

      // Get DDR address and size.
   LBCO     DDR_ADDR, CONST_PRURAM, PRU_DDR_ADDR, 4
   //LBCO     DDR_SIZE, CONST_PRURAM, PRU_DDR_SIZE, 4


      // No drive selected
   MOV      DRIVE_DATA, 1
      // Clear various variable
   SBCO     RZERO, CONST_PRURAM, PRU0_EXIT, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_HEAD_SELECT_GLITCH_VALUE, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_HEAD_SELECT_GLITCH_COUNT, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_DRIVE0_CUR_CYL, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_DRIVE1_CUR_CYL, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_DRIVE0_LAST_ARM_CYL, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_DRIVE1_LAST_ARM_CYL, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_CUR_HEAD, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_CUR_SELECT_HEAD, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_RQUEUE_UNDERRUN, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_WQUEUE_OVERRUN, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_SEEK_TIME, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST0, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST1, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST2, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST3, 4
   SBCO     RZERO, CONST_PRURAM, PRU_TEST4, 4
   SBCO     RZERO, CONST_PRURAM, PRU0_ECAP_OVERRUN, 4
   MOV      CUR_DRIVE_SEL, 0

      // Set multiply only mode. Only needs to be done once
   MOV      r25, 0                 
   XOUT     0, r25, 1

      // Set the initial state of the various control lines. Inactive is high
      // for drive signals.
   CLR      r30, R30_SEEK_COMPLETE_BIT
   CLR      r30, R30_READY_BIT    
   CLR      r30, R30_DRIVE0_SEL  
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY)
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE0_LED) | (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   CLR      r30, R30_WRITE_FAULT_BIT
   CLR      r30, R30_INDEX_BIT
#ifdef REVC
   SET      r30, R30_TRACK_0_BIT
#else
   MOV      r0, 1 << GPIO0_TRACK_0
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
#endif
      // Turn off receiving MFM write data in so we can send data out
   SET      r30, R30_MFM0_IN_ENABLE       
   SET      r30, R30_MFM1_IN_ENABLE       

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
wait_initial_cmd:
      // Keep our time cleared. Cycle counter stops if it overflows
   SBBO     RZERO, CYCLE_CNTR, 0, 4
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4
   LBCO     r1, CONST_PRURAM, PRU0_EXIT, 4 
   QBNE     EXIT, r1, 0
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     mfm_setup, r1, CMD_START
   JMP      wait_initial_cmd

mfm_setup:
      // Enable GPIO lines to interrupt on rising and falling edge.
      // We want to know when they change.
      // Get which select line is ours and set only it to interrupt.
      // Set all the head select lines to interrupt
      // We OR our bits with current register contents to avoid interfering
      // with other users. Minor race conditions between read and write.
   MOV       r0, 0
   LBCO      r1, CONST_PRURAM, PRU0_DRIVE0_SELECT, 4
   SET       r0, r1 
   LBCO      r1, CONST_PRURAM, PRU0_DRIVE1_SELECT, 4
   QBEQ      no_drive1, r1, 0
   SET       r0, r1 
no_drive1:
   MOV       r2, GPIO_DRIVE_HEAD_LINES
   LBCO      r1, CONST_PRURAM, PRU0_DRIVE0_NUM_HEAD, 4
   LBCO      r3, CONST_PRURAM, PRU0_DRIVE1_NUM_HEAD, 4
   // Use maximum for the two drives. We don't support one drive using 
   // reduced write and the other not 
   MAX       r1, r1, r3	
   // If greater than 8 use all head select. Otherwise ignore MSB which
   // is reduced write current on earlier drives
   MOV       r3, 0
   QBLT      headok, r1, 8   // If 8 < heads branch, we need all lines
   CLR       r2, GPIO_HEAD3
   SET       r3, GPIO_HEAD3
headok:
   QBLT      headok2, r1, 4  // If 4 < heads then we only need 3 lines else 2            
   CLR       r2, GPIO_HEAD2
   SET       r3, GPIO_HEAD2
headok2:
   SBCO      r3, CONST_PRURAM, PRU0_HEAD_MASK, 4
   OR        r0, r0, r2
   MOV       r1, GPIO0 | GPIO_RISINGDETECT
   LBBO      r2, r1, 0, 4
      // Combine with what's already in the register
   OR        r2, r0, r2
   SBBO      r2, r1, 0, 4
   MOV       r1, GPIO0 | GPIO_FALLINGDETECT
   LBBO      r2, r1, 0, 4
   OR        r2, r0, r2
   SBBO      r2, r1, 0, 4
      // AND CLEAR INTERRUPT 0
   MOV       r1, GPIO0 | GPIO_IRQSTATUS_0
   SBBO      r0, r1, 0, 4
      // AND ENABLE INTERRUPT 0
   MOV       r1, GPIO0 | GPIO_IRQSTATUS_SET_0
   SBBO      r0, r1, 0, 4

// Now start emulating a drive
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   LBCO     START_INDEX_TIME, CONST_PRURAM, PRU0_START_INDEX_TIME, 4
   LBCO     END_INDEX_TIME, CONST_PRURAM, PRU0_END_INDEX_TIME, 4
      // Make sure we can't trigger glitch logic in select_head
   MOV      r1, 0x12345678
   SBCO     r1, CONST_PRURAM, PRU0_LAST_SELECT_HEAD, 4
      // Not waiting a command
   SBCO     RZERO, CONST_PRURAM, PRU0_WAITING_CMD, 4
      // Make sure we have the current head select etc.
#ifdef REVC
   CALL     get_select_head
   JMP      select
#else
   JMP      select_head
#endif

wait_cmd:
      // Let select_head & step know we are waiting for a command
   MOV      r0, 1
   SBCO     r0, CONST_PRURAM, PRU0_WAITING_CMD, 4
      // If needed handle select or head line change interrupt?
#ifdef REVC
   QBBS     select, r31, CUR_DRIVE_SEL
   QBBS     handle_head, r31, 30        
#else
   QBBS     select_head, r31, 30        
#endif
      // Got a step pulse? Only unbuffered/ST506 should do this.
   QBBC     step, r31, R31_STEP_BIT     // Doing a seek?
      // Check for time wrap and if index pulse needs updating
   CALL     check_rotation             
   CALL     set_index
      // Get command and branch to correct routine. 
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBNE     wait_cmd, r1, CMD_START
   CALL     do_cmd_start
      // If the cylinder hasn't changed start sending MFM data otherwise
      // tell the arm to fetch us the correct data
   LBBO     r0, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_LAST_ARM_CYL, 4
   QBEQ     next_mfm, r0, r3
   CALL     send_arm_cyl
   JMP      wait_cmd

do_cmd_start:
   SBCO     RZERO, CONST_PRURAM, PRU0_WAITING_CMD, 4
      // This measures time from when we signaled the ARM we needed a different
      // cylinder to when we got it. For performance monitoring.
   LBCO     r0, CONST_IEP, IEP_COUNT, 4
   SBCO     r0, CONST_PRURAM, PRU0_SEEK_TIME, 4
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   RET

next_mfm:
      // Turn on signals (active low) driven by inverter. These are turned 
      // off if drive not selected which then goes here when selected again
   CALL     set_track0
   SET      r30, R30_READY_BIT    
   SET      r30, R30_SEEK_COMPLETE_BIT 
next_mfmb:
#ifdef REVC
   QBBS     select, r31, CUR_DRIVE_SEL
   QBBS     handle_head, r31, 30        
#else
   QBBS     select_head, r31, 30        
#endif
   QBBC     step, r31, R31_STEP_BIT  // Got step pulse?
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
wait_bit_set:
   XIN      10, PRU1_BUF_STATE, 4              
   QBBC     step, r31, R31_STEP_BIT  // Got step pulse?
   QBNE     wait_bit_set, PRU1_STATE, STATE_READ_BIT_SET

   XIN      10, TRACK_BIT, 4
SBCO TRACK_BIT, CONST_PRURAM, 0xf8, 4

   MOV      r29, TRACK_BIT
   LBCO     r28, CONST_PRURAM, PRU0_BIT_PRU_CLOCKS, 4 // data-rate dependent
   MOV      r28, r28               // NOP, multiply has one cycle latency
   XIN      0, r26, 8              // Get multiply results (27,26=28*29)
                                   // r26 = TRACK_BIT converted to clocks
   LBBO     r1, CYCLE_CNTR, 0, 4   // Get current time
   SUB      r0, r26, r1   // Offset between current time and bit time
   QBBC     nowrap, r0, 31
   LBCO     r2, CONST_PRURAM, PRU0_ROTATION_TIME, 4
   ADD      r0, r0, r2
// DUP, not needed
   MOV      r2, MAX_TIME_OFFSET
   QBLT     wait_wrap, r2, r0
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
   HALT
wait_wrap:
   QBBC     step, r31, R31_STEP_BIT  // Got step pulse?
   CALL     check_rotation
   CALL     set_index
      // Loop until CYCLE counter wraps (less than previous read in r1)
   LBBO     r2, CYCLE_CNTR, 0, 4   // Get current time
   QBGE     wait_wrap, r1, r2
nowrap: 
   MOV      r2, MAX_TIME_OFFSET
   QBLT     no_time_err, r2, r0
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
   HALT
no_time_err:
   QBBC     step, r31, R31_STEP_BIT  // Got step pulse?
   CALL     check_rotation
   CALL     set_index
      // Loop until CYCLE counter is greater than data start time (r26)
   LBBO     r2, CYCLE_CNTR, 0, 4   // Get current time
   QBGE     no_time_err, r2, r26

      // Verify queue filled
   XIN      10, PRU1_BUF_STATE, 4              
   QBEQ     filled, PRU1_STATE, STATE_READ_FILLED
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
   HALT
filled:

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
   QBEQ     end_track, PWM_WORD, 0         // Time 0 marks end of data
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
   QBBC     write, r31, R31_WRITE_GATE  // If write gate active handle it
#ifdef REVC
   QBBS     select, r31, CUR_DRIVE_SEL
   QBBS     handle_head, r31, 30        
#else
   QBBS     select_head, r31, 30        
#endif
   QBBC     step, r31, R31_STEP_BIT     // Doing a seek?
   CALL     set_index                   // Keep index pulse updated
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
end_track:
      // Starting a new track at the beginning. Reset time and counters to
      // beginning.
   MOV      TRACK_BIT, 0
 LBBO     r1, CYCLE_CNTR, 0, 4
SBCO r1, CONST_PRURAM, 0xf0, 4
 MOV     r2, 165000*20
 QBLE     clrok, r1, r2
halt
clrok:
#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
   SBBO     r24, r25, 0, 4
#endif
   SBBO     RZERO, CYCLE_CNTR, 0, 4     // Set time to beginning of track
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4  // Prevent overflow and halt
#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_CLEARDATAOUT
   SBBO     r24, r25, 0, 4
#endif
   LBCO     r1, CONST_PRURAM, PRU0_EXIT, 4 
   QBEQ     read, r1, 0
   JMP      EXIT

#ifdef REVC
select:
      // Stop PRU 1. We will check stopped later
   MOV      PRU0_STATE, STATE_READ_DONE
   XOUT     10, PRU0_BUF_STATE, 4
      // Special case of step will restart here when it started
      // a step but didn't actually need to.
step_restart:
      // If we we aren't selected anymore handle it
   LBCO     CUR_DRIVE_SEL, CONST_PRURAM, PRU0_DRIVE0_SELECT, 4
   QBEQ     selected0, CUR_DRIVE_SEL, 0      // If zero always selected (radial select)
   QBBC     selected0, r31, CUR_DRIVE_SEL    // Branch if matches drive 0 select
   LBCO     CUR_DRIVE_SEL, CONST_PRURAM, PRU0_DRIVE1_SELECT, 4
   QBEQ     notsel, CUR_DRIVE_SEL, 0          // If zero no second drive
   QBBS     notsel, r31, CUR_DRIVE_SEL

      // If this drive is currently selected we don't have to do anything
   QBEQ     selected, DRIVE_DATA, DRIVE_DATA_BYTES
   MOV      DRIVE_DATA, DRIVE_DATA_BYTES          // Offset of drive 1 data
   SET      r30, R30_READY_BIT    
   SET      r30, R30_SEEK_COMPLETE_BIT 
     // Cylinder of currently selected drive
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   QBEQ     track0a, r3, 0
   CLR      r30, R30_TRACK_0_BIT
   JMP      finish1
track0a:
   SET      r30, R30_TRACK_0_BIT
finish1:
   // Turn on drive 1 select and LED and off drive 0
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY) | (1 << GPIO0_DRIVE0_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   CLR      r30, R30_DRIVE0_SEL  
   JMP      selected

selected_always:
   // Point to a bit that will always be zero to prevent calling
   // select
   MOV      CUR_DRIVE_SEL, 20
selected0:
      // If this drive is currently selected we don't have to do anything
   QBEQ     selected, DRIVE_DATA, 0
   MOV      DRIVE_DATA, 0
   SET      r30, R30_READY_BIT    
   SET      r30, R30_SEEK_COMPLETE_BIT 
     // Cylinder of currently selected drive
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   QBEQ     track0b, r3, 0
   CLR      r30, R30_TRACK_0_BIT
   JMP      finish2
track0b:
   SET      r30, R30_TRACK_0_BIT
finish2:
   // Turn on drive 0 select and LED and off drive 1
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY) | (1 << GPIO0_DRIVE0_LED)
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   SET      r30, R30_DRIVE0_SEL  
selected:
restart_lp:
   XIN      10, PRU1_BUF_STATE, 4
   QBNE     selected, PRU1_STATE, STATE_READ_DONE
   XOUT     10, DRIVE_DATA, 4       // Send currently selected drive offset.
      // We are selected so turn on selected signal
      // If we are not waiting for a command start outputting MFM data else
      // return to waiting for a command
   LBCO     r0, CONST_PRURAM, PRU0_WAITING_CMD, 4
   QBEQ     next_mfmb, r0, 0
   JMP      wait_cmd


      // We aren't selected, turn off signals and wait for select again
notsel:
      // Indicate no drive selected
   MOV      DRIVE_DATA, 1
      // Turn off all the control signals since we aren't selected
   CLR      r30, R30_SEEK_COMPLETE_BIT
   CLR      r30, R30_READY_BIT    
   CLR      r30, R30_DRIVE0_SEL  
   CLR      r30, R30_INDEX_BIT
   CLR      r30, R30_TRACK_0_BIT
   MOV      r0, (1 << GPIO0_DRIVE0_LED) | (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY)
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   SBCO     RZERO, CONST_ECAP, CAP2, 4  // Turn off data
   MOV      CUR_DRIVE_SEL, 0

waitsel:
      // Check if we should exit
   LBCO     r1, CONST_PRURAM, PRU0_EXIT, 4 
   QBNE     EXIT, r1, 0
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     handle_start, r1, CMD_START  // And start, update seek time
      // Keep track of drive rotation while waiting
   CALL     check_rotation             
      // Are we selected?
   QBBC     select, r31, R31_SEL1_BIT
   QBBC     select, r31, R31_SEL2_BIT
   JMP      waitsel

handle_head:
      // Stop PRU 1. We will check stopped later
   MOV      PRU0_STATE, STATE_READ_DONE
   XOUT     10, PRU0_BUF_STATE, 4
      // Clear GPIO 0 interrupt before reading value so if it changes we will
      // get a new interrupt and not miss a change
   MOV      r1, GPIO0 | GPIO_IRQSTATUS_0
      // Clear all the lines we use
   MOV      r4, GPIO_DRIVE_HEAD_LINES
   SBBO     r4, r1, 0, 4
      // This read is needed to prevent extra interrupts. Since
      // writes are posted the actual register write may happen after
      // we clear the flag in int controller below unless we read it
      // back to make sure the write is done. Is glitch logic really
      // needed? Replaced by getting head
   //LBBO     r4, r1, 0, 4               
      // Copy current value and get new head and select
   CALL     get_select_head

      // Clear interrupt status flag in interrupt controller
   MOV      r0, GPIO0_EVT
   SBCO     r0, CONST_PRUSSINTC, SICR_OFFSET, 4

   LBCO     r1, CONST_PRURAM, PRU0_LAST_SELECT_HEAD, 4
   SBCO     r24, CONST_PRURAM, PRU0_LAST_SELECT_HEAD, 4 // Update
      // If they are the same report a glitch for debugging
   QBEQ     glitch, r24, r1
   JMP      selected

#else

      // Handle GPIO interrupt from head or select lines changing
      // This routine needs to be under 1us from head change to
      // reaching next_mfm where we check for step pulses to prevent
      // loosing pulses. We currently are around .7 us
      // Must set valid head and select before leaving routine
select_head:
      // Stop PRU 1. We will check stopped later
   MOV      PRU0_STATE, STATE_READ_DONE
   XOUT     10, PRU0_BUF_STATE, 4
// Removed to try to speed up select. Doesn't seem to be needed.
//      // Wait a little bit for the lines to settle
//   LBCO     r1, CONST_IEP, IEP_COUNT, 4     // Get time
//   ADD      r1, r1, SETTLE_TIME             // How long to wait
//settle_lp:
//   LBCO     r4, CONST_IEP, IEP_COUNT, 4     // Get time
//   QBLT     settle_lp, r1, r4               // Did we reach settle time, no
      // Clear GPIO 0 interrupt before reading value so if it changes we will
      // get a new interrupt and not miss a change
   MOV      r1, GPIO0 | GPIO_IRQSTATUS_0
      // Save which line caused interrupt. Only enable for debugging
      // since slow
   //LBBO     r4, r1, 0, 4
   //SBCO     r4, CONST_PRURAM, PRU_TEST4, 4
      // Clear all the lines we use
   MOV      r4, GPIO_DRIVE_SELECT_HEAD_LINES
   SBBO     r4, r1, 0, 4
      // This read is needed to prevent extra interrupts. Since
      // writes are posted the actual register write may happen after
      // we clear the flag in int controller below unless we read it
      // back to make sure the write is done. Is glitch logic really
      // needed? Replaced by getting head
   //LBBO     r4, r1, 0, 4               
      // Copy current value and get new head and select
   CALL     get_select_head

      // Clear interrupt status flag in interrupt controller
   MOV      r0, GPIO0_EVT
   SBCO     r0, CONST_PRUSSINTC, SICR_OFFSET, 4

   LBCO     r1, CONST_PRURAM, PRU0_LAST_SELECT_HEAD, 4
   SBCO     r24, CONST_PRURAM, PRU0_LAST_SELECT_HEAD, 4 // Update
      // If they are the same report a glitch for debugging
   QBEQ     glitch, r24, r1

      // Glitch comes back here after handling it
restart_lp:
   XIN      10, PRU1_BUF_STATE, 4
   QBNE     restart_lp, PRU1_STATE, STATE_READ_DONE
      // Special case of step will restart here when it started
      // a step but didn't actually need to.
step_restart:
      // If we we aren't selected anymore handle it
   LBCO     CUR_DRIVE_SEL, CONST_PRURAM, PRU0_DRIVE0_SELECT, 4
   QBEQ     selected0, CUR_DRIVE_SEL, 0  // If zero always selected (radial select)
   QBBC     selected0, r24, CUR_DRIVE_SEL // Branch if matches drive 0 select
   LBCO     CUR_DRIVE_SEL, CONST_PRURAM, PRU0_DRIVE1_SELECT, 4
   QBEQ     notsel, CUR_DRIVE_SEL, 0       // If zero no second drive
   QBBS     notsel, r24, CUR_DRIVE_SEL
      // If this drive is currently selected we don't have to do anything
   QBEQ     selected, DRIVE_DATA, DRIVE_DATA_BYTES
   MOV      DRIVE_DATA, DRIVE_DATA_BYTES          // Offset of drive 1 data
   SET      r30, R30_READY_BIT    
   SET      r30, R30_SEEK_COMPLETE_BIT 
     // Cylinder of currently selected drive
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   // Turn on drive 1 select and LED and off drive 0
   MOV      r0, (1 << GPIO0_DRIVE1_LED)
   QBEQ     track0a, r3, 0
   SET      r0, GPIO0_TRACK_0
track0a:
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY) | (1 << GPIO0_DRIVE0_LED)
   QBNE     not_track0a, r3, 0
   SET      r0, GPIO0_TRACK_0
not_track0a:
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   CLR      r30, R30_DRIVE0_SEL  
   JMP      selected
selected0:
      // If this drive is currently selected we don't have to do anything
   QBEQ     selected, DRIVE_DATA, 0
   MOV      DRIVE_DATA, 0
   SET      r30, R30_READY_BIT    
   SET      r30, R30_SEEK_COMPLETE_BIT 
     // Cylinder of currently selected drive
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   // Turn on drive 0 select and LED and off drive 1
   MOV      r0, (1 << GPIO0_DRIVE1_SEL_RECOVERY) | (1 << GPIO0_DRIVE0_LED)
   QBEQ     track0b, r3, 0
   SET      r0, GPIO0_TRACK_0
track0b:
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_DRIVE1_LED)
   QBNE     not_track0b, r3, 0
   SET      r0, GPIO0_TRACK_0
not_track0b:
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   SET      r30, R30_DRIVE0_SEL  
selected:
   XOUT     10, DRIVE_DATA, 4       // Send currently selected drive offset.
      // We are selected so turn on selected signal
      // If we are not waiting for a command start outputting MFM data else
      // return to waiting for a command
   LBCO     r0, CONST_PRURAM, PRU0_WAITING_CMD, 4
   QBEQ     next_mfmb, r0, 0
   JMP      wait_cmd // We aren't selected, turn off signals and wait for select again
notsel:
      // Indicate no drive selected
   MOV      DRIVE_DATA, 1
      // Turn off all the control signals since we aren't selected
   CLR      r30, R30_SEEK_COMPLETE_BIT
   CLR      r30, R30_READY_BIT    
   CLR      r30, R30_DRIVE0_SEL  
   CLR      r30, R30_INDEX_BIT
   MOV      r0, (1 << GPIO0_DRIVE0_LED) | (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r0, (1 << GPIO0_TRACK_0) | (1 << GPIO0_DRIVE1_SEL_RECOVERY)
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   SBCO     RZERO, CONST_ECAP, CAP2, 4  // Turn off data
   MOV      CUR_DRIVE_SEL, 0

waitsel:
      // Check if we should exit
   LBCO     r1, CONST_PRURAM, PRU0_EXIT, 4 
   QBNE     EXIT, r1, 0
   LBCO     r1, CONST_PRURAM, PRU0_CMD, 4 
   QBEQ     handle_start, r1, CMD_START  // And start, update seek time
      // Keep track of drive rotation while waiting
   CALL     check_rotation             
   QBBS     select_head, r31, 30        // Select or head line change interrupt?
   JMP      waitsel

#endif


glitch:
   SBCO     r24, CONST_PRURAM, PRU0_HEAD_SELECT_GLITCH_VALUE, 4
   LBCO     r1, CONST_PRURAM, PRU0_HEAD_SELECT_GLITCH_COUNT, 4
   ADD      r1, r1, 1
   SBCO     r1, CONST_PRURAM, PRU0_HEAD_SELECT_GLITCH_COUNT, 4
   JMP      restart_lp

handle_start:
   CALL     do_cmd_start
      // For unbuffered seeks if after we started the first seek we get
      // more step pulses and then the drive is deselected this logic will
      // fetch the correct cylinder. We check both drives in case the
      // other drive was selected for a short time.
   MOV      DRIVE_DATA, 0
   LBBO     r0, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_LAST_ARM_CYL, 4
   QBNE     handle_cyl_change, r0, r3
   MOV      DRIVE_DATA, DRIVE_DATA_BYTES
   LBBO     r0, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   LBBO     r3, DRIVE_DATA, PRU0_DRIVE0_LAST_ARM_CYL, 4
   QBNE     handle_cyl_change, r0, r3
   MOV      DRIVE_DATA, 1    // Mark as not selected to reset DRIVE_DATA
   JMP      waitsel
handle_cyl_change: 
   CALL     send_arm_cyl
   MOV      DRIVE_DATA, 1    // Mark as not selected to reset DRIVE_DATA
   JMP      waitsel

      // Handle seeks. We count the number of step pulses that are within
      // SEEK_FINISH_TIME and add/subtract from current cylinder based on
      // direction signal. After we have final cylinder we tell the ARM to
      // get us the cylinder data. 
step:
   SBCO     RZERO, CONST_ECAP, CAP2, 4        // Turn off data
   CLR      r30, R30_SEEK_COMPLETE_BIT        // Indicate seek in progress

      // Get current cylinder for selected drive
   LBBO     r1, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   QBBS     step_out, R31, R31_SEEK_DIR_BIT   // Which direction
      // Direction is in increment cyl. Over limit will be handled
      // when seek done.
   ADD      r1, r1, 1
   JMP      step_done
step_out:
   QBEQ     step_done, r1, 0                // At limit, ignore
   SUB      r1, r1, 1
step_done:
   MOV      r3, r1                          // Save for later
      // Tell other PRU read done and wait for other PRU to acknowledge
   MOV      PRU0_STATE, STATE_READ_DONE
   XOUT     10, PRU0_BUF_STATE, 4
step_done_lp:
   XIN      10, PRU1_BUF_STATE, 4
   QBNE     step_done_lp, PRU1_STATE, STATE_READ_DONE   
      // Reset timer for step pulse timeout
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4
      // Wait this long for step to go inactive
   MOV      r1, STEP_MAX_LOW_TIME
waitstephigh:
      // Keep our time and index signal updated while waiting
   CALL     check_rotation             
   CALL     set_index
   QBBS     stephigh, r31, R31_STEP_BIT  
   LBCO     r0, CONST_IEP, IEP_COUNT, 4
   QBLT     waitstephigh, r1, r0
      // The Northstar controller waits for drive ready with step low.
      // This is ok for ST506 which don't indicate seek in progress until
      // step goes inactive. Later drives and this code set seek in progress 
      // when step goes active. 
      // If step is active greater than the limit we set seek in
      // progress inactive and wait for step to go high to restart operation,
      // or drive to not be selected. No seek is performed in these cases. It
      // probably would be better to output data but that is difficult with
      // current logic.
   SET      r30, R30_SEEK_COMPLETE_BIT        // Indicate seek not in progress
waitstephigh2:
      // Keep our time and index signal updated while waiting
   CALL     check_rotation             
   CALL     set_index
#ifdef REVC
   QBBS     select, r31, CUR_DRIVE_SEL
   QBBS     handle_head, r31, 30        
#else
   QBBS     select_head, r31, 30       // Select or head line change interrupt? 
#endif
   QBBC     waitstephigh2, r31, R31_STEP_BIT  
      // Check we didn't step past end of disk. We shouldn't have stepped
      // at all here but check just in case the controller does weird things
   CALL     limit_cyl
      // Update track zero signal in case it changed
   CALL     set_track0
      // Restart outputting with data we already have.
   JMP      step_restart
stephigh:
      // Good seek, update cylinder
   SBBO     r3, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
      // Reset timer for step pulse timeout
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4
      // Wait this long for more pulse for buffered seek
   MOV      r1, SEEK_FINISH_TIME
seek_lp:
   QBBC     step, r31, R31_STEP_BIT  // Got another pulse
   CALL     check_rotation             
   CALL     set_index
#ifdef REVC
      // If we are no longer selected finish seek, bit low when selected
   QBBS     seek_wait_done, r31, CUR_DRIVE_SEL
#else
   CALL     get_select_head
      // If we are no longer selected finish seek, bit low when selected
   QBBS     seek_wait_done, r24, CUR_DRIVE_SEL
#endif
   LBCO     r0, CONST_IEP, IEP_COUNT, 4
   QBLT     seek_lp, r1, r0

seek_wait_done:
      // Check we didn't step past end of disk
   CALL     limit_cyl
      // Update track zero signal in case it changed
   CALL     set_track0
      // If we were waiting for a command don't send interrupt, go back
      // to waiting for command
   LBCO     r0, CONST_PRURAM, PRU0_WAITING_CMD, 4
   QBNE     wait_cmd, r0, 0
      // Update data and send interrupt to ARM
   CALL     send_arm_cyl
   JMP      wait_cmd

limit_cyl:
      // One common behavior when trying to step past the last track is for
      // the drive to recalibrate back to track 0. The other behavior is
      // to continue stepping until it hits the mechanical stop. This
      // implements going to track 0 which Vaxstation 2000 needs.
      // Limit cylinder value to number of cyl-1
   LBBO     r24, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   LBBO     r25, DRIVE_DATA, PRU0_DRIVE0_NUM_CYL, 4
   QBLT     cyl_ok, r25, r24            
   LDI      r24, 0          // At limit, go back to track 0
   SBBO     r24, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
cyl_ok:
   RET

send_arm_cyl:
      // Save cylinder we sent to ARN so we can detect if more seeks done
      // while waiting.
   LBBO     r25, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   SBBO     r25, DRIVE_DATA, PRU0_DRIVE0_LAST_ARM_CYL, 4
      // Send notification to Host to read next cylinder
   MOV       r31.b0, PRU0_ARM_INTERRUPT+16

      // Reset timer for seek time measurement
   SBCO     RZERO, CONST_IEP, IEP_COUNT, 4
   RET
   
      // Set/clear index signal based on rotation time
set_index:
   LBBO     r25, CYCLE_CNTR, 0, 4
      // Handle both cases, normal start and end are both before time
      // wrap and other where index starts before time wraps back to 0 and
      // ends after
   QBGT     indnorm, START_INDEX_TIME, END_INDEX_TIME
      // If greater than start or less than end then bit should be set
   QBGT     setind, START_INDEX_TIME, r25
   QBLT     setind, END_INDEX_TIME, r25
   CLR      r30, R30_INDEX_BIT        // Turn off index
   RET
indnorm:
   QBLT     clrind, START_INDEX_TIME, r25
   QBGT     clrind, END_INDEX_TIME, r25
setind:
#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
#endif

   SET      r30, R30_INDEX_BIT        // Turn on index

#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_CLEARDATAOUT
   SBBO     r24, r25, 0, 4
#endif

   RET
clrind:
   CLR      r30, R30_INDEX_BIT        // Turn off index
   RET

      // Reset timer if we passed end of track time
check_rotation:
   LBCO     r24, CONST_PRURAM, PRU0_ROTATION_TIME, 4
   LBBO     r25, CYCLE_CNTR, 0, 4     // Get time
   QBGT     clr_time, r24, r25        // If at disk rotation time reset time
   RET
clr_time:
#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
#endif

   SBBO     RZERO, CYCLE_CNTR, 0, 4   // Clear time

#ifdef ignore
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_CLEARDATAOUT
   SBBO     r24, r25, 0, 4
#endif
   RET

      // Set track 0 signal based on current cylinder
#ifdef REVC
set_track0:
   LBBO     r25, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   QBEQ     track0, r25, 0
   CLR      r30, R30_TRACK_0_BIT
   RET
track0:
   SET      r30, R30_TRACK_0_BIT
   RET
#else
set_track0:
   MOV      r24, 1 << GPIO0_TRACK_0
      // Cylinder of currently selected drive
   LBBO     r25, DRIVE_DATA, PRU0_DRIVE0_CUR_CYL, 4
   QBEQ     track0, r25, 0
   MOV      r25, GPIO0 | GPIO_CLEARDATAOUT
   JMP      settrack0
track0:
   MOV      r25, GPIO0 | GPIO_SETDATAOUT
settrack0:
   SBBO     r24, r25, 0, 4
   RET
#endif

// Get select and head value
// Returns PRU0_CUR_SELECT_HEAD in r24
get_select_head:
   MOV      r25, GPIO0 | GPIO_DATIN
   LBBO     r24, r25, 0, 4                   // Read bits
   MOV      r25, GPIO_DRIVE_SELECT_HEAD_LINES
   AND      r24, r24, r25                    // Our bits
   LBCO     r25, CONST_PRURAM, PRU0_HEAD_MASK, 4
   OR       r24, r24, r25                    // Set head lines we are ignoring
      // If both write and step active things aren't correct
   QBBS     nowrite, r31, R31_WRITE_GATE 
   QBBS     nowrite, r31, R31_STEP_BIT
   SET      r24, CUR_SELECT_HEAD_WRITE_ERR
nowrite:
   SBCO     r24, CONST_PRURAM, PRU0_CUR_SELECT_HEAD, 4
   MOV      r25, GPIO_DRIVE_HEAD_LINES
   AND      r24, r24, r25                    // Our head bits
   XOR      r24, r24, r25                    // All high is head 0 so invert
   LSR      r24, r24, GPIO_HEAD0             // Move to low bits
   LBBO     r25, DRIVE_DATA, PRU0_DRIVE0_NUM_HEAD, 4
   SUB      r25, r25, 1                      // Convert to maximum value
   QBLE     storeh1, r25, r24
   MOV      r24, r25                         // Greater than max so set to max
storeh1:
   SBCO     r24, CONST_PRURAM, PRU0_CUR_HEAD, 4
   LBCO     r24, CONST_PRURAM, PRU0_CUR_SELECT_HEAD, 4
   RET

      // Capture write data from the controller.
      // We switch the ECAP from PWM mode to capture mode
write:
   XOUT     10, TRACK_BIT, 4        // Send current bit count.
SBCO TRACK_BIT, CONST_PRURAM, 0xe0, 4
LBBO     r0, CYCLE_CNTR, 0, 4     // Get time
SBCO r0, CONST_PRURAM, 0xe4, 4
   MOV      PRU0_STATE, STATE_WRITE_WAIT
   MOV      PRU0_BUF_OFFSET, 0
   XOUT     10, PRU0_BUF_STATE, 4 
wait_pru1_write:
      // Loop until PRU1 switches to ready to take write data
   XIN      10, PRU1_BUF_STATE, 4
   QBNE     wait_pru1_write, PRU1_STATE, STATE_WRITE_WAIT
   MOV      PRU0_STATE, STATE_WRITE
   XOUT     10, PRU0_BUF_STATE, 4         

      // Trigger cap1 on rising edges. Reset count on trigger to get delta time
      //Enable cap1-4 buffer registers. Divide by 1, stop on emulation suspend
      // continuous mode, wrap after 4, no rearm, counter free run, no sync
      // capture mode. Reset sequencer
   QBNE     drive1, DRIVE_DATA, 0
   CLR      r30, R30_MFM0_IN_ENABLE         // Turn on receiving write data 
   JMP      stop_pwm
drive1:
   CLR      r30, R30_MFM1_IN_ENABLE         // Turn on receiving write data 
stop_pwm:
   // Trigger positive edges only, reset ECAP CAP counter
   // This sequence of resetting ECAP and flags seems necessary to
   // get capture to start with CAP1 register which the code needs.
   SBCO     RZERO, CONST_ECAP, ECCTL1, 4    // Stop capture/pwm
   SBCO     RZERO, CONST_ECAP, TSCTR, 4     // Clear counter
   MOV      r0, 0x1e01aa         // ECCTL1 = 0x01aa & ECCTL2 = 0x1e
   MOV      r1, 0                // ECEINT and ECFLG
   MOV      r2, 0x1e             // ECCLR = 0x1e, ECFRC
   SBCO     r0, CONST_ECAP, ECCTL1, 12 // Load above registers
caploop:
   LBCO     r0, CONST_ECAP, ECFLG, 2  // Read flags
      // If all flags set we may have lost a transition
   QBNE     cnext1, r0, 0x1e          
   SBCO     r0, CONST_PRURAM, PRU0_ECAP_OVERRUN, 4
cnext1:
   QBBS     get1, r0, 1               // Branch if cap1 has data
   CALL     checkstuff
   JMP      caploop
get1:
   LBCO     r10, CONST_ECAP, CAP1, 4  // Get count
   ADD      r10, r10, 1               // Count is 1 less than delta time
      // Clear overflow flag so check for flag 0x1e works
   MOV      r0, 0x22                  
   SBCO     r0, CONST_ECAP, ECCLR, 2  // Clear flag
   CALL     sendwrite
      // Repeat above for the other 3 capture registers
chk2:
   LBCO     r0, CONST_ECAP, ECFLG, 2 
   QBNE     cnext2, r0, 0x1e
   SBCO     r0, CONST_PRURAM, PRU0_ECAP_OVERRUN, 4
cnext2:
   QBBS     get2, r0, 2             
   CALL     checkstuff
   JMP      chk2
get2:
   LBCO     r10, CONST_ECAP, CAP2, 4
   ADD      r10, r10, 1
   MOV      r0, 0x24
   SBCO     r0, CONST_ECAP, ECCLR, 2
   CALL     sendwrite
chk3:
   LBCO     r0, CONST_ECAP, ECFLG, 2
   QBNE     cnext3, r0, 0x1e
   SBCO     r0, CONST_PRURAM, PRU0_ECAP_OVERRUN, 4
cnext3:
   QBBS     get3, r0, 3
   CALL     checkstuff
   JMP      chk3
get3:
   LBCO     r10, CONST_ECAP, CAP3, 4
   ADD      r10, r10, 1
   MOV      r0, 0x28
   SBCO     r0, CONST_ECAP, ECCLR, 2
   CALL     sendwrite
chk4:
   LBCO     r0, CONST_ECAP, ECFLG, 2
   QBNE     cnext4, r0, 0x1e
   SBCO     r0, CONST_PRURAM, PRU0_ECAP_OVERRUN, 4
cnext4:
   QBBS     get4, r0, 4
   CALL     checkstuff
   JMP      chk4
get4:
   LBCO     r10, CONST_ECAP, CAP4, 4
   ADD      r10, r10, 1
   MOV      r0, 0x30
   SBCO     r0, CONST_ECAP, ECCLR, 2
   CALL     sendwrite

   CALL     check_rotation           // These are slow so only do once per loop
   CALL     set_index
      // All the checkstuff above will be skipped if all capture registers
      // used. Make sure we check once.
   CALL     checkstuff
   JMP      caploop

// Send write delta times to PRU 1
// r10 is word to write in
// *** Changes r1 ***
sendwrite:
      // Store word and update pointer
   SBCO     r10, CONST_PRUSHAREDRAM, PRU0_BUF_OFFSET, 4
   ADD      PRU0_BUF_OFFSET, PRU0_BUF_OFFSET, 4
   AND      PRU0_BUF_OFFSET, PRU0_BUF_OFFSET, SHARED_DELTAS_WRITE_MASK
   XOUT     10, PRU0_BUF_STATE, 4             // Send offset to PRU 1
   XIN      10, PRU1_BUF_STATE, 4             // Get PRU 1 offset
      // See READ version for description
#ifdef MEASURE_QUEUE_FULL_WRITE
   QBBS     chke, PRU1_BUF_STATE, 16
   SUB      r24, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET
   QBBC     notneg, r24, 31
   ADD      r24, r24, SHARED_DELTAS_WRITE_MASK
   ADD      r24, r24, 1
notneg:
   QBGE     chke, MIN_QUEUE_LEFT, r24
   MOV      MIN_QUEUE_LEFT, r24
   SBCO     MIN_QUEUE_LEFT, CONST_PRURAM, PRU_TEST3, 4
   SBCO     PRU0_BUF_STATE, CONST_PRURAM, PRU_TEST1, 4
   SBCO     PRU1_BUF_STATE, CONST_PRURAM, PRU_TEST2, 4
#else
   QBNE     notfullw, PRU0_BUF_OFFSET, PRU1_BUF_OFFSET
   LBCO     r24, CONST_PRURAM, PRU0_WQUEUE_OVERRUN, 4
   ADD      r24, r24, 1
   SBCO     r24, CONST_PRURAM, PRU0_WQUEUE_OVERRUN, 4
   SBCO     PRU0_BUF_STATE, CONST_PRURAM, PRU_TEST1, 4
   SBCO     PRU1_BUF_STATE, CONST_PRURAM, PRU_TEST2, 4
notfullw:
#endif
   RET

      // Check any stuff we need to at each delta. 
checkstuff:
      // Assume they will turn off write gate if they change select or head
   QBBS     write_done, r31, R31_WRITE_GATE  // Branch if write gate inactive
   RET
   
      // Done write, switch back to read
write_done:
// If it writes only zeros at the end we don't handle it. Halt if this occurs
LBCO     r0, CONST_ECAP, TSCTR, 4     // Get time since last edge on MFM data
MOV      r1, 2000/5                   // Check if more than 10 microseconds
QBLE     noextra, r1, r0
   MOV      r24, (1 << GPIO1_TEST)
   MOV      r25, GPIO1 | GPIO_SETDATAOUT
   SBBO     r24, r25, 0, 4
HALT
noextra:
   // Just turn off both, no need to see which was on
   SET      r30, R30_MFM0_IN_ENABLE       // Turn off receiving write data 
   SET      r30, R30_MFM1_IN_ENABLE       // Turn off receiving write data 
   MOV      r10, 0                       // Send zero to indicate end of data
   CALL     sendwrite
LBBO     r0, CYCLE_CNTR, 0, 4     // Get time
SBCO r0, CONST_PRURAM, 0xe8, 4
write_done_lp:
   XIN      10, PRU1_BUF_STATE, 4
   QBNE     write_done_lp, PRU1_STATE, STATE_WRITE_DONE
   JMP      next_mfm

      // ARM has requested us to exit
EXIT:
   CLR      r30, R30_SEEK_COMPLETE_BIT
   CLR      r30, R30_READY_BIT    
   CLR      r30, R30_DRIVE0_SEL  
   MOV      r0, 1 << GPIO0_DRIVE1_SEL_RECOVERY
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
   CLR      r30, R30_WRITE_FAULT_BIT
   CLR      r30, R30_INDEX_BIT
#ifdef REVC
   CLR      r30, R30_TRACK_0_BIT
#else
   MOV      r0, 1 << GPIO0_TRACK_0
   MOV      r1, GPIO0 | GPIO_CLEARDATAOUT
   SBBO     r0, r1, 0, 4
#endif
   MOV      r0, (1 << GPIO0_DRIVE0_LED) | (1 << GPIO0_DRIVE1_LED)
   MOV      r1, GPIO0 | GPIO_SETDATAOUT
   SBBO     r0, r1, 0, 4
   MOV      r1, CMD_STATUS_OK
   SBCO     r1, CONST_PRURAM, PRU0_CMD, 4 // Indicate command completed ok
   // Tell PRU 1 to exit. 
   MOV      PRU0_STATE, STATE_EXIT
   XOUT     10, PRU0_BUF_STATE, 4
   MOV      r0, -1          // -1 tells seek task to exit on ARM
   SBCO     r0, CONST_PRURAM, PRU0_DRIVE0_CUR_CYL, 4
   SBCO     r0, CONST_PRURAM, PRU0_DRIVE1_CUR_CYL, 4
      // Tell ARM to read CYL
   MOV      r31.b0, PRU0_ARM_INTERRUPT+16
wait_arm:
   // Wait for ARM to service the previous interrupt before we
   // send one for exiting. ARM sets DRIVE0_CUR_CYL to zero.
   LBCO     r0, CONST_PRURAM, PRU0_DRIVE0_CUR_CYL, 4
   QBEQ     wait_arm, r0.b3, 0xff
      // Send notification to Host for program completion
   MOV      r31.b0, PRU0_ARM_INTERRUPT+16
   SBCO     RZERO, CONST_ECAP, CAP2, 4  // Turn off data

      // Halt the processor
   HALT

      // Restarted here if the ARM wishes to abort an operation. Any needed
      // cleanup should be here to restore to a good state. Currently none
   .ORIGIN RESTART_ADDR
   MOV      r0, CMD_STATUS_OK
   SBCO     r0, CONST_PRURAM, PRU0_CMD, 4
   JMP      wait_cmd
