// These defines are used in both the PRU assembly language and in the C
// code so they can only be simple defines.


// Registers used locally and XFER bank 10 to communicate with PRU0
#define PRU1_BUF_OFFSET r8.w0
#define PRU1_STATE      r8.b3
#define PRU1_BUF_STATE  r8
#define PRU0_BUF_OFFSET r9.w0
#define PRU0_STATE      r9.b3
#define PRU0_BUF_STATE  r9
#define TRACK_BIT       r18

#define STATE_IDLE         1
#define STATE_READ         2
#define STATE_READ_FILLED  4
#define STATE_READ_DONE    5
#define STATE_RESTART_READ 9
#define STATE_EXIT        10

#define PRU_TEST0                    0x24
#define PRU_TEST1                    0x28
#define PRU_TEST2                    0x2c
#define PRU_TEST3                    0x30
#define PRU_TEST4                    0x34


   // Save pin setting to restore on exit
#define PRU0_MCASP0_ACLKX            0x50
   // Number of bytes in each drive# value. 
#define DRIVE_DATA_BYTES                4
   // PRU 0-1 queue underrun count
#define PRU0_RQUEUE_UNDERRUN         0x6c
#define PRU0_WQUEUE_OVERRUN          0x70

// Used to convert bit-count to cycles at current rate (4 bytes)
#define PRU0_BIT_PRU_CLOCKS          0x8c

// Cycle count for PWM logic 1 at current data rate (4 bytes)
#define PRU0_DEFAULT_PULSE_WIDTH     0x90

// Start and end time for generating index pulse in PRU clocks 
#define PRU0_START_INDEX_TIME        0x94
#define PRU0_END_INDEX_TIME          0x98

// Time for a disk rotation in PRU clocks
#define PRU0_ROTATION_TIME           0x9c

   // Non zero indicates error in converting bits to PWM words
#define PRU1_BAD_PATTERN_COUNT       0x40
#define PRU1_CUR_HEAD                0x44
#define PRU1_DRIVE0_TRACK_HEADER_BYTES 0x50
#define PRU1_DRIVE0_TRACK_DATA_BYTES 0x58

// inverse bit period scaled 2^32, 2^32/bit_period 
#define PRU1_INV_BIT_PERIOD_S32      0x60

// Nominal bit cell cycles
#define PRU1_BIT_PRU_CLOCKS          0x68
// DMA channel to use
#define PRU1_DMA_CHANNEL             0x6c
// Last DMA RAM and DDR offset, internal PRU use
#define PRU1_NEXT_DMA_RAM_OFFSET     0x70
#define PRU1_NEXT_DMA_DDR_OFFSET     0x74

#define PRU_WORD_SIZE_BYTES 4

   // Location bit lookup table stored
#define PRU1_BIT_TABLE               0x80

// Maximum size of cylinder buffer.
#define DDR_DRIVE_BUFFER_MAX_SIZE    (16*32768)
   // PRU queues are in shared memory locations 0 to mask
#define SHARED_PWM_READ_MASK  0x1f
#define SHARED_DELTAS_WRITE_MASK  0x7f

// Bits in registers for the control lines
#define R30_WRITE_GATE 0


#define GPIO1_TEST    29
