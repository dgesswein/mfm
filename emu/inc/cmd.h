// These defines are used in both the PRU assembly language and in the C
// code so they can only be simple defines.

// The commands to the PRU
#define CMD_NONE        0
#define CMD_START       1

// The command status. All commands other than CMD_READ_TRACK
// return their status when done. CMD_READ_TRACK returns
// CMD_STATUS_READ_STARTED when the read has started and CMD_STATUS_OK
// or an error status when the read is finished.
#define CMD_STATUS_OK       0x0ff
#define CMD_STATUS_READ_STARTED 0x100


// Registers used locally and XFER bank 10 to communicate with PRU0
#define PRU1_BUF_OFFSET r8.w0
#define PRU1_STATE      r8.b3
#define PRU1_BUF_STATE  r8
#define PRU0_BUF_OFFSET r9.w0
#define PRU0_STATE      r9.b3
#define PRU0_BUF_STATE  r9
#define TRACK_BIT       r18
// 0 for first drive, 4 for second to access variables that depend on selected drive
#define DRIVE_DATA      r20

#define STATE_IDLE         1
#define STATE_READ         2
#define STATE_READ_BIT_SET 3
#define STATE_READ_FILLED  4
#define STATE_READ_DONE    5
#define STATE_WRITE_WAIT   6
#define STATE_WRITE        7
#define STATE_WRITE_DONE   8
#define STATE_RESTART_READ 9
#define STATE_EXIT        10

// All these are the memory address to communicate with the PRU code

   // These are on both PRUs.
   // Physical address and size of shared DDR memory
#define PRU_DDR_ADDR                 0x00    
#define PRU_DDR_SIZE                 0x04
   // Physical address of PRU dataram
#define PRU_DATARAM_ADDR             0x08

#define PRU_TEST0                    0x0c
#define PRU_TEST1                    0x10
#define PRU_TEST2                    0x14
#define PRU_TEST3                    0x18
#define PRU_TEST4                    0x1c

   // Command location, command data, and status
#define PRU0_CMD                     0x30
#define PRU0_CMD_DATA                0x34
#define PRU0_STATUS                  0x38 
   // Count and register value if we got a GPIO interrupt without data changing
#define PRU0_HEAD_SELECT_GLITCH_VALUE 0x3c
#define PRU0_HEAD_SELECT_GLITCH_COUNT 0x40
   // Decoded head and raw bits from GPIO register. See PRU0_DRIVE#_CUR_CYL
   // for current cylinder
#define PRU0_CUR_HEAD                0x48
#define PRU0_CUR_SELECT_HEAD         0x4c
   // Time in 200 MHz clocks from ARM interrupt to data returned
#define PRU0_SEEK_TIME               0x50
   // Number of bytes in each drive# value. 
#define DRIVE_DATA_BYTES                4
   // Parameters of simulated disk
#define PRU0_DRIVE0_NUM_CYL          0x54
#define PRU0_DRIVE1_NUM_CYL          0x58
#define PRU0_DRIVE0_NUM_HEAD         0x5c
#define PRU0_DRIVE1_NUM_HEAD         0x60
   // Internal variable
#define PRU0_WAITING_CMD             0x64
   // Overrun on ECAP if non zero
#define PRU0_ECAP_OVERRUN            0x68
   // PRU 0-1 queue underrun count
#define PRU0_RQUEUE_UNDERRUN         0x6c
#define PRU0_WQUEUE_OVERRUN          0x70
   // Drive select numbers we should respond to. 0 or bit number in GPIO
   // register that is our drive select. Keep adjacent.
#define PRU0_DRIVE0_SELECT           0x74
#define PRU0_DRIVE1_SELECT           0x78
   // The select and head lines we are looking at. If < 8 cyl then only low 3
#define PRU0_HEAD_MASK               0x7c

   // Current cylinder
#define PRU0_DRIVE0_CUR_CYL          0x80
#define PRU0_DRIVE1_CUR_CYL          0x84

   // 0 normal, 1 to command PRU to exit
#define PRU0_EXIT                    0x88

// Used to convert bit-count to cycles at current rate (4 bytes)
#define PRU0_BIT_PRU_CLOCKS          0x8c

// Cycle count for PWM logic 1 at current data rate (4 bytes)
#define PRU0_DEFAULT_PULSE_WIDTH     0x90

// Start and end time for generating index pulse in PRU clocks 
#define PRU0_START_INDEX_TIME        0x94
#define PRU0_END_INDEX_TIME          0x98

// Time for a disk rotation in PRU clocks
#define PRU0_ROTATION_TIME           0x9c

// Select line mask for current drive selected
#define PRU0_CUR_DRIVE_SEL           0xa0

// Last cylinder requested from ARM
#define PRU0_DRIVE0_LAST_ARM_CYL     0xa4
#define PRU0_DRIVE1_LAST_ARM_CYL     0xa8
#define PRU0_LAST_SELECT_HEAD        0xac

   // Bits 0-n indicate track 0-n been modified in cylinder data
#define PRU1_DRIVE0_TRK_DIRTY        0x30
   // Bits 0-n indicate track 0-n been modified in cylinder data
#define PRU1_DRIVE1_TRK_DIRTY        0x34
   // Non zero indicates error in converting bits to PWM words
#define PRU1_BAD_PATTERN_COUNT       0x38
#define PRU1_DRIVE0_TRACK_HEADER_BYTES 0x50
#define PRU1_DRIVE1_TRACK_HEADER_BYTES 0x54
#define PRU1_DRIVE0_TRACK_DATA_BYTES 0x58
#define PRU1_DRIVE1_TRACK_DATA_BYTES 0x5c

// inverse bit period scaled 2^32, 2^32/bit_period 
#define PRU1_INV_BIT_PERIOD_S32      0x60

// If we don't see a transition by this number of cycles, 
// assume zero.
#define PRU1_ZERO_BIT_THRESHOLD      0x64
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
#define R30_DRIVE0_SEL 1
#define R30_SEEK_COMPLETE_BIT 2
#define R30_WRITE_FAULT_BIT 3
#define R30_READY_BIT 4
#define R30_INDEX_BIT 5
#define R30_MFM0_IN_ENABLE 15
#define R30_MFM1_IN_ENABLE 14

#define R31_WRITE_GATE 0
#define R31_SEEK_DIR_BIT 6
#define R31_STEP_BIT 7

#define REVB_DETECT_PIN  46 // GPIO 1_14

#define GPIO0_TRACK_0 30
#define GPIO0_DRIVE0_LED 14
#define GPIO0_DRIVE1_LED 15
#define GPIO0_DRIVE1_SEL_RECOVERY 31

#define GPIO1_TEST    29

#define GPIO_SELECT1  22
#define GPIO_SELECT2  23
#ifdef REVB
#define GPIO_HEAD0     8
#define GPIO_HEAD1     9
#define GPIO_HEAD2    10
#define GPIO_HEAD3    11
#define GPIO_DRIVE_SELECT_LINES (1 << GPIO_SELECT1) | (1 << GPIO_SELECT2)
#else
#define GPIO_HEAD0     2
#define GPIO_HEAD1     3
#define GPIO_HEAD2     4
#define GPIO_HEAD3     5
#define GPIO_SELECT3  26
#define GPIO_SELECT4  27
#define GPIO_DRIVE_SELECT_LINES (1 << GPIO_SELECT1) | (1 << GPIO_SELECT2) | (1 << GPIO_SELECT3) | (1 << GPIO_SELECT4)
#endif
#define CUR_SELECT_HEAD_WRITE_ERR 31

#define GPIO_DRIVE_HEAD_LINES (1 << GPIO_HEAD0) | (1 << GPIO_HEAD1) | (1 << GPIO_HEAD2) | (1 << GPIO_HEAD3)
#define GPIO_DRIVE_SELECT_HEAD_LINES (GPIO_DRIVE_HEAD_LINES | GPIO_DRIVE_SELECT_LINES)

// Address in PRU code for stopping current operation
#define RESTART_ADDR 0x400
// Offset from start of PRU0_DATARAM to the PRU control register
#define PRU_CONTROL_REG 0x22000
#define PRU_STATUS_REG  0x22004
#define PRU_DEBUG       0x22400

// TODO, these should be common 
#define CLOCK_BASE 0x44e00000
#define CM_AUTOIDEL_DPLL_DISP 0x448
#define CM_IDLEST_DPLL_DISP 0x448
#define CM_SSC_DELTAMSTEP_DPLL_DISP 0x44c
#define CM_SSC_MODFREQDIV_DPLL_DISP 0x450
#define CM_CLKSEL_DPLL_DISP 0x454
#define CM_CLKMODE_DPLL_DISP 0x498
#define CM_DIV_M2_DPLL_DISP 0x4a4
#define CLKSEL_PRU_ICSS_OCP_CLK 0x530
