// These defines are used in both the PRU assembly language and in the C
// code so they can only be simple defines.

// The commands to the PRU
#define CMD_NONE        0
#define CMD_EXIT        1
// The commands to the PRU for mfm_read
#define CMD_READ_TRACK  2
#define CMD_RPM         3
#define CMD_SEEK_FAST   4
#define CMD_SEEK_SLOW   5
#define CMD_SEEK_SLOW_TRACK0 6
#define CMD_CHECK_READY 7
// The commands to the PRU for mfm_write
#define CMD_WRITE_TRACK 8

// The command status. All commands other than CMD_READ_TRACK
// return their status when done. CMD_READ_TRACK returns
// CMD_STATUS_READ_STARTED when the read has started and CMD_STATUS_OK
// or an error status when the read is finished.
#define CMD_STATUS_WAIT_READY 0x100
#define CMD_STATUS_OK       0x200
#define CMD_STATUS_READY_ERR 0x300
#define CMD_STATUS_INDEX_TIMEOUT 0x400
#define CMD_STATUS_SEEK_COMPLETE_ERR 0x500
#define CMD_STATUS_READ_OVERFLOW 0x600
#define CMD_STATUS_READ_OVERRUN 0x700
#define CMD_STATUS_READ_STARTED 0x800
#define CMD_STATUS_DELTA_OVERFLOW 0x900

// These are the control parameters to communicate with the PRU code
#define PRU_DDR_ADDR      0x00      // Physical address of shared DDR memory
#define PRU_DDR_SIZE      0x04      // Size of shared DDR memory
#define PRU0_WRITE_PTR    0x08	    // Delta write pointer
#define PRU0_CMD          0x0c      // Command location
#define PRU0_CMD_DATA     0x10      // Data for command
#define PRU0_STATUS       0x14      // Drive status, only valid when idle
#define PRU0_START_TIME_CLOCKS 0x18 // Delay from index until we start
                                    // capturing transitions
#define PRU_DATARAM_ADDR  0x1c      // Physical address of the PRU memory
#define PRU0_BOARD_REVISION 0x20    // 0 = A etc

#define REVB_DETECT_PIN  46 // GPIO 1_14
#define REVC_DETECT_PIN  61 // GPIO 1_29
// Rev A,B
#define GPIO0_TRACK_0_BIT    30
// Rev C
#define GPIO3_TRACK_0_BIT_REVC 17
#define GPIO3_START_PIN      (32*3)
// Where the physical lines show up in the PRU registers
#define R30_SEEK_DIR_BIT  6
#define R30_STEP_BIT      7
#define R30_MFM0_IN_ENABLE  15

#define R31_DRIVE_SEL         1
#define R31_SEEK_COMPLETE_BIT 2
// Rev A,B
#define R31_WRITE_FAULT_BIT   3
// Rev C
#define GPIO1_WRITE_FAULT_BIT 19
#define R31_READY_BIT         4
#define R31_INDEX_BIT         5

// Address in PRU code for stopping current operation
#define RESTART_ADDR 0x400
// Offset from start of PRU0_DATARAM to the PRU control register
#define PRU_CONTROL_REG 0x22000
#define PRU_STATUS_REG  0x22004
#define PRU_DEBUG       0x22400

// Registers for configuring the PRU clock
#define CLOCK_BASE 0x44e00000
#define CM_AUTOIDEL_DPLL_DISP 0x448
#define CM_IDLEST_DPLL_DISP 0x448
#define CM_SSC_DELTAMSTEP_DPLL_DISP 0x44c
#define CM_SSC_MODFREQDIV_DPLL_DISP 0x450
#define CM_CLKSEL_DPLL_DISP 0x454
#define CM_CLKMODE_DPLL_DISP 0x498
#define CM_DIV_M2_DPLL_DISP 0x4a4
#define CLKSEL_PRU_ICSS_OCP_CLK 0x530
