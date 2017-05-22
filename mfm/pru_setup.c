// Routines for interfacing with the PRU/PRUSS. Some of the routines are
// generic and others specific to communicating with the code running on
// the PRU. Only PRU 0 is currently used.
//
// Call pru_setup first to open PRU device and map memory.
// Call pru_shutdown when done to close PRU device.
// These routines are specific to the code running on the PRU.
// Call pru_exec_cmd to send a command to PRU and wait for response.
// Call pru_get_cmd_status to get the command completion status.
// Call pru_get_cmd_data to get data value set by a command.
// Call pru_restart to restart PRU from the restart address.
// Call pru_get_pc to get the program counter of the PRU. For debugging
// Call pru_write_mem to write data to memory
// Call pru_read_mem to read data from memory
// Call pru_read_word to read a 32 bit word from memory
// Call pru_write_word to write a 32 bit word to memory
//
// TODO: Use cache control to make memory transfers faster with PRU
//
// 05/19/17 DJG Add ability to dump PRU shared memory.
// 12/24/15 DJG Comment cleanup
// 11/22/15 DJG Add 15 MHz data rate support.
// 05/17/15 DJG Added routines to allow dumping of state if PRU halts
//   due to error.
// 01/04/15 DJG Added pru_set_clock to set the PRU clock to generate the
//   desired mfm clock and data bit rate.
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
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#include "msg.h"
#include "cmd.h"

#include "pru_setup.h"
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// This is memory for communicating with the PRU/PRUSS processors
// DDR is the shared region of main memory, pru_dataram# is data memory for pru #.
// pru_sharedram is the shared PRU local memory
static void *ddr_mem, *pru_dataram0, *pru_dataram1, *pru_sharedram;
static uint32_t ddr_phys_addr;
static int ddr_mem_size;
static int num_pru;
static int data_mem_type[] = {MEM_PRU0_DATA, MEM_PRU1_DATA};

// Map DDR shared memory segment into our address space and return addresses
// and size.
//
// ddrmem: Returns pointer to memory (virtual address).
// ddr_phys_addr: Returns physical address of memory.
// return: Size of region in bytes.
static int pru_allocate_ddr_memory(void **ddrmem, uint32_t *ddr_phys_addr)
{
   uint32_t ddr_offset, ddr_mem_size;
   FILE *fin;

   fin = fopen("/sys/class/uio/uio0/maps/map1/addr", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map address");
      exit(1);
   }
   fscanf(fin, "%x", ddr_phys_addr);
   fclose(fin);

   fin = fopen("/sys/class/uio/uio0/maps/map1/offset", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map offset");
      exit(1);
   }
   fscanf(fin, "%x", &ddr_offset);
   fclose(fin);

   fin = fopen("/sys/class/uio/uio0/maps/map1/size", "r");
   if (fin == NULL) {
      perror("Unable to open DDR map address");
      exit(1);
   }
   fscanf(fin, "%x", &ddr_mem_size);
   fclose(fin);
   //printf("DDR base %x offset %x size %x\n", ddr_base, ddr_offset,
   //       ddrMemSize);

   *ddrmem = prussdrv_get_virt_addr(*ddr_phys_addr + ddr_offset);

   return (ddr_mem_size);
}

// Set up PRU device and map memory
//
// num_pru_in: Number of PRU to setup, 1 or 2
// return: size of DDR memory segment in bytes.
int  pru_setup(int num_pru_in)
{
   tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
   int ret;

   // If clock was changed resetting board can get it in a bad state. Make
   // sure it is good
   pru_set_clock(10000000, PRU_SET_CLOCK_NO_HALT);

   num_pru = num_pru_in;

   /* Initialize the PRU */
   prussdrv_init();

   /* Open PRU Interrupt */
   ret = prussdrv_open(PRU_EVTOUT_0);
   if (ret) {
      msg(MSG_FATAL, "prussdrv_open open 0 failed\n");
      return -1;
   }
   if (num_pru == 2) {
      ret = prussdrv_open(PRU_EVTOUT_1);
      if (ret) {
         msg(MSG_FATAL, "prussdrv_open open 1 failed\n");
         return -1;
      }
   }

   /* Get the interrupt initialized */
   prussdrv_pruintc_init(&pruss_intc_initdata);

   /* Allocate PRU memory. */
   if (prussdrv_map_prumem(PRUSS0_PRU0_DATARAM,(void **) &pru_dataram0) == -1) {
      msg(MSG_FATAL, "prussdrv_map_prumem PRUSS0_PRU0_DATARAM map failed\n");
      return -1;
   }
#ifdef PRU_DATARAM_ADDR
   pru_write_word(MEM_PRU0_DATA, PRU_DATARAM_ADDR,  
      prussdrv_get_phys_addr((void *) pru_dataram0));
#endif

   if (prussdrv_map_prumem(PRUSS0_PRU1_DATARAM,(void **) &pru_dataram1) == -1) {
      msg(MSG_FATAL, "prussdrv_map_prumem PRUSS0_PRU1_DATARAM map failed\n");
      return -1;
   }
#ifdef PRU_DATARAM_ADDR
   pru_write_word(MEM_PRU1_DATA, PRU_DATARAM_ADDR,  
      prussdrv_get_phys_addr((void *) pru_dataram1));
#endif

   if (prussdrv_map_prumem(PRUSS0_SHARED_DATARAM,(void **) &pru_sharedram) == -1) {
      msg(MSG_FATAL, "prussdrv_map_prumem PRUSS0_SHARED_DATARAM map failed\n");
      return -1;
   }

   ddr_mem_size = pru_allocate_ddr_memory(&ddr_mem, &ddr_phys_addr);

   // Give the DDR memory region address to the PRU
   pru_write_word(MEM_PRU0_DATA, PRU_DDR_ADDR,  ddr_phys_addr);
   pru_write_word(MEM_PRU0_DATA, PRU_DDR_SIZE,  ddr_mem_size - 1);
   pru_write_word(MEM_PRU1_DATA, PRU_DDR_ADDR,  ddr_phys_addr);
   pru_write_word(MEM_PRU1_DATA, PRU_DDR_SIZE,  ddr_mem_size - 1);

   return ddr_mem_size;
}

// Wait for completion from PRU and shut down device.
void pru_shutdown() {
   // Wait until PRU0 has finished execution
   prussdrv_pru_wait_event (PRU_EVTOUT_0);
   prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
   prussdrv_pru_disable(0);
   if (num_pru == 2) {
      prussdrv_pru_wait_event (PRU_EVTOUT_1);
      prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU1_ARM_INTERRUPT);
      prussdrv_pru_disable(1);
   }

   prussdrv_exit ();
}


// This sends a command and data word to the PRU and waits for completion or,
// for reads, that the read has started. To execute a command first write the
// data value if needed then write the command to the command address. The PRU
// will execute the command and update the command address with the command
// status. The read command which we wish to overlap processing data with
// reading returns a command started status and then a command done status.
// See cmd.h for commands.
//
// cmd: Command to execute.
// data: Data value for command.
// return: Zero if no error, command status if error
int pru_exec_cmd(uint32_t cmd, uint32_t data)
{
   uint32_t cmd_word;

   // The command location is also used for status. Wait for it to change
   // from what we write
   pru_write_word(MEM_PRU0_DATA, PRU0_CMD_DATA,  data);
   pru_write_word(MEM_PRU0_DATA, PRU0_CMD,  cmd);
   while ((cmd_word = pru_read_word(MEM_PRU0_DATA, PRU0_CMD)) == cmd) {
      usleep(500);
   };
   // If not an expected result print an error message
   if (cmd_word != CMD_STATUS_OK && cmd_word != CMD_STATUS_READ_STARTED) {
      msg(MSG_ERR, "Command %d fault %x status %x\n", cmd, 
         cmd_word, pru_read_word(MEM_PRU0_DATA, PRU0_STATUS));
      return cmd_word;
   }
   return 0;
}

// Return the command status value. See cmd.h for values.
uint32_t pru_get_cmd_status(void) {
   return pru_read_word(MEM_PRU0_DATA, PRU0_CMD);
}

// Return the command data value. Not all commands return data.
uint32_t pru_get_cmd_data(void) {
   return pru_read_word(MEM_PRU0_DATA, PRU0_CMD_DATA);
}

// Call this routine to restart PRU at the restart address
// The control register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
//
// pru_num: PRU number, 0 or 1
void pru_restart(int pru_num)
{
   // Set the starting address to STOP_ADDR and restart the PRU.
   pru_write_word(data_mem_type[pru_num], PRU_CONTROL_REG, (RESTART_ADDR << 16) | 2);
}

// Call this routine to get PRU program counter
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
uint32_t pru_get_pc(int pru_num)
{
   // Set the starting address to STOP_ADDR and restart the PRU.
   return pru_read_word(data_mem_type[pru_num], PRU_STATUS_REG);
}

// Call this routine to get PRU halt state
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
uint32_t pru_get_halt(int pru_num)
{
   // Check running bit
   return (pru_read_word(data_mem_type[pru_num], PRU_CONTROL_REG) & 0x8000) == 0;
}

// Call this routine to print registers
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
void pru_print_registers(int pru_num)
{
   int i, reg;

   // Halt so registers are readable
   pru_write_word(data_mem_type[pru_num], PRU_CONTROL_REG,
      pru_read_word(data_mem_type[pru_num], PRU_CONTROL_REG) & ~2);
   for (i = 0; i < 32*4; i += 4) {
      if (i % 32 == 0) {
         msg(MSG_FATAL, "%02d: ",i/4);
      }
      // Set the starting address to STOP_ADDR and restart the PRU.
      reg = pru_read_word(data_mem_type[pru_num], PRU_DEBUG + i);
      msg(MSG_FATAL, "%08x ",reg);
      if (i % 32 == 28) {
         msg(MSG_FATAL,"\n");
      }
   }
   msg(MSG_FATAL,"\n");
}

// Call this routine to print memory
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
// type: type of memory to dump
// start: start offset in bytes
// len : length in bytes
void pru_print_memory(MEM_TYPE mem_type, int start, int len)
{
   int i, reg;

   for (i = start; i < start+len; i += 4) {
      if (i % 32 == 0) {
         msg(MSG_FATAL, "%04x: ",i);
      }
      // Set the starting address to STOP_ADDR and restart the PRU.
      reg = pru_read_word(mem_type, i);
      msg(MSG_FATAL, "%08x ",reg);
      if (i % 32 == 28) {
         msg(MSG_FATAL,"\n");
      }
   }
   msg(MSG_FATAL,"\n");
}

// This returns the size and a pointer to the specified memory type
//
// mem_type: Type of memory to read
// mem_ptr: Address of memory
// mem_size: Size of memory
static void get_mem_addr_size(MEM_TYPE mem_type, uint8_t **mem_ptr,
       int *mem_size) {
   switch(mem_type) {
      case MEM_PRU0_DATA:
         *mem_ptr = pru_dataram0;
         *mem_size = 8*1024;
      break;
      case MEM_PRU1_DATA:
         *mem_ptr = pru_dataram1;
         *mem_size = 8*1024;
      break;
      case MEM_PRU_SHARED:
         *mem_ptr = pru_sharedram;
         *mem_size = 12*1024;
      break;
      case MEM_DDR:
         *mem_ptr = ddr_mem;
         *mem_size = ddr_mem_size;
      break;
      default:
         msg(MSG_FATAL, "Invalid memory type %d\n", mem_type);
         exit(1);
   }
}

// Write to memory. The DDR memory is uncached so slow.
// TODO: Should use cached memory and use cache flush/invalidate
// to improve performance.
//
// Offset and length are in bytes
// mem_type: Type of memory to read
// data: Pointer to location to write data to
// len: Length in bytes to read
// offset: Offset into memory in bytes to read from
int pru_write_mem(MEM_TYPE mem_type, void *data, int len, int offset)
{
   uint8_t *mem_ptr;
   int mem_size;

   get_mem_addr_size(mem_type, &mem_ptr, &mem_size);

   if (offset+len > mem_size || offset < 0 || len < 0) {
      msg(MSG_FATAL, "Transfer exceeds DDR size %d %d %d\n", offset, len,
            mem_size);
      exit(1);
   }
   memcpy(&mem_ptr[offset], data, len);
   return len;
}

// Read from memory
//
// mem_type: Type of memory to read
// data: Pointer to location to write data to
// len: Length in bytes to read
// offset: Offset into memory in bytes to read from
int pru_read_mem(MEM_TYPE mem_type, void *data, int len, int offset)
{
   uint8_t *mem_ptr;
   int mem_size;

   get_mem_addr_size(mem_type, &mem_ptr, &mem_size);

   if (offset+len > mem_size || offset < 0 || len < 0) {
      msg(MSG_FATAL, "Transfer exceeds DDR size %d %d %d\n", offset, len,
            mem_size);
      exit(1);
   }
   memcpy(data, &mem_ptr[offset], len);
   return len;
}

// Write a word to memory. Offset is in bytes from start of memory type
// 
// mem_type: Type of memory to read
// offset: Offset into memory in bytes
// return: Memory contents
uint32_t pru_read_word(MEM_TYPE mem_type, int offset) {
   uint8_t *mem_ptr;
   uint32_t *mem_ptr_32;
   int mem_size;

   get_mem_addr_size(mem_type, &mem_ptr, &mem_size);
   mem_ptr_32 = (uint32_t *) (mem_ptr + offset);

   return *mem_ptr_32;
}

// Write a word to memory. 
//
// mem_type: Type of memory to write
// offset: Offset into memory in bytes
// value: Value to write
void pru_write_word(MEM_TYPE mem_type, int offset, uint32_t value) {
   uint8_t *mem_ptr;
   uint32_t *mem_ptr_32;
   int mem_size;

   get_mem_addr_size(mem_type, &mem_ptr, &mem_size);
   mem_ptr_32 = (uint32_t *) (mem_ptr + offset);
   *mem_ptr_32 = value;
}

// Wait for bits to be set in a memory location. Timeout after a second
// ptr: Address to check
// mask: Mask for bits to check
// value: Value looking for
// desc: Description for error message on failure
static void wait_bits(uint32_t *ptr, uint32_t mask, uint32_t value, char *desc) {
   int i;
   int good = 0;

   // Wait up to a second
   for (i = 0; !good && i < 1000; i++) {
      if ((*ptr & mask) == value) {
         good = 1;
      } else {
         usleep(1000); // wait 1 millisecond/
      } 
   }
   if (!good) { 
      msg(MSG_FATAL, "Didn't get %x waiting for %s mask %x reg value %x\n",
         value, desc, mask, *ptr);
      exit(1);
   }
}
// This routine changes the clock to the PRU if necessary to get the proper
// rate from the PWM. This changes the processor and some of the peripherals
// include ecap. The code only knows how to deal with specific target bit rates
// since calculating proper dividers is non trivial.
// tgt_bitrate_hz: Target bitrate for mfm clock and data in Hertz
// return: PRU clock rate in Hertz
uint32_t pru_set_clock(uint32_t tgt_bitrate_hz, int halt) {
   int fd;
   uint32_t *ptr;
   int map_length = 0x1000;
   uint32_t rc = 0;
   uint32_t orig_control_reg[2];
   int pru_num;
   struct {
     int bitrate;
     int pre_divide;
     int mult;
     int post_divide;
   } rates[] = {
      // This assumes input clock is 24 MHz. We pre divide by 1, multiply by 
      // 33 and divide by 4 to get clock of 198 MHz. That
      // divided by 18 gives desired 11 MHz bit rate.
      { 11000000, 1, 33, 4},
      // This gives 195 MHz, divided by 13 gives 15 MHz rate. Pre divide
      // by 2, multiply by 64 and divide by 4.
      { 15000000, 2, 65, 4},
   };
   int ndx;

   fd = open("/dev/mem", O_RDWR);
   ptr = mmap(0, map_length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 
      0x44e00000);
   if (ptr == MAP_FAILED) {
      msg(MSG_FATAL, "Clock mmap failed", strerror(errno));
      exit(1);
   }
   // Halt so switching clock doesn't mess up PRU
   for (pru_num = 0; pru_num < 2 && halt == PRU_SET_CLOCK_HALT; pru_num++) {
      orig_control_reg[pru_num] = pru_read_word(data_mem_type[pru_num], 
         PRU_CONTROL_REG);
      pru_write_word(data_mem_type[pru_num], PRU_CONTROL_REG,
         orig_control_reg[pru_num] & ~2);
   }
   if (tgt_bitrate_hz == 10000000) {
      // Select normal 200 MHz clock
      *(ptr + CLKSEL_PRU_ICSS_OCP_CLK/4) = 0;
      rc = 200000000; // Default clock works for this
   } else {
      for (ndx = 0; ndx < ARRAYSIZE(rates); ndx++) {
         if (rates[ndx].bitrate == tgt_bitrate_hz) {
            break;
         }
      }
      if (ndx >= ARRAYSIZE(rates)) { 
         msg(MSG_FATAL, "Don't know how to set target bitrate to %d Hz\n",
            tgt_bitrate_hz);
         exit(1);
      }
      // Turn off spread spectrum
      *(ptr + CM_SSC_DELTAMSTEP_DPLL_DISP/4) = 0;
      *(ptr + CM_SSC_MODFREQDIV_DPLL_DISP/4) = 0;
      // Set to bypass so we can update
      *(ptr + CM_CLKMODE_DPLL_DISP/4) = 4;
      wait_bits(ptr + CM_IDLEST_DPLL_DISP/4, 0x100, 0x100, "PLL bypass");
      *(ptr + CM_CLKSEL_DPLL_DISP/4) = (rates[ndx].mult << 8) | 
         (rates[ndx].pre_divide - 1);
      *(ptr + CM_DIV_M2_DPLL_DISP/4) = 0x100 | rates[ndx].post_divide;
      // Enable PLL
      *(ptr + CM_CLKMODE_DPLL_DISP/4) = 7;
      // Verify locked and out of bypass
      wait_bits(ptr + CM_IDLEST_DPLL_DISP/4, 0x101, 0x1,"Locked and not bypass");
      // Select display clock for PRU
      *(ptr + CLKSEL_PRU_ICSS_OCP_CLK/4) = 1;
      rc = 24000000 * rates[ndx].mult / rates[ndx].pre_divide / 
         rates[ndx].post_divide;
   }
   // Restore PRU state
   for (pru_num = 0; pru_num < 2 && halt == PRU_SET_CLOCK_HALT; pru_num++) {
      pru_write_word(data_mem_type[pru_num], PRU_CONTROL_REG,
         orig_control_reg[pru_num]);
   }
   munmap(ptr, map_length);
   close(fd);
   return rc;
}
