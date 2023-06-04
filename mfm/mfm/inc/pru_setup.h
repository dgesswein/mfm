/*
 * pru_setup.h
 *
 *  Created on: Dec 20, 2013
 *      Author: djg
 */

#ifndef PRU_SETUP_H_
#define PRU_SETUP_H_

typedef enum {
   MEM_PRU0_DATA,
   MEM_PRU1_DATA,
   MEM_PRU_SHARED,
   MEM_DDR
} MEM_TYPE;

int  pru_setup(int num_pru_in);
void pru_shutdown(void);
int pru_exec_cmd(uint32_t cmd, uint32_t data);
uint32_t pru_get_cmd_status(void);
uint32_t pru_get_cmd_data(void);
void pru_restart(int pru_num);
uint32_t pru_get_pc(int pru_num);
uint32_t pru_get_halt(int pru_num);
void pru_print_registers(int pru_num);
void pru_print_memory(MEM_TYPE mem_type, int start, int len);
int pru_write_mem(MEM_TYPE mem_type, void *data, int len, int offset);
int pru_read_mem(MEM_TYPE mem_type, void *data, int len, int offset);
uint32_t pru_read_word(MEM_TYPE mem_type, int loc);
void pru_write_word(MEM_TYPE mem_type, int loc, uint32_t value);
#define PRU_SET_CLOCK_HALT 1
#define PRU_SET_CLOCK_NO_HALT 0
uint32_t pru_set_clock(uint32_t tgt_bitrate_hz, int halt);
#endif /* PRU_SETUP_H_ */
