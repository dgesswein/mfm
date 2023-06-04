
// CRC/ECC module definitions
#ifndef CRC_ECC_H_
#define CRC_ECC_H_

// This contains the polynomial, polynomial length, CRC initial value,
// and maximum span for ECC correction. Use span 0 for no ECC correction.
typedef struct {
   uint64_t init_value; 
   uint64_t poly;
   uint32_t length;
   uint32_t ecc_max_span;
} CRC_INFO; 
  
// Routine prototypes
uint64_t crc_revbits(uint64_t v, int length);
uint64_t crc64(uint8_t bytes[], int num_bytes, CRC_INFO *crc_info);
int ecc64(uint8_t bytes[], int num_bytes, uint64_t syndrome, 
   CRC_INFO *crc_info);
uint64_t checksum64(uint8_t *bytes, int num_bytes, CRC_INFO *crc_info);
uint64_t eparity64(uint8_t *bytes, int num_bytes, CRC_INFO *crc_info);
#endif /* CRC_ECC_H_ */
