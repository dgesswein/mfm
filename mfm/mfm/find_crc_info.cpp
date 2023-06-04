#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include <vector>
#include <string>
#include <functional>
#include <algorithm>

#include <iostream>
using namespace std;
#include <iomanip>

// Holds biggest real MFM drive
uint8_t buf[181996480];
// sect_bytes includes size of header and CRC
int crc_bytes, data_bytes, sect_bytes;
int crc_chars;

int num_blocks = 4;

// This contains the polynomial, polynomial length, CRC initial value,
// and maximum span for ECC correction. Use span 0 for no ECC correction.
typedef struct {
   uint64_t init_value; 
   uint64_t poly;
   uint32_t length;
   uint32_t ecc_max_span;
} CRC_INFO; 

// Reverse the bit order of v. 00100001b will become 10000100b
// Not that efficient but we don't use it that often.
//
// v: value to reverse
// length: length of v in bits
// return: reverse of v
uint64_t crc_revbits(uint64_t v, int length)
{
   uint64_t i;
   uint64_t ov;
   uint64_t stop;

   if (length == 64) 
      stop = 0;
   else 
      stop = (uint64_t) 1 << length;

   ov = 0;
   for (i = 1; i != stop; i <<= 1) {
      ov <<= 1;
      if (v & i) {
         ov |= 1;
      }
   }
   return ov;
}

// Calculate the reverse CRC which gives you the 4 initial bytes
// the CRC started with. 
uint64_t crc64r(uint8_t byte, uint64_t crc, CRC_INFO *crc_info)
{
   uint64_t poly;
   // Loop counters
   int bit;

   poly = crc_revbits(crc_info->poly, crc_info->length);
   crc = crc ^ crc_revbits(byte, crc_info->length);
   for (bit = 1; bit <= 8; bit++) {
      // if leftmost (most significant) bit is set xor in the polynomial
      if (crc & ((uint64_t) 1 << (crc_info->length-1))) {
         crc = ((crc ^ poly) << 1) | 1;
      } else {
         crc = crc << 1;
      }
   }
   // Trim the result to the requested length
   if (crc_info->length == 64)
      return crc;
   else
      return crc & (((uint64_t) 1 << crc_info->length)-1);
}

// This runs the CRC backwards calculating the CRC values from end to
// beggining. The CRC at the end is the initial value if the same
// bytes processed as the disk controller did.
uint64_t crc64rb(uint8_t byte, uint64_t crc, CRC_INFO *crc_info)
{
   uint64_t poly = crc_info->poly;
   // Loop counters
   int bit;
   for (bit = 1; bit <= 8; bit++) {
      // if least significant bit is set xor in the polynomial
      // This is only correct of polynomial LSB is set
      if (crc & 0x1) {
         crc = ((crc ^ poly) >> 1) | ((uint64_t) 1 << (crc_info->length-1));
      } else {
         crc = crc >> 1;
      }
   }
   crc = crc ^ ((uint64_t) byte << (crc_info->length - 8));
   return crc;
}

// Get CRC from sector data array
uint64_t get_crc(int sector) {
   int crc_size = sect_bytes - data_bytes;
   uint64_t crc = 0;
   int i;

   for (i = 0; i < crc_size; i++) {
      crc = (crc << 8) | buf[sector + data_bytes + i];
   }
   return crc;
}

// Calculate a CRC up to 64 bits long over the specified data.
// CRC_INFO specifies the CRC length, polynomial, and initial value.
// The CRC is calculated most significant bit first.
// The CRC result is returned. Zero is normally defined as no error
// though other values can be used.
// There are two forms the CRC is normally implemented in. The polynomial can
// be converted between the two forms by
// poly = revbits(poly, length) << 1) | 1
//
// bytes: bytes to calculate CRC over
// num_bytes: length of bytes
// crc_info: CRC parameters to use
// return: CRC of bytes
uint64_t crc64(uint8_t bytes[], int num_bytes, CRC_INFO *crc_info)
{
   int64_t crc;
   // This improves performance vs. accessing directly in loop for the
   // GCC versions tested. The CRC polynomial
   uint64_t poly = crc_info->poly;
   // Loop counters
   int index, bit;

   crc = crc_info->init_value;
   for (index = 0; index < num_bytes; index++) {
      crc = crc ^ ((uint64_t) bytes[index] << (crc_info->length-8));
      for (bit = 1; bit <= 8; bit++) {
         // if leftmost (most significant) bit is set xor in the polynomial
         if (crc & ((uint64_t) 1 << (crc_info->length-1))) {
            crc = (crc << 1) ^ poly;
         } else {
            crc = crc << 1;
         }
      }
   }
   // Trim the result to the requested length
   if (crc_info->length == 64)
      return crc;
   else
      return crc & (((uint64_t) 1 << crc_info->length)-1);
}

// This finds the initial value by either reversing the CRC or by brute force
// if the polynomial isn't the normal form with LSB set.
// It also prints the reversed calculation to verify the starting bytes
void find_init(uint64_t poly, int sector)
{
   CRC_INFO crc_info;
   uint64_t crc;
   uint64_t allones_init, init = 0;
   int i;
   // Try all ones first. Size will be adjusted later
   static uint64_t last_init = 0xffffffffffffffff;

   crc_info.poly = poly;
   crc_info.length = crc_bytes*8;

   if (!(poly & 1)) {
      cout << "Poly LSB not one, using brute force\n";
      
      if (crc_bytes != 8) {
         last_init = last_init & (((uint64_t) 1 << crc_bytes*8)-1);
      }
      crc_info.init_value = last_init;
      if (crc64(&buf[sector], sect_bytes, &crc_info) == 0) {
         cout << "initial value " << hex << setw(crc_chars) << crc_info.init_value << endl;
      } else {
         for (crc_info.init_value = 0; 
               crc_info.init_value < ((uint64_t) 1 << crc_info.length); 
               crc_info.init_value++) {
            if ((crc_info.init_value & 0xfffff)  == 0) {
               cout << "Checking " << hex << setw(crc_chars) << crc_info.init_value << endl;
            }
            if (crc64(&buf[sector], sect_bytes, &crc_info) == 0) {
               cout << "initial value " << hex << setw(crc_chars) << crc_info.init_value << endl;
               last_init = crc_info.init_value;
               break;
            }
         }
      }
   } else {
      crc = get_crc(sector);
      allones_init = 0;
      allones_init = ~allones_init >> (64 - crc_info.length);

      for (i = data_bytes-1; i >= 0; i--) {
	 crc = crc64rb(buf[sector + i], crc, &crc_info);
	 if (crc == 0 || crc == allones_init) {
	    cout << "initial value " << hex << setw(crc_chars) << crc << " ignoring first " << dec << i << " bytes\n";
	    init = crc;
	    break;
	 }
	 if (i == 0) {
	    cout << "initial value " << hex << setw(crc_chars) << crc << endl;
	    init = crc;
	 }
      }

      cout << "First 4 bytes " << hex << setfill('0') << setw(2) << 
         +buf[sector] << setw(2) << +buf[sector+1] << setw(2) <<  +buf[sector+2] << setw(2) << +buf[sector+3] << endl;
      crc = 0;
      for (i = sect_bytes-1; i >= 0; i--) {
	 crc = crc64r(buf[sector + i], crc, &crc_info);
	 if (i <= 8) {
	    cout << dec << i << " " << hex << setw(crc_chars) << +crc_revbits(crc, crc_info.length) << " " << setw(crc_chars) << +(crc_revbits(crc, crc_info.length) ^ init) << endl;
         }
      }
   }
}

struct hash_data_s {
   int sect_offset;
   size_t hash;
   bool operator<(const hash_data_s& rhs) const { return hash < rhs.hash; }
};

// Extact polynomial. This only works when the polynomial was xored in
// with the shift. Otherwise you get zero
void do_poly2(uint64_t crc1, uint64_t crc2, uint64_t crc3, int sector_offset)
{
   crc2 ^= crc1;
   crc3 ^= crc1;
   crc3 = (crc2 << 1) ^ crc3;
   if (crc_bytes != 8) {
      crc3 = crc3 & (((uint64_t) 1 << crc_bytes*8)-1);
   }

   cout << "poly 2 " << hex << setfill('0') << setw(crc_chars) <<  crc3 << " ";
   if (crc3 != 0) {
      find_init(crc3, sector_offset);
   } else {
      cout << endl;
   }
}

// Extract polynomial. This only works when one of crc2, crc3 the polynomial
// was xored in with the shift and the other wasn't. Otherwise you get 0.
// If the polynomial was xored in both it is unknown if any other method can
// find the xor value to get the real polynomial
void do_poly3(uint64_t crc1, uint64_t crc2, uint64_t crc3, int sector_offset)
{
   crc3 = (crc2 << 1) ^ crc3;
   crc2 = (crc1 << 1) ^ crc2;
   crc3 = crc2 ^ crc3;
   if (crc_bytes != 8) {
      crc3 = crc3 & (((uint64_t) 1 << crc_bytes*8)-1);
   }
   cout << "poly 3 " << hex << setw(crc_chars) << crc3 << " ";
   if (crc3 != 0) {
      find_init(crc3, sector_offset);
   } else {
      cout << endl;
   }
}

// This does bit patterns 00 01 10 or 11 10 01
// See 3bit matches for logic
void check_2bit_matches(vector<int> matching_sectors, int first_byte)
{
   //vector<uint64_t> sector_crc;
   struct data_s{
      uint16_t match_bits;
      uint8_t check_bits;
      uint64_t crc;
      int sector_offset;
      bool operator<(const data_s& rhs) const { return match_bits < rhs.match_bits; }
   } data[matching_sectors.size()];
   uint16_t bits;

   for (int bit = 0; bit <= 14; bit++) {
      for (unsigned int j = 0; j < matching_sectors.size(); j++) {
         bits = (uint16_t) buf[matching_sectors[j] + first_byte] << 8 |
              buf[matching_sectors[j] + first_byte + 1];
//printf("bit %d j %d bits %x\n",bit, j, bits);
         data[j].crc = get_crc(matching_sectors[j]);
         data[j].match_bits = 0;
         if (bit > 0) {
            data[j].match_bits = bits << (16 - bit);
            data[j].match_bits >>= 16 - bit;
//printf("match %x\n",data[j].match_bits);
         }
         if (bit < 14) {
            data[j].match_bits |= (bits >> (bit + 2)) << bit;
         }
         data[j].check_bits = (bits >> bit) & 0x3;
         data[j].sector_offset = matching_sectors[j];
//printf("match %x check %x\n", data[j].match_bits, data[j].check_bits);
      }
      sort(data, &data[matching_sectors.size()]);
      uint32_t last_bits = 0x10000;
      int matched[4];
      fill(matched, end(matched), -1);
      for (unsigned int j = 0; j < matching_sectors.size(); j++) {
         if (last_bits == data[j].match_bits) {
//printf("j %d match %x check bits %x\n",j, data[j].match_bits, data[j].check_bits);
            matched[data[j].check_bits] = j;
         }
         if (last_bits != data[j].match_bits || j == matching_sectors.size()-1) {
            if (matched[0] != -1 && matched[1] != -1 && matched[2] != -1) {
               do_poly2(data[matched[0]].crc, data[matched[1]].crc, 
                 data[matched[2]].crc, data[matched[0]].sector_offset);
            } else if (matched[3] != -1 && matched[1] != -1 && matched[2] != -1) {
               do_poly2(data[matched[3]].crc, data[matched[2]].crc, 
                 data[matched[1]].crc, data[matched[3]].sector_offset);
            }
            fill(matched, end(matched), -1);
//printf("j %d match %x check bits %x\n",j, data[j].match_bits, data[j].check_bits);
            matched[data[j].check_bits] = j;
         }
         last_bits = data[j].match_bits;
      }
   }
}

// This does bit patterns 001 010 100 or 110 101 011.
void check_3bit_matches(vector<int> matching_sectors, int first_byte)
{
   //vector<uint64_t> sector_crc;
   struct data_s{
      uint16_t match_bits;
      uint8_t check_bits;
      uint64_t crc;
      int sector_offset;
      bool operator<(const data_s& rhs) const { return match_bits < rhs.match_bits; }
   } data[matching_sectors.size()];
   uint16_t bits;

   // For each 3 bit sequence in the 16 bits we store the 3 bits we are
   // processing in check_bits and the rest in match bits. To extract the
   // polynomial the match_bits must be the same for the 3 bit sequences.
   // To do that we sort the structure by match_bits then check the check_bits
   // for entries that match_bits match.
   for (int bit = 0; bit <= 13; bit++) {
      for (unsigned int j = 0; j < matching_sectors.size(); j++) {
         // Get the 16 bits from the sector data
         bits = (uint16_t) buf[matching_sectors[j] + first_byte] << 8 |
              buf[matching_sectors[j] + first_byte + 1];
//printf("bit %d j %d bits %x\n",bit, j, bits);
         // Save other data we need
         data[j].sector_offset = matching_sectors[j];
         data[j].crc = get_crc(matching_sectors[j]);
         // Separate into match_bits and check_bits
         data[j].match_bits = 0;
         if (bit > 0) {
            data[j].match_bits = bits << (16 - bit);
            data[j].match_bits >>= 16 - bit;
//printf("match %x\n",data[j].match_bits);
         }
         if (bit < 13) {
            data[j].match_bits |= (bits >> (bit + 3)) << bit;
         }
         data[j].check_bits = (bits >> bit) & 0x7;
//printf("match %x check %x\n", data[j].match_bits, data[j].check_bits);
      }
      sort(data, &data[matching_sectors.size()]);
      // Force first time through last_bits to not match
      uint32_t last_bits = 0x10000;
      int matched[8];
      fill(matched, end(matched), -1);
      for (unsigned int j = 0; j < matching_sectors.size(); j++) {
         // Save what check_bits found for the match_bits
         if (last_bits == data[j].match_bits) {
//printf("j %d match %x check bits %x\n",j, data[j].match_bits, data[j].check_bits);
            matched[data[j].check_bits] = j;
         }
         // If starting new sequence or last entry verify if correct check_bitgs
         // seen. If so calculate the polynomail
         if (last_bits != data[j].match_bits || j == matching_sectors.size()-1) {
            if (matched[1] != -1 && matched[2] != -1 && matched[4] != -1) {
               do_poly3(data[matched[1]].crc, data[matched[2]].crc, 
                  data[matched[4]].crc, data[matched[1]].sector_offset);
            } 
            if (matched[6] != -1 && matched[5] != -1 && matched[3] != -1) {
               do_poly3(data[matched[6]].crc, data[matched[5]].crc, 
                  data[matched[3]].crc, data[matched[6]].sector_offset);
            }
            fill(matched, end(matched), -1);
//printf("jb %d match %x check bits %x\n",j, data[j].match_bits, data[j].check_bits);
            matched[data[j].check_bits] = j;
         }
         last_bits = data[j].match_bits;
      }
   }
}
int main(int argc, char *argv[]) {
   int len;
   int n,b;

   sect_bytes = data_bytes + crc_bytes;
   hash<string> hash_fn;
   vector<size_t> unique_sectors;
   vector<int> unique_sectors_offset;
   vector<hash_data_s> sect_hashes;


   if (argc != 3) {
      cerr << "Usage: total_bytes crc_bytes\n";
      return 1;
   }
   sect_bytes = atoi(argv[1]);
   crc_bytes = atoi(argv[2]);
   crc_chars = crc_bytes * 2;
   data_bytes = sect_bytes - crc_bytes;

   len = read(STDIN_FILENO, buf, sizeof(buf));
   
   if (len == sizeof(buf)) {
      cerr << "Increase file buffer size\n";
      return 1;
   }


#if 0
   // This was for generating data to test with
   int i,j;
   for (i = 0; i < 11; i++) {
      CRC_INFO crc_info;

      crc_info.poly = 0x00a00805;
      crc_info.length = 32;
      crc_info.init_value = 0;
      uint64_t crc;
      memset(&buf[i*sect_bytes], 0, sect_bytes);
if (i != 4)
      buf[i*sect_bytes - i / 8 + 49] = 1 << (i % 8);
      //buf[i*sect_bytes - i / 8 + 49] |= 1;
      //buf[i*sect_bytes - i / 8 + 49] ^= 0xff;
printf("Set %d to %x\n",- i / 8 + 49, buf[i*sect_bytes - i / 8 + 49]);
      buf[i*sect_bytes] = 2;
      crc = crc64(&buf[i*sect_bytes], data_bytes, &crc_info);
printf("i %d crc %llx\n",i, crc);
      for (j = 3; j >= 0; j--) {
         buf[i*sect_bytes+data_bytes+j] = crc;
         crc >>= 8;
      }
   }
#endif

   int print_count = 0;
   // Hash each sector and ignore any duplicates
   for (n = 0; n < len; n += sect_bytes) {
      if (print_count++ % 1024 == 0) {
         cout << "Processing sector " << print_count << " of " << len / sect_bytes << '\r';
      }
      
      size_t h = hash_fn(string((char *) &buf[n], sect_bytes));
      vector<size_t>::iterator loc = find(unique_sectors.begin(), unique_sectors.end(), h);
      if (loc == unique_sectors.end()) {
         unique_sectors.push_back(h);
         unique_sectors_offset.push_back(n);
      }
   }
   cout << endl;
   cout << dec << unique_sectors.size() << " unique sectors of " << len / sect_bytes << " total\n";

   // For each byte in sector make hashes excluding byte and byte + 1.
   // Find all matching hashes to check if bit pattern suitable for
   // decoding CRC
   for (b = 0; b <= data_bytes-2 ; b++) {
       for (unsigned int i = 0; i < unique_sectors.size(); i++) {
          char tbuf[sect_bytes];

          // Copy data before bytes dropped
          if (b != 0) {
             memcpy(tbuf, &buf[unique_sectors_offset[i]], b);
          }
          // and after
          if (b < data_bytes-2) {
             memcpy(&tbuf[b], &buf[unique_sectors_offset[i] + b + 2], data_bytes-b-2);
          }

          struct hash_data_s hash_data; 
             
          hash_data.hash = hash_fn(string((char *) tbuf, sect_bytes-1));
          hash_data.sect_offset = unique_sectors_offset[i];
          sect_hashes.push_back(hash_data);
       }
       sort(sect_hashes.begin(), sect_hashes.end());
       
       size_t last_hash = 0; 
       vector<int> matching_sectors;
       // Build list of sectors with matching hash 
       // If sufficient number matches then check 2 bit and 3 bit
       // sequences for correct pattern
       for (unsigned int i = 0; i < sect_hashes.size(); i++) {
          // If first or same hash add sector
          if (i == 0 || sect_hashes[i].hash == last_hash) {
             matching_sectors.push_back(sect_hashes[i].sect_offset);
          };
          // If not same hash then process what we found
          if ((i > 0 && sect_hashes[i].hash != last_hash) ||
                  i == sect_hashes.size()-1) {
             if (matching_sectors.size() >= 3) {
                check_2bit_matches(matching_sectors, b);
             }
             if (matching_sectors.size() >= 3) {
                check_3bit_matches(matching_sectors, b);
             }
             // Clear previous and add new sector
             matching_sectors = {sect_hashes[i].sect_offset};
          }
          last_hash = sect_hashes[i].hash;
       }
       sect_hashes.clear();
   }
}
