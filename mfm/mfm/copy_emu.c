// copy track xfer_cyl, xfer_head from good.emu to bad.emu
// build with 
//   gcc -o copy_emu copy_emu.c emu_tran_file.c msg.c crc_ecc.c -Iinc -lm

#include <stdint.h>
#include <stdio.h>

#include "inc/emu_tran_file.h"
#include "inc/crc_ecc.h"
#include "inc/mfm_decoder.h"

int main() {
   int in_fd, out_fd;
   EMU_FILE_INFO emu_in_file_info, emu_out_file_info;
   int cyl,head;
   unsigned int words[MAX_TRACK_WORDS];
   unsigned int words2[MAX_TRACK_WORDS];
   int xfer_cyl = 0;
   int xfer_head = 0;
   FILE *out;
   int i;

   in_fd = emu_file_read_header("good.emu", &emu_in_file_info, 0);   
   out_fd = emu_file_read_header("bad.emu", &emu_out_file_info, 1);   


   emu_file_seek_track(in_fd, xfer_cyl, xfer_head, &emu_in_file_info);
   emu_file_read_track_bits(in_fd, &emu_in_file_info, words, ARRAYSIZE(words),
     &cyl, &head); 

#if 0
   emu_file_seek_track(out_fd, xfer_cyl, xfer_head, &emu_out_file_info);
   emu_file_read_track_bits(out_fd, &emu_out_file_info, words2, ARRAYSIZE(words2),
     &cyl, &head); 
for (i = 0; i < 4000; i++) {
   if (words[i] != words2[i]) {
      printf("Diff %d %x %x\n", i, words[i], words2[i]);
   }
}
words[1038] = words2[1038];
#endif
   emu_file_seek_track(out_fd, xfer_cyl, xfer_head, &emu_out_file_info);
   emu_file_write_track_bits(out_fd, words, ARRAYSIZE(words), xfer_cyl,
      xfer_head, emu_out_file_info.track_data_size_bytes); 

   emu_file_close(in_fd, 0);
   emu_file_close(out_fd, 0);

#if 0
   out = fopen("/tmp/good_track", "w");
   for (i = 0; i < emu_out_file_info.track_data_size_bytes / 4; i++) {
      fprintf(out, "%08x\n",words2[i]);
   }
   fclose(out);
   out = fopen("/tmp/bad_track", "w");
   for (i = 0; i < emu_out_file_info.track_data_size_bytes / 4; i++) {
      fprintf(out, "%08x\n",words[i]);
   }
   fclose(out);
#endif
}
