# indent -kr -nut -i3
#
# Execute 'make' to create prucode.bin and mfm_read
# Other options:
# make all
# make pru
# make mfm_read
# make mfm_util
# make clean
#

PRU = prucode0.bin
PRU2 = mfm_write0.bin mfm_write1.bin

OBJDIR=obj
INCDIR=inc

LIBRARIES = pthread prussdrv m rt

SOURCES =  mfm_read.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c pru_setup.c msg.c parse_cmdline.c analyze.c \
	deltas_read.c drive.c emu_tran_file.c corvus_mfm_decoder.c \
	northstar_mfm_decoder.c board.c drive_read.c tagged_mfm_decoder.c \
        perq_mfm_decoder.c
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.c=.o))
SOURCES2 =  mfm_util.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c msg.c parse_cmdline.c emu_tran_file.c corvus_mfm_decoder.c \
	northstar_mfm_decoder.c analyze.c deltas_read_file.c drive_file.c \
        tagged_mfm_decoder.c perq_mfm_decoder.c
OBJECTS2 = $(addprefix $(OBJDIR)/, $(SOURCES2:.c=.o))
SOURCES3 =  mfm_write.c msg.c parse_cmdline.c emu_tran_file.c \
	drive.c pru_setup.c crc_ecc.c board.c drive_write.c
OBJECTS3 = $(addprefix $(OBJDIR)/, $(SOURCES3:.c=.o))
INCLUDES = $(addprefix $(INCDIR)/, analyze.h cmd.h crc_ecc.h deltas_read.h \
	drive.h emu_tran_file.h mfm_decoder.h msg.h parse_cmdline.h \
	pru_setup.h version.h)

CC = c99

CPP = c++
GOT_CPP := $(shell $(CPP) --version 2>/dev/null)
ifndef GOT_CPP
$(info no c++ compiler found, CRC support program not built)
	CPP = echo c++
endif

FP_ABI = 
#Rev B angstrom require softfp
ifneq ($(wildcard /usr/lib/arm-angstrom-linux-gnueabi),)
	FP_ABI = -mfloat-abi=softfp 
endif

# If local copy of PRU package exists use it. Currently the
# package isn't provided for Angstrom and doesn't work for Debian
# Used copy from https://github.com/beagleboard/am335x_pru_package.git
# run pru_sw/utils/pasm_source/linuxbuild
PRU_PACKAGE_PATH = ../am335x_pru_package
LIB_PATH =
INCL_PATH = -I $(INCDIR)/ -I /usr/include/libiberty/
PASM = pasm
ifneq ($(wildcard $(PRU_PACKAGE_PATH)),)
   $(info Using local copy of am335x_pru_package)
   PASM = $(PRU_PACKAGE_PATH)/pru_sw/utils/pasm
   LIB_PATH = $(PRU_PACKAGE_PATH)/pru_sw/app_loader/lib/
   INCL_PATH += -I $(PRU_PACKAGE_PATH)/pru_sw/app_loader/include/
endif
PASM += -V2 -L

UNAME_S := $(shell uname -m)
EXTRA_DEFINE =
ifeq ($(UNAME_S), armv7l)
        EXTRA_DEFINE = -mcpu=cortex-a8 -mfpu=vfpv3 $(FP_ABI) \
        -ffast-math -Ofast
else
	GOT_PASM := $(shell $(PASM) --version 2>/dev/null)
	ifndef GOT_PASM
$(info no pasm found, PRU programs not built)
		PASM = echo pasm
	endif
endif

CFLAGS = $(EXTRA_DEFINE) $(INCL_PATH) -O3 -g -Wall -D_FILE_OFFSET_BITS=64 -D_XOPEN_SOURCE=600

all : $(PRU) $(PRU2) mfm_util ext2emu find_crc_info mfm_write mfm_read
pru : $(PRU) $(PRU2)

mfm_read :  $(OBJECTS)
	$(CC)  $(OBJECTS)  -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@
mfm_util :   $(OBJECTS2)
	$(CC)  $(OBJECTS2) $(LIB_PATH:%=-L %) -lm -lrt -liberty -o $@
ext2emu : mfm_util
	ln -s mfm_util ext2emu
mfm_write :  $(OBJECTS3)
	$(CC)  $(OBJECTS3)  -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@

find_crc_info : find_crc_info.cpp
	$(CPP) -O3 -std=c++0x -Wall $< -o $@

clean :
	rm -rf $(OBJDIR)/*.o *.bin mfm_read mfm_util core *~ find_crc_info

%.bin: %.p prucode.hp $(INCDIR)/cmd.h drive_operations.p
	$(PASM) -b $<

$(OBJDIR)/%.o: %.c $(INCLUDES) | obj
	$(CC) $(CFLAGS) -c -o $@ $<
obj:
	mkdir $@
