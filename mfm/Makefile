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

OBJDIR=obj
INCDIR=inc

LIBRARIES = pthread prussdrv m rt

SOURCES =  mfm_read.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c pru_setup.c msg.c parse_cmdline.c analyze.c \
	deltas_read.c drive.c emu_tran_file.c corvus_mfm_decoder.c \
	northstar_mfm_decoder.c board.c
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.c=.o))
SOURCES2 =  mfm_util.c mfm_decoder.c wd_mfm_decoder.c xebec_mfm_decoder.c \
	crc_ecc.c msg.c parse_cmdline.c emu_tran_file.c corvus_mfm_decoder.c \
        northstar_mfm_decoder.c analyze.c deltas_read_file.c drive_file.c
OBJECTS2 = $(addprefix $(OBJDIR)/, $(SOURCES2:.c=.o))
INCLUDES = $(addprefix $(INCDIR)/, analyze.h cmd.h crc_ecc.h deltas_read.h \
	drive.h emu_tran_file.h mfm_decoder.h msg.h parse_cmdline.h pru_setup.h)

CC = gcc

FP_ABI = 
#Rev B angstrom require softfp
ifneq ($(wildcard /usr/lib/arm-angstrom-linux-gnueabi),)
	FP_ABI = -mfloat-abi=softfp 
endif

# If local copy of PRU package exists use it. Currently the
# package isn't provided for Angstrom and doesn't work for Debian
PRU_PACKAGE_PATH = ../am335x_pru_package-master
PASM = pasm
LIB_PATH =
INCL_PATH = -I $(INCDIR)/ -I /usr/include/libiberty/
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
	   
endif

CFLAGS = $(EXTRA_DEFINE) $(INCL_PATH) -O3 -g -Wall -D_FILE_OFFSET_BITS=64

all : $(PRU) mfm_util mfm_read
pru : $(PRU)

mfm_read :  $(OBJECTS)
	$(CC)  $(OBJECTS)  -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@
mfm_util :   $(OBJECTS2)
	$(CC)  $(OBJECTS2) $(LIB_PATH:%=-L %) -lm -lrt -liberty -o $@

clean :
	rm -rf $(OBJDIR)/*.o *.bin mfm_read mfm_util core *~

%.bin: %.p prucode.hp $(INCDIR)/cmd.h
	$(PASM) -b $<

$(OBJDIR)/%.o: %.c $(INCLUDES) | obj
	$(CC) $(CFLAGS) -c -o $@ $<
obj:
	mkdir $@
