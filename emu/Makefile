# indent -kr -nut -i3
#
# Execute 'make' to create prucode.bin and mfm_emu
# Other options:
# make clean
# make all
# make pru
# make clean
#

PRU = prucode0_reva.bin prucode0_revb.bin prucode0_revc.bin prucode1_reva.bin prucode1_revb.bin prucode1_revc.bin
PROJECT = mfm_emu

OBJDIR=obj
INCDIR=inc

LIBRARIES = pthread prussdrv m rt

SOURCES =  mfm_emu.c ../mfm/pru_setup.c ../mfm/msg.c parse_cmdline.c \
	../mfm/emu_tran_file.c ../mfm/crc_ecc.c ../mfm/board.c
OBJECTS = $(addprefix $(OBJDIR)/, $(subst ../mfm/,,$(subst .c,.o,$(SOURCES))))
INCLUDES = $(addprefix $(INCDIR)/, cmd.h parse_cmdline.h) ../mfm/$(INCDIR)/msg.h \
	../mfm/$(INCDIR)/emu_tran_file.h ../mfm/$(INCDIR)/crc_ecc.h \
	../mfm/$(INCDIR)/pru_setup.h ../mfm/$(INCDIR)/version.h

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
INCL_PATH = -I $(INCDIR)/  -I ../mfm/$(INCDIR)
ifneq ($(wildcard $(PRU_PACKAGE_PATH)),)
   $(info Using local copy of am335x_pru_package)
   PASM = $(PRU_PACKAGE_PATH)/pru_sw/utils/pasm
   LIB_PATH = $(PRU_PACKAGE_PATH)/pru_sw/app_loader/lib/
   INCL_PATH += -I $(PRU_PACKAGE_PATH)/pru_sw/app_loader/include/
endif

UNAME_S := $(shell uname -m)
EXTRA_DEFINE =
ifeq ($(UNAME_S), armv7l)
        EXTRA_DEFINE = -mcpu=cortex-a8 -mfpu=vfpv3 $(FP_ABI) \
        -ffast-math -Ofast
	   
endif
PASM += -V2

CFLAGS = $(EXTRA_DEFINE) $(INCL_PATH) -O3 -g -Wall -D_FILE_OFFSET_BITS=64

all : $(PRU) $(PROJECT)
pru : $(PRU)

$(PROJECT) : $(OBJECTS)
	$(CC)  $(OBJECTS) -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@

clean :
	echo $(OBJ)
	rm -rf $(OBJDIR)/*.o *.bin $(PROJECT) core *~

%_reva.bin: %.p prucode.hp $(INCDIR)/cmd.h
	$(PASM) -b -L $< $*_reva

%_revb.bin: %.p prucode.hp $(INCDIR)/cmd.h
	$(PASM) -b -L -DREVB $< $*_revb

%_revc.bin: %.p prucode.hp $(INCDIR)/cmd.h
	$(PASM) -b -L -DREVC $< $*_revc

$(OBJDIR)/%.o: %.c $(INCLUDES) | obj
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/%.o: ../mfm/%.c $(INCLUDES) | obj
	$(CC) $(CFLAGS) -c -o $@ $<
obj:
	mkdir $@
