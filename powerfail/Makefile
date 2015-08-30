# indent -kr -nut -i3
#
# Execute 'make' to create powerfail
# Other options:
# make clean
# make all
# make project
# make clean
#

project = powerfail

OBJDIR=obj
INCDIR=inc

LIBRARIES = pthread m
INCL_PATH = -I $(INCDIR)/ -I ../mfm/$(INCDIR)
#-I ../mfm/$(INCDIR) 

SOURCES =  powerfail.c parse_cmdline.c ../mfm/msg.c 
OBJECTS = $(addprefix $(OBJDIR)/, $(subst ../mfm/,,$(subst .c,.o,$(SOURCES))))
INCLUDES = $(addprefix $(INCDIR)/, parse_cmdline.h)

CC = gcc
EXTRA_DEFINE =
CFLAGS = $(EXTRA_DEFINE) $(INCL_PATH) -O3 -g -Wall -D_FILE_OFFSET_BITS=64 -iquote . -iquote ../mfm

all : $(project)

$(project) : $(OBJECTS)
	$(CC)  $(OBJECTS) -Wl,-rpath=$(LIB_PATH) $(LIB_PATH:%=-L %) $(LIBRARIES:%=-l%) -o $@

clean :
	echo $(OBJ)
	rm -rf $(OBJDIR)/*.o $(project) core *~

$(OBJDIR)/%.o: %.c $(INCLUDES) obj
	$(CC) $(CFLAGS) -c -o $@ $<
$(OBJDIR)/%.o: ../mfm/%.c $(INCLUDES)
	$(CC) $(CFLAGS) -c -o $@ $<
obj:
	mkdir $@
