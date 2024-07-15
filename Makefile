# indent -kr -nut -i3
#

# Do not:
# o  use make's built-in rules and variables
#    (this increases performance and avoids hard-to-debug behaviour);
# o  print "Entering directory ...";
MAKEFLAGS += -rR --no-print-directory

SUBDIRS := $(wildcard */.)
TARGETS := all clean

SUBDIRS_TARGETS := \
	$(foreach t,$(TARGETS),$(addsuffix $t,$(SUBDIRS)))

.PHONY : $(TARGETS) $(SUBDIRS_TARGETS)

$(TARGETS) : % : $(addsuffix %,$(SUBDIRS))

update:
	cd mfm
	git pull --ff-only
	make

$(SUBDIRS_TARGETS) :
	$(MAKE) -C $(@D) $(@F:.%=%)
	@/bin/echo -e "\n"

help:
	@echo "Targets:"
	@echo "          all:         Build all MFM emulator binaries"
	@echo "        clean:         Remove all build artifacts"
	@echo "       update:         Updates code from github and rebuilds"
	@echo ""
	@echo "This top-level Makefile recurses over these subdirectories:"
	@echo "        $(wildcard */)"

