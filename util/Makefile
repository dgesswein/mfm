SUDO = /usr/bin/sudo
UNAME_R = $(shell uname -r)

all clean:

install:
	$(SUDO) cp -r etc/* /etc
	$(SUDO) chmod +x /etc/kernel/postinst.d/zzz-mfm-emu
	$(SUDO) /etc/kernel/postinst.d/zzz-mfm-emu $(UNAME_R)

.SILENT:
