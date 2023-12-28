SUDO = /usr/bin/sudo
UNAME_R = $(shell uname -r)
SYSTEMCTL = /bin/systemctl
SYSTEMD_UNIT = /etc/systemd/system/mfm_emu.service
SYSTEMD_UNIT_CONF = /etc/mfm_emu.conf
BASH_ALIASES = /home/debian/.bash_aliases

all clean:

install: | $(SYSTEMD_UNIT)
	$(SUDO) cp -r etc/* /etc
	pwd
	$(SUDO) cp MFM-EMU-1.dts /opt/source/$(shell ./getdtb.sh $(UNAME_R))/src/arm/overlays
	$(SUDO) chmod +x /etc/kernel/postinst.d/zzz-mfm-emu
	$(SUDO) /etc/kernel/postinst.d/zzz-mfm-emu $(UNAME_R)

$(SYSTEMD_UNIT): | $(SYSTEMD_UNIT_CONF)
	$(SUDO) cp mfm_emu.service /etc/systemd/system/mfm_emu.service
	$(SUDO) $(SYSTEMCTL) --system daemon-reload

$(SYSTEMD_UNIT_CONF):
	$(SUDO) cp mfm_emu.conf $@

$(BASH_ALIASES):
	$(SUDO) cp bash_aliases $@
