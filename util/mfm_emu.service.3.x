#execute systemctl --system daemon-reload after changing

[Unit]
Description=MFM Emulator
After=syslog.target fsck-root.service fsck@.service
Before=sysinit.target shutdown.target
DefaultDependencies=no
Conflicts=shutdown.target

[Service]
ExecStart=/root/emu/start_mfm_emu

#StandardOutput=journal+console
StandardOutput=null
StandardError=journal+console

[Install]
WantedBy=sysinit.target
