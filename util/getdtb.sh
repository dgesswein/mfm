#!/bin/bash
vernum=$(echo $1 | cut -f 1-2 -d.)
case "$1" in
        "3."*)          exit ;;
        *"-ti"*)        dtbdir="dtb-${vernum}-ti" ;;
        *"-bone"*)      dtbdir="dtb-${vernum}-bone" ;;
        *)              dtbdir="dtb-${vernum}"
esac
echo $dtbdir
