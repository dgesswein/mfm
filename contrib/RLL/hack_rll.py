#!/usr/bin/env python3

# Copyright (c) 2025 Poul-Henning Kamp <phk@phk.freebsd.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

'''
   Decode Western Digital WD1006 RLL 2,7 ST506 disk content
   ========================================================

   This script was written and used to decode the data content
   of a disk attached to a WD1006 RLL 2,7 controller in an old
   PC.

   The disk was read with David Gesswein's MFM Emulator:

	http://www.pdp8online.com/mfm/mfm.shtml

   And saved as raw flux-transitions in a "transitions file.

   This code makes no attempt to understand the track structure
   of the transition file, it simply treats all of it as a flux
   stream and prints out whatever address marks and sectors as
   hexdumps on stdout.

   Western Digitals more advanced diskcontrollers could use either
   a 32 or 56 bit CRC polynomial, defaulting to 56 bit.

   That 56-bit polynomial is documented on pdf page 45 in:

   http://bitsavers.org/pdf/westernDigital/pc_disk_controller/WD1007A_AT_ESDI_AM8753/WD1007-WAH_OEM_Manual_198802.pdf

   On behalf of Datamuseum.dk and myself:

   Many thanks to Davis Gesswein for designing the MFM-reader
   and just as many thanks to All Kossow for bitsavers.org.

   Poul-Henning

'''

import sys

import crcmod

# The address marks used the usual CRC-16
crc_func = crcmod.mkCrcFun(poly=0x11021, rev=False, initCrc=0xffff)

# The sectors use a 56 bit "ECC" code which is really just a CRC
# but CRCmod does not have a 56 bit variant, so...

def crc_poly(data, n, poly, crc=0):
    for d in data:
        crc ^= d << (n - 8)
        for _ in range(8):
            crc <<= 1
            if crc & (1 << n):
                crc ^= poly
    return crc

datapoly = 0
for p in (56, 52,50,43,41,34,30,26,24,8, 0):
    datapoly |= 1<<(p)

def first_pass(fn):
    ''' Iterate the input file as clock periods '''
    # We really should do the proper thing with headers etc.
    with open(fn, "rb") as file:
        while True:
            buffer = file.read(4096)
            if len(buffer) == 0:
                return
            for octet in buffer:
                yield max(0, int((octet+6.666)/13.333))

def second_pass(fn):
    ''' Iterate over sequences separated by clock sync pattern '''
    acc = []
    for x in first_pass(fn):
        acc.append(x)
        if acc[-2:] == [8, 3]:
            yield acc[:-2]
            acc = []
    yield acc

def third_pass(fn):
    ''' Iterate as text-representation of waveform '''
    for buf in second_pass(fn):
        yield "|".join("-" * (dt-1) for dt in buf) + "|"

def fourth_pass(fn):
    ''' Decode RLL 2,7 as binary bits as text '''
    for buf in third_pass(fn):
        # We emit the length here, to help resolve which AM belongs to which DM
        print(4, len(buf))
        retval = []
        pos = 2
        while pos < len(buf):
            if buf[pos:pos+4] == '-|--':
                retval.append('10')
                pos += 4
            elif buf[pos:pos+4] == '|---':
                retval.append('11')
                pos += 4
            elif buf[pos:pos+6] == '|--|--':
                retval.append('000')
                pos += 6
            elif buf[pos:pos+6] == '---|--':
                retval.append('010')
                pos += 6
            elif buf[pos:pos+6] == '--|---':
                retval.append('011')
                pos += 6
            elif buf[pos:pos+8] == '--|--|--':
                retval.append('0010')
                pos += 8
            elif buf[pos:pos+8] == '----|---':
                retval.append('0011')
                pos += 8
            else:
                break
                retval.append(buf[0])
                pos += 1
        yield ''.join(retval)

def fifth_pass(fn):
    ''' Iterate as bytes '''
    for buf in fourth_pass(fn):
        yield bytes(int(buf[x:x+8], 2) for x in range(1, len(buf)-7, 8))

def main(fn):
    ''' Check CRC and emit '''
    for buf in fifth_pass(fn):
        if len(buf) == 0:
            pass
        elif buf[0] == 0xf8:
            c = crc_poly(b'\xa1' + buf[:520], 56, datapoly, crc=(1<<56)-1)
            print("6 DM", "%5d" % len(buf), "%014x" % c, buf[:520].hex())
        elif buf[0] > 0xf8:
            c = crc_func(b'\xa1' + buf[:6])
            print("6 AM", "%5d" % len(buf), "%04x" % c, buf[:6].hex())
        else:
            print("6 ??", "%5d" % len(buf), "f", buf[:20].hex())

if __name__ == "__main__":
    assert len(sys.argv)  == 2
    main(sys.argv[1])
