// This program monitors the input voltage to the MFM reader/emulator
// and powers down the board when input power is lost.
//
// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
//
// 06/05/16 DJG Fix using read buffer when read failed
// 09/07/14 DJG Ignore temporary A/D read failures 
//
// MFM disk utilities is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MFM disk utilities is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MFM disk utilities.  If not, see <http://www.gnu.org/licenses/>.
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>

#include "msg.h"
#include "parse_cmdline.h"

#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))

// check every 50 milliseconds
#define SLEEP_TIME_US 50000 

// process id of sub command
int pid = 0;

void shutdown_signal(void);

// Main routine. 
int main(int argc, char *argv[])
{
   // Program parameters
   DRIVE_PARAMS drive_params;
   // Access to A/D channel input voltage is connected to
   int fd;
   char *ad_dev = "/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
   char buf[100];
   int rc;
   // Counts voltage under threhold. We require more than one to prevent
   // shutdown from a transient
   int under_count = 0;
   // For collecting statistics for debug print
   int stat_count = 0;
   float ad_sum = 0;
   float ad_min = 999;
   float ad_max = -999;
   float voltage;
   // A/D read failures
   int err_counter = 0;

   // Cleanup when we exit
   signal(SIGINT,(__sighandler_t) shutdown_signal);

   // Find out what we should do
   parse_cmdline(argc, argv, &drive_params);

   fd = open(ad_dev, O_RDONLY);
   if (fd < 0) {
      msg(MSG_FATAL, "Unable to open A/D device %s:\n  %s\n",ad_dev,
         strerror(errno));
      exit(1);
   }

   // If a command was specifed execute it in a separate process
   if (drive_params.command != NULL ) {
      int rc;

      pid = fork();
      if (pid == -1) {
         msg(MSG_FATAL, "Fork failed %s", strerror(errno));
         exit(1);
      }
      if (pid == 0) {
         setpgid(0,0);
         rc = system(drive_params.command);
         if (rc == -1) {
            msg(MSG_FATAL, "Executing command failed: %s\n",
               strerror(errno));
         }
         exit(WEXITSTATUS(rc));
      }
   }

   // Read A/D and if enough samples are under threshold tell command
   // we executed to terminate then execute poweroff command
   while(1) {
      rc = pread(fd, buf, sizeof(buf), 0);
      // For unknown reasions the read occasionally fails with 
      // Resource temporarily unavailable. Don't give up unless it
      // happens too often.
      if (rc < 0) {
         if (err_counter++ > 5) {
            msg(MSG_FATAL, "A/D read failed %s\n",strerror(errno));
            exit(1);
         } else {
            msg(MSG_INFO, "A/D read failed %d %s\n",err_counter, 
               strerror(errno));
         }
      } else if (rc == 0) {
         msg(MSG_FATAL, "A/D read returned no data\n");
         exit(1);
      } else {
         err_counter = 0;
         voltage = atoi(buf) * 1.8 / 4096.0 / drive_params.scale;
         if (stat_count <= 50 && drive_params.debug) {
            ad_sum += voltage;
            if (voltage > ad_max) {
               ad_max = voltage;
            }
            if (voltage < ad_min) {
               ad_min = voltage;
            }
            stat_count++;
            if (stat_count == 50) {
               msg(MSG_INFO, "Average %.2fV max %.2fV min %.2fV\n",
                     ad_sum / stat_count, ad_max, ad_min);
            }
         }
         if (voltage < drive_params.threshold) {
            if (drive_params.debug) {
               msg(MSG_INFO, "Voltage %.2f under threshold %.2f count %d\n",
                     voltage, drive_params.threshold, under_count);
            }
            if (++under_count
                  >= round(drive_params.wait / SLEEP_TIME_US * 1e6)) {
               if (pid != 0) {
                  kill(-pid, SIGINT);
                  waitpid(pid, NULL, 0);
               }
               rc = system(drive_params.powercmd);
               if (rc == -1) {
                  msg(MSG_FATAL, "Executing power off command failed: %s\n",
                        strerror(errno));
               }
               exit(0);
            }
         } else {
            under_count = 0;
         }
      }
      usleep(SLEEP_TIME_US);
   }

   return (0);
}

// SIGINT/ control-c handler. Call our shutdown and exit
void shutdown_signal(void)
{
   if (pid != 0) {
      kill(-pid, SIGINT);
      waitpid(pid, NULL, 0);
   }
   exit(1);
}
