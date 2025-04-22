/*
 *  hwserver: a simple implentation of Gwyfile compatible server for RP AFM operation
 *  Copyright (C) 2022 Petr Klapetek, Miroslav Valtr
 *  E-mail: klapetek@gwyddion.net.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *  Boston, MA 02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/unistd.h>
#include <sys/fcntl.h> 
#include <sys/termios.h> 
#include <sys/types.h>

#include <math.h>

#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <errno.h> 
#include <signal.h>

#include "cmirp.h"
       

int main(int argc, char *argv[])
{
     int j;
     double error1, dac1, a1, p1;
     double adc1, adc2;
     librpSet rp;

     double zfrom, zto, dz, z;
     int nz;

     if (argc<2) {
         fprintf(stderr, "Syntax: ./test_nenoprobe zfrom zto nz\n");
         return 0;
     }

     zfrom = atof(argv[1]);
     zto = atof(argv[2]);
     nz = atoi(argv[3]);
     dz = (zto-zfrom)/nz;

     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 2, 1, LIBRP_RANGE_SMALL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     //init state
     librp_SetState(&rp, LIBRP_MODE_PROPORTIONAL, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

     librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0);
     librp_SetSignalProcessing(&rp, LIBRP_CH_2, 12, 0, 0, 0, 0, 0, 0);

     //librp_SetSetpoint(&rp, LIBRP_CH_1, 0, 0);
     //librp_SetSetpoint(&rp, LIBRP_CH_2, 0, 0);

     //librp_SetPid(&rp, LIBRP_CH_1, 0.1, 0, 0, 0, 0);
     //librp_SetPid(&rp, LIBRP_CH_2, 0.1, 0, 0, 0, 0);

     //librp_SetGen(&rp, LIBRP_CH_1, 1000, 0);
     //librp_SetGen(&rp, LIBRP_CH_2, 1000, 0);

     for (z=zfrom; z<zto; z+=dz)
     {

        librp_SetDAC(&rp, LIBRP_CH_1, z);
        usleep(10000); 

        for (j=0; j<1; j++) {
           librp_ReadADC(&rp, &adc1, &adc2);
           librp_ReadResults(&rp, LIBRP_CH_1, &error1, &dac1, 0);
           librp_ReadLockin(&rp, LIBRP_CH_1, &a1, &p1);

           fprintf(stdout, "z %g adc %g %g  Ch1 dac: %g  e: %g   amplitude: %g  phase %g\n", z, adc1, adc2, dac1, error1, a1, p1);
        }
     }

     //feedback off
     librp_SetState(&rp, LIBRP_MODE_PROPORTIONAL, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

     librp_SetDAC(&rp, LIBRP_CH_1, 0);

     if (!librp_Close(&rp)) printf("# RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

