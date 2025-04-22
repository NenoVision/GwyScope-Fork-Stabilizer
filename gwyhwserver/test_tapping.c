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
     int i, j;
     double error1, dac1, error2, dac2, set1, set2, debug, amplitude1, phase1, amplitude2, phase2, adc1, adc2;
     double sinval, cosval;
     double freq;
     librpSet rp;

     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 2, 1, LIBRP_RANGE_SMALL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     //init state
     librp_SetDDSRange(&rp, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL);
     librp_SetRPADCCal(&rp, 0.1, 1.45);
     librp_SetState(&rp, LIBRP_MODE_NCAMPLITUDE, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow //LOCKIN a HRLOCKIN OK

     librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0);
     librp_SetSignalProcessing(&rp, LIBRP_CH_2, 12, 0, 0, 0, 0, 0, 0);

     sleep(1);
     for (i=78000; i<81500; i+=10)
     {
         freq = i;
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0.2, 0);  //first is scanning, second is kept

         usleep(10000);

         librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
         librp_ReadADC(&rp, &adc1, &adc2);
         printf("%g 1: amplitude %g  phase %g  adcs %g %g\n", freq, amplitude1, phase1, adc1, adc2);
     }

     //feedback off
     //librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback
     //librp_SetDAC(&rp, LIBRP_CH_1, 0);
     //librp_SetDAC(&rp, LIBRP_CH_2, 0);

 

     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

