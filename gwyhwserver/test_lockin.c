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
     double freq, ampl;
     double aval[10000], pval[10000], sval[10000];
     librpSet rp;

     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 3, 0, LIBRP_RANGE_SMALL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     //init state
     librp_SetDDSRange(&rp, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL);
     librp_SetRPADCCal(&rp, 0.1, 1.45);
     librp_SetState(&rp, LIBRP_MODE_LOCKIN, LIBRP_MODE_LOCKIN, 0, 0); //mode, feedback,

     librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0);

     freq = 5000;
     librp_SetGen(&rp, LIBRP_CH_1, freq, 0.5, 0); //2: b   1: a
//     sleep(20);

//     librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0); //2: b   1: a
    
/*
     for (i=0; i<10000; i+=100)
     {
         freq = 4500;//1000;
         ampl = i/10000.0;
         //if (i%2000==0) freq = 4600;
         //else if ((i+1000)%2000==0) freq = 4500;

         librp_SetGen(&rp, LIBRP_CH_1, freq, ampl, 0); 
 
         librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
         aval[i] = amplitude1;
         pval[i] = phase1;
         librp_ReadADC(&rp, &adc1, &adc2);
         sval[i] = adc1;
        // printf("%g 1: amplitude %g  phase %g  adcs %g %g\n", freq, amplitude1, phase1, adc1, adc2);

         usleep(100000);
     }

    for (i=0; i<10000; i+=1) printf("%d %g %g %g\n", i, aval[i], pval[i], sval[i]);

//     librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback

//     librp_SetDAC(&rp, LIBRP_CH_1, 0);
//     librp_SetDAC(&rp, LIBRP_CH_2, 0);
 */

     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

