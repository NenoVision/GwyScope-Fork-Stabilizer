/*
 *  confuseacat: a simple testing program for RP AD and DA converters
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
     int i, j, k, muxA, muxB, confused;
     double dacx, dacy;
     double lrdac[16], adc[16], adc1, adc2, avadc1, avadc2;
     librpSet rp;


     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_FULL, 2, 1, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow   /was 1, 0 to use z feedback
     librp_SetDDSRange(&rp, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL); //hr off now

     muxA = 10;  //mux A (RP1 IN1) setting; 1-16, channel[7] is aux where Akiyama board output is connected
     muxB = 1;  //mux B 

     dacx = dacy = 0;
     avadc1 = avadc2 = 0;

     for (k=0; k<10; k++) {
        librp_SPICycle(&rp, dacx, dacy, 0, lrdac, adc, 1, 1, muxA, muxB); //go to start
        librp_ReadADC(&rp, &adc1, &adc2);
        printf("adc: %g %g\n", adc1, adc2);
        
        avadc1 += adc1/10.0;
        avadc2 += adc2/10.0;

        usleep(10000);
     }
  
     confused = 0;
     for (i=0; i<10000; i++) 
     {
         for (j=0; j<10000; j++)
         {
             dacx = (double)i/100000.0;
             dacy = (double)j/100000.0;

             librp_SPICycle(&rp, dacx, dacy, 0, lrdac, adc, 1, 1, muxA, muxB); //go to start
             librp_ReadADC(&rp, &adc1, &adc2);
             if (j==0) printf("adc: %g %g  xy %g %g\n", adc1, adc2, dacx, dacy);

             if (fabs(adc1-avadc1)>3)  {
                 confused = 1;
                 break;
             }
         }
         if (confused) break;
     }

     if (confused) {    
        printf("Cat was confused!\n");

        for (k=0; k<10; k++) {
           librp_ReadADC(&rp, &adc1, &adc2);
           printf("adc: %g %g\n", adc1, adc2);
           usleep(10000);
        }
     } else {
        printf("Nothing happened\n");
     }
 
     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

