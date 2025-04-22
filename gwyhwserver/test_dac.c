/*
 *  hwserver: a simple implentationa of Gwyfile compatible server for RP AFM operation
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
     int i, j, n;
     librpSet rp;

     double val, step, debug, error, dacval;
     double lrdac[16];
     double adc[16];
     double av;
     double dv;

     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 2, 1, LIBRP_RANGE_SMALL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     //init state
     librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

     for (i=0; i<16; i++) lrdac[i] = 0;

//     step = 0.000122; //14bit
     step = 0.00003051757; //16bit
//     for (val = (-0.0196338-0.00003051757); val <= (-0.0196338 + 0.00003051757); val+=step)  //-0.02... -0.01
//     for (val = 0.01; val <=0.04; val+=step)
     for (n=0; n<1000; n++)
     {

        if (n%2==0) val = 191;
        else val = 192;

        librp_SetDAC(&rp, LIBRP_CH_1, val);
         
        for (i=0; i<100; i++) {
           av = dv = 0;
  //         for (j=0; j<100; j++) {
              librp_SPICycle(&rp, 0.0, 0.0, 0.0, lrdac, adc, 1, 1, 11, 1);
              librp_ReadDebug(&rp, &debug);
              librp_ReadResults(&rp, LIBRP_CH_1, &error, &dacval, 1);
//              printf("debug %g\n", debug);
              printf("%g %g %g %g\n", val, adc[1], debug, dacval);
//              av += adc[1];
//              dv += debug;
 //          }
 //          printf("%g  %g  %g\n", val, av/100.0, dv/100.0);
//           printf("%g  %g %g %g %g %g %g %g %g  %g %g %g %g %g %g %g %g\n", val, adc[0], adc[1], adc[2], adc[3], adc[4], adc[5], adc[6], adc[7], adc[8], adc[9], adc[10], adc[11], adc[12], adc[13], adc[14], adc[15]);
        }
     }

/*
     for (val = -0.01; val >= -0.02; val-=step) 
     {

        librp_SetDAC(&rp, LIBRP_CH_1, val);
         
        for (i=0; i<100; i++) {
           av = dv = 0;
  //         for (j=0; j<100; j++) {
              librp_SPICycle(&rp, 0.0, 0.0, 0.0, lrdac, adc, 1, 1, 11, 1);
              librp_ReadDebug(&rp, &debug);
//              printf("debug %g\n", debug);
              printf("%g %g %g\n", val, adc[1], debug);
//              av += adc[1];
//              dv += debug;
 //          }
 //          printf("%g  %g  %g\n", val, av/100.0, dv/100.0);
//           printf("%g  %g %g %g %g %g %g %g %g  %g %g %g %g %g %g %g %g\n", val, adc[0], adc[1], adc[2], adc[3], adc[4], adc[5], adc[6], adc[7], adc[8], adc[9], adc[10], adc[11], adc[12], adc[13], adc[14], adc[15]);
        }
     }
*/

     if (!librp_Close(&rp)) printf("# RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

