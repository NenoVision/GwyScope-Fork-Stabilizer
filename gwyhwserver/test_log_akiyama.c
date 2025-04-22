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
     int i, j, jlast, todo, muxA, muxB, filter, phaselimit, fmlimit;
     int swap_in[12], swap_out[12];
     double amplitude1, phase1;
     double fmresult, amresult, error, zpiezo, eav, fmthreshold, fmp, fmi, fmd, fmap, fmai, fmad, pidvalue;
     double freq, setfreq, dacval, debug;
     double vfrom, vmax, lrdac[16], adc[16], adc1, adc2;
     double phaseraw = 1.0 ;//536870912.0;
     librpSet rp;

     if (argc<2) {
        fprintf(stderr, "Missing todo. -1 means fast spectrum, 0 means spectrum, 1 means feedback at given frequency in Hz (another argument)\n");
        return 0;
     }

     todo = atoi(argv[1]);
     if (todo==1 || todo==2) setfreq = atoi(argv[2]);


     printf("starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 2, 1, LIBRP_RANGE_SMALL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     librp_SetPhaseShift(&rp, LIBRP_CH_1, 3);
     librp_SetDDSRange(&rp, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL); //hr off now

     muxA = 8;  //mux A (RP1 IN1) setting; 1-16, channel[7] is aux where Akiyama board output is connected
     muxB = 1;  //mux B 
     for (j=0; j<16; j++) lrdac[j] = 0;

     librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
     librp_SetDAC(&rp, LIBRP_CH_2, 0.0); 

     for (i=0; i<12; i++) swap_in[i] = swap_out[i] = 1; 
     librp_SetSwaps(&rp, swap_in, swap_out);

     if (todo==-1) 
     {
        librp_SetState(&rp, LIBRP_MODE_AKIYAMA, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow
        librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0); 

        for (i=49500; i<51000; i+=1)
        {
            freq = i;
            librp_SetGen(&rp, LIBRP_CH_1, freq, 0.1, 0); //0.6 max

            usleep(10000);
            librp_ReadFMResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadDebug(&rp, &debug);
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            librp_ReadADC(&rp, &adc1, &adc2);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
            printf("%g 1: amplitude %g  phase %g   fmresult %g  debug %g  adc %g %g\n", freq, amplitude1, phase1, fmresult, debug, adc1, adc2);
        }
     //feedback off
      //  librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback

        librp_SetDAC(&rp, LIBRP_CH_1, 0);
        librp_SetDAC(&rp, LIBRP_CH_2, 0);
     }
     else if (todo==0) 
     {
        librp_SetState(&rp, LIBRP_MODE_AKIYAMA, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

        librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0);

       for (j=0; j<200; j+=1) 
       {
           dacval = 0 +(double)j/200.0;
           librp_SetDAC(&rp, LIBRP_CH_2, dacval); 

           printf("\n\n");  
           printf("# dacval %g\n", dacval);
           fprintf(stderr, "dacval %g\n", dacval);

//           for (i=51200; i<51500; i+=1)
           for (i=50000; i<50700; i+=5)
           {
               freq = i;
               librp_SetGen(&rp, LIBRP_CH_1, freq, 0.1, 0); //0.6 max

               usleep(10000);
               librp_ReadFMResult(&rp, &fmresult, &amresult); //sproduct
               librp_ReadDebug(&rp, &debug);

               librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
               printf("%g 1: amplitude %g  phase %g   fmresult %g  debug %g\n", freq, amplitude1, phase1, fmresult, debug);
           }
      }
      dacval = 0.07;
      librp_SetDAC(&rp, LIBRP_CH_1, dacval); 

     //feedback off
        librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

   //     librp_SetDAC(&rp, LIBRP_CH_1, 0);
   //     librp_SetDAC(&rp, LIBRP_CH_2, 0);
     }
     else if (todo==2) {
        librp_SetState(&rp, LIBRP_MODE_AKIYAMA, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

        librp_SetSignalProcessing(&rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0);
        librp_SetPid(&rp, LIBRP_CH_1, 0.008, 0.008, 0, 0); //z piezo pid  0.05 0.05 reference, 0.11 still fine
     
        fmp = 0.1; //0.1 reference  0.18 still fine
        fmi = 0.1; //0.1 reference 
        fmd = 0;
        fmap = 0.005; //0.005  reference
        fmai = 0.005; //0.005 reference
        fmad = 0;
        fmthreshold = 8; //20 reference, 30 noisy, 10 slow, all would need to adjust PID
        vfrom = 0;//;//6;//3.2;
        vmax = 10;//8;//9;
        filter = 0;
        phaselimit = 7; //7 is max
        fmlimit = 7; //1:7.5 Hz 2: 15 Hz, 3: ~30 Hz 4: 60 Hz

        librp_SetFM(&rp, 0, 0, 0, fmp, fmi, fmd, fmap, fmai, fmad, phaselimit, fmlimit); //not set setpoint yet
 
        freq = setfreq;
        printf("# Setting frequency to %g\n", setfreq);
        librp_SetGen(&rp, LIBRP_CH_1, setfreq, 0.08, 0); 
        usleep(2000000);

        lrdac[0] = vfrom;
        librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB); //go to start
        librp_SetDAC(&rp, LIBRP_CH_2, 0.0); 


        printf("# test setting z feedback on, setpoint 20\n");
        librp_SetSetpoint(&rp, LIBRP_CH_1, fmthreshold, 0);
        librp_SetState(&rp, LIBRP_MODE_AKIYAMA, LIBRP_MODE_OFF, 1, 0); //mode, feedback, swap, zslow   /was 1, 0 to use z feedback

        usleep(1000000);

        //don't do anything at start
        for (i=0; i<10000; i++) 
        {
            librp_ReadFMResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
            librp_ReadDebug(&rp, &debug);
            librp_ReadResults(&rp, LIBRP_CH_1, &error, &zpiezo, 0);     
            librp_ReadZSlow(&rp, &zpiezo);
            phase1 *= phaseraw;
            debug *= phaseraw;
            printf("%g 1: amplitude %g  phase %g   fmresult %g  amresult %g  error %g  zpiezo %g debug %g\n", vfrom, amplitude1, phase1, fmresult, amresult, error, zpiezo, debug);
            usleep(100);
        }

        //measure phase at start
        eav = 0;
        for (i=0; i<100; i++) {
              librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
              eav += phase1;
              usleep(1000);
        }
        eav /= 100;

        usleep(1000);
        //start phase/frequency setpoint
        printf("# Setting feedback on\n");

        printf("# Setting setpoint to %g\n", eav);
        librp_SetFM(&rp, 1, 0, eav, fmp, fmi, fmd, fmap, fmai, fmad, phaselimit, fmlimit);


        dacval = vfrom;
        lrdac[0] = dacval;
        librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

        for (i=0; i<100; i++) 
        {
           librp_ReadFMResult(&rp, &fmresult, &amresult); //sproduct
           librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
           librp_ReadDebug(&rp, &debug);
           librp_ReadResults(&rp, LIBRP_CH_1, &error, &zpiezo, 1);     
           librp_ReadZSlow(&rp, &zpiezo);
           phase1 *= phaseraw;
           debug *= phaseraw;
           printf("%g 1: amplitude %g  phase %g   fmresult %g  amresult %g  error %g  zpiezo %g debug %g\n", dacval, amplitude1, phase1, fmresult, amresult, error, zpiezo, debug);
           usleep(10000);
       }

        printf("# test setting z feedback off\n");
        librp_SetState(&rp, LIBRP_MODE_AKIYAMA, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow
        usleep(10000);

        //stop the phase/frequency feedback
        printf("# Setting feedback off\n");
        librp_SetFM(&rp, 0, 0, eav, fmp, fmi, fmd, fmap, fmai, fmad, phaselimit, fmlimit);


     }
  

     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

