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
     int i, j, todo, muxA, muxB, nwaves, decimation, filter_amplitude, filter_phase, lfr;
     double amp, amplitude1, phase1, amplitude2, phase2, debug, debug2, phaseshift, l1x, l1y, l2x, l2y;
     double setfreq1, setfreq2;
     double lrdac[16], adc[16];
     librpSet rp;
     //struct timespec tstart={0,0}, tend={0,0};

     todo = atoi(argv[1]);
     setfreq1 = atoi(argv[2]);
     setfreq2 = atoi(argv[3]);

     printf("# Loading the bitstream... ");
     fflush(stdout);
     system("cat system_wrapper.bit > /dev/xdevcfg");
     printf("done.\n");

     nwaves = 2;
     decimation = 12;
     filter_amplitude = 1;
     filter_phase = 0;
     lfr = 0;

        //Init(librpSet *rpset, int hrdac_regime, int hrdac1_range, int hrdac2_range, int hrdac3_range, 
        //                      int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
        //                      int rp1_input_hv, int rp2_input_hv, int rp_bare_output, int rp_bare_input);

     printf("# starting RP API...\n");
     if (!librp_Init(&rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, 1, 1, 0, 0, 6, 0, 0, 0, 0))
         printf("# init done.\n");
     else{
         fprintf(stderr, "Error: cannot start RP API, exiting\n");
         return 0;
     }

         //SetState(librpSet *rpset, int state, int feedback, int out1, int out2, int outhr, int swap_in, int swap_out, int pidskip)
     librp_SetState(&rp, LIBRP_MODE_A1, 0, 0, 1, 4, 1, 0, 0);  

     phaseshift = 3;
       // SetSignalProcessing(librpSet *rpset, int channel, int decimation, int loopback, int hr, int phaseshift, 
       //int debugsource, int filter_amplitude, int filter_phase, int nwaves, int lfr);
     librp_SetSignalProcessing(&rp, LIBRP_CH_1, decimation, 0, 0, phaseshift, 0, filter_amplitude, filter_phase, nwaves, lfr, 0);
     librp_SetSignalProcessing(&rp, LIBRP_CH_2, decimation, 0, 1, phaseshift, 0, filter_amplitude, filter_phase, nwaves, lfr, 0);


     librp_SetGen(&rp, LIBRP_CH_1, setfreq1, 0, 0); 
        //int librp_SetKPFM(librpSet *rpset, int mode, double frequency, double amplitude, 
        //double p, double i, double d, double offset, int filter, int filter_amplitude, int filter_phase, int nwaves, int lfr);
     
     filter_phase = 1;
     decimation = 12;
     amp = 2;
     librp_SetKPFM(&rp, todo, setfreq2, setfreq2, amp, 0.25, 0, 0, 2.0, decimation, filter_amplitude, filter_phase, nwaves, 0, 1);

     muxA = 4;  
     muxB = 4; 
     for (j=0; j<16; j++) lrdac[j] = 0;
     lrdac[0] = 0;
     librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

     usleep(10000000);
 
     librp_SetKPFM(&rp, todo, setfreq2, setfreq2, amp, 0.25, 0, 0, 2.0, decimation, filter_amplitude, filter_phase, nwaves, 0, 1);
      for (i=0; i<10000; i++)
     {
         librp_ReadDebug(&rp, &debug, &debug2);
         librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
         librp_ReadLockin(&rp, LIBRP_CH_2, &amplitude2, &phase2);
         librp_ReadLockinAxes(&rp, LIBRP_CH_1, &l1x, &l1y);
         librp_ReadLockinAxes(&rp, LIBRP_CH_2, &l2x, &l2y);
    
         printf("l2in %g    l1 %g  %g   l2 %g  %g  l1x %g %g  l2x %g %g\n", debug2, amplitude1, phase1, amplitude2, phase2, l1x, l1y, l2x, l2y);
     }
     librp_SetKPFM(&rp, todo, setfreq2, setfreq2, amp, 0.25, 0, 0, -2.0, decimation, filter_amplitude, filter_phase, nwaves, 0, 1);
     for (i=0; i<10000; i++)
     {
         librp_ReadDebug(&rp, &debug, &debug2);
         librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
         librp_ReadLockin(&rp, LIBRP_CH_2, &amplitude2, &phase2);
         librp_ReadLockinAxes(&rp, LIBRP_CH_1, &l1x, &l1y);
         librp_ReadLockinAxes(&rp, LIBRP_CH_2, &l2x, &l2y);
   
         printf("l2in %g    l1 %g  %g   l2 %g  %g  l1x %g %g  l2x %g %g\n", debug2, amplitude1, phase1, amplitude2, phase2, l1x, l1y, l2x, l2y);
     }
     librp_SetKPFM(&rp, todo, setfreq2, setfreq2, amp, 0.25, 0, 0, 2.0, decimation, filter_amplitude, filter_phase, nwaves, 0, 1);
     for (i=0; i<10000; i++)
     {
         librp_ReadDebug(&rp, &debug, &debug2);
         librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
         librp_ReadLockin(&rp, LIBRP_CH_2, &amplitude2, &phase2);
         librp_ReadLockinAxes(&rp, LIBRP_CH_1, &l1x, &l1y);
         librp_ReadLockinAxes(&rp, LIBRP_CH_2, &l2x, &l2y);
   
         printf("l2in %g    l1 %g  %g   l2 %g  %g  l1x %g %g  l2x %g %g\n", debug2, amplitude1, phase1, amplitude2, phase2, l1x, l1y, l2x, l2y);
     }
  


     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

