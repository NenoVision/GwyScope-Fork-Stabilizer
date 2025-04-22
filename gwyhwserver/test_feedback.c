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
       
#define ON 1
#define OFF 0

int main(int argc, char *argv[])
{
     int i, j, ret, swap;
     int muxA, muxB;
     double error, dac, hrdac, fmpresult, fmaresult;
     double adc1, adc2, lrdac[16], adc[16];

     librpSet rp;

     //load the bitstream
     printf("# Loading the bitstream... ");
     fflush(stdout);
     system("cat system_wrapper.bit > /dev/xdevcfg");
     printf("done.\n");

     printf("# starting RP API...\n");

     //int librp_Init            (librpSet *rpset, int hrdac_regime, int hrdac1_range, int hrdac2_range, int hrdac3_range,
     //                           int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
     //                           int rp1_input_hv, int rp2_input_hv, int rp_bare_output, int rp_bare_input);

     if (!librp_Init(&rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, 
                                  1, 1, 0, 0, 6, 
                                  0, 0, 0, 0)) printf("# init done.\n");
     else{
         fprintf(stderr, "Error: cannot start RP API, exiting\n");
         return 0;
     }

     swap = 1;

     //librpSet *rpset, int state, int feedback, int out1, int out2, int outhr, int swap_in, int swap_out, int pidskip
     librp_SetState(&rp, LIBRP_MODE_IN1, OFF, 0, 7, 4, swap, 0, 1);

     muxA = 4;
     muxB = 5;
     for (j=0; j<16; j++) lrdac[j] = 0;
     if ((ret=librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 0, 0, muxA, muxB)) != LIBRP_OK)
            fprintf(stderr, "Error: SPICycle returned %d\n", ret);

     //librp_SetSignalProcessing(librpSet *rpset, int channel, int decimation, int loopback, int hr, int phaseshift, int debugsource, int filter_amplitude, int filter_phase, int nwaves, int lfr, int intosc);
     librp_SetSignalProcessing(&rp, LIBRP_CH_1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0);

     librp_SetSetpoint(&rp, 1, 0);

     //double p, double i, double d, int errorshift);
     librp_SetPid(&rp, 0.0000, 0.1, 0, 11);

     librp_SetState(&rp, LIBRP_MODE_IN1, ON, 16, 7, 4, swap, 0, 1);

     for (i=0; i<2000; i++)
     {
  //      librp_SetHRDAC3(&rp, 0.5);
        librp_ReadADC(&rp, &adc1, &adc2);
        librp_ReadResults(&rp, &error, &dac, 0);
        librp_ReadHRDAC(&rp, &hrdac);
        librp_ReadPLLResult(&rp, &fmpresult, &fmaresult);

        fprintf(stderr, "adc %g  e %g %g  hr %g   debug %g %g\n", adc1, error, dac, hrdac, fmpresult, fmaresult);
        //usleep(1000);
     }

      
     //set_zpiezo(hw, hrdac, TRUE);
     librp_SetFeedback(&rp, 0);
     for (i=0; i<2000; i++)
     {
        librp_ReadADC(&rp, &adc1, &adc2);
        librp_ReadResults(&rp, &error, &dac, 0);
        librp_ReadHRDAC(&rp, &hrdac);
        librp_ReadPLLResult(&rp, &fmpresult, &fmaresult);

        fprintf(stderr, "adc %g  e %g %g  hr %g   debug %g %g\n", adc1, error, dac, hrdac, fmpresult, fmaresult);
        //usleep(1000);
     }

     librp_SetSetpoint(&rp, 1.1, 0);
     librp_SetFeedback(&rp, 1);


     for (i=0; i<2000; i++)
     {
        librp_ReadADC(&rp, &adc1, &adc2);
        librp_ReadResults(&rp, &error, &dac, 0);
        librp_ReadHRDAC(&rp, &hrdac);
        librp_ReadPLLResult(&rp, &fmpresult, &fmaresult);

        fprintf(stderr, "adc %g  e %g %g  hr %g   debug %g %g\n", adc1, error, dac, hrdac, fmpresult, fmaresult);
        //usleep(1000);
     }


     //sleep(1000);

     librp_SetState(&rp, LIBRP_MODE_IN1, OFF, 16, 7, 4, swap, 0, 1);


     if (!librp_Close(&rp)) printf("RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

