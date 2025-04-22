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

#include <pthread.h>
#include <math.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <errno.h> 
#include <signal.h>

#include "cmirp.h"

int main(int argc, char *argv[]){

    librpSet rp;
    int i, j, read_adc1, read_adc2, ret, muxA, muxB;
    double hrdac1, hrdac2, hrdac3, lrdac[16], adc[16];
    clock_t t;
    struct timespec deadline;

    printf("# Loading the bitstream... ");
    fflush(stdout);
    system("cat system_wrapper.bit > /dev/xdevcfg");
    printf("done.\n");

    printf("# starting RP API...\n");

    //int librp_Init            (librpSet *rpset, int hrdac_range, int hrdac_nchannels, int zpiezo_hrdac, int zpiezo_hrdac_range,
    //                           int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
    //                           int rp1_input_hv, int rp2_input_hv, int rp_bare_output);
    if (!librp_Init(&rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, 1, 1, 0, 0, 6, 0, 0, 0, 0))
        printf("# init done.\n");
    else{
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
    }

    //set pid parameters
    //int librp_SetState        (librpSet *rpset, int state, int feedback, int out1, int out2, int outhr, int swap_in, int swap_out, int pidskip);
    librp_SetState(&rp, 1, 0, 1, 1, 2, 0, 0, 1);

    hrdac1 =  0;  //value for 20 bit DAC 1
    hrdac2 =  0;  //value for 20 bit DAC 2
    hrdac3 =  0;  //unused value for 20 bit DAC 3

    read_adc1 = 1;  //should we read adc 1-8?
    read_adc2 = 1;  //should we read adc 9-16?
    muxA = 1;  //mux A (RP1 IN1) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX1
    muxB = 1;  //mux B (RP1 IN2) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX2
/*
    for (i=0; i<=10; i++){
        hrdac1 = (double)i/1.0 - 10.0;
        hrdac2 = 10.0 - (double)i/1.0;
        hrdac3 = (double)i/1.0;

        librp_SetHRDAC1(&rp, hrdac1);
        librp_SetHRDAC2(&rp, hrdac2);
        librp_SetHRDAC3(&rp, hrdac3);

        if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
            fprintf(stderr, "Error: SPICycle returned %d\n", ret);

        usleep(10000);

        for (j=0; j<3; j++){
           if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
               fprintf(stderr, "Error: SPICycle returned %d\n", ret);

           printf("hrdac: %lf %lf %lf  adc16ch: %lf %lf %lf\n",
                  hrdac1, hrdac2, hrdac3,
                  adc[0], adc[1], adc[2]); 
          
           usleep(100000);
       }
    }
*/
    printf("# generating fast signals\n");
    t = clock();

    deadline.tv_sec = 0;
    deadline.tv_nsec = 1000;


    for (i=0; i<=10000000; i++){
        //hrdac1 = i%2;//sin(((double)i)/31.414592);
        //hrdac2 = (i+1)%2;//cos(((double)i)/31.414592);
        //hrdac3 = 0;//cos(((double)i + 100)/31.414592);;
        hrdac1 = sin(((double)i)*(2*M_PI)/10);
      //  hrdac2 = cos(((double)i)*(2*M_PI)/10);
      //  hrdac3 = cos(((double)i + 100)/31.414592);;

        librp_SetHRDAC1(&rp, hrdac1);
      //  librp_SetHRDAC2(&rp, hrdac2);
      //  librp_SetHRDAC3(&rp, hrdac3);

       // usleep(1);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &deadline, NULL);
        //clock_nanosleep(CLOCK_REALTIME, 0, &deadline, NULL);
        deadline.tv_nsec += 1000;
    }
    t = clock() - t;
    printf ("It took me %ld clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
    printf("# done\n");

    if (!librp_Close(&rp))
        printf("# RP API closed\n");
    else 
        fprintf(stderr, "Error: cannot stop RP API\n");

    return 0;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 */
