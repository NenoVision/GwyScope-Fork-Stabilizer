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
     int i, j, jlast, todo, muxA, muxB, filter, phaselimit, fmlimit, reached, phaseshift, amon, phon, phs, nwaves, decimation, filter_phase, lfr;
     int swap_in[12], swap_out[12];
     double amplitude1, phase1, amplitude, phase;
     double fmresult, amresult, error, zpiezo, phav, aav, eav, fmthreshold, fmp, fmi, fmd, fmap, fmai, fmad, pidvalue, amp, ami, amd, buf, fmerror;
     double freq, setfreq, dacval, debug, debug2, zpiezoval, zpiezoinit, dacmax, dac;
     double vfrom, vmax, lrdac[16], adc[16], adc1, adc2;
     double phaseraw = 1.0 ;//536870912.0;
     librpSet rp;
     struct timespec tstart={0,0}, tend={0,0};

     if (argc<2) {
        fprintf(stderr, "Missing todo. -1 means fast spectrum, 0 means spectrum, 1 means feedback at given frequency in Hz (another argument)\n");
        return 0;
     }

     todo = atoi(argv[1]);
     if  (todo==0 || todo==1 || todo==2 || todo==3 || todo==-2 || todo==-3 || todo==-4) setfreq = atoi(argv[2]);

     printf("# Loading the bitstream... ");
     fflush(stdout);
     system("cat system_wrapper.bit > /dev/xdevcfg");
     printf("done.\n");

     nwaves = 0;
     decimation = 12;
     filter_phase = 1;
     lfr = 1;

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
     librp_SetState(&rp, LIBRP_MODE_FM, 0, 0, 4, 4, 1, 0, 0);  

     phaseshift = 3;
     // SetSignalProcessing(librpSet *rpset, int channel, int decimation, int loopback, int hr, int phaseshift, int debugsource, int filter_amplitude, int filter_phase, int nwaves, int lfr);
     librp_SetSignalProcessing(&rp, LIBRP_CH_1, decimation, 0, 0, phaseshift, 0, 0, filter_phase, nwaves, lfr, 0);

     muxA = 10;  
     muxB = 11; 
     for (j=0; j<16; j++) lrdac[j] = 0;
     lrdac[0] = 0;

     librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

     if (todo==-1) //scan frequency
     {
         for (i=430000; i<434000; i+=1)
     //    for (i=100000; i<1000000; i+=100)
         {
             freq = (double)i/10.0;
             librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0); 

             usleep(10000);
             librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
             librp_ReadDebug(&rp, &debug, &debug2);
             librp_ReadADC(&rp, &adc1, &adc2);
 
             librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);     
             printf("%g    amplitude %g  phase %g   fmresult %g  debug %g  adc %g %g\n", freq, amplitude1, phase1, fmresult, debug, adc1, adc2);
         }
      }
      else if (todo==-2) { //watch what frequency is doing when we change phase setpoint

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 1.0, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         usleep(2000000);

         phav = aav = 0;
         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            aav += amplitude1;
            usleep(2000);
         }
         phav /= 100.0;
         aav /= 100.0;
         printf("# phase is %g\n", phav);
         fprintf(stderr, "# phase is %g amplitude %g\n", phav, aav);

         fmp = 0.1;//about 0.1 fine
         fmi = 0.05;//about 0.05 fine
         fmd = 0;
         amp = 0.01; //0.005
         ami = 0.1; //0.04
         amd = 0;
         amon = 0;
         phase = phav;
         //aav = 0.9;//67128;
         printf("# Setting setpoint to %g  %g\n", phav, aav);
             //SetPLL(librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pidskip, int pll_input);
         librp_SetPLL(&rp, 0, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 7, 3, 0, 0); //was 7 7 0 0
         librp_SetSetpoint(&rp, 0, 0);

         usleep(20000);
         printf("# Setting fm feedback on\n");
         librp_SetPLL(&rp, 1, amon, phav, aav, fmp, fmi, fmd, amp, ami, amd, 7, 3, 0, 0); //1:15 Hz, 2: 30 Hz

         clock_gettime(CLOCK_MONOTONIC, &tstart);
         for (i=0; i<200000; i++)
         {

         /*   phase = phav + 0.1*sin(((double)i)/1000.0);
            librp_SetPLL(&rp, 1, amon, phase, aav, fmp, fmi, fmd, amp, ami, amd, 6, 6, 0, 0);
*/

/*            if (i==2000) {
               phase = phav - 0.3;
               librp_SetPLL(&rp, 1, amon, phase, aav, fmp, fmi, fmd, amp, ami, amd, 6, 6, 0, 0);
            }
 */
            if ((i%4000==0)) {
               phase = phav + 0.15;
               librp_SetPLL(&rp, 1, amon, phase, aav, fmp, fmi, fmd, amp, ami, amd, 7, 3, 0, 0);

            } 
            else if ((i-2000)%4000==0) {
               phase = phav - 0.15;
               librp_SetPLL(&rp, 1, amon, phase, aav, fmp, fmi, fmd, amp, ami, amd, 7, 3, 0, 0);
            } 
            if (i==50000) {
               amon = 1;
               librp_SetPLL(&rp, 1, amon, phase, aav, fmp, fmi, fmd, amp, ami, amd, 7, 3, 0, 0);
            }

    
         //   usleep(100);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadDebug(&rp, &buf, &fmerror);
            librp_ReadResults(&rp, &error, &zpiezo, 0);
            clock_gettime(CLOCK_MONOTONIC, &tend);
            printf("%g    amplitude %g  phase %g fmresult %g fmerror %g ph %g  amr %g errorsignal %g\n", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec), amplitude1, phase1, fmresult, fmerror, phase, amresult, error);
         }

         librp_SetPLL(&rp, 0, 0, phav, 0.1, fmp, fmi, fmd, 0, 0, 0, 0, 0, 1, 0);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0);
     }
      else if (todo==-3) { //watch what phase is doing at frequency change, no feedback

         freq = setfreq;
         amplitude = 2.0;
         phs = 0;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);
         librp_SetPhaseShift(&rp, LIBRP_CH_1, phs);

         librp_SetPLL(&rp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 0, 0);

         clock_gettime(CLOCK_MONOTONIC, &tstart);
         for (i=0; i<14000; i++)
         {
            if (i==4000 || i==8000 || i==12000) {
               freq = setfreq + 10;
               //amplitude = 2.2;
       //        phs = 0;
               librp_SetGen(&rp, LIBRP_CH_1, freq, amplitude, 0);
      //         librp_SetSignalProcessing(&rp, LIBRP_CH_1, decimation, 0, 0, phs, 0, 0, filter_phase, nwaves, lfr, 0);            
            } 
            else if (i==6000 || i==10000) {
               freq = setfreq;
               //amplitude = 2.0;
        //       phs = 1;
               librp_SetGen(&rp, LIBRP_CH_1, freq, amplitude, 0);
           //    librp_SetSignalProcessing(&rp, LIBRP_CH_1, decimation, 0, 0, phs, 0, 0, filter_phase, nwaves, lfr, 0);            
            } 
            //usleep(100);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            clock_gettime(CLOCK_MONOTONIC, &tend);
            printf("%g    amplitude %g  phase %g fmresult %g freq %g\n", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec), amplitude1, phase1, fmresult, /*phs*//*amplitude*/freq);
         }

         librp_SetPLL(&rp, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0);
     }
      else if (todo==-4) { //watch what frequency is doing when we change phase setpoint

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0.7, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         usleep(2000000);

         phav = aav = 0;
         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            aav += amplitude1;
            usleep(2000);
         }
         phav /= 100.0;
         aav /= 100.0;
         fprintf(stderr, "# amplitude is %g\n", aav);

         fmp = 0.0;//about 0.1 fine
         fmi = 0.0;//about 0.05 fine
         fmd = 0;
         amp = 0.005;
         ami = 0.04;
         amd = 0;
         phon = 0;
         phase = phav;
         amplitude = aav;
         printf("# Setting setpoint to %g\n", phav);
             //SetPLL(librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pidskip, int pll_input);
         librp_SetPLL(&rp, 0, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 6, 4, 0, 0); //was 7 7 0 0

         usleep(20000);
         printf("# Setting fm feedback on\n");
         librp_SetPLL(&rp, phon, 1, phav, amplitude, fmp, fmi, fmd, amp, ami, amd, 6, 4, 0, 0); //1:15 Hz, 2: 30 Hz

         clock_gettime(CLOCK_MONOTONIC, &tstart);
         for (i=0; i<200000; i++)
         {
            if ((i%4000==0)) {
               amplitude = aav-((double)i)/1e6 + 0.15;
               librp_SetPLL(&rp, phon, 1, phase, amplitude, fmp, fmi, fmd, amp, ami, amd, 6, 4, 0, 0);

            } 
            else if ((i-2000)%4000==0) {
               amplitude = aav-((double)i)/1e6 - 0.15;
               librp_SetPLL(&rp, phon, 1, phase, amplitude, fmp, fmi, fmd, amp, ami, amd, 6, 4, 0, 0);
            } 
    
            usleep(100);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadDebug(&rp, &buf, &fmerror);
            clock_gettime(CLOCK_MONOTONIC, &tend);
            printf("%g    amplitude %g  phase %g fmresult %g fmerror %g amp %g amresult %g\n", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec), amplitude1, phase1, fmresult, fmerror, amplitude, amresult);
         }

         librp_SetPLL(&rp, 0, 0, phav, 0.1, fmp, fmi, fmd, 0, 0, 0, 0, 0, 1, 0);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0);
     }
         else if (todo==0) { //move and watch what phase is doing

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         dacmax = 10.0;
         usleep(2000000);

         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            usleep(2000);
         }
         phav /= 100.0;
         printf("# phase is %g\n", phav);
         fprintf(stderr, "# phase is %g\n", phav);

         for (i=0; i<60000; i++)
         {
            dac = (double)i/6000.0;
            if (dac>dacmax) break;

            lrdac[0] = dac;
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            usleep(2000);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            printf("%g    amplitude %g  phase %g\n", dac, amplitude1, phase1);

            if (fabs(phase1-phav)>0.2) {
                fprintf(stderr, "# phase threshold stop at %g\n", dac);
                break;
            }
         }
         for (i=0; i<80000; i++)
         {
            if ((i%4000)==0) {
               lrdac[0] = dac + 0.05;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            else if (((i-2000)%4000)==0) {
               lrdac[0] = dac;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            
            usleep(500);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            printf("%g    amplitude %g  phase %g\n", lrdac[0], amplitude1, phase1);
         } 
         do {
            dac -= 0.0001;
            lrdac[0] = dac;
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

            usleep(500);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            printf("%g    amplitude %g  phase %g\n", lrdac[0], amplitude1, phase1);
          } while (dac>0);


         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
 
    }
     else if (todo==1) { //move and watch what phase is doing at frequency changes

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         dacmax = 10.0;
         usleep(2000000);

         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            usleep(2000);
         }
         phav /= 100.0;
         printf("# phase is %g\n", phav);
         fprintf(stderr, "# phase is %g\n", phav);

         for (i=0; i<10000; i++)
         {
            dac = (double)i/1000.0;
            if (dac>dacmax) break;

            lrdac[0] = dac;
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            usleep(2000);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            printf("%g    amplitude %g  phase %g\n", freq, amplitude1, phase1);

            if (fabs(phase1-phav)>0.1) {
                fprintf(stderr, "# phase threshold stop at %g\n", dac);
                break;
            }
         }
         for (i=0; i<14000; i++)
         {
            if (i==4000 || i==8000 || i==12000) {
               freq = setfreq - 40;
               librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);
            } 
            else if (i==6000 || i==10000) {
               freq = setfreq + 40;
               librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);
            } 
            
            usleep(500);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            printf("%g    amplitude %g  phase %g\n", freq, amplitude1, phase1);
         }

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
     }
     else if (todo==2) { //move and watch what frequency is doing

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 2.0, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         dacmax = 10.0;
         usleep(2000000);

         phav = aav = 0;
         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            aav += amplitude1;
            usleep(2000);
         }
         phav /= 100.0;
         aav /= 100.0;
         printf("# phase is %g\n", phav);
         fprintf(stderr, "# phase is %g\n", phav);

         fmp = 0.1;
         fmi = 0.05;
         fmd = 0;
         amp = 0.0;
         ami = 0.4;
         amd = 0;
         printf("# Setting setpoint to %g\n", phav);
             //SetPLL(librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pidskip, int pll_input);
         librp_SetPLL(&rp, 0, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 6, 6, 0, 0);

         usleep(20000);
         printf("# Setting fm feedback on\n");
         librp_SetPLL(&rp, 1, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 6, 6, 0, 0);

         for (i=0; i<60000; i++)
         {
            dac = (double)i/6000.0;
            if (dac>dacmax) break;

            lrdac[0] = dac;
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            usleep(1000);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            printf("%g    amplitude %g  phase %g fmresult %g fmerror %g\n", dac, amplitude1, phase1, fmresult, fmerror);

            if (fabs(fmresult)>45) {
                fprintf(stderr, "# frequency shift threshold stop at %g\n", dac);
                break;
            }
         }
         dac -= 0.03;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
         usleep(100000); 

         for (i=0; i<80000; i++)
         {
            if ((i%4000)==0) {
               lrdac[0] = dac + 0.02;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            else if (((i-2000)%4000)==0) {
               lrdac[0] = dac - 0.02;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            
            usleep(100);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadDebug(&rp, &buf, &fmerror);
            printf("%g    amplitude %g  phase %g fmresult %g fmerror %g\n", lrdac[0], amplitude1, phase1, fmresult, fmerror);
         }


         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
         librp_SetPLL(&rp, 0, 0, phav, 0.1, fmp, fmi, fmd, 0, 0, 0, 0, 0, 1, 0);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0);
     }
     else if (todo==3) { //move and watch what piezo is doing

         freq = setfreq;
         printf("# Setting frequency to %g\n", freq);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 2, 0);

         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);

         dacmax = 9.0;
         usleep(2000000);

         phav = aav = 0;
         for (i=0; i<100; i++) 
         {
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            phav += phase1;
            aav += amplitude1;
            usleep(2000);
         }
         phav /= 100.0;
         aav /= 100.0;
         printf("# phase is %g\n", phav);
         fprintf(stderr, "# phase is %g\n", phav);

         //frekvence by mela byt spis nizsi
         //so far best: fmpid 0 0.2 0 skip 0!   pid 0 0.2 0, skip 2 bitshift 8, fmthreshold 30
         fmp = 0.05; //0.02 so far best 
         fmi = 0.05;  //0.1 so far best
         fmd = 0;
         amp = 0.0;
         ami = 0.1;
         amd = 0;
         fmthreshold = 160; //30-50 almost no impact, the set or reported values are somehow wrong by factor of 4, not in hwseerver!
         printf("# Setting setpoint to %g\n", phav);
             //SetPLL(librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pidskip, int pll_input);
         librp_SetPLL(&rp, 0, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 6, 2, 0, 0); //4 3 so far best

         usleep(20000);
         printf("# Setting fm feedback on\n");
         librp_SetPLL(&rp, 1, 0, phav, aav, fmp, fmi, fmd, amp, ami, amd, 6, 2, 0, 0); //4 3 so far best
         librp_SetSetpoint(&rp, fmthreshold, 0);
         librp_SetPid(&rp, 0.0, 0.01, 0, 2); //z piezo pid  0.01 so far best
         for (i=0; i<100; i++)
         {
            lrdac[0] = 0;
            usleep(2000);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadResults(&rp, &error, &zpiezo, 0);
            librp_ReadHRDAC(&rp, &zpiezo);
            librp_ReadDebug(&rp, &buf, &fmerror);
            printf("%g    amplitude %g  phase %g fmresult %g zpiezo %g error %g fmerror %g\n", lrdac[0], amplitude1, phase1, fmresult, zpiezo, error, fmerror);
         }

         printf("# Setting zpiezo feedback on\n");
         librp_SetState(&rp, LIBRP_MODE_FM, 1, 0, 4, 4, 1, 0, 1);  
         usleep(1000000);
         librp_ReadResults(&rp, &error, &zpiezo, 0);
         librp_ReadHRDAC(&rp, &zpiezoinit);
         fprintf(stderr, "# initial zpiezo value %g error %g\n", zpiezoinit, error);
         usleep(1000000);
         librp_ReadResults(&rp, &error, &zpiezo, 0);
         librp_ReadHRDAC(&rp, &zpiezoinit);
         fprintf(stderr, "# initial zpiezo value %g error %g\n", zpiezoinit, error);
   
         for (i=0; i<10000; i++)
         {
            dac = (double)i/1000.0;
            if (dac>dacmax) break;

            lrdac[0] = dac;
            librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            usleep(2000);

            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadResults(&rp, &error, &zpiezo, 0);
            librp_ReadHRDAC(&rp, &zpiezo);
            librp_ReadDebug(&rp, &buf, &fmerror);
            printf("%g    amplitude %g  phase %g fmresult %g zpiezo %g error %g fmerror %g\n", lrdac[0], amplitude1, phase1, fmresult, zpiezo, error, fmerror);

            if (fabs(zpiezo)<5) {
                fprintf(stderr, "# zpiezo threshold stop at %g\n", zpiezo);
                break;
            }
         }


         dac -= 0.1;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
 
         for (i=0; i<80000; i++)
         {
            if ((i%4000)==0) {
               lrdac[0] = dac + 0.1;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            else if (((i-2000)%4000)==0) {
               lrdac[0] = dac - 0.1;
               librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
            } 
            
            usleep(400);
 
            librp_ReadLockin(&rp, LIBRP_CH_1, &amplitude1, &phase1);
            librp_ReadPLLResult(&rp, &fmresult, &amresult); //sproduct
            librp_ReadResults(&rp, &error, &zpiezo, 0);
            librp_ReadHRDAC(&rp, &zpiezo);
            librp_ReadDebug(&rp, &buf, &fmerror);
            printf("%g    amplitude %g  phase %g fmresult %g zpiezo %g error %g fmerror %g\n", lrdac[0], amplitude1, phase1, fmresult, zpiezo, error, fmerror);

        }


         dac = 0;
         lrdac[0] = dac;
         librp_SPICycle(&rp, 0, 0, 0, lrdac, adc, 1, 1, muxA, muxB);
         librp_SetPLL(&rp, 0, 0, phav, 0.1, fmp, fmi, fmd, 0, 0, 0, 0, 0, 1, 0);
         librp_SetGen(&rp, LIBRP_CH_1, freq, 0, 0);

     }


      if (!librp_Close(&rp)) printf("RP API closed\n");
      else {
          fprintf(stderr, "Error: cannot stop RP API\n");
     
      }

      return 0;
}

