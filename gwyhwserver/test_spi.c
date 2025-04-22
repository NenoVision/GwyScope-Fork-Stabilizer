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
#include <math.h>
#include <stdint.h>
#include <unistd.h>

#include "cmirp.h"

int main(int argc, char *argv[]){

    librpSet rp;
    int i, j, read_adc1, read_adc2, ret, muxA, muxB;
    double hrdac1, hrdac2, hrdac3, lrdac[16], adc[16], adc1, adc2;

    //load the bitstream
    printf("# Loading the bitstream... ");
    fflush(stdout);
    system("cat system_wrapper.bit > /dev/xdevcfg");
    printf("done.\n");

    printf("# starting RP API...\n");
    if (!librp_Init(&rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, LIBRP_RANGE_SMALL, 1, 1, 0, 0, 6, 0, 0, 0, 0))
        printf("# init done.\n");
    else{
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
    }

    librp_SetState(&rp, LIBRP_MODE_OFF, 0, 1, 1, 2, 0, 0, 1);

    hrdac1 =  0;  //value for 20 bit DAC 1
    hrdac2 =  0;  //value for 20 bit DAC 2
    hrdac3 =  0;  //value for 20 bit DAC 3

    read_adc1 = 1;  //should we read adc 1-8?
    read_adc2 = 1;  //should we read adc 9-16?
    muxA = 4;  //mux A (RP1 IN1) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX1
    muxB = 5;  //mux B (RP1 IN2) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX2

    do {
	    for (i=10; i<=20; i++){
		    hrdac1 = (double)i/1.0 - 10.0;
		    hrdac2 = (double)i/1.0 - 10.0;
		    hrdac3 = (double)i/1.0 - 10.0;
		    for (j=0; j<16; j++)
			    lrdac[j] = -10.0 + 1.0*(double)i;  //values for 16 bit DAC

		    librp_SetHRDAC1(&rp, hrdac1);
		    librp_SetHRDAC2(&rp, hrdac2);
		    librp_SetHRDAC3(&rp, hrdac3);

		    if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
			    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

		    usleep(10000);

		    for (j=0; j<1; j++){
			    if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
				    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

			    librp_ReadADC(&rp, &adc1, &adc2);
			    //           printf("%lf %lf %lf %lf\n",
			    //                  adc[0], adc[1], adc[2], adc[3]);
			    printf("hrdac: %lf %lf %lf  rp: %lf %lf  adc16ch: %lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf\n",
					    hrdac1, hrdac2, hrdac3,
					    adc1, adc2,
					    adc[0], adc[1], adc[2], adc[3], 
					    adc[4], adc[5], adc[6], adc[7], 
					    adc[8], adc[9], adc[10], adc[11], 
					    adc[12], adc[13], adc[14], adc[15]);
			    usleep(1000000);
		    }
	    }
	    for (i=20; i>=10; i--){
		    hrdac1 = (double)i/1.0 - 10.0;
		    hrdac2 = (double)i/1.0 - 10.0;
		    hrdac3 = (double)i/1.0 - 10.0;
		    for (j=0; j<16; j++)
			    lrdac[j] = -10.0 + 1.0*(double)i;  //values for 16 bit DAC

		    librp_SetHRDAC1(&rp, hrdac1);
		    librp_SetHRDAC2(&rp, hrdac2);
		    librp_SetHRDAC3(&rp, hrdac3);

		    if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
			    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

		    //usleep(10000);

		    for (j=0; j<1; j++){
			    if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
				    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

			    librp_ReadADC(&rp, &adc1, &adc2);
			    //           printf("%lf %lf %lf %lf\n",
			    //                  adc[0], adc[1], adc[2], adc[3]);
			    printf("hrdac: %lf %lf %lf  rp: %lf %lf  adc16ch: %lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf\n",
					    hrdac1, hrdac2, hrdac3,
					    adc1, adc2,
					    adc[0], adc[1], adc[2], adc[3], 
					    adc[4], adc[5], adc[6], adc[7], 
					    adc[8], adc[9], adc[10], adc[11], 
					    adc[12], adc[13], adc[14], adc[15]);
			    //usleep(5000000);
		    }
	    }
    } while (1);

    if (!librp_Close(&rp))
        printf("# RP API closed\n");
    else 
        fprintf(stderr, "Error: cannot stop RP API\n");

    return 0;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 */
