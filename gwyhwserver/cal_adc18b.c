/*
 *  cal_adc18b: a simple tool for calibration of 18b ADCs
 *  Copyright (C) 2022 Miroslav Valtr, Petr Klapetek
 *  E-mail: miraval@seznam.cz.
 *
 *  It reads num_samples provided by user as fast as possible.
 *  Acquired values are printed on stdout; use redirection to output it to a file.
 *  The samplig speed is on average 2850 Sps (@ maximum oversampling).
 *  100 000 samples takes ~15 MB of disk space.
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
    int j, read_adc1 = 1, read_adc2 = 1, ret, muxA = 1, muxB = 1;
    double hrdac1 = 0.0, hrdac2 = 0.0, hrdac3 = 0.0, lrdac[16], adc[16];
    long int max;

    if (argc < 2) {
        fprintf(stderr,"ERROR, not enough parameters provided\nSyntax: %s num_samples\n", argv[0]);
        exit(1);
    }
    max = atol(argv[1]);

    //load the bitstream
    //printf("# Loading the bitstream... ");
    //fflush(stdout);
    system("cat system_wrapper.bit > /dev/xdevcfg");
    //printf("done.\n");

    //printf("# starting RP API...\n");
    if (!librp_Init(&rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, 1, 1, 0, 0, 6, 0, 0, 0, 0)){
        //printf("# init done.\n");
    }
    else{
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
    }

    librp_SetState(&rp, LIBRP_MODE_OFF, 0, 1, 1, 2, 0, 0, 1);

    for (j=0; j < max; j++){
        if ((ret=librp_SPICycle(&rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
            fprintf(stderr, "Error: SPICycle returned %d\n", ret);

        fprintf(stdout, "%lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf  %lf %lf %lf %lf\n",
                adc[0], adc[1], adc[2], adc[3], 
                adc[4], adc[5], adc[6], adc[7], 
                adc[8], adc[9], adc[10], adc[11], 
                adc[12], adc[13], adc[14], adc[15]);
        //usleep(1000000);
    }

    if (!librp_Close(&rp)){
        //printf("# RP API closed\n");
    }
    else 
        fprintf(stderr, "Error: cannot stop RP API\n");

    return 0;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 : */
