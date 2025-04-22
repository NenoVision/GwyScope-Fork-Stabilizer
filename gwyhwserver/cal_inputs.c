/*
 * cal_inputs: a simple tool for determination of RP inputs calibration coefficients
 *
 * A voltage is generated using 16b DAC in ch0 and is read by both RP inputs and 18b ADC in ch0.
 * Therefore a wire connection between these two channels must exist and both multiprexers 
 * must be se to 1 (muxA = muxB = 1).
 * 
 * If outputs testing is desired (TEST_OUTPUTS is defined), another two wires are necessary, connect RP_FAST_DAC1 to 18b ADC ch1 and
 * RP_FAST_DAC2 to 18b ADC ch2.
 * 
 * Next, the parameters describing the actual HW configuration must be set. In the following is the case when
 * both RP inputs jumpers are set to LV and RP is connected to CMIPITAYA board.
 * hw.rp1_input_hv = 0;
 * hw.rp2_input_hv = 0;
 * hw.rp_bare_output = 0;
 * hw.rp_bare_input = 0;
 * 
 * Calibration coefficients (such as rpadc1_divhigh_lv_offset) are set in some predefined values. If they are correct, then
 * the fitting results saved in data*.fit files should show a=1 and b=0. If a new calibration is required the respective slope
 * and offset coefficients should be set to 1 and 0, respectively and a subsequent running of the program shows new coefficients.
 * 
 * The fitting is acomplished using Gnuplot. Use "apt-get install gnuplot" to install it before running this tool.
 * 
 * Testing of RP outputs is done in two passes, in the first pass low frequency signals are generated which are then sampled using
 * the two channels of 18b ADC. This can be used for verification of parameters used for signal generation. In the second pass, high
 * freq. signals are used for testing of lock-in amplitude.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>

#include "cmirp.h"
#include "data.h"
#include "gwyfile.h"

#define TEST_OUTPUTS

int main(int argc, char *argv[]){

    HWData hw;;
    int i, j, read_adc1, read_adc2, ret, muxA, muxB;
    double lrdac[16], adc[16], hrdac1 =0.0 , hrdac2 = 0.0, hrdac3 = 0.0, adc1, adc2;
    double amp1, phase1, amp2, phase2, timestamp;
    FILE *file;
    struct timespec now;

    //init calibration values, with NenoVision note for finalisation
    hw.rpadc1_bare_lv_offset = 0.0; // Do not edit
    hw.rpadc1_bare_lv_slope = 1.0; // Do not edit //8191; 
    hw.rpadc1_bare_hv_offset = 0.0; // Do not edit
    hw.rpadc1_bare_hv_slope = 1.0; // Do not edit //410;

    hw.rpadc2_bare_lv_offset = 0.0; // Do not edit
    hw.rpadc2_bare_lv_slope = 1.0; // Do not edit //8191; 
    hw.rpadc2_bare_hv_offset = 0.0; // Do not edit
    hw.rpadc2_bare_hv_slope = 1.0; // Do not edit //410;

    hw.rpadc1_divhigh_lv_offset = 0.0; //First "b" parameter 
    hw.rpadc1_divhigh_lv_slope = 1.0; //First "a" parameter, typical is number 700.0-819.1;
    hw.rpadc1_divhigh_hv_offset = 0.0;// Do not edit
    hw.rpadc1_divhigh_hv_slope = 1.0; // Do not edit //41;

    hw.rpadc1_divlow_lv_offset = 0.0; //Third "b" parameter 
    hw.rpadc1_divlow_lv_slope = 1.0; //Third "a" parameter, typical is number 7000.0-8191.0
    hw.rpadc1_divlow_hv_offset = 0.0; // Do not edit
    hw.rpadc1_divlow_hv_slope = 1.0; // Do not edit //410;

    hw.rpadc2_divhigh_lv_offset = 0.0; //Second "b" parameter
    hw.rpadc2_divhigh_lv_slope = 1.0; //Second "a" parameter, typical is number 700.0-819.1
    hw.rpadc2_divhigh_hv_offset = 0.0; // Do not edit
    hw.rpadc2_divhigh_hv_slope = 1.0; // Do not edit //41;

    hw.rpadc2_divlow_lv_offset = 0.0; //Fourth "b" parameter
    hw.rpadc2_divlow_lv_slope = 1.0; //Fourth "a" parameter, typical is number 7000.0-8191.0
    hw.rpadc2_divlow_hv_offset = 0.0; // Do not edit
    hw.rpadc2_divlow_hv_slope = 1.0; // Do not edit //410;

    hw.rpdac1_bare_offset = 0; // Do not edit
    hw.rpdac1_bare_slope = 8191; // Do not edit
    hw.rpdac2_bare_offset = 0; // Do not edit
    hw.rpdac2_bare_slope = 8191; // Do not edit

    hw.rpdac1_offset = 0; //First "b" parameter from cal_outputs
    hw.rpdac1_slope = 819.1; //First "a" parameter from cal_outputs typical is number 700.0-819.1
    hw.rpdac2_offset = 0; //Second "b" parameter from cal_outputs
    hw.rpdac2_slope = 819.1; //Second "a" parameter from cal_outputs typical is number 700.0-819.1

    hw.rp.input_range1 = 1;
    hw.rp.input_range2 = 1;
    hw.rp1_input_hv = 0;
    hw.rp2_input_hv = 0;
    hw.rp_bare_output = 0;
    hw.rp_bare_input = 0;

    //load the bitstream
    printf("# Loading the bitstream... ");
    fflush(stdout);
    system("cat system_wrapper.bit > /dev/xdevcfg");
    printf("done.\n");

    printf("# starting RP API...\n");
    /*
       int librp_Init(librpSet *rpset, int hrdac_regime, int hrdac1_range, int hrdac2_range, int hrdac3_range,
       int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
       int rp1_input_hv, int rp2_input_hv, int rp_bare_output, int rp_bare_input)
     */
    if (!librp_Init(&hw.rp, LIBRP_HRDAC_REGIME_FPGA, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, hw.rp.input_range1, hw.rp.input_range2, 0, 0, 6, hw.rp1_input_hv, hw.rp2_input_hv, hw.rp_bare_output, hw.rp_bare_input))
        printf("# init done.\n");
    else{
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
    }

    librp_LoadCalData(&hw.rp,
                      hw.rpadc1_bare_lv_offset, hw.rpadc1_bare_lv_slope, hw.rpadc1_bare_hv_offset, hw.rpadc1_bare_hv_slope,
                      hw.rpadc2_bare_lv_offset, hw.rpadc2_bare_lv_slope, hw.rpadc2_bare_hv_offset, hw.rpadc2_bare_hv_slope,
                      hw.rpadc1_divhigh_lv_offset, hw.rpadc1_divhigh_lv_slope, hw.rpadc1_divhigh_hv_offset, hw.rpadc1_divhigh_hv_slope,
                      hw.rpadc1_divlow_lv_offset, hw.rpadc1_divlow_lv_slope, hw.rpadc1_divlow_hv_offset, hw.rpadc1_divlow_hv_slope,
                      hw.rpadc2_divhigh_lv_offset, hw.rpadc2_divhigh_lv_slope, hw.rpadc2_divhigh_hv_offset, hw.rpadc2_divhigh_hv_slope,
                      hw.rpadc2_divlow_lv_offset, hw.rpadc2_divlow_lv_slope, hw.rpadc2_divlow_hv_offset, hw.rpadc2_divlow_hv_slope,
                      hw.rpdac1_bare_offset, hw.rpdac1_bare_slope, hw.rpdac2_bare_offset, hw.rpdac2_bare_slope,
                      hw.rpdac1_offset, hw.rpdac1_slope, hw.rpdac2_offset, hw.rpdac2_slope);

    read_adc1 = 1;  //should we read adc 1-8?
    read_adc2 = 1;  //should we read adc 9-16?
    muxA = 1;  //mux A (RP1 IN1) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX1
    muxB = 1;  //mux B (RP1 IN2) setting; 1-16 to select ADC_MUX1-ADC_MUX16; default ADC_MUX2

    file = fopen("data.txt", "w");
    if (!file){
        fprintf(stderr, "Couldn't open data.txt for writing!\n");
        return -1;
    }

    if (hw.rp_bare_input == 1){
        fprintf(file, "# rp: adc1 adc2 adc16ch: ch0; bare input: %d\n", hw.rp_bare_input);
        if ((hw.rp1_input_hv == 1) && (hw.rp2_input_hv == 1)){
            for (i=0; i<=2000; i++){
                lrdac[0] = -10.0 + 0.01*(double)i;

                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                //usleep(10000);

                for (j=0; j<10; j++){
                    if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                        fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                    librp_ReadADC(&hw.rp, &adc1, &adc2);
                    fprintf(file, "%lf %lf %lf\n", adc1, adc2, adc[0]);

                    //usleep(1000);
                }
            }
        }
        else{
            for (i=0; i<=200; i++){
                lrdac[0] = -1.0 + 0.01*(double)i;

                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                //usleep(10000);

                for (j=0; j<10; j++){
                    if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                        fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                    librp_ReadADC(&hw.rp, &adc1, &adc2);
                    fprintf(file, "%lf %lf %lf\n", adc1, adc2, adc[0]);

                    //usleep(1000);
                }
            }
        }
        fclose(file);
        system("gnuplot < fit_data_bare.gpl");

        if (!librp_Close(&hw.rp))
            printf("# RP API closed\n");
        else 
            fprintf(stderr, "Error: cannot stop RP API\n");

        return 0;

    }
    else{
        fprintf(file, "# rp: adc1 adc2 adc16ch: ch0; bare input: %d input_range1: %d input_input2: %d\n", hw.rp_bare_input, hw.rp.input_range1, hw.rp.input_range2);
        for (i=0; i<=2000; i++){
            lrdac[0] = -10.0 + 0.01*(double)i;

            if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                fprintf(stderr, "Error: SPICycle returned %d\n", ret);

            //usleep(10000);

            for (j=0; j<10; j++){
                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                librp_ReadADC(&hw.rp, &adc1, &adc2);
                fprintf(file, "%lf %lf %lf\n", adc1, adc2, adc[0]);

                //usleep(1000);
            }
        }

        hw.rp.input_range1 = 0;
        hw.rp.input_range2 = 0;
        librp_SetInputRange(&hw.rp, hw.rp.input_range1, hw.rp.input_range2);

        fprintf(file, "\n\n# rp: adc1 adc2 adc16ch: ch0; bare input: %d input_range1: %d input_input2: %d\n", hw.rp_bare_input, hw.rp.input_range1, hw.rp.input_range2);
        if ((hw.rp1_input_hv == 1) && (hw.rp2_input_hv == 1)){
            for (i=0; i<=2000; i++){
                lrdac[0] = -10.0 + 0.01*(double)i;

                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                //usleep(10000);

                for (j=0; j<10; j++){
                    if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                        fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                    librp_ReadADC(&hw.rp, &adc1, &adc2);
                    fprintf(file, "%lf %lf %lf\n", adc1, adc2, adc[0]);

                    //usleep(1000);
                }
            }
        }
        else{
            for (i=0; i<=200; i++){
                lrdac[0] = -1.0 + 0.01*(double)i;

                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                //usleep(10000);

                for (j=0; j<10; j++){
                    if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                        fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                    librp_ReadADC(&hw.rp, &adc1, &adc2);
                    fprintf(file, "%lf %lf %lf\n", adc1, adc2, adc[0]);

                    //usleep(1000);
                }
            }
        }
    }
    fclose(file);
    system("gnuplot < fit_data.gpl");

#ifdef TEST_OUTPUTS
    librp_SetDDSRange(&hw.rp, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL);
    librp_SetState(&hw.rp, LIBRP_MODE_OFF, 0, 0, 1, 5, 0, 0, 1);
    librp_SetSignalProcessing(&hw.rp, LIBRP_CH_1, 12, 0, 0, 0, 0, 0, 0, 2, 0, 0);
    librp_SetSignalProcessing(&hw.rp, LIBRP_CH_2, 12, 0, 0, 0, 0, 0, 0, 2, 0, 0);
    librp_SetGen(&hw.rp, LIBRP_CH_1, 10, 0.25, -0.25);
    librp_SetGen(&hw.rp, LIBRP_CH_2, 20, 5.0, 1.0);

    file = fopen("data_osc.txt", "w");
    if (!file){
        fprintf(stderr, "Couldn't open data_osc.txt for writing!\n");
        return -1;
    }

    hw.rp.input_range1 = 1;
    hw.rp.input_range2 = 1;
    librp_SetInputRange(&hw.rp, hw.rp.input_range1, hw.rp.input_range2);
    muxA = 2;
    muxB = 3;

    clock_gettime(CLOCK_MONOTONIC, &now);
    hw.start.tv_sec = now.tv_sec;
    hw.start.tv_nsec = now.tv_nsec;

    for (j=0; j<1000; j++){
        if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
            fprintf(stderr, "Error: SPICycle returned %d\n", ret);

        clock_gettime(CLOCK_MONOTONIC, &now);
        timestamp = ((double)now.tv_sec + (double)now.tv_nsec / 1.0e9) - ((double)hw.start.tv_sec + (double)hw.start.tv_nsec / 1.0e9);
        fprintf(file, "%lf %lf %lf\n", timestamp, adc[1], adc[2]);

        //usleep(1000);
    }

    librp_SetGen(&hw.rp, LIBRP_CH_1, 50000, 0.25, -0.25);
    librp_SetGen(&hw.rp, LIBRP_CH_2, 20000, 5.0, 1.0);
    usleep(1000);
    librp_ReadLockin(&hw.rp, LIBRP_CH_1, &amp1, &phase1);
    librp_ReadLockin(&hw.rp, LIBRP_CH_2, &amp2, &phase2);
    fprintf(stderr, "\namplitude 1 (0.25 V): %lf phase 1: %lf amplitude 2 (5 V): %lf phase 2: %lf\n\n", amp1, phase1, amp2, phase2);
#endif

    if (!librp_Close(&hw.rp))
        printf("# RP API closed\n");
    else 
        fprintf(stderr, "Error: cannot stop RP API\n");

    return 0;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 */
