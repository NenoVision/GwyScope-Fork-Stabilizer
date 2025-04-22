/*
 * cal_outputs: a simple tool for determination of RP outputs calibration coefficients
 *
 * Voltages are generated using Red Pitaya's DACs (Neno: Excitation and Bias monitor) and they are read by 18b ADC in ch3 (Neno: input4) and ch4 (Neno:input5).
 * Therefore a wire connection between these two channels must exist.
 * 
 * Next, the parameter describing the actual HW configuration must be set. In the following is the case when
 * RP is connected to CMIPITAYA board.
 * hw.rp_bare_output = 0;
 * 
 * Calibration coefficients (such as rpdac1_offset) are set in some predefined values. If they are correct, then
 * the fitting results saved in data*.fit files should show a=819 (or a=8191 for bare outputs) and b=0. If a new calibration is required the respective slope
 * and offset coefficients should be set to 819.1 (or 8191 for bare outputs) and 0, respectively and a subsequent running of the program shows new coefficients.
 * Manual tuning of coefficients might be required until the desired precision is reached.
 * 
 * The fitting is acomplished using Gnuplot. Use "apt-get install gnuplot" to install it before running this tool.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "cmirp.h"
#include "data.h"
#include "gwyfile.h"

int main(int argc, char *argv[]){

    HWData hw;;
    int i, j, read_adc1, read_adc2, ret, muxA, muxB;
    double lrdac[16], adc[16], hrdac1 =0.0 , hrdac2 = 0.0, hrdac3 = 0.0, value;
    FILE *file;

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

    file = fopen("data_out.txt", "w");
    if (!file){
        fprintf(stderr, "Couldn't open data_out.txt for writing!\n");
        return -1;
    }

    if (hw.rp_bare_output == 1){
        fprintf(file, "# rp: dac1 dac2 adc16ch: ch1 ch2; bare output: %d\n", hw.rp_bare_output);
        for (i=0; i<=200; i++){
            value = -1.0 + 0.01*(double)i;
            librp_SetDAC(&hw.rp, LIBRP_CH_1, value);
            librp_SetDAC(&hw.rp, LIBRP_CH_2, value);

            for (j=0; j<10; j++){
                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                fprintf(file, "%lf %lf %lf %lf\n", value, value, adc[3], adc[4]);

                //usleep(1000);
            }
        }
        fclose(file);
        system("gnuplot < fit_data_bare_out.gpl");

        if (!librp_Close(&hw.rp))
            printf("# RP API closed\n");
        else 
            fprintf(stderr, "Error: cannot stop RP API\n");

        return 0;

    }
    else{
        fprintf(file, "# rp: dac1 dac2 adc16ch: ch1 ch2; bare output: %d\n", hw.rp_bare_output);
        for (i=0; i<=2000; i++){
            value = -10.0 + 0.01*(double)i;
            librp_SetDAC(&hw.rp, LIBRP_CH_1, value);
            librp_SetDAC(&hw.rp, LIBRP_CH_2, value);

            for (j=0; j<10; j++){
                if ((ret=librp_SPICycle(&hw.rp, hrdac1, hrdac2, hrdac3, lrdac, adc, read_adc1, read_adc2, muxA, muxB)) != LIBRP_OK)
                    fprintf(stderr, "Error: SPICycle returned %d\n", ret);

                fprintf(file, "%lf %lf %lf %lf\n", value, value, adc[3], adc[4]);

                //usleep(1000);
            }
        }
    }
    fclose(file);
    system("gnuplot < fit_data_out.gpl");

    if (!librp_Close(&hw.rp))
        printf("# RP API closed\n");
    else 
        fprintf(stderr, "Error: cannot stop RP API\n");

    return 0;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 */
