/*
 *  cmirp: a library for controlling FPGA DSP
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

#ifndef CMIRP
#define CMIRP

#include <linux/spi/spidev.h>
#include <linux/types.h>

typedef unsigned char byte;

#define LIBRP_OK 0
#define LIBRP_SPI_ALREADY_INIT 1

//FPGA feedback regime
typedef enum
{   
    LIBRP_MODE_OFF      = 0,
    LIBRP_MODE_IN1      = 1,
    LIBRP_MODE_IN2      = 2,
    LIBRP_MODE_A1       = 3,
    LIBRP_MODE_A2       = 4,
    LIBRP_MODE_P1       = 5,
    LIBRP_MODE_P2       = 6,
    LIBRP_MODE_FM       = 7,
    LIBRP_MODE_PEAK1    = 8,
    LIBRP_MODE_PEAK2    = 9
} librpMode;

//FPGA output channel signal
typedef enum
{
    LIBRP_RPDAC_EXC1    = 0,
    LIBRP_RPDAC_EXC2    = 1,
    LIBRP_RPDAC_DAC1    = 2,
    LIBRP_RPDAC_DAC2    = 3,
    LIBRP_RPDAC_RES     = 4,
    LIBRP_RPDAC_OFF     = 5,
    LIBRP_RPDAC_EXCS    = 6
} librpRPDACRule;

//FPGA HR output channel signal
typedef enum
{
    LIBRP_HRDAC_EXC1    = 0,
    LIBRP_HRDAC_EXC2    = 1,
    LIBRP_HRDAC_DAC     = 2,
    LIBRP_HRDAC_RES     = 4,
    LIBRP_HRDAC_OFF     = 5,
    LIBRP_HRDAC_EXCS    = 6
} librpHRDACRule;

//HRDAC regime, choose what drives it
typedef enum
{
    LIBRP_HRDAC_REGIME_FPGA    = 0,
    LIBRP_HRDAC_REGIME_CPU     = 1,
    LIBRP_HRDAC_REGIME_MIX     = 2
} librpHRDACRegime;


//16bit slow DAC output rules, now implemented in hwserver
typedef enum {
    LIBRP_RULE_NOTHING     = 0,
    LIBRP_RULE_X           = 1,
    LIBRP_RULE_Y           = 2,
    LIBRP_RULE_Z           = 3,
    LIBRP_RULE_IN1         = 4,
    LIBRP_RULE_IN2         = 5,
    LIBRP_RULE_IN3         = 6,
    LIBRP_RULE_IN4         = 7,
    LIBRP_RULE_IN5         = 8,
    LIBRP_RULE_IN6         = 9,
    LIBRP_RULE_IN7         = 10,
    LIBRP_RULE_IN8         = 11,
    LIBRP_RULE_IN9         = 12,
    LIBRP_RULE_IN10        = 13,
    LIBRP_RULE_IN11        = 14,
    LIBRP_RULE_IN12        = 15,
    LIBRP_RULE_IN13        = 16,
    LIBRP_RULE_IN14        = 17,
    LIBRP_RULE_IN15        = 18,
    LIBRP_RULE_IN16        = 19,
    LIBRP_RULE_ERROR       = 20,
    LIBRP_RULE_ZPIEZO      = 21,
    LIBRP_RULE_AMPLITUDE1  = 22,
    LIBRP_RULE_PHASE1      = 23,
    LIBRP_RULE_AMPLITUDE2  = 24,
    LIBRP_RULE_PHASE2      = 25,
    LIBRP_RULE_FMRESULT    = 26,
    LIBRP_RULE_AMRESULT    = 27,
    LIBRP_RULE_ADC1        = 28,
    LIBRP_RULE_ADC2        = 29
} librpOutRule;

//quantity to be ramped
typedef enum
{
    LIBRP_RAMP_Z           = 0,
    LIBRP_RAMP_OUT1        = 1,
    LIBRP_RAMP_OUT2        = 2,
    LIBRP_RAMP_OUT3        = 3,
    LIBRP_RAMP_OUT4        = 4,
    LIBRP_RAMP_OUT5        = 5,
    LIBRP_RAMP_OUT6        = 6,
    LIBRP_RAMP_OUT7        = 7,
    LIBRP_RAMP_OUT8        = 8,
    LIBRP_RAMP_OUT9        = 9,
    LIBRP_RAMP_OUT10        = 10,
    LIBRP_RAMP_OUT11        = 11,
    LIBRP_RAMP_OUT12        = 12,
    LIBRP_RAMP_OUT13        = 13,
    LIBRP_RAMP_OUT14        = 14,
    LIBRP_RAMP_OUT15        = 15,
    LIBRP_RAMP_OUT16        = 16,
    LIBRP_RAMP_TIME         = 17
} librpRampQuantity;

//20bit DAC range, must match the pins
typedef enum
{
    LIBRP_RANGE_FULL   = 0, //+-10 V
    LIBRP_RANGE_SMALL  = 1 //0-10 V
} librpHRDACRange;

//KPFM operation regime, now implemented in hwserver
typedef enum
{
    LIBRP_KPFM_OFF        = 0,
    LIBRP_KPFM_AM_MANUAL  = 1,
    LIBRP_KPFM_FM_MANUAL  = 2,
    LIBRP_KPFM_AM         = 3,
    LIBRP_KPFM_FM         = 4,
    LIBRP_KPFM_SIDEBAND_MANUAL   = 5,  //excitation at low freq, detection a resonance +-lowfreq
    LIBRP_KPFM_SIDEBAND          = 6
} librpKPFM;

//DART operation regime, now implemented in hwserver
typedef enum
{
    LIBRP_DART_OFF        = 0,
    LIBRP_DART_MANUAL     = 1,
    LIBRP_DART_ON         = 2
} librpDART;


typedef struct
{
    int memfd;

    void *cfg0;  //input: l1_amplitude, l1_phase                                                 0x4200_0000
    void *cfg1;  //input: result1, result2                                                       0x4120_0000
    void *cfg2;  //input: adc, zslow                                                             0x4121_0000
    void *cfg3;  //input: l2_amplitude, l2_phase                                                 0x4122_0000
    void *cfg4;  //input: fmresults, debug2                                                      0x4123_0000   druhy vracen na debug1 v signal_decoder
    void *cfg5;  //output: setpoint1, daczhr                                                     0x4124_0000
    void *cfg6;  //output: pid1, pid2                                                            0x4125_0000
    void *cfg7;  //output: cleaner1, cleaner2                                                    0x4128_0000
    void *cfg8;  //output: fmsettings, dacz2                                                     0x4129_0000
    void *cfg9;  //output: dout1, dout2                                                          0x412A_0000
    void *cfg10; //output: state, dacz1                                                          0x4126_0000
    void *cfg11; //output: gen1, gen2                                                            0x4127_0000
    void *cfg12; //output: fm_amplitude_setpoint, pidoffset                                      0x412B_0000
    void *cfg13; //output: fm_s_p, fm_i_d                                                        0x412C_0000
    void *cfg14; //output: fmam_p_i, input: fmam_result                                          0x412D_0000
    void *cfg15; //output: gen1_offset, gen2_offset                                              0x412E_0000
    void *cfg16; //output: gen3, debug3                                                          0x412F_0000
    void *cfg17; //output: x, y                                                                  0x4130_0000
    void *cfg18; //output: l1sine, l1cosine                                                      0x4131_0000
    //void *cfg19; //input:  posx, posy unconnected                                                0x4132_0000
    //void *cfg20; //output: ntodo, input: ndone                                                   0x4133_0000
    //this seems to be maximum we can fit now

    uint32_t pins;
    uint32_t pins2;
    int32_t dac1val;
    int32_t dac2val;
    int32_t hrdacval;

    int state;   //feedback loop state
    int feedback;  //feedback on/off

    int rpdacrule1;  //rules for internal organisation of outputs on RP, should match the state
    int rpdacrule2;
    int rpdacrulehr;
    int hrdac_regime;         //use HRDAC from FPGA or CPU
    int hrdac1_range;         //range of HRDAC, check jumpers
    int hrdac2_range;
    int hrdac3_range;

    int pidskip;    //pid slowdown factor
    int pllskip;   

    int ddswidth;
    double freqfactor; 

    double dac1cal;  //conversion factors for fast RP DACs
    double dac2cal;
    double dac1shift;
    double dac2shift;

    double adc1cal;  //conversion factors for fast RP ADCs
    double adc2cal;
    double adc1shift;
    double adc2shift;

    //cleaner, signal path and lockin settings
    int debugsource1;    //sin/cos lockin output
    int debugsource2;
    int hr1;             //high resolution lock-in
    int hr2;
    int loopback1;       //use signal decoder loopback data as input
    int loopback2;
    int decimation1;     //filter decimation 0-12
    int decimation2;
    int filter_phase1;   //apply lockin output phase filter
    int filter_phase2;
    int intosc1;         //use internal auxiliary oscillator
    int intosc2;

    //generator
    double frequency1;   //generator frequency
    double frequency2;
    double amplitude1;   //generator amplitude
    double amplitude2;
    double offset1;      //generator offset
    double offset2;
    int increment1;     //increments, only for reporting
    int increment2;
    int increment3;

    //range settings
    int spi_fd;
    int hrdac_range;
    int hrdac_nchannels;
    int zpiezo_hrdac;
    int zpiezo_hrdac_range;
    int input_range1;
    int input_range2;
    int dds1_range;
    int dds2_range;
    int rp1_input_hv;
    int rp2_input_hv;
    int rp_bare_input;
    int rp_bare_output;

    int pll_input;

    double rpadc1_bare_lv_offset;
    double rpadc1_bare_lv_slope;
    double rpadc1_bare_hv_offset;
    double rpadc1_bare_hv_slope;

    double rpadc2_bare_lv_offset;
    double rpadc2_bare_lv_slope;
    double rpadc2_bare_hv_offset;
    double rpadc2_bare_hv_slope;

    double rpadc1_divhigh_lv_offset;
    double rpadc1_divhigh_lv_slope;
    double rpadc1_divhigh_hv_offset;
    double rpadc1_divhigh_hv_slope;

    double rpadc1_divlow_lv_offset;
    double rpadc1_divlow_lv_slope;
    double rpadc1_divlow_hv_offset;
    double rpadc1_divlow_hv_slope;

    double rpadc2_divhigh_lv_offset;
    double rpadc2_divhigh_lv_slope;
    double rpadc2_divhigh_hv_offset;
    double rpadc2_divhigh_hv_slope;

    double rpadc2_divlow_lv_offset;
    double rpadc2_divlow_lv_slope;
    double rpadc2_divlow_hv_offset;
    double rpadc2_divlow_hv_slope;

    double rpdac1_bare_offset;
    double rpdac1_bare_slope;
    double rpdac2_bare_offset;
    double rpdac2_bare_slope;

    double rpdac1_offset;
    double rpdac1_slope;
    double rpdac2_offset;
    double rpdac2_slope;

    double pidoffset;
 
    int swap_in;
    int swap_out;

    //lock-in evaluation phase shift
    int phaseshift1;
    int phaseshift2;

    int nwaves1; //only for reading
    int nwaves2; 

    int kpfm_mode;
    int dart_mode;

    int oversampling;

    //previous 20bit values to save calls
    unsigned long old_hr1;
    unsigned long old_hr2;
    unsigned long old_hr3;

    //previous slow DAC values to save calls
    unsigned long old_lr[16];

    //previous multiplexer values to save calls
    int old_muxA;
    int old_muxB;

    int debug;
    int check;

} librpSet;

typedef enum {
    LIBRP_CH_1 = 1,
    LIBRP_CH_2 = 2
} librpChannel;

//general functions
int librp_Init            (librpSet *rpset, int hrdac_regime, int hrdac1_range, int hrdac2_range, int hrdac3_range, 
                           int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
                           int rp1_input_hv, int rp2_input_hv, int rp_bare_output, int rp_bare_input);
int librp_Init_SPI_devices(librpSet *rpset);
int librp_SetInputRange   (librpSet *rpset, int range1, int range2);
int librp_SetDDSRange     (librpSet *rpset, int range1, int range2);
int librp_SetRPADCCal     (librpSet *rpset, double offset, double slope);
int librp_SetOversampling (librpSet *rpset, int oversampling);
int librp_Close           (librpSet *rpset);

//read pid and lockin results
int librp_ReadResults     (librpSet *rpset, double *error, double *dac, int raw);
int librp_ReadLockin      (librpSet *rpset, int channel, double *amplitude, double *phase);
int librp_ReadLockinAxes  (librpSet *rpset, int channel, double *x, double *y);

//read extra results
int librp_ReadADC         (librpSet *rpset, double *adc1, double *adc2);
int librp_ReadHRDAC       (librpSet *rpset, double *dac);
int librp_ReadDebug       (librpSet *rpset, double *debug, double *debug3);
int librp_ReadPLLResult   (librpSet *rpset, double *pllresult, double *ampresult);

//set channel parameters and output values
int librp_SetSignalProcessing(librpSet *rpset, int channel, int decimation, int loopback, int hr, int phaseshift, int debugsource, int filter_amplitude, int filter_phase, int nwaves, int lfr, int intosc);
int librp_SetPhaseShift   (librpSet *rpset, int channel, int shift);
int librp_SetGen          (librpSet *rpset, int channel, double frequency, double amplitude, double offset);
int librp_SetGenIncrement (librpSet *rpset, int channel, int increment);
int librp_SetDAC          (librpSet *rpset, int channel, double value);
int librp_SetHRDAC1       (librpSet *rpset, double value);
int librp_SetHRDAC2       (librpSet *rpset, double value);
int librp_SetHRDAC3       (librpSet *rpset, double value);
int librp_SetPidOffset    (librpSet *rpset, double value);
double librp_GetPidOffset (librpSet *rpset);
int librp_SetAuxGen       (librpSet *rpset, double frequency);
int librp_SetAuxGenIncrement (librpSet *rpset, int increment);

//convenience function to set KPFM parameters
int librp_SetKPFM         (librpSet *rpset, int mode, double frequency, double frequency_generate_aux, double amplitude, double p, double i, double d, double offset, int filter, int filter_amplitude, int filter_phase, int nwaves, int lfr, int init);

//function to get a suitable sideband KPFM frequency
double librp_GetKPFMSidebandFreq (librpSet *rpset, double desiredfreq, int *sincrement1, int *sincrement2, int *sincrement3);

//convenience function to set DART
int librp_SetDART         (librpSet *rpset, int mode, double frequency, double amplitude, double p, double i, double d, double freqspan, int filter, int lfr);

//set pid parameters
int librp_SetState        (librpSet *rpset, int state, int feedback, int out1, int out2, int outhr, int swap_in, int swap_out, int pidskip);
int librp_SetFeedback     (librpSet *rpset, int feedback);
int librp_SetSetpoint     (librpSet *rpset, double setpoint, int raw);
int librp_SetPid          (librpSet *rpset, double p, double i, double d, int errorshift);
int librp_SetPLL          (librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pidskip, int pll_input);

//set extra
int librp_DPinSetState    (librpSet *rpset, int pin, int on);
int librp_D2PinSetState   (librpSet *rpset, int pin, int on);
int librp_DMultiPinSetState(librpSet *rpset, int npin, ...);

//calibrate inputs
int librp_LoadCalData(librpSet *rpset,
                     double rpadc1_bare_lv_offset, double rpadc1_bare_lv_slope, double rpadc1_bare_hv_offset, double rpadc1_bare_hv_slope,
                     double rpadc2_bare_lv_offset, double rpadc2_bare_lv_slope, double rpadc2_bare_hv_offset, double rpadc2_bare_hv_slope,
                     double rpadc1_divhigh_lv_offset, double rpadc1_divhigh_lv_slope, double rpadc1_divhigh_hv_offset, double rpadc1_divhigh_hv_slope, 
                     double rpadc1_divlow_lv_offset, double rpadc1_divlow_lv_slope, double rpadc1_divlow_hv_offset, double rpadc1_divlow_hv_slope,
                     double rpadc2_divhigh_lv_offset, double rpadc2_divhigh_lv_slope, double rpadc2_divhigh_hv_offset, double rpadc2_divhigh_hv_slope,
                     double rpadc2_divlow_lv_offset, double rpadc2_divlow_lv_slope, double rpadc2_divlow_hv_offset, double rpadc2_divlow_hv_slope,
                     double rpdac1_bare_offset, double rpdac1_bare_slope, double rpdac2_bare_offset, double rpdac2_bare_slope,
                     double rpdac1_offset, double rpdac1_slope, double rpdac2_offset, double rpdac2_slope);

//read and write all the spi data
int librp_SPICycle(librpSet *rpset, double hrdac1, double hrdac2, double hrdac3, double *lrdac, double *adc, int read_adc1, int read_adc2, int mux1, int mux2);

int configureEXP(librpSet *rpset);
int select_EXP(librpSet *rpset);
int change_spi_mode(librpSet *rpset, uint8_t mode);

#endif


