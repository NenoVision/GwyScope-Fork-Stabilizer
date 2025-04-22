/*
 *  myrpfake: implementation of librp functions not requiring a Red Pitaya board
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

#ifndef MYRP
#define MYRP

#define LIBRP_OK 0

typedef struct
{
    int memfd;

    void *cfg0;  //input: adc1, adc2                                                             0x4200_0000
    void *cfg1;  //input: dac1 (zpiezo), dac2 (dither)                                           0x4120_0000
    void *cfg2;  //input: errorsignal  output: settings [1 onoff, 2:4 mode, 16:31 digital pins]  0x4121_0000
    void *cfg3;  //output: setpoint1, pid1 [0:4 decimation, 5:9 delay]                           0x4122_0000
    void *cfg4;  //output: setpoint2, pid2 [0:4 decimation, 6:9 delay]                           0x4123_0000
    void *cfg5;  //output: amplitude, frequency                                                  0x4124_0000
    void *cfg6;  //output: zset,  aux                                                            0x4125_0000

    double **fake_z; //topography
    double **fake_a; //topography
    double **fake_b; //topography
    double **fake_dz; //topography derivative, i.e. error signal
    double fake_xreal;
    double fake_yreal;
    int fake_xres;
    int fake_yres;

    double x; 
    double y;
    double z;
    int fb_on;
    double freq;

    uint32_t settings;

} librpSet;

typedef enum
{
    LIBRP_MODE_OFF           = 0,
    LIBRP_MODE_PROPORTIONAL  = 1,
    LIBRP_MODE_NCAMPLITUDE   = 2,
    LIBRP_MODE_NCPHASE       = 3,
    LIBRP_MODE_AKIYAMA       = 4,
    LIBRP_MODE_PRSA          = 5,
    LIBRP_MODE_STM           = 6,
    LIBRP_MODE_KPFM          = 7
} librpMode;

typedef enum {
    LIBRP_RULE_NOTHING  = 0,
    LIBRP_RULE_X        = 1,
    LIBRP_RULE_Y        = 2,
    LIBRP_RULE_Z        = 3,
    LIBRP_RULE_IN1      = 4,
    LIBRP_RULE_IN2      = 5,
    LIBRP_RULE_IN3      = 6,
    LIBRP_RULE_IN4      = 7,
    LIBRP_RULE_IN5      = 8,
    LIBRP_RULE_IN6      = 9,
    LIBRP_RULE_IN7      = 10,
    LIBRP_RULE_IN8      = 11,
    LIBRP_RULE_IN9      = 12,
    LIBRP_RULE_IN10      = 13,
    LIBRP_RULE_IN11      = 14,
    LIBRP_RULE_IN12      = 15,
    LIBRP_RULE_IN13      = 16,
    LIBRP_RULE_IN14      = 17,
    LIBRP_RULE_IN15      = 18,
    LIBRP_RULE_IN16      = 19
} librpOutRule;

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
    LIBRP_RAMP_OUT16        = 16
} librpRampQuantity;

typedef enum {
    LIBRP_CH_1 = 1,
    LIBRP_CH_2 = 2
} librpChannel;


int librp_Init            (librpSet *rpset);
int librp_Close           (librpSet *rpset);
int librp_FakeAt          (librpSet *rpset, double x, double y);
int librp_ReadVals        (librpSet *rpset, double *error_signal, double *zpiezo, double *adc1, double *adc2, double *dither, double *aux, double *amplitude1, double *phase1); 
int librp_ReadAVals       (librpSet *rpset, double *error_signal, double *zpiezo, double *adc1, double *adc2, double *dither, double *aux, double *amplitude1, double *phase1, int nav, int usleeptime);
int librp_SetFeedback     (librpSet *rpset, int on);
int librp_SetMode         (librpSet *rpset, int mode);
int librp_SetPin          (librpSet *rpset, int pin, int on);
int librp_SetSetpoint     (librpSet *rpset, int channel, double setpoint, int raw);
int librp_SetPid          (librpSet *rpset, int channel, double pid_p, double pid_i, double pid_d);
int librp_SetFrequency    (librpSet *rpset, double frequency);
int librp_SetAmplitude    (librpSet *rpset, double amplitude);
int librp_SetZpiezo       (librpSet *rpset, double zpiezo);

#endif


