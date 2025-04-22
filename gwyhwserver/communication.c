
/*
 *  hwserver: a simple implentationa of Gwyfile compatible server for RP AFM operation
 *  Copyright (C) 2020 Petr Klapetek
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
#include <sys/file.h>

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

#include <sys/times.h>


#include "gwyfile.h"
#include "clientserver.h"

#include "cmirp.h"

#include "data.h"
#include "rptable.h"
#include "stabilizer.h"
#include "communication.h"

#define OUT_OF_RANGE 12345
#define HWDEBUG 0

#define CLAMP(x, low, high) ({\
  __typeof__(x) __x = (x); \
  __typeof__(low) __low = (low);\
  __typeof__(high) __high = (high);\
  __x > __high ? __high : (__x < __low ? __low : __x);\
  })


void stop_scan(HWData *hw, bool gotostart);
void run_adaptive_scan(HWData *hw);
void run_line_scan(HWData *hw);
void run_script_scan(HWData *hw);
void run_ramp(HWData *hw);

void run_stream(HWData *hw);
void stop_stream(HWData *hw);

void set_mode_and_feedback(HWData *hw)
{
    int req = hw->modereq;

//printf("zpiezo before feedback off %g\n", hw->zpiezo);

    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->mode = req;
    pthread_rwlock_unlock(&hw->controlmutex);

    pthread_rwlock_rdlock(&hw->controlmutex);
    librp_SetState(&hw->rp, hw->modeset[hw->mode].error_source, hw->feedback, hw->modeset[hw->mode].out1, hw->modeset[hw->mode].out2, hw->modeset[hw->mode].outhr, hw->modeset[hw->mode].swap_in, hw->modeset[hw->mode].swap_out, hw->modeset[hw->mode].pidskip);
    pthread_rwlock_unlock(&hw->controlmutex);

    librp_SetInputRange(&hw->rp, hw->modeset[hw->mode].input1_range, hw->modeset[hw->mode].input2_range);

     librp_SetPid(&hw->rp, hw->pid_p, hw->pid_i, hw->pid_d, hw->modeset[hw->mode].bitshift);

    //printf("feedback set to %d\n", hw->feedback);

//printf("zpiezo after feedback off %g\n", hw->zpiezo);

}

void set_fm_feedback(HWData *hw)
{
    //if (hw->debug) 
    //printf("setting fm feedback to %d, setpoint to %g  (SP: filt %d  lp %d  hr %d  psh %d filph %d)\n",
    //                hw->pllfeedback, hw->pidpll_setpoint, hw->filter1, hw->loopback1, hw->modeset[hw->mode].lockin1_hr, hw->phaseshift1, hw->modeset[hw->mode].lockin1_filter_phase);
    //printf("setting fm feedback (FM: fb1 %d fb2 %d sp %g pid2 %g %g %g  pid3 %g %g %g plim %d flim %d)\n",
    //                hw->pllfeedback, hw->ampfeedback, hw->pidpll_setpoint, hw->pidpll_p, hw->pidpll_i, hw->pidpll_d, hw->pidamplitude_p, hw->pidamplitude_i, hw->pidamplitude_d, hw->pll_phase_limit_factor, hw->pll_frequency_limit_factor);

    librp_SetSignalProcessing(&hw->rp, LIBRP_CH_1, hw->filter1, hw->loopback1,
                              hw->modeset[hw->mode].lockin1_hr, hw->phaseshift1, hw->debugsource1,
                              hw->modeset[hw->mode].lockin1_filter_amplitude, hw->modeset[hw->mode].lockin1_filter_phase, 
                              hw->modeset[hw->mode].lockin1_nwaves, hw->modeset[hw->mode].lockin1_lfr, (&hw->rp)->intosc1);

    librp_SetSignalProcessing(&hw->rp, LIBRP_CH_2, hw->filter2, hw->loopback2,
                              hw->modeset[hw->mode].lockin2_hr, hw->phaseshift2, hw->debugsource2,
                              hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                              hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, (&hw->rp)->intosc2);


    librp_SetPLL(&hw->rp, hw->pllfeedback, hw->ampfeedback, hw->pidpll_setpoint, hw->pidamplitude_setpoint,
                          hw->pidpll_p, hw->pidpll_i, hw->pidpll_d,
                          hw->pidamplitude_p, hw->pidamplitude_i, hw->pidamplitude_d,
                          hw->pll_phase_limit_factor, hw->pll_frequency_limit_factor,
                          hw->modeset[hw->mode].pllskip, hw->modeset[hw->mode].pll_input);
}

void set_zpiezo(HWData *hw, double value, bool force)
{
    int status;

    //printf("force %d sad %d sl %d\n", force, hw->scanning_adaptive, hw->scanning_line);
    //printf("set zpiezo: value %g hrdac %g\n", value, hw->hrdac3);
    if (force || !(hw->scanning_adaptive || hw->scanning_line) ) {
        hw->setzpiezovalue = value;
        if ((&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU) {
      //     printf("set to %g   zcal %g  zshift %g  result %g\n", hw->setzpiezovalue, hw->zcal, hw->zshift, hw->setzpiezovalue*hw->zcal + hw->zshift);
           status = librp_SetHRDAC3(&hw->rp, hw->setzpiezovalue*hw->zcal + hw->zshift);
        }
        else status = librp_SetDAC(&hw->rp, LIBRP_CH_2, hw->setzpiezovalue*hw->zcal + hw->zshift); 

        //printf("value %g was set\n", hw->setzpiezovalue);
          // printf("set DAC 2 to %g, which means %g\n", hw->setzpiezovalue, hw->setzpiezovalue*hw->zcal + hw->zshift);
        if (status != LIBRP_OK) fprintf(stderr, "RP set dac failed\n");
    }
}

void set_filter(HWData *hw, int channel)
{
    int status;

    if (channel==LIBRP_CH_1) {
       status = librp_SetSignalProcessing(&hw->rp, channel, hw->filter1, hw->loopback1, hw->modeset[hw->mode].lockin1_hr, hw->phaseshift1, hw->debugsource1, 
                                          hw->modeset[hw->mode].lockin1_filter_amplitude, hw->modeset[hw->mode].lockin1_filter_phase, 
                                          hw->modeset[hw->mode].lockin1_nwaves, hw->modeset[hw->mode].lockin1_lfr, (&hw->rp)->intosc1);
       if (status != LIBRP_OK) fprintf(stderr, "RP set filter failed\n");
       else printf("filter 1 set to %d  loopback %d\n", hw->filter1, hw->loopback1);
    } else {
       status = librp_SetSignalProcessing(&hw->rp, channel, hw->filter2, hw->loopback2, hw->modeset[hw->mode].lockin2_hr, hw->phaseshift2, hw->debugsource2, 
                                          hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                                          hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, (&hw->rp)->intosc2);
       if (status != LIBRP_OK) fprintf(stderr, "RP set filter failed\n");
       else printf("filter 2 set to %d  loopback %d\n", hw->filter2, hw->loopback2);
    }
}


void set_pid(HWData *hw)
{
    int status;

    if (hw->feedback && (&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU)      //swich off feedback for setting the PID to prevent strange behavior
    {
        set_zpiezo(hw, hw->zpiezo, TRUE);
        usleep(1000);
        librp_SetFeedback(&hw->rp, 0);
        usleep(1000);
    }

    status = librp_SetPid(&hw->rp, hw->pid_p, hw->pid_i, hw->pid_d, hw->modeset[hw->mode].bitshift); //FIXME, was 0, now w multiply the signal by 8
    if (status != LIBRP_OK) fprintf(stderr, "RP set PID failed\n");

    if (hw->feedback && (&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU)
    {
        usleep(1000);
        librp_SetFeedback(&hw->rp, hw->feedback);
    }
}


void set_setpoint(HWData *hw)
{
    int status; 

    if (hw->feedback && (&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU)      //swich off feedback for setting the PID to prevent strange behavior
    {
printf("setting piezo to %g\n", hw->zpiezo);
        set_zpiezo(hw, hw->zpiezo, TRUE);
        usleep(1000);
        librp_SetFeedback(&hw->rp, 0);
        usleep(100);
    }

    status = librp_SetSetpoint(&hw->rp, hw->pid_setpoint, 0);
    if (status != LIBRP_OK) fprintf(stderr, "RP set setpoint failed\n");

    if (hw->feedback && (&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU) 
    {
        usleep(100);
        librp_SetFeedback(&hw->rp, hw->feedback);
    }
}

void set_phaseshift(HWData *hw)
{
    int status;

    if (hw->debug) printf("setting phaseshift to %d %d\n", hw->phaseshift1, hw->phaseshift2);

    status = librp_SetSignalProcessing(&hw->rp, LIBRP_CH_1, hw->filter1, hw->loopback1, hw->modeset[hw->mode].lockin1_hr, hw->phaseshift1, hw->debugsource1, 
                                       hw->modeset[hw->mode].lockin1_filter_amplitude, hw->modeset[hw->mode].lockin1_filter_phase, 
                                       hw->modeset[hw->mode].lockin1_nwaves, hw->modeset[hw->mode].lockin1_lfr, (&hw->rp)->intosc1);
    if (status != LIBRP_OK) fprintf(stderr, "RP set phaseshift failed\n");

    status = librp_SetSignalProcessing(&hw->rp, LIBRP_CH_2, hw->filter2, hw->loopback2, hw->modeset[hw->mode].lockin2_hr, hw->phaseshift2, hw->debugsource2, 
                                       hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                                       hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, (&hw->rp)->intosc2);
    if (status != LIBRP_OK) fprintf(stderr, "RP set phaseshift failed\n");
}

void set_freq(HWData *hw, int channel)
{
    int status;

    if (channel==LIBRP_CH_1)
    {
       status = librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, hw->f1_amplitude, hw->f1_offset);
       if (status != LIBRP_OK) fprintf(stderr, "RP set gen failed\n");
    } else {
        status = librp_SetGen(&hw->rp, LIBRP_CH_2, hw->f2_frequency, hw->f2_amplitude, hw->f2_offset);
        if (status != LIBRP_OK) fprintf(stderr, "RP set gen failed\n");
    }
}

void set_freq3(HWData *hw, double frequency)
{
    int status;

    status = librp_SetAuxGen(&hw->rp, frequency);

    if (status != LIBRP_OK) fprintf(stderr, "RP set gen failed\n");
}

void set_oversampling(HWData *hw)
{
    int status;
    status = librp_SetOversampling(&hw->rp, hw->oversampling);

    if (status != LIBRP_OK) fprintf(stderr, "RP set oversampling failed\n");
}

void run_motor(HWData *hw, int motor_number, double motor_distance)
{
    if (hw->debug) printf("Running motor %d to distance %g\n", motor_number, motor_distance);

    if (motor_distance>0) {
        if (hw->debug) printf("Running ssh nenovision-service@172.16.13.35 '/home/nenovision-service/movez.py c 50'\n");
        system("ssh nenovision-service@172.16.13.35 '/home/nenovision-service/movez.py c 50' &");
    }
    else {
        if (hw->debug) printf("Running ssh nenovision-service@172.16.13.35 '/home/nenovision-service/movez.py c -50'\n");
        system("ssh nenovision-service@172.16.13.35 '/home/nenovision-service/movez.py c -50' &");
    }

}

void stop_scanning_script_thread(HWData *hw)
{
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->stop_scanning_script = TRUE;
    pthread_rwlock_unlock(&hw->controlmutex);

    if (hw->script_thread == 1) {
        pthread_join(hw->script_thread, NULL);
        hw->scan_thread_was_set = 0;
    }
}

void check_double_value(GwyfileObject *gwyf, char *key, GwyfileObject *response, double *value, pthread_rwlock_t *mutex, bool *dset)
{
    GwyfileItem *valueitem;

    valueitem = gwyfile_object_get(gwyf, key);
    if (valueitem) {
         if (mutex) pthread_rwlock_wrlock(mutex);
         *value = gwyfile_item_get_double(valueitem);
         if (mutex) pthread_rwlock_unlock(mutex);
         if (dset) *dset = 1; 
         //printf("%s set to %g\n", key, *value);
         if (response) gwyfile_object_add(response, gwyfile_item_new_double(key, *value));
    }
}
void check_int_value(GwyfileObject *gwyf, char *key, GwyfileObject *response, int *value, pthread_rwlock_t *mutex, bool *dset)
{
    GwyfileItem *valueitem;

    valueitem = gwyfile_object_get(gwyf, key);
    if (valueitem) {
         if (mutex) pthread_rwlock_wrlock(mutex);
         *value = gwyfile_item_get_int32(valueitem);
         if (mutex) pthread_rwlock_unlock(mutex);
         if (dset) *dset = 1; 
         //printf("%s set to %d\n", key, *value);
         if (response) gwyfile_object_add(response, gwyfile_item_new_int32(key, *value));
    }
}
void check_routing_value(GwyfileObject *gwyf, char *key, int *value, pthread_rwlock_t *mutex)
{
    GwyfileItem *valueitem;
    const char *string;

    valueitem = gwyfile_object_get(gwyf, key);
    if (valueitem) {
         if (mutex) pthread_rwlock_wrlock(mutex);
         string = gwyfile_item_get_string(valueitem);
         if (strcmp(string, "nothing")==0) *value = LIBRP_RULE_NOTHING;
         else if (strcmp(string, "x")==0) *value = LIBRP_RULE_X;
         else if (strcmp(string, "y")==0) *value = LIBRP_RULE_Y;
         else if (strcmp(string, "z")==0) *value = LIBRP_RULE_Z;
         else if (strcmp(string, "in1")==0) *value = LIBRP_RULE_IN1;
         else if (strcmp(string, "in2")==0) *value = LIBRP_RULE_IN2;
         else if (strcmp(string, "in3")==0) *value = LIBRP_RULE_IN3;
         else if (strcmp(string, "in4")==0) *value = LIBRP_RULE_IN4;
         else if (strcmp(string, "in5")==0) *value = LIBRP_RULE_IN5;
         else if (strcmp(string, "in6")==0) *value = LIBRP_RULE_IN6;
         else if (strcmp(string, "in7")==0) *value = LIBRP_RULE_IN7;
         else if (strcmp(string, "in8")==0) *value = LIBRP_RULE_IN8;
         else if (strcmp(string, "in9")==0) *value = LIBRP_RULE_IN9;
         else if (strcmp(string, "in10")==0) *value = LIBRP_RULE_IN10;
         else if (strcmp(string, "in11")==0) *value = LIBRP_RULE_IN11;
         else if (strcmp(string, "in12")==0) *value = LIBRP_RULE_IN12;
         else if (strcmp(string, "in13")==0) *value = LIBRP_RULE_IN13;
         else if (strcmp(string, "in14")==0) *value = LIBRP_RULE_IN14;
         else if (strcmp(string, "in15")==0) *value = LIBRP_RULE_IN15;
         else if (strcmp(string, "in16")==0) *value = LIBRP_RULE_IN16;
         else if (strcmp(string, "error")==0) *value = LIBRP_RULE_ERROR;
         else if (strcmp(string, "zpiezo")==0) *value = LIBRP_RULE_ZPIEZO;
         else if (strcmp(string, "amplitude1")==0) *value = LIBRP_RULE_AMPLITUDE1;
         else if (strcmp(string, "phase1")==0) *value = LIBRP_RULE_PHASE1;
         else if (strcmp(string, "amplitude2")==0) *value = LIBRP_RULE_AMPLITUDE2;
         else if (strcmp(string, "phase2")==0) *value = LIBRP_RULE_PHASE2;
         else if (strcmp(string, "fmresult")==0) *value = LIBRP_RULE_FMRESULT;
         else if (strcmp(string, "amresult")==0) *value = LIBRP_RULE_AMRESULT;
         else if (strcmp(string, "adc1")==0) *value = LIBRP_RULE_ADC1;
         else if (strcmp(string, "adc2")==0) *value = LIBRP_RULE_ADC2;
         else *value = LIBRP_RULE_NOTHING;
         if (mutex) pthread_rwlock_unlock(mutex);
    }
}
static 
const char* rule_to_string(int rule)
{ 
    if (rule==LIBRP_RULE_X) return "x";
    else if (rule==LIBRP_RULE_Y) return "y";
    else if (rule==LIBRP_RULE_Z) return "y";
    else if (rule==LIBRP_RULE_IN1) return "in1";
    else if (rule==LIBRP_RULE_IN2) return "in2";
    else if (rule==LIBRP_RULE_IN3) return "in3";
    else if (rule==LIBRP_RULE_IN4) return "in4";
    else if (rule==LIBRP_RULE_IN5) return "in5";
    else if (rule==LIBRP_RULE_IN6) return "in6";
    else if (rule==LIBRP_RULE_IN7) return "in7";
    else if (rule==LIBRP_RULE_IN8) return "in8";
    else if (rule==LIBRP_RULE_IN9) return "in9";
    else if (rule==LIBRP_RULE_IN10) return "in10";
    else if (rule==LIBRP_RULE_IN11) return "in11";
    else if (rule==LIBRP_RULE_IN12) return "in12";
    else if (rule==LIBRP_RULE_IN13) return "in13";
    else if (rule==LIBRP_RULE_IN14) return "in14";
    else if (rule==LIBRP_RULE_IN15) return "in15";
    else if (rule==LIBRP_RULE_IN16) return "in16";
    else if (rule==LIBRP_RULE_ERROR) return "error";
    else if (rule==LIBRP_RULE_ZPIEZO) return "zpiezo";
    else if (rule==LIBRP_RULE_AMPLITUDE1) return "amplitude1";
    else if (rule==LIBRP_RULE_PHASE1) return "phase1";
    else if (rule==LIBRP_RULE_AMPLITUDE2) return "amplitude2";
    else if (rule==LIBRP_RULE_PHASE2) return "phase2";
    else if (rule==LIBRP_RULE_FMRESULT) return "fmresult";
    else if (rule==LIBRP_RULE_AMRESULT) return "amresult";
    else if (rule==LIBRP_RULE_ADC1) return "adc1";
    else if (rule==LIBRP_RULE_ADC2) return "adc2";

    else return "nothing";
}  
void check_ramp_quantity(GwyfileObject *gwyf, char *key, int *value, pthread_rwlock_t *mutex)
{
    GwyfileItem *valueitem;
    const char *string;

    valueitem = gwyfile_object_get(gwyf, key);
    if (valueitem) {
         if (mutex) pthread_rwlock_wrlock(mutex);
         string = gwyfile_item_get_string(valueitem);
//printf("string is %s\n", string);
         if (strcmp(string, "z")==0) *value = LIBRP_RAMP_Z;
         else if (strcmp(string, "out1")==0) *value = LIBRP_RAMP_OUT1;
         else if (strcmp(string, "out2")==0) *value = LIBRP_RAMP_OUT2;
         else if (strcmp(string, "out3")==0) *value = LIBRP_RAMP_OUT3;
         else if (strcmp(string, "out4")==0) *value = LIBRP_RAMP_OUT4;
         else if (strcmp(string, "out5")==0) *value = LIBRP_RAMP_OUT5;
         else if (strcmp(string, "out6")==0) *value = LIBRP_RAMP_OUT6;
         else if (strcmp(string, "out7")==0) *value = LIBRP_RAMP_OUT7;
         else if (strcmp(string, "out8")==0) *value = LIBRP_RAMP_OUT8;
         else if (strcmp(string, "out9")==0) *value = LIBRP_RAMP_OUT9;
         else if (strcmp(string, "out10")==0) *value = LIBRP_RAMP_OUT10;
         else if (strcmp(string, "out11")==0) *value = LIBRP_RAMP_OUT11;
         else if (strcmp(string, "out12")==0) *value = LIBRP_RAMP_OUT12;
         else if (strcmp(string, "out13")==0) *value = LIBRP_RAMP_OUT13;
         else if (strcmp(string, "out14")==0) *value = LIBRP_RAMP_OUT14;
         else if (strcmp(string, "out15")==0) *value = LIBRP_RAMP_OUT15;
         else if (strcmp(string, "out16")==0) *value = LIBRP_RAMP_OUT16;
         else if (strcmp(string, "time")==0) *value = LIBRP_RAMP_TIME;
         if (mutex) pthread_rwlock_unlock(mutex);
    }
}
static 
const char* ramp_quantity_to_string(int rule)
{ 
    if (rule==LIBRP_RAMP_Z) return "z";
    else if (rule==LIBRP_RAMP_OUT1) return "out1";
    else if (rule==LIBRP_RAMP_OUT2) return "out2";
    else if (rule==LIBRP_RAMP_OUT3) return "out3";
    else if (rule==LIBRP_RAMP_OUT4) return "out4";
    else if (rule==LIBRP_RAMP_OUT5) return "out5";
    else if (rule==LIBRP_RAMP_OUT6) return "out6";
    else if (rule==LIBRP_RAMP_OUT7) return "out7";
    else if (rule==LIBRP_RAMP_OUT8) return "out8";
    else if (rule==LIBRP_RAMP_OUT9) return "out9";
    else if (rule==LIBRP_RAMP_OUT10) return "out10";
    else if (rule==LIBRP_RAMP_OUT11) return "out11";
    else if (rule==LIBRP_RAMP_OUT12) return "out12";
    else if (rule==LIBRP_RAMP_OUT13) return "out13";
    else if (rule==LIBRP_RAMP_OUT14) return "out14";
    else if (rule==LIBRP_RAMP_OUT15) return "out15";
    else if (rule==LIBRP_RAMP_OUT16) return "out16";
    else if (rule==LIBRP_RAMP_TIME) return "time";
    else return "nothing";
}  





void check_boolean_value(GwyfileObject *gwyf, char *key, GwyfileObject *response, bool *value, pthread_rwlock_t *mutex, bool *dset)
{
    GwyfileItem *valueitem;

    valueitem = gwyfile_object_get(gwyf, key);
    if (valueitem) {
         if (mutex) pthread_rwlock_wrlock(mutex);
         *value = gwyfile_item_get_bool(valueitem);
         if (mutex) pthread_rwlock_unlock(mutex);
         if (dset) *dset = 1; 
         //printf("%s set to %d\n", key, *value);
         if (response) gwyfile_object_add(response, gwyfile_item_new_bool(key, *value));
    }
}

GwyfileObject* check_storage(GwyfileObject *gwyf, pthread_rwlock_t *mutex, const char *todo, bool *a1, bool *p1, bool *a2, bool *p2, bool *in, bool *set, bool *fmd, bool *kpfm, bool *dart, bool *l1x, bool *l1y, bool *l2x, bool *l2y, bool *out9, bool *slp)
{
    GwyfileObject *response;

    check_boolean_value(gwyf, "a1", NULL, a1, mutex, NULL);
    check_boolean_value(gwyf, "p1", NULL, p1, mutex, NULL);
    check_boolean_value(gwyf, "a2", NULL, a2, mutex, NULL);
    check_boolean_value(gwyf, "p2", NULL, p2, mutex, NULL);
    check_boolean_value(gwyf, "in1", NULL, in, mutex, NULL);
    check_boolean_value(gwyf, "in2", NULL, in+1, mutex, NULL);
    check_boolean_value(gwyf, "in3", NULL, in+2, mutex, NULL);
    check_boolean_value(gwyf, "in4", NULL, in+3, mutex, NULL);
    check_boolean_value(gwyf, "in5", NULL, in+4, mutex, NULL);
    check_boolean_value(gwyf, "in6", NULL, in+5, mutex, NULL);
    check_boolean_value(gwyf, "in7", NULL, in+6, mutex, NULL);
    check_boolean_value(gwyf, "in8", NULL, in+7, mutex, NULL);
    check_boolean_value(gwyf, "in9", NULL, in+8, mutex, NULL);
    check_boolean_value(gwyf, "in10", NULL, in+9, mutex, NULL);
    check_boolean_value(gwyf, "in11", NULL, in+10, mutex, NULL);
    check_boolean_value(gwyf, "in12", NULL, in+11, mutex, NULL);
    check_boolean_value(gwyf, "in13", NULL, in+12, mutex, NULL);
    check_boolean_value(gwyf, "in14", NULL, in+13, mutex, NULL);
    check_boolean_value(gwyf, "in15", NULL, in+14, mutex, NULL);
    check_boolean_value(gwyf, "in16", NULL, in+15, mutex, NULL);
    check_boolean_value(gwyf, "set", NULL, set, mutex, NULL);
    check_boolean_value(gwyf, "fmdrive", NULL, fmd, mutex, NULL);
    check_boolean_value(gwyf, "kpfm", NULL, kpfm, mutex, NULL);
    check_boolean_value(gwyf, "dart", NULL, dart, mutex, NULL);
    check_boolean_value(gwyf, "l1x", NULL, l1x, mutex, NULL);
    check_boolean_value(gwyf, "l1y", NULL, l1y, mutex, NULL);
    check_boolean_value(gwyf, "l2x", NULL, l2x, mutex, NULL);
    check_boolean_value(gwyf, "l2y", NULL, l2y, mutex, NULL);
    check_boolean_value(gwyf, "out9", NULL, out9, mutex, NULL);
    check_boolean_value(gwyf, "slp", NULL, slp, mutex, NULL);

    *set = TRUE; //FIXME, test only

    pthread_rwlock_rdlock(mutex);
    response = gwyfile_object_new("GS",
                gwyfile_item_new_string_copy("todo", todo),
                gwyfile_item_new_bool("a1", *a1),
                gwyfile_item_new_bool("p1", *p1),
                gwyfile_item_new_bool("a2", *a2),
                gwyfile_item_new_bool("p2", *p2),
                gwyfile_item_new_bool("in1", *in),
                gwyfile_item_new_bool("in2", *(in+1)),
                gwyfile_item_new_bool("in3", *(in+2)),
                gwyfile_item_new_bool("in4", *(in+3)),
                gwyfile_item_new_bool("in5", *(in+4)),
                gwyfile_item_new_bool("in6", *(in+5)),
                gwyfile_item_new_bool("in7", *(in+6)),
                gwyfile_item_new_bool("in8", *(in+7)),
                gwyfile_item_new_bool("in9", *(in+8)),
                gwyfile_item_new_bool("in10", *(in+9)),
                gwyfile_item_new_bool("in11", *(in+10)),
                gwyfile_item_new_bool("in12", *(in+11)),
                gwyfile_item_new_bool("in13", *(in+12)),
                gwyfile_item_new_bool("in14", *(in+13)),
                gwyfile_item_new_bool("in15", *(in+14)),
                gwyfile_item_new_bool("in16", *(in+15)),
                gwyfile_item_new_bool("set", *set),
                gwyfile_item_new_bool("fmdrive", *fmd),
                gwyfile_item_new_bool("kpfm", *kpfm),
                gwyfile_item_new_bool("dart", *dart),
                gwyfile_item_new_bool("l1x", *l1x),
                gwyfile_item_new_bool("l1y", *l1y),
                gwyfile_item_new_bool("l2x", *l2x),
                gwyfile_item_new_bool("l2y", *l2y),
                gwyfile_item_new_bool("out9", *out9),
                gwyfile_item_new_bool("slp", *slp),
                NULL);

    printf("storage: in1 %d  in2 %d\n", in[0], in[1]);
    pthread_rwlock_unlock(mutex);
    return response;
}


/* interpret all the client requests. When it would be critical, not modify values directly, but
request their modification to be done in cartthread when the situation is suitable for that*/
GwyfileObject *interpret_request(HWData *hw, GwyfileObject *gwyf)
{
    int i, from, to, length, nzvals;
    bool ismove, isfreq1, isfreq2, isaout, isdout, isxsl, isysl;
    bool ispid, ispidpll, ispidamplitude, issetpoint, issetpointpll, isfeedback, ispllfeedback, ispll, isphaseshift, iskpfm, iskpfmfreq, isdart, isfreq3, isoversampling, ismode, isfilter, iszmove, isrange;
    GwyfileObject *response;
    GwyfileItem *item, *valueitem;
    double vin[16], verrorsignal, vzpiezo, vtimestamp, vx, vy, va1, va2, vp1, vp2, vadc1, vadc2, vamresult, vkpfm, vdart, vl1x, vl1y, vl2x, vl2y;
    double vxreq, vyreq, vzreq, setzpiezovalue;
    bool vfeedback, vpllfeedback, vampfeedback;
    const char *todo;
    bool ca1, cp1, ca2, cp2, cin[16], cfm, ckpfm, cdart, cl1x, cl1y, cl2x, cl2y, cout9, cslp;
    const double *vals, *xyvals;
    const char *param_string;
    double param_value, motor_distance, xsl, ysl;
    int param_found;
    int load_from, load_to, motor_number, ndata;
    char description[50];
    int scan_id;

    if (hw->inrequest) printf("Uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu\n");
    hw->inrequest = 1;

    //check what is inside
    if (HWDEBUG) printf("object has %d items\n", gwyfile_object_nitems(gwyf));
    for (i=0; i<gwyfile_object_nitems(gwyf); i++) {
        if (HWDEBUG) printf("item %d key: %s\n", i, gwyfile_item_name(gwyfile_object_get_nth(gwyf, i)));
            
    }
    
    //get todo
    item = gwyfile_object_get(gwyf, "todo");

    if (!item) {
        fprintf(stderr, "Wrong command, it has no todo.\n");
        response = gwyfile_object_new("GS",
                   gwyfile_item_new_string_copy("todo", "failed, no todo"),
                   NULL);
        return response;
    }
 
    todo = gwyfile_item_get_string(item); 
    if (HWDEBUG) printf("todo is: %s\n", todo);

    // printf("recieved request: %s\n", gwyfile_item_get_string(gwyfile_object_get(gwyf, "todo")));

    if (strcmp(todo, "state") == 0) {
 
            valueitem = gwyfile_object_get(gwyf, "mode");
            if (valueitem) {
                pthread_rwlock_wrlock(&hw->controlmutex);
                hw->modereq = -1;
                for (i=0; i<10; i++) 
                {
                   if (strcmp(gwyfile_item_get_string(valueitem), hw->modeset[i].name)==0) hw->modereq = i;
                }
                if (hw->modereq == -1)
                {
                   printf("wrong mode command %s\n", gwyfile_item_get_string(valueitem));
                   hw->modereq = hw->mode;
                }
                printf("Mode set to %d (%s)\n", hw->modereq, gwyfile_item_get_string(valueitem));
                pthread_rwlock_unlock(&hw->controlmutex);
                set_mode_and_feedback(hw);
                set_fm_feedback(hw);
                printf("mode set done\n");
            }
            ismode = isfilter = 0;
            check_int_value(gwyf, "lockin1_hr", NULL, &hw->modeset[hw->mode].lockin1_hr, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "lockin2_hr", NULL, &hw->modeset[hw->mode].lockin2_hr, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "input1_range", NULL, &hw->modeset[hw->mode].input1_range, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "input2_range", NULL, &hw->modeset[hw->mode].input2_range, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "lockin1_filter_amplitude", NULL, &hw->modeset[hw->mode].lockin1_filter_amplitude, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "lockin1_filter_phase", NULL, &hw->modeset[hw->mode].lockin1_filter_phase, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "lockin2_filter_amplitude", NULL, &hw->modeset[hw->mode].lockin2_filter_amplitude, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "lockin2_filter_phase", NULL, &hw->modeset[hw->mode].lockin2_filter_phase, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "lockin1_nwaves", NULL, &hw->modeset[hw->mode].lockin1_nwaves, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "lockin2_nwaves", NULL, &hw->modeset[hw->mode].lockin2_nwaves, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "bitshift", NULL, &hw->modeset[hw->mode].bitshift, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "pidskip", NULL, &hw->modeset[hw->mode].pidskip, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "pllskip", NULL, &hw->modeset[hw->mode].pllskip, &hw->controlmutex, &isfilter);
            check_int_value(gwyf, "mux1", NULL, &hw->modeset[hw->mode].mux1, &hw->controlmutex, &ismode);
            check_int_value(gwyf, "mux2", NULL, &hw->modeset[hw->mode].mux2, &hw->controlmutex, &ismode);
            check_boolean_value(gwyf, "swap_in", NULL, &hw->modeset[hw->mode].swap_in, &hw->controlmutex, &ismode);

            pthread_rwlock_rdlock(&hw->controlmutex);
            double x_range_prev = hw->xrange;
            double y_range_prev = hw->yrange;
            double z_range_prev = hw->zrange;
            pthread_rwlock_unlock(&hw->controlmutex);

            check_double_value(gwyf, "x_range", NULL, &hw->xrange, &hw->controlmutex, NULL);
            check_double_value(gwyf, "y_range", NULL, &hw->yrange, &hw->controlmutex, NULL);
            check_double_value(gwyf, "z_range", NULL, &hw->zrange, &hw->controlmutex, NULL);

            pthread_rwlock_wrlock(&hw->controlmutex);
            if (x_range_prev != hw->xrange || y_range_prev != hw->yrange || z_range_prev != hw->zrange) {
                ini_cleanup(hw);
            }
            pthread_rwlock_unlock(&hw->controlmutex);

            if (ismode) set_mode_and_feedback(hw); //set it again to apply all the other changes
            if (isfilter) set_fm_feedback(hw);

            response = gwyfile_object_new("GS",
                       gwyfile_item_new_string_copy("mode", hw->modeset[hw->modereq].name),
                       NULL);

            pthread_rwlock_rdlock(&hw->controlmutex);
            gwyfile_object_add(response, gwyfile_item_new_double("x_cal_factor", hw->xcal));
            gwyfile_object_add(response, gwyfile_item_new_double("y_cal_factor", hw->ycal));
            gwyfile_object_add(response, gwyfile_item_new_double("z_cal_factor", hw->zcal));
            gwyfile_object_add(response, gwyfile_item_new_double("x_shift_factor", hw->xshift));
            gwyfile_object_add(response, gwyfile_item_new_double("y_shift_factor", hw->yshift));
            gwyfile_object_add(response, gwyfile_item_new_double("z_shift_factor", hw->zshift));
            gwyfile_object_add(response, gwyfile_item_new_double("x_range", hw->xrange));
            gwyfile_object_add(response, gwyfile_item_new_double("y_range", hw->yrange));
            gwyfile_object_add(response, gwyfile_item_new_double("z_range", hw->zrange));
            gwyfile_object_add(response, gwyfile_item_new_int32("hrdac_regime", hw->hrdac_regime));
            gwyfile_object_add(response, gwyfile_item_new_int32("hrdac1_range", hw->hrdac1_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("hrdac2_range", hw->hrdac2_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("hrdac3_range", hw->hrdac3_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("scan_mode", hw->xymode));
            gwyfile_object_add(response, gwyfile_item_new_int32("rp1_input_hv", hw->rp1_input_hv));
            gwyfile_object_add(response, gwyfile_item_new_int32("rp2_input_hv", hw->rp2_input_hv));
            gwyfile_object_add(response, gwyfile_item_new_int32("dds1_range", hw->dds1_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("dds2_range", hw->dds2_range));

            gwyfile_object_add(response, gwyfile_item_new_int32("mux1", hw->modeset[hw->mode].mux1));
            gwyfile_object_add(response, gwyfile_item_new_int32("mux2", hw->modeset[hw->mode].mux2));
            gwyfile_object_add(response, gwyfile_item_new_int32("error_source", hw->modeset[hw->mode].error_source));
            gwyfile_object_add(response, gwyfile_item_new_bool("swap_in", hw->modeset[hw->mode].swap_in));
            gwyfile_object_add(response, gwyfile_item_new_bool("swap_out", hw->modeset[hw->mode].swap_out));
            gwyfile_object_add(response, gwyfile_item_new_int32("bitshift", hw->modeset[hw->mode].bitshift));
            gwyfile_object_add(response, gwyfile_item_new_bool("pll", hw->modeset[hw->mode].pll));
            gwyfile_object_add(response, gwyfile_item_new_int32("pll_input", hw->modeset[hw->mode].pll_input));
            gwyfile_object_add(response, gwyfile_item_new_int32("input1_range", hw->modeset[hw->mode].input1_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("input2_range", hw->modeset[hw->mode].input2_range));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin1_hr", hw->modeset[hw->mode].lockin1_hr));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin2_hr", hw->modeset[hw->mode].lockin2_hr));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin1_filter_amplitude", hw->modeset[hw->mode].lockin1_filter_amplitude));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin1_filter_phase", hw->modeset[hw->mode].lockin1_filter_phase));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin2_filter_amplitude", hw->modeset[hw->mode].lockin2_filter_amplitude));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin2_filter_phase", hw->modeset[hw->mode].lockin2_filter_phase));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin1_nwaves", hw->modeset[hw->mode].lockin1_nwaves));
            gwyfile_object_add(response, gwyfile_item_new_int32("lockin2_nwaves", hw->modeset[hw->mode].lockin2_nwaves));
            gwyfile_object_add(response, gwyfile_item_new_int32("pidskip", hw->modeset[hw->mode].pidskip));
            gwyfile_object_add(response, gwyfile_item_new_int32("pllskip", hw->modeset[hw->mode].pllskip));
            gwyfile_object_add(response, gwyfile_item_new_int32("out1", hw->modeset[hw->mode].out1));
            gwyfile_object_add(response, gwyfile_item_new_int32("out2", hw->modeset[hw->mode].out2));
            gwyfile_object_add(response, gwyfile_item_new_int32("outhr", hw->modeset[hw->mode].outhr));

            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode0", hw->modeset[0].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode1", hw->modeset[1].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode2", hw->modeset[2].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode3", hw->modeset[3].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode4", hw->modeset[4].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode5", hw->modeset[5].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode6", hw->modeset[6].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode7", hw->modeset[7].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode8", hw->modeset[8].name));
            gwyfile_object_add(response, gwyfile_item_new_string_copy("mode9", hw->modeset[9].name));

            pthread_rwlock_unlock(&hw->controlmutex);

    }
    else if (strcmp(todo, "start_log") == 0) {
            printf("start logging\n");
            pthread_rwlock_wrlock(&hw->cardmutex);
            hw->stoplogging = 0;
            hw->startlogging = 1;
            pthread_rwlock_unlock(&hw->cardmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "start_log"),
                        NULL);


    } else if (strcmp(todo, "stop_log") == 0) {
            printf("stop logging\n");
            pthread_rwlock_wrlock(&hw->cardmutex);
            hw->startlogging = 0;
            hw->stoplogging = 1;
            pthread_rwlock_unlock(&hw->cardmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop_log"),
                        NULL);


    } 
    else if (strcmp(todo, "get") == 0) {

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get"),
                        NULL);
            pthread_rwlock_rdlock(&hw->controlmutex);
            if (gwyfile_object_get(gwyf, "version")) gwyfile_object_add(response, gwyfile_item_new_string_copy("version", hw->version));
            if (gwyfile_object_get(gwyf, "hwtime")) gwyfile_object_add(response, gwyfile_item_new_double("hwtime", ((double)hw->start.tv_sec + (double)hw->start.tv_nsec / 1.0e9)));
            if (gwyfile_object_get(gwyf, "moving")) gwyfile_object_add(response, gwyfile_item_new_bool("moving", (hw->moving || hw->zmoving)));
            if (gwyfile_object_get(gwyf, "scanning_line")) gwyfile_object_add(response, gwyfile_item_new_bool("scanning_line", hw->scanning_line));
            if (gwyfile_object_get(gwyf, "scanning_adaptive")) gwyfile_object_add(response, gwyfile_item_new_bool("scanning_adaptive", hw->scanning_adaptive));
            if (gwyfile_object_get(gwyf, "scanning_script")) gwyfile_object_add(response, gwyfile_item_new_bool("scanning_script", hw->scanning_script));
            if (gwyfile_object_get(gwyf, "ramp_running")) gwyfile_object_add(response, gwyfile_item_new_bool("ramp_running", hw->ramp_running));
            if (gwyfile_object_get(gwyf, "pid_p")) gwyfile_object_add(response, gwyfile_item_new_double("pid_p", hw->pid_p));
            if (gwyfile_object_get(gwyf, "pid_i")) gwyfile_object_add(response, gwyfile_item_new_double("pid_i", hw->pid_i));
            if (gwyfile_object_get(gwyf, "pid_d")) gwyfile_object_add(response, gwyfile_item_new_double("pid_d", hw->pid_d));
            if (gwyfile_object_get(gwyf, "pid_pll_p")) gwyfile_object_add(response, gwyfile_item_new_double("pid_pll_p", hw->pidpll_p));
            if (gwyfile_object_get(gwyf, "pid_pll_i")) gwyfile_object_add(response, gwyfile_item_new_double("pid_pll_i", hw->pidpll_i));
            if (gwyfile_object_get(gwyf, "pid_pll_d")) gwyfile_object_add(response, gwyfile_item_new_double("pid_pll_d", hw->pidpll_d));
            if (gwyfile_object_get(gwyf, "pid_amplitude_p")) gwyfile_object_add(response, gwyfile_item_new_double("pid_amplitude_p", hw->pidamplitude_p));
            if (gwyfile_object_get(gwyf, "pid_amplitude_i")) gwyfile_object_add(response, gwyfile_item_new_double("pid_amplitude_i", hw->pidamplitude_i));
            if (gwyfile_object_get(gwyf, "pid_amplitude_d")) gwyfile_object_add(response, gwyfile_item_new_double("pid_amplitude_d", hw->pidamplitude_d));
            if (gwyfile_object_get(gwyf, "pid_amplitude_setpoint")) gwyfile_object_add(response, gwyfile_item_new_double("pid_amplitude_setpoint", hw->pidamplitude_setpoint));
            if (gwyfile_object_get(gwyf, "pid_kpfm_p")) gwyfile_object_add(response, gwyfile_item_new_double("pid_kpfm_p", hw->pidkpfm_p));
            if (gwyfile_object_get(gwyf, "pid_kpfm_i")) gwyfile_object_add(response, gwyfile_item_new_double("pid_kpfm_i", hw->pidkpfm_i));
            if (gwyfile_object_get(gwyf, "pid_kpfm_d")) gwyfile_object_add(response, gwyfile_item_new_double("pid_kpfm_d", hw->pidkpfm_d));
            if (gwyfile_object_get(gwyf, "pid_dart_p")) gwyfile_object_add(response, gwyfile_item_new_double("pid_dart_p", hw->piddart_p));
            if (gwyfile_object_get(gwyf, "pid_dart_i")) gwyfile_object_add(response, gwyfile_item_new_double("pid_dart_i", hw->piddart_i));
            if (gwyfile_object_get(gwyf, "pid_dart_d")) gwyfile_object_add(response, gwyfile_item_new_double("pid_dart_d", hw->piddart_d));
            if (gwyfile_object_get(gwyf, "pid_setpoint")) gwyfile_object_add(response, gwyfile_item_new_double("pid_setpoint", hw->pid_setpoint));
            if (gwyfile_object_get(gwyf, "pid_pll_setpoint")) gwyfile_object_add(response, gwyfile_item_new_double("pid_pll_setpoint", hw->pidpll_setpoint));
            if (gwyfile_object_get(gwyf, "pll_phase_limit_factor")) gwyfile_object_add(response, gwyfile_item_new_int32("pll_phase_limit_factor", hw->pll_phase_limit_factor));
            if (gwyfile_object_get(gwyf, "pll_frequency_limit_factor")) gwyfile_object_add(response, gwyfile_item_new_int32("pll_frequency_limit_factor", hw->pll_frequency_limit_factor));
            if (gwyfile_object_get(gwyf, "phaseshift1")) gwyfile_object_add(response, gwyfile_item_new_int32("phaseshift1", hw->phaseshift1));
            if (gwyfile_object_get(gwyf, "phaseshift2")) gwyfile_object_add(response, gwyfile_item_new_int32("phaseshift2", hw->phaseshift2));
            if (gwyfile_object_get(gwyf, "freq1_f")) gwyfile_object_add(response, gwyfile_item_new_double("freq1_f", hw->f1_frequency));
            if (gwyfile_object_get(gwyf, "freq1_a")) gwyfile_object_add(response, gwyfile_item_new_double("freq1_a", hw->f1_amplitude));
            if (gwyfile_object_get(gwyf, "freq1_o")) gwyfile_object_add(response, gwyfile_item_new_double("freq1_o", hw->f1_offset));
            if (gwyfile_object_get(gwyf, "freq2_f")) gwyfile_object_add(response, gwyfile_item_new_double("freq2_f", hw->f2_frequency));
            if (gwyfile_object_get(gwyf, "freq2_a")) gwyfile_object_add(response, gwyfile_item_new_double("freq2_a", hw->f2_amplitude));
            if (gwyfile_object_get(gwyf, "freq2_o")) gwyfile_object_add(response, gwyfile_item_new_double("freq2_o", hw->f2_offset));
            if (gwyfile_object_get(gwyf, "freq3_f")) gwyfile_object_add(response, gwyfile_item_new_double("freq3_f", hw->f3_frequency));
            if (gwyfile_object_get(gwyf, "filter1")) gwyfile_object_add(response, gwyfile_item_new_int32("filter1", hw->filter1));
            if (gwyfile_object_get(gwyf, "filter2")) gwyfile_object_add(response, gwyfile_item_new_int32("filter2", hw->filter2));
            if (gwyfile_object_get(gwyf, "kpfm_mode")) gwyfile_object_add(response, gwyfile_item_new_int32("kpfm_mode", hw->kpfm_mode));
            if (gwyfile_object_get(gwyf, "kpfm_feedback_source")) gwyfile_object_add(response, gwyfile_item_new_int32("kpfm_feedback_source", hw->kpfm_source));
            if (gwyfile_object_get(gwyf, "kpfm_feedback_direction")) gwyfile_object_add(response, gwyfile_item_new_int32("kpfm_feedback_direction", hw->kpfm_dir));
            if (gwyfile_object_get(gwyf, "kpfm_no_autoset")) gwyfile_object_add(response, gwyfile_item_new_bool("kpfm_no_autoset", hw->kpfm_no_action));
            if (gwyfile_object_get(gwyf, "dart_mode")) gwyfile_object_add(response, gwyfile_item_new_int32("dart_mode", hw->dart_mode));
            if (gwyfile_object_get(gwyf, "dart_frequency")) gwyfile_object_add(response, gwyfile_item_new_double("dart_frequency", hw->dart_frequency));
            if (gwyfile_object_get(gwyf, "dart_amplitude")) gwyfile_object_add(response, gwyfile_item_new_double("dart_amplitude", hw->dart_amplitude));
            if (gwyfile_object_get(gwyf, "dart_freqspan")) gwyfile_object_add(response, gwyfile_item_new_double("dart_freqspan", hw->dart_freqspan));
            if (gwyfile_object_get(gwyf, "oversampling")) gwyfile_object_add(response, gwyfile_item_new_int32("oversampling", hw->oversampling));
            if (gwyfile_object_get(gwyf, "intosc1")) gwyfile_object_add(response, gwyfile_item_new_int32("intosc1", (&hw->rp)->intosc1));
            if (gwyfile_object_get(gwyf, "intosc2")) gwyfile_object_add(response, gwyfile_item_new_int32("intosc2", (&hw->rp)->intosc2));
            if (gwyfile_object_get(gwyf, "loopback1")) gwyfile_object_add(response, gwyfile_item_new_int32("loopback1", (&hw->rp)->loopback1));
            if (gwyfile_object_get(gwyf, "loopback2")) gwyfile_object_add(response, gwyfile_item_new_int32("loopback2", (&hw->rp)->loopback2));
            if (gwyfile_object_get(gwyf, "simple_link_parameter")) gwyfile_object_add(response, gwyfile_item_new_double("simple_link_parameter", hw->simple_link_parameter));
            if (gwyfile_object_get(gwyf, "simple_link_channel")) gwyfile_object_add(response, gwyfile_item_new_int32("simple_link_channel", hw->simple_link_channel));
            pthread_rwlock_unlock(&hw->controlmutex);
    }
    else if (strcmp(todo, "set") == 0) {

            isfreq1 = isfreq2 = isfreq3 = 0;
            ispid = ispidpll = ispidamplitude = issetpoint = issetpointpll = 0;
            ispll = 0;
            isphaseshift = 0;
            iskpfm = iskpfmfreq = 0;
            isdart = 0;
            isoversampling = 0;

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set"),
                        NULL);

            valueitem = gwyfile_object_get(gwyf, "hwtime");
            if (valueitem) {
                printf("Reset hw time\n");
                pthread_rwlock_wrlock(&hw->controlmutex);
                clock_gettime(CLOCK_MONOTONIC, &hw->start); 
                pthread_rwlock_unlock(&hw->controlmutex);

                pthread_rwlock_rdlock(&hw->controlmutex);
                gwyfile_object_add(response, gwyfile_item_new_double("hwtime", ((double)hw->start.tv_sec + (double)hw->start.tv_nsec / 1.0e9)));
                pthread_rwlock_unlock(&hw->controlmutex);
            }

            check_double_value(gwyf, "pid_p", response, &hw->pid_p, &hw->controlmutex, &ispid);
            check_double_value(gwyf, "pid_i", response, &hw->pid_i, &hw->controlmutex, &ispid);
            check_double_value(gwyf, "pid_d", response, &hw->pid_d, &hw->controlmutex, &ispid);
            check_double_value(gwyf, "pid_setpoint", response, &hw->pid_setpoint, &hw->controlmutex, &issetpoint);

            check_double_value(gwyf, "pid_pll_p", response, &hw->pidpll_p, &hw->controlmutex, &ispidpll);
            check_double_value(gwyf, "pid_pll_i", response, &hw->pidpll_i, &hw->controlmutex, &ispidpll);
            check_double_value(gwyf, "pid_pll_d", response, &hw->pidpll_d, &hw->controlmutex, &ispidpll);
            check_double_value(gwyf, "pid_pll_setpoint", response, &hw->pidpll_setpoint, &hw->controlmutex, &issetpointpll);

            check_double_value(gwyf, "pid_amplitude_p", response, &hw->pidamplitude_p, &hw->controlmutex, &ispidamplitude);
            check_double_value(gwyf, "pid_amplitude_i", response, &hw->pidamplitude_i, &hw->controlmutex, &ispidamplitude);
            check_double_value(gwyf, "pid_amplitude_d", response, &hw->pidamplitude_d, &hw->controlmutex, &ispidamplitude);
            check_double_value(gwyf, "pid_amplitude_setpoint", response, &hw->pidamplitude_setpoint, &hw->controlmutex, &ispidamplitude);

            check_double_value(gwyf, "pid_kpfm_p", response, &hw->pidkpfm_p, &hw->controlmutex, &iskpfm);
            check_double_value(gwyf, "pid_kpfm_i", response, &hw->pidkpfm_i, &hw->controlmutex, &iskpfm);
            check_double_value(gwyf, "pid_kpfm_d", response, &hw->pidkpfm_d, &hw->controlmutex, &iskpfm);

            check_double_value(gwyf, "pid_dart_p", response, &hw->piddart_p, &hw->controlmutex, &isdart);
            check_double_value(gwyf, "pid_dart_i", response, &hw->piddart_i, &hw->controlmutex, &isdart);
            check_double_value(gwyf, "pid_dart_d", response, &hw->piddart_d, &hw->controlmutex, &isdart);
  
            check_int_value(gwyf, "pll_phase_limit_factor", response, &hw->pll_phase_limit_factor, &hw->controlmutex, &ispll);
            check_int_value(gwyf, "pll_frequency_limit_factor", response, &hw->pll_frequency_limit_factor, &hw->controlmutex, &ispll);

            check_int_value(gwyf, "phaseshift1", response, &hw->phaseshift1, &hw->controlmutex, &isphaseshift);
            check_int_value(gwyf, "phaseshift2", response, &hw->phaseshift2, &hw->controlmutex, &isphaseshift);

            check_double_value(gwyf, "freq1_f", response, &hw->f1_frequency, &hw->controlmutex, &isfreq1);
            check_double_value(gwyf, "freq1_a", response, &hw->f1_amplitude, &hw->controlmutex, &isfreq1);
            check_double_value(gwyf, "freq1_o", response, &hw->f1_offset, &hw->controlmutex, &isfreq1);

            check_double_value(gwyf, "freq2_f", response, &hw->f2_frequency, &hw->controlmutex, &isfreq2);
            check_double_value(gwyf, "freq2_a", response, &hw->f2_amplitude, &hw->controlmutex, &isfreq2);
            check_double_value(gwyf, "freq2_o", response, &hw->f2_offset, &hw->controlmutex, &isfreq2);

            check_double_value(gwyf, "freq3_f", response, &hw->f3_frequency, &hw->controlmutex, &isfreq3);

            check_int_value(gwyf, "filter1", response, &hw->filter1, &hw->controlmutex, &hw->isfilter1);
            check_int_value(gwyf, "filter2", response, &hw->filter2, &hw->controlmutex, &hw->isfilter2);

            check_int_value(gwyf, "kpfm_mode", response, &hw->kpfm_mode, &hw->controlmutex, &iskpfmfreq);
            check_int_value(gwyf, "kpfm_feedback_source", response, &hw->kpfm_source, &hw->controlmutex, &iskpfm);
            check_int_value(gwyf, "kpfm_feedback_direction", response, &hw->kpfm_dir, &hw->controlmutex, &iskpfm);
            check_boolean_value(gwyf, "kpfm_no_autoset", response, &hw->kpfm_no_action, &hw->controlmutex, &iskpfm);

            check_int_value(gwyf, "dart_mode", response, &hw->dart_mode, &hw->controlmutex, &isdart);
            check_double_value(gwyf, "dart_frequency", response, &hw->dart_frequency, &hw->controlmutex, &isdart);
            check_double_value(gwyf, "dart_amplitude", response, &hw->dart_amplitude, &hw->controlmutex, &isdart);
            check_double_value(gwyf, "dart_freqspan", response, &hw->dart_freqspan, &hw->controlmutex, &isdart);
 
            check_int_value(gwyf, "oversampling", response, &hw->oversampling, &hw->controlmutex, &isoversampling);

            check_int_value(gwyf, "intosc1", response, &((&hw->rp)->intosc1), &hw->controlmutex, &hw->isfilter1);
            check_int_value(gwyf, "intosc2", response, &((&hw->rp)->intosc2), &hw->controlmutex, &hw->isfilter2);
            check_int_value(gwyf, "loopback1", response, &hw->loopback1, &hw->controlmutex, &hw->isfilter1);
            check_int_value(gwyf, "loopback2", response, &hw->loopback2, &hw->controlmutex, &hw->isfilter2);

            check_double_value(gwyf, "simple_link_parameter", response, &hw->simple_link_parameter, NULL, NULL);
            check_int_value(gwyf, "simple_link_channel", response, &hw->simple_link_channel, NULL, NULL);


            if (hw->isfilter1) {
                set_filter(hw, LIBRP_CH_1);
                hw->isfilter1 = 0;
            }
            if (hw->isfilter2) {
                set_filter(hw, LIBRP_CH_2);
                hw->isfilter2 = 0;
            }

            if (ispid) set_pid(hw); 
            if (ispidpll || ispidamplitude || ispll) 
            {
                if (hw->modeset[hw->mode].pll) set_fm_feedback(hw);
            }    

            if (issetpoint) set_setpoint(hw); 
          
            if (isfreq1) set_freq(hw, LIBRP_CH_1); 
            if (isfreq2) {
               set_freq(hw, LIBRP_CH_2);
               if (iskpfm) {
                  iskpfm = 0;
                  iskpfmfreq = 1;
               }
             }
            if (isfreq3) {
               set_freq3(hw, hw->f3_frequency);
               if (iskpfm) {
                  iskpfm = 0;
                  iskpfmfreq = 1;
               }
            }

            if (isphaseshift) set_phaseshift(hw);

            if (isoversampling) set_oversampling(hw);

            if (iskpfmfreq) {
               if (hw->kpfm_mode != LIBRP_KPFM_OFF && !hw->kpfm_no_action && !(hw->scanning_line || hw->scanning_adaptive || hw->scanning_script)) hw->kpfm_feedback = TRUE;

               set_mode_and_feedback(hw);
               librp_SetKPFM(&hw->rp, hw->kpfm_mode, hw->f2_frequency, hw->f3_frequency, hw->f2_amplitude, hw->pidkpfm_p, hw->pidkpfm_i, hw->pidkpfm_d, hw->f2_offset, hw->filter2, 
                             hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                             hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, 1);
            }

            if (iskpfm) {
               if (hw->kpfm_mode != LIBRP_KPFM_OFF && !hw->kpfm_no_action && !(hw->scanning_line || hw->scanning_adaptive || hw->scanning_script)) hw->kpfm_feedback = TRUE;

               set_mode_and_feedback(hw);
               librp_SetKPFM(&hw->rp, hw->kpfm_mode, hw->f2_frequency, hw->f3_frequency, hw->f2_amplitude, hw->pidkpfm_p, hw->pidkpfm_i, hw->pidkpfm_d, hw->f2_offset, hw->filter2, 
                             hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                             hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, 0);
            }

            if (isdart) {
               librp_SetDART(&hw->rp, hw->dart_mode, hw->dart_frequency, hw->dart_amplitude, hw->piddart_p, hw->piddart_i, hw->piddart_d, hw->dart_freqspan, 
                             hw->filter1, hw->modeset[hw->mode].lockin1_lfr);
               librp_SetFeedback(&hw->rp, hw->feedback);
            }

    }
    else if (strcmp(todo, "reset_spi") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "reset_spi"),
                        NULL);

            printf("reset SPI command received hrdac %g %g %g\n", hw->hrdac1, hw->hrdac2, hw->hrdac3);

            pthread_rwlock_wrlock(&hw->controlmutex);
            hw->reset_spi = TRUE;
            pthread_rwlock_unlock(&hw->controlmutex);

            usleep(100000);

            pthread_rwlock_wrlock(&hw->controlmutex);   //force SPICycle to load the actual values again
            for (i=0; i<16; i++) (&(hw->rp))->old_lr[i] = OUT_OF_RANGE;
            (&(hw->rp))->old_hr1 = OUT_OF_RANGE;
            (&(hw->rp))->old_hr2 = OUT_OF_RANGE;
            (&(hw->rp))->old_hr3 = OUT_OF_RANGE;
            pthread_rwlock_unlock(&hw->controlmutex);

            printf("reset probably done, %g %g %g\n", hw->hrdac1, hw->hrdac2, hw->hrdac3);
    }

    else if (strcmp(todo, "set_simulation") == 0) {
        response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_simulation"),
                        NULL);
        check_boolean_value(gwyf, "enable", response, &hw->isFake, &hw->controlmutex, NULL);
    }

    else if (strcmp(todo, "stop") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop"),
                        NULL);


            // We have to wait for script thread to end here in communication thread
            // before we proceed to stop everything else. If we didnt, scan script could
            // alter hw params (like scanning_line) after our stops have been completed.
            // That would create a wierd state in which it would want to scan something
            // but it could not. Some communication commands would not work as well.
            stop_scanning_script_thread(hw);
            stop_scan(hw, FALSE);      

            Stabilizer* stbl = hw->stabilizer;
            pthread_rwlock_wrlock(&stbl->mutex);
            stbl_stop(hw);
            pthread_rwlock_unlock(&stbl->mutex);

            pthread_rwlock_wrlock(&hw->controlmutex);
            hw->stop_ramp = 1;
            pthread_rwlock_unlock(&hw->controlmutex);

            pthread_rwlock_wrlock(&hw->movemutex);
            hw->moving = 0;
            hw->moved = 1;
            hw->movingcounter = 0;
            hw->zmoving = 0;
            hw->zmoved = 1;
            hw->zmovingcounter = 0;
            pthread_rwlock_unlock(&hw->movemutex);
    }
    else if (strcmp(todo, "set_scan") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_scan"),
                        NULL);

            isxsl = isysl = 0;
            check_double_value(gwyf, "speed", response, &hw->speed, &hw->controlmutex, NULL);
            check_double_value(gwyf, "zspeed", response, &hw->zspeed, &hw->controlmutex, NULL);
            check_double_value(gwyf, "delay", response, &hw->delay, &hw->controlmutex, NULL);
            check_double_value(gwyf, "xslope", response, &xsl, &hw->controlmutex, &isxsl);
            check_double_value(gwyf, "yslope", response, &ysl, &hw->controlmutex, &isysl);
            check_double_value(gwyf, "xsloperef", response, &hw->xsloperef, &hw->controlmutex, NULL);
            check_double_value(gwyf, "ysloperef", response, &hw->ysloperef, &hw->controlmutex, NULL);
            check_boolean_value(gwyf, "subtract_slope", response, &hw->subtract_slope, &hw->controlmutex, NULL);

            if (isxsl) hw->xslope = tan(xsl);
            if (isysl) hw->yslope = tan(ysl);

            if (isxsl || isysl) printf("slope corrections: %g %g rad, %g %g  center %g %g\n", xsl, ysl, hw->xslope, hw->yslope, hw->xsloperef, hw->ysloperef);
           
    }
    else if (strcmp(todo, "stop_scan") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop_scan"),
                        NULL);
            printf("stop scan issued\n");

            // We have to wait for script thread to end here in communication thread
            // before we proceed to stop everything else. If we didnt, scan script could
            // alter hw params (like scanning_line) after our stops have been completed.
            // That would create a wierd state in which it would want to scan something
            // but it could not. Some communication commands would not work as well.
            stop_scanning_script_thread(hw);
            stop_scan(hw, TRUE);      
    }
    else if (strcmp(todo, "pause_scan") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "pause_scan"),
                        NULL);
            check_boolean_value(gwyf, "pause", response, &hw->pause_scan, &hw->controlmutex, NULL);
    }
      else if (strcmp(todo, "set_standby_storage") == 0) {
            response = check_storage(gwyf, &hw->controlmutex, todo, &hw->c_sb_a1, &hw->c_sb_p1, &hw->c_sb_a2, &hw->c_sb_p2, hw->c_sb_in, &hw->c_sb_set, &hw->c_sb_fmd, &hw->c_sb_kpfm, &hw->c_sb_dart, &hw->c_sb_l1x, &hw->c_sb_l1y, &hw->c_sb_l2x, &hw->c_sb_l2y, &hw->c_sb_out9, &hw->c_sb_slp);
    }
    else if (strcmp(todo, "set_scan_storage") == 0) {
            response = check_storage(gwyf, &hw->controlmutex, todo, &hw->c_sc_a1, &hw->c_sc_p1, &hw->c_sc_a2, &hw->c_sc_p2, hw->c_sc_in, &hw->c_sc_set, &hw->c_sc_fmd, &hw->c_sc_kpfm, &hw->c_sc_dart, &hw->c_sc_l1x, &hw->c_sc_l1y, &hw->c_sc_l2x, &hw->c_sc_l2y, &hw->c_sc_out9, &hw->c_sc_slp);
    }
    else if (strcmp(todo, "set_ramp_storage") == 0) {
            response = check_storage(gwyf, &hw->controlmutex, todo, &hw->c_rm_a1, &hw->c_rm_p1, &hw->c_rm_a2, &hw->c_rm_p2, hw->c_rm_in, &hw->c_rm_set, &hw->c_rm_fmd, &hw->c_rm_kpfm, &hw->c_rm_dart, &hw->c_rm_l1x, &hw->c_rm_l1y, &hw->c_rm_l2x, &hw->c_rm_l2y, &hw->c_rm_out9, &hw->c_rm_slp);
    }
    else if (strcmp(todo, "set_stream_storage") == 0) {
            printf("command set stream storage\n");
            response = check_storage(gwyf, &hw->controlmutex, todo, &hw->c_st_a1, &hw->c_st_p1, &hw->c_st_a2, &hw->c_st_p2, hw->c_st_in, &hw->c_st_set, &hw->c_st_fmd, &hw->c_st_kpfm, &hw->c_st_dart, &hw->c_st_l1x, &hw->c_st_l1y, &hw->c_st_l2x, &hw->c_st_l2y, &hw->c_st_out9, &hw->c_st_slp);
            //FIXME: no x,y,z,e,ts in the check storage, now are set always to false.
    }
     else if (strcmp(todo, "clear_script_params") == 0) {

            pthread_rwlock_wrlock(&hw->controlmutex);
            hw->script_param_n = 0;
            pthread_rwlock_unlock(&hw->controlmutex);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "clear_script_params"),
                        NULL);
     }
     else if (strcmp(todo, "set_script_param") == 0) {

            valueitem = gwyfile_object_get(gwyf, "key");
            if (valueitem) {
               param_string = gwyfile_item_get_string(valueitem);
            }
            valueitem = gwyfile_object_get(gwyf, "value");
            if (valueitem) {
               param_value = gwyfile_item_get_double(valueitem);
            }
            param_found = 0;
            for (i=0; i<hw->script_param_n; i++) {
               if (strcmp(hw->script_param_key[i], param_string)==0)
               {
                   printf("altering parameter %d, key %s to %g\n", i, param_string, param_value);
                   hw->script_param_value[i] = param_value;
                   param_found = 1;
               }
            }
            if (!param_found) {
               printf("adding parameter %d key %s as %g\n", hw->script_param_n, param_string, param_value);  
               snprintf(hw->script_param_key[hw->script_param_n], sizeof(hw->script_param_key[hw->script_param_n]), "%s", param_string);
               hw->script_param_value[hw->script_param_n] = param_value;
               hw->script_param_n++;
          
            }
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_script_param"),
                        gwyfile_item_new_string_copy("key", param_string),
                        gwyfile_item_new_double("value", param_value),
                        NULL);
 
     }
     else if (strcmp(todo, "set_feedback") == 0) {

            isfeedback = ispllfeedback = 0;
            check_boolean_value(gwyf, "feedback", NULL, &hw->feedback, &hw->controlmutex, &isfeedback);
            check_boolean_value(gwyf, "feedback_pll", NULL, &hw->pllfeedback, &hw->controlmutex, &ispllfeedback);
            check_boolean_value(gwyf, "feedback_amplitude", NULL, &hw->ampfeedback, &hw->controlmutex, &ispllfeedback);
            check_boolean_value(gwyf, "feedback_kpfm", NULL, &hw->kpfm_feedback, &hw->controlmutex, NULL);
            check_double_value(gwyf, "zpiezo", NULL, &setzpiezovalue, &hw->controlmutex, &hw->setzpiezo);

            //if (hw->setzpiezo) printf("set feedback: setting zpiezo to %g\n", setzpiezovalue);

            if (isfeedback) set_mode_and_feedback(hw);
            if (ispllfeedback) set_fm_feedback(hw);
            if (hw->setzpiezo) {
               //printf("set zpiezo to %g\n", setzpiezovalue);
               set_zpiezo(hw, setzpiezovalue, TRUE);
               hw->setzpiezo = 0;
            }

            vfeedback = hw->feedback;
            vpllfeedback = hw->pllfeedback;
            vampfeedback = hw->ampfeedback;
            vzpiezo = hw->zpiezo;

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_feedback"),
                        gwyfile_item_new_bool("feedback", vfeedback),
                        gwyfile_item_new_bool("feedback_pll", vpllfeedback),
                        gwyfile_item_new_bool("feedback_amplitude", vampfeedback),
                        gwyfile_item_new_bool("feedback_kpfm", hw->kpfm_feedback),
                        gwyfile_item_new_double("zpiezo", vzpiezo),
                        NULL);
    }
    else if (strcmp(todo, "set_routing") == 0) {

            check_routing_value(gwyf, "out1", &hw->routing_rule[0], &hw->controlmutex);
            check_routing_value(gwyf, "out2", &hw->routing_rule[1], &hw->controlmutex);
            check_routing_value(gwyf, "out3", &hw->routing_rule[2], &hw->controlmutex);
            check_routing_value(gwyf, "out4", &hw->routing_rule[3], &hw->controlmutex);
            check_routing_value(gwyf, "out5", &hw->routing_rule[4], &hw->controlmutex);
            check_routing_value(gwyf, "out6", &hw->routing_rule[5], &hw->controlmutex);
            check_routing_value(gwyf, "out7", &hw->routing_rule[6], &hw->controlmutex);
            check_routing_value(gwyf, "out8", &hw->routing_rule[7], &hw->controlmutex);
            check_routing_value(gwyf, "out9", &hw->routing_rule[8], &hw->controlmutex);
            check_routing_value(gwyf, "out10", &hw->routing_rule[9], &hw->controlmutex);
            check_routing_value(gwyf, "out11", &hw->routing_rule[10], &hw->controlmutex);
            check_routing_value(gwyf, "out12", &hw->routing_rule[11], &hw->controlmutex);
            check_routing_value(gwyf, "out13", &hw->routing_rule[12], &hw->controlmutex);
            check_routing_value(gwyf, "out14", &hw->routing_rule[13], &hw->controlmutex);
            check_routing_value(gwyf, "out15", &hw->routing_rule[14], &hw->controlmutex);
            check_routing_value(gwyf, "out16", &hw->routing_rule[15], &hw->controlmutex);

            pthread_rwlock_rdlock(&hw->controlmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_routing"),
                        gwyfile_item_new_string_copy("out1", rule_to_string(hw->routing_rule[0])),
                        gwyfile_item_new_string_copy("out2", rule_to_string(hw->routing_rule[1])),
                        gwyfile_item_new_string_copy("out3", rule_to_string(hw->routing_rule[2])),
                        gwyfile_item_new_string_copy("out4", rule_to_string(hw->routing_rule[3])),
                        gwyfile_item_new_string_copy("out5", rule_to_string(hw->routing_rule[4])),
                        gwyfile_item_new_string_copy("out6", rule_to_string(hw->routing_rule[5])),
                        gwyfile_item_new_string_copy("out7", rule_to_string(hw->routing_rule[6])),
                        gwyfile_item_new_string_copy("out8", rule_to_string(hw->routing_rule[7])),
                        gwyfile_item_new_string_copy("out9", rule_to_string(hw->routing_rule[8])),
                        gwyfile_item_new_string_copy("out10", rule_to_string(hw->routing_rule[9])),
                        gwyfile_item_new_string_copy("out11", rule_to_string(hw->routing_rule[10])),
                        gwyfile_item_new_string_copy("out12", rule_to_string(hw->routing_rule[11])),
                        gwyfile_item_new_string_copy("out13", rule_to_string(hw->routing_rule[12])),
                        gwyfile_item_new_string_copy("out14", rule_to_string(hw->routing_rule[13])),
                        gwyfile_item_new_string_copy("out15", rule_to_string(hw->routing_rule[14])),
                        gwyfile_item_new_string_copy("out16", rule_to_string(hw->routing_rule[15])),
                        NULL);
            pthread_rwlock_unlock(&hw->controlmutex);
            printf("Routing set.\n");
    }
      else if (strcmp(todo, "move_to") == 0) {
            ismove = iszmove = 0;
            check_double_value(gwyf, "xreq", NULL, &hw->xreq, &hw->controlmutex, &ismove);
            check_double_value(gwyf, "yreq", NULL, &hw->yreq, &hw->controlmutex, &ismove);
            check_double_value(gwyf, "zreq", NULL, &hw->zreq, &hw->controlmutex, &iszmove);

            //if (ismove) printf("move to command: %g %g\n", hw->xreq, hw->yreq);
            //if (iszmove) printf("z move to command: %g\n", hw->zreq);

            pthread_rwlock_wrlock(&hw->movemutex);
            if (ismove>0) {
                hw->moveto = 1;
            }
            vxreq = hw->xreq;
            vyreq = hw->yreq;

            if (iszmove>0) {
                hw->zmoveto = 1;   
            }
            vzreq = hw->zreq;
            pthread_rwlock_unlock(&hw->movemutex);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "move_to"),
                        gwyfile_item_new_double("xreq", vxreq),
                        gwyfile_item_new_double("yreq", vyreq),
                        gwyfile_item_new_double("zreq", vzreq),
                        NULL);
    }
    else if (strcmp(todo, "set_out") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_out"),
                        NULL);


            // NENO
            check_double_value(gwyf, "out9_offset", response, &hw->out9_offset, &hw->controlmutex, NULL);
            /////

            isaout = isdout = 0;
            check_double_value(gwyf, "out1", response, &hw->aout[0], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out2", response, &hw->aout[1], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out3", response, &hw->aout[2], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out4", response, &hw->aout[3], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out5", response, &hw->aout[4], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out6", response, &hw->aout[5], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out7", response, &hw->aout[6], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out8", response, &hw->aout[7], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out9", response, &hw->aout[8], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out10", response, &hw->aout[9], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out11", response, &hw->aout[10], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out12", response, &hw->aout[11], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out13", response, &hw->aout[12], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out14", response, &hw->aout[13], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out15", response, &hw->aout[14], &hw->controlmutex, &isaout);
            check_double_value(gwyf, "out16", response, &hw->aout[15], &hw->controlmutex, &isaout);
    
           printf("set analog value: %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g %g\n", 
                                  hw->aout[0], hw->aout[1], hw->aout[2], hw->aout[3],
                                  hw->aout[4], hw->aout[5], hw->aout[6], hw->aout[7],
                                  hw->aout[8], hw->aout[9], hw->aout[10], hw->aout[11],
                                  hw->aout[12], hw->aout[13], hw->aout[14], hw->aout[15]);

            if (isaout) {
               pthread_rwlock_wrlock(&hw->controlmutex);
               hw->isaout = 1;
               pthread_rwlock_unlock(&hw->controlmutex);
            }

            check_boolean_value(gwyf, "dout1", response, &hw->dout[0], &hw->controlmutex, &isdout);
            check_boolean_value(gwyf, "dout2", response, &hw->dout[1], &hw->controlmutex, &isdout);
            check_boolean_value(gwyf, "dout3", response, &hw->dout[2], &hw->controlmutex, &isdout);
            check_boolean_value(gwyf, "dout4", response, &hw->dout[3], &hw->controlmutex, &isdout);
            
            if (isdout) {
               pthread_rwlock_wrlock(&hw->controlmutex);
               hw->isdout = 1;
               pthread_rwlock_unlock(&hw->controlmutex);
            }

     } 
    else if (strcmp(todo, "get_out") == 0) {
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_out"),
                        NULL);

            pthread_rwlock_rdlock(&hw->controlmutex);

            gwyfile_object_add(response, gwyfile_item_new_double("out1", hw->aout[0]));
            gwyfile_object_add(response, gwyfile_item_new_double("out2", hw->aout[1]));
            gwyfile_object_add(response, gwyfile_item_new_double("out3", hw->aout[2]));
            gwyfile_object_add(response, gwyfile_item_new_double("out4", hw->aout[3]));
            gwyfile_object_add(response, gwyfile_item_new_double("out5", hw->aout[4]));
            gwyfile_object_add(response, gwyfile_item_new_double("out6", hw->aout[5]));
            gwyfile_object_add(response, gwyfile_item_new_double("out7", hw->aout[6]));
            gwyfile_object_add(response, gwyfile_item_new_double("out8", hw->aout[7]));
            gwyfile_object_add(response, gwyfile_item_new_double("out9", hw->aout[8]));
            gwyfile_object_add(response, gwyfile_item_new_double("out10", hw->aout[9]));
            gwyfile_object_add(response, gwyfile_item_new_double("out11", hw->aout[10]));
            gwyfile_object_add(response, gwyfile_item_new_double("out12", hw->aout[11]));
            gwyfile_object_add(response, gwyfile_item_new_double("out13", hw->aout[12]));
            gwyfile_object_add(response, gwyfile_item_new_double("out14", hw->aout[13]));
            gwyfile_object_add(response, gwyfile_item_new_double("out15", hw->aout[14]));
            gwyfile_object_add(response, gwyfile_item_new_double("out16", hw->aout[15]));

            gwyfile_object_add(response, gwyfile_item_new_double("dout1", hw->dout[0]));
            gwyfile_object_add(response, gwyfile_item_new_double("dout2", hw->dout[1]));
            gwyfile_object_add(response, gwyfile_item_new_double("dout3", hw->dout[2]));
            gwyfile_object_add(response, gwyfile_item_new_double("dout4", hw->dout[3]));

            pthread_rwlock_unlock(&hw->controlmutex);
     }
    else if (strcmp(todo, "read") == 0) {
            pthread_rwlock_rdlock(&hw->cardmutex);

            if (hw->x_external) vx = (hw->in[hw->x_external_source] + hw->x_external_offset)*hw->x_external_slope;
            else vx = hw->xpos;
            if (hw->y_external) vy = (hw->in[hw->y_external_source] + hw->y_external_offset)*hw->y_external_slope;
            else vy = hw->ypos;

            vzpiezo = hw->zpiezo;
            verrorsignal = hw->errorsignal;
            vtimestamp = hw->timestamp;
            va1 = hw->amplitude1;
            va2 = hw->amplitude2;
            vp1 = hw->phase1;
            vp2 = hw->phase2;
            vadc1 = hw->adc1;
            vadc2 = hw->adc2;
            vamresult = hw->amresult;
            vkpfm = hw->f2_offset;
            vdart = hw->dart_frequency + hw->dart_frequencyshift;
            vl1x = hw->l1x;
            vl1y = hw->l1y;
            vl2x = hw->l2x;
            vl2y = hw->l2y;
            for (i=0; i<16; i++) vin[i] = hw->in[i];
            ca1 = hw->c_sb_a1;
            cp1 = hw->c_sb_p1;
            ca2 = hw->c_sb_a2;
            cp2 = hw->c_sb_p2;
            cfm = hw->c_sb_fmd;
            ckpfm = hw->c_sb_kpfm;
            cdart = hw->c_sb_dart;
            cl1x = hw->c_sb_l1x;
            cl1y = hw->c_sb_l1y;
            cl2x = hw->c_sb_l2x;
            cl2y = hw->c_sb_l2y;
            cout9 = hw->c_sb_out9;
            cslp = hw->c_sb_slp;
            for (i=0; i<16; i++) cin[i] = hw->c_sb_in[i];
            pthread_rwlock_unlock(&hw->cardmutex);

            //printf("z: %g\n", hw->zpiezo);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "read"),
                        gwyfile_item_new_double("x", vx),
                        gwyfile_item_new_double("y", vy),
                        gwyfile_item_new_double("z", vzpiezo),
                        gwyfile_item_new_double("e", verrorsignal),
                        gwyfile_item_new_double("ts", vtimestamp),
                        gwyfile_item_new_double("adc1", vadc1),
                        gwyfile_item_new_double("adc2", vadc2),
                        NULL);

            if (ca1) gwyfile_object_add(response, gwyfile_item_new_double("a1", va1));
            if (cp1) gwyfile_object_add(response, gwyfile_item_new_double("p1", vp1));
            if (ca2) gwyfile_object_add(response, gwyfile_item_new_double("a2", va2));
            if (cp2) gwyfile_object_add(response, gwyfile_item_new_double("p2", vp2));
            if (cin[0]) gwyfile_object_add(response, gwyfile_item_new_double("in1", vin[0]));
            if (cin[1]) gwyfile_object_add(response, gwyfile_item_new_double("in2", vin[1]));
            if (cin[2]) gwyfile_object_add(response, gwyfile_item_new_double("in3", vin[2]));
            if (cin[3]) gwyfile_object_add(response, gwyfile_item_new_double("in4", vin[3]));
            if (cin[4]) gwyfile_object_add(response, gwyfile_item_new_double("in5", vin[4]));
            if (cin[5]) gwyfile_object_add(response, gwyfile_item_new_double("in6", vin[5]));
            if (cin[6]) gwyfile_object_add(response, gwyfile_item_new_double("in7", vin[6]));
            if (cin[7]) gwyfile_object_add(response, gwyfile_item_new_double("in8", vin[7]));
            if (cin[8]) gwyfile_object_add(response, gwyfile_item_new_double("in9", vin[8]));
            if (cin[9]) gwyfile_object_add(response, gwyfile_item_new_double("in10", vin[9]));
            if (cin[10]) gwyfile_object_add(response, gwyfile_item_new_double("in11", vin[10]));
            if (cin[11]) gwyfile_object_add(response, gwyfile_item_new_double("in12", vin[11]));
            if (cin[12]) gwyfile_object_add(response, gwyfile_item_new_double("in13", vin[12]));
            if (cin[13]) gwyfile_object_add(response, gwyfile_item_new_double("in14", vin[13]));
            if (cin[14]) gwyfile_object_add(response, gwyfile_item_new_double("in15", vin[14]));
            if (cin[15]) gwyfile_object_add(response, gwyfile_item_new_double("in16", vin[15]));
            if (cfm) gwyfile_object_add(response, gwyfile_item_new_double("fmdrive", vamresult));
            if (ckpfm) gwyfile_object_add(response, gwyfile_item_new_double("kpfm", vkpfm));
            if (cdart) gwyfile_object_add(response, gwyfile_item_new_double("dart", vdart));
            if (cl1x) gwyfile_object_add(response, gwyfile_item_new_double("l1x", vl1x));
            if (cl1y) gwyfile_object_add(response, gwyfile_item_new_double("l1y", vl1y));
            if (cl2x) gwyfile_object_add(response, gwyfile_item_new_double("l2x", vl2x));
            if (cl2y) gwyfile_object_add(response, gwyfile_item_new_double("l2y", vl2y));
    }
    /*else if (strcmp(todo, "get_scan_ndata") == 0) {
            pthread_rwlock_rdlock(&hw->controlmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_scan_ndata"),
                        gwyfile_item_new_int32("n", hw->scan_ndata),
                        NULL);
            pthread_rwlock_unlock(&hw->controlmutex);
    }*/
    else if (strcmp(todo, "set_scan_path_data") == 0) {

            //printf("set scan path data received\n");
            stop_scan(hw, TRUE);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_scan_path_data"),
                        NULL);
            check_int_value(gwyf, "n", response, &hw->scan_path_ndata, &hw->controlmutex, NULL);

            if (hw->scan_path_ndata>0)
            {
               //stop_scan(hw);
               pthread_rwlock_wrlock(&hw->controlmutex);
               if (hw->loadingscandata == 0) {
                  //printf("allocating scan data\n");
                  if (hw->scan_path_xydata != NULL) free(hw->scan_path_xydata);
                  hw->scan_path_xydata = (double *)malloc(2*hw->scan_path_ndata*sizeof(double));

                  hw->lift_usezdata = FALSE;
                  valueitem = gwyfile_object_get(gwyf, "z");
                  if (valueitem)
                  {
                     if (!hw->lift_zdata) hw->lift_zdata = (double *)malloc(hw->scan_path_ndata*sizeof(double));
                     else hw->lift_zdata = (double *)realloc(hw->lift_zdata, hw->scan_path_ndata*sizeof(double));
                     hw->lift_usezdata = TRUE;
                  }
                  hw->loadingscandata = 1;
               }
               check_int_value(gwyf, "from", response, &load_from, NULL, NULL);
               check_int_value(gwyf, "to", response, &load_to, NULL, NULL);

               valueitem = gwyfile_object_get(gwyf, "xydata");
               if (!valueitem) fprintf(stderr, "Error! run_scan_path message does not contain xydata\n");
               else {
                  xyvals = gwyfile_item_get_double_array(valueitem);
                  //printf("filling data from %d to %d\n", load_from, load_to);
                  for (i=0; i<(load_to-load_from); i++) {
                      hw->scan_path_xydata[2*load_from + 2*i] = xyvals[2*i];
                      hw->scan_path_xydata[2*load_from + 2*i + 1] = xyvals[2*i+1];
                     // printf("path %d: %g %g\n", 2*load_from + 2*i, hw->scan_path_xydata[2*load_from + 2*i], hw->scan_path_xydata[2*load_from + 2*i + 1]);
                  }

                  valueitem = gwyfile_object_get(gwyf, "z");
                  if (valueitem)
                  {
                     vals = gwyfile_item_get_double_array(valueitem);

                     for (i=0; i<(load_to-load_from); i++) hw->lift_zdata[load_from + i] = vals[i];
                  }
               }
               pthread_rwlock_unlock(&hw->controlmutex);

              // printf("scan path part received. %d points, from %d to %d\n", (load_to-load_from), load_from, load_to);
              // printf("scan path state. %d points, values (%g %g), (%g %g), (%g %g)..., (%g %g), (%g %g)\n",
              //        hw->scan_path_ndata, hw->scan_path_xydata[0], hw->scan_path_xydata[1], hw->scan_path_xydata[2], hw->scan_path_xydata[3], hw->scan_path_xydata[4], hw->scan_path_xydata[5],
              //        hw->scan_path_xydata[2*hw->scan_path_ndata-4], hw->scan_path_xydata[2*hw->scan_path_ndata-3], hw->scan_path_xydata[2*hw->scan_path_ndata-2], hw->scan_path_xydata[2*hw->scan_path_ndata-1]);
 
               if (load_to==hw->scan_path_ndata) hw->loadingscandata = 0;
               else hw->loadingscandata = 1;
            }
    }
    else if (strcmp(todo, "run_scan_path") == 0) {

            printf("run scan path received\n");

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_scan_path"),
                        NULL);
            check_int_value(gwyf, "n", response, &hw->scan_path_ndata, &hw->controlmutex, NULL);

            if (hw->scan_path_ndata>0)
            {
               stop_scan(hw, TRUE);
               pthread_rwlock_rdlock(&hw->controlmutex);
               printf("scan path completed. %d points, values (%g %g), (%g %g), (%g %g)..., (%g %g), (%g %g)\n",
                      hw->scan_path_ndata, hw->scan_path_xydata[0], hw->scan_path_xydata[1], hw->scan_path_xydata[2], hw->scan_path_xydata[3], hw->scan_path_xydata[4], hw->scan_path_xydata[5],
                      hw->scan_path_xydata[2*hw->scan_path_ndata-4], hw->scan_path_xydata[2*hw->scan_path_ndata-3], hw->scan_path_xydata[2*hw->scan_path_ndata-2], hw->scan_path_xydata[2*hw->scan_path_ndata-1]);
               pthread_rwlock_unlock(&hw->controlmutex);

               run_adaptive_scan(hw);
               hw->loadingscandata = 0;
            }
    }
    else if (strcmp(todo, "run_scan_line") == 0) {

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_scan_line"),
                        NULL);

            hw->scan_line_ndata_prev = hw->scan_line_ndata;
            check_int_value(gwyf, "n", response, &hw->scan_line_ndata, &hw->controlmutex, NULL);
            check_double_value(gwyf, "xfrom", response, &hw->scan_line_xfrom, &hw->controlmutex, NULL);
            check_double_value(gwyf, "yfrom", response, &hw->scan_line_yfrom, &hw->controlmutex, NULL);
            check_double_value(gwyf, "xto", response, &hw->scan_line_xto, &hw->controlmutex, NULL);
            check_double_value(gwyf, "yto", response, &hw->scan_line_yto, &hw->controlmutex, NULL);

            hw->lift_usezdata = FALSE;
            valueitem = gwyfile_object_get(gwyf, "z");
            if (valueitem)
            {
               nzvals = gwyfile_item_array_length(valueitem);
               if (nzvals != hw->scan_line_ndata) fprintf(stderr, "Error! Number of points in z data (%d) does not correspond to line resolution (%d)\n", nzvals, hw->scan_line_ndata);
               else {
                  vals = gwyfile_item_get_double_array(valueitem);

                  if (!hw->lift_zdata) hw->lift_zdata = (double *)malloc(nzvals*sizeof(double));
                  else hw->lift_zdata = (double *)realloc(hw->lift_zdata, nzvals*sizeof(double));

                  for (i=0; i<nzvals; i++) hw->lift_zdata[i] = vals[i];
                  hw->lift_usezdata = TRUE;
               }
            }

            run_line_scan(hw);
    }
    else if (strcmp(todo, "run_scan_script") == 0) {

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_scan_script"),
                        NULL);
            check_int_value(gwyf, "n", response, &hw->scan_script_ndata, &hw->controlmutex, NULL);
            check_int_value(gwyf, "scan_id", response, &hw->new_scan_id, &hw->controlmutex, NULL);

            valueitem = gwyfile_object_get(gwyf, "script");
            if (valueitem) {
                snprintf(hw->script, sizeof(hw->script), "%s", gwyfile_item_get_string(valueitem));
                printf("Set script  ----------------------\n%s\n------------------------\n", hw->script);
            }

            run_script_scan(hw);
    }
     else if (strcmp(todo, "run_ramp") == 0) {

            //printf("command run_ramp\n");
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_ramp"),
                        NULL);
            check_int_value(gwyf, "n", response, &hw->ramp_npos, &hw->controlmutex, NULL);
            check_double_value(gwyf, "from", response, &hw->ramp_from, &hw->controlmutex, NULL);
            check_double_value(gwyf, "to", response, &hw->ramp_to, &hw->controlmutex, NULL);
            check_double_value(gwyf, "start_delay", response, &hw->ramp_start_delay, &hw->controlmutex, NULL);
            check_double_value(gwyf, "peak_delay", response, &hw->ramp_peak_delay, &hw->controlmutex, NULL);
            check_double_value(gwyf, "time_up", response, &hw->ramp_time_up, &hw->controlmutex, NULL);
            check_double_value(gwyf, "time_down", response, &hw->ramp_time_down, &hw->controlmutex, NULL);

            check_ramp_quantity(gwyf, "quantity", &hw->ramp_quantity, &hw->controlmutex);

            printf("run ramp with quantity %d\n", hw->ramp_quantity);
            gwyfile_object_add(response, gwyfile_item_new_string_copy("quantity", ramp_quantity_to_string(hw->ramp_quantity)));

            run_ramp(hw);
    }
     else if (strcmp(todo, "move_motor") == 0) {

            printf("command move_motor\n");
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "move_motor"),
                        NULL);
            check_int_value(gwyf, "n", response, &motor_number, NULL, NULL);
            check_double_value(gwyf, "distance", response, &motor_distance, NULL, NULL);

            printf("calling configure exp\n");


            change_spi_mode(&hw->rp, SPI_MODE_3);
            select_EXP(&hw->rp);
            configureEXP(&hw->rp);

            //run_motor(hw, motor_number, motor_distance);
    }
     else if (strcmp(todo, "get_scan_ndata") == 0) {
            pthread_rwlock_rdlock(&hw->controlmutex);
            ndata = hw->scan_ndata;
            scan_id = hw->scan_id;
            pthread_rwlock_unlock(&hw->controlmutex);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_scan_ndata"),
                        gwyfile_item_new_int32("n", ndata),
                        gwyfile_item_new_int32("scan_id", scan_id),
                        NULL);
    }
    else if (strcmp(todo, "get_ramp_ndata") == 0) {

            //printf("command get_ramp_ndata\n");
            pthread_rwlock_rdlock(&hw->controlmutex);
            ndata =  hw->ramp_ndata;
            pthread_rwlock_unlock(&hw->controlmutex);

            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_ramp_ndata"),
                        gwyfile_item_new_int32("n", ndata),
                        NULL);
    }
    else if (strcmp(todo, "get_scan_data") == 0) {
            check_int_value(gwyf, "from", NULL, &from, NULL, NULL);
            check_int_value(gwyf, "to", NULL, &to, NULL, NULL);

            length = to-from;

            pthread_rwlock_rdlock(&hw->controlmutex);
            ndata = hw->scan_ndata;
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_scan_data"),
                        gwyfile_item_new_int32("from", from),
                        gwyfile_item_new_int32("to", to),
                        NULL);
            pthread_rwlock_unlock(&hw->controlmutex);

            if (length > 0)
            {
                //gwyfile_object_add(response, gwyfile_item_new_bool("error", FALSE));
                gwyfile_object_add(response, gwyfile_item_new_double_array_copy("x", hw->scan_x_data + from, length)); //TODO should all this be inside mutex?
                gwyfile_object_add(response, gwyfile_item_new_double_array_copy("y", hw->scan_y_data + from, length));
                gwyfile_object_add(response, gwyfile_item_new_double_array_copy("z", hw->scan_z_data + from, length));
                gwyfile_object_add(response, gwyfile_item_new_double_array_copy("e", hw->scan_e_data + from, length));
                gwyfile_object_add(response, gwyfile_item_new_double_array_copy("ts", hw->scan_ts_data + from, length));

                if (hw->c_sc_a1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a1", hw->scan_a1_data + from, length));
                if (hw->c_sc_p1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p1", hw->scan_p1_data + from, length));
                if (hw->c_sc_a2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a2", hw->scan_a2_data + from, length));
                if (hw->c_sc_p2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p2", hw->scan_p2_data + from, length));
                for (i=0; i<16; i++) {
                    snprintf(description, sizeof(description), "in%d", i+1);
                    if (hw->c_sc_in[i]) {
                        gwyfile_object_add(response, gwyfile_item_new_double_array_copy(description, hw->scan_in_data[i] + from, length));
                        //printf("ch %d  sending %g\n", i, *(hw->scan_in_data[i]+from));

                    }
                }
                if (hw->c_sc_set) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("set", hw->scan_s_data + from, length));
                if (hw->c_sc_fmd) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("fmdrive", hw->scan_fmd_data + from, length));
                if (hw->c_sc_kpfm) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("kpfm", hw->scan_kpfm_data + from, length));
                if (hw->c_sc_dart) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("dart", hw->scan_dart_data + from, length));
                if (hw->c_sc_l1x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1x", hw->scan_l1x_data + from, length));
                if (hw->c_sc_l1y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1y", hw->scan_l1y_data + from, length));
                if (hw->c_sc_l2x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2x", hw->scan_l2x_data + from, length));
                if (hw->c_sc_l2y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2y", hw->scan_l2y_data + from, length));
                if (hw->c_sc_out9) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("out9", hw->scan_out9_data + from, length));
                if (hw->c_sc_slp) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("slp", hw->scan_slp_data + from, length));
            }

    }
    else if (strcmp(todo, "get_ramp_data") == 0) {
            printf("command get_ramp_data\n");

            check_int_value(gwyf, "from", NULL, &from, NULL, NULL);
            check_int_value(gwyf, "to", NULL, &to, NULL, NULL);

            length = to-from;

            pthread_rwlock_rdlock(&hw->controlmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_ramp_data"),
                        gwyfile_item_new_int32("from", from),
                        gwyfile_item_new_int32("to", to),
                        NULL);
            pthread_rwlock_unlock(&hw->controlmutex);

            if (length>0)
            {
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("q", hw->ramp_q_data + from, length)); //should all this be inside mutex?
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("x", hw->ramp_x_data + from, length)); //should all this be inside mutex?
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("y", hw->ramp_y_data + from, length));
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("z", hw->ramp_z_data + from, length));
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("e", hw->ramp_e_data + from, length));
               gwyfile_object_add(response, gwyfile_item_new_double_array_copy("ts", hw->ramp_ts_data + from, length));

               if (hw->c_rm_a1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a1", hw->ramp_a1_data + from, length));
               if (hw->c_rm_p1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p1", hw->ramp_p1_data + from, length));
               if (hw->c_rm_a2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a2", hw->ramp_a2_data + from, length));
               if (hw->c_rm_p2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p2", hw->ramp_p2_data + from, length));
               for (i=0; i<16; i++) {
                  snprintf(description, sizeof(description), "in%d", i+1);
                  if (hw->c_rm_in[i]) gwyfile_object_add(response, gwyfile_item_new_double_array_copy(description, hw->ramp_in_data[i] + from, length));
               }
               if (hw->c_rm_set) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("set", hw->ramp_s_data + from, length));
               if (hw->c_rm_fmd) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("fmdrive", hw->ramp_fmd_data + from, length));
               if (hw->c_rm_kpfm) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("kpfm", hw->ramp_kpfm_data + from, length));
               if (hw->c_rm_dart) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("dart", hw->ramp_dart_data + from, length));
               if (hw->c_rm_l1x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1x", hw->ramp_l1x_data + from, length));
               if (hw->c_rm_l1y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1y", hw->ramp_l1y_data + from, length));
               if (hw->c_rm_l2x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2x", hw->ramp_l2x_data + from, length));
               if (hw->c_rm_l2y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2y", hw->ramp_l2y_data + from, length));
               if (hw->c_rm_out9) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("out9", hw->ramp_out9_data + from, length));
               if (hw->c_rm_slp) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("slp", hw->ramp_slp_data + from, length));
             }

    }
    else if (strcmp(todo, "get_stream_data") == 0) {
            from = hw->stream_lastsent;
            to = hw->stream_ndata;

            //if there is wrap around, send only part of data and ignore rest
            if (to<from) { 
               from = 0;
            }   

            length = to-from;
            hw->stream_lastsent = to;

            pthread_rwlock_rdlock(&hw->controlmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "get_stream_data"),
                        gwyfile_item_new_int32("from", from),
                        gwyfile_item_new_int32("to", to),
                        NULL);
            pthread_rwlock_unlock(&hw->controlmutex);

            //printf("sending %d stream data\n", length);
            if (length>0)
            {
               if (hw->c_st_x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("x", hw->stream_x_data + from, length)); //TODO should all this be inside mutex?
               if (hw->c_st_y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("y", hw->stream_y_data + from, length));
               if (hw->c_st_z) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("z", hw->stream_z_data + from, length));
               if (hw->c_st_e) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("e", hw->stream_e_data + from, length));
               if (hw->c_st_ts) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("ts", hw->stream_ts_data + from, length));

               if (hw->c_st_a1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a1", hw->stream_a1_data + from, length));
               if (hw->c_st_p1) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p1", hw->stream_p1_data + from, length));
               if (hw->c_st_a2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("a2", hw->stream_a2_data + from, length));
               if (hw->c_st_p2) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("p2", hw->stream_p2_data + from, length));
               for (i=0; i<16; i++) {
                  snprintf(description, sizeof(description), "in%d", i+1);
                  if (hw->c_st_in[i]) {
                     gwyfile_object_add(response, gwyfile_item_new_double_array_copy(description, hw->stream_in_data[i] + from, length));
                     //printf("ch %d  sending %g  %s\n", i, *(hw->stream_in_data[i]+from), description);

                  }
               }
               if (hw->c_st_set) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("set", hw->stream_s_data + from, length));
               if (hw->c_st_fmd) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("fmdrive", hw->stream_fmd_data + from, length));
               if (hw->c_st_kpfm) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("kpfm", hw->stream_kpfm_data + from, length));
               if (hw->c_st_dart) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("dart", hw->stream_dart_data + from, length));
               if (hw->c_st_l1x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1x", hw->stream_l1x_data + from, length));
               if (hw->c_st_l1y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l1y", hw->stream_l1y_data + from, length));
               if (hw->c_st_l2x) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2x", hw->stream_l2x_data + from, length));
               if (hw->c_st_l2y) gwyfile_object_add(response, gwyfile_item_new_double_array_copy("l2y", hw->stream_l2y_data + from, length));
            }

    }
    else if (strcmp(todo, "stop_ramp") == 0) {
            pthread_rwlock_wrlock(&hw->controlmutex);
            hw->stop_ramp = 1;
            pthread_rwlock_unlock(&hw->controlmutex);
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop_ramp"),
                        NULL);
    }
    else if (strcmp(todo, "run_stream") == 0) {
            printf("run stream\n");
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_stream"),
                        NULL);

            check_int_value(gwyf, "n", response, &hw->allocated_stream_ndata, &hw->controlmutex, NULL);

            run_stream(hw);
    }
    else if (strcmp(todo, "stop_stream") == 0) {
            printf("stop stream\n");
            response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop_stream"),
                        NULL);
            stop_stream(hw);
    }

    else if (strcmp(todo, "set_stabilizer") == 0) {
        response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "set_stabilizer"),
                        NULL);

        valueitem = gwyfile_object_get(gwyf, "tuning_param");
        if (valueitem) {
            const char* tuning_param = gwyfile_item_get_string(valueitem);
            char* buff = realloc(hw->stabilizer->tuning_param, strlen(tuning_param));
            strcpy(buff, tuning_param);
            hw->stabilizer->tuning_param = buff;
        }

        valueitem = gwyfile_object_get(gwyf, "control_output");
        if (valueitem) {
            const char* control_output = gwyfile_item_get_string(valueitem);
            char* buff = realloc(hw->stabilizer->control_output, strlen(control_output));
            strcpy(buff, control_output);
            hw->stabilizer->control_output = buff;
        }

        check_double_value(gwyf, "step", response, &hw->stabilizer->step, &hw->controlmutex, NULL);
        check_double_value(gwyf, "setpoint", response, &hw->stabilizer->setpoint, &hw->controlmutex, NULL);
        check_double_value(gwyf, "control_range", response, &hw->stabilizer->control_range, &hw->controlmutex, NULL);
        check_int_value(gwyf, "oversampling", response, &hw->stabilizer->oversampling, &hw->controlmutex, NULL);
        gwyfile_object_add(response, gwyfile_item_new_string_copy("tuning_param", hw->stabilizer->tuning_param));
        gwyfile_object_add(response, gwyfile_item_new_string_copy("control_output", hw->stabilizer->control_output));
        gwyfile_object_add(response, gwyfile_item_new_bool("stabilizer_running", hw->stabilizer->mode != STBL_FINISHED));
    }

    else if (strcmp(todo, "run_stabilizer") == 0) {
        response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "run_stabilizer"),
                        NULL);

        Stabilizer* stbl = hw->stabilizer;
        pthread_rwlock_wrlock(&stbl->mutex);
        bool start = stbl_start(hw);
        pthread_rwlock_unlock(&stbl->mutex);

        gwyfile_object_add(response, gwyfile_item_new_bool("stabilizer_running", start));
    }

    else if (strcmp(todo, "stop_stabilizer") == 0) {
        response = gwyfile_object_new("GS",
                        gwyfile_item_new_string_copy("todo", "stop_stabilizer"),
                        NULL);

        Stabilizer* stbl = hw->stabilizer;
        pthread_rwlock_wrlock(&stbl->mutex);
        stbl_stop(hw);
        pthread_rwlock_unlock(&stbl->mutex);

        gwyfile_object_add(response, gwyfile_item_new_bool("stabilizer_running", hw->stabilizer->mode != STBL_FINISHED));
    }

    else {
        fprintf(stderr, "Unknown command: %s\n", todo);
        response = gwyfile_object_new("GS",
                    gwyfile_item_new_string_copy("todo", "unknown"),
                    NULL);
    }
  
    hw->inrequest = 0;

    //printf("response is: %s\n", gwyfile_item_get_string(gwyfile_object_get(response, "todo")));
    return response;
}

static GwyfileObject *
recv_gwyfile_message_blocking(int sock, char **buffer, size_t *bufsize, size_t *have_bytes)
{
    int err;
    GwyfileObject *gwyf;
   
    while (!(gwyf = recv_gwyfile_message2(sock, buffer, bufsize, have_bytes, &err)) && err == EAGAIN);
    if (err < 0) return NULL;

    return gwyf; 
}

/*
* This will handle connection for each client
* */
void *connection_handler(void *hwdata)
{
    HWData *hw = (HWData *)hwdata;

    //Get the socket descriptor
    int* sock = (int*)hw->socket;

    char *buffer = hw->buffer;
    size_t bufsize = hw->bufsize;
    size_t have_bytes = 0;

    GwyfileObject *gwyf, *response;
    int err;

    //if (HWDEBUG) 
    printf("connection handler\n");
   
//    while ((gwyf = recv_gwyfile_message2(sock, &buffer, &bufsize, &have_bytes, &err))) {
    while ((gwyf = recv_gwyfile_message_blocking(*sock, &buffer, &bufsize, &have_bytes))) {
        response = interpret_request(hw, gwyf);
        gwyfile_object_free(gwyf);

        if ((err = send_gwyfile_message(response, *sock, &buffer, &bufsize)) < 0) {
            gwyfile_object_free(response);
            break;
        }
        gwyfile_object_free(response);
    }

    if (err == 0)
        printf("Communication finished successfully.\n");
    else
        fprintf(stderr, "Communication failed.\n");

    close(*sock);
    free(sock);

    return 0;
}


