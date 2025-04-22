
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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "data.h"
#include "statistics.h"
#include "stabilizer.h"
#include "luafunctions.h"
#include "fake.h"

#define DLUF 0

int gws_clear(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua clear called\n");

    hw->scan_ndata = hw->scan_completed_ndata = 0;
    return 0;
}

int gws_get_nvals(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get nvals called\n");

    lua_pushnumber(L, hw->scan_ndata);
    return 1;
}


int gws_get_scan_param(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int i;
    char *string = lua_tostring(L, 2);
    double value = -1;
    if (DLUF) printf("Lua get scan params called\n");

    for (i=0; i<hw->script_param_n; i++) {
        if (strcmp(hw->script_param_key[i], string)==0)
        {
            value = hw->script_param_value[i];
            printf("providing to Lua parameter %s value %g\n", string, value);
        }
    }

    lua_pushnumber(L, value);
    return 1;
}

int gws_check_if_stopped(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    bool stop_scanning_script;

    if (DLUF) printf("Lua check stopped called\n");

    pthread_rwlock_rdlock(&hw->controlmutex);
    stop_scanning_script = hw->stop_scanning_script;
    pthread_rwlock_unlock(&hw->controlmutex);

    lua_pushnumber(L, stop_scanning_script);
    return 1;
}

int gws_check_if_paused(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua check paused called\n");

    lua_pushnumber(L, hw->pause_scan);
    return 1;
}

int gws_get_x(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get x called\n");

    pthread_rwlock_rdlock(&hw->movemutex);
    lua_pushnumber(L, hw->xpos);
    pthread_rwlock_unlock(&hw->movemutex);

    return 1;
}
int gws_get_y(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get y called\n");

    pthread_rwlock_rdlock(&hw->movemutex);
    lua_pushnumber(L, hw->ypos);
    pthread_rwlock_unlock(&hw->movemutex);

    return 1;
}
int gws_get_z(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get z called\n");

    pthread_rwlock_rdlock(&hw->cardmutex);
    lua_pushnumber(L, hw->zpiezo);
    pthread_rwlock_unlock(&hw->cardmutex);

    return 1;
}
int gws_get_e(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get e called\n");

    pthread_rwlock_rdlock(&hw->cardmutex);
    lua_pushnumber(L, hw->errorsignal);
    pthread_rwlock_unlock(&hw->cardmutex);

    return 1;
}
int gws_get_in(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int channel = lua_tonumber(L, 2);
    if (DLUF) printf("Lua get in called\n");

    pthread_rwlock_rdlock(&hw->cardmutex);
    lua_pushnumber(L, hw->in[channel]);
    pthread_rwlock_unlock(&hw->cardmutex);

    return 1;
}

int gws_get_t(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get t called\n");

    pthread_rwlock_rdlock(&hw->cardmutex);
    lua_pushnumber(L, hw->timestamp);
    pthread_rwlock_unlock(&hw->cardmutex);

    return 1;
}


int gws_get_z_at(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get_z_at called\n");

    int i, minpos, index_from, index_to, set;
    double x, y, result, dist, mindist;

    x = lua_tonumber(L, 2);
    y = lua_tonumber(L, 3);
    index_from = lua_tonumber(L, 4);
    index_to = lua_tonumber(L, 5);
    set = lua_tonumber(L, 6);

    if (DLUF) printf("getz: from %d to %d  x %g y %g  set %d\n", index_from, index_to, x, y, set); 

    if (index_from < 0) index_from = 0;
    if (index_to < 0) index_to = 0;
    if (index_to > hw->scan_ndata) index_to = hw->scan_ndata;
    if (index_to > hw->scan_ndata) index_to = hw->scan_ndata;

    mindist = 1e10;
    minpos = 0;
    for (i=index_from; i<index_to; i++) {
        if (DLUF) printf("i %d  set %g\n", i, hw->scan_s_data[i]);
        if (hw->scan_s_data[i] == set) {
           dist = (x - hw->scan_x_data[i])*(x - hw->scan_x_data[i]) + (y - hw->scan_y_data[i])*(y - hw->scan_y_data[i]);
           if (dist<mindist) {
               mindist = dist;
               minpos = i;
           }
        }
    }
    result = hw->scan_z_data[minpos];

    lua_pushnumber(L, result);
    return 1;
}
int gws_get_entry(lua_State *L)
{
    int index;
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    index = lua_tonumber(L, 2);
    if (DLUF) printf("Lua get entry called\n");

    if (index<hw->scan_ndata) {
        lua_pushnumber(L, hw->scan_x_data[index]);
        lua_pushnumber(L, hw->scan_y_data[index]);
        lua_pushnumber(L, hw->scan_z_data[index]);
        lua_pushnumber(L, hw->scan_e_data[index]);
        lua_pushnumber(L, hw->scan_ts_data[index]);
        lua_pushnumber(L, hw->scan_s_data[index]);
    }
    else {
        lua_pushnumber(L, -1);
        lua_pushnumber(L, -1);
        lua_pushnumber(L, -1);
        lua_pushnumber(L, -1);
        lua_pushnumber(L, -1);
        lua_pushnumber(L, -1);
    }
    return 8;
}

int gws_move_to(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double x, y, z, xapos, yapos, zapos;
    bool is_z = FALSE, is_xy = FALSE, xy_done = FALSE, z_done = FALSE, feedback = FALSE, stop_scanning_script = FALSE;
    if (DLUF) printf("Lua move to called\n");

    x = lua_tonumber(L, 2);
    y = lua_tonumber(L, 3);
    z = lua_tonumber(L, 4);

    double dist, xdir, ydir, zdir, zdist;
    pthread_rwlock_wrlock(&hw->controlmutex);
    xdir = x - hw->xpos;
    ydir = y - hw->ypos;
    zdir = z - hw->zpiezo;
    feedback = hw->feedback;
    pthread_rwlock_unlock(&hw->controlmutex);

    dist = sqrt(xdir*xdir + ydir*ydir);
    zdist = fabs(zdir);
    is_xy = dist > 1e-15;
    is_z = feedback == FALSE && zdist > 1e-15;
    if (!is_xy && !is_z) {
        return 0;
    }

    pthread_rwlock_wrlock(&hw->controlmutex);
    if (is_z) {
        hw->zreq = z;
    }

    if (is_xy) {
        hw->xreq = x;
        hw->yreq = y;
    }
    pthread_rwlock_unlock(&hw->controlmutex);


    pthread_rwlock_wrlock(&hw->movemutex);
    if (is_z) {
        hw->zmoveto = 1;
    }

    if (is_xy) {
        hw->moveto = 1;
    }
    pthread_rwlock_unlock(&hw->movemutex);

    //printf("gws_move_to called\n");
    for (int i=0; i<1000000; i++)
    {
        usleep(100);

        pthread_rwlock_rdlock(&hw->controlmutex);
        xapos = hw->xpos;
        yapos = hw->ypos;
        zapos = hw->zpos;
        stop_scanning_script = hw->stop_scanning_script;
        pthread_rwlock_unlock(&hw->controlmutex);

        xy_done = is_xy ? xapos == x && yapos == y : TRUE;
        z_done = is_z ? zapos == z : TRUE;

        if ((xy_done && z_done) || stop_scanning_script == TRUE) break; //could use also moving status
    }
    //printf("move completed in %d steps\n", i);

    return 0;
}

int gws_set_feedback(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int feedback = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set feedback called\n");

    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->feedback = feedback;
    librp_SetFeedback(&hw->rp, hw->feedback);
    pthread_rwlock_unlock(&hw->controlmutex);

    return 0;
}

int gws_set_speed(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double speed = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set speed called\n");

    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->speed = speed;
    pthread_rwlock_unlock(&hw->controlmutex);
    return 0;
}


//store data at actual x and y, add z, use set number
int gws_store_point(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int set, n;
    if (DLUF) printf("Lua store point called\n");

    set = lua_tonumber(L, 2);

    if (hw->isFake) {
        n = store_fake_point_data(hw, set);
    } else {
        n = store_actual_point_data(hw, set);
    }
    if (DLUF && n%100==0) printf("stored point %d z %g\n", n, hw->scan_z_data[n]);
    lua_pushnumber(L, n);

    return 1;
}

int gws_set_fm_feedback(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int feedback = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set fm feedback called\n");

    hw->pllfeedback = feedback;
    librp_SetPLL(&hw->rp, feedback, hw->ampfeedback, hw->pidpll_setpoint, hw->pidamplitude_setpoint,
                hw->pidpll_p, hw->pidpll_i, hw->pidpll_d, 
                hw->pidamplitude_p, hw->pidamplitude_i, hw->pidamplitude_d, 
                hw->pll_phase_limit_factor, hw->pll_frequency_limit_factor, 
                hw->modeset[hw->mode].pllskip, hw->modeset[hw->mode].pll_input);

    return 0;
}

int gws_set_fm_amplitude_feedback(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int feedback = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set fm amplitude feedback called\n");

    hw->ampfeedback = feedback;
    librp_SetPLL(&hw->rp, hw->pllfeedback, feedback, hw->pidpll_setpoint, hw->pidamplitude_setpoint, 
                hw->pidpll_p, hw->pidpll_i, hw->pidpll_d, 
                hw->pidamplitude_p, hw->pidamplitude_i, hw->pidamplitude_d, 
                hw->pll_phase_limit_factor, hw->pll_frequency_limit_factor, 
                hw->modeset[hw->mode].pllskip, hw->modeset[hw->mode].pll_input);

    return 0;
}

int gws_set_kpfm_feedback(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int feedback = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set fm KPFM feedback called\n");

    hw->kpfm_feedback = feedback;

    return 0;
}

int gws_set_gen_amplitude(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int channel = lua_tonumber(L, 2);
    double amplitude = lua_tonumber(L, 3);
    if (DLUF) printf("Lua set amplitude called\n");

    if (channel==LIBRP_CH_1) {
        hw->f1_amplitude = amplitude;
        librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, amplitude, hw->f1_offset);
    }
    else {
        hw->f2_amplitude = amplitude;
        librp_SetGen(&hw->rp, LIBRP_CH_2, hw->f2_frequency, amplitude, hw->f2_offset);
    }

    return 0;
}

int gws_usleep(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int sleeptime = lua_tonumber(L, 2);
    if (DLUF) printf("Lua usleep called\n");

    usleep(sleeptime);

    return 0;
}



int gws_scan_and_store(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double xto, yto;
    int i, n, res, set, scanning;
    bool stop_scanning_script = FALSE;
    if (DLUF) printf("Lua line scan called\n");

    xto = lua_tonumber(L, 2);
    yto = lua_tonumber(L, 3);
    res = lua_tonumber(L, 4);
    set = lua_tonumber(L, 5);

    hw->lift_usezdata = FALSE;

    //printf("running ls %g %g %d %d\n", xto, yto, res, set);
    run_line_and_store(hw, xto, yto, res, set);
    usleep(100000);

    for (i=0; i<1000000; i++)
    {
        pthread_rwlock_rdlock(&hw->controlmutex);
        stop_scanning_script = hw->stop_scanning_script;
        scanning = hw->scanning_line;
        pthread_rwlock_unlock(&hw->controlmutex);

        if (stop_scanning_script == TRUE || !scanning) break;

        usleep(100);
    }
    hw->scan_completed_ndata = hw->scan_ndata;
    n = hw->scan_ndata;
    //printf("ls %g %g completed in %d iterations  ndata %d\n", xto, yto, i, hw->scan_ndata);

    lua_pushnumber(L, n);
 
    return 1;
}

int gws_scan_and_store_lift(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double xto, yto, lift;
    int i, res, set, scanning, zstart, zindex;
    bool nofbaction, stop_scanning_script = FALSE;
    if (DLUF) printf("Lua line lift scan called\n");

    xto = lua_tonumber(L, 2);
    yto = lua_tonumber(L, 3);
    res = lua_tonumber(L, 4);
    set = lua_tonumber(L, 5);
    zstart = lua_tonumber(L, 6);
    lift = lua_tonumber(L, 7);
    nofbaction = lua_tonumber(L, 8);

    if (!hw->lift_zdata) hw->lift_zdata = (double *)malloc(res*sizeof(double));
    else hw->lift_zdata = (double *)realloc(hw->lift_zdata, res*sizeof(double));

    for (i=0; i<res; i++) {
       zindex = zstart + i;
       if (zindex>=0 && zindex<hw->scan_ndata)
           hw->lift_zdata[i] = hw->scan_z_data[zindex] + lift;
       else hw->lift_zdata[i] = 0;

       //printf("%d %d %g\n", i, zindex, hw->lift_zdata[i]);
    }
    hw->lift_usezdata = TRUE;
    hw->lift_no_fb_action = nofbaction;

    //printf("running ls lift %g %g %d %d\n", xto, yto, res, set);
    run_line_and_store(hw, xto, yto, res, set);
    usleep(100000);

    for (i=0; i<1000000; i++)
    {
        pthread_rwlock_rdlock(&hw->controlmutex);
        stop_scanning_script = hw->stop_scanning_script;
        scanning = hw->scanning_line;
        pthread_rwlock_unlock(&hw->controlmutex);

        if (stop_scanning_script == TRUE || !scanning) break;

        usleep(100);
    }
    hw->scan_completed_ndata = hw->scan_ndata;
    //printf("ls lift %g %g completed in %d iterations  ndata %d\n", xto, yto, i, hw->scan_ndata);
 
    return 0;
}

int gws_set_pidamplitude_setpoint(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double pidamplitude_setpoint = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set amplitude pid setpoint called\n");
 
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->pidamplitude_setpoint = pidamplitude_setpoint;
    pthread_rwlock_unlock(&hw->controlmutex);

    librp_SetPLL(&hw->rp, hw->pllfeedback, hw->ampfeedback, hw->pidpll_setpoint, hw->pidamplitude_setpoint,
                hw->pidpll_p, hw->pidpll_i, hw->pidpll_d, 
                hw->pidamplitude_p, hw->pidamplitude_i, hw->pidamplitude_d, 
                hw->pll_phase_limit_factor, hw->pll_frequency_limit_factor, 
                hw->modeset[hw->mode].pllskip, hw->modeset[hw->mode].pll_input);

    return 0;
}


int gws_get_speed(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get speed called\n");
 
    pthread_rwlock_rdlock(&hw->cardmutex);
    lua_pushnumber(L, hw->speed);
    pthread_rwlock_unlock(&hw->cardmutex);
 
    return 1;
}


int gws_get(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua get called\n");
    char *string = lua_tostring(L, 2);

    if (strcmp(string, "hwtime") == 0) lua_pushnumber(L, ((double)hw->start.tv_sec + (double)hw->start.tv_nsec / 1.0e9));
    else if (strcmp(string, "moving") == 0) lua_pushboolean(L, hw->moving && hw->zmoving);
    else if (strcmp(string, "scanning_line") == 0) lua_pushboolean(L, hw->scanning_line);
    else if (strcmp(string, "scanning_adaptive") == 0) lua_pushboolean(L, hw->scanning_adaptive);
    else if (strcmp(string, "scanning_script") == 0) lua_pushboolean(L, hw->scanning_script);
    else if (strcmp(string, "ramp_running") == 0) lua_pushboolean(L, hw->ramp_running);
    else if (strcmp(string, "pid_p") == 0) lua_pushnumber(L, hw->pid_p);
    else if (strcmp(string, "pid_i") == 0) lua_pushnumber(L, hw->pid_i);
    else if (strcmp(string, "pid_d") == 0) lua_pushnumber(L, hw->pid_d);
    else if (strcmp(string, "pid_pll_p") == 0) lua_pushnumber(L, hw->pidpll_p);
    else if (strcmp(string, "pid_pll_i") == 0) lua_pushnumber(L, hw->pidpll_i);
    else if (strcmp(string, "pid_pll_d") == 0) lua_pushnumber(L, hw->pidpll_d);
    else if (strcmp(string, "pid_amplitude_p") == 0) lua_pushnumber(L, hw->pidamplitude_p);
    else if (strcmp(string, "pid_amplitude_i") == 0) lua_pushnumber(L, hw->pidamplitude_i);
    else if (strcmp(string, "pid_amplitude_d") == 0) lua_pushnumber(L, hw->pidamplitude_d);
    else if (strcmp(string, "pid_kpfm_p") == 0) lua_pushnumber(L, hw->pidkpfm_p);
    else if (strcmp(string, "pid_kpfm_i") == 0) lua_pushnumber(L, hw->pidkpfm_i);
    else if (strcmp(string, "pid_kpfm_d") == 0) lua_pushnumber(L, hw->pidkpfm_d);
    else if (strcmp(string, "pid_dart_p") == 0) lua_pushnumber(L, hw->piddart_p);
    else if (strcmp(string, "pid_dart_i") == 0) lua_pushnumber(L, hw->piddart_i);
    else if (strcmp(string, "pid_dart_d") == 0) lua_pushnumber(L, hw->piddart_d);
    else if (strcmp(string, "pid_setpoint") == 0) lua_pushnumber(L, hw->pid_setpoint);
    else if (strcmp(string, "pid_pll_setpoint") == 0) lua_pushnumber(L, hw->pidpll_setpoint);
    else if (strcmp(string, "pid_amplitude_setpoint") == 0) lua_pushnumber(L, hw->pidamplitude_setpoint);
    else if (strcmp(string, "pll_phase_limit_factor") == 0) lua_pushnumber(L, hw->pll_phase_limit_factor);
    else if (strcmp(string, "pll_frequency_limit_factor") == 0) lua_pushnumber(L, hw->pll_frequency_limit_factor);
    else if (strcmp(string, "phaseshift1") == 0) lua_pushnumber(L, hw->phaseshift1);
    else if (strcmp(string, "phaseshift2") == 0) lua_pushnumber(L, hw->phaseshift2);
    else if (strcmp(string, "freq1_f") == 0) lua_pushnumber(L, hw->f1_frequency);
    else if (strcmp(string, "freq1_a") == 0) lua_pushnumber(L, hw->f1_amplitude);
    else if (strcmp(string, "freq1_o") == 0) lua_pushnumber(L, hw->f1_offset);
    else if (strcmp(string, "freq2_f") == 0) lua_pushnumber(L, hw->f2_frequency);
    else if (strcmp(string, "freq2_a") == 0) lua_pushnumber(L, hw->f2_amplitude);
    else if (strcmp(string, "freq2_o") == 0) lua_pushnumber(L, hw->f2_offset);
    else if (strcmp(string, "freq3_f") == 0) lua_pushnumber(L, hw->f3_frequency);
    else if (strcmp(string, "filter1") == 0) lua_pushnumber(L, hw->filter1);
    else if (strcmp(string, "filter2") == 0) lua_pushnumber(L, hw->filter2);
    else if (strcmp(string, "kpfm_mode") == 0) lua_pushnumber(L, hw->kpfm_mode);
    else if (strcmp(string, "kpfm_feedback_source") == 0) lua_pushnumber(L, hw->debugsource2);
    else if (strcmp(string, "kpfm_feedback_direction") == 0) lua_pushnumber(L, hw->kpfm_dir);
    else if (strcmp(string, "dart_mode") == 0) lua_pushnumber(L, hw->dart_mode);
    else if (strcmp(string, "dart_frequency") == 0) lua_pushnumber(L, hw->dart_frequency);
    else if (strcmp(string, "dart_amplitude") == 0) lua_pushnumber(L, hw->dart_amplitude);
    else if (strcmp(string, "dart_freqspan") == 0) lua_pushnumber(L, hw->dart_freqspan);
    else if (strcmp(string, "oversampling") == 0) lua_pushnumber(L, hw->oversampling);
    else if (strcmp(string, "feedback_pll") == 0) lua_pushboolean(L, hw->pllfeedback);
    else if (strcmp(string, "feedback_amplitude") == 0) lua_pushboolean(L, hw->ampfeedback);
    else if (strcmp(string, "feedback") == 0) lua_pushboolean(L, hw->feedback);
    else if (strcmp(string, "lockin1_nwaves") == 0) lua_pushnumber(L, hw->modeset[hw->mode].lockin1_nwaves);
    else if (strcmp(string, "lockin2_nwaves") == 0) lua_pushnumber(L, hw->modeset[hw->mode].lockin2_nwaves);

    return 1;
}

int gws_set(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua set called\n");
    char *string = lua_tostring(L, 2);
    
    if (strcmp(string, "lockin1_nwaves") == 0) {
         hw->modeset[hw->mode].lockin1_nwaves = (int)lua_tonumber(L, 3);
         set_fm_feedback(hw);
    }
    else if (strcmp(string, "lockin2_nwaves") == 0) {
         hw->modeset[hw->mode].lockin2_nwaves = (int)lua_tonumber(L, 3);
         set_fm_feedback(hw);
    }

    return 0;
}

int gws_set_output(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int channel_number = lua_tonumber(L, 2);
    double value = lua_tonumber(L, 3);
    if (DLUF) printf("Lua set output called\n");

    if (channel_number >= 1 && channel_number <= 16) {
        pthread_rwlock_wrlock(&hw->controlmutex);
        hw->aout[channel_number - 1] = value;
        hw->isaout = 1;
        pthread_rwlock_unlock(&hw->controlmutex);
    }

    return 0;
}

int gws_set_zpiezo(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    double value = lua_tonumber(L, 2);
    if (DLUF) printf("Lua set zpiezo called\n");

    pthread_rwlock_wrlock(&hw->controlmutex);
    set_zpiezo(hw, value, FALSE);
    pthread_rwlock_unlock(&hw->controlmutex);

    return 0;
}

int gws_set_zpiezo_to_actual(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    if (DLUF) printf("Lua set zpiezo to actual called\n");

    pthread_rwlock_wrlock(&hw->controlmutex);
    set_zpiezo(hw, hw->zpiezo, FALSE);
    pthread_rwlock_unlock(&hw->controlmutex);

    return 0;
}


//get correlation length from scanned data between given indices
int gws_get_correlation_length(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int start, end, n;
    double result;

    if (DLUF)
        printf("Lua get correlation length called\n");

    start = lua_tonumber(L, 2);
    end = lua_tonumber(L, 3);
    n = end - start;

    if (n > 0)
       result = get_correlation_length(hw->scan_z_data + start, n);
    else
        result = 1;

    lua_pushnumber(L, result);

    return 1;
}

//get period from scanned data between given indices
int gws_get_period(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int start, end, n;
    double result;

    if (DLUF) printf("Lua get period called\n");

    start = lua_tonumber(L, 2);
    end = lua_tonumber(L, 3);
    n = end - start;

    if (n>0)
       result = get_period(hw->scan_z_data + start, n);
    else result = 1;

    lua_pushnumber(L, result);

    return 1;
}

//get optimum angle from five scanned data arrays between given indices
int gws_get_optimum_angle(lua_State *L)
{
    HWData* hw = (HWData *)lua_touserdata(L, 1);
    int i, start[5], end[5], n[5];
    double angle[5];
    double period[5];
    double result;
    double coeff[3];

    if (DLUF) printf("Lua get period called\n");

    for (i = 0; i<5; i++) {
       start[i] = lua_tonumber(L, 3*i + 2);
       end[i] = lua_tonumber(L, 3*i + 3);
       angle[i] = lua_tonumber(L, 3*i + 4);
       n[i] = end[i] - start[i];
       if (n[i]>0)
          period[i] = get_period(hw->scan_z_data + start[i], n[i]);
       else period[i] = 1; //this would mean chaos
       printf("period for %d %d   %g     %g\n", start[i], end[i], angle[i], period[i]);
    }

    fit_parabola(angle, period, 5, coeff);
    printf("coeffs: %g %g %g   angle %g rad  %g deg\n", coeff[0], coeff[1], coeff[2], -coeff[1]/(2*coeff[2]), -coeff[1]/(2*coeff[2])*180/M_PI);

    if (coeff[1]==0 || coeff[2]==0) result = 0;
    else result = -coeff[1]/(2*coeff[2]);

    lua_pushnumber(L, result);

    return 1;
}

int gws_stabilize_setpoint(lua_State* L) {
    bool stop_scanning_script = FALSE;
    HWData* hw = (HWData *)lua_touserdata(L, 1);

    Stabilizer* stbl = hw->stabilizer;
    pthread_rwlock_wrlock(&stbl->mutex);
    stbl_start(hw);
    pthread_rwlock_unlock(&stbl->mutex);

    usleep(100000);

    bool succesful = FALSE;
    for (int i = 0; i < 1000000; i++) {
        pthread_rwlock_rdlock(&stbl->mutex);
        StabilizerMode mode = stbl->mode;
        pthread_rwlock_unlock(&stbl->mutex);

        pthread_rwlock_rdlock(&hw->controlmutex);
        stop_scanning_script = hw->stop_scanning_script;
        pthread_rwlock_unlock(&hw->controlmutex);

        if (mode == STBL_FINISHED) {
            succesful = TRUE;
            break;
        }

        if (stop_scanning_script == TRUE) {
            break;
        }

        usleep(100);
    }

    if (!succesful) {
        pthread_rwlock_wrlock(&stbl->mutex);
        stbl_stop(hw);
        pthread_rwlock_unlock(&stbl->mutex);
    }

    int result = hw->stabilizer->adjusted ? 1 : 0;
    lua_pushnumber(L, result);

    return 1;
}

void register_functions(lua_State *L)
{
  lua_pushcfunction(L, gws_clear);
  lua_setglobal(L, "gws_clear");

  lua_pushcfunction(L, gws_get_nvals);
  lua_setglobal(L, "gws_get_nvals");

  lua_pushcfunction(L, gws_get_scan_param);
  lua_setglobal(L, "gws_get_scan_param");

  lua_pushcfunction(L, gws_check_if_stopped);
  lua_setglobal(L, "gws_check_if_stopped");

  lua_pushcfunction(L, gws_check_if_paused);
  lua_setglobal(L, "gws_check_if_paused");

  lua_pushcfunction(L, gws_get_x);
  lua_setglobal(L, "gws_get_x");

  lua_pushcfunction(L, gws_get_y);
  lua_setglobal(L, "gws_get_y");

  lua_pushcfunction(L, gws_get_z);
  lua_setglobal(L, "gws_get_z");

  lua_pushcfunction(L, gws_get_e);
  lua_setglobal(L, "gws_get_e");

  lua_pushcfunction(L, gws_get_in);
  lua_setglobal(L, "gws_get_in1");

  lua_pushcfunction(L, gws_get_t);
  lua_setglobal(L, "gws_get_t");

  lua_pushcfunction(L, gws_get_z_at);
  lua_setglobal(L, "gws_get_z_at");

  lua_pushcfunction(L, gws_get_entry);
  lua_setglobal(L, "gws_get_entry");

  lua_pushcfunction(L, gws_set_feedback);
  lua_setglobal(L, "gws_set_feedback");

  lua_pushcfunction(L, gws_set_speed);
  lua_setglobal(L, "gws_set_speed");

  lua_pushcfunction(L, gws_get_speed);
  lua_setglobal(L, "gws_get_speed");

  lua_pushcfunction(L, gws_move_to);
  lua_setglobal(L, "gws_move_to");

  lua_pushcfunction(L, gws_store_point);
  lua_setglobal(L, "gws_store_point");

  lua_pushcfunction(L, gws_set_fm_feedback);
  lua_setglobal(L, "gws_set_fm_feedback");

  lua_pushcfunction(L, gws_set_fm_amplitude_feedback);
  lua_setglobal(L, "gws_set_fm_amplitude_feedback");

  lua_pushcfunction(L, gws_set_kpfm_feedback);
  lua_setglobal(L, "gws_set_kpfm_feedback");

  lua_pushcfunction(L, gws_set_gen_amplitude);
  lua_setglobal(L, "gws_set_gen_amplitude");

  lua_pushcfunction(L, gws_set_pidamplitude_setpoint);
  lua_setglobal(L, "gws_set_pidamplitude_setpoint");

  lua_pushcfunction(L, gws_usleep);
  lua_setglobal(L, "gws_usleep");

  lua_pushcfunction(L, gws_scan_and_store);
  lua_setglobal(L, "gws_scan_and_store");

  lua_pushcfunction(L, gws_scan_and_store_lift);
  lua_setglobal(L, "gws_scan_and_store_lift");

  lua_pushcfunction(L, gws_get);
  lua_setglobal(L, "gws_get");

  lua_pushcfunction(L, gws_set);
  lua_setglobal(L, "gws_set");

  lua_pushcfunction(L, gws_set_output);
  lua_setglobal(L, "gws_set_output");

  lua_pushcfunction(L, gws_set_zpiezo);
  lua_setglobal(L, "gws_set_zpiezo");

  lua_pushcfunction(L, gws_set_zpiezo_to_actual);
  lua_setglobal(L, "gws_set_zpiezo_to_actual");

  lua_pushcfunction(L, gws_get_correlation_length);
  lua_setglobal(L, "gws_get_correlation_length");

  lua_pushcfunction(L, gws_get_period);
  lua_setglobal(L, "gws_get_period");

  lua_pushcfunction(L, gws_get_optimum_angle);
  lua_setglobal(L, "gws_get_optimum_angle");

  lua_pushcfunction(L, gws_stabilize_setpoint);
  lua_setglobal(L, "gws_stabilize_setpoint");
}

void run_lua_scan(HWData *hw)
{

  lua_State *L = luaL_newstate();

  printf("lua open libs\n");
  luaL_openlibs(L);

  printf("lua push light user data\n");
  lua_pushlightuserdata(L, (void*)hw);

  printf("lua set global\n");
  lua_setglobal(L, "p");

  printf("lua dofile\n");
  luaL_dofile(L, "scan.lua");
  lua_setglobal(L, "scan");
  lua_settop(L, 0);

  printf("lua register functions\n");
  register_functions(L);
  lua_getglobal(L, "scan");
  lua_getfield(L, -1, "runit");

  printf("lua call\n");
  lua_call(L, 0, 0);

  printf("Lua completed, we got %d values\n", hw->scan_ndata);

}



