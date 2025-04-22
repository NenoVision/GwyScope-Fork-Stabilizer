
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

#include "gwyfile.h"
#include "clientserver.h"

#ifdef FAKE
#include "myrpfake.h"
#else
#include "cmirp.h"
#endif

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0


#include "data.h"

int modehandler(HWData *hw, int mode, const char* section, const char* name,
                   const char* value)
{
    char modename[50];
    sprintf(modename, "mode%d", mode);

    if (MATCH(modename, "name")) {
        sprintf(hw->modeset[mode].name, "%s", value); //this is strange and might rename mode?
    } else if (MATCH(modename, "mux1")) {
        hw->modeset[mode].mux1 = atoi(value);
    } else if (MATCH(modename, "mux2")) {
        hw->modeset[mode].mux2 = atoi(value);
    } else if (MATCH(modename, "error_source")) {
        hw->modeset[mode].error_source = atoi(value);
    } else if (MATCH(modename, "swap_in")) {
        hw->modeset[mode].swap_in = atoi(value);
    } else if (MATCH(modename, "swap_out")) {
        hw->modeset[mode].swap_out = atoi(value);
    } else if (MATCH(modename, "error_bit_shift")) {
        hw->modeset[mode].bitshift = atoi(value);
    } else if (MATCH(modename, "pll")) {
        hw->modeset[mode].pll = atoi(value);
    } else if (MATCH(modename, "pll_input")) {
        hw->modeset[mode].pll_input = atoi(value);
    } else if (MATCH(modename, "input1_range")) {
        hw->modeset[mode].input1_range = atoi(value);
    } else if (MATCH(modename, "input2_range")) {
        hw->modeset[mode].input2_range = atoi(value);
    } else if (MATCH(modename, "lockin1_hr")) {
        hw->modeset[mode].lockin1_hr = atoi(value);
    } else if (MATCH(modename, "lockin2_hr")) {
        hw->modeset[mode].lockin2_hr = atoi(value);
    } else if (MATCH(modename, "lockin1_nwaves")) {
        hw->modeset[mode].lockin1_nwaves = atoi(value);
    } else if (MATCH(modename, "lockin2_nwaves")) {
        hw->modeset[mode].lockin2_nwaves = atoi(value);
    } else if (MATCH(modename, "lockin1_filter_amplitude")) {
        hw->modeset[mode].lockin1_filter_amplitude = atoi(value);
    } else if (MATCH(modename, "lockin1_filter_phase")) {
        hw->modeset[mode].lockin1_filter_phase = atoi(value);
    } else if (MATCH(modename, "lockin2_filter_amplitude")) {
        hw->modeset[mode].lockin2_filter_amplitude = atoi(value);
    } else if (MATCH(modename, "lockin2_filter_phase")) {
        hw->modeset[mode].lockin2_filter_phase = atoi(value);
    } else if (MATCH(modename, "pidskip")) {
        hw->modeset[mode].pidskip = atoi(value);
    } else if (MATCH(modename, "pllskip")) {
        hw->modeset[mode].pllskip = atoi(value);
    } else if (MATCH(modename, "out1")) {
        hw->modeset[mode].out1 = atoi(value);
    } else if (MATCH(modename, "out2")) {
        hw->modeset[mode].out2 = atoi(value);
    } else if (MATCH(modename, "outhr")) {
        hw->modeset[mode].outhr = atoi(value);
    } else if (MATCH(modename, "lockin1_lf")) {
        hw->modeset[mode].lockin1_lfr = atoi(value);
    } else if (MATCH(modename, "lockin2_lf")) {
        hw->modeset[mode].lockin2_lfr = atoi(value);
    } else return 0;

    return 1;  
}

int inihandler(void* user, const char* section, const char* name,
                   const char* value)
{
    HWData* hw = (HWData*)user;

    if (MATCH("general", "debug")) {
        hw->debug = atoi(value);
    } else if (MATCH("hardware", "scan_mode")) {
        hw->xymode = atoi(value);
    } else if (MATCH("hardware", "xrange")) {
        hw->xrange = atof(value);
    } else if (MATCH("hardware", "yrange")) {
        hw->yrange = atof(value);
    } else if (MATCH("hardware", "zrange")) {
        hw->zrange = atof(value);
    } else if (MATCH("hardware", "hrdac_regime")) {
        hw->hrdac_regime = atoi(value);
    } else if (MATCH("hardware", "hrdac1_range")) {
        hw->hrdac1_range = atoi(value);
    } else if (MATCH("hardware", "hrdac2_range")) {
        hw->hrdac2_range = atoi(value);
    } else if (MATCH("hardware", "hrdac3_range")) {
        hw->hrdac3_range = atoi(value);
    } else if (MATCH("hardware", "dds1_range")) {
        hw->dds1_range = atoi(value);
    } else if (MATCH("hardware", "dds2_range")) {
        hw->dds2_range = atoi(value);
    } else if (MATCH("hardware", "rp1_input_hv")) {
        hw->rp1_input_hv = atoi(value);
    } else if (MATCH("hardware", "rp2_input_hv")) {
        hw->rp2_input_hv = atoi(value);
    } else if (MATCH("hardware", "rp_bare_output")) {
        hw->rp_bare_output = atoi(value);
    } else if (MATCH("hardware", "timestep")) {
        hw->scantimestep = atoi(value);
    } else if (MATCH("hardware", "oversampling")) {
        hw->oversampling = atoi(value);
    } else if (MATCH("hardware", "rp_bare_input")) {
        hw->rp_bare_input = atoi(value);
    } else if (MATCH("hardware", "trigger_data_out")) {
        hw->triggerout = atoi(value);
    } else if (MATCH("hardware", "x_external")) {
        hw->x_external = atoi(value);
    } else if (MATCH("hardware", "y_external")) {
        hw->y_external = atoi(value);
    } else if (MATCH("hardware", "x_external_source")) {
        hw->x_external_source = atoi(value);
    } else if (MATCH("hardware", "y_external_source")) {
        hw->y_external_source = atoi(value);
    } else if (MATCH("hardware", "x_external_offset")) {
        hw->x_external_offset = atof(value);
    } else if (MATCH("hardware", "y_external_offset")) {
        hw->y_external_offset = atof(value);
    } else if (MATCH("hardware", "x_external_slope")) {
        hw->x_external_slope = atof(value);
    } else if (MATCH("hardware", "y_external_slope")) {
        hw->y_external_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_bare_lv_offset")) {
        hw->rpadc1_bare_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_bare_lv_slope")) {
        hw->rpadc1_bare_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_bare_hv_offset")) {
        hw->rpadc1_bare_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_bare_hv_slope")) {
        hw->rpadc1_bare_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_bare_lv_offset")) {
        hw->rpadc2_bare_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_bare_lv_slope")) {
        hw->rpadc2_bare_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_bare_hv_offset")) {
        hw->rpadc2_bare_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_bare_hv_slope")) {
        hw->rpadc2_bare_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_divhigh_lv_offset")) {
        hw->rpadc1_divhigh_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_divhigh_lv_slope")) {
        hw->rpadc1_divhigh_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_divhigh_hv_offset")) {
        hw->rpadc1_divhigh_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_divhigh_hv_slope")) {
        hw->rpadc1_divhigh_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_divlow_lv_offset")) {
        hw->rpadc1_divlow_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_divlow_lv_slope")) {
        hw->rpadc1_divlow_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc1_divlow_hv_offset")) {
        hw->rpadc1_divlow_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc1_divlow_hv_slope")) {
        hw->rpadc1_divlow_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_divhigh_lv_offset")) {
        hw->rpadc2_divhigh_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_divhigh_lv_slope")) {
        hw->rpadc2_divhigh_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_divhigh_hv_offset")) {
        hw->rpadc2_divhigh_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_divhigh_hv_slope")) {
        hw->rpadc2_divhigh_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_divlow_lv_offset")) {
        hw->rpadc2_divlow_lv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_divlow_lv_slope")) {
        hw->rpadc2_divlow_lv_slope = atof(value);
    } else if (MATCH("hardware", "rpadc2_divlow_hv_offset")) {
        hw->rpadc2_divlow_hv_offset = atof(value);
    } else if (MATCH("hardware", "rpadc2_divlow_hv_slope")) {
        hw->rpadc2_divlow_hv_slope = atof(value);
    } else if (MATCH("hardware", "rpdac1_bare_offset")) {
        hw->rpdac1_bare_offset = atof(value);
    } else if (MATCH("hardware", "rpdac1_bare_slope")) {
        hw->rpdac1_bare_slope = atof(value);
    } else if (MATCH("hardware", "rpdac2_bare_offset")) {
        hw->rpdac2_bare_offset = atof(value);
    } else if (MATCH("hardware", "rpdac2_bare_slope")) {
        hw->rpdac2_bare_slope = atof(value);
    } else if (MATCH("hardware", "rpdac1_offset")) {
        hw->rpdac1_offset = atof(value);
    } else if (MATCH("hardware", "rpdac1_slope")) {
        hw->rpdac1_slope = atof(value);
    } else if (MATCH("hardware", "rpdac2_offset")) {
        hw->rpdac2_offset = atof(value);
    } else if (MATCH("hardware", "rpdac2_slope")) {
        hw->rpdac2_slope = atof(value);
    } else if (MATCH("hardware", "rpx_ip")) {
        sprintf(hw->rpx_ip, "%s", value);
    } else if (MATCH("hardware", "rpx_port")) {
        hw->rpx_port = atoi(value);
    } else if (MATCH("hardware", "rpy_ip")) {
        sprintf(hw->rpy_ip, "%s", value);
    } else if (MATCH("hardware", "rpy_port")) {
        hw->rpy_port = atoi(value);
    } else if (MATCH("hardware", "simple_link_ip")) {
        sprintf(hw->simple_link_ip, "%s", value); 
    } else if (MATCH("hardware", "simple_link_port")) {
        hw->simple_link_port = atoi(value);
    } else if (MATCH("hardware", "simple_link_parameter")) {
        hw->simple_link_parameter = atof(value);
    } else if (MATCH("hardware", "simple_link_channel")) {
        hw->simple_link_channel = atoi(value);
    } else if (modehandler(hw, 0, section, name, value) ||
               modehandler(hw, 1, section, name, value) ||
               modehandler(hw, 2, section, name, value) ||
               modehandler(hw, 3, section, name, value) ||
               modehandler(hw, 4, section, name, value) ||
               modehandler(hw, 5, section, name, value) ||
               modehandler(hw, 6, section, name, value) ||
               modehandler(hw, 7, section, name, value) ||
               modehandler(hw, 8, section, name, value) ||
               modehandler(hw, 9, section, name, value))
      return 0; /* unknown section/name, error */   

    return 1;
}


int ini_cleanup(HWData* hw)
{
    int i;

    //change the settings of conversion factors
    if (hw->hrdac1_range == LIBRP_RANGE_FULL) {
        hw->xcal = 20.0/hw->xrange;
        hw->xshift = -10.0;
        hw->xpos = hw->xreq = hw->xrange/2.0;
        hw->moveto = TRUE;

    } else {
        hw->xcal = 10.0/hw->xrange;
        hw->xshift = 0.0;
    }

    if (hw->hrdac2_range == LIBRP_RANGE_FULL) {
        hw->ycal = 20.0/hw->yrange;
        hw->yshift = -10.0;
        hw->ypos = hw->yreq = hw->yrange/2.0;
        hw->moveto = TRUE;

    } else {
        hw->ycal = 10.0/hw->yrange;
        hw->yshift = 0.0;
    }

    if (hw->hrdac3_range == LIBRP_RANGE_FULL) {
        hw->hrzcal = 20.0/hw->zrange;
        hw->hrzshift = -10.0;
        hw->zcal = 20.0/hw->zrange; //fast dac +-1 V, i.e. the default full range
        hw->zshift = -10.0;
     } else {
        hw->hrzcal = 10.0/hw->zrange;
        hw->hrzshift = 0.0;
        hw->zcal = 10.0/hw->zrange; //fast dac 0-1 V, to match the half range of hrdac
        hw->zshift = 0;
    }

    //debug information
    for (i=0; i<10; i++) {
       printf("mode %d: name %s mux %d %d bitshift %d\n", i, hw->modeset[i].name, hw->modeset[i].mux1, hw->modeset[i].mux2, hw->modeset[i].bitshift);
    }

    printf("ini cleanup completed\n");
    return 1;
}


void init_data(HWData *hw)
{
   int i;

   hw->buffer = NULL;
   hw->bufsize = 0;

   hw->debug = 0;
   hw->isFake = FALSE;

   hw->mode = LIBRP_MODE_OFF;
   hw->modereq = LIBRP_MODE_OFF;
   hw->xymode = LIBRP_XY_PI;
   hw->feedback = FALSE;
   hw->pllfeedback = FALSE;
   hw->ampfeedback = FALSE;
   hw->table_fd = -1;
   hw->xrange = 100e-6;
   hw->yrange = 100e-6;
   hw->zrange = 10e-6; 
   hw->hrdac1_range = LIBRP_RANGE_SMALL;
   hw->hrdac2_range = LIBRP_RANGE_SMALL;
   hw->hrdac3_range = LIBRP_RANGE_SMALL;
   hw->hrdac_regime = LIBRP_HRDAC_REGIME_FPGA;
   hw->dds1_range = LIBRP_RANGE_FULL;
   hw->dds2_range = LIBRP_RANGE_FULL;
   hw->hrdac1 = 0;
   hw->hrdac2 = 0;
   hw->hrdac3 = 0;
   hw->phaseshift1 = 0;
   hw->phaseshift2 = 0;
   hw->rp1_input_hv = 0;
   hw->rp2_input_hv = 0;
   hw->rp_bare_output = 0;
   hw->x_external = 0;
   hw->y_external = 0;
   hw->x_external_source = 0;
   hw->y_external_source = 1;
   hw->x_external_offset = 0;
   hw->y_external_offset = 0;
   hw->x_external_slope = 1;
   hw->y_external_slope = 1;

   hw->xslope = 0;
   hw->yslope = 0;
   hw->xsloperef = 0;
   hw->ysloperef = 0;
   hw->subtract_slope = 0;

   hw->actual_set = 0;

   hw->wakercount = 0;
   hw->spiercount = 0;
   hw->pwakercount = 0;
   hw->pspiercount = 0;

   hw->inrequest = 0;
   hw->triggerout = 0;
   hw->scan_thread_was_set = 0;

   hw->oversampling = 6;
   hw->undersampling_factor = 1;

   hw->simple_link_channel = 0;
   hw->simple_link_parameter = 1e8;

   for (i=0; i<10; i++)
   {
      hw->modeset[i].input1_range = 0;
      hw->modeset[i].input2_range = 0;
      hw->modeset[i].lockin1_filter_amplitude = 0;
      hw->modeset[i].lockin2_filter_amplitude = 0;
      hw->modeset[i].lockin1_filter_phase = 0;
      hw->modeset[i].lockin2_filter_phase = 0;
      hw->modeset[i].pidskip = 0;
      hw->modeset[i].pllskip = 0;
      hw->modeset[i].mux1 = 1;
      hw->modeset[i].mux2 = 1;
      hw->modeset[i].error_source = 0;
      hw->modeset[i].swap_in = 0;
      hw->modeset[i].swap_out = 0;
      hw->modeset[i].bitshift = 0;
      hw->modeset[i].lockin1_hr = 0;
      hw->modeset[i].lockin2_hr = 0;
      hw->modeset[i].lockin1_nwaves = 0;
      hw->modeset[i].lockin2_nwaves = 0;
      hw->modeset[i].pll = 0;
      hw->modeset[i].pll_input = 0;
      hw->modeset[i].out1 = 0;
      hw->modeset[i].out2 = 4;
      hw->modeset[i].outhr = 4;
      hw->modeset[i].lockin1_lfr = 0;
      hw->modeset[i].lockin2_lfr = 0;
    }
 
   hw->xpos = 0;
   hw->ypos = 0;
   hw->zpiezo = 0;
   hw->errorsignal = 0;
   hw->amplitude1 = 0;
   hw->phase1 = 0;
   hw->amplitude2 = 0;
   hw->phase2 = 0;
   hw->timestamp = 0;
   hw->adc1 = 0;
   hw->adc2 = 0;
   hw->l1x = 0;
   hw->l2x = 0;
   hw->l1y = 0;
   hw->l2y = 0;   

   hw->scan_line_ndata = 0;
   hw->scan_line_ndata_prev = -1;

   hw->pll_phase_limit_factor = 7;
   hw->pll_frequency_limit_factor = 7;

   hw->debugsource1 = 0;
   hw->debugsource2 = 0;

   hw->filter1 = 12;
   hw->isfilter1 = TRUE;
   hw->filter2 = 32;
   hw->isfilter2 = TRUE;
   hw->loopback1 = 0;
   hw->loopback2 = 0;

   hw->kpfm_mode = LIBRP_KPFM_OFF;
   hw->kpfm_dir = 0;
   hw->kpfm_source = 0;
   hw->kpfm_feedback = 0;

   hw->dart_mode = LIBRP_DART_OFF;
   hw->dart_frequency = 0;
   hw->dart_frequencyshift = 0;
   hw->dart_amplitude = 0;
   hw->dart_freqspan = 0;

   hw->therewasalift = 0;
   hw->lift_no_fb_action = FALSE;
   hw->kpfm_no_action = FALSE;

   hw->zreq = 0;
   hw->zpos = 0;
   hw->zspeed = 5e-6;
   hw->zmovingcounter = 0;
   hw->zmoving = FALSE;
   hw->zmoved = FALSE;
   hw->zmoveto = FALSE;
   hw->zstep = 0;

   hw->loadingscandata = FALSE;
   
   //init calibration values (just for case they would not be loaded)
   hw->rp_bare_input = 0;
   hw->rpadc1_bare_lv_offset = 0;
   hw->rpadc1_bare_lv_slope = 8191;
   hw->rpadc1_bare_hv_offset = 0;
   hw->rpadc1_bare_hv_slope = 410;

   hw->rpadc2_bare_lv_offset = 0;
   hw->rpadc2_bare_lv_slope = 8191;
   hw->rpadc2_bare_hv_offset = 0;
   hw->rpadc2_bare_hv_slope = 410;

   hw->rpadc1_divhigh_lv_offset = 0;
   hw->rpadc1_divhigh_lv_slope = 819;
   hw->rpadc1_divhigh_hv_offset = 0;
   hw->rpadc1_divhigh_hv_slope = 41;

   hw->rpadc1_divlow_lv_offset = 0;
   hw->rpadc1_divlow_lv_slope = 8191;
   hw->rpadc1_divlow_hv_offset = 0;
   hw->rpadc1_divlow_hv_slope = 410;

   hw->rpadc2_divhigh_lv_offset = 0;
   hw->rpadc2_divhigh_lv_slope = 819;
   hw->rpadc2_divhigh_hv_offset = 0;
   hw->rpadc2_divhigh_hv_slope = 41;

   hw->rpadc2_divlow_lv_offset = 0;
   hw->rpadc2_divlow_lv_slope = 8191;
   hw->rpadc2_divlow_hv_offset = 0;
   hw->rpadc2_divlow_hv_slope = 410;

   hw->rpdac1_bare_offset = 0;
   hw->rpdac1_bare_slope = 8191;
   hw->rpdac2_bare_offset = 0;
   hw->rpdac2_bare_slope = 8191;

   hw->rpdac1_offset = 0;
   hw->rpdac1_slope = 819.1;
   hw->rpdac2_offset = 0;
   hw->rpdac2_slope = 819.1;
  
   for (i=0; i<16; i++) hw->routing_rule[i] = LIBRP_RULE_NOTHING;

   //standby storage
   hw->c_sb_a1 = TRUE;
   hw->c_sb_p1 = TRUE;
   hw->c_sb_a2 = FALSE;
   hw->c_sb_p2 = FALSE;
   for (i=0; i<16; i++) hw->c_sb_in[i] = FALSE;
   hw->c_sb_set = FALSE; 
   hw->c_sb_fmd = FALSE;
   hw->c_sb_kpfm = FALSE;
   hw->c_sb_dart = FALSE;
   hw->c_sb_l1x = FALSE;
   hw->c_sb_l1y = FALSE;
   hw->c_sb_l2x = FALSE;
   hw->c_sb_l2y = FALSE;
   hw->c_sb_out9 = FALSE;
   hw->c_sb_slp = FALSE;

   //scan storage
   hw->c_sc_a1 = FALSE;
   hw->c_sc_p1 = FALSE;
   hw->c_sc_a2 = FALSE;
   hw->c_sc_p2 = FALSE;
   for (i=0; i<16; i++) hw->c_sc_in[i] = FALSE;
   hw->c_sc_set = FALSE;
   hw->c_sc_fmd = FALSE;
   hw->c_sc_kpfm = FALSE;
   hw->c_sc_dart = FALSE;
   hw->c_sc_l1x = FALSE;
   hw->c_sc_l1y = FALSE;
   hw->c_sc_l2x = FALSE;
   hw->c_sc_l2y = FALSE;
   hw->c_sc_out9 = FALSE;
   hw->c_sc_slp = FALSE;

   //ramp storage
   hw->c_rm_a1 = FALSE;
   hw->c_rm_p1 = FALSE;
   hw->c_rm_a2 = FALSE;
   hw->c_rm_p2 = FALSE;
   for (i=0; i<16; i++) hw->c_rm_in[i] = FALSE;
   hw->c_rm_set = FALSE;
   hw->c_rm_fmd = FALSE;
   hw->c_rm_kpfm = FALSE;
   hw->c_rm_dart = FALSE;
   hw->c_rm_l1x = FALSE;
   hw->c_rm_l1y = FALSE;
   hw->c_rm_l2x = FALSE;
   hw->c_rm_l2y = FALSE;
   hw->c_rm_out9 = FALSE;
   hw->c_rm_slp = FALSE;

   //stream storage
   hw->c_st_x = FALSE;
   hw->c_st_y = FALSE;
   hw->c_st_z = FALSE;
   hw->c_st_e = FALSE;
   hw->c_st_ts = FALSE;
   hw->c_st_a1 = FALSE;
   hw->c_st_p1 = FALSE;
   hw->c_st_a2 = FALSE;
   hw->c_st_p2 = FALSE;
   for (i=0; i<16; i++) hw->c_st_in[i] = FALSE;
   hw->c_st_set = FALSE;
   hw->c_st_fmd = FALSE;
   hw->c_st_kpfm = FALSE;
   hw->c_st_dart = FALSE;
   hw->c_st_l1x = FALSE;
   hw->c_st_l1y = FALSE;
   hw->c_st_l2x = FALSE;
   hw->c_st_l2y = FALSE;


   hw->scan_id = 0;
   hw->new_scan_id = 0;

   hw->scan_ndata = 0;
   hw->scan_completed_ndata = 0;
   hw->allocated_scan_ndata = 0;
   hw->scan_path_ndata = 0;
   hw->scan_path_xydata = NULL;

   hw->scanning_adaptive = FALSE;
   hw->scanning_line = FALSE;
   hw->scanning_script = FALSE;
   hw->stop_scanning_script = FALSE;

   hw->scan_x_data = NULL;
   hw->scan_y_data = NULL;
   hw->scan_z_data = NULL;
   hw->scan_e_data = NULL;
   hw->scan_a1_data = NULL;
   hw->scan_p1_data = NULL;
   hw->scan_a2_data = NULL;
   hw->scan_p2_data = NULL;
   hw->scan_ts_data = NULL;
   hw->scan_in_data = (double **)malloc(16*sizeof(double *));
   for (i=0; i<16; i++) hw->scan_in_data[i] = NULL;
   hw->scan_s_data = NULL;   
   hw->scan_fmd_data = NULL;   
   hw->scan_kpfm_data = NULL;   
   hw->scan_dart_data = NULL;   
   hw->scan_l1x_data = NULL;   
   hw->scan_l1y_data = NULL;   
   hw->scan_l2x_data = NULL;   
   hw->scan_l2y_data = NULL;
   hw->scan_out9_data = NULL;
   hw->scan_slp_data = NULL;   

   for (i=0; i<16; i++) hw->in[i] = 0;

   hw->lift_zdata = NULL;
 
   hw->start_ramp = 0;
   hw->stop_ramp = 0;
   hw->ramp_running = 0;
   hw->ramp_iterator = 0;
   hw->ramp_counter = 0;
   hw->ramp_npos = 0;
   hw->ramp_offset = 0;
   hw->ramp_plan_npos = 0;
   hw->ramp_plan_ntime = NULL;
   hw->ramp_plan_todo = NULL;
   hw->ramp_plan_value = NULL;

   hw->ramp_q_data = NULL;
   hw->ramp_x_data = NULL;
   hw->ramp_y_data = NULL;
   hw->ramp_z_data = NULL;
   hw->ramp_e_data = NULL;
   hw->ramp_a1_data = NULL;
   hw->ramp_p1_data = NULL;
   hw->ramp_a2_data = NULL;
   hw->ramp_p2_data = NULL;
   hw->ramp_ts_data = NULL;
   hw->ramp_in_data = (double **)malloc(16*sizeof(double *));
   for (i=0; i<16; i++) hw->ramp_in_data[i] = NULL;
   hw->ramp_s_data = NULL;
   hw->ramp_fmd_data = NULL;
   hw->ramp_kpfm_data = NULL;
   hw->ramp_dart_data = NULL;
   hw->ramp_l1x_data = NULL;
   hw->ramp_l1y_data = NULL;
   hw->ramp_l2x_data = NULL;
   hw->ramp_l2y_data = NULL;
   hw->ramp_out9_data = NULL;
   hw->ramp_slp_data = NULL;
   hw->ramp_start_z_feedback = TRUE;

   hw->streaming = FALSE;
   hw->stream_ndata = 0;
   hw->stream_lastsent = 0;
   hw->allocated_stream_ndata = 0;
   hw->stream_x_data = NULL;
   hw->stream_y_data = NULL;
   hw->stream_z_data = NULL;
   hw->stream_e_data = NULL;
   hw->stream_a1_data = NULL;
   hw->stream_p1_data = NULL;
   hw->stream_a2_data = NULL;
   hw->stream_p2_data = NULL;
   hw->stream_ts_data = NULL;
   hw->stream_in_data = (double **)malloc(16*sizeof(double *));
   for (i=0; i<16; i++) hw->stream_in_data[i] = NULL;
   hw->stream_s_data = NULL;
   hw->stream_fmd_data = NULL;
   hw->stream_kpfm_data = NULL;
   hw->stream_dart_data = NULL;
   hw->stream_l1x_data = NULL;
   hw->stream_l1y_data = NULL;
   hw->stream_l2x_data = NULL;
   hw->stream_l2y_data = NULL;
   
   hw->pid_p = 0;
   hw->pid_i = 0;
   hw->pid_d = 0;
   hw->pid_setpoint = 0;

   hw->pidpll_p = 0;
   hw->pidpll_i = 0;
   hw->pidpll_d = 0;
   hw->pidpll_setpoint = 0;

   hw->pidamplitude_p = 0.02; 
   hw->pidamplitude_i = 0.2;
   hw->pidamplitude_d = 0;
   hw->pidamplitude_setpoint = 0.1;

   hw->pidkpfm_p = 0;
   hw->pidkpfm_i = 0;
   hw->pidkpfm_d = 0;

   hw->piddart_p = 0;
   hw->piddart_i = 0;
   hw->piddart_d = 0;
 
   hw->xreq = 0;
   hw->yreq = 0;
   hw->moveto = FALSE;

   hw->setzpiezo = FALSE;
   hw->setzpiezovalue = 0;

   //analog outputs using the auxiliary DAC
   for (i=0; i<16; i++) hw->aout[i] = 0;
   hw->out9_offset = 0;
   hw->isaout = FALSE;

   //digital outputs using FPGA slow dac pins

   for (i=0; i<4; i++) hw->dout[i] = 0;
   hw->isdout = FALSE;
   
   hw->startlogging = FALSE;
   hw->stoplogging = FALSE;
   hw->logging = FALSE;

   hw->speed = 0.1e-6;
   hw->delay = 100;
   hw->moving = FALSE;
   hw->moved = FALSE;

   //frequency outputs to be set completely (new synthesis)
   hw->f1_frequency = 0;
   hw->f2_frequency = 0;
   hw->f3_frequency = 67158;
   hw->f1_amplitude = 0;
   hw->f2_amplitude = 0;
   hw->f1_offset = 0;
   hw->f2_offset = 0;

   //mutexes
   pthread_rwlock_init(&hw->cardmutex, NULL);
   pthread_rwlock_init(&hw->movemutex, NULL);
   pthread_rwlock_init(&hw->controlmutex, NULL);
   pthread_rwlock_init(&hw->rampmutex, NULL);
   pthread_rwlock_init(&hw->wakermutex, NULL);
   pthread_rwlock_init(&hw->spiermutex, NULL);
   pthread_rwlock_init(&hw->spimutex, NULL);

   snprintf(hw->version, sizeof(hw->version), "v4.1.19");

   //internal parameters, regime, etc.
   hw->xcal = 20.0/50e-6;
   hw->ycal = 20.0/50e-6;
   hw->zcal = 2.0/10e-6;//z piezo voltage to length conversion factor, now assuming 10 um z range for 1 V full output range
   hw->xshift = -10.0;
   hw->yshift = -10.0;
   hw->zshift = -1.0; //shift of z piezo voltage before multiplication by zcal
   hw->movingcounter = 0;
   hw->xstep = 0;
   hw->ystep = 0;
   hw->waker_running = 0;
   hw->spier_running = 0;

   hw->pause_scan = 0;
   hw->script_param_n = 0;

   hw->run_next = 0;

   hw->scantimestep = 3000000;   //5000000 for PI table;  //originally 10000000, this means 100 Hz, 1000000 is 1 kHz, 100000 is 10 kHz, but sometimes waker tries to run duplicitely
   hw->running = FALSE;


   Stabilizer* stbl = malloc(sizeof(Stabilizer));
   pthread_rwlock_init(&stbl->mutex, NULL);
   stbl->is_active = FALSE;
   stbl->mode = STBL_FINISHED;
   stbl->tuning_param = malloc(sizeof(char) * strlen(""));
   stbl->control_output = malloc(sizeof(char) * strlen(""));
   stbl->sum = 0;
   stbl->count = 0;
   stbl->oversampling = 10;
   stbl->adjusted = FALSE;
   hw->stabilizer = stbl;
}

void allocate_scan_data(HWData *hw, int ndata)
{
    int i;

    if (hw->debug) printf("allocating %d scan data\n", ndata);

    hw->allocated_scan_ndata = ndata;

    if (hw->scan_x_data == NULL) hw->scan_x_data = (double *)malloc(ndata*sizeof(double));
    else hw->scan_x_data = (double *)realloc(hw->scan_x_data, ndata*sizeof(double));

    if (hw->scan_y_data == NULL) hw->scan_y_data = (double *)malloc(ndata*sizeof(double));
    else hw->scan_y_data = (double *)realloc(hw->scan_y_data, ndata*sizeof(double));

    if (hw->scan_z_data == NULL) hw->scan_z_data = (double *)malloc(ndata*sizeof(double));
    else hw->scan_z_data = (double *)realloc(hw->scan_z_data, ndata*sizeof(double));

    if (hw->scan_e_data == NULL) hw->scan_e_data = (double *)malloc(ndata*sizeof(double));
    else hw->scan_e_data = (double *)realloc(hw->scan_e_data, ndata*sizeof(double));

    if (hw->scan_ts_data == NULL) hw->scan_ts_data = (double *)malloc(ndata*sizeof(double));
    else hw->scan_ts_data = (double *)realloc(hw->scan_ts_data, ndata*sizeof(double));

    if (hw->c_sc_a1) {
       if (hw->scan_a1_data == NULL) hw->scan_a1_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_a1_data = (double *)realloc(hw->scan_a1_data, ndata*sizeof(double));
    }
    if (hw->c_sc_p1) {
       if (hw->scan_p1_data == NULL) hw->scan_p1_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_p1_data = (double *)realloc(hw->scan_p1_data, ndata*sizeof(double));
    }
    if (hw->c_sc_a2) {
       if (hw->scan_a2_data == NULL) hw->scan_a2_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_a2_data = (double *)realloc(hw->scan_a2_data, ndata*sizeof(double));
    }
    if (hw->c_sc_p2) {
       if (hw->scan_p2_data == NULL) hw->scan_p2_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_p2_data = (double *)realloc(hw->scan_p2_data, ndata*sizeof(double));
    }
    for (i=0; i<16; i++)
    {
       if (hw->c_sc_in[i]) {
          if (hw->scan_in_data[i] == NULL) hw->scan_in_data[i] = (double *)malloc(ndata*sizeof(double));
          else hw->scan_in_data[i] = (double *)realloc(hw->scan_in_data[i], ndata*sizeof(double));
       }
    }
    if (hw->c_sc_set) {
       if (hw->scan_s_data == NULL) hw->scan_s_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_s_data = (double *)realloc(hw->scan_s_data, ndata*sizeof(double));
    }
    if (hw->c_sc_fmd) {
       if (hw->scan_fmd_data == NULL) hw->scan_fmd_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_fmd_data = (double *)realloc(hw->scan_fmd_data, ndata*sizeof(double));
    }
    if (hw->c_sc_kpfm) {
       if (hw->scan_kpfm_data == NULL) hw->scan_kpfm_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_kpfm_data = (double *)realloc(hw->scan_kpfm_data, ndata*sizeof(double));
    }
    if (hw->c_sc_dart) {
       if (hw->scan_dart_data == NULL) hw->scan_dart_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_dart_data = (double *)realloc(hw->scan_dart_data, ndata*sizeof(double));
    }
    if (hw->c_sc_l1x) {
       if (hw->scan_l1x_data == NULL) hw->scan_l1x_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_l1x_data = (double *)realloc(hw->scan_l1x_data, ndata*sizeof(double));
    }
    if (hw->c_sc_l1y) {
       if (hw->scan_l1y_data == NULL) hw->scan_l1y_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_l1y_data = (double *)realloc(hw->scan_l1y_data, ndata*sizeof(double));
    }
    if (hw->c_sc_l2x) {
       if (hw->scan_l2x_data == NULL) hw->scan_l2x_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_l2x_data = (double *)realloc(hw->scan_l2x_data, ndata*sizeof(double));
    }
    if (hw->c_sc_l2y) {
       if (hw->scan_l2y_data == NULL) hw->scan_l2y_data = (double *)malloc(ndata*sizeof(double));
       else hw->scan_l2y_data = (double *)realloc(hw->scan_l2y_data, ndata*sizeof(double));
    }
    if (hw->c_sc_out9) {
       if (hw->scan_out9_data == NULL) hw->scan_out9_data = (double *)malloc(ndata*sizeof(double)); 
       else hw->scan_out9_data = (double *)realloc(hw->scan_out9_data, ndata*sizeof(double));
    }
    if (hw->c_sc_slp) {
       if (hw->scan_slp_data == NULL) hw->scan_slp_data = (double *)malloc(ndata*sizeof(double)); 
       else hw->scan_slp_data = (double *)realloc(hw->scan_slp_data, ndata*sizeof(double));
    }
}

void allocate_ramp_data(HWData *hw, int ndata)
{   
    int i;
    
    if (hw->debug) printf("Allocate ramp data\n");
    
    if (hw->ramp_q_data == NULL) hw->ramp_q_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_q_data = (double *)realloc(hw->ramp_q_data, ndata*sizeof(double));
    
    if (hw->ramp_x_data == NULL) hw->ramp_x_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_x_data = (double *)realloc(hw->ramp_x_data, ndata*sizeof(double));
    
    if (hw->ramp_y_data == NULL) hw->ramp_y_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_y_data = (double *)realloc(hw->ramp_y_data, ndata*sizeof(double));
    
    if (hw->ramp_z_data == NULL) hw->ramp_z_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_z_data = (double *)realloc(hw->ramp_z_data, ndata*sizeof(double));
    
    if (hw->ramp_e_data == NULL) hw->ramp_e_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_e_data = (double *)realloc(hw->ramp_e_data, ndata*sizeof(double));
    
    if (hw->ramp_ts_data == NULL) hw->ramp_ts_data = (double *)malloc(ndata*sizeof(double));
    else hw->ramp_ts_data = (double *)realloc(hw->ramp_ts_data, ndata*sizeof(double));
    
    if (hw->c_rm_a1) {
       if (hw->ramp_a1_data == NULL) hw->ramp_a1_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_a1_data = (double *)realloc(hw->ramp_a1_data, ndata*sizeof(double));
    }
    if (hw->c_rm_p1) {
       if (hw->ramp_p1_data == NULL) hw->ramp_p1_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_p1_data = (double *)realloc(hw->ramp_p1_data, ndata*sizeof(double));
    }
    if (hw->c_rm_a2) {
       if (hw->ramp_a2_data == NULL) hw->ramp_a2_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_a2_data = (double *)realloc(hw->ramp_a2_data, ndata*sizeof(double));
    }
    if (hw->c_rm_p2) {
       if (hw->ramp_p2_data == NULL) hw->ramp_p2_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_p2_data = (double *)realloc(hw->ramp_p2_data, ndata*sizeof(double));
    }
    for (i=0; i<16; i++)
    {  
       if (hw->c_rm_in[i]) {
          if (hw->ramp_in_data[i] == NULL) hw->ramp_in_data[i] = (double *)malloc(ndata*sizeof(double));
          else hw->ramp_in_data[i] = (double *)realloc(hw->ramp_in_data[i], ndata*sizeof(double));
       }
    }
    if (hw->c_rm_set) {
       if (hw->ramp_s_data == NULL) hw->ramp_s_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_s_data = (double *)realloc(hw->ramp_s_data, ndata*sizeof(double));
    }
    if (hw->c_rm_fmd) {
       if (hw->ramp_fmd_data == NULL) hw->ramp_fmd_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_fmd_data = (double *)realloc(hw->ramp_fmd_data, ndata*sizeof(double));
    }
    if (hw->c_rm_kpfm) {
       if (hw->ramp_kpfm_data == NULL) hw->ramp_kpfm_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_kpfm_data = (double *)realloc(hw->ramp_kpfm_data, ndata*sizeof(double));
    }
    if (hw->c_rm_dart) {
       if (hw->ramp_dart_data == NULL) hw->ramp_dart_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_dart_data = (double *)realloc(hw->ramp_dart_data, ndata*sizeof(double));
    }
    if (hw->c_rm_l1x) {
       if (hw->ramp_l1x_data == NULL) hw->ramp_l1x_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_l1x_data = (double *)realloc(hw->ramp_l1x_data, ndata*sizeof(double));
    }
    if (hw->c_rm_l1y) {
       if (hw->ramp_l1y_data == NULL) hw->ramp_l1y_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_l1y_data = (double *)realloc(hw->ramp_l1y_data, ndata*sizeof(double));
    }
    if (hw->c_rm_l2x) {
       if (hw->ramp_l2x_data == NULL) hw->ramp_l2x_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_l2x_data = (double *)realloc(hw->ramp_l2x_data, ndata*sizeof(double));
    }
    if (hw->c_rm_l2y) {
       if (hw->ramp_l2y_data == NULL) hw->ramp_l2y_data = (double *)malloc(ndata*sizeof(double));
       else hw->ramp_l2y_data = (double *)realloc(hw->ramp_l2y_data, ndata*sizeof(double));
    }
    if (hw->c_rm_out9) {
       if (hw->ramp_out9_data == NULL) hw->ramp_out9_data = (double *)malloc(ndata*sizeof(double)); 
       else hw->ramp_out9_data = (double *)realloc(hw->ramp_out9_data, ndata*sizeof(double));
    }
    if (hw->c_rm_slp) {
       if (hw->ramp_slp_data == NULL) hw->ramp_slp_data = (double *)malloc(ndata*sizeof(double)); 
       else hw->ramp_slp_data = (double *)realloc(hw->ramp_slp_data, ndata*sizeof(double));
    }
}

void allocate_stream_data(HWData *hw, int ndata)
{
    int i;

    /*if (hw->debug)*/ printf("allocating %d stream data\n", ndata);

    hw->allocated_stream_ndata = ndata;

    if (hw->c_st_x) {
       if (hw->stream_x_data == NULL) hw->stream_x_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_x_data = (double *)realloc(hw->stream_x_data, ndata*sizeof(double));
    }

    if (hw->c_st_y) {
       if (hw->stream_y_data == NULL) hw->stream_y_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_y_data = (double *)realloc(hw->stream_y_data, ndata*sizeof(double));
    }

    if (hw->c_st_z) {
       if (hw->stream_z_data == NULL) hw->stream_z_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_z_data = (double *)realloc(hw->stream_z_data, ndata*sizeof(double));
    }

    if (hw->c_st_e) {
       if (hw->stream_e_data == NULL) hw->stream_e_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_e_data = (double *)realloc(hw->stream_e_data, ndata*sizeof(double));
    }

    if (hw->c_st_ts) {
       if (hw->stream_ts_data == NULL) hw->stream_ts_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_ts_data = (double *)realloc(hw->stream_ts_data, ndata*sizeof(double));
    }

    if (hw->c_st_a1) {
       if (hw->stream_a1_data == NULL) hw->stream_a1_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_a1_data = (double *)realloc(hw->stream_a1_data, ndata*sizeof(double));
    }
    if (hw->c_st_p1) {
       if (hw->stream_p1_data == NULL) hw->stream_p1_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_p1_data = (double *)realloc(hw->stream_p1_data, ndata*sizeof(double));
    }
    if (hw->c_st_a2) {
       if (hw->stream_a2_data == NULL) hw->stream_a2_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_a2_data = (double *)realloc(hw->stream_a2_data, ndata*sizeof(double));
    }
    if (hw->c_st_p2) {
       if (hw->stream_p2_data == NULL) hw->stream_p2_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_p2_data = (double *)realloc(hw->stream_p2_data, ndata*sizeof(double));
    }
    for (i=0; i<16; i++)
    {
       if (hw->c_st_in[i]) {
          if (hw->stream_in_data[i] == NULL) hw->stream_in_data[i] = (double *)malloc(ndata*sizeof(double));
          else hw->stream_in_data[i] = (double *)realloc(hw->stream_in_data[i], ndata*sizeof(double));
       }
    }
    if (hw->c_st_set) {
       if (hw->stream_s_data == NULL) hw->stream_s_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_s_data = (double *)realloc(hw->stream_s_data, ndata*sizeof(double));
    }
    if (hw->c_st_fmd) {
       if (hw->stream_fmd_data == NULL) hw->stream_fmd_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_fmd_data = (double *)realloc(hw->stream_fmd_data, ndata*sizeof(double));
    }
    if (hw->c_st_kpfm) {
       if (hw->stream_kpfm_data == NULL) hw->stream_kpfm_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_kpfm_data = (double *)realloc(hw->stream_kpfm_data, ndata*sizeof(double));
    }
    if (hw->c_st_dart) {
       if (hw->stream_dart_data == NULL) hw->stream_dart_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_dart_data = (double *)realloc(hw->stream_dart_data, ndata*sizeof(double));
    }
    if (hw->c_st_l1x) {
       if (hw->stream_l1x_data == NULL) hw->stream_l1x_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_l1x_data = (double *)realloc(hw->stream_l1x_data, ndata*sizeof(double));
    }
    if (hw->c_st_l1y) {
       if (hw->stream_l1y_data == NULL) hw->stream_l1y_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_l1y_data = (double *)realloc(hw->stream_l1y_data, ndata*sizeof(double));
    }
    if (hw->c_st_l2x) {
       if (hw->stream_l2x_data == NULL) hw->stream_l2x_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_l2x_data = (double *)realloc(hw->stream_l2x_data, ndata*sizeof(double));
    }
    if (hw->c_st_l2y) {
       if (hw->stream_l2y_data == NULL) hw->stream_l2y_data = (double *)malloc(ndata*sizeof(double));
       else hw->stream_l2y_data = (double *)realloc(hw->stream_l2y_data, ndata*sizeof(double));
    }
}


//stores actual state as a new data point to the scan array and increments the counter
int store_actual_point_data(HWData *hw, int set)
{
   int i;

   if (hw->scan_ndata >= hw->allocated_scan_ndata) return hw->scan_ndata-1; //we have no more space to store anything

   pthread_rwlock_rdlock(&hw->movemutex);
   if (hw->x_external) hw->scan_x_data[hw->scan_ndata] = (hw->in[hw->x_external_source] + hw->x_external_offset)*hw->x_external_slope; 
   else hw->scan_x_data[hw->scan_ndata] = hw->xpos;

   if (hw->y_external) hw->scan_y_data[hw->scan_ndata] = (hw->in[hw->y_external_source] + hw->y_external_offset)*hw->y_external_slope; 
   else hw->scan_y_data[hw->scan_ndata] = hw->ypos;
   pthread_rwlock_unlock(&hw->movemutex);

   pthread_rwlock_rdlock(&hw->cardmutex);
   if (hw->subtract_slope) hw->scan_z_data[hw->scan_ndata] = hw->zpiezo - (librp_GetPidOffset(&hw->rp) - hw->zshift)/hw->zcal;
   else hw->scan_z_data[hw->scan_ndata] = hw->zpiezo;
   hw->scan_e_data[hw->scan_ndata] = hw->errorsignal;
   hw->scan_ts_data[hw->scan_ndata] = hw->timestamp;
   if (hw->c_sc_a1) hw->scan_a1_data[hw->scan_ndata] = hw->amplitude1;
   if (hw->c_sc_p1) hw->scan_p1_data[hw->scan_ndata] = hw->phase1;
   if (hw->c_sc_a2) hw->scan_a2_data[hw->scan_ndata] = hw->amplitude2;
   if (hw->c_sc_p2) hw->scan_p2_data[hw->scan_ndata] = hw->phase2;
   if (hw->c_sc_kpfm) hw->scan_kpfm_data[hw->scan_ndata] = hw->f2_offset;
   if (hw->c_sc_dart) hw->scan_dart_data[hw->scan_ndata] = hw->dart_frequency + hw->dart_frequencyshift;
   if (hw->c_sc_l1x) hw->scan_l1x_data[hw->scan_ndata] = hw->l1x;
   if (hw->c_sc_l1y) hw->scan_l1y_data[hw->scan_ndata] = hw->l1y;
   if (hw->c_sc_l2x) hw->scan_l2x_data[hw->scan_ndata] = hw->l2x;
   if (hw->c_sc_l2y) hw->scan_l2y_data[hw->scan_ndata] = hw->l2y;
   if (hw->c_sc_out9) hw->scan_out9_data[hw->scan_ndata] = hw->aout[8];
   if (hw->c_sc_slp) hw->scan_slp_data[hw->scan_ndata] = (librp_GetPidOffset(&hw->rp) - hw->zshift)/hw->zcal;


   for (i=0; i<16; i++) if (hw->c_sc_in[i]) hw->scan_in_data[i][hw->scan_ndata] = hw->in[i];

   if (hw->c_sc_set) hw->scan_s_data[hw->scan_ndata] = set;
   if (hw->c_sc_fmd) hw->scan_fmd_data[hw->scan_ndata] = hw->amresult;
   pthread_rwlock_unlock(&hw->cardmutex);

//   printf("stored %d: %g %g %g\n", hw->scan_ndata, hw->scan_x_data[hw->scan_ndata], hw->scan_y_data[hw->scan_ndata], hw->scan_z_data[hw->scan_ndata]);

   hw->scan_ndata++;

   return hw->scan_ndata-1;
}

//stores actual state as a new ramp point to the ramp array and increments the counter
int store_actual_ramp_data(HWData *hw, int set)
{
   int i;
   hw->ramp_q_data[hw->ramp_ndata] = hw->ramp_plan_value[hw->ramp_iterator];

   pthread_rwlock_rdlock(&hw->movemutex);
   hw->ramp_x_data[hw->ramp_ndata] = hw->xpos;
   hw->ramp_y_data[hw->ramp_ndata] = hw->ypos;
   pthread_rwlock_unlock(&hw->movemutex);

   pthread_rwlock_rdlock(&hw->cardmutex);
   hw->ramp_z_data[hw->ramp_ndata] = hw->zpiezo;
   hw->ramp_e_data[hw->ramp_ndata] = hw->errorsignal;
   hw->ramp_ts_data[hw->ramp_ndata] = hw->timestamp;
   if (hw->c_rm_a1) hw->ramp_a1_data[hw->ramp_ndata] = hw->amplitude1;
   if (hw->c_rm_p1) hw->ramp_p1_data[hw->ramp_ndata] = hw->phase1;
   if (hw->c_rm_a2) hw->ramp_a2_data[hw->ramp_ndata] = hw->amplitude2;
   if (hw->c_rm_p2) hw->ramp_p2_data[hw->ramp_ndata] = hw->phase2;
   if (hw->c_rm_out9) hw->ramp_out9_data[hw->ramp_ndata] = hw->aout[8];

   for (i=0; i<16; i++) if (hw->c_rm_in[i]) hw->ramp_in_data[i][hw->ramp_ndata] = hw->in[i];

   if (hw->c_rm_set) hw->ramp_s_data[hw->ramp_ndata] = set;
   if (hw->c_rm_fmd) hw->ramp_fmd_data[hw->ramp_ndata] = hw->amresult;
   if (hw->c_rm_kpfm) hw->ramp_kpfm_data[hw->ramp_ndata] = hw->f2_offset;
   if (hw->c_rm_dart) hw->ramp_dart_data[hw->ramp_ndata] = hw->dart_frequency + hw->dart_frequencyshift;
   if (hw->c_rm_l1x) hw->ramp_l1x_data[hw->ramp_ndata] = hw->l1x;
   if (hw->c_rm_l1y) hw->ramp_l1y_data[hw->ramp_ndata] = hw->l1y;
   if (hw->c_rm_l2x) hw->ramp_l2x_data[hw->ramp_ndata] = hw->l2x;
   if (hw->c_rm_l2y) hw->ramp_l2y_data[hw->ramp_ndata] = hw->l2y;
   if (hw->c_rm_slp) hw->ramp_slp_data[hw->ramp_ndata] = (librp_GetPidOffset(&hw->rp) - hw->zshift)/hw->zcal;
  
      
   pthread_rwlock_unlock(&hw->cardmutex);

   hw->ramp_ndata++;

   return hw->ramp_ndata-1;
}

//stores actual state as a new data point to the stream array
int store_actual_stream_data(HWData *hw, int set)
{
   int i;

   if (hw->stream_ndata >= (hw->allocated_stream_ndata-1)) //wrap around
       hw->stream_ndata = 0;

   //pthread_rwlock_rdlock(&hw->movemutex);
   if (hw->c_st_x) hw->stream_x_data[hw->stream_ndata] = hw->xpos;
   if (hw->c_st_y) hw->stream_y_data[hw->stream_ndata] = hw->ypos;
   //pthread_rwlock_unlock(&hw->movemutex);

   //pthread_rwlock_rdlock(&hw->cardmutex);
   if (hw->c_st_z) hw->stream_z_data[hw->stream_ndata] = hw->zpiezo;
   if (hw->c_st_e) hw->stream_e_data[hw->stream_ndata] = hw->errorsignal;
   if (hw->c_st_ts) hw->stream_ts_data[hw->stream_ndata] = hw->timestamp;
   if (hw->c_st_a1) hw->stream_a1_data[hw->stream_ndata] = hw->amplitude1;
   if (hw->c_st_p1) hw->stream_p1_data[hw->stream_ndata] = hw->phase1;
   if (hw->c_st_a2) hw->stream_a2_data[hw->stream_ndata] = hw->amplitude2;
   if (hw->c_st_p2) hw->stream_p2_data[hw->stream_ndata] = hw->phase2;
   if (hw->c_st_kpfm) hw->stream_kpfm_data[hw->stream_ndata] = hw->f2_offset;
   if (hw->c_st_dart) hw->stream_dart_data[hw->stream_ndata] = hw->dart_frequency + hw->dart_frequencyshift;
   if (hw->c_st_l1x) hw->stream_l1x_data[hw->stream_ndata] = hw->l1x;
   if (hw->c_st_l1y) hw->stream_l1y_data[hw->stream_ndata] = hw->l1y;
   if (hw->c_st_l2x) hw->stream_l2x_data[hw->stream_ndata] = hw->l2x;
   if (hw->c_st_l2y) hw->stream_l2y_data[hw->stream_ndata] = hw->l2y;

   for (i=0; i<16; i++) if (hw->c_st_in[i]) hw->stream_in_data[i][hw->stream_ndata] = hw->in[i];

   if (hw->c_st_set) hw->stream_s_data[hw->stream_ndata] = set;
   if (hw->c_st_fmd) hw->stream_fmd_data[hw->stream_ndata] = hw->amresult;
   //pthread_rwlock_unlock(&hw->cardmutex);

   hw->stream_ndata++;

   return hw->stream_ndata-1;
}

