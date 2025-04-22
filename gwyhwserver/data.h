/*
 *  hwserver: a simple implentation of Gwyfile compatible server for RP AFM operation
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

#ifndef HWDATA
#define HWDATA

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <time.h>
#include <stddef.h>
#include <stdio.h>
#include <pthread.h>

#ifdef FAKE
#include "myrpfake.h"
#else
#include "cmirp.h"
#endif

#define TRUE 1 
#define FALSE 0

typedef enum
{
    STBL_COLLECT,
    STBL_STABILIZE,
    STBL_FINISHED
} StabilizerMode;

typedef struct {
    pthread_rwlock_t mutex;

    bool is_active;
    StabilizerMode mode;

    double setpoint;
    double step;
    double control_range;
    int oversampling;

    char* tuning_param;
    char* control_output;

    double sum;
    int count;
    double mean;
    // whether adjustment of control_output was needed in one iteration
    bool adjusted;
} Stabilizer;

typedef enum
{
    LIBRP_XY_OFF      = 0,
    LIBRP_XY_VOLTAGE  = 1,
    LIBRP_XY_PI       = 2,
    LIBRP_XY_RPTABLE  = 3
} librpXYMode;

typedef struct {
    char name[50];
    int mux1;  //multiplexer to RP in1
    int mux2;  //multiplexer to RP in2

    int error_source; //error source type, 0-proportional, etc.
    bool swap_in;       //swap error source direction
    bool swap_out;      //swap zpiezo signal direction
    int bitshift;     //error signal bitshift to equalize it

    int pll;          //use pll to adjust frequency
    int pll_input;    //1/2 input phase to be used for pll

    int input1_range; //RP1 input range, 0/1
    int input2_range; //RP2 input range, 0/1
    int lockin1_hr;   //high resolution lockin option for lockin1
    int lockin1_nwaves; //number of waves to evaluate for lockin1
    int lockin2_hr;   //high resolution lockin option for lockin2
    int lockin2_nwaves; //number of waves to evaluate for lockin2

    int lockin1_filter_amplitude; //lockin1 filter for amplitude
    int lockin1_filter_phase;     //lockin1 filter for phase
    int lockin2_filter_amplitude; //lockin2 filter for amplitude
    int lockin2_filter_phase;     //lockin2 filter for phase

    int pidskip;             //pid loop slowdown factor
    int pllskip;             //pll pid loop slowdown factor

    int out1;                //RP1 output routing
    int out2;                //RP2 output routing (if hr is enabled))
    int outhr;               //RP fast 20bit dac output (if hr is enabled)

    int lockin1_lfr;                //low frequency generator option
    int lockin2_lfr;                //low frequency generator option

} librpModeSettings;

typedef struct {

   /*connection socket*/
   int *socket;
   int server_socket;

   int debug;
   bool isFake;

   /*version number*/
   char version[100];

   char *buffer;
   size_t bufsize;

   int reset_spi;

   librpModeSettings modeset[10];

   /*scan mode*/
   int mode;                 //measurement mode
   int modereq;              //requested measurement mode
   int xymode;               //xy scan mode

   //int zpiezo_hrdac;         //use 20bit z piezo    
   //int hrdac_range;          //range of 20bit dacs
   //int zpiezo_hrdac_range;   //range of 20bit dac if used as zpiezo
   int hrdac_regime;         //use HRDAC from FPGA or CPU
   int hrdac1_range;         //range of HRDAC, check jumpers
   int hrdac2_range;
   int hrdac3_range;
   int dds1_range;           //range of DDS generator 1
   int dds2_range;           //range of DDS generator 2
   int rp1_input_hv;         //RP ADC1 jumper settings
   int rp2_input_hv;         //RP ADC2 jumper settings
   int rp_bare_output;       //RP output is used without the x10 board
   int rp_bare_input;        //RP is used without the divider board
   int x_external;           //external x voltage sensor is used
   int y_external;
   int x_external_source;    //external x voltage sensor channel
   int y_external_source;
   double x_external_offset;  //voltage shift for x voltage sensor, in Volts
   double y_external_offset;
   double x_external_slope;   //slope to convert from Volts to meters
   double y_external_slope;   

   int oversampling;          //ADC oversampling 
   int undersampling_factor;  //fast scan undersampling factor

   int table_fd;              //RS232 scanning table file descriptor
   int rpx_socket;            //RP scan: socket, port, ip address
   int rpy_socket;
   int rpx_port;
   int rpy_port;
   char rpx_ip[50];
   char rpy_ip[50];

   timer_t waker_id;          
   timer_t logger_id;
   timer_t spier_id;

   double xrange;             //scanner ranges
   double yrange;
   double zrange;

   double hrdac1;            //data for 20bit DACs
   double hrdac2;
   double hrdac3;

   /*feedback*/
   bool feedback;            //main feedback on/off
   bool pllfeedback;         //pll feedback on/off   //was fmfeedback1
   bool ampfeedback;   //pll amplitude feedback on/off, was fmfeedback2

   int pll_phase_limit_factor;     //pll phase limit
   int pll_frequency_limit_factor; //pll frequency limit
   int phaseshift1;          //lockin1 phase shift
   int phaseshift2;          //lockin2 phase shift

   int debugsource1;
   int debugsource2;

   double xslope;    //sample slope for pid offset at tilted samples
   double yslope;  
   double xsloperef; //position reference for calculating the slope
   double ysloperef;
   bool subtract_slope;

   int triggerout;
   pthread_t script_thread;

   bool scan_thread_was_set;

   /*simple link*/
   int simple_link_channel;
   char simple_link_ip[50];
   int simple_link_port;
   int simple_link_socket;
   double simple_link_parameter;

   /*calibration*/

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

   /*kpfm*/
   int kpfm_mode;
   int kpfm_source;
   double aamplitude2;
   int kpfm_dir;
   bool kpfm_feedback;

   /*dart*/
   int dart_mode;
   double dart_frequency;
   double dart_frequencyshift;
   double dart_amplitude;
   double dart_freqspan;

   int therewasalift;

   /*actual readout and computed values*/
   double xpos;
   double ypos;
   double zpiezo;
   double errorsignal;
   double amplitude1;
   double phase1;
   double amplitude2;
   double phase2;
   double in[16];
   double timestamp;
   double adc1;
   double adc2;
   double fmresult;
   double amresult;
   double l1x;
   double l1y;
   double l2x;
   double l2y;

   //debug data
   int wakercount;
   int spiercount;
   int pwakercount;
   int pspiercount;


   // when client requests new scan, save id of the scan to the new_scan_id, and when data are
   // cleared for the new scan, save id to the scan_id
   int scan_id;
   int new_scan_id;

   //scan data
   int scan_ndata; //already scanned points
   int scan_completed_ndata; //already scanned and processed points, for Lua line scans
   int allocated_scan_ndata; //what was allocated, depends on mode
   double *scan_x_data;
   double *scan_y_data;
   double *scan_z_data;
   double *scan_e_data;
   double *scan_a1_data;
   double *scan_p1_data;
   double *scan_a2_data;
   double *scan_p2_data;
   double *scan_ts_data;
   double **scan_in_data;
   double *scan_s_data;
   double *scan_fmd_data;
   double *scan_kpfm_data;
   double *scan_dart_data;
   double *scan_l1x_data;
   double *scan_l1y_data;
   double *scan_l2x_data;
   double *scan_l2y_data;
   double *scan_out9_data;
   double *scan_slp_data;

   double *lift_zdata;
   bool lift_usezdata;
   bool lift_no_fb_action; //do not start/stop automatically feedback when going to lift
   bool kpfm_no_action;    //do not start/stop automatically kpfm
  
   bool loadingscandata;

   double *scan_path_xydata;
   int scan_path_ndata;
   bool scanning_adaptive;
   bool scanning_script;
   bool stop_scanning_script;

   int scan_line_ndata;
   int scan_line_ndata_prev;
   double scan_line_xfrom;
   double scan_line_yfrom;
   double scan_line_xto;
   double scan_line_yto;
   bool scanning_line;

   //lua data
   int scan_script_ndata;
   char script[15000];
   bool pause_scan;
   char script_param_key[50][50];
   double script_param_value[50];
   int script_param_n;

   //ramp data
   int ramp_ndata; //already scanned points
   double *ramp_q_data;
   double *ramp_x_data;
   double *ramp_y_data;
   double *ramp_z_data;
   double *ramp_e_data;
   double *ramp_a1_data;
   double *ramp_p1_data;
   double *ramp_a2_data;
   double *ramp_p2_data;
   double *ramp_ts_data;
   double **ramp_in_data;
   double *ramp_s_data;
   double *ramp_fmd_data;
   double *ramp_kpfm_data;
   double *ramp_dart_data;
   double *ramp_l1x_data;
   double *ramp_l1y_data;
   double *ramp_l2x_data;
   double *ramp_l2y_data;
   double *ramp_out9_data;
   double *ramp_slp_data;
   int ramp_npos;
   double ramp_from;
   double ramp_to;
   double ramp_start_delay;
   double ramp_peak_delay;
   double ramp_time_up;
   double ramp_time_down;
   double ramp_offset;
   int ramp_quantity;
   int ramp_peak_nrules;
   int ramp_peak_rule_channel[10];
   int ramp_peak_rule_logic[10];
   double ramp_peak_rule_value[10];
   int start_ramp;
   int stop_ramp;
   int ramp_running;
   int ramp_iterator;
   int ramp_counter;
   int *ramp_plan_ntime;
   int *ramp_plan_todo;
   double *ramp_plan_value;
   int ramp_plan_npos;


   // if Z ramp is started, save state of the feedback and
   // after it stops, set feedback back to that value
   bool ramp_start_z_feedback;

   //stream data
   int stream_ndata; //already scanned points
   int allocated_stream_ndata;
   double *stream_x_data;
   double *stream_y_data;
   double *stream_z_data;
   double *stream_e_data;
   double *stream_a1_data;
   double *stream_p1_data;
   double *stream_a2_data;
   double *stream_p2_data;
   double *stream_ts_data;
   double **stream_in_data;
   double *stream_s_data;
   double *stream_fmd_data;
   double *stream_kpfm_data;
   double *stream_dart_data;
   double *stream_l1x_data;
   double *stream_l1y_data;
   double *stream_l2x_data;
   double *stream_l2y_data;
   bool streaming;
   int stream_lastsent;
 
   //pid settings for FPGA
   double pid_p;
   double pid_i;
   double pid_d;
   double pid_setpoint;

   double pidpll_p;  //was pid2, probably
   double pidpll_i;
   double pidpll_d;
   double pidpll_setpoint;

   double pidamplitude_p;  //for akiyama amplitude, was pid3
   double pidamplitude_i;
   double pidamplitude_d;
   double pidamplitude_setpoint;

   double pidkpfm_p;  //pid for KPFM
   double pidkpfm_i;
   double pidkpfm_d;

   double piddart_p;  //pid for DART
   double piddart_i;
   double piddart_d;

   //xy scanner move command target values and command status
   double xreq;
   double yreq;
   bool moveto;

   //z move command workaround (move at some speed)
   double zreq;
   double zpos;
   double zspeed;
   int zmovingcounter;
   bool zmoving;
   bool zmoved;
   bool zmoveto;
   double zstep;
 
   //actual set to store data to when scanning
   int actual_set; 

   //z piezo command when feedback is off
   bool setzpiezo;
   double setzpiezovalue;

   //what should be collected in standby mode
   bool c_sb_a1;
   bool c_sb_p1;
   bool c_sb_a2;
   bool c_sb_p2;
   bool c_sb_in[16];
   bool c_sb_set;
   bool c_sb_fmd;
   bool c_sb_kpfm;
   bool c_sb_dart;
   bool c_sb_l1x;
   bool c_sb_l1y;
   bool c_sb_l2x;
   bool c_sb_l2y;
   bool c_sb_out9;
   bool c_sb_slp;

   //what should be collected in scan mode
   bool c_sc_a1;
   bool c_sc_p1;
   bool c_sc_a2;
   bool c_sc_p2;
   bool c_sc_in[16];
   bool c_sc_set;
   bool c_sc_fmd;
   bool c_sc_kpfm;
   bool c_sc_dart;
   bool c_sc_l1x;
   bool c_sc_l1y;
   bool c_sc_l2x;
   bool c_sc_l2y;
   bool c_sc_out9;
   bool c_sc_slp;


   //what should be collected in ramp mode
   bool c_rm_a1;
   bool c_rm_p1;
   bool c_rm_a2;
   bool c_rm_p2;
   bool c_rm_in[16];
   bool c_rm_set;
   bool c_rm_fmd;
   bool c_rm_kpfm;
   bool c_rm_dart;
   bool c_rm_l1x;
   bool c_rm_l1y;
   bool c_rm_l2x;
   bool c_rm_l2y;
   bool c_rm_out9;
   bool c_rm_slp;

  //what should be collected in stream mode
   bool c_st_x;
   bool c_st_y;
   bool c_st_z;
   bool c_st_e;
   bool c_st_ts;
   bool c_st_a1;
   bool c_st_p1;
   bool c_st_a2;
   bool c_st_p2;
   bool c_st_in[16];
   bool c_st_set;
   bool c_st_fmd;
   bool c_st_kpfm;
   bool c_st_dart;
   bool c_st_l1x;
   bool c_st_l1y;
   bool c_st_l2x;
   bool c_st_l2y;
   bool c_st_out9;
   bool c_st_slp;


   //filters
   int filter1;
   bool isfilter1;
   int filter2;
   bool isfilter2;
   int loopback1;
   int loopback2;

   //analog outputs using the auxiliary DAC
   double aout[16];  //output values for all the channels
   double out9_offset;
   int isaout;   //request for setting the auxiliary DAC

   //routing
   int routing_rule[16];
   double routing_value[16];

   //digital outputs using FPGA slow dac pins
   bool dout[4];
   bool isdout;
   
   //local logging
   bool startlogging;
   bool stoplogging;
   FILE *fw;
   int logging;

   //move parameters and status variables
   double speed;
   double delay;
   bool moving;
   bool moved;
   bool run_next;

   //frequency outputs to be set completely (new synthesis)
   double f1_frequency;
   double f2_frequency;
   double f3_frequency;
   double f1_amplitude;
   double f2_amplitude;
   double f1_offset;
   double f2_offset;

   /*rp library settings*/
   librpSet rp;

   struct timespec start;

   //mutexes
   pthread_rwlock_t cardmutex;
   pthread_rwlock_t movemutex;
   pthread_rwlock_t controlmutex;
   pthread_rwlock_t rampmutex;
   pthread_rwlock_t wakermutex;
   pthread_rwlock_t spiermutex;
   pthread_rwlock_t spimutex;

   //internal parameters, regime, etc.
   double xcal;
   double ycal;
   double zcal;
   double hrzcal;
   double xshift;
   double yshift;
   double zshift;
   double hrzshift;
   int movingcounter;
   double xstep;
   double ystep;

   long long scantimestep;
   bool running;
   atomic_bool waker_running;
   atomic_bool spier_running;

   int inrequest;

   pthread_t cth;

   Stabilizer* stabilizer;

} HWData;

void init_data(HWData *hw);

void  allocate_scan_data(HWData *hw, int ndata);
void  allocate_ramp_data(HWData *hw, int ndata);
void  allocate_stream_data(HWData *hw, int ndata);

int   store_actual_point_data(HWData *hw, int set);
int   store_actual_ramp_data(HWData *hw, int set);
int   store_actual_stream_data(HWData *hw, int set);

void  run_line_and_store(HWData *hw, 
                         double xto, double yto, int res, int set);

int   ini_cleanup(HWData* hw);
int   inihandler(void* user, const char* section, const char* name,
                   const char* value);

bool is_channel_valid(const char *chan);
double read_channel(HWData* hw, const char* chan); 
void write_out(HWData* hw, const char* chan, double value);
double read_out(HWData* hw, const char* chan);

#endif //HWDATA
