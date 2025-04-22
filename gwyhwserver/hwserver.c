
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
#include <sys/file.h>
#include <sys/timerfd.h>

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
#include "communication.h"

#include "cmirp.h"

#include "data.h"
#include "rptable.h"
#include "simplelink.h"
#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"

#include "luafunctions.h"

#include "ini.h"

#include "stabilizer.h"
#include "fake.h"

#define CLAMP(x, low, high) ({\
  __typeof__(x) __x = (x); \
  __typeof__(low) __low = (low);\
  __typeof__(high) __high = (high);\
  __x > __high ? __high : (__x < __low ? __low : __x);\
  })

#define MAX(a,b) (((a)>(b))?(a):(b))

#define HWDEBUG 0

#define OUT_OF_RANGE 12345

typedef enum
{
    RAMP_SET    = 0,
    RAMP_STORE  = 1
} RampTodo;


void *connection_handler(void *);

void set_mode_and_feedback(HWData *hw);


int global_stop = 0; //to stop after catching signal
void stop(HWData *hw);

int waker_stop = 0;
pthread_t waker_tid;
void* waker_thread(void *ptr);


void spier(union sigval val)
{
    HWData *hw = (HWData *)val.sival_ptr;

//static void spier(int sig, siginfo_t *si, void *uc)
//{
   // HWData *hw = (HWData *)si->_sifields._rt.si_sigval.sival_ptr;
    double lrdac[16], adc[16];
    double hrdac1, hrdac2, hrdac3, xpos, ypos, imagpart;
    int i, mode, running, allrunning;

    //printf("r\n");

    //pthread_rwlock_rdlock(&hw->spiermutex);
    running = hw->spier_running;
    //pthread_rwlock_unlock(&hw->spiermutex);

    if (running) {   //this should not happen - it means that last spier call was not completed yet
       //if (hw->debug)
       //printf("Error, spier is already running\n");
       return;
    }
    //printf("s\n");
    //pthread_rwlock_wrlock(&hw->spiermutex);
    hw->spier_running = 1;
    //pthread_rwlock_unlock(&hw->spiermutex);

    hw->spiercount++;

    //pthread_rwlock_rdlock(&hw->controlmutex);
    mode = hw->mode;
    allrunning = hw->running;
    //pthread_rwlock_unlock(&hw->controlmutex);

    if (!allrunning) return;

    if (hw->reset_spi) {
        librp_Init_SPI_devices(&hw->rp);
        librp_SetInputRange(&hw->rp, (&hw->rp)->input_range1, (&hw->rp)->input_range2);
        librp_SetState(&hw->rp, (&hw->rp)->state, (&hw->rp)->feedback, (&hw->rp)->rpdacrule1, (&hw->rp)->rpdacrule2, 
                       (&hw->rp)->rpdacrulehr, (&hw->rp)->swap_in, (&hw->rp)->swap_out, (&hw->rp)->pidskip);

        hw->reset_spi = 0;
    }

    for (i=0; i<16; i++) {
       if (hw->routing_rule[i] == LIBRP_RULE_NOTHING) {
           lrdac[i] = hw->aout[i];
           // NENO
           // our hardware introduces some offset (e.g. user will set 1V, but the
           // real value will be 0.8V), so we correct it with out9_offset
           if (i == 8) {
             lrdac[i] += hw->out9_offset;
           }
           /////
       }
       else if (hw->routing_rule[i] >= LIBRP_RULE_IN1 && hw->routing_rule[i] <= LIBRP_RULE_IN16) 
       {
           lrdac[i] = adc[hw->routing_rule[i] - LIBRP_RULE_IN1];
       }
       else if (hw->routing_rule[i] == LIBRP_RULE_X || hw->routing_rule[i] == LIBRP_RULE_Y) 
       {
           //pthread_rwlock_rdlock(&hw->movemutex);
           xpos = hw->xpos;
           ypos = hw->ypos;
           //pthread_rwlock_unlock(&hw->movemutex);

           if (hw->routing_rule[i] == LIBRP_RULE_X) lrdac[i] = xpos*10000.0;
           else if (hw->routing_rule[i] == LIBRP_RULE_Y) lrdac[i] = ypos*10000.0;
       }
       else {
           //pthread_rwlock_rdlock(&hw->cardmutex);
           if (hw->routing_rule[i] == LIBRP_RULE_ERROR) lrdac[i] = hw->errorsignal;
           else if (hw->routing_rule[i] == LIBRP_RULE_ZPIEZO) lrdac[i] = hw->zpiezo*10000.0;  //from +-100 um to +-1 V
           else if (hw->routing_rule[i] == LIBRP_RULE_AMPLITUDE1) lrdac[i] = hw->amplitude1;
           else if (hw->routing_rule[i] == LIBRP_RULE_PHASE1) lrdac[i] = hw->phase1;
           else if (hw->routing_rule[i] == LIBRP_RULE_AMPLITUDE2) lrdac[i] = hw->amplitude2;
           else if (hw->routing_rule[i] == LIBRP_RULE_PHASE2) lrdac[i] = hw->phase2;
           else if (hw->routing_rule[i] == LIBRP_RULE_FMRESULT) lrdac[i] = hw->fmresult/100.0;
           else if (hw->routing_rule[i] == LIBRP_RULE_AMRESULT) lrdac[i] = hw->amresult;
           else if (hw->routing_rule[i] == LIBRP_RULE_ADC1) lrdac[i] = hw->adc1;
           else if (hw->routing_rule[i] == LIBRP_RULE_ADC2) lrdac[i] = hw->adc2;
           else if (hw->routing_rule[i] == LIBRP_RULE_Z) lrdac[i] = hw->zpiezo;
           else lrdac[i] = 0;
           //pthread_rwlock_unlock(&hw->cardmutex);

        }
    }

    //pthread_rwlock_rdlock(&hw->movemutex);
    hrdac1 = hw->hrdac1;
    hrdac2 = hw->hrdac2;
    hrdac3 = hw->hrdac3;
    //pthread_rwlock_unlock(&hw->movemutex);

    librp_SPICycle(&hw->rp, hrdac1, hrdac2, hrdac3, lrdac, adc, 1, 1, hw->modeset[mode].mux1, hw->modeset[mode].mux2);

    //pthread_rwlock_wrlock(&hw->cardmutex);
    for (i=0; i<16; i++) {
       if (i==(hw->simple_link_channel-1)) {
          hw->in[i] = simple_link_get_value(hw->simple_link_socket, hw->simple_link_parameter, &imagpart);
          hw->in[i+1] = imagpart;
          hw->in[i+2] = sqrt(hw->in[i]*hw->in[i] + hw->in[i+1]*hw->in[i+1]);
          hw->in[i+3] = atan2(hw->in[i+1], hw->in[i]);
       } else if (hw->simple_link_channel==0 || (i<hw->simple_link_channel || i>(hw->simple_link_channel+2)))
           hw->in[i] = adc[i];
    }
    //pthread_rwlock_unlock(&hw->cardmutex);
    //printf("%d %g\n", hw->simple_link_channel-1, hw->in[hw->simple_link_channel-1]);

    //pthread_rwlock_wrlock(&hw->spiermutex);
    hw->spier_running = 0;
    //pthread_rwlock_unlock(&hw->spiermutex);

    //printf("t\n");
}

/*do something really periodically*/
//void waker(union sigval val)
//{
//    HWData *hw = (HWData *)val.sival_ptr;

static void waker(HWData* hw)
{
    char sendtext[15];
    int i, xymode, running, allrunning;
    bool vmoveto, vzmoveto;
    double pvxpos, pvypos, vxpos, vypos, buf;
    double verrorsignal, vz, va1, vp1, va2, vp2, vadc1, vadc2, vl1x, vl1y, vl2x, vl2y, csine, ccosine, kpfmsignal;
    double dist, step, xdir, ydir, zdir;
    struct timespec now;
    double vxcal, vxshift, vycal, vyshift, timestamp, fmresult, amresult;
    double xslope, yslope, xsloperef, ysloperef;

    running = hw->waker_running;
    allrunning = hw->running;

    if (!allrunning) return;


    if (running) {   //this should not happen - it means that last waker call was not completed yet
       //if (hw->debug)
       printf("Error, waker is already running\n");
       return;
    }
    hw->wakercount++;
    hw->waker_running = 1;

    pvxpos = pvypos = 0;

    //get all the actual values
    vxpos = hw->xpos;
    vypos = hw->ypos;
    vmoveto = hw->moveto;
    vzmoveto = hw->zmoveto;

    xymode = hw->xymode;
    vxcal = hw->xcal;
    vycal = hw->ycal;
    vxshift = hw->xshift;
    vyshift = hw->yshift;
    xslope = hw->xslope;
    yslope = hw->yslope;
    xsloperef = hw->xsloperef;
    ysloperef = hw->ysloperef;

    if (hw->triggerout) librp_D2PinSetState(&hw->rp, hw->triggerout-1, 0);

    //read the rf inputs and feedback loop result
    //read fpga data
    librp_ReadResults(&hw->rp, &verrorsignal, &buf, 0);  //error signal comes always from RP channel 1.
    if ((&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU) { //hrdac is used for z, the zpiezo value comes from it
        librp_ReadHRDAC(&hw->rp, &vz);
        vz = (vz - hw->hrzshift)/hw->hrzcal;
    } else { //z piezo comes from RP channel 2 FIXME, maybe wrong now?
        librp_ReadResults(&hw->rp, &buf, &vz, 0); 
        vz = (vz - hw->zshift)/hw->zcal;
     }

    librp_ReadLockin(&hw->rp, LIBRP_CH_1, &va1, &vp1);
    librp_ReadLockin(&hw->rp, LIBRP_CH_2, &va2, &vp2);
    librp_ReadLockinAxes(&hw->rp, LIBRP_CH_1, &vl1x, &vl1y);
    librp_ReadLockinAxes(&hw->rp, LIBRP_CH_2, &vl2x, &vl2y);

    librp_ReadADC(&hw->rp, &vadc1, &vadc2);
    librp_ReadPLLResult(&hw->rp, &fmresult, &amresult);
    
    clock_gettime(CLOCK_MONOTONIC, &now);
    timestamp = ((double)now.tv_sec + (double)now.tv_nsec / 1.0e9) - ((double)hw->start.tv_sec + (double)hw->start.tv_nsec / 1.0e9);

    //this was covered by card mutex
    hw->timestamp = timestamp;

    hw->adc1 = vadc1;
    hw->adc2 = vadc2;

    hw->zpiezo = vz;
    hw->errorsignal = verrorsignal; 
    hw->amplitude1 = va1;
    hw->phase1 = vp1;
    hw->amplitude2 = va2;
    hw->phase2 = vp2;
    hw->fmresult = fmresult;
    hw->amresult = amresult;
    hw->l1x = vl1x;
    hw->l1y = vl1y;
    hw->l2x = vl2x;
    hw->l2y = vl2y;

    //run ramp if requested
    if (hw->start_ramp == 1) {
        //printf("start ramp\n");
        hw->start_ramp = 0;
        hw->ramp_ndata = 0;
        hw->ramp_iterator = 0;
        hw->ramp_counter = 0;
        hw->ramp_running = 1;

        if (hw->ramp_quantity == LIBRP_RAMP_Z)
        {
           set_zpiezo(hw, vz, TRUE);
           hw->ramp_offset = vz;

           hw->ramp_start_z_feedback = hw->feedback;
           hw->feedback = FALSE;
           set_mode_and_feedback(hw);

           //printf("feedback off, piezo set to %g\n", vz);
        }
    }

    if (hw->ramp_running == 1) {
//        printf("ramp running, counter %d  plan npos %d   iterator %d  ntime %d counter %d\n", hw->ramp_counter, hw->ramp_plan_npos, hw->ramp_iterator, hw->ramp_plan_ntime[hw->ramp_iterator], hw->ramp_counter);
        if (hw->ramp_plan_ntime[hw->ramp_iterator] == hw->ramp_counter)  //do something in this counter step
        {
//            printf("ramp: do %d at moment %d\n", hw->ramp_plan_todo[hw->ramp_iterator], hw->ramp_counter);   //do 1 at moment 1304 tam bylo dvakrat po sobe, storing ramp value nasledne taktez, proc?  pak chybelo jedno dato, dalsi zapis byl dve dal.
            if (hw->ramp_plan_todo[hw->ramp_iterator] == RAMP_SET) 
            {
//               printf("ramp: set z piezo value to %g\n", hw->ramp_plan_value[hw->ramp_iterator] + hw->ramp_offset); 
               if (hw->ramp_quantity == LIBRP_RAMP_Z) {
                  set_zpiezo(hw, hw->ramp_plan_value[hw->ramp_iterator] + hw->ramp_offset, TRUE);
               } else {
                  hw->aout[hw->ramp_quantity-1] = hw->ramp_plan_value[hw->ramp_iterator];
                  hw->isaout = 1;
               }
            }
            else if (hw->ramp_plan_todo[hw->ramp_iterator] == RAMP_STORE) 
            {
              //here comes the trigger
              if (hw->triggerout) librp_D2PinSetState(&hw->rp, hw->triggerout-1, 1);

 //             printf("todo: storing ramp value %d\n", hw->ramp_ndata);
              store_actual_ramp_data(hw, 0); //set 0, we don't use sets for ramp yet
            }

            hw->ramp_iterator++;
        }
        hw->ramp_counter++;

        if (hw->stop_ramp || hw->ramp_counter>hw->ramp_plan_ntime[hw->ramp_plan_npos-1] 
                          || hw->ramp_ndata >= 2*hw->ramp_npos) //the second contition would mean that the plan was wrong
        {
            //printf("ramp done: counter %d  planntime %d    ndata %d 2*npos %d\n", 
            //        hw->ramp_counter, hw->ramp_plan_ntime[hw->ramp_plan_npos-1], hw->ramp_ndata, 2*hw->ramp_npos);
            hw->ramp_running = 0;
            hw->stop_ramp = 0;

           if (hw->ramp_quantity == LIBRP_RAMP_Z)
           {
              // move to the starting position
              set_zpiezo(hw, hw->ramp_offset, FALSE);

              // set feedback to the value before starting the z ramp
              hw->feedback = hw->ramp_start_z_feedback;
              set_mode_and_feedback(hw);
           }
        }
    }

    //do adaptive scan if requested  TODO some mutexes are missing?
    if (hw->scanning_adaptive || hw->scanning_line) {
       if ((hw->scan_ndata - hw->scan_completed_ndata) >= hw->scan_path_ndata) 
       {
           //printf("line completed: natad %d complndata %d   path %d setting sl to 0\n", hw->scan_ndata, hw->scan_completed_ndata, hw->scan_path_ndata); 
           hw->scanning_adaptive = 0;
           hw->scanning_line = 0;

           if (hw->therewasalift) //reset everything after the lift mode was completed
           {
              if (!hw->kpfm_no_action) 
              {
                 if (hw->kpfm_mode == LIBRP_KPFM_AM || hw->kpfm_mode == LIBRP_KPFM_AM_MANUAL)  {  //switch KPFM off if it was on in the lift mode and it is AM KPFM
                    librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, hw->f1_amplitude, hw->f1_offset); //piezo shaking on again
                    librp_SetKPFM(&hw->rp, hw->kpfm_mode, hw->f2_frequency, hw->f3_frequency, 0.0, 
                                  hw->pidkpfm_p, hw->pidkpfm_i, hw->pidkpfm_d, 
                                  hw->f2_offset, hw->filter2, 
                                  hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                                  hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, 0);

                     hw->kpfm_feedback = FALSE; 
                 }
                 usleep(10000); //wait a bit before switching feedback on
              }
              if (!hw->lift_no_fb_action) 
              {
                 hw->feedback = TRUE;
                 set_mode_and_feedback(hw);

                 usleep(10000);
              }
              //printf("lift completed at %g %g\n", hw->xpos, hw->ypos);
           }
           hw->therewasalift = 0; //we are now running a normal scan
         
       }
       else {
           if (hw->moveto==0 && hw->moving==0 && hw->moved == 1) { //some movement was finished right now, store value and do the next move, as nobody else should issue movements when we do adaptive scan
              hw->moved = 0;
              //printf("storing scan point %d lift %d\n", hw->scan_ndata, hw->lift_usezdata);
              for (i=0; i<hw->undersampling_factor; i++)
                  if ((hw->scan_ndata-hw->scan_completed_ndata)<hw->scan_path_ndata) {
                      //here comes the trigger
                      if (hw->triggerout) librp_D2PinSetState(&hw->rp, hw->triggerout-1, 1);
                      if (hw->isFake) {
                        store_fake_point_data(hw, hw->actual_set);
                      } else {
                        store_actual_point_data(hw, hw->actual_set); //set 0, we use sets only in script mode
                      }
                  }
              hw->run_next = 1;
           }
           if (hw->run_next == 1)  //this split is to allow pauses in scans
           { 
              if (hw->pause_scan==0 && (hw->scan_ndata-hw->scan_completed_ndata) < hw->scan_path_ndata) //do next movement if we are not at the end
              {
                 hw->run_next = 0;
                 hw->xreq = hw->scan_path_xydata[2*(hw->scan_ndata-hw->scan_completed_ndata)];
                 hw->yreq = hw->scan_path_xydata[2*(hw->scan_ndata-hw->scan_completed_ndata) + 1];

                 if (hw->lift_usezdata) 
                 {
                     set_zpiezo(hw, hw->lift_zdata[hw->scan_ndata-hw->scan_completed_ndata], TRUE);
                 }
                 vmoveto = hw->moveto = 1; //issue next move command
                // printf("move to %g %g\n", hw->xreq, hw->yreq);
              }
           }
       }  
    }

    //if streaming is in progress, store data in every cycle. No trigger.
    if (hw->streaming && hw->allocated_stream_ndata) store_actual_stream_data(hw, 0);

    //xy movement with constant velocity ///////////////////////////////////////////////////////////////
    //plan next xy movement
    if (vmoveto) {
       //printf("moveto\n");
       //determine the maximum number of steps and step in x and y
       xdir = (hw->xreq - hw->xpos);
       ydir = (hw->yreq - hw->ypos);
       dist = sqrt(xdir*xdir + ydir*ydir);
       if (dist>1e-15) {
          step = hw->speed*hw->scantimestep*1e-9; //scan time step is in nanoseconds
          hw->xstep = xdir*step/dist;
          hw->ystep = ydir*step/dist;
          hw->movingcounter = dist/step;
          hw->undersampling_factor = 1; 
          //undersampling: je-li movingcounter < 1, nastav undersampling N a vynásob tím krok, takže se pojede dál (musí se jet po lince).
          if (hw->movingcounter < 1) {
             //printf("undersampling candidate: dist %g  step %g  ratio %g  factor %d\n", dist, step, dist/step, (int)(step/dist + 0.5));
             hw->undersampling_factor = (int)(step/dist + 0.5);
             hw->movingcounter = (int)((double)hw->undersampling_factor*dist/step); 
             hw->xstep *= hw->undersampling_factor;
             hw->ystep *= hw->undersampling_factor;
          }

          hw->moving = 1;
          hw->moved = 0;             
          //printf("start movement to %g %g from %g %g, step %g movingcounter %d\n", hw->xreq, hw->yreq, hw->xpos, hw->ypos, step, hw->movingcounter);
          //printf("smc %d\n", hw->movingcounter);
      } else {
          hw->moving = 0;
          hw->moved = 1;
          //printf("vmoveto movement finished moved = %d, moving = %d  pos %g %g\n", hw->moved, hw->moving, hw->xpos, hw->ypos);
      }

      hw->moveto = 0;
    }

    //move if we are in the movement
    if (hw->moving) {
        hw->xpos += hw->xstep;
        hw->ypos += hw->ystep;
        vxpos = hw->xpos;
        vypos = hw->ypos;

        //printf("move: x %g  y %g   xstep %g ystep %g  counter %d\n", hw->xpos, hw->ypos, hw->xstep, hw->ystep, hw->movingcounter);
        hw->movingcounter--;
        if (hw->movingcounter<=0) {
           hw->moving = 0;
           vxpos = hw->xpos = hw->xreq;
           vypos = hw->ypos = hw->yreq;
           hw->moved = 1;  //this is to issue the last position
           //printf("main loop movement finished moved = %d, moving = %d\n", hw->moved, hw->moving);
        } 

       //we will move really now, so if needed, adjust the pid offset as well
       if (fabs(xslope)>1e-6 || fabs(yslope)>1e-6) 
          librp_SetPidOffset(&hw->rp, ((vxpos-xsloperef)*xslope + (vypos-ysloperef)*yslope)*hw->zcal + hw->zshift);
        else
          librp_SetPidOffset(&hw->rp, 0);

       if (xymode==LIBRP_XY_VOLTAGE) {
          hw->hrdac1 = vxpos*vxcal + vxshift;
          hw->hrdac2 = vypos*vycal + vyshift;
          if ((&hw->rp)->hrdac_regime != LIBRP_HRDAC_REGIME_CPU)
          {
             librp_SetHRDAC1(&hw->rp, hw->hrdac1);
             librp_SetHRDAC2(&hw->rp, hw->hrdac2);
          }
       }
       else if (xymode==LIBRP_XY_PI) {
          hw->hrdac1 = hw->hrdac2 = 0;
          if (vypos != pvypos) {
             sprintf(sendtext, "MOV 1 %.4f\n", vypos*1.0e6); //value in um
             write(hw->table_fd, sendtext, strlen(sendtext));
          }
          if (vxpos != pvxpos) {
             sprintf(sendtext, "MOV 2 %.4f\n", vxpos*1.0e6); //value in um
             write(hw->table_fd, sendtext, strlen(sendtext));
          }
          pvxpos = vxpos;
          pvypos = vypos;
       } else {
          //printf("rpt move: %g %g\n", vxpos, vypos);
          rpt_move_to(hw->rpx_socket, vxpos);
          rpt_move_to(hw->rpy_socket, vypos);
       }

       hw->hrdac3 = 0;
       pvxpos = vxpos;
       pvypos = vypos;
    } 

    //z movement with constant velocity //////////////////////////////////
    //plan next z movement
    if (vzmoveto) {
       hw->zpos = hw->zpiezo;
       zdir = (hw->zreq - hw->zpos);
       //determine the maximum number of steps and step in x and y
       dist = fabs(zdir);
       if (dist>1e-15) {
          step = hw->zspeed*hw->scantimestep*1e-9; //scan time step is in nanoseconds
          hw->zstep = zdir*step/dist;
          hw->zmovingcounter = dist/step;
          hw->zmoving = 1;
          hw->zmoved = 0;             
          //printf("z movement started, actual %g dist %g step %g\n", hw->zpos, dist, hw->zstep);
      } else {
          hw->zmoved = 1;
          //printf("vmoveto movement finished moved = %d, moving = %d  pos %g %g\n", hw->moved, hw->moving, hw->xpos, hw->ypos);
      }

      hw->zmoveto = 0;
    }

    //move if we are in the movement
    if (hw->zmoving) {
        hw->zpos += hw->zstep;
        hw->zmovingcounter--;
        if (hw->zmovingcounter<=0) {
           hw->zmoving = 0;
           hw->zpos = hw->zreq;
           hw->zmoved = 1; 
        } 

       //send the zpos command
       set_zpiezo(hw, hw->zpos, FALSE);
    } 

    //this now works always when kpfm is on (and feedback off for AM mode)
    if (hw->kpfm_feedback && (hw->kpfm_mode == LIBRP_KPFM_AM || hw->kpfm_mode == LIBRP_KPFM_FM ||  hw->kpfm_mode == LIBRP_KPFM_SIDEBAND)) {

        csine = vl2x;
        ccosine = vl2y;

        if (hw->kpfm_source) kpfmsignal = ccosine;
        else kpfmsignal = csine;

        if (hw->kpfm_dir==0)
           hw->f2_offset -= hw->pidkpfm_p*kpfmsignal*1000;
        else
           hw->f2_offset += hw->pidkpfm_p*kpfmsignal*1000;

        if (hw->f2_offset>5) hw->f2_offset = 5;
        if (hw->f2_offset<-5) hw->f2_offset = -5;

        //printf("fo %g %g %g %g\n", csine, ccosine, hw->pidkpfm_p, hw->f2_offset);

        librp_SetKPFM(&hw->rp, hw->kpfm_mode, hw->f2_frequency, hw->f3_frequency, hw->f2_amplitude, hw->pidkpfm_p, hw->pidkpfm_i, hw->pidkpfm_d, hw->f2_offset, hw->filter2, 
                      hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                      hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, 0);
    }
    
    if (hw->dart_mode == LIBRP_DART_ON) {

        if (va1>va2) hw->dart_frequencyshift -= 100.0*hw->piddart_p;
        else hw->dart_frequencyshift += 100.0*hw->piddart_p;

        if (hw->dart_frequencyshift > 5000)  hw->dart_frequencyshift = 5000;
        if (hw->dart_frequencyshift < -5000) hw->dart_frequencyshift = -5000; 

        printf("dart on! %g %g  shift %g\n", va1, va2, hw->dart_frequencyshift);

        librp_SetDART(&hw->rp, hw->dart_mode, hw->dart_frequency + hw->dart_frequencyshift, hw->dart_amplitude, hw->piddart_p, hw->piddart_i, hw->piddart_d, 
                      hw->dart_freqspan, hw->filter1, hw->modeset[hw->mode].lockin1_lfr);
    } else hw->dart_frequencyshift = 0;


   Stabilizer* stbl = hw->stabilizer;
   pthread_rwlock_wrlock(&stbl->mutex);
   if (stbl->is_active && !hw->ramp_running && !hw->scanning_line && !hw->scanning_adaptive && !hw->moving) {
      stbl_run(hw);
   }
   pthread_rwlock_unlock(&stbl->mutex);

   hw->waker_running = 0;
}


void stop_scan(HWData *hw, bool gotostart)
{
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->scanning_adaptive = hw->scanning_line = hw->scanning_script = 0;
    pthread_rwlock_unlock(&hw->controlmutex);

    //move to the beginning if there was some scan path
    pthread_rwlock_wrlock(&hw->controlmutex);
    if (hw->scan_path_ndata && gotostart) {
          hw->xreq = hw->scan_path_xydata[0];
          hw->xreq = hw->scan_path_xydata[1];
          hw->moveto = 1;
    }
    pthread_rwlock_unlock(&hw->controlmutex);
   
    //if there was a (kpfm) lift and was not stopped automatically, start feedback again
    if (hw->therewasalift) //reset everything after the lift mode was completed
    {
       if (!hw->kpfm_no_action)
       {
           if (!hw->lift_no_fb_action)
           {
              hw->feedback = TRUE;
              set_mode_and_feedback(hw);

              usleep(10000);
           }
           hw->therewasalift = 0; //we are now running a normal scan
       }
    }
}

void run_adaptive_scan(HWData *hw)
{
    printf("starting adaptive scan for %d values\n", hw->scan_path_ndata);

    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->scan_ndata = hw->scan_completed_ndata = 0;
    pthread_rwlock_unlock(&hw->controlmutex);

    printf("allocating storage for adaptive scan\n");
    allocate_scan_data(hw, hw->scan_path_ndata);

    //move to the beginning
    pthread_rwlock_wrlock(&hw->movemutex);
    hw->xreq = hw->scan_path_xydata[0];
    hw->yreq = hw->scan_path_xydata[1];
    if (hw->lift_usezdata) {
       set_zpiezo(hw, hw->lift_zdata[0], TRUE);
    }
    hw->moveto = 1;
    hw->scanning_adaptive = 1;
    pthread_rwlock_unlock(&hw->movemutex);

//    printf("issuing move to command to %g %g\n", hw->xreq, hw->yreq);
  
    //when movement is finished, moved=1. This means that point can be stored in waker and next movement issued.
}

void run_line_scan(HWData *hw)
{
    int i;
    double xstart, ystart, xend, yend;

    pthread_rwlock_wrlock(&hw->controlmutex);
    if (!hw->scanning_script) hw->scan_ndata = hw->scan_completed_ndata = 0;
    //hw->scanning_line = 1;
    xstart = hw->scan_line_xfrom;
    ystart = hw->scan_line_yfrom;
    xend = hw->scan_line_xto;
    yend = hw->scan_line_yto;

    //if (hw->scan_path_xydata != NULL) free(hw->scan_path_xydata);
    if (hw->scan_line_ndata_prev != hw->scan_line_ndata || hw->scan_path_xydata == NULL) {
             if (hw->scan_path_xydata == NULL) {
                 if (hw->debug) printf("allocating %d xy scandata\n", hw->scan_line_ndata);
                 hw->scan_path_xydata = (double*) malloc(2*hw->scan_line_ndata*sizeof(double));
             }
             else {
                 if (hw->debug) printf("re-allocating %d xy scandata\n", hw->scan_line_ndata);
                 hw->scan_path_xydata = (double*) realloc(hw->scan_path_xydata, 2*hw->scan_line_ndata*sizeof(double));
             } 
             if (hw->debug) printf("done.\n");
             hw->scan_line_ndata_prev = hw->scan_line_ndata;
    } else if (hw->debug) printf("no need to allocate scan xydata: res %d prev %d\n", hw->scan_line_ndata, hw->scan_line_ndata_prev);

    for (i=0; i<hw->scan_line_ndata; i++) {
        hw->scan_path_xydata[2*i] = xstart + (xend-xstart)*(double)i/((double)hw->scan_line_ndata);
        hw->scan_path_xydata[2*i+1] = ystart + (yend-ystart)*(double)i/((double)hw->scan_line_ndata);
    }
    hw->scan_path_ndata = hw->scan_line_ndata;
    pthread_rwlock_unlock(&hw->controlmutex);

    if (!hw->scanning_script) allocate_scan_data(hw, hw->scan_line_ndata); //if we are in Lua script, all this was already allocated

    //move to the beginning
    pthread_rwlock_wrlock(&hw->movemutex);
    hw->xreq = hw->scan_path_xydata[0];
    hw->yreq = hw->scan_path_xydata[1];
    pthread_rwlock_unlock(&hw->movemutex);
 
    if (hw->lift_usezdata) {
       //switch feedback off
       if (hw->debug) printf("lift mode profile will be run, zdata[0] = %g\n", hw->lift_zdata[0]);
       pthread_rwlock_wrlock(&hw->controlmutex);
       hw->therewasalift = 1;  //we are now running a lift scan
       set_zpiezo(hw, hw->lift_zdata[0], TRUE); //to keep z when feedback is switched off
       pthread_rwlock_unlock(&hw->controlmutex);

       usleep(10000);

       if (!hw->lift_no_fb_action) 
       {
          pthread_rwlock_wrlock(&hw->controlmutex);
          hw->feedback = FALSE; 
          pthread_rwlock_unlock(&hw->controlmutex);
          printf("there should be lift, switch feedback off\n");
          set_mode_and_feedback(hw);
 
          usleep(10000);
       }

       //we are in lift, start KPFM by setting its amplitude to the right vlaue, switch also the amplitude off.  The opposite is done in waker, at end of the profile
       if (!hw->kpfm_no_action) 
       {
          if (hw->kpfm_mode != LIBRP_KPFM_OFF) {
              if (hw->kpfm_mode == LIBRP_KPFM_AM || hw->kpfm_mode == LIBRP_KPFM_AM_MANUAL) hw->f2_offset = 0; //reset the value

              if (hw->kpfm_mode == LIBRP_KPFM_AM || hw->kpfm_mode == LIBRP_KPFM_AM_MANUAL) librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, 0.0, hw->f1_offset); //piezo oscillation off

              librp_SetKPFM(&hw->rp, hw->kpfm_mode, hw->f2_frequency, hw->f3_frequency, hw->f2_amplitude, hw->pidkpfm_p, hw->pidkpfm_i, hw->pidkpfm_d, hw->f2_offset, hw->filter2, 
                            hw->modeset[hw->mode].lockin2_filter_amplitude, hw->modeset[hw->mode].lockin2_filter_phase, 
                            hw->modeset[hw->mode].lockin2_nwaves, hw->modeset[hw->mode].lockin2_lfr, 0);
              if (hw->debug) printf("kpfm amplitude set to %g\n", hw->f2_amplitude);

              hw->kpfm_feedback = TRUE;
          }
       }
       usleep(10000);

    } else {
       if (hw->debug) printf("normal profile will be run\n");
       hw->therewasalift = 0;
       //switching lift mode off was moved to the end of the lift profile
    }
    //printf("run line scan issuing moveto %g %g\n", hw->xreq, hw->yreq);
    hw->scanning_line = 1;
    pthread_rwlock_wrlock(&hw->movemutex);
    hw->moveto = 1;
    pthread_rwlock_unlock(&hw->movemutex);
 
    //when movement is finished, moved=1. This means that point can be stored in waker and next movement issued.
}

void run_line_and_store(HWData *hw, double xto, double yto, int res, int set)
{
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->scan_line_xfrom = hw->xpos;
    hw->scan_line_yfrom = hw->ypos;
    hw->scan_line_xto = xto;
    hw->scan_line_yto = yto;
    hw->scan_line_ndata = res;
    hw->actual_set = set;
    pthread_rwlock_unlock(&hw->controlmutex);

    run_line_scan(hw);
    //hw->scan_completed_ndata = hw->scan_ndata; //store so far completed number of points, this is now called late as the waker already confirms that scan is done.
    //printf("completed ndata set to %d, line and store finished\n", hw->scan_completed_ndata);
}

/*do lua scan if this is requested*/
void *script_scan_thread(void *ptr)
{
    HWData *hw = (HWData *)ptr;
    FILE *fw;
    if (hw->debug) printf("starting script scan for up to %d values\n", hw->scan_script_ndata);

    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->scan_ndata = hw->scan_completed_ndata = 0;
    hw->scanning_script = 1;
    hw->scan_id = hw->new_scan_id;
    hw->stop_scanning_script = FALSE;
    pthread_rwlock_unlock(&hw->controlmutex);

    if (hw->debug) printf("writing scan.lua file\n");
    fw = fopen("scan.lua", "w");
    fprintf(fw, "%s", hw->script);
    fclose(fw);

    if (hw->debug) printf("allocating storage for script scan: %d values\n", hw->scan_script_ndata);
    allocate_scan_data(hw, hw->scan_script_ndata);
    if (hw->debug) printf("hw pointer %p  zdata pointer: %p\n", hw, hw->scan_z_data);

    if (hw->debug) printf("running lua script\n");
    run_lua_scan(hw);

    if (hw->debug) printf("lua script completed\n");
    pthread_rwlock_wrlock(&hw->movemutex);
    hw->scanning_script = 0;
    pthread_rwlock_unlock(&hw->movemutex);

    return 0;
}


void run_script_scan(HWData *hw)
{
    int ret;

    if (hw->scan_thread_was_set == 0) {
          hw->scan_thread_was_set = 1;
    } else {
          pthread_join(hw->script_thread, NULL);
          hw->scan_thread_was_set = 0;
    }

    ret = pthread_create(&hw->script_thread, NULL, script_scan_thread, (void*) hw);
    if (ret) {
          if (hw->debug) printf("Error: cannot start scan thread\n");
    }
    if (hw->debug) printf("script scan thread started\n");
}

void run_ramp(HWData *hw)
{
    int i, n, counter, timestep_up, timestep_down;
    double value, from, to, startvalue;

    if (hw->debug) printf("Run ramp\n");

    pthread_rwlock_wrlock(&hw->controlmutex); //TODO: reduce this very long mutex
    hw->ramp_ndata = 0;
    from = hw->ramp_from;
    to = hw->ramp_to;
   
    allocate_ramp_data(hw, 2*hw->ramp_npos);
 
    //fill the ramp plan
    if (hw->ramp_quantity == LIBRP_RAMP_TIME)
        hw->ramp_plan_npos = hw->ramp_npos + 40; //only store commands, one direction, plus something to go to the start position and back
    else
        hw->ramp_plan_npos = 4*hw->ramp_npos + 40;  //set and store pairs, both directions, plus something to go to the start position and back
    if (hw->debug) printf("ramp plan: %d values, plan npos is %d\n", hw->ramp_npos, hw->ramp_plan_npos);

    if (hw->ramp_plan_ntime == NULL) hw->ramp_plan_ntime = (int *)malloc(hw->ramp_plan_npos*sizeof(int));
    else hw->ramp_plan_ntime = (int *)realloc(hw->ramp_plan_ntime, hw->ramp_plan_npos*sizeof(int));

    if (hw->ramp_plan_todo == NULL) hw->ramp_plan_todo = (int *)malloc(hw->ramp_plan_npos*sizeof(int));
    else hw->ramp_plan_todo = (int *)realloc(hw->ramp_plan_todo, hw->ramp_plan_npos*sizeof(int));

    if (hw->ramp_plan_value == NULL) hw->ramp_plan_value = (double *)malloc(hw->ramp_plan_npos*sizeof(double));
    else hw->ramp_plan_value = (double *)realloc(hw->ramp_plan_value, hw->ramp_plan_npos*sizeof(double));


    timestep_up = (int)(hw->ramp_time_up/hw->ramp_npos/(hw->scantimestep*1e-9)); //one ramp value step in timestep units
    timestep_down = (int)(hw->ramp_time_down/hw->ramp_npos/(hw->scantimestep*1e-9)); //one ramp value step in timestep units

    if (timestep_up<1) timestep_up = 1;
    if (timestep_down<1) timestep_down = 1;

    if (hw->debug) printf("timestep up:  time up %g  npos %d  scantimestep*1e9 %g, timestep %g\n", hw->ramp_time_up, hw->ramp_npos, hw->scantimestep*1e-9, hw->ramp_time_up/hw->ramp_npos/(hw->scantimestep*1e-9));

    n = 0;

    //get actual value for smooth transition to the start
    if (hw->ramp_quantity != LIBRP_RAMP_Z) startvalue = hw->aout[hw->ramp_quantity-1];
    else startvalue = 0; //this is handled by ramp_offset when feedback is switched off 

    counter = 0;
    //go from the actual value to the start
    if (hw->ramp_quantity != LIBRP_RAMP_TIME) {
        for (i=0; i<20; i++)
        {
            value = startvalue + (from - startvalue)*(double)(i+1)/20.0;
            hw->ramp_plan_todo[n] = RAMP_SET;
            hw->ramp_plan_ntime[n] = counter;
            hw->ramp_plan_value[n] = value;
            counter += timestep_up;
            n++;
        }
    }
    counter += hw->ramp_start_delay/(hw->scantimestep*1e-9);    

    //run ramp up
    for (i=0; i<hw->ramp_npos; i++) {
        if (hw->ramp_quantity != LIBRP_RAMP_TIME) {
           value = from + (to-from)*(double)(i+1)/((double)hw->ramp_npos);
           hw->ramp_plan_todo[n] = RAMP_SET;
           hw->ramp_plan_ntime[n] = counter;
           hw->ramp_plan_value[n] = value;
           counter += timestep_up;
           n++;
        }
        else {
           counter += MAX(timestep_up-1, 0);
        }

        hw->ramp_plan_todo[n] = RAMP_STORE;
        hw->ramp_plan_ntime[n] = counter;
        hw->ramp_plan_value[n] = value;
        counter += 1;
        n++;
    }

    if (hw->ramp_quantity != LIBRP_RAMP_TIME) {
        //wait at peak
        counter += hw->ramp_peak_delay/(hw->scantimestep*1e-9);

        //run ramp down
        for (i=0; i<hw->ramp_npos; i++) {
           value = from + (to-from)*(double)(hw->ramp_npos-i-1)/((double)hw->ramp_npos);
           hw->ramp_plan_todo[n] = RAMP_SET;
           hw->ramp_plan_ntime[n] = counter;
           hw->ramp_plan_value[n] = value;
           counter += timestep_down;
           n++;
        

           hw->ramp_plan_todo[n] = RAMP_STORE;
           hw->ramp_plan_ntime[n] = counter;
           hw->ramp_plan_value[n] = value;
           counter += 1;
           n++;
       }
    }

    //return to the value preceding ramp
    if (hw->ramp_quantity != LIBRP_RAMP_TIME) {
        for (i=0; i<20; i++)
        {
            value = from + (startvalue - from)*(double)(i+1)/20.0;
            hw->ramp_plan_todo[n] = RAMP_SET;
            hw->ramp_plan_ntime[n] = counter;
            hw->ramp_plan_value[n] = value;
            counter += timestep_up;
            n++;
        }
    }

    hw->ramp_plan_npos = n;

/*
    for (i=0; i<hw->ramp_plan_npos; i++) {
        printf("ramp plan %d: time %d todo %d value %g\n", i, hw->ramp_plan_ntime[i], hw->ramp_plan_todo[i], hw->ramp_plan_value[i]);
    }
*/
    pthread_rwlock_unlock(&hw->controlmutex);

    //assume that user had moved to the ramp start independently, we only do ramp.
    pthread_rwlock_wrlock(&hw->movemutex);
    hw->stop_ramp = 0;
    hw->start_ramp = 1;
    pthread_rwlock_unlock(&hw->movemutex);
   
}

void run_stream(HWData *hw)
{
    printf("run stream started\n");
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->stream_ndata = 0;
    pthread_rwlock_unlock(&hw->controlmutex);

    allocate_stream_data(hw, hw->allocated_stream_ndata);

    hw->streaming = 1;
}

void stop_stream(HWData *hw)
{
    hw->streaming = 0;
}


/*report results periodically*/
void logger(union sigval val)
{
    HWData *hw = (HWData *)val.sival_ptr;

//static void logger(int sig, siginfo_t *si, void *uc)
//{
//    HWData *hw = (HWData *)si->_sifields._rt.si_sigval.sival_ptr;
    double buf, vin1, vin2, debug, timestamp, xpos, ypos, zpiezo, errorsignal, in1, in2, fmresult, amresult, amplitude1, phase1, amplitude2, phase2;

    if (hw->startlogging && hw->logging==0) {
       hw->fw = fopen("data.txt", "w");
       hw->startlogging = 0;
       hw->logging = 1;
    }
    if (hw->stoplogging && hw->logging==1) {
       fclose(hw->fw);
       hw->stoplogging = 0;
       hw->logging = 0;
    }

    if (global_stop) stop(hw);


    if (hw->logging) fprintf(hw->fw, "%+12.10e  %g %g %g %g   %g %g\n", 
                             hw->timestamp, hw->xpos, hw->ypos, hw->zpiezo, hw->errorsignal, hw->in[0], hw->in[1]);
 
    librp_ReadDebug(&hw->rp, &buf, &debug);
    /*if (hw->debug)*/ 
//    pthread_rwlock_rdlock(&hw->cardmutex);
    timestamp = hw->timestamp;
    xpos = hw->xpos;
    ypos = hw->ypos;
    zpiezo = hw->zpiezo;
    errorsignal = hw->errorsignal;
    in1 = hw->adc1;
    in2 = hw->adc2;
    vin1 = hw->in[0];
    vin2 = hw->in[1];
    fmresult = hw->fmresult;
    amresult = hw->amresult;
    amplitude1 = hw->amplitude1;
    phase1 = hw->phase1;
    amplitude2 = hw->amplitude2;
    phase2 = hw->phase2;
//    pthread_rwlock_unlock(&hw->cardmutex);

    if (hw->pwakercount == hw->wakercount) {
        printf("Error: waker died, trying to resurrect it\n.");
        waker_stop = 1;
        pthread_join(waker_tid, NULL);
        if (pthread_create(&waker_tid, NULL, &waker_thread, (void*)hw) < 0) {
            printf("could not create thread\n");
        }
    }
    if (hw->pspiercount == hw->spiercount) {
        printf("Error: spier died\n.");
        exit(0);
    }
    hw->pwakercount = hw->wakercount;
    hw->pspiercount = hw->spiercount;
   

    // printf("RP actual at %g: e: %g z: %g  debug %g  add1 %g  ap %g %g  pos %g %g  adcs %g %g  l1 %g %g  in %g %g\n",
    //         timestamp, errorsignal, zpiezo, debug, amresult, amplitude1, phase1, xpos, ypos, in1, in2,
    //         hw->l1x, hw->l1y, vin1, vin2);
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}
/*
void create_intimer(HWData *hw)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;

    printf("creating waker timer\n");
    hw->waker_running = 0;

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_value.sival_ptr = hw;
    se.sigev_notify_function = waker;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = 1;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = hw->scantimestep;

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if (status == -1) fprintf(stderr, "Error creating timer\n");

    status = timer_settime(timer_id, 0, &ts, 0);
    if (status == -1) fprintf(stderr, "Error setting timer time\n");
    hw->waker_id = timer_id;
}
*/

void* waker_thread(void *ptr)
{
    waker_stop = 0;

    HWData *hw = (HWData *)ptr;
    hw->waker_running = 0;

    // set thread priority, may increase consistency
    struct sched_param schedparm = {};
    schedparm.sched_priority = 1;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedparm);

    struct itimerspec ts;
    ts.it_value.tv_sec = 1;
    ts.it_value.tv_nsec = hw->scantimestep;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = hw->scantimestep;

    int fd = timerfd_create(CLOCK_REALTIME, 0);
    if (fd == -1) {
        printf("Could not create timer\n");
        return NULL;
    }
    if (timerfd_settime(fd, 0, &ts, NULL) == -1) {
        printf("Could not set timer\n");
        return NULL;
    }

    uint64_t exp;
    while (!waker_stop) {
        read(fd, &exp, sizeof(uint64_t));
        waker(hw);
    }

    // cleanup, When all file descriptors associated with the
    // same timer object have been closed, the timer is disarmed
    // and its resources are freed by the kernel.
    close(fd);

    return NULL;
}


void create_spitimer(HWData *hw)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;

    printf("creating SPI timer\n");
    hw->spier_running = 0;

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_value.sival_ptr = hw;
    se.sigev_notify_function = spier;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = 1;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 1000000; //200000; //was 1000000

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if (status == -1) fprintf(stderr, "Error creating timer\n");

    status = timer_settime(timer_id, 0, &ts, 0);
    if (status == -1) fprintf(stderr, "Error setting timer time\n");
    hw->spier_id = timer_id;
}


/*
void create_spitimer(HWData *hw)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;
    struct sigaction sa = { 0 };

    printf("creating spi timer\n");
    hw->waker_running = 0;

    se.sigev_notify = SIGEV_SIGNAL;
    se.sigev_signo = SIGRTMIN+1;
    se.sigev_value.sival_ptr = hw;
    sa.sa_sigaction = spier;
    sa.sa_flags = SA_SIGINFO;//SA_RESTART;//SA_SIGINFO;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = 1;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = 1000000;


    // Initialize signal 
    sigemptyset(&sa.sa_mask);
    printf("Establishing handler for signal %d\n", SIGRTMIN+1);

    // Register signal handler 
    if (sigaction(SIGRTMIN+1, &sa, NULL) == -1){
        fprintf(stderr, "Error sigaction: %s\n", strerror(errno));
        return;
    }

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if (status == -1) fprintf(stderr, "Error creating timer\n");

    status = timer_settime(timer_id, 0, &ts, 0);
    if (status == -1) fprintf(stderr, "Error setting timer time\n");
    hw->spier_id = timer_id;
}
*/

void create_logger(HWData *hw)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;

    printf("creating logger timer\n");

    se.sigev_notify = SIGEV_THREAD;
    se.sigev_value.sival_ptr = hw;
    se.sigev_notify_function = logger;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = 2;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 2;
    ts.it_interval.tv_nsec = 0;

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if (status == -1) fprintf(stderr, "Error creating timer\n");

    status = timer_settime(timer_id, 0, &ts, 0);
    if (status == -1) fprintf(stderr, "Error setting timer time\n");
    hw->logger_id = timer_id;
}

/*
void create_logger(HWData *hw)
{
    timer_t timer_id;
    int status;
    struct itimerspec ts;
    struct sigevent se;
    struct sigaction sa = { 0 };

    printf("creating spi timer\n");
    hw->waker_running = 0;

    se.sigev_notify = SIGEV_SIGNAL;
    se.sigev_signo = SIGRTMIN+2;
    se.sigev_value.sival_ptr = hw;
    sa.sa_sigaction = logger;
    sa.sa_flags = SA_SIGINFO;//SA_RESTART;//SA_SIGINFO;
    se.sigev_notify_attributes = NULL;

    ts.it_value.tv_sec = 1;
    ts.it_value.tv_nsec = 0;
    ts.it_interval.tv_sec = 2;
    ts.it_interval.tv_nsec = 0;


    // Initialize signal 
    sigemptyset(&sa.sa_mask);
    printf("Establishing handler for signal %d\n", SIGRTMIN+2);

    // Register signal handler 
    if (sigaction(SIGRTMIN+2, &sa, NULL) == -1){
        fprintf(stderr, "Error sigaction: %s\n", strerror(errno));
        return;
    }

    status = timer_create(CLOCK_REALTIME, &se, &timer_id);
    if (status == -1) fprintf(stderr, "Error creating timer\n");

    status = timer_settime(timer_id, 0, &ts, 0);
    if (status == -1) fprintf(stderr, "Error setting timer time\n");
    hw->logger_id = timer_id;
}
*/

void intHandler(int sig)
{
    global_stop = 1;
}

void stop(HWData *hw)
{
    char sendtext[15];

    printf("stopping the hwserver...\n");    
    pthread_rwlock_wrlock(&hw->controlmutex);
    hw->running = 0;
    pthread_rwlock_unlock(&hw->controlmutex);
    sleep(1);

    printf("deleting timers\n");

    timer_delete(hw->logger_id);
    //timer_delete(hw->waker_id);

    // stop waker
    waker_stop = 1;
    printf("Wainting for waker thread to join\n");
    pthread_join(waker_tid, NULL);
    printf("Thread joined\n");

    sleep(1);

    //stop all the outputs
    librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, 0.0, 0.0);
    librp_SetGen(&hw->rp, LIBRP_CH_2, hw->f2_frequency, 0.0, 0.0);
    set_zpiezo(hw, 0, TRUE);

    if (hw->xymode == LIBRP_XY_PI) {
       /*set servo on*/
       printf("setting servo x off\n");
       sprintf(sendtext, "SVO 1 0\n");
       write(hw->table_fd, sendtext, strlen(sendtext));
       printf("servo x unset\n");


       printf("setting servo y off\n");
       sprintf(sendtext, "SVO 2 0\n");
       write(hw->table_fd, sendtext, strlen(sendtext));
       printf("servo y unset\n");
       printf("Servo regime unset for both stages\n");
    }

    printf("deleting spier\n");
    timer_delete(hw->spier_id); //this leads to segfault, don't know why

    if (hw->simple_link_channel>0) {
        printf("stopping simple link\n");
        sleep(5);
        close(hw->simple_link_socket);
    }



    printf("stopping RP API...\n");

    if (!librp_Close(&hw->rp)) printf("done.\n");

    close(hw->server_socket);

    sleep(2);

    exit(0);

}



int main(int argc, char *argv[])
{
     HWData hw;

     int portno, userini;
     int socket_desc, client_sock, c;
     struct sockaddr_in server, client;
     struct hostent *rpserver;
     struct sockaddr_in rpserv_addr;

     int pid_file = open("/var/run/hwserver.pid", O_CREAT | O_RDWR, 0666);
     int rc = flock(pid_file, LOCK_EX | LOCK_NB);
     if (rc) {
         fprintf(stderr, "ERROR: another hwserver seems to be running\n");
         exit(1);
         //  if(EWOULDBLOCK == errno) 
         //  ; // another instance is running
     }

     if (argc < 2) {
         fprintf(stderr,"ERROR, not enough parameters provided\nSyntax: ./hwserver port (ini_file)\n");
         exit(1);
     }
   
     printf("starting RP hwserver...\n");
     global_stop = 0;

     signal(SIGINT, intHandler);
     signal(SIGTERM, intHandler);

     init_data(&hw);
     hw.running = 1;
     hw.reset_spi = 0;

     printf("hwserver version: %s\n", hw.version);

     //load the bitstream
     printf("Loading the bitstream... ");
     fflush(stdout);
     system("cat system_wrapper.bit > /dev/xdevcfg");
     printf("done.\n");

     //load the ini file

     userini = 0;
     if (argc==3) {
         if (ini_parse(argv[2], inihandler, &hw) < 0) 
             printf("Can't load user supplied ini file '%s', using default one.\n", argv[2]);
         else userini = 2;
     }

     if (!userini) {
         if (ini_parse("hwserver.ini", inihandler, &hw) < 0) 
             printf("Can't load default file 'hwserver.ini'.\n");
         else userini = 1;
     }

     if (userini==0) printf("No configuration found, using default values\n");
     else if (userini==1) printf("Configuration loaded from default file 'hwserver.ini'.\n");
     else if (userini==2) printf("Configuration loaded from user provided file '%s'.\n", argv[2]);

     ini_cleanup(&hw);  
 
     printf("starting RP API: HRDAC regime %d, ranges  %d %d %d...\n", hw.hrdac_regime, hw.hrdac1_range, hw.hrdac2_range, hw.hrdac3_range);
  
 
     if (!librp_Init(&hw.rp, hw.hrdac_regime, hw.hrdac1_range, hw.hrdac2_range, hw.hrdac3_range,
                      hw.modeset[hw.mode].input1_range, hw.modeset[hw.mode].input2_range, hw.dds1_range, hw.dds2_range, hw.oversampling,
                      hw.rp1_input_hv, hw.rp2_input_hv, hw.rp_bare_output, hw.rp_bare_input)) printf("done.\n");
     else {
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

 
     if (hw.debug>1) hw.rp.debug = 1;

     //set initial mode
     printf("setting initial modes: %d\n", hw.mode);
     librp_SetState(&hw.rp, hw.modeset[hw.mode].error_source, hw.feedback, hw.modeset[hw.mode].out1, hw.modeset[hw.mode].out2, hw.modeset[hw.mode].outhr, 
                     hw.modeset[hw.mode].swap_in, hw.modeset[hw.mode].swap_out, hw.modeset[hw.mode].pidskip); 
     librp_SetPLL(&hw.rp, hw.pllfeedback, hw.ampfeedback, hw.pidpll_setpoint, hw.pidamplitude_setpoint,
                          hw.pidpll_p, hw.pidpll_i, hw.pidpll_d,
                          hw.pidamplitude_p, hw.pidamplitude_i, hw.pidamplitude_d,
                          hw.pll_phase_limit_factor, hw.pll_frequency_limit_factor,
                          hw.modeset[hw.mode].pllskip, hw.modeset[hw.mode].pll_input);
     librp_SetInputRange(&hw.rp, hw.modeset[hw.mode].input1_range, hw.modeset[hw.mode].input2_range);

     set_zpiezo(&hw, 0, TRUE); //init the z piezo value

     //init filters
     set_filter(&hw, LIBRP_CH_1);
     set_filter(&hw, LIBRP_CH_2);

     //init PIDs
     set_pid(&hw);
     set_zpiezo(&hw, 0, TRUE); //init the z piezo value

     //init hardware time
     clock_gettime(CLOCK_MONOTONIC, &hw.start);

    /*PI communication */
     if (hw.xymode == LIBRP_XY_PI) {

        struct termios options;
        char sendtext[15];
        printf("Opening XY table control link..."); fflush(stdout);
        hw.table_fd = open("/dev/ttyPS1", O_WRONLY | O_NOCTTY | O_NDELAY);
        if (hw.table_fd == -1)
        {
            perror("open_port: Unable to open /dev/ttyS0 - PI XY table");
        } else
        {
            fcntl(hw.table_fd, F_SETFL, 0);
        }
        tcgetattr(hw.table_fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag &= ~CSIZE;//
        options.c_cflag |= CS8;
        options.c_cflag |= CRTSCTS;//
        options.c_oflag &= ~OPOST; //raw output//
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ECHO | ECHONL | ISIG);
        tcsetattr(hw.table_fd, TCSANOW, &options);
        printf("Done.\n");

        sleep(1);

        /*set servo on*/
        printf("setting servo x\n");
        sprintf(sendtext, "SVO 1 1\n");
        write(hw.table_fd, sendtext, strlen(sendtext)); //number of chars + 1
        printf("servo x set\n");

        printf("setting servo y\n");
        sprintf(sendtext, "SVO 2 1\n");
        write(hw.table_fd, sendtext, strlen(sendtext)); //number of chars + 1
        printf("servo y set\n");
        printf("Servo regime set for both stages\n");
     }
     else if (hw.xymode == LIBRP_XY_RPTABLE) {

        //x axis
        hw.rpx_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (hw.rpx_socket < 0) {
            perror("Cannot create socket");
            exit(EXIT_FAILURE);
        }
        rpserver = gethostbyname(hw.rpx_ip);
        if (rpserver == NULL) {
            fprintf(stderr, "Cannot resolve hostname %s.\n", hw.rpx_ip);
            exit(EXIT_FAILURE);
        }
        bzero(&rpserv_addr, sizeof(rpserv_addr));
        rpserv_addr.sin_family = AF_INET;
        bcopy(rpserver->h_addr, &rpserv_addr.sin_addr.s_addr, rpserver->h_length);
        rpserv_addr.sin_port = htons(hw.rpx_port);
        if (connect(hw.rpx_socket, (struct sockaddr*)&rpserv_addr, sizeof(rpserv_addr)) < 0) {
           fprintf(stderr, "Cannot connect to address %s, port %d: %s\n", hw.rpx_ip, hw.rpx_port, strerror(errno));
           exit(EXIT_FAILURE);
        }

        //y axis
        hw.rpy_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (hw.rpy_socket < 0) {
            perror("Cannot create socket");
            exit(EXIT_FAILURE);
        }
        rpserver = gethostbyname(hw.rpy_ip);
        if (rpserver == NULL) {
            fprintf(stderr, "Cannot resolve hostname %s.\n", hw.rpy_ip);
            exit(EXIT_FAILURE);
        }
        bzero(&rpserv_addr, sizeof(rpserv_addr));
        rpserv_addr.sin_family = AF_INET;
        bcopy(rpserver->h_addr, &rpserv_addr.sin_addr.s_addr, rpserver->h_length);
        rpserv_addr.sin_port = htons(hw.rpy_port);
        if (connect(hw.rpy_socket, (struct sockaddr*)&rpserv_addr, sizeof(rpserv_addr)) < 0) {
           fprintf(stderr, "Cannot connect to address %s, port %d: %s\n", hw.rpy_ip, hw.rpy_port, strerror(errno));
           exit(EXIT_FAILURE);
        }

        //set zero
        rpt_set_zero(hw.rpx_socket);
        rpt_set_zero(hw.rpy_socket);

        //set feedback
        rpt_set_feedback(hw.rpx_socket, 1);
        rpt_set_feedback(hw.rpy_socket, 1);
     }

     //connect to simple communication link for external sensor if needed
     if (hw.simple_link_channel > 0) {

        hw.simple_link_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (hw.simple_link_socket < 0) {
            perror("Cannot create socket");
            exit(EXIT_FAILURE);
        }
        rpserver = gethostbyname(hw.simple_link_ip);
        if (rpserver == NULL) {
            fprintf(stderr, "Cannot resolve hostname %s.\n", hw.simple_link_ip);
            exit(EXIT_FAILURE);
        }
        bzero(&rpserv_addr, sizeof(rpserv_addr));
        rpserv_addr.sin_family = AF_INET;
        bcopy(rpserver->h_addr, &rpserv_addr.sin_addr.s_addr, rpserver->h_length);
        rpserv_addr.sin_port = htons(hw.simple_link_port);
        if (connect(hw.simple_link_socket, (struct sockaddr*)&rpserv_addr, sizeof(rpserv_addr)) < 0) {
           fprintf(stderr, "Cannot connect to address %s, port %d: %s\n", hw.simple_link_ip, hw.simple_link_port, strerror(errno));
           exit(EXIT_FAILURE);
        }

        printf("Simple link set up\n");
     }

 
    if (pthread_create(&waker_tid, NULL, &waker_thread, (void*)&hw) < 0) {
        perror("could not create thread");
        return 1;
    }
     create_logger(&hw);
     create_spitimer(&hw);

     //start communication interface to allow client to connect
     //Create socket
     socket_desc = socket(AF_INET, SOCK_STREAM, 0);
     if (socket_desc == -1)
     {
         printf("Could not create socket");
     }
     puts("Socket created");

     //Prepare the sockaddr_in structure     
     server.sin_family = AF_INET;
     server.sin_addr.s_addr = INADDR_ANY;
     //server.sin_port = htons(8888);
     portno = atoi(argv[1]);
     server.sin_port = htons(portno);

     //Bind
     if (bind(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0)
     {
         //print the error message
         perror("bind failed. Error");
         return 1;
     }
     puts("bind done");

     //Listen
     listen(socket_desc, 3);
     hw.server_socket = socket_desc;

     //Accept and incoming connection
     puts("Waiting for incoming connections...");
     c = sizeof(struct sockaddr_in);

     /*get configuration messages and respond as requested. Multiple hosts are allowed.*/
     
    while ((client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)))
    {
        if (client_sock == -1 && errno == EINTR) continue;
        else {
           puts("Connection accepted");

           pthread_t sniffer_thread;
           hw.socket = malloc(sizeof(int));
           *hw.socket = client_sock;

           if (pthread_create(&sniffer_thread, NULL, connection_handler, (void*)&hw) < 0)
           {
               perror("could not create thread");
               return 1;
           }
           //Now join the thread , so that we dont terminate before the thread
           //pthread_join( sniffer_thread , NULL);
           puts("Handler assigned");
        }
    }

    //if (client_sock < 0)
    //{
    //    perror("accept failed");
    //    return 1;
   // }

   return 0;
}


