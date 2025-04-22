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

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include "myrpfake.h"


/* librp_Init ---------------------------------------------------------------------------------
*  Initializes all the memory access connections
*
*  Returns 0 if everything is fine.
*/
int librp_Init(librpSet *rpset)
{
   FILE *fr;
   double val;
   int i, j;

   rpset->fake_xres = 500;//0;
   rpset->fake_yres = 500;//0;
   rpset->fake_xreal = 100e-6;
   rpset->fake_yreal = 100e-6;

   rpset->fb_on = 0;

   printf("loading fake data\n");

   rpset->fake_z = (double **) malloc(rpset->fake_yres*sizeof(double *));
   rpset->fake_a = (double **) malloc(rpset->fake_yres*sizeof(double *));
   rpset->fake_b = (double **) malloc(rpset->fake_yres*sizeof(double *));
   rpset->fake_dz = (double **) malloc(rpset->fake_yres*sizeof(double *));

   for (j=0; j<rpset->fake_yres; j++) {
      rpset->fake_z[j] = (double *) malloc(rpset->fake_xres*sizeof(double));
      rpset->fake_a[j] = (double *) malloc(rpset->fake_xres*sizeof(double));
      rpset->fake_b[j] = (double *) malloc(rpset->fake_xres*sizeof(double));
      rpset->fake_dz[j] = (double *) malloc(rpset->fake_xres*sizeof(double));
   }

   //load all fake data
   fr = fopen("fakez.txt", "r");
   for (j=0; j<rpset->fake_yres; j++) {
       for (i=0; i<rpset->fake_xres; i++) {
           fscanf(fr, "%lf", &val);
           rpset->fake_z[i][j] = val;
       }
   }
   fclose(fr);

   fr = fopen("fakea.txt", "r");
   for (j=0; j<rpset->fake_yres; j++) {
       for (i=0; i<rpset->fake_xres; i++) {
           fscanf(fr, "%lf", &val);
           rpset->fake_a[i][j] = val;
       }
   }
   fclose(fr);

   fr = fopen("fakeb.txt", "r");
   for (j=0; j<rpset->fake_yres; j++) {
       for (i=0; i<rpset->fake_xres; i++) {
           fscanf(fr, "%lf", &val);
           rpset->fake_b[i][j] = val;
       }
   }
   fclose(fr);


   for (j=0; j<rpset->fake_yres; j++) {
       for (i=0; i<rpset->fake_xres; i++) {
           if (i==(rpset->fake_xres-1)) rpset->fake_dz[i][j] = 0;
           else rpset->fake_dz[i][j] = rpset->fake_z[i+1][j] - rpset->fake_z[i][j];
       }
   }
    
   printf("fake data loaded\n");

    return LIBRP_OK;
}

/* librp_Close ----------------------------------------------------------------------------------
*  Closes all the memory access connections.
* 
*  Returns 0 if everything is fine.
*/
int librp_Close(librpSet *rpset)
{
/*    munmap(rpset->cfg0, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg1, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg2, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg3, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg4, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg5, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg6, sysconf(_SC_PAGESIZE));
    close(rpset->memfd);
*/
    return LIBRP_OK;
}

static double
get_at(double **data, int xres, int yres, double xreal, double yreal, double x, double y)
{
    if (x<0 || y<0 || x>=xreal || y>=yreal) return 0;
    
    double dx, dy, dax, day;
    int ix, iy;

    dx = x*xres/xreal;
    dy = y*yres/yreal;
    ix = (int)dx;
    iy = (int)dy;
    dax = dx-ix;
    day = dy-iy;

    if (ix>=(xres-1) && iy>=(yres-1)) return data[xres-1][yres-1];
    else if (ix>=(xres-1)) return data[xres-1][iy];
    else if (iy>=(yres-1)) return data[ix][yres-1];
   
    return day*(1-dax)*data[ix][iy+1] + day*dax*data[ix+1][iy+1] +
                                (1-day)*(1-dax)*data[ix][iy] + (1-day)*dax*data[ix+1][iy];

}

static int
get_values(librpSet *rpset, double *err, double *z, double *in1, double *in2, double *dth, double *ax, double *a1, double *p1)
{
   double xat = rpset->x;
   double yat = rpset->y;
   double zat = rpset->z;

   if (rpset->fb_on == 0) {
      *z = zat;
      *in1 = 0.5;
      *in2 = 1.5; 
      *dth = 2.5;
      *err = 0;
      *ax = 1;
      *a1 = 100.0/(sqrt(100 + (rpset->freq - 45000)*(rpset->freq - 45000)));
      *p1 = atan((rpset->freq - 45000)/100) - M_PI/2;

   }
   else {
    
      *z = get_at(rpset->fake_z, rpset->fake_xres, rpset->fake_yres, rpset->fake_xreal, rpset->fake_xreal, xat, yat);
      *err = get_at(rpset->fake_dz, rpset->fake_xres, rpset->fake_yres, rpset->fake_xreal, rpset->fake_xreal, xat, yat);
      *in1 = get_at(rpset->fake_a, rpset->fake_xres, rpset->fake_yres, rpset->fake_xreal, rpset->fake_xreal, xat, yat);;
      *in2 = -get_at(rpset->fake_b, rpset->fake_xres, rpset->fake_yres, rpset->fake_xreal, rpset->fake_xreal, xat, yat);
      *dth = 3;
      *ax = -(*in2);
      *a1 = 0.1;
      *p1 = get_at(rpset->fake_b, rpset->fake_xres, rpset->fake_yres, rpset->fake_xreal, rpset->fake_xreal, xat, yat);

    //  printf("get values at %g %g : z %g\n", xat*1e6, yat*1e6, *z);
   }

   *z += (rand()/(double)RAND_MAX)*1e-9;
   *err += (rand()/(double)RAND_MAX)*1e-9;
   *in1 += (rand()/(double)RAND_MAX)*0.3;
   *in2 += (rand()/(double)RAND_MAX)*0.2;
   *ax += (rand()/(double)RAND_MAX)*0.1;
   *a1 += (rand()/(double)RAND_MAX)*0.1;
   *p1 += (rand()/(double)RAND_MAX)*0.1;

   //printf("internal %g %g\n", *a1, *p1);

   return 0;
}

int librp_FakeAt(librpSet *rpset, double x, double y)
{
   rpset->x = x;
   rpset->y = y;
   return 0;
}

/* librp_RedVals  ------------------------------------------------------------
*  Reads actual values of error signal (ADC1) and piezo output (DAC1).
*  
*  Parameters:
*  *error_signal: returns value of error signal
*  *zpiezo:       returns value of zpiezo (passed to dac1)
*  *adc1:         returns raw adc1 value
*  *adc2:         returns raw adc2 value
*  *dither:       returns dithering signal (passed to dac2)
*  *aux:          returns auxliary value (e.g. phase)
*  *a1:           returns amplitude 1
*  *p1:           returns phase 1
*
*  For sychronisation purposes, the signals should be better read from an independent
*  simultaneous sampling ADC.
*  Returns 0 if everything is fine.
*/
int librp_ReadVals(librpSet *rpset, double *error_signal, double *zpiezo, double *adc1, double *adc2, double *dither, double *aux, double *amplitude1, double *phase1) 
{
  double err, z, in1, in2, dth, ax, a1, p1;

  get_values(rpset, &err, &z, &in1, &in2, &dth, &ax, &a1, &p1);

  *error_signal = err;  
  *zpiezo = z; //((double)z)*1e-9 + 6e-6;   //roughly +-6000 to 0-12 um
  *adc1 = in1; 
  *adc2 = in2;
  *dither = dth;
  *aux = ax;
  *amplitude1 = a1;
  *phase1 = p1;

  return LIBRP_OK;
}

/* librp_ReadAVals  ----------------------------------------------------------
*  Reads actual values of error signal (ADC1) and piezo output (DAC1) with a simple averaging.
*
*  Parameters:
*  *error_signal: returns value of error signal
*  *zpiezo:       returns value of zpiezo (passed to dac1)
*  *adc1:         returns raw adc1 value
*  *adc2:         returns raw adc2 value
*  *dither:       returns dithering signal (passed to dac2)
*  *aux:          returns auxliary signal (e.g. phase)
*  nav:           number of values to average
*  usleeptime:    time in microseconds to wait between them

*  For sychronisation purposes, the signals should be better read from an independent
*  simultaneous sampling ADC.
*  Returns 0 if everything is fine.
*/
int librp_ReadAVals(librpSet *rpset, double *error_signal, double *zpiezo, double *adc1, double *adc2, double *dither, double *aux, double *amplitude1, double *phase1, int nav, int usleeptime) 
{
  int i;
  double err, z, in1, in2, dth, ax, a1, p1;
  double sumerr, sumz, sumin1, sumin2, sumdth, sumaux, suma1, sump1;

  sumerr = sumz = sumin1 = sumin2 = sumdth = sumaux = suma1 = sump1 = 0;

  for (i=0; i<nav; i++) 
  {
     get_values(rpset, &err, &z, &in1, &in2, &dth, &ax, &a1, &p1);

     sumin1 += in1;
     sumin2 += in2;
     sumz += z;
     sumdth += dth;
     sumerr += err;
     sumaux += ax;
     suma1 += a1;
     sump1 += p1;

     usleep(usleeptime);
  }

  *error_signal = sumerr/(double)nav;
  *zpiezo = sumz/(double)nav;
  *adc1 = sumin1/(double)nav;
  *adc2 = sumin2/(double)nav;
  *dither = sumdth/(double)nav;
  *aux = sumaux/(double)nav;  
  *amplitude1 = suma1/(double)nav;
  *phase1 = sump1/(double)nav;

  return LIBRP_OK;
}

/* librp_SetFeedback ------------------------------------------------------------------------
*  Set feedback loop on or off.
*
*  When the feedback loop is switched off, piezo is set to the last user supplied piezo value; 
*  if we want to stay at the same place, set this value before switching the feedback loop off.
*  Returns 0 if everything is fine.
*/
int librp_SetFeedback(librpSet *rpset, int on)
{
   if (on) rpset->settings |= 1;
   else rpset->settings &= ~1;

   rpset->fb_on = on;

  // *((uint32_t *)(rpset->cfg2 + 8)) = rpset->settings;

   return LIBRP_OK;
} 

/* librp_SetMode -----------------------------------------------------------------------------
*  Set feedback loop mode.
*
*  Parameters: 
*    mode:     
*    LIBRP_MODE_OFF           = 0,
*    LIBRP_MODE_PROPORTIONAL  = 1,
*    LIBRP_MODE_NCAMPLITUDE   = 2,
*    LIBRP_MODE_NCPHASE       = 3,
*    LIBRP_MODE_AKIYAMA       = 4,
*    LIBRP_MODE_PRSA          = 5,
*    LIBRP_MODE_STM           = 6,
*    LIBRP_MODE_KPFM          = 7
*
*  Returns 0 if everything is fine.
*/
int librp_SetMode(librpSet *rpset, int mode)
{
   int method, quantity, fm, invert;

   method = quantity = fm = invert = 0;

   if (mode == LIBRP_MODE_NCAMPLITUDE) {
      method = 1;
   }
   if (mode == LIBRP_MODE_NCPHASE) {
      method = 1;
      quantity = 1;
   }

   if (method) rpset->settings |= 1<<1;
   else rpset->settings &= ~(1<<1);

   if (quantity) rpset->settings |= 1<<2;
   else rpset->settings &= ~(1<<2);

   if (fm) rpset->settings |= 1<<3;
   else rpset->settings &= ~(1<<3);

   if (invert) rpset->settings |= 1<<4;
   else rpset->settings &= ~(1<<4);

//   *((uint32_t *)(rpset->cfg2 + 8)) = rpset->settings;

   return LIBRP_OK;
} 
/* librp_SetPin ------------------------------------------------------------------------------
*  Set particular output pin on (high) or off (low). 
*  Pins are numbered 0-15, representing first dio_n [0-7] and then dio_p [8-15] pins
*
*  Returns 0 if everything is fine.
*/
int librp_SetPin(librpSet *rpset, int pin, int on)
{
   if (on) rpset->settings |= 1<<(pin+15);
   else rpset->settings &= ~(1<<(pin+15));

 //  *((uint32_t *)(rpset->cfg2 + 8)) = rpset->settings;

   return LIBRP_OK;
} 

/* librp_SetSetpoint -------------------------------------------------------------------------
*  Set feedback loop setpoint value. The same quantitiy and range as reported in error signal
*  should be used, which for different feedback loops can have different meaning.
*  
*  Returns 0 if everything is fine.
*/
int librp_SetSetpoint(librpSet *rpset, int channel, double setpoint, int raw)
{
   int32_t value;

   value = (int32_t)setpoint;
   //printf("setting setpoint to %u\n", value);
 //  *((int32_t *)(rpset->cfg3)) = value;

   return LIBRP_OK;
} 

/* librp_SetPid  ---------------------------------------------------------------------
*  Sets PID feedback loop settings and parameters, including switching it on and off.
* 
*  Parameters:
*  channel:     unused
*  pid_p:       P term
*  pid_i:       I term
*  pid_d:       D term
* 
*  Now we only construct delay parameter from P term and pass it to the feedback loop.
*  Returns 0 if everything is fine.
* */
int librp_SetPid(librpSet *rpset, int channel, double pid_p, double pid_i, double pid_d)
{
   uint32_t value;
   value = (uint32_t)(30.0*pid_p); //0.03 means 1,  1 means 30, which is 11111 
   //printf("pid set to %u\n", value);
//   *((uint32_t *)(rpset->cfg3 + 8)) = value;
   return LIBRP_OK;
}

/* librp_SetFrequency --------------------------------------------------------------------------
*  Sets the dithering frequency, i.e. frequency that goes to DAC2.
*  Resolution is 1 Hz.
*
*  Parameters:
*  frequency:   desired frequency in Hz
*  Returns 0 if everything is fine.
*/
int librp_SetFrequency(librpSet *rpset, double frequency) 
{ 
   uint32_t value;
   value = (uint32_t)(frequency*4294967296.0/3.0e6/41.65); //empiric from acmaster

   rpset->freq = frequency;
   //printf("librp: frequency set to %g, meaning %u\n", frequency, value);

 //  *((uint32_t *)(rpset->cfg5)) = value;

   return LIBRP_OK;
}

/* librp_SetAmplitude -------------------------------------------------------------------------
*  Sets the amplitude of dithering signal, in the range of 0 to 1 V. Resolution is about 0.001 V.
* 
*  Parameters:
*  amplitude:   desired amplitude in V
*
*  Returns 0 if everything is fine.
*/
int librp_SetAmplitude(librpSet *rpset, double amplitude)
{
  //internally we supply the multiplication factor for  dds*factor/1024, which gives 1024 values
   uint32_t value;
   value = (uint32_t)(14*(1.0-amplitude)); //empiric from acmaster
   //printf("librp setting amplitude %g  this means bit shift %u\n", amplitude, value);

 //  *((uint32_t *)(rpset->cfg5 + 8)) = value;

   return LIBRP_OK;
}

/* librp_SetZpiezo -------------------------------------------------------------------------
*  Sets the zpiezo value used in case when feedback is off.
* 
*  Parameters:
*  zpiezo:   desired zpiezo value dac units
*  Returns 0 if everything is fine.
*/
int librp_SetZpiezo(librpSet *rpset, double zpiezo)
{
  //internally we supply the multiplication factor for  dds*factor/1024, which gives 1024 values
   int32_t value;
   value = zpiezo;//(uint32_t)((zpiezo-6e-6)*1e9); //roughly 0-12 um to +-6000 dac units

 //  *((int32_t *)(rpset->cfg6)) = value;

   rpset->z = zpiezo;

   return LIBRP_OK;
}




