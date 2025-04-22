
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


#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>

#include "cmirp.h"

#include <linux/spi/spidev.h>
#include <linux/types.h>

typedef unsigned char byte;

#define LIBRP_HIGH 1
#define LIBRP_LOW 0

#define SPI_SPEED 20000000 //1000000 // 1 MHz for testing; 50 MHz maximum speed

#define RP_DIO0_N 0
#define RP_DIO1_N 1
#define RP_DIO2_N 2
#define RP_DIO3_N 3
#define RP_DIO4_N 4
#define RP_DIO5_N 5
#define RP_DIO6_N 6
#define RP_DIO7_N 7
#define RP_DIO0_P 8
#define RP_DIO1_P 9
#define RP_DIO2_P 10
#define RP_DIO3_P 11
#define RP_DIO4_P 12
#define RP_DIO5_P 13
#define RP_DIO6_P 14
#define RP_DIO7_P 15 // now FPGA SPI CS

/* Constants definition */
// DAC 20b (DAC11001)
const byte DAC20BCONFIG1 = 0x02;     // config1 register
const byte DAC20BCONFIG2 = 0x06;     // config2 register
const byte DACDATA = 0x01;     // data register
const byte READ = 0b10000000;  // DAC11001's read command
const byte WRITE = 0b01111111; // DAC11001's write command
const unsigned long ZEROCODE = 0x000000; // max negative voltage
const unsigned long MIDCODE = 0x7FFFF0;
const unsigned long FULLCODE = 0xFFFFF0; // max positive voltage
const int LDAC1Pin = RP_DIO6_P; // Load DAC1
const int LDAC2Pin = RP_DIO6_N; // Load DAC2; now FPGA SPI CLK
const int LDAC3Pin = RP_DIO7_P; // Load DAC3; now FPGA SPI MOSI

// ADC 18b (ADS8598H)
const int ADCCONVSTPin = RP_DIO5_N; // ADS8598H conversion start pin

// 4-16 decoder (74HC4515)
const int DECA0Pin = RP_DIO0_P; // 4-16 decoder A0 pin
const int DECA1Pin = RP_DIO0_N; // 4-16 decoder A1 pin
const int DECA2Pin = RP_DIO1_P; // 4-16 decoder A2 pin
const int DECA3Pin = RP_DIO1_N; // 4-16 decoder A3 pin
const int DECLEPin = RP_DIO2_P; // 4-16 decoder LE pin

// DAC 16b (DAC81416)
const byte SPICONFIG = 0x03;     // SPI config register
const byte GENCONFIG = 0x04;     // SPI config register
const byte SYNCCONFIG = 0x06;     // SPI config register
const byte DACPWDWN = 0x09;     // SPI config register
const byte DACRANGE0 = 0x0A;     // DAC range 0 config register
const byte DACRANGE1 = 0x0B;     // DAC range 1 config register
const byte DACRANGE2 = 0x0C;     // DAC range 2 config register
const byte DACRANGE3 = 0x0D;     // DAC range 3 config register
//const byte TRIGGER = 0x0E;     // trigger config register
const byte DAC0 = 0x10;     // DAC0 data register
const byte DAC1 = 0x11;     // DAC1 data register
const byte DAC2 = 0x12;     // DAC2 data register
const byte DAC3 = 0x13;     // DAC3 data register
const byte DAC4 = 0x14;     // DAC4 data register
const byte DAC5 = 0x15;     // DAC5 data register
const byte DAC6 = 0x16;     // DAC6 data register
const byte DAC7 = 0x17;     // DAC7 data register
const byte DAC8 = 0x18;     // DAC8 data register
const byte DAC9 = 0x19;     // DAC9 data register
const byte DAC10 = 0x1A;     // DAC10 data register
const byte DAC11 = 0x1B;     // DAC11 data register
const byte DAC12 = 0x1C;     // DAC12 data register
const byte DAC13 = 0x1D;     // DAC13 data register
const byte DAC14 = 0x1E;     // DAC14 data register
const byte DAC15 = 0x1F;     // DAC15 data register
const byte READ1 = 0b10000000;  // DAC81416's read command
const byte WRITE1 = 0b01111111; // DAC81416's write command
const unsigned long ZEROCODE1 = 0x0000; // max negative voltage
const unsigned long MIDCODE1 = 0x8000; // zero voltage
const unsigned long FULLCODE1 = 0xFFFF; // max positive voltage
const int LDACPin = RP_DIO5_P; // Load DAC

// IO expander's memory register addresses:
const byte CONFIG2 = 0x04;     // configuration register
const byte PORTCONFIG1 = 0x09; // ports P7-P4 configuration register
const byte PORTCONFIG2 = 0x0A; // ports P11-P8 configuration register
const byte PORTCONFIG3 = 0x0B; // ports P15-P12 configuration register
const byte PORTCONFIG4 = 0x0C; // ports P19-P16 configuration register
const byte PORTCONFIG5 = 0x0D; // ports P23-P20 configuration register
const byte PORTCONFIG6 = 0x0E; // ports P27-P24 configuration register
const byte PORTCONFIG7 = 0x0F; // ports P31-P28 configuration register
const byte PORT4 = 0x24; // port P4 data register (mux enable ENA)
const byte PORT5 = 0x25; // port P5 data register (mux enable ENB)
const byte PORT6 = 0x26; // port P6 data register (ADC18b A&B reset)
const byte PORT7 = 0x27; // port P7 data register (DAC16b reset)
const byte PORTS4TO7 = 0x40; // ports P4 to P7 data register
const byte PORTS8TO15 = 0x48; // ports P8 to P15 data register (mux A&B addresses)
const byte PORT16 = 0x30; // port P16 data register (DAC11001 CLR1)
const byte PORTS16TO23 = 0x50; // ports P16 to P23 data registers
const byte PORT17 = 0x31; // port P17 data register (DAC11001 CLR2)
const byte PORT18 = 0x32; // port P18 data register (DAC11001 CLR3)
const byte PORT19 = 0x33; // port P19 data register (DAC11001 ALARM1)
const byte PORT20 = 0x34; // port P20 data register (DAC11001 ALARM2)
const byte PORT21 = 0x35; // port P21 data register (DAC11001 ALARM3)
const byte PORT22 = 0x36; // port P22 data register (MAX4649 SW_BUF1)
const byte PORT23 = 0x37; // port P23 data register (MAX4649 SW_BUF2)
const byte PORT24 = 0x38; // port P24 data register (ADS8598H A&B OS0)
const byte PORTS24TO31 = 0x58; // ports P24 to P31 data registers
const byte PORT25 = 0x39; // port P25 data register (ADS8598H A&B OS1)
const byte PORT26 = 0x3A; // port P26 data register (ADS8598H A&B OS2)
const byte PORT28 = 0x3C; // port P28 data register (STATUS LED)
const byte PORT29 = 0x3D; // port P29 data register (ERROR LED)
const byte PORT30 = 0x3E; // port P30 data register (RUN LED)
const byte READ2 = 0b10000000;  // MAX7301's read command
const byte WRITE2 = 0b01111111; // MAX7301's write command

/* Inline functions definition (common) */
static int init_spi(librpSet *rpset, uint8_t mode);
//static int change_spi_mode(librpSet *rpset, uint8_t mode);
static int release_spi(librpSet *rpset);
static int update_values(librpSet *rpset);
static int update_values_DAC16b(librpSet *rpset);
static int prepare_io(librpSet *rpset);
// DAC11001
static int configureDAC20b(int fd, byte thisRegister, unsigned long thisValue, int N);
static int writeDAC20b_XYZ(int fd, byte thisRegister1, unsigned long thisValue1,
                           byte thisRegister2, unsigned long thisValue2,
                           byte thisRegister3, unsigned long thisValue3);
static int writeDAC20b_XY(int fd, byte thisRegister1, unsigned long thisValue1,
                           byte thisRegister2, unsigned long thisValue2);

// ADS8598H
static int conversion(librpSet *rpset);
static int reset_ADC18b_AB(librpSet *rpset);
static int readADC18b(librpSet *rpset, double *voltage);

// DAC81416
static int configureDAC16b(librpSet *rpset);
static int writeDAC16b(librpSet *rpset, byte thisRegister, unsigned long thisValue);
static int reset_DAC16b(librpSet *rpset);

// MAX7301
//static int configureEXP(librpSet *rpset);
static int set_mux(librpSet *rpset, int muxA, int muxB);
static int set_oversampling_factor(librpSet *rpset, int OS);
static int set_led(librpSet *rpset, int led, int val);
static int set_input_range(librpSet *rpset);

// set 4-16 decoder
static int select_DAC20b(librpSet *rpset);
static int select_DAC16b(librpSet *rpset);
static int select_ADC18b_A(librpSet *rpset);
static int select_ADC18b_B(librpSet *rpset);
//static int select_EXP(librpSet *rpset);
static int librp_InitHRDAC(librpSet *rpset);

char * int2bin(int32_t i)
{
    size_t bits = sizeof(int32_t) * CHAR_BIT;

    char * str = malloc(bits + 1);
    if(!str) return NULL;
    str[bits] = 0;

    // type punning because signed shift is implementation-defined
    unsigned u = *(unsigned *)&i;
    for(; bits--; u >>= 1)
        str[bits] = u & 1 ? '1' : '0';

    return str;
}

char * short2bin(int16_t i)
{
    size_t bits = sizeof(int16_t) * CHAR_BIT;

    char * str = malloc(bits + 1);
    if(!str) return NULL;
    str[bits] = 0;

    // type punning because signed shift is implementation-defined
    unsigned u = *(unsigned *)&i;
    for(; bits--; u >>= 1)
        str[bits] = u & 1 ? '1' : '0';

    return str;
}

void copy_bits(uint32_t from, uint16_t *to, 
               uint32_t start, uint32_t end) 
{
    int mask;

    *to = 0;
    *to = from >> start;
    mask = (1 << (end-start)) - 1;
    *to = *to & mask;
} 

int librp_Init(librpSet *rpset, int hrdac_regime, int hrdac1_range, int hrdac2_range, int hrdac3_range,  
               int input_range1, int input_range2, int dds1_range, int dds2_range, int oversampling,
               int rp1_input_hv, int rp2_input_hv, int rp_bare_output, int rp_bare_input)
{

    if ((rpset->memfd = open("/dev/mem", O_RDWR)) < 0) {
        perror("open");
        return 1;
    }

    rpset->cfg0 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x42000000);
    rpset->cfg1 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41200000);
    rpset->cfg2 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41210000);
    rpset->cfg3 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41220000);
    rpset->cfg4 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41230000);
    rpset->cfg5 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41240000);
    rpset->cfg6 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41250000);
    rpset->cfg7 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41280000);
    rpset->cfg8 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41290000);
    rpset->cfg9 = mmap(NULL, sysconf(_SC_PAGESIZE),
                       PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412A0000);
    rpset->cfg10 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41260000);
    rpset->cfg11 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41270000);
    rpset->cfg12 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412B0000);
    rpset->cfg13 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412C0000);
    rpset->cfg14 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412D0000);
    rpset->cfg15 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412E0000);
    rpset->cfg16 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x412F0000);
    rpset->cfg17 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41300000);
    rpset->cfg18 = mmap(NULL, sysconf(_SC_PAGESIZE),
                        PROT_READ|PROT_WRITE, MAP_SHARED, rpset->memfd, 0x41310000);

    rpset->check = 0;

    rpset->debug = 0;
    rpset->pins = 0;
    rpset->pins2 = 0;
    rpset->dac1val = 0;
    rpset->dac2val = 0;
    rpset->hrdacval = 0; 
    rpset->spi_fd = -1;
    rpset->rp_bare_input = rp_bare_input;
    rpset->rp_bare_output = rp_bare_output;

    rpset->rpdac1_bare_slope = 8191.0; //2^13-1, factor of 10 was not there for bare RP and should be modified for this variant
    rpset->rpdac1_bare_offset = 0;

    rpset->rpdac1_slope = 8191.0/10.0; //2^13-1, factor of 10 was not there for bare RP and should be modified for this variant
    rpset->rpdac1_offset = 0;

    rpset->rpdac2_bare_slope = 8191.0; //2^13-1, factor of 10 was not there for bare RP and should be modified for this variant
    rpset->rpdac2_bare_offset = 0;

    rpset->rpdac2_slope = 8191.0/10.0; //2^13-1, factor of 10 was not there for bare RP and should be modified for this variant
    rpset->rpdac2_offset = 0;

    //init calibration values
    rpset->rpadc1_bare_lv_offset = 0;
    rpset->rpadc1_bare_lv_slope = 8191;
    rpset->rpadc1_bare_hv_offset = 0;
    rpset->rpadc1_bare_hv_slope = 410;
    
    rpset->rpadc2_bare_lv_offset = 0;
    rpset->rpadc2_bare_lv_slope = 8191;
    rpset->rpadc2_bare_hv_offset = 0;
    rpset->rpadc2_bare_hv_slope = 410;
    
    rpset->rpadc1_divhigh_lv_offset = 0;
    rpset->rpadc1_divhigh_lv_slope = 819;
    rpset->rpadc1_divhigh_hv_offset = 0;
    rpset->rpadc1_divhigh_hv_slope = 41;
    
    rpset->rpadc1_divlow_lv_offset = 0;
    rpset->rpadc1_divlow_lv_slope = 8191;
    rpset->rpadc1_divlow_hv_offset = 0;
    rpset->rpadc1_divlow_hv_slope = 410;
    
    rpset->rpadc2_divhigh_lv_offset = 0;
    rpset->rpadc2_divhigh_lv_slope = 819;
    rpset->rpadc2_divhigh_hv_offset = 0;
    rpset->rpadc2_divhigh_hv_slope = 41;
    
    rpset->rpadc2_divlow_lv_offset = 0;
    rpset->rpadc2_divlow_lv_slope = 8191;
    rpset->rpadc2_divlow_hv_offset = 0;
    rpset->rpadc2_divlow_hv_slope = 410;


    rpset->old_hr1 = -123; //unreal value, don't skip the first value to be set
    rpset->old_hr2 = -123; 
    rpset->old_hr3 = -123; 

    rpset->rpdacrule1 = LIBRP_RPDAC_DAC1;
    rpset->rpdacrule2 = LIBRP_RPDAC_DAC2;
    rpset->rpdacrulehr = LIBRP_RPDAC_DAC2;

    rpset->hrdac_regime = hrdac_regime;
    rpset->hrdac1_range = hrdac1_range;
    rpset->hrdac2_range = hrdac2_range;
    rpset->hrdac3_range = hrdac3_range;
    rpset->input_range1 = input_range1;
    rpset->input_range2 = input_range2;
    rpset->dds1_range = dds1_range;
    rpset->dds2_range = dds2_range;
    rpset->rp1_input_hv = rp1_input_hv;
    rpset->rp2_input_hv = rp2_input_hv;
    rpset->pll_input = 0;

    rpset->kpfm_mode = 0;
    rpset->dart_mode = 0;

    rpset->debugsource1 = 0;
    rpset->debugsource2 = 0;
    rpset->hr1 = 0;
    rpset->hr2 = 0;
    rpset->loopback1 = 0;
    rpset->loopback2 = 0;
    rpset->decimation1 = 0;
    rpset->decimation2 = 0;
    rpset->filter_phase1 = 0;
    rpset->filter_phase2 = 0;
    rpset->phaseshift1 = 0;
    rpset->phaseshift2 = 0;
    rpset->intosc1 = 0;
    rpset->intosc2 = 0;

    rpset->pidskip = 0;
    rpset->pllskip = 0;

    rpset->pidoffset = 0;   
 
    rpset->feedback = 0; 
    rpset->swap_in = 0;
    rpset->swap_out = 0;

    rpset->oversampling = oversampling;

    rpset->ddswidth = 36;//32 for default, 36 for wide
    rpset->freqfactor = 0.465/pow(2, rpset->ddswidth-30)/4.0;

    librp_Init_SPI_devices(rpset); //this includes range and oversampling set
    librp_SetInputRange(rpset, input_range1, input_range2);
    librp_SetState(rpset, LIBRP_MODE_OFF, rpset->feedback, rpset->rpdacrule1, rpset->rpdacrule2, rpset->rpdacrulehr, rpset->swap_in, rpset->swap_out, rpset->pidskip);

    return LIBRP_OK;

}

int librp_LoadCalData(librpSet *rpset, 
                     double rpadc1_bare_lv_offset, double rpadc1_bare_lv_slope, double rpadc1_bare_hv_offset, double rpadc1_bare_hv_slope, 
                     double rpadc2_bare_lv_offset, double rpadc2_bare_lv_slope, double rpadc2_bare_hv_offset, double rpadc2_bare_hv_slope,
                     double rpadc1_divhigh_lv_offset, double rpadc1_divhigh_lv_slope, double rpadc1_divhigh_hv_offset, double rpadc1_divhigh_hv_slope, 
                     double rpadc1_divlow_lv_offset, double rpadc1_divlow_lv_slope, double rpadc1_divlow_hv_offset, double rpadc1_divlow_hv_slope,
                     double rpadc2_divhigh_lv_offset, double rpadc2_divhigh_lv_slope, double rpadc2_divhigh_hv_offset, double rpadc2_divhigh_hv_slope, 
                     double rpadc2_divlow_lv_offset, double rpadc2_divlow_lv_slope, double rpadc2_divlow_hv_offset, double rpadc2_divlow_hv_slope,
                     double rpdac1_bare_offset, double rpdac1_bare_slope, double rpdac2_bare_offset, double rpdac2_bare_slope,
                     double rpdac1_offset, double rpdac1_slope, double rpdac2_offset, double rpdac2_slope)
{
    rpset->rpadc1_bare_lv_offset = rpadc1_bare_lv_offset;
    rpset->rpadc1_bare_lv_slope = rpadc1_bare_lv_slope;
    rpset->rpadc1_bare_hv_offset = rpadc1_bare_hv_offset;
    rpset->rpadc1_bare_hv_slope = rpadc1_bare_hv_slope;

    rpset->rpadc2_bare_lv_offset = rpadc2_bare_lv_offset;
    rpset->rpadc2_bare_lv_slope = rpadc2_bare_lv_slope;
    rpset->rpadc2_bare_hv_offset = rpadc2_bare_hv_offset;
    rpset->rpadc2_bare_hv_slope = rpadc2_bare_hv_slope;

    rpset->rpadc1_divhigh_lv_offset = rpadc1_divhigh_lv_offset;
    rpset->rpadc1_divhigh_lv_slope = rpadc1_divhigh_lv_slope;
    rpset->rpadc1_divhigh_hv_offset = rpadc1_divhigh_hv_offset;
    rpset->rpadc1_divhigh_hv_slope = rpadc1_divhigh_hv_slope;

    rpset->rpadc1_divlow_lv_offset = rpadc1_divlow_lv_offset;
    rpset->rpadc1_divlow_lv_slope = rpadc1_divlow_lv_slope;
    rpset->rpadc1_divlow_hv_offset = rpadc1_divlow_hv_offset;
    rpset->rpadc1_divlow_hv_slope = rpadc1_divlow_hv_slope;

    rpset->rpadc2_divhigh_lv_offset = rpadc2_divhigh_lv_offset;
    rpset->rpadc2_divhigh_lv_slope = rpadc2_divhigh_lv_slope;
    rpset->rpadc2_divhigh_hv_offset = rpadc2_divhigh_hv_offset;
    rpset->rpadc2_divhigh_hv_slope = rpadc2_divhigh_hv_slope;

    rpset->rpadc2_divlow_lv_offset = rpadc2_divlow_lv_offset;
    rpset->rpadc2_divlow_lv_slope = rpadc2_divlow_lv_slope;
    rpset->rpadc2_divlow_hv_offset = rpadc2_divlow_hv_offset;
    rpset->rpadc2_divlow_hv_slope = rpadc2_divlow_hv_slope;

    rpset->rpdac1_bare_offset = rpdac1_bare_offset;
    rpset->rpdac1_bare_slope = rpdac1_bare_slope;
    rpset->rpdac2_bare_offset = rpdac2_bare_offset;
    rpset->rpdac2_bare_slope = rpdac2_bare_slope;

    rpset->rpdac1_offset = rpdac1_offset;
    rpset->rpdac1_slope = rpdac1_slope;
    rpset->rpdac2_offset = rpdac2_offset;
    rpset->rpdac2_slope = rpdac2_slope;

    librp_SetInputRange(rpset, rpset->input_range1, rpset->input_range2);

    return LIBRP_OK;
}

int librp_SetInputRange(librpSet *rpset, int range1, int range2)
{
    rpset->input_range1 = range1;
    rpset->input_range2 = range2;

    if (!rpset->rp_bare_input) {
       if (rpset->input_range1 == 0){
           if (rpset->rp1_input_hv) {
               rpset->adc1cal = rpset->rpadc1_divlow_hv_slope;
               rpset->adc1shift = rpset->rpadc1_divlow_hv_offset;
           } else {
               rpset->adc1cal = rpset->rpadc1_divlow_lv_slope;
               rpset->adc1shift = rpset->rpadc1_divlow_lv_offset;
           }
        } else {
           if (rpset->rp1_input_hv) {
               rpset->adc1cal = rpset->rpadc1_divhigh_hv_slope;
               rpset->adc1shift = rpset->rpadc1_divhigh_hv_offset;
           } else {
               rpset->adc1cal = rpset->rpadc1_divhigh_lv_slope;
               rpset->adc1shift = rpset->rpadc1_divhigh_lv_offset;
           }
       }
       if (rpset->input_range2 == 0){
           if (rpset->rp2_input_hv) {
               rpset->adc2cal = rpset->rpadc2_divlow_hv_slope;
               rpset->adc2shift = rpset->rpadc2_divlow_hv_offset;
           } else {
               rpset->adc2cal = rpset->rpadc2_divlow_lv_slope;
               rpset->adc2shift = rpset->rpadc2_divlow_lv_offset;
           }
        } else {
           if (rpset->rp2_input_hv) {
               rpset->adc2cal = rpset->rpadc2_divhigh_hv_slope;
               rpset->adc2shift = rpset->rpadc2_divhigh_hv_offset;
           } else {
               rpset->adc2cal = rpset->rpadc2_divhigh_lv_slope;
               rpset->adc2shift = rpset->rpadc2_divhigh_lv_offset;
           }
       }
    } else {
        if (rpset->rp1_input_hv) {
            rpset->adc1cal = rpset->rpadc1_bare_hv_slope;
            rpset->adc1shift = rpset->rpadc1_bare_hv_offset;
        } else {
            rpset->adc1cal = rpset->rpadc1_bare_lv_slope;
            rpset->adc1shift = rpset->rpadc1_bare_lv_offset;
        }
        if (rpset->rp2_input_hv) {
            rpset->adc2cal = rpset->rpadc2_bare_hv_slope;
            rpset->adc2shift = rpset->rpadc2_bare_hv_offset;
        } else {
            rpset->adc2cal = rpset->rpadc2_bare_lv_slope;
            rpset->adc2shift = rpset->rpadc2_bare_lv_offset;
        }
     }

    if (rpset->rp_bare_output) {
       rpset->dac1cal = rpset->rpdac1_bare_slope;
       rpset->dac1shift = rpset->rpdac1_bare_offset;

       rpset->dac2cal = rpset->rpdac2_bare_slope;
       rpset->dac2shift = rpset->rpdac2_bare_offset;
    } else {
       rpset->dac1cal = rpset->rpdac1_slope;
       rpset->dac1shift = rpset->rpdac1_offset;

       rpset->dac2cal = rpset->rpdac2_slope;
       rpset->dac2shift = rpset->rpdac2_offset;
    }

 
    if (change_spi_mode(rpset, SPI_MODE_3) != LIBRP_OK){
        fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
        return -1;
    }

    if (select_EXP(rpset) != LIBRP_OK) {
        fprintf(stderr, "select_EXP failed!\n");
        return -1;
    }

    set_input_range(rpset);

    return LIBRP_OK;
}

int librp_SetDDSRange(librpSet *rpset, int range1, int range2)
{
    rpset->dds1_range = range1;
    rpset->dds2_range = range2;
    librp_SetState(rpset, rpset->state, rpset->feedback, rpset->rpdacrule1, rpset->rpdacrule2, rpset->rpdacrulehr, rpset->swap_in, rpset->swap_out, rpset->pidskip);

    return LIBRP_OK;
}

int librp_SetOversampling(librpSet *rpset, int oversampling)
{
    rpset->oversampling = oversampling;

    if (change_spi_mode(rpset, SPI_MODE_3) != LIBRP_OK){
        fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
        return -1;
    }   
    
    if (select_EXP(rpset) != LIBRP_OK) {
        fprintf(stderr, "select_EXP failed!\n");
        return -1;
    }

    set_oversampling_factor(rpset, oversampling);

    return LIBRP_OK;
}


int librp_Init_SPI_devices(librpSet *rpset)
{
    int i, ret;
    double hrdac1, hrdac2, hrdac3;

    if (rpset->hrdac_regime != LIBRP_HRDAC_REGIME_CPU) 
    {
       librp_SetHRDAC1(rpset, 0.0);
       librp_SetHRDAC2(rpset, 0.0);
       librp_SetHRDAC3(rpset, 0.0);

       librp_InitHRDAC(rpset);

       librp_SetHRDAC1(rpset, 0.0);
       librp_SetHRDAC2(rpset, 0.0);
       librp_SetHRDAC3(rpset, 0.0);
    }
    

    for (i=0; i<16; i++){
        if (librp_DPinSetState(rpset, i, LIBRP_LOW) != LIBRP_OK){
            fprintf(stderr, "librp_DPinSetState failed!\n");
            return -1;
        }
    }

    for (i=0; i<16; i++){
        if (librp_D2PinSetState(rpset, i, LIBRP_LOW) != LIBRP_OK){
            fprintf(stderr, "librp_D2PinSetState failed!\n");
            return -1;
        }
    }


    if (prepare_io(rpset) != LIBRP_OK){
        fprintf(stderr, "prepare_io failed.\n");
        return -1;
    }

    ret = init_spi(rpset, SPI_MODE_3);
    if (ret == LIBRP_SPI_ALREADY_INIT) printf("SPI was already init\n");
    else if (ret != LIBRP_OK){
        fprintf(stderr, "Initialization of SPI failed. Error: %s\n", strerror(errno));
        return -1;
    }

    if (change_spi_mode(rpset, SPI_MODE_3) != LIBRP_OK){
        fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
        return -1;
    }

    if (select_EXP(rpset) != LIBRP_OK) {
        fprintf(stderr, "select_EXP failed!\n");
        return -1;
    }

    if (configureEXP(rpset) != LIBRP_OK){
        fprintf(stderr, "configureEXP failed.\n");
        return -1;
    }
    if (set_input_range(rpset) != LIBRP_OK){
        fprintf(stderr, "set_input_range failed.\n");
        return -1;
    }
    if (reset_ADC18b_AB(rpset) != LIBRP_OK){ // done via IO expander!
        fprintf(stderr, "reset_ADC18b_AB failed.\n");
        return -1;
    }
    if (set_oversampling_factor(rpset, rpset->oversampling) != LIBRP_OK){ // done via IO expander!
        fprintf(stderr, "set_oversampling_factor failed.\n");
        return -1;
    }
    if (reset_DAC16b(rpset) != LIBRP_OK){ // done via IO expander!; DAC must be configured after reset
        fprintf(stderr, "reset_DAC16b failed.\n");
        return -1;
    }
    usleep(1e3); // wait for 1 ms after the Power-on-Reset event

    if (rpset->hrdac_regime == LIBRP_HRDAC_REGIME_CPU)
    {

       if (change_spi_mode(rpset, SPI_MODE_2) != LIBRP_OK){
           fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
           return -1;
       }

       if (select_DAC20b(rpset) != LIBRP_OK) {
           fprintf(stderr, "select_DAC20b failed!\n");
           return -1;
       }

       if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG2, 0x000040, 3) != LIBRP_OK){ // default (UP_RATE 0.5 MHz with 21 MHz SCLK)
	       fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
	       return -1;
       }
/*
       if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG2, 0x000060, 3) != LIBRP_OK){ // UP_RATE 0.4 MHz with 16 MHz SCLK
	       fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
	       return -1;
       }
*/
       //printf("configure DAC (20b)\n");
       // Configure DAC: Select VREF (+5 V ~ -5V)
       // TnH mode (Good THD), LDAC mode and power-up the DAC
       if (rpset->hrdac_range == LIBRP_RANGE_FULL) {
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x000D00, 3) != LIBRP_OK){ //only all channels now allowed, output +-10 V 
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
       } else {
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x000C80, 3) != LIBRP_OK){ //only all channels now allowed, output 0-10 V  
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
       }
/*
       if (rpset->hrdac_range == LIBRP_RANGE_FULL) {
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x000D00, 3) != LIBRP_OK){ //only all channels now allowed, output +-10 V 
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x002900, 3) != LIBRP_OK){ //only all channels now allowed, output +-10 V ; TnH > 2^14 (default); FSDO = 0
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x042900, 3) != LIBRP_OK){ //only all channels now allowed, output +-10 V ; TnH > 2^15; FSDO = 0
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x0C2900, 3) != LIBRP_OK){ //only all channels now allowed, output +-10 V ; TnH > 2^12; FSDO = 0
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
       } else {
          if (configureDAC20b(rpset->spi_fd, DAC20BCONFIG1, 0x000C80, 3) != LIBRP_OK){ //only all channels now allowed, output 0-10 V  
              fprintf(stderr, "configureDAC20b failed : %s\n", strerror(errno));
              return -1;
          }
       }
*/
    }
    if (change_spi_mode(rpset, SPI_MODE_2) != LIBRP_OK){
        fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
        return -1;
    }

    if (select_DAC16b(rpset) != LIBRP_OK) {
        fprintf(stderr, "select_DAC16b failed!\n");
        return -1;
    }

    if (configureDAC16b(rpset) != LIBRP_OK){
        fprintf(stderr, "configureDAC failed.\n");
        return -1;
    }

    return LIBRP_OK;
}

int librp_Close(librpSet *rpset)
{
    munmap(rpset->cfg0, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg1, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg2, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg3, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg4, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg5, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg6, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg8, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg9, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg10, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg11, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg12, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg13, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg14, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg15, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg16, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg17, sysconf(_SC_PAGESIZE));
    munmap(rpset->cfg18, sysconf(_SC_PAGESIZE));
    close(rpset->memfd);

    if (release_spi(rpset) != LIBRP_OK){
        fprintf(stderr, "Relase of SPI resources failed, Error: %s\n", strerror(errno));
        return -1;
    }

    return LIBRP_OK;
}

int librp_SetPhaseShift(librpSet *rpset, int channel, int shift)
{
    if (channel == LIBRP_CH_1) rpset->phaseshift1 = shift;
    else rpset->phaseshift2 = shift;

    //printf("phaseshift set: %d %d\n", rpset->phaseshift1, rpset->phaseshift2);

    return LIBRP_OK;
}

static inline double raw_adc_to_real(librpSet *rpset, int val, int channel)
{
    if (channel == LIBRP_CH_1) return ((double)val + rpset->adc1shift)/rpset->adc1cal;
    else return ((double)val + rpset->adc2shift)/rpset->adc2cal;
}

static inline double raw_amplitude_to_real(librpSet *rpset, int val, int channel)
{
    double ampdiv = 1480.0;
    if (channel == LIBRP_CH_1) {
       if (rpset->hr1) ampdiv *= 64;
       if (rpset->input_range1 == 0) ampdiv *= 10.0;
    }
    else {
       if (rpset->hr2) ampdiv *= 64*8;
       if (rpset->input_range2 == 0) ampdiv *= 10.0;
    }
    return val/ampdiv;
}    
static inline double real_amplitude_to_raw(librpSet *rpset, double val, int channel)
{
    double ampdiv = 1480.0;
    if (channel == LIBRP_CH_1) {
       if (rpset->hr1) ampdiv *= 64;
       if (rpset->input_range1 == 0) ampdiv *= 10.0;
    }
    else {
       if (rpset->hr2) ampdiv *= 64*8;
       if (rpset->input_range2 == 0) ampdiv *= 10.0;
    }
    return val*ampdiv;
} 

int librp_ReadADC(librpSet *rpset, double *adc1, double *adc2)
{
    uint32_t data;
    int16_t part1;
    int16_t part2;

    data = *((uint32_t *)(rpset->cfg2));

    copy_bits(data, &part1, 0, 16);
    copy_bits(data, &part2, 16, 32);

    *adc1 = raw_adc_to_real(rpset, part1, LIBRP_CH_1); 
    *adc2 = raw_adc_to_real(rpset, part2, LIBRP_CH_2); 

    //printf("raw adc: %d  %d   val %g %g\n",  part1, part2, *adc1, *adc2);//int2bin(part1));

    return LIBRP_OK;
}

int librp_ReadResults(librpSet *rpset, double *error, double *dac, int raw)
{
    int state;
    double ddac, derr;

    state = rpset->state;

    ddac = (double)(*((int32_t *)(rpset->cfg1)));
    derr = (double)(*((int32_t *)(rpset->cfg1+8)));
    
    //printf("raw dacr, error %g %g\n", dac, derr);

    if (raw) {
        *error = derr;
        *dac = ddac;
    } else {
        if (state == LIBRP_MODE_IN1) *dac = (ddac - rpset->dac1shift)/rpset->dac1cal;
        else if (state == LIBRP_MODE_IN2) *dac = (ddac - rpset->dac2shift)/rpset->dac2cal;

        if (state==LIBRP_MODE_A1) *error = raw_amplitude_to_real(rpset, derr, LIBRP_CH_1); 
        else if (state==LIBRP_MODE_A2) *error = raw_amplitude_to_real(rpset, derr, LIBRP_CH_2); 
        else if (state==LIBRP_MODE_P1 || state==LIBRP_MODE_P2) *error = 0.0009765625*derr;  //8.0*65536.0/536870912.0; //2^29
        else if (state==LIBRP_MODE_FM) *error = derr*rpset->freqfactor; //conversion from generator units to Hertz, 4/34, FIXME /4.0 should not be there
        else {
           if (state == LIBRP_MODE_IN1) *error = raw_adc_to_real(rpset, derr, LIBRP_CH_1); 
           else *error = raw_adc_to_real(rpset, derr, LIBRP_CH_2);                         
        }
    }

    return LIBRP_OK;
}

int librp_ReadLockin(librpSet *rpset, int channel, double *amplitude, double *phase)
{
    int32_t iamplitude, iphase;

    if (channel == LIBRP_CH_1) {
        iamplitude     = *((int32_t *)(rpset->cfg0));
        iphase         = *((int32_t *)(rpset->cfg0 + 8));
    } else {
        iamplitude     = *((int32_t *)(rpset->cfg3));
        iphase         = *((int32_t *)(rpset->cfg3 + 8));
    }

    *amplitude = raw_amplitude_to_real(rpset, iamplitude, channel);
    *phase = ((double) iphase)*1.86264515e-9; // 1.0/536870912.0; //2^29

    return LIBRP_OK;
}

int librp_ReadLockinAxes(librpSet *rpset, int channel, double *sine, double *cosine)
{
    int32_t isine, icosine;

    if (channel == LIBRP_CH_1) {
        isine     = *((int32_t *)(rpset->cfg18));
        icosine   = *((int32_t *)(rpset->cfg18 + 8));
    } else {
        isine     = *((int32_t *)(rpset->cfg4 + 8));  //connected as debug2
        icosine   = *((int32_t *)(rpset->cfg16 + 8)); //connected as debug3
    }
    *sine = raw_amplitude_to_real(rpset, (double)isine, channel)/8.0; //bitshift factor in evaluator.v
    *cosine = raw_amplitude_to_real(rpset, (double)icosine, channel)/8.0;

    return LIBRP_OK;
}


int librp_ReadPLLResult(librpSet *rpset, double *pllresult, double *ampresult)
{
    int32_t value;

    value = *((int32_t *)(rpset->cfg4));
//    *pllresult = (double)value;
    *pllresult = rpset->freqfactor*(double)value;

    value = *((int32_t *)(rpset->cfg14+8));
//    *ampresult = (double)value;

    if (!rpset->pll_input) *ampresult = (double)(value)/rpset->dac1cal;
    else *ampresult = (double)(value)/rpset->dac2cal;

    return LIBRP_OK;
}

int librp_ReadHRDAC(librpSet *rpset, double *dac)
{
    int32_t value, svalue;

    value = *((int32_t *)(rpset->cfg2 + 8));
    //value &= ~(1 << 24);  //now the zslow signal is no more mixed with mask, so we don't need this
    svalue = value >> 4;

//    *dac = (double)svalue;

    if (rpset->hrdac3_range == LIBRP_RANGE_FULL) *dac = 20.0*((double)svalue)/1048576 - 10.0;
    else *dac = 10.0*((double)svalue)/1048576;

    //printf("readhrdac: %d %d -> %g\n", value, svalue, *dac);

    return LIBRP_OK;
}

int librp_ReadDebug(librpSet *rpset, double *debug, double *debug3)
{
    int32_t value;

    value = *((int32_t *)(rpset->cfg4 + 8));
    *debug = (double)value;

    value = *((int32_t *)(rpset->cfg16 + 8));
    *debug3 = (double)value;

    return LIBRP_OK;
}

int librp_SetGen(librpSet *rpset, int channel, double frequency, double amplitude, double offset)
{
    uint32_t amp, value = 0;
    int32_t increment;
    double systemclock, phasewidth;
    int32_t ioffset;

    systemclock = 125e6; //dds synthetiser system clock. This is strange, why it is not the 3 MHz set in the dds synthetiser dialog? Probably the dialog value of 3 MHz does not mean anything and clock has to come externally?
    phasewidth = rpset->ddswidth;
    increment = (int32_t)((double)frequency*pow(2, phasewidth)/systemclock);

    //printf("ff %g\n", pow(2, phasewidth)/systemclock);

    //printf("f %g  increment is %u\n", frequency, increment);
    //if (channel==1) 
    //printf("set freq: ch %d  f %g a %g o %g    i %d\n", channel, frequency, amplitude, offset, increment);
 
    if (amplitude>10.0) amplitude = 10;
    if ((channel==1 && rpset->dds1_range==LIBRP_RANGE_SMALL) || (channel==2 && rpset->dds2_range==LIBRP_RANGE_SMALL)) {
       if (amplitude>5) amplitude = 5;
    }
  
    if (channel == LIBRP_CH_1) {  
       amp = (uint32_t)(amplitude*rpset->dac1cal); //10 V is converted to 8191
       ioffset = (int32_t)(offset*rpset->dac1cal + rpset->dac1shift);
    } else {
       amp = (uint32_t)(amplitude*rpset->dac2cal); //10 V is converted to 8191
       ioffset = (int32_t)(offset*rpset->dac2cal + rpset->dac2shift);
    } 
    value = amp;
    value |= (ioffset+30000) << 16;
 
    //printf("f%d increment %d amp %d ioffset %d  daccals %g %g\n", channel, increment, amp, ioffset, rpset->dac1cal, rpset->dac2cal);
    //printf("incre  : %s\n", int2bin(increment)); 
    //printf("value  : %s\n", int2bin(value)); 

    if (channel==1)
    {
        *((int32_t *)(rpset->cfg11)) = increment;
        *((uint32_t *)(rpset->cfg15)) = value;
        rpset->frequency1 = frequency;
        rpset->amplitude1 = amplitude;
        rpset->offset1 = offset;
        rpset->increment1 = increment;
    }
    else  
    {
        *((int32_t *)(rpset->cfg11+8)) = increment;
        *((uint32_t *)(rpset->cfg15+8)) = value;
        rpset->frequency2 = frequency;
        rpset->amplitude2 = amplitude;
        rpset->offset2 = offset;
        rpset->increment2 = increment;
    }

    return LIBRP_OK;
}

int librp_SetGenByIncrement(librpSet *rpset, int channel, int increment, double amplitude, double offset)
{
    uint32_t amp, value = 0;
    double systemclock, phasewidth;
    int32_t ioffset;

    systemclock = 125e6; //dds synthetiser system clock. This is strange, why it is not the 3 MHz set in the dds synthetiser dialog? Probably the dialog value of 3 MHz does not mean anything and clock has to come externally?
    phasewidth = rpset->ddswidth;

    if (amplitude>10.0) amplitude = 10;
    if ((channel==1 && rpset->dds1_range==LIBRP_RANGE_SMALL) || (channel==2 && rpset->dds2_range==LIBRP_RANGE_SMALL)) {
       if (amplitude>5) amplitude = 5;
    }
  
    if (channel == LIBRP_CH_1) {  
       amp = (uint32_t)(amplitude*rpset->dac1cal); //10 V is converted to 8191
       ioffset = (int32_t)(offset*rpset->dac1cal + rpset->dac1shift);
    } else {
       amp = (uint32_t)(amplitude*rpset->dac2cal); //10 V is converted to 8191
       ioffset = (int32_t)(offset*rpset->dac2cal + rpset->dac2shift);
    } 
    value = amp;
    value |= (ioffset+30000) << 16;

    if (channel==1)
    {
        *((int32_t *)(rpset->cfg11)) = increment;
        *((uint32_t *)(rpset->cfg15)) = value;
        rpset->frequency1 = ((double)increment)/(pow(2, phasewidth)/systemclock);
        rpset->amplitude1 = amplitude;
        rpset->offset1 = offset;
        rpset->increment1 = increment;
    }
    else  
    {
        *((int32_t *)(rpset->cfg11+8)) = increment;
        *((uint32_t *)(rpset->cfg15+8)) = value;
        rpset->frequency2 = ((double)increment)/(pow(2, phasewidth)/systemclock);
        rpset->amplitude2 = amplitude;
        rpset->offset2 = offset;
        rpset->increment2 = increment;
    }

    return LIBRP_OK;
}

int librp_SetGenIncrement(librpSet *rpset, int channel, int increment)
{
    //printf("set increment: ch %d  i %d\n", channel, increment);
    if (channel==1)
    {
        *((int32_t *)(rpset->cfg11)) = increment;
        rpset->increment1 = increment;
    }
    else  
    {
        *((int32_t *)(rpset->cfg11+8)) = increment;
        rpset->increment2 = increment;
    }

    return LIBRP_OK;
}

int librp_SetAuxGen(librpSet *rpset, double frequency)
{
    int32_t increment;
    double systemclock, phasewidth;

    systemclock = 125e6; //dds synthetiser system clock. This is strange, why it is not the 3 MHz set in the dds synthetiser dialog? Probably the dialog value of 3 MHz does not mean anything and clock has to come externally?
    phasewidth = rpset->ddswidth;
    increment = (int32_t)((double)frequency*pow(2, phasewidth)/systemclock);

    //printf("f3 increment %d for frequency %g\n", increment, frequency);
    rpset->increment3 = increment;   

    *((int32_t *)(rpset->cfg16)) = increment;

    return LIBRP_OK;

}

int librp_SetAuxGenIncrement(librpSet *rpset, int increment)
{
    //printf("f3 increment %d\n", increment);
    rpset->increment3 = increment;   

    *((int32_t *)(rpset->cfg16)) = increment;

    return LIBRP_OK;
}

int librp_SetPLL(librpSet *rpset, int on, int aon, double phase, double amplitude, double p, double i, double d, double ap, double ai, double ad, int phase_limit, int frequency_limit, int pllskip, int pll_input)
{
    double divfactor = 8.0*65536.0/536870912.0;
    uint32_t valuea = 0, valueb = 0, valued = 0, vamplitude = 0;
    int32_t phasevalue, valuec = 0;
    double kp, ki, kd;
    int32_t pidp, pidi, pidd, pidap, pidai, pidad;
    int amskip = 2;

    rpset->pllskip = pllskip; 
    rpset->pll_input = pll_input; 
 
    if (rpset->debug) printf("CMIRP: Set FM to on %d %d  phase sp %g pids (%g %g %g) (%g %g %g) limits %d %d\n", on, aon, phase, p, i, d, ap, ai, ad, phase_limit, frequency_limit);
 
    if (p<0) p = 0;
    if (p>1.0) p = 1.0;
    
    if (i<0) i = 0;
    if (i>1.0) i = 1.0;
    
    if (d<0) d = 0;
    if (d>1.0) d = 1.0;
    
    kp = p*10000;
    ki = i*10000;
    kd = d*10000;
    
    pidp = kp;
    pidi = ki;
    pidd = kd;

    if (ap<0) ap = 0;
    if (ap>1.0) ap = 1.0;
    
    if (ai<0) ai = 0;
    if (ai>1.0) ai = 1.0;
    
    if (ad<0) ad = 0;
    if (ad>1.0) ad = 1.0;
   
 
    kp = ap*10000;
    ki = ai*10000;
    kd = ad*10000;

    pidap = kp;
    pidai = ki;
    pidad = 0;
   
    valuea |= pidp + 30000;
    valuea |= (pidi + 30000) << 16;
    valueb |= pidd + 30000;
    valueb |= (pidad + 30000) << 16;
    valued |= pidap + 30000;
    valued |= (pidai + 30000) << 16;

    if (phase_limit<0) phase_limit = 0;
    if (phase_limit>7) phase_limit = 7;

    phasevalue = (int32_t)(phase/divfactor) + 10000;
    valuec = phasevalue << 16;
    if (on) valuec |= 1;
    if (aon) valuec |= 1 << 1;
    valuec |= phase_limit << 4;
    valuec |= frequency_limit << 7;
    valuec |= pllskip << 10;
    if (pll_input) valuec |= 1 << 12;
    valuec |= amskip << 13;
    
    //printf("# set fm value: (%s)  (%s)  (%s) fm pid  %d %d %d  am pid %d %d %d  phasevalue %d\n", int2bin(valuea), int2bin(valueb), int2bin(valuec), pidp, pidi, pidd, pidap, pidai, pidad, phasevalue);

    *((uint32_t *)(rpset->cfg13)) = valuea;
    *((uint32_t *)(rpset->cfg13 + 8)) = valueb;  
    *((int32_t *)(rpset->cfg8)) = valuec;
    *((uint32_t *)(rpset->cfg14)) = valued;

    if (!pll_input) vamplitude = (int32_t)(real_amplitude_to_raw(rpset, amplitude, LIBRP_CH_1));
    else vamplitude = (int32_t)(real_amplitude_to_raw(rpset, amplitude, LIBRP_CH_2));

    *((uint32_t *)(rpset->cfg12)) = vamplitude; 

    return LIBRP_OK;
}

//filter, loopback, lockin parameters
int librp_SetSignalProcessing(librpSet *rpset, int channel, int decimation, int loopback, int hr, int phaseshift, int debugsource, int filter_amplitude, int filter_phase, int nwaves, int lfr, int intosc)
{
    int on, dec, phase, nwav;
    uint32_t cleaner_settings = 0;
    
    if (lfr) rpset->ddswidth = 36;
    else rpset->ddswidth = 32;
    rpset->freqfactor = 0.465/pow(2, rpset->ddswidth-30)/4.0;

    if (decimation == 0) on = 0;
    else on = 1;

    phase = phaseshift;
    if (phase > 3) phase = 3;
    if (phase < 0) phase = 0;

    nwav = nwaves;
    if (nwaves > 7) nwaves = 7;
    if (nwaves < 0) nwaves = 0;

    if (on) cleaner_settings |= 1<<0;
    else cleaner_settings &= ~(1<<0);

    if (loopback) cleaner_settings |= 1<<1;
    else cleaner_settings &= ~(1<<1);

    if (hr) cleaner_settings |= 1<<2;
    else cleaner_settings &= ~(1<<2);

    //printf("signal processing: channel %d: l filters %d %d\n", channel, filter_amplitude, filter_phase);

    if (debugsource) cleaner_settings |= 1<<3;
    else cleaner_settings &= ~(1<<3);

    if (phaseshift) cleaner_settings |= phase<<4;

    if (filter_phase) cleaner_settings |= 1<<6;
    else cleaner_settings &= ~(1<<6);

    if (filter_amplitude) cleaner_settings |= 1<<7;
    else cleaner_settings &= ~(1<<7);

    dec = decimation;
    if (dec>=255) dec = 255;
    cleaner_settings |= dec << 8;

    if (intosc) cleaner_settings |= 1<<16;
    else cleaner_settings &= ~(1<<16);

    if (nwav) cleaner_settings |= nwav<<17;
    if (lfr) cleaner_settings |= 1<<20;

    if (channel==1) {
        *((uint32_t *)(rpset->cfg7)) = cleaner_settings;
        rpset->phaseshift1 = phase;
        rpset->debugsource1 = debugsource;
        rpset->hr1 = hr;
        rpset->loopback1 = loopback;
        rpset->decimation1 = decimation;
        rpset->filter_phase1 = filter_phase;
        rpset->nwaves1 = nwaves;
    }
    else { 
        *((uint32_t *)(rpset->cfg7 + 8)) = cleaner_settings;
        rpset->phaseshift2 = phase;
        rpset->debugsource2 = debugsource;
        rpset->hr2 = hr;
        rpset->loopback2 = loopback;
        rpset->decimation2 = decimation;
        rpset->filter_phase2 = filter_phase;
        rpset->nwaves2 = nwaves;
    }

    return LIBRP_OK;
}


int librp_DPinSetState(librpSet *rpset, int pin, int on)
{
    if (on) rpset->pins |= 1<<(pin+16);
    else rpset->pins &= ~(1<<(pin+16));

    //if (mprotect(rpset->cfg9, 32, PROT_READ | PROT_WRITE)) {
    //    perror("Couldn’t un-mprotect");
    //    exit(errno);
    //} 

    *((uint32_t *)(rpset->cfg9)) = rpset->pins;

    //if (mprotect(rpset->cfg9, 32, PROT_READ)) {
    //    perror("Couldn’t mprotect");
    //    exit(errno);
    //} 

    return LIBRP_OK;
}

int librp_D2PinSetState(librpSet *rpset, int pin, int on)
{
    if (on) rpset->pins2 |= 1<<(pin);
    else rpset->pins2 &= ~(1<<(pin));

    //if (mprotect(rpset->cfg9 + 8, 32, PROT_READ | PROT_WRITE)) {
    //    perror("Couldn’t un-mprotect");
    //    exit(errno);
    //} 

    *((uint32_t *)(rpset->cfg9 + 8)) = rpset->pins2;

    //if (mprotect(rpset->cfg9 + 8, 32, PROT_READ)) {
    //    perror("Couldn’t mprotect");
    //    exit(errno);
    //} 

    return LIBRP_OK;
}

int librp_DMultiPinSetState(librpSet *rpset, int npin, ...)
{
    int i, pin, on;
    va_list list;

    va_start(list, npin);
    //for (i=0; i<npin; i+=2) 
    for (i=0; i<npin; i++) 
    {
       pin = va_arg(list, int);
       on = va_arg(list, int);

       if (on) rpset->pins |= 1<<(pin+16);
       else rpset->pins &= ~(1<<(pin+16));
    }

    //if (mprotect(rpset->cfg9, 32, PROT_READ | PROT_WRITE)) {
    //    perror("Couldn’t un-mprotect");
    //    exit(errno);
    //} 

    *((uint32_t *)(rpset->cfg9)) = rpset->pins;

    //if (mprotect(rpset->cfg9, 32, PROT_READ)) {
    //    perror("Couldn’t mprotect");
    //    exit(errno);
    //} 


    va_end(list);

    return LIBRP_OK;
}

int librp_SetFeedback(librpSet *rpset, int feedback)
{
    librp_SetState(rpset, rpset->state, feedback, rpset->rpdacrule1, rpset->rpdacrule2, rpset->rpdacrulehr, rpset->swap_in, rpset->swap_out, rpset->pidskip);
    return LIBRP_OK;
}

int librp_SetState(librpSet *rpset, int state, int feedback, int out1, int out2, int outhr, int swap_in, int swap_out, int pidskip)
{
    uint32_t value = 0;

    if (rpset->debug) printf("CMIRP: Set state to %d  feedback %d\n", state, feedback);

    rpset->rpdacrule1 = out1; 
    rpset->rpdacrule2 = out2;
    rpset->rpdacrulehr = outhr; 
    rpset->state = state;
    rpset->feedback = feedback;
    rpset->swap_in = swap_in;
    rpset->swap_out = swap_out;
    rpset->pidskip = pidskip;
 
    //printf("# set state %d swap settings: %d %d  feedback %d\n", state, swap_in, swap_out, feedback);
    //printf("# dac rules: %d %d %d\n", rpset->rpdacrule1, rpset->rpdacrule2, rpset->rpdacrulehr); 
 
    value |= state;                    //9 bits representing state
    if (feedback) value |= 1<<9;       //feedback on/off
    if (swap_in) value |= 1<<10;        //swap input in channel 1
    if (swap_out) value |= 1<<11;      //swap pid result in channel 1

    value |= rpset->rpdacrule1<<14;    //routing to rp dac1 
    value |= rpset->rpdacrule2<<17;    //routing to rp dac2
    value |= rpset->rpdacrulehr<<20;   //routing to spi 20bit hrdac_zpiezo

    if (rpset->dds1_range==1) value |= 1<<23;
    if (rpset->dds2_range==1) value |= 1<<24;

    value |= rpset->pidskip<<25;   
 
    //printf("# modes %d value: %s\n", state, int2bin(value));

    *((uint32_t *)(rpset->cfg10)) = value;

    return LIBRP_OK;
}

int librp_SetKPFM(librpSet *rpset, int mode, double frequency, double frequency_generate_aux, double amplitude, double p, double i, double d, double offset, int filter, int filter_amplitude, int filter_phase, int nwaves, int lfr, int init)
{
     int loopback, intosc = 0;
     int increment1, increment2, increment3;
     //pid parameters are now used only in hwserver, not on the FPGA

     if (mode==LIBRP_KPFM_FM || mode==LIBRP_KPFM_FM_MANUAL) loopback = 1;
     else loopback = 0;

     if (init && (mode==LIBRP_KPFM_SIDEBAND_MANUAL || mode==LIBRP_KPFM_SIDEBAND)) {
         printf("Sideband KPFM freqs: mechanical driving %g Hz, ac lockin %g Hz, aux ac output %g Hz\n", rpset->frequency1, frequency, frequency_generate_aux);
         librp_GetKPFMSidebandFreq(rpset, frequency_generate_aux, &increment1, &increment2, &increment3);
         librp_SetAuxGenIncrement(rpset, increment3); //aux is what goes out
         rpset->increment1 = increment1;
         rpset->increment2 = increment2;
         rpset->increment3 = increment3;
         intosc = 1;
     }

     increment1 = rpset->increment1;
     increment2 = rpset->increment2;
     increment3 = rpset->increment3;

     librp_SetSignalProcessing(rpset, LIBRP_CH_2, filter, loopback, rpset->hr2, rpset->phaseshift2, 0, filter_amplitude, filter_phase, nwaves, lfr, intosc); 

     if (mode==LIBRP_KPFM_SIDEBAND_MANUAL || mode==LIBRP_KPFM_SIDEBAND) {
         librp_SetGenIncrement(rpset, LIBRP_CH_1, increment1);
         librp_SetGenByIncrement(rpset, LIBRP_CH_2, increment2, amplitude, offset);
         librp_SetAuxGenIncrement(rpset, increment3); //aux is what goes out
     } 
     else
     {
         librp_SetGen(rpset, LIBRP_CH_2, frequency, amplitude, offset);
     }

     rpset->loopback2 = loopback;
     rpset->kpfm_mode = mode;
     rpset->intosc2 = intosc;

     //printf("# KPFM set to mode %d, gen %g %g  pid %g %g %g offset %g filter %d lockin filters %d %d loopback %d\n", mode, frequency, amplitude, p, i, d, offset, filter, filter_amplitude, filter_phase, loopback);

     return LIBRP_OK;
}

//use f1 increment to calculate approximate f2 and f3 to have nice increment ratio or f1/f2
double librp_GetKPFMSidebandFreq(librpSet *rpset, double desiredfreq, int *sincrement1, int *sincrement2, int *sincrement3)
{
   double frequency1 = rpset->frequency1;
   double frequency2 = frequency1 + desiredfreq;
   double newfreq1, newfreq2, newfreq3;
   int increment1 = rpset->increment1; 
   int increment2, increment3, newincrement1, newincrement2, newincrement3;
   double m, k;
   int im, ik, nwaves, i1, i2, i3, newim;
   double systemclock = 125e6;
   double phasewidth = rpset->ddswidth;
   double factor = pow(2, phasewidth)/systemclock;
   double dist, mindist;

   if (rpset->nwaves2 == 0) nwaves = 1;
   else if (rpset->nwaves2 == 1) nwaves = 2;
   else if (rpset->nwaves2 == 2) nwaves = 4;
   else if (rpset->nwaves2 == 3) nwaves = 8;
   else if (rpset->nwaves2 == 4) nwaves = 32;
   else if (rpset->nwaves2 == 5) nwaves = 128;
   else nwaves = 512;

   printf("suggesting KPFM: freqs f1 %g f2 %g desired %g\n", frequency1, frequency2, desiredfreq); 

   newincrement1 = increment1 = (int32_t)(frequency1*factor);
   newincrement2 = increment2 = (int32_t)(frequency2*factor);
   newincrement3 = increment3 = (int32_t)(desiredfreq*factor);

   mindist = 1e10;

   //difference and positive detection matching best
   for (i1=(increment1-1000); i1<(increment1+1000); i1++)
   {
      for (i2=(increment2-1000); i2<(increment2+1000); i2++)
      {
        i3 = i1 + i2;
        m = nwaves*(double)i2/(double)i3;
        im = floor(m);
        dist = sqrt((increment1-i1)*(increment1-i1) + (increment2-i2)*(increment2-i2));
        if (fabs((double)im - m)==0 && dist < mindist) {
            printf("got it, matching increments %d %d %d  %d %d %d\n", i1, i2, i3, i1, i2*nwaves, i3*im);
            newincrement1 = i1;
            newincrement2 = i2;
            newincrement3 = i3;
            newim = im;
            mindist = dist;
        }
      }
   }

  //difference and negative detection matching best
/*
   for (i1=(increment1-1000); i1<(increment1+1000); i1++)
   {
      for (i2=(increment2-1000); i2<(increment2+1000); i2++)
      {
        i3 = i1 - i2;
        m = nwaves*(double)i2/(double)i3;
        im = floor(m);
        dist = sqrt((increment1-i1)*(increment1-i1) + (increment2-i2)*(increment2-i2));
        if (fabs((double)im - m)==0 && dist < mindist) {
            printf("got it, matching increments %d %d %d  %d %d %d\n", i1, i2, i3, i1, i2*nwaves, i3*im);
            newincrement1 = i1;
            newincrement2 = i2;
            newincrement3 = i3;
            newim = im;
            mindist = dist;
        }
      }
   }
*/

/* //all three matching best   
   for (i1=(increment1-4000); i1<(increment1+4000); i1++)
   {
     for (i2=(increment2-4000); i2<(increment2+4000); i2++)
     {
        m = nwaves*(double)i2/(double)i1;
        im = floor(m);
        dist = sqrt((increment1-i1)*(increment1-i1) + (increment2-i2)*(increment2-i2));
        if (fabs((double)im - m)==0) {
            for (i3=(increment3-1000); i3<(increment3+1000); i3++)
            {
               k = nwaves*(double)i2/(double)i3;
               ik = floor(k);
               if (fabs((double)ik - k)==0 && dist<mindist) {
                 printf("got it, matching increments %d %d %d  %d %d %d dist %g\n", i2, i1, i3, i2*nwaves, i1*im, i3*ik, dist);
                 newincrement3 = i3;
                 newincrement2 = i2;
                 newincrement1 = i1;
                 newim = im;
                 mindist = dist;
              }
            }
        }
     }
   }
*/
/* //slow and detection matching best
   for (i2=(increment2-1000); i2<(increment2+1000); i2++)
   {
      for (i3=(increment3-100); i3<(increment3+100); i3++)
      {
        m = nwaves*(double)i2/(double)i3;
        im = floor(m);
        if (fabs((double)im - m)==0) {
            printf("got it, matching increments %d %d   %d %d\n", i2, i3, i2*nwaves, i3*im);
            newincrement2 = i2;
            newincrement3 = i3;
            newim = im;
        }
      }
   }
*/
   newfreq1 = newincrement1/factor;
   newfreq2 = newincrement2/factor;
   newfreq3 = newincrement3/factor;
 
   printf("suggested KPFM: desired f1 %.10f  f2 %.10f  increments  %d %d   new: f1 %.10f  f2 %.10f  f3 %.10f  increments %d %d %d\n", 
        frequency1, frequency2, increment1, increment2, newfreq1, newfreq2, newfreq3, newincrement1, newincrement2, newincrement3);

   *sincrement1 = newincrement1;
   *sincrement2 = newincrement2;
   *sincrement3 = newincrement3;

   return newfreq3;
}

int librp_SetDART(librpSet *rpset, int mode, double frequency, double amplitude, double p, double i, double d, double freqspan, int filter, int lfr)
{
     //pid parameters are now used only in hwserver, not on the FPGA
     if (mode !=0) {

        librp_SetSignalProcessing(rpset, LIBRP_CH_1, filter, 0, 1, rpset->phaseshift1, 0, 1, 1, 2, lfr, 0); //always filter both amplitude and phase 
        librp_SetSignalProcessing(rpset, LIBRP_CH_2, filter, 0, 1, rpset->phaseshift1, 0, 1, 1, 2, lfr, 0); 

        librp_SetGen(rpset, LIBRP_CH_1, frequency-freqspan, amplitude, 0);
        librp_SetGen(rpset, LIBRP_CH_2, frequency+freqspan, amplitude, 0);

        rpset->dart_mode = mode;

        //printf("# DART set to mode %d, freqs %g %g amplitude %g  pid %g %g %g filter %d lockin filters 1 1\n", mode, frequency-freqspan, frequency+freqspan, amplitude, p, i, d, filter);
     } 
     return LIBRP_OK;
}


int librp_InitHRDAC(librpSet *rpset)
{
    int32_t smallrange = 0x02000C80;
    int32_t fullrange = 0x02000D00;
    //int32_t fullrange = 0x02002900; // FSDO=0
    int i;

    printf("init hrdac to %d %d %d\n", rpset->hrdac1_range, rpset->hrdac2_range, rpset->hrdac3_range);
   
    for (i=0; i<20; i++)
    {
       if (rpset->hrdac1_range == LIBRP_RANGE_FULL) *((int32_t *)(rpset->cfg17)) = fullrange;
       else *((int32_t *)(rpset->cfg17)) = smallrange;

       if (rpset->hrdac2_range == LIBRP_RANGE_FULL) *((int32_t *)(rpset->cfg17 + 8)) = fullrange;
       else *((int32_t *)(rpset->cfg17 + 8)) = smallrange;

       if (rpset->hrdac3_range == LIBRP_RANGE_FULL) *((int32_t *)(rpset->cfg5 + 8)) = fullrange;
       else  *((int32_t *)(rpset->cfg5 + 8)) = smallrange;

       usleep(20000);      
    }

    return LIBRP_OK;
}

int librp_SetDAC(librpSet *rpset, int channel, double value)
{
    int32_t newval;

    //printf("######################################   chan %d val %g 2raw %d\n", channel, value, (int32_t)(value*rpset->dac2cal + rpset->dac2shift));
    //value: depending on range and mode: limit to +-10 V, 0-10 V, +-1 V
    if (channel == LIBRP_CH_1) {
       newval = (int32_t)(value*rpset->dac1cal + rpset->dac1shift);
       *((int32_t *)(rpset->cfg10 + 8)) = newval;
       rpset->dac1val = newval; //unused for anything useful now
    }
    else if (channel == LIBRP_CH_2)    //udelat radeji samostatnou funkci set_hrdac?
    {
       newval = (int32_t)(value*rpset->dac2cal + rpset->dac2shift);
       *((int32_t *)(rpset->cfg8 + 8)) = newval;
       rpset->dac2val = newval; //unused for anything useful now
    }

    return LIBRP_OK;
}

int librp_SetHRDAC1(librpSet *rpset, double value)
{
    int32_t ivalue, mask, newval;

    //variant with mixing with mask
    if (rpset->hrdac1_range == LIBRP_RANGE_FULL) ivalue = (int32_t)( ( (unsigned long)((value + 10.0)/20.0*1048576) ) << 4 ); 
    else ivalue = (int32_t)( ( (unsigned long)((value)/10.0*1048576) ) << 4 );

    if (ivalue > FULLCODE) ivalue = FULLCODE;
    mask = 0x01000000;
    newval = mask | ivalue;

    *((int32_t *)(rpset->cfg17)) = newval;
    //printf("setting hrdac1 to %g  ivalue %d\n", value, ivalue);
    return LIBRP_OK;
}

int librp_SetHRDAC2(librpSet *rpset, double value)
{
    int32_t ivalue, mask, newval;

    //variant with mixing with mask
    if (rpset->hrdac2_range == LIBRP_RANGE_FULL) ivalue = (int32_t)( ( (unsigned long)((value + 10.0)/20.0*1048576) ) << 4 ); 
    else ivalue = (int32_t)( ( (unsigned long)((value)/10.0*1048576) ) << 4 );

    if (ivalue > FULLCODE) ivalue = FULLCODE;
    mask = 0x01000000;
    newval = mask | ivalue;

    *((int32_t *)(rpset->cfg17 + 8)) = newval;
    //printf("setting hrdac2 to %g  ivalue %d\n", value, ivalue);
    return LIBRP_OK;
}

int librp_SetHRDAC3(librpSet *rpset, double value)
{
    int32_t ivalue;

    //variant without mixing with mask, for hrdac3 this is done on FPGA
    if (rpset->hrdac3_range == LIBRP_RANGE_FULL) ivalue = (int32_t)( ( (unsigned long)((value + 10.0)/20.0*1048576) ) << 4 ); //should this be done at RP only?
    else ivalue = (int32_t)( ( (unsigned long)((value)/10.0*1048576) ) << 4 );

    if (ivalue > FULLCODE) ivalue = FULLCODE;

    *((int32_t *)(rpset->cfg5 + 8)) = ivalue;
    rpset->hrdacval = ivalue;//unused for anything useful now
   
    //printf("setting hrdac to %g  ivalue %d\n", value, ivalue);
    return LIBRP_OK;
}

int librp_SetPidOffset(librpSet *rpset, double value)
{
    int32_t ivalue;

    //we pass the data the same way as HRDAC, regardless if we finally use it or not, but do not offset it, this value is the offset itself
    if (rpset->hrdac3_range == LIBRP_RANGE_FULL) ivalue = (int32_t)( ( (unsigned long)((value)/20.0*1048576) ) << 4 );
    else ivalue = (int32_t)( ( (unsigned long)((value)/10.0*1048576) ) << 4 );

    *((int32_t *)(rpset->cfg12 + 8)) = ivalue;

    rpset->pidoffset = value;
   
    //printf("setting pid offset to %g  ivalue %d\n", value, ivalue);

    return LIBRP_OK;
}

double librp_GetPidOffset(librpSet *rpset)
{
    return rpset->pidoffset;
   
}

int librp_SetSetpoint(librpSet *rpset, double setpoint, int raw)
{
    int32_t value;
    int state;
    double divfactor = 8.0*65536.0/536870912.0; //2^29

    state = rpset->state;

    if (raw) {
        value = (int32_t)setpoint;
    } else {

        if (state==LIBRP_MODE_A1) value = (int32_t)(real_amplitude_to_raw(rpset, setpoint, LIBRP_CH_1)); 
        else if (state==LIBRP_MODE_A2) value = (int32_t)(real_amplitude_to_raw(rpset, setpoint, LIBRP_CH_2)); 
        else if (state==LIBRP_MODE_P1 || state==LIBRP_MODE_P2) value = (int32_t)(setpoint/divfactor);
        else if (state==LIBRP_MODE_FM) value = (int32_t)(setpoint/rpset->freqfactor); //conversion from Hertz to generator units, was 34/4
        else if (state==LIBRP_MODE_IN1) value = (int32_t)((setpoint)*rpset->adc1cal - rpset->adc1shift);
        else value = (int32_t)((setpoint)*rpset->adc2cal - rpset->adc2shift);

        //printf("# setting raw setpoint to %g  (%d)\n", setpoint, value);
    }

    *((int32_t *)(rpset->cfg5)) = value;

    return LIBRP_OK;
}

//range is 0-1 for all the three parameters.
int librp_SetPid(librpSet *rpset, double p, double i, double d, int errorshift)
{
    uint32_t valuea = 0, valueb = 0;
    double kp, ki, kd;
    int32_t pidp, pidi, pidd;
    double t;

    if (p<0) p = 0;
    if (p>1.0) p = 1.0;

    if (i<0) i = 0;
    if (i>1.0) i = 1.0;

    if (d<0) d = 0;
    if (d>1.0) d = 1.0;

/*
    kp = p*20000; //was 20000 for all
    ki = i*20000;
    kd = d*20000;

    t = 0.2;
    pidp = kp + ki*t + kd/t;
    pidi = -kp - 2*kd/t;
    pidd = kd/t;
*/
    //printf("# pid %g %g %g set to %d %d %d  range is -30000...30000\n", p, i, d, pidp, pidi, pidd);

    kp = p*10000;
    ki = i*10000;
    kd = d*10000;

    pidp = kp;
    pidi = ki;
    pidd = 0;

    if (pidp > 30000) pidp = 30000;
    if (pidi > 30000) pidi = 30000;
    if (pidd > 30000) pidd = 30000;

    if (pidp < -30000) pidp = -30000;
    if (pidi < -30000) pidi = -30000;
    if (pidd < -30000) pidd = -30000;


    valuea |= pidp + 30000;
    valuea |= (pidi + 30000) << 16;
    valueb |= pidd + 30000;
    valueb |= errorshift<<16;

    //printf("# pid %g %g %g set to %d %d %d\n", p, i, d, pidp, pidi, pidd);

    *((uint32_t *)(rpset->cfg6)) = valuea;
    *((uint32_t *)(rpset->cfg6 + 8)) = valueb;

    return LIBRP_OK;
}


int librp_SPICycle(librpSet *rpset, double hrdac1, double hrdac2, double
                   hrdac3, double *lrdac, double *adc, int read_adcA, int
                   read_adcB, int multiplexerA, int multiplexerB) {
    int i;
    unsigned long hr1, hr2, hr3;
    unsigned long lr[16];
    double value;
    int set, setit[16];


    // set both multiplexers
    if (multiplexerA != rpset->old_muxA || multiplexerB != rpset->old_muxB){
        if (select_EXP(rpset) != LIBRP_OK) {
            fprintf(stderr, "select_EXP failed!\n");
            return -1;
        }
        if (change_spi_mode(rpset, SPI_MODE_3) != LIBRP_OK){
            fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
            return -1;
        }
        if (set_mux(rpset, multiplexerA, multiplexerB) != LIBRP_OK) {
            fprintf(stderr, "set_mux failed (A: %d  B: %d)!\n", multiplexerA, multiplexerB);
            return -1;
        }
        rpset->old_muxA = multiplexerA;
        rpset->old_muxB = multiplexerB;
    }

    if (rpset->hrdac_regime == LIBRP_HRDAC_REGIME_CPU)
    {

    // set voltage on DAC (20b)
    //prepare the values to be sent


    if (rpset->hrdac_range == LIBRP_RANGE_FULL) 
    {
       if (hrdac1>10) hrdac1 = 10;
       if (hrdac1<-10) hrdac1 = -10;
       hr3 = ( ( (unsigned long)((hrdac1 + 10.0)/20.0*1048576) ) << 4 );
       if (hr3 > FULLCODE) hr3 = FULLCODE;

       if (hrdac2>10) hrdac2 = 10;
       if (hrdac2<-10) hrdac2 = -10;
       hr2 = ( ( (unsigned long)((hrdac2 + 10.0)/20.0*1048576) ) << 4 );
       if (hr2 > FULLCODE) hr2 = FULLCODE;

       if (hrdac3>10) hrdac3 = 10;
       if (hrdac3<-10) hrdac3 = -10;
       hr1 = ( ( (unsigned long)((hrdac3 + 10.0)/20.0*1048576) ) << 4 );
       if (hr1 > FULLCODE) hr1 = FULLCODE;
    } else {
       if (hrdac1>10) hrdac1 = 10;
       if (hrdac1<0) hrdac1 = 0;
       hr3 = ( ( (unsigned long)((hrdac1)/10.0*1048576) ) << 4 );
       if (hr3 > FULLCODE) hr3 = FULLCODE;

       if (hrdac2>10) hrdac2 = 10;
       if (hrdac2<0) hrdac2 = 0;
       hr2 = ( ( (unsigned long)((hrdac2)/10.0*1048576) ) << 4 );
       if (hr2 > FULLCODE) hr2 = FULLCODE;

       if (hrdac3>10) hrdac3 = 10;
       if (hrdac3<0) hrdac3 = 0;
       hr1 = ( ( (unsigned long)((hrdac3)/10.0*1048576) ) << 4 );
       if (hr1 > FULLCODE) hr1 = FULLCODE;
    }

    
    if (hr1 != rpset->old_hr1 || hr2 != rpset->old_hr2 || hr3 != rpset->old_hr3) 
    {
        if (select_DAC20b(rpset) != LIBRP_OK) {
            fprintf(stderr, "select_DAC20b failed!\n");
            return -1;
        }
        if (change_spi_mode(rpset, SPI_MODE_2) != LIBRP_OK){
            fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
            return -1;
        }
        //printf("write DAC (20b)\n");
        if (rpset->hrdac_regime==LIBRP_HRDAC_REGIME_MIX) 
        {
           if (writeDAC20b_XY(rpset->spi_fd,
                               DACDATA, hr2,
                               DACDATA, hr3) != LIBRP_OK){
               fprintf(stderr, "writeDAC20b_XY failed!\n");
               return -1;
           }
         }
        else {
              if (writeDAC20b_XYZ(rpset->spi_fd, DACDATA, hr1,
                                  DACDATA, hr2,
                                  DACDATA, hr3) != LIBRP_OK){
                  fprintf(stderr, "writeDAC20b_XYZ failed!\n");
                  return -1;
              }
           }
           rpset->old_hr1 = hr1; 
           rpset->old_hr2 = hr2; 
           rpset->old_hr3 = hr3; 
        }
    }

    // set voltage on DAC (16b)
    // prepare the values to be sent
    set = 0;
    for (i=0; i<16; i++) {
        value = lrdac[i];
        if (value>10) value = 10;
        if (value<-10) value = -10;
        lr[i] = (value+10.0)/20*65535;
        if (lr[i] != rpset->old_lr[i]) setit[i] = 1;
        else setit[i] = 0;

        set += setit[i];
    }

    if (set) {
        //printf("select DAC81416\n");
        if (select_DAC16b(rpset) != LIBRP_OK) {
            fprintf(stderr, "select_DAC16b failed!\n");
            return -1;
        }
        if (change_spi_mode(rpset, SPI_MODE_2) != LIBRP_OK){
            fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
            return -1;
        }
        //printf("write DAC (16b)\n");
        for (i=0; i<16; i++) 
        {
            if (setit[i]) 
            {
                //printf("DAC%d: %lf\n", i, lrdac[i]);
                if (writeDAC16b(rpset, DAC0+i, lr[i]) != LIBRP_OK){
                    fprintf(stderr, "writeDAC16b failed\n");
                    return -1;
                }
                rpset->old_lr[i] = lr[i]; 
            }
        }
        if (update_values_DAC16b(rpset) != LIBRP_OK){
            fprintf(stderr, "update_values_DAC16b failed.\n");
            return -1;
        }
        usleep(50); // settling time of 12 us according to datasheet; probably more is needed
    }

    // read voltages using ADS8598H A and/or B
    if (read_adcA || read_adcB){
        if (change_spi_mode(rpset, SPI_MODE_0) != LIBRP_OK){ // was SPI_MODE_3
            fprintf(stderr, "Change of SPI mode failed. Error: %s\n", strerror(errno));
            return -1;
        }
        if (conversion(rpset) != LIBRP_OK){
            fprintf(stderr, "conversion() failed.\n");
            return -1;
        }
        usleep(127); // Conversion time: BUSY high time (depends on oversampling - see ADC8598H's datasheet)
        if (read_adcA){
            if (select_ADC18b_A(rpset) != LIBRP_OK) {
                fprintf(stderr, "select_ADC18b_A failed!\n");
                return -1;
            }
            if (readADC18b(rpset, adc) != LIBRP_OK){
                fprintf(stderr, "readADC18b() failed.\n");
                return -1;
            }
        }
        if (read_adcB){
            if (select_ADC18b_B(rpset) != LIBRP_OK) {
                fprintf(stderr, "select_ADC18b_B failed!\n");
                return -1;
            }
            if(readADC18b(rpset, adc+8) != LIBRP_OK){
                fprintf(stderr, "readADC18b() failed.\n");
                return -1;
            }
        }
    }

    return LIBRP_OK;
}

static int init_spi(librpSet *rpset, uint8_t mode)
{
    /* Opening file stream */
    if (rpset->spi_fd >= 0) {
       printf("spi fd is %d\n", rpset->spi_fd);
       return LIBRP_SPI_ALREADY_INIT;
    }

    rpset->spi_fd = open("/dev/spidev1.0", O_RDWR | O_NOCTTY);
    if (rpset->spi_fd < 0){
        fprintf(stderr, "Error opening spidev1.0. Error: %s\n", strerror(errno));
        return -1;
    }

    if (ioctl(rpset->spi_fd, SPI_IOC_WR_MODE, &mode) < 0){
        fprintf(stderr, "Error setting SPI MODE. Error: %s\n", strerror(errno));
        return -1;
    }

    /* Setting SPI bus speed */
    int spi_speed = SPI_SPEED;

    if (ioctl(rpset->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0){
        fprintf(stderr, "Error setting SPI MAX_SPEED_HZ. Error: %s\n", strerror(errno));
        return -1;
    }

    //printf("# SPI opened\n");

    return LIBRP_OK;
}

int change_spi_mode(librpSet *rpset, uint8_t mode)
{
    if (ioctl(rpset->spi_fd, SPI_IOC_WR_MODE, &mode) < 0){
        fprintf(stderr, "Error setting SPI MODE (change_spi_mode). Error: %s\n", strerror(errno));
        return -1;
    }

    return LIBRP_OK;
}

static int release_spi(librpSet *rpset)
{
    /* Release the spi resources */
    close(rpset->spi_fd);

    return LIBRP_OK;
}

static int update_values(librpSet *rpset)
{
    // set pins low 
    if (librp_DMultiPinSetState(rpset, 3, LDAC1Pin, LIBRP_LOW, LDAC2Pin, LIBRP_LOW, LDAC3Pin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }

    //usleep(1);

    // set pins high 
    if (librp_DMultiPinSetState(rpset, 3, LDAC1Pin, LIBRP_HIGH, LDAC2Pin, LIBRP_HIGH, LDAC3Pin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }


    return LIBRP_OK;
}

static int update_values_DAC16b(librpSet *rpset){
    // set pin low 
    if (librp_DPinSetState(rpset, LDACPin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }

    //usleep(1);

    // set pin high 
    if (librp_DPinSetState(rpset, LDACPin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }

    return LIBRP_OK;
}

static int prepare_io(librpSet *rpset){

/*
    // DAC11001
    // set pin direction - not needed, pins are always out
    // set pins high
    // LDAC2&3 now used for FPGA SPI, X and Y DAC 20b values updated via Linux SPI
    if (librp_DPinSetState(rpset, LDAC1Pin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }
    if (librp_DPinSetState(rpset, LDAC2Pin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }
    if (librp_DPinSetState(rpset, LDAC3Pin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }
*/
    // 4-16 decoder
    // set pin direction - not needed, pins are always out
    // set address pins low; latch enable high to operate

    if (librp_DMultiPinSetState(rpset, 5, 
                                DECA0Pin, LIBRP_LOW,
                                DECA1Pin, LIBRP_LOW,
                                DECA2Pin, LIBRP_LOW,
                                DECA3Pin, LIBRP_LOW,
                                DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }

    // ADS8598H A&B
    // set pin direction - not needed, pins are always out
    // set pins high
    if (librp_DPinSetState(rpset, ADCCONVSTPin, LIBRP_HIGH) != LIBRP_OK) {
 
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }

    // DAC81416 
    // set pin direction - not needed, pins are always out
    // set pin high
    if (librp_DPinSetState(rpset, LDACPin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }

    return LIBRP_OK;
}

//Sends the same write command to N DAC11001 daisy-chained (ioctl version)
static int configureDAC20b(int fd, byte thisRegister, unsigned long thisValue, int N)
{
    int size = N;
    byte buf[4];
    int i;
    struct spi_ioc_transfer xfer[size];
    int                     status;

    memset(xfer, 0, sizeof xfer);

    // now combine the register address and the command into one byte:
    buf[0] = thisRegister & WRITE;
    buf[3] = thisValue & 0xFF;
    buf[2] = (thisValue >> 8) & 0xFF;
    buf[1] = (thisValue >> 16) & 0xFF;

    //fprintf(stdout, "reg: 0x%02x\tvalue: 0x%06lx (0x%02x%02x%02x)\n", thisRegister, thisValue, 
    //        buf[1], buf[2], buf[3]);

    for (i=0; i<N; i++){
        xfer[i].tx_buf = (unsigned long)(buf); // xfer[i].tx_buf = (__u64)((__u32)buf);
        xfer[i].rx_buf = (unsigned long)(buf);
        xfer[i].len = 4;

    }

    status = ioctl(fd, SPI_IOC_MESSAGE(N), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

//Sends different write commands to three DAC11001 daisy-chained (ioctl version)
static int writeDAC20b_XYZ(int fd, byte thisRegister1, unsigned long thisValue1,
                           byte thisRegister2, unsigned long thisValue2,
                           byte thisRegister3, unsigned long thisValue3) 
{
    byte buf1[4];
    byte buf2[4];
    byte buf3[4];
    int size = 3;
    struct spi_ioc_transfer xfer[size];
    int                     status;

    memset(xfer, 0, sizeof xfer);

    buf1[0] = thisRegister1 & WRITE;
    buf1[3] = thisValue1 & 0xFF;
    buf1[2] = (thisValue1 >> 8) & 0xFF;
    buf1[1] = (thisValue1 >> 16) & 0xFF;
    /*
       fprintf(stdout, "reg1: 0x%02x\tvalue1: 0x%06lx (DEC:%lu) voltage1: %lf\n", 
       thisRegister1, 
       thisValue1, 
       thisValue1, 
       ((float)(thisValue1 >> 4)+1)/1048576*20.0 - 10.0);
     */
    /*    
          fprintf(stdout, "value1: 0x%06lx (DEC:%lu) voltage1: %lf\n", 
          thisValue1, 
          thisValue1, 
          ((float)(thisValue1 >> 4)+1)/1048576*20.0 - 10.0);
     */
    buf2[0] = thisRegister2 & WRITE;
    buf2[3] = thisValue2 & 0xFF;
    buf2[2] = (thisValue2 >> 8) & 0xFF;
    buf2[1] = (thisValue2 >> 16) & 0xFF;
    /*    
          fprintf(stdout, "value2: 0x%06lx (DEC:%lu) voltage2: %lf\n", 
          thisValue2, 
          thisValue2, 
          ((float)(thisValue2 >> 4)+1)/1048576*20.0 - 10.0);
     */
    buf3[0] = thisRegister3 & WRITE;
    buf3[3] = thisValue3 & 0xFF;
    buf3[2] = (thisValue3 >> 8) & 0xFF;
    buf3[1] = (thisValue3 >> 16) & 0xFF;
    /*    
          fprintf(stdout, "value3: 0x%06lx (DEC:%lu) voltage3: %lf\n", 
          thisValue3, 
          thisValue3, 
          ((float)(thisValue3 >> 4)+1)/1048576*20.0 - 10.0);
     */

    xfer[0].tx_buf = (unsigned long)(buf1);
    xfer[0].rx_buf = (unsigned long)(buf1);
    xfer[0].len = 4;

    xfer[1].tx_buf = (unsigned long)(buf2);
    xfer[1].rx_buf = (unsigned long)(buf2);
    xfer[1].len = 4;

    xfer[2].tx_buf = (unsigned long)(buf3);
    xfer[2].rx_buf = (unsigned long)(buf3);
    xfer[2].len = 4;

    status = ioctl(fd, SPI_IOC_MESSAGE(3), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    // sequence of transfers in the SPI message is obviously reversed; hence voltage1 <-> thisValue3
    /*
       printf("voltage1: %lf voltage2: %lf voltage3: %lf\n",
       ((float)(thisValue3 >> 4))/1048576*20.0 - 10.0,
       ((float)(thisValue2 >> 4))/1048576*20.0 - 10.0,
       ((float)(thisValue1 >> 4))/1048576*20.0 - 10.0);
     */
    return LIBRP_OK;
}

//Sends different write commands to two DAC11001 daisy-chained (ioctl version)
static int writeDAC20b_XY(int fd, byte thisRegister1, unsigned long thisValue1,
                           byte thisRegister2, unsigned long thisValue2)
{
    byte buf1[4];
    byte buf2[4];
    int size = 2;
    struct spi_ioc_transfer xfer[size];
    int                     status;

    memset(xfer, 0, sizeof xfer);

    buf1[0] = thisRegister1 & WRITE;
    buf1[3] = thisValue1 & 0xFF;
    buf1[2] = (thisValue1 >> 8) & 0xFF;
    buf1[1] = (thisValue1 >> 16) & 0xFF;
    /*
       fprintf(stdout, "reg1: 0x%02x\tvalue1: 0x%06lx (DEC:%lu) voltage1: %lf\n", 
       thisRegister1, 
       thisValue1, 
       thisValue1, 
       ((float)(thisValue1 >> 4)+1)/1048576*20.0 - 10.0);
     */
    /*    
          fprintf(stdout, "value1: 0x%06lx (DEC:%lu) voltage1: %lf\n", 
          thisValue1, 
          thisValue1, 
          ((float)(thisValue1 >> 4)+1)/1048576*20.0 - 10.0);
     */
    buf2[0] = thisRegister2 & WRITE;
    buf2[3] = thisValue2 & 0xFF;
    buf2[2] = (thisValue2 >> 8) & 0xFF;
    buf2[1] = (thisValue2 >> 16) & 0xFF;
    /*    
          fprintf(stdout, "value2: 0x%06lx (DEC:%lu) voltage2: %lf\n", 
          thisValue2, 
          thisValue2, 
          ((float)(thisValue2 >> 4)+1)/1048576*20.0 - 10.0);
     */

    xfer[0].tx_buf = (unsigned long)(buf1);
    xfer[0].rx_buf = (unsigned long)(buf1);
    xfer[0].len = 4;

    xfer[1].tx_buf = (unsigned long)(buf2);
    xfer[1].rx_buf = (unsigned long)(buf2);
    xfer[1].len = 4;

    status = ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    // sequence of transfers in the SPI message is obviously reversed; hence voltage1 <-> thisValue2
    /*
       printf("voltage1: %lf voltage2: %lf\n",
       ((float)(thisValue2 >> 4))/1048576*20.0 - 10.0,
       ((float)(thisValue1 >> 4))/1048576*20.0 - 10.0);
     */
    return LIBRP_OK;
}

// simultaneous conversion start at both ADCs
static int conversion(librpSet *rpset)
{

    if (librp_DPinSetState(rpset, ADCCONVSTPin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }

    //usleep(1000);

    if (librp_DPinSetState(rpset, ADCCONVSTPin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "rp_DpinSetState failed!\n");
        return -1;
    }

    return LIBRP_OK;
}

//read from ADS8598H
static int readADC18b(librpSet *rpset, double *voltage)
{
    //    int       i
    int size = 1;
    byte buf[18];
    int status;
    unsigned long val18b;
    int negative;
    int nativeInt;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 18;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    //    for (i=0; i<18; i++)
    //        fprintf(stdout, "%02x ", buf[i]);
    //    fprintf(stdout, "\n");

    // voltage 1
    val18b = 0;
    val18b = (buf[0]<<10)+(buf[1]<<2)+(buf[2]>>6);
    val18b = val18b & 0x03FFFF; // 0000 0011 1111 1111 1111 1111 in binary, i.e. last 18 bits
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[0] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[0] = (double)nativeInt/131071*10.0;
    }

//    fprintf(stdout, "0x%06lx %d %lf\n", val18b, nativeInt, voltage[0]);

    // voltage 2
    val18b = 0;
    val18b = ((buf[2])<<12)+(buf[3]<<4)+(buf[4]>>4);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[1] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[1] = (double)nativeInt/131071*10.0;
    }

    // voltage 3
    val18b = 0;
    val18b = ((buf[4])<<14)+(buf[5]<<6)+(buf[6]>>2);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[2] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[2] = (double)nativeInt/131071*10.0;
    }

    // voltage 4
    val18b = 0;
    val18b = ((buf[6])<<16)+(buf[7]<<8)+(buf[8]);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[3] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[3] = (double)nativeInt/131071*10.0;
    }

    // voltage 5
    val18b = 0;
    val18b = ((buf[9])<<10)+(buf[10]<<2)+(buf[11]>>6);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[4] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[4] = (double)nativeInt/131071*10.0;
    }

    // voltage 6
    val18b = 0;
    val18b = ((buf[11])<<12)+(buf[12]<<4)+(buf[13]>>4);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[5] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[5] = (double)nativeInt/131071*10.0;
    }

    // voltage 7
    val18b = 0;
    val18b = ((buf[13])<<14)+(buf[14]<<6)+(buf[15]>>2);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[6] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[6] = (double)nativeInt/131071*10.0;
    }

    // voltage 8
    val18b = 0;
    val18b = ((buf[15])<<16)+(buf[16]<<8)+(buf[17]);
    val18b = val18b & 0x03FFFF;
    negative = (val18b & (1 << 17)) != 0;
    if (negative){
        nativeInt = val18b | ~((1 << 18) - 1);
        voltage[7] = (double)nativeInt/131072*10.0;
    }
    else{
        nativeInt = val18b;
        voltage[7] = (double)nativeInt/131071*10.0;
    }

    return LIBRP_OK;
}

// configure DAC81416
static int configureDAC16b(librpSet *rpset)
{
    // DEV-PWDWN = 0 sets the device in active mode
    if (writeDAC16b(rpset, SPICONFIG, 0x0A84) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), SPICONFIG failed\n");
        return -1;
    }

    // enable internal reference
    if (writeDAC16b(rpset, GENCONFIG, 0x3F00) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), GENCONFIG failed\n");
        return -1;
    }

    //  all DAC outputs are set to update in response to an LDAC trigger (synchronous mode)
    if (writeDAC16b(rpset, SYNCCONFIG, 0xFFFF) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), SYNCCONFIG failed\n");
        return -1;
    }

    // set DAC[15:12] range to (-10V, +10V)
    if (writeDAC16b(rpset, DACRANGE0, 0xAAAA) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), DACRANGE0 failed\n");
        return -1;
    }

    // set DAC[11:8] range to (-10V, +10V)
    if (writeDAC16b(rpset, DACRANGE1, 0xAAAA) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), DACRANGE1 failed\n");
        return -1;
    }

    // set DAC[7:4] range to (-10V, +10V)
    if (writeDAC16b(rpset, DACRANGE2, 0xAAAA) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), DACRANGE2 failed\n");
        return -1;
    }

    // set DAC[3:0] range to (-10V, +10V)
    if (writeDAC16b(rpset, DACRANGE3, 0xAAAA) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), DACRANGE3 failed\n");
        return -1;
    }

    //  enable all DACs (bring from power-down mode)
    if (writeDAC16b(rpset, DACPWDWN, 0x0000) != LIBRP_OK){
        fprintf(stderr, "configureDAC(), DACPWDWN failed\n");
        return -1;
    }

    return LIBRP_OK;
}

//Sends a write command to DAC81416
static int writeDAC16b(librpSet *rpset, byte thisRegister, unsigned long thisValue)
{
    int size = 1;
    byte buf[3], rxbuf[3];
    struct spi_ioc_transfer xfer[size];
    int                     status;

    memset(xfer, 0, sizeof xfer);

    // now combine the register address and the command into one byte:
    buf[0] = thisRegister & WRITE1;
    buf[2] = thisValue & 0xFF;
    buf[1] = (thisValue >> 8) & 0xFF;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(rxbuf);
    xfer[0].len = 3;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    /*
       fprintf(stdout, "reg: 0x%02x\tvalue: 0x%04lx (0x%02x%02x) voltage: %lf\n", thisRegister, thisValue,
       buf[1], buf[2], ((float)thisValue)/65536*20.0 - 10.0);
     */
    //printf("voltage: %lf\n", ((float)thisValue)/65536*20.0 - 10.0);

    return LIBRP_OK;
}

// set LED diodes 
static int set_led(librpSet *rpset, int led, int val)
{
    int size = 1;
    byte buf[2];
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    if ( (led < 0 || led > 2) || (val < 0 || val > 1)){
        fprintf(stderr, "Such LED diode setting is not supported!\n");
        return -1;
    }

    if (val){
        buf[1] = 1;
    }
    else{
        buf[1] = 0;
    }

    switch (led){
        case 0: // STATUS LED
            buf[0] = PORT28 & WRITE2;
            break;

        case 1: // ERROR LED
            buf[0] = PORT29 & WRITE2;
            break;

        case 2: // RUN LED
            buf[0] = PORT30 & WRITE2;
            break;
    }

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

// set input range 
static int set_input_range(librpSet *rpset)
{
    int size = 1;
    byte buf[2];
    byte range_in1, range_in2;
    int status;
    int input_range1 = 0;
    int input_range2 = 0;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    input_range1 = rpset->input_range1;
    input_range2 = rpset->input_range2;

    //printf("setting input range to %d %d\n", input_range1, input_range2);

    if ( input_range1 < 0 || input_range1 > 1 || input_range2 < 0 || input_range2 > 1){
        fprintf(stderr, "Such range setting is not supported!\n");
        return -1;
    }

    if ( input_range1 == 0 ){
        range_in1 = 0;
    }
    else{
        range_in1 = 1;
    }
    if ( input_range2 == 0 ){
        range_in2 = 0;
    }
    else{
        range_in2 = 1;
    }

    buf[0] = PORT22 & WRITE2;
    buf[1] = range_in1;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORT23 & WRITE2;
    buf[1] = range_in2;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

// set oversampling 
int set_oversampling_factor(librpSet *rpset, int OS)
{
    int size = 1;
    byte buf[2];
    byte OS2, OS1, OS0;
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    //printf("# setting oversampling factor to %d\n", OS);

    if ( OS < 0 || OS > 6 ){
        fprintf(stderr, "Such oversampling is not supported!\n");
        return -1;
    }

    switch (OS){
        case 0: // no oversampling
            OS2 = 0;
            OS1 = 0;
            OS0 = 0;
            break;

        case 1: // OS ratio 2
            OS2 = 0;
            OS1 = 0;
            OS0 = 1;
            break;

        case 2: // OS ratio 4
            OS2 = 0;
            OS1 = 1;
            OS0 = 0;
            break;

        case 3: // OS ratio 8
            OS2 = 0;
            OS1 = 1;
            OS0 = 1;
            break;

        case 4: // OS ratio 16
            OS2 = 1;
            OS1 = 0;
            OS0 = 0;
            break;

        case 5: // OS ratio 32
            OS2 = 1;
            OS1 = 0;
            OS0 = 1;
            break;

        case 6: // OS ratio 64
            OS2 = 1;
            OS1 = 1;
            OS0 = 0;
            break;
    }

    buf[0] = PORT26 & WRITE2;
    buf[1] = OS2;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORT25 & WRITE2;
    buf[1] = OS1;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORT24 & WRITE2;
    buf[1] = OS0;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

// set both multiplexers A&B
static int set_mux(librpSet *rpset, int muxA, int muxB)
{
    int size = 1;
    byte buf[2];
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    if ( (muxA < 1 || muxA > 16) || (muxB < 1 || muxB > 16) ){
        fprintf(stderr, "wrong channels selected (mux)\n");
        return -1;
    }

    buf[0] = PORTS8TO15 & WRITE2;
        
    // A3A (P11); A2A (P10); A1A (P9); A0A (P8) => RP1 IN1
    // A3B (P15); A2B (P14); A1B (P13); A0B (P12) => RP1 IN2
    //      A3      A2      A1      A0      channel (96p_connector)
    //      0       0       0       0       1 (ADC_MUX_1)
    //      0       0       0       1       2 (ADC_MUX_2)
    //      0       0       1       0       3 (ADC_MUX_3)
    //      0       0       1       1       4 (ADC_MUX_4)
    //      0       1       0       0       5 (ADC_MUX_5)
    //      0       1       0       1       6 (ADC_MUX_6)
    //      0       1       1       0       7 (ADC_MUX_7)
    //      0       1       1       1       8 (ADC_MUX_8)
    //      1       0       0       0       9 (ADC_MUX_9)
    //      1       0       0       1       10 (ADC_MUX_10)
    //      1       0       1       0       11 (ADC_MUX_11)
    //      1       0       1       1       12 (ADC_MUX_12)
    //      1       1       0       0       13 (ADC_MUX_13)
    //      1       1       0       1       14 (ADC_MUX_14)
    //      1       1       1       0       15 (ADC_MUX_15)
    //      1       1       1       1       16 (ADC_MUX_16)
    
    buf[1] = ( (muxB - 1) << 4 ) + (muxA - 1);
    //printf("muxA %d muxB %d buf1 0x%02x\n", muxA, muxB, buf[1]);

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

// configure IO expander
int configureEXP(librpSet *rpset)
{
    int size = 1;
    byte buf[2];
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    buf[0] = CONFIG2 & WRITE2;
    buf[1] = 0x01; // 0x01 normal operation; 0x00 shutdown

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG1 & WRITE2;
    // P7-4 output; P7 -> 01; P6 -> 01; P5 -> 01; P4 -> 01 => 0x55
    buf[1] = 0x55;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTS4TO7 & WRITE2;
    //buf[0] = PORT7 & WRITE2;
    // P7-4 output; P7 -> 0; P6 -> 1; P5 -> 1; P4 -> 1 => 0x07
    // DAC16b reset; ADC18bit reset; mux A enable; mux B enable
    buf[1] = 0x07;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG2 & WRITE2;
    // P11-8 output; P11 -> 01; P10 -> 01; P9 -> 01; P8 -> 01
    buf[1] = 0x55;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG3 & WRITE2;
    // P15-12 output; P15 -> 01; P14 -> 01; P13 -> 01; P12 -> 01
    buf[1] = 0x55;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTS8TO15 & WRITE2;
    // P11-8 output; P11 -> 0; P10 -> 0; P9 -> 0; P8 -> 0
    // A3A; A2A; A1A; A0A => RP1 input1 connected to ADC_MUX_1 (via MUX A)
    // P15-12 output; P15 -> 0; P14 -> 0; P13 -> 0; P12 -> 1
    // A3B; A2B; A1B; A0B => RP1 input2 connected to ADC_MUX_2 (via MUX B)
    buf[1] = 0x10;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    rpset->old_muxA = 1;
    rpset->old_muxB = 2;

    buf[0] = PORTCONFIG4 & WRITE2;
    // P19 input (w/o pull-up) P18-P16 output; P19 -> 10; P18 -> 01; P17 -> 01; P16 -> 01
    // ALARM1; CLR3; CLR2; CLR1
    buf[1] = 0x95;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG5 & WRITE2;
    // P23, P22 output, P21, P20 input (w/o pull-up); P23 -> 01; P22 -> 01; P21 -> 10; P20 -> 10
    // SW_BUF2; SW_BUF1; ALARM3; ALARM2
    buf[1] = 0x5A;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTS16TO23 & WRITE2;
    // P19 input (w/o pull-up) P18-P16 output; P19 -> 0; P18 -> 1; P17 -> 1; P16 -> 1
    // ALARM1; CLR3; CLR2; CLR1
    // P23, P22 output, P21, P20 input (w/o pull-up); P23 -> 0; P22 -> 0; P21 -> 0; P20 -> 0
    // SW_BUF2; SW_BUF1; ALARM3; ALARM2
    buf[1] = 0x07;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG6 & WRITE2;
    // P27-P24 output; P27 -> 01; P26 -> 01; P25 -> 01; P24 -> 01
    buf[1] = 0x55;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTCONFIG7 & WRITE2;
    // P31-P28 output; P31 -> 01; P30 -> 01; P29 -> 01; P28 -> 01
    buf[1] = 0x55;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORTS24TO31 & WRITE2;
    // P27-P24 output; P27 -> 0; P26 -> 0; P25 -> 0; P24 -> 0
    // not connected; OS2; OS1; OS0 - [0;0;0] means no oversampling
    // P31-P28 output; P31 -> 0; P30 -> 1; P29 -> 0; P28 -> 0
    // not connected; RUN; ERROR; STATUS
    buf[1] = 0x40;

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

static int select_DAC20b(librpSet *rpset){
    //printf("select DAC11001\n");
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_LOW) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    if (librp_DMultiPinSetState(rpset, 3, DECA0Pin, LIBRP_LOW, DECA1Pin, LIBRP_LOW, DECA2Pin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }
    
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    // A3 is always LOW

    return LIBRP_OK;
}

static int select_DAC16b(librpSet *rpset){
    //printf("select DAC81416\n");
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_LOW) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    if (librp_DMultiPinSetState(rpset, 3, DECA0Pin, LIBRP_LOW, DECA1Pin, LIBRP_LOW, DECA2Pin, LIBRP_HIGH) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
 //       fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    // A3 is always LOW


    return LIBRP_OK;
}

static int select_ADC18b_A(librpSet *rpset){
    //printf("select ADS8598H A\n");
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_LOW) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    if (librp_DMultiPinSetState(rpset, 3, DECA0Pin, LIBRP_HIGH, DECA1Pin, LIBRP_HIGH, DECA2Pin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    // A3 is always LOW

    return LIBRP_OK;
}

static int select_ADC18b_B(librpSet *rpset){
    //printf("select ADS8598H B\n");
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_LOW) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    if (librp_DMultiPinSetState(rpset, 3, DECA0Pin, LIBRP_LOW, DECA1Pin, LIBRP_HIGH, DECA2Pin, LIBRP_LOW) != LIBRP_OK) {
       fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    // A3 is always LOW

    return LIBRP_OK;
}

int select_EXP(librpSet *rpset){
    //printf("select MAX7301\n");
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_LOW) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    if (librp_DMultiPinSetState(rpset, 3, DECA0Pin, LIBRP_HIGH, DECA1Pin, LIBRP_LOW, DECA2Pin, LIBRP_LOW) != LIBRP_OK) {
        fprintf(stderr, "librp_DpinSetState failed!\n");
        return -1;
    }
//    if (librp_DPinSetState(rpset, DECLEPin, LIBRP_HIGH) != LIBRP_OK) {
//        fprintf(stderr, "librp_DpinSetState failed!\n");
//        return -1;
//    }
    // A3 is always LOW

    return LIBRP_OK;
}

static int reset_DAC16b(librpSet *rpset){
    int size = 1;
    byte buf[2];
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    buf[0] = PORT7 & WRITE2;
    buf[1] = 0x00; // reset condition

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORT7 & WRITE2;
    buf[1] = 0x01; // come out of reset condition

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

static int reset_ADC18b_AB(librpSet *rpset){
    int size = 1;
    byte buf[2];
    int status;

    struct spi_ioc_transfer xfer[size];

    memset(xfer, 0, sizeof xfer);

    buf[0] = PORT6 & WRITE2;
    buf[1] = 0x01; // reset condition; same signal used for both ADCs

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    buf[0] = PORT6 & WRITE2;
    buf[1] = 0x00; // come out of reset condition; same signal used for both ADCs

    xfer[0].tx_buf = (unsigned long)(buf);
    xfer[0].rx_buf = (unsigned long)(buf);
    xfer[0].len = 2;

    status = ioctl(rpset->spi_fd, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    return LIBRP_OK;
}

/* vim: set cin et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 */

