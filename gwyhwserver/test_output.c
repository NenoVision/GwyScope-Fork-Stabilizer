/*
 *  hwserver: a simple implentation of Gwyfile compatible server for RP AFM operation
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

#include <math.h>

#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <errno.h> 
#include <signal.h>

#include "cmirp.h"

#define VAL (1.0)
       

int main(int argc, char *argv[])
{
     int i;
     librpSet rp;

     printf("starting RP API...\n");
     //if (!librp_Init(&rp, LIBRP_RANGE_SMALL, 2, 1, LIBRP_RANGE_SMALL)) printf("done.\n");
     if (!librp_Init(&rp, LIBRP_RANGE_FULL, 2, 1, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL, LIBRP_RANGE_FULL)) printf("done.\n");
     else {
        fprintf(stderr, "Error: cannot start RP API, exiting\n");
        return 0;
     }

     //init state
     librp_SetState(&rp, LIBRP_MODE_OFF, LIBRP_MODE_OFF, 0, 0); //mode, feedback, swap, zslow

     for (i=0; i<5; i++)
     {

        librp_SetDAC(&rp, LIBRP_CH_1, -VAL);
        librp_SetDAC(&rp, LIBRP_CH_2, -VAL);
        usleep(3000000); 
        librp_SetDAC(&rp, LIBRP_CH_1, VAL);
        librp_SetDAC(&rp, LIBRP_CH_2, VAL);
        usleep(3000000); 
  
     }

     librp_SetDAC(&rp, LIBRP_CH_1, 0);
     librp_SetDAC(&rp, LIBRP_CH_2, 0);


     if (!librp_Close(&rp)) printf("# RP API closed\n");
     else {
         fprintf(stderr, "Error: cannot stop RP API\n");
     
     }

     return 0;
}

