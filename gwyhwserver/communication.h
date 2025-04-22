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

#ifndef HWCOMM
#define HWCOMM

#include "data.h"

#ifdef FAKE
#include "myrpfake.h"
#else
#include "cmirp.h"
#endif


void *connection_handler(void *hwdata);

void FillSwaps(HWData *hw, int swap_in[12], int swap_out[12]);
void set_mode_and_feedback(HWData *hw);
void set_fm_feedback(HWData *hw);
void set_zpiezo(HWData *hw, double value, bool force);
void set_filter(HWData *hw, int channel);
void set_pid(HWData *hw);
void set_setpoint(HWData *hw);
void set_phaseshift(HWData *hw);
void set_freq(HWData *hw, int channel);
void set_freq3(HWData *hw, double frequency);
void run_motor(HWData *hw, int motor_number, double motor_distance);
void stop_scanning_script_thread(HWData *hw);


#endif //HWCOMM
