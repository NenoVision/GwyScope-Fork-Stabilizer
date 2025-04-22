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

#ifndef LUAF
#define LUAF

#include "data.h"

#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>


int gws_clear(lua_State *L);
int gws_get_nvals(lua_State *L);
int gws_get_scan_param(lua_State *L);
int gws_check_if_stopped(lua_State *L);
int gws_check_if_paused(lua_State *L);
int gws_get_x(lua_State *L);
int gws_get_y(lua_State *L);
int gws_get_z(lua_State *L);
int gws_get_e(lua_State *L);
int gws_get_in(lua_State *L);
int gws_get_t(lua_State *L);
int gws_get_z_at(lua_State *L);
int gws_get_entry(lua_State *L);
int gws_move_to(lua_State *L);
int gws_set_feedback(lua_State *L);
int gws_set_speed(lua_State *L);
int gws_get_speed(lua_State *L);
int gws_store_point(lua_State *L);
int gws_set_fm_feedback(lua_State *L);
int gws_set_fm_amplitude_feedback(lua_State *L);
int gws_set_gen_amplitude(lua_State *L);
int gws_set_pidamplitude_setpoint(lua_State *L);
int gws_set_kpfm_feedback(lua_State *L);
int gws_set(lua_State *L);
int gws_set_output(lua_State *L);
int gws_set_zpiezo(lua_State *L);
int gws_usleep(lua_State *L);
int gws_scan_and_store(lua_State *L);
int gws_scan_and_store_lift(lua_State *L);
int gws_set_zpiezo_to_actual(lua_State *L);
int gws_get(lua_State *L);
int gws_get_correlation_length(lua_State *L);
int gws_get_period(lua_State *L);
int gws_get_optimum_angle(lua_State *L);

int gws_stabilize_setpoint(lua_State* L);

void register_functions(lua_State *L);
void run_lua_scan(HWData *hw);

#endif


