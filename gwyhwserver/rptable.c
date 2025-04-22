
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
#include <math.h>

#include "clientserver.h"
#include "rptable.h"

void
rpt_set_zero(int sock)
{
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "set"),
                                 gwyfile_item_new_double("zero", 0.0),
                                 NULL);

    if (send_gwyfile_message(message, sock, &buffer, &bufsize) < 0)
    {
        printf("Error at gwyfile message send\n");
        return;
    }

    answer = recv_gwyfile_message(sock, &buffer, &bufsize, &err);
    if (err < 0)
    {
        printf("Error at gwyfile message receive\n");
        return;
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}

void
rpt_set_pid(int sock, double pid_p, double pid_i, double pid_d)
{
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "set"),
                                 gwyfile_item_new_double("pid_p", pid_p),
                                 gwyfile_item_new_double("pid_i", pid_i),
                                 gwyfile_item_new_double("pid_d", pid_d),
                                 NULL);

    if (send_gwyfile_message(message, sock, &buffer, &bufsize) < 0)
    {
        printf("Error at gwyfile message send\n");
        return;
    }

    answer = recv_gwyfile_message(sock, &buffer, &bufsize, &err);
    if (err < 0)
    {
        printf("Error at gwyfile message receive\n");
        return;
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}


void
rpt_set_feedback(int sock, bool feedback)
{   
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;
    
    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "set"),
                                 gwyfile_item_new_bool("feedback", feedback),
                                 NULL);
    
    if (send_gwyfile_message(message, sock, &buffer, &bufsize) < 0)
    {   
        printf("Error at gwyfile message send\n");
        return;
    }
    
    answer = recv_gwyfile_message(sock, &buffer, &bufsize, &err);
    if (err < 0)
    {   
        printf("Error at gwyfile message receive\n");
        return;
    }
    
    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}

void
rpt_move_to(int sock, double pos)
{
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "move_to"),
                                 gwyfile_item_new_double("pos", pos),
                                 NULL);

    if (send_gwyfile_message(message, sock, &buffer, &bufsize) < 0)
    {
        printf("Error at gwyfile message send\n");
        return;
    }

    answer = recv_gwyfile_message(sock, &buffer, &bufsize, &err);
    if (err < 0)
    {
        printf("Error at gwyfile message receive\n");
        return;
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}

double
rpt_get_values(int sock)
{
    GwyfileObject *message;
    GwyfileItem *valueitem;
    char *buffer = NULL;
    size_t bufsize = 0;
    GwyfileObject *answer;
    int err;
    double pos;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "read"),
                                 NULL);

    if (send_gwyfile_message(message, sock, &buffer, &bufsize) < 0)
    {
        printf("Error at gwyfile message send\n");
        return 0;
    }

    answer = recv_gwyfile_message(sock, &buffer, &bufsize, &err);
    if (err < 0)
    {
        printf("Error at gwyfile message receive\n");
        return 0;
    }

    valueitem = gwyfile_object_get(answer, "pos");
    if (!valueitem) fprintf(stderr, "Error! read message does not contain pos\n");
    else {
        pos = gwyfile_item_get_double(valueitem);
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);

    return pos;
}


