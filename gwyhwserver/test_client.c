/*
 *  hwserver: a simple implentationa of Gwyfile compatible server for RP AFM operation
 *  Copyright (C) 2022 Petr Klapetek, Miroslav Valtr, David Nečas
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

const char *servername = "127.0.0.1";

static void
get_hwtime(int sock)
{
    GwyfileObject *message;
    GwyfileItem *valueitem;
    char *buffer = NULL;
    size_t bufsize = 0;
    GwyfileObject *answer;
    int err;
    double hwtime;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "get"),
                                 gwyfile_item_new_double("hwtime", 0.0),
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
     
    valueitem = gwyfile_object_get(answer, "hwtime");
    if (!valueitem) fprintf(stderr, "Error! read message does not contain hwtime\n");
    else {
        hwtime = gwyfile_item_get_double(valueitem);
        printf("HWtime: %g\n", hwtime);
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}

static void
get_values(int sock, int print)
{
    GwyfileObject *message;
    GwyfileItem *valueitem;
    char *buffer = NULL;
    size_t bufsize = 0;
    GwyfileObject *answer;
    int err;
    double errorsignal;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "read"),
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
     
    valueitem = gwyfile_object_get(answer, "x");
    if (!valueitem) fprintf(stderr, "Error! read message does not contain hwtime\n");
    else {
        errorsignal = gwyfile_item_get_double(valueitem);
        if (print) printf("error signal: %g\n", errorsignal);
    }

    gwyfile_object_free(message);
    gwyfile_object_free(answer);
    free(buffer);
}

static void
stop_scan(int sock)
{
    GwyfileObject *message;
    char *buffer = NULL;
    size_t bufsize = 0;
    GwyfileObject *answer;
    int err;

    message = gwyfile_object_new("GS",
                                 gwyfile_item_new_string_copy("todo", "stop_scan"),
                                 gwyfile_item_new_double("hwtime", 0.0),
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

static void
run_scan_adaptive(int sock, int xres, int yres)
{
    GwyfileObject *message;
    GwyfileObject *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;
    double *xypos, *sendpos;
    int i, j, n, from, to;
    int npos = xres*yres;
    int npossend = ceil((int)npos/1000.0);
    int sendsize;
    
    printf("%d data will be sent by %d stripes\n", npos, npossend);

    xypos = (double *)malloc(2*npos*sizeof(double));
    sendpos = (double *)malloc(2*npossend*sizeof(double));

    n = 0;
    for (i=0; i<xres; i++) {
       for (j=0; j<yres; j++) {
           xypos[n++] = (double)i*1e-9;
           xypos[n++] = (double)j*1e-9;
       }
    }

    n = 0;
    do {
       i = 0;
       from = n;
       do {
           sendpos[i++] = xypos[2*n];
           sendpos[i++] = xypos[2*n+1];
           n++;
       } while ((n<npos) && i<(2*npossend));
       to = n;
       sendsize = to-from;
       //printf("from: %d to: %d  n %d\n", from, to, sendsize);

       message = gwyfile_object_new("GS",
                           gwyfile_item_new_string_copy("todo", "set_scan_path_data"),
                           gwyfile_item_new_int32("n", npos),
                           gwyfile_item_new_int32("from", from),
                           gwyfile_item_new_int32("to", to),
                           gwyfile_item_new_double_array_copy("xydata", sendpos, 2*npossend),
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

    } while (n<npos);

    message = gwyfile_object_new("GS",
                           gwyfile_item_new_string_copy("todo", "run_scan_path"),
                           gwyfile_item_new_int32("n", npos),
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

    free(xypos);
    free(sendpos);
}


void move_table(int sock, double x, double y)
{
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;

    message = gwyfile_object_new("GS",
                           gwyfile_item_new_string_copy("todo", "move_to"),
                           gwyfile_item_new_double("xreq", x),
                           gwyfile_item_new_double("yreq", y),
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
}

void set_speed(int sock, double val)
{
    GwyfileObject *message, *answer;
    char *buffer = NULL;
    size_t bufsize = 0;
    int err;

    message = gwyfile_object_new("GS",
                           gwyfile_item_new_string_copy("todo", "set_scan"),
                           gwyfile_item_new_double("speed", val/1.0e6), //entered in um/s converted to m/s
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

}



int main(int argc, char *argv[])
{
    int i;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    int portno, sock;
    if (argc < 2) {
        printf("Usage: server PORTNUMBER\n");
        exit(EXIT_SUCCESS);
    }
    portno = atoi(argv[1]);
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Cannot create socket");
        exit(EXIT_FAILURE);
    }
    server = gethostbyname(servername);
    if (server == NULL) {
        fprintf(stderr, "Cannot resolve hostname %s.\n", servername);
        exit(EXIT_FAILURE);
    }
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy(server->h_addr, &serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        fprintf(stderr, "Cannot connect to address %s, port %d: %s\n", servername, portno, strerror(errno));
        exit(EXIT_FAILURE);
    }

//    get_hwtime(sock);

//    for (i=0; i<100000000; i++) {
//       get_values(sock, 1);
//       usleep(100000);
//    }

  //  run_scan_adaptive(sock, 500, 500);
  //  sleep(10);
     //stop_scan(sock);

    printf("set speed\n");
    set_speed(sock, 3); //in um/s
    sleep(1);

    printf("moving forward\n");
    move_table(sock, 10e-6, 0);
    sleep(10); 
    printf("moving back\n");
    move_table(sock, 0, 0);  
    sleep(10); 

    return 0;
}


