
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
#include <arpa/inet.h> // inet_addr()
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h> // bzero()
#include <sys/socket.h>
#include <unistd.h> // read(), write(), close()


#define MAX 100

double
simple_link_get_value(int sock, double parameter, double *result2)
{
    char buff[MAX];
    char part1[MAX];
    char part2[MAX];
    char *delimiter;
    double realpart = 0, imagpart = 0;
    int n, size;
    
    snprintf(buff, MAX, "%g", parameter);
    //printf("w\n");
    write(sock, buff, sizeof(buff));
    bzero(buff, sizeof(buff));
    //printf("r\n");
    n = read(sock, buff, sizeof(buff));
    if (n>50 && n<80) {
    //printf("Recieved %s %d\n", buff, n);

    delimiter = strstr(buff+5, "0.000000000000000");
    size = delimiter-buff;
    if (size>MAX) size = MAX;
    snprintf(part1, size, "%s", buff);
    realpart = atof(part1);
    snprintf(part2, MAX, "%s", delimiter+22);
    imagpart = atof(part2);
    //printf("real %g imag %g\n", realpart, imagpart);
    }
    *result2 = imagpart;
    return realpart;
}

