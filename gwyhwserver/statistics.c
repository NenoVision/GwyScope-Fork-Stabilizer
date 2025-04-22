/*
 *  hwserver: a simple implentation of Gwyfile compatible server for RP AFM operation
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

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include "statistics.h"

#define MAX(a, b) ((b) > (a) ? (b) : (a))
#define MIN(a, b) ((b) < (a) ? (b) : (a))
#define ARRAY_LENGTH(a) (sizeof(a)/sizeof(a[0]))
#define ARRAY_CLEAR(array, n) memset((array), 0, (n)*sizeof((array)[0]))
#define ARRAY_ASSIGN(dest, source, n) memcpy((dest), (source), (n)*sizeof((dest)[0]))
#define CMODULUS(z) (creal(z)*creal(z) + cimag(z)*cimag(z))

static int
smooth_upper_bound(int n)
{
    static const int primes[] = { 2, 3, 5, 7 };

    int r;

    for (r = 1; ; ) {
        int j;

        for (j = 0; j < (int)ARRAY_LENGTH(primes); j++) {
            int p;

            p = primes[j];
            while (n % p == 0) {
                n /= p;
                r *= p;
            }
        }

        if (n == 1)
            return r;

        n++;
    }
}

static int
nice_size_for_fftw(int size)
{
    if (size <= 16)
        return size;

    size = smooth_upper_bound(size);
    if (size % 2)
        size = smooth_upper_bound(size+1);

    return size;
}

static void
simple_window_welch(fftw_complex *data, int ndata)
{
    int i;

    for (i = 0; i < ndata; i++) {
        double x = 2.0*(i + 0.5)/ndata - 1.0;
        data[i] *= 1 - x*x;
    }
}

static void
get_line_coeffs(const double *data, int ndata,
                double *a, double *b)
{
    double s0z = 0.0, s1z = 0.0;
    double mean_i = 0.5*(ndata - 1);
    double s_ii = ndata/3.0 * (ndata*ndata - 1)/4.0;
    int i;

    for (i = 0; i < ndata; i++) {
        s0z += data[i];
        s1z += data[i]*(i - mean_i);
    }

    *b = s1z/s_ii;
    *a = s0z/ndata - *b*mean_i;
}

static void
line_level_into_complex(const double *data, int ndata,
                        fftw_complex *cdata, int nc)
{
    double a, b;
    int i;

    get_line_coeffs(data, ndata, &a, &b);
    for (i = 0; i < ndata; i++)
        cdata[i] = data[i] - (a + b*i);
    ARRAY_CLEAR(cdata + ndata, nc - ndata);
}

// estimate correlation length from a profile, in pixels
double
get_correlation_length(const double *data, int ndata)
{
    static int seen = 1;

    int size = nice_size_for_fftw(2*ndata);
    fftw_complex *r = fftw_alloc_complex(size);
    fftw_complex *f = fftw_alloc_complex(size);
    fftw_plan plan;
    double m, x = -1.0, threshold;
    int i;

    plan = fftw_plan_dft_1d(size, r, f, FFTW_FORWARD, FFTW_DESTROY_INPUT | FFTW_ESTIMATE);

    line_level_into_complex(data, ndata, r, size);
    if (!seen) {
        for (i = 0; i < ndata; i++)
            printf("A %d %g\n", i, creal(r[i]));
    }

    fftw_execute(plan);

    for (i = 0; i < ndata; i++)
        r[i] = CMODULUS(f[i]);

    fftw_execute(plan);

    fftw_destroy_plan(plan);
    fftw_free(r);

    if (!seen) {
        printf("\n\n");
        for (i = 0; i < ndata; i++)
            printf("A %d %g\n", i, creal(f[i]));
    }

    m = creal(f[0]);
    threshold = 0.2*m;
    for (i = 1; i < size; i++) {
        if (creal(f[i]) < threshold)
            break;
    }

    //printf("%g %g (%g)\n", creal(f[i-1]), creal(f[i]), threshold);
    if (i < size && creal(f[i-1]) > creal(f[i]))
        x = i-1 + (creal(f[i-1]) - threshold)/(creal(f[i-1] - f[i]));
    if (x < 0.1 || x > size)
        x = i;

    fftw_free(f);

    //fprintf(stderr, "RAW T %g\n", x);
    seen = 1;

    return x;
}

/* Refined FFT from frequency f0 to f1, m points (zeroth corresponding to f0, (m-1)th to f1).
 * Using Bluestein's algorithm. */
static void
zoom_fft(const fftw_complex *d, int n, fftw_complex *f, int m, double f0, double f1)
{
    /* The range of chirp coefficient w indices is -n+1,-n+2,...,m-2,m-1.
     * The range of data d indices is 0,1,...,n-1.
     * We only need convolution result to be correct for indices 0,1,...,m-1.
     * This means we do not have to pad, i.e. the minimum transform length is m+n-1. */
    int minsize = m + n - 1;
    int size = nice_size_for_fftw(minsize);
    fftw_complex *x = fftw_alloc_complex(size);
    fftw_complex *w = fftw_alloc_complex(size);
    fftw_complex *fx = fftw_alloc_complex(size);
    fftw_complex *fw = fftw_alloc_complex(size);
    fftw_plan fplan = fftw_plan_dft_1d(size, x, fw, FFTW_FORWARD, FFTW_DESTROY_INPUT | FFTW_ESTIMATE);
    fftw_plan bplan = fftw_plan_dft_1d(size, fx, x, FFTW_BACKWARD, FFTW_DESTROY_INPUT | FFTW_ESTIMATE);
    double q, D = (m - 1)/(f1 - f0);
    int k, mm = MIN(m, n);

    /* Precompute the factors w_k = exp(-2πik²/(ND)) */
    w[0] = 1.0;
    q = M_PI/(n*D);
    for (k = 1; k < mm; k++)
        w[k] = w[size - k] = cexp(q*k*k*I);
    /* Only one of the two following actually does something, depending on which of m and n is larger. */
    for (k = n; k < m; k++)
        w[k] = cexp(q*k*k*I);
    for (k = m; k < n; k++)
        w[size - k] = cexp(q*k*k*I);
    ARRAY_CLEAR(w + m, size - minsize);

    /* Transform premultiplied data. */
    x[0] = d[0];
    q = -2.0*M_PI*f0/n;
    for (k = 1; k < n; k++)
        x[k] = cexp(q*k*I)*conj(w[size - k])*d[k];
    ARRAY_CLEAR(x + n, size - n);
    fftw_execute(fplan);
    ARRAY_ASSIGN(fx, fw, size);

    /* Transform chirp w. */
    ARRAY_ASSIGN(x, w, size);
    fftw_execute(fplan);
    fftw_destroy_plan(fplan);

    /* Multiply and transform back. */
    for (k = 0; k < size; k++)
        fx[k] *= fw[k];
    fftw_execute(bplan);
    fftw_destroy_plan(bplan);
    fftw_free(fx);
    fftw_free(fw);

    /* And finally post-multiply by w.
     * FIXME: q may be wrong here by some power of size. */
    q = 1.0/size/sqrt(n);
    for (k = 0; k < m; k++)
        f[k] = q*x[k]*conj(w[k]);
    fftw_free(x);
    fftw_free(w);
}

// get period from a profile, in pixels
double
get_period(const double *data, int ndata)
{
    int size = nice_size_for_fftw(2*ndata);
    fftw_complex *r = fftw_alloc_complex(size);
    fftw_complex *f = fftw_alloc_complex(size);
    fftw_plan plan;
    double m, mm, f0, f1, fbest, Tbest = -1.0;
    int i, j, ibest;

    plan = fftw_plan_dft_1d(size, r, f, FFTW_FORWARD, FFTW_PRESERVE_INPUT | FFTW_ESTIMATE);

    line_level_into_complex(data, ndata, r, size);
    simple_window_welch(r, ndata);

    fftw_execute(plan);

    fftw_destroy_plan(plan);

    /*
    for (i = 0; i < size/2; i++)
        printf("%g\n", CMODULUS(f[i]));
    printf("\n\n");
    */

    // Eliminate the peak at 0 even if it is the highest.
    m = CMODULUS(f[0]);
    for (i = 1; i < size/5; i++) {
        mm = CMODULUS(f[i]);
        if (!(mm < m))
            break;
        m = mm;
    }
    //printf("start at %d\n", i+1);

    // We have no idea. The period may be very long or very short or nonexistent…
    if (i == size/5)
        goto end;

    // Find the other highest value.
    ibest = i;
    m = CMODULUS(f[ibest]);
    for (j = i+1; j < size/2; j++) {
        mm = CMODULUS(f[j]);
        if (mm > m) {
            ibest = j;
            m = mm;
        }
    }
    //printf("highest value at %d\n", ibest);

    // We have no idea. The period may be very long or very short or nonexistent…
    if (ibest == size/2)
        goto end;

    fbest = ibest * (double)ndata/size;
    //printf("frequency estimate %g\n", fbest);
    //printf("period estimate %g\n", ndata/fbest);
    f0 = fbest - 1.2;
    f1 = fbest + 1.2;
    zoom_fft(r, ndata, f, ndata, f0, f1);

    ibest = 0;
    m = CMODULUS(f[ibest]);
    for (j = 1; j < ndata; j++) {
        mm = CMODULUS(f[j]);
        if (mm > m) {
            ibest = j;
            m = mm;
        }
    }
    //printf("refined maximum at index %d\n", ibest);

    fbest = f0 + (f1 - f0)*ibest/(ndata - 1.0);
    //printf("refined frequency estimate %g\n", fbest);
    Tbest = ndata/fbest;

end:
    fftw_free(f);
    fftw_free(r);

    return Tbest;
}

int
refine_maximum(const double *y, double *x)
{
    double b, D;

    *x = 0.0;
    D = y[2] + y[0] - 2.0*y[1];
    b = 0.5*(y[0] - y[2]);
    if (D == 0.0 || fabs(D) < fabs(b))
        return 0;

    *x = b/D;
    return 1;
}

// Get offset of second with respect to first (positive means second is shifted to the right), in pixels.
double
get_mutual_offset(const double *first, const double *second, int ndata, double *score)
{
    int size = nice_size_for_fftw(2*ndata);
    fftw_complex *r = fftw_alloc_complex(size);
    fftw_complex *f = fftw_alloc_complex(size);
    fftw_complex *g = fftw_alloc_complex(size);
    fftw_plan plan;
    double x, m, mm, peak[3];
    int i, besti;

    plan = fftw_plan_dft_1d(size, r, f, FFTW_FORWARD, FFTW_DESTROY_INPUT | FFTW_ESTIMATE);

    line_level_into_complex(first, ndata, r, size);
    fftw_execute(plan);
    ARRAY_ASSIGN(g, f, size);

    line_level_into_complex(second, ndata, r, size);
    fftw_execute(plan);

    for (i = 0; i < size; i++)
        r[i] = conj(f[i])*g[i];
    fftw_execute(plan);

    fftw_destroy_plan(plan);
    fftw_free(g);

    besti = 0;
    m = creal(f[0]);
    for (i = 1; i <= ndata/4; i++) {
        mm = creal(f[size-i])/(size - i);
        if (mm > m) {
            besti = i;
            m = mm;
        }
        mm = creal(f[i])/(size - i);
        if (mm > m) {
            besti = i;
            m = mm;
        }
    }

    /*
    for (i = -ndata/4; i <= ndata/4; i++) {
        printf("%d %g\n", i, creal(f[(size+i) % size]));
    }
    */
    //printf("best i %d\n", besti);

    i = (besti + size-1) % size;
    peak[0] = creal(f[i])/(MAX(size - i, i));

    peak[1] = m/size;

    i = (besti + 1) % size;
    peak[2] = creal(f[i])/(MAX(size - i, i));

    //printf("peak [%g %g %g]\n", peak[0], peak[1], peak[2]);
    refine_maximum(peak, &x);
    //printf("refinement %g\n", x);
    x += (besti >= size/2 ? size - besti : besti);

    if (score) {
        double s12 = 0.0, s1 = 0.0, s2 = 0.0;

        /* Fix the sign of the integer offset. */
        if (besti >= size/2) {
            const double *t = first;
            first = second;
            second = t;
            besti = size - besti;
        }

        line_level_into_complex(first, ndata, r, ndata);
        line_level_into_complex(second, ndata, f, ndata);

        for (i = 0; i < ndata - besti; i++) {
            double z1 = creal(r[i]), z2 = creal(f[i + besti]);
            s12 += z1*z2;
            s1 += z1*z1;
            s2 += z2*z2;
        }
        if (s1 <= 0.0)
            s1 = 1.0;
        if (s2 <= 0.0)
            s2 = 1.0;

        *score = s12/sqrt(s1*s2);
    }

    fftw_free(r);
    fftw_free(f);

    return x;
}

void
fit_parabola(const double *xdata, const double *ydata, int ndata,
             double *coeffs)
{
    double s1 = ndata, sx = 0.0, sx2 = 0.0, sx3 = 0.0, sx4 = 0.0, sy = 0.0, sxy = 0.0, sx2y = 0.0, D;
    double p3, p4, p5;
    int i;

    if (ndata < 3) {
        fprintf(stderr, "too few points to fit parabola\n");
        ARRAY_CLEAR(coeffs, 3);
        return;
    }

    for (i = 0; i < ndata; i++) {
        double x = xdata[i], x2 = x*x, y = ydata[i];

        sx += x;
        sx2 += x2;
        sx3 += x2*x;
        sx4 += x2*x2;
        sy += y;
        sxy += x*y;
        sx2y += x2*y;
    }

    D = s1*sx2*sx4 + 2.0*sx*sx2*sx3 - s1*sx3*sx3 - sx*sx*sx4 - sx2*sx2*sx2;
    if (D == 0.0) {
        fprintf(stderr, "too few points to fit parabola\n");
        ARRAY_CLEAR(coeffs, 3);
        return;
    }
    p3 = sx*sx2 - s1*sx3;
    p4 = sx*sx3 - sx2*sx2;
    p5 = sx2*sx3 - sx*sx4;
    coeffs[0] = (sy*(sx2*sx4 - sx3*sx3) + sxy*p5 + sx2y*p4)/D;
    coeffs[1] = (sy*p5 + sxy*(s1*sx4 - sx2*sx2) + sx2y*p3)/D;
    coeffs[2] = (sy*p4 + sxy*p3 + sx2y*(s1*sx2 - sx*sx))/D;
}

/* vim: set cin columns=120 tw=118 et ts=4 sw=4 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 : */
