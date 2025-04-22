/*
 * $Id: clientserver.h,v 1.10 2022/07/03 15:12:06 klapetek Exp $
 *
 * This file provides some common functions for client.c and server.c.
 *
 * I, the copyright holder of this work, release this work into the public
 * domain.  This applies worldwide.  In some countries this may not be legally
 * possible; if so: I grant anyone the right to use this work for any purpose,
 * without any conditions, unless such conditions are required by law.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/types.h>
#include "gwyfile.h"

static void
ensure_buffer_size(char **buffer, size_t *bufsize, size_t len)
{
    if (len <= *bufsize)
        return;

    if (2*(*bufsize) >= len)
        *bufsize *= 2;
    else
        *bufsize = len;

    *buffer = realloc(*buffer, *bufsize);
}

#ifdef __GNUC__
__attribute__((unused))
#endif
static GwyfileObject*
recv_gwyfile_message(int sock, char **buffer, size_t *bufsize, int *err)
{
    GwyfileError *error = NULL;
    GwyfileObject *message;
    size_t objsize, len = 0;
    ssize_t read_size;

    /* Purposedly make the buffer small at the beginning. */
    if (!*bufsize || !*buffer)
        ensure_buffer_size(buffer, bufsize, 16);

    *err = -1;
//    printf("Trying to read %u bytes.\n", (unsigned)(*bufsize - len));
    while ((read_size = read(sock, *buffer + len, *bufsize - len)) > 0) {
//        printf("Read %ld bytes from socket %d\n", (long int)read_size, sock);
        len += read_size;
        /* Process the message we have in the buffer. */
        if (!(message = gwyfile_object_read_memory(*buffer, len, &objsize, &error))) {
            assert(error);
            if (error->domain == GWYFILE_ERROR_DOMAIN_DATA && error->code == GWYFILE_ERROR_TRUNCATED) {
                /* There is an incomplete message. */
                gwyfile_error_clear(&error);
                if (len == *bufsize) {
                    ensure_buffer_size(buffer, bufsize, 2*(*bufsize));
//                    fprintf(stderr, "Enlarging buffer to %lu.\n", (unsigned long)(*bufsize));
                }
                continue;
            }
            else {
                fprintf(stderr, "Failed to read message. %s\n", error->message);
                fprintf(stderr, "Giving up.\n");
                gwyfile_error_clear(&error);
                return NULL;
            }
        }
//        printf("Obtained message of %lu bytes\n", (unsigned long int)objsize);
        *err = 0;
        return message;
    }
    /* If either read_size < 0 or we have still something in buffer then an error occurred. */
    if (read_size < 0) {
        fprintf(stderr, "recv() failed: %s.\n", strerror(errno));
        return NULL;
    }
    else if (len > 0) {
        fprintf(stderr, "Incomplete message.\n");
        return NULL;
    }

    /* Returning NULL with zero error means there is simply nothing more to read. */
    *err = 0;
    return NULL;
}

#ifdef __GNUC__
__attribute__((unused))
#endif
static GwyfileObject*
recv_gwyfile_message2(int sock, char **buffer, size_t *bufsize, size_t *have_bytes, int *err)
{
    GwyfileError *error = NULL;
    GwyfileObject *message;
    size_t objsize;
    ssize_t read_size;
    int first_try = 1;

    /* Purposedly make the buffer small at the beginning. */
    if (!*bufsize || !*buffer)
        ensure_buffer_size(buffer, bufsize, 16);

    while (1) {
        if (!first_try || !*have_bytes) {
        //    printf("Trying to read %u bytes.\n", (unsigned)(*bufsize - *have_bytes));
            if ((read_size = read(sock, *buffer + *have_bytes, *bufsize - *have_bytes)) <= 0) {
                if (errno != EINTR) break;
            } else
               *have_bytes += read_size;
        }
        else {
         //   printf("Trying to use the %lu bytes left in buffer.\n", (unsigned long int)(*have_bytes));
        }
        /* Process the message we have in the buffer. */
        if ((message = gwyfile_object_read_memory(*buffer, *have_bytes, &objsize, &error))) {
            //printf("Obtained message of %lu bytes (%lu bytes remaining)\n",
            //       (unsigned long int)objsize, (unsigned long int)(*have_bytes - objsize));
            /* Handled piece-wise objects by keeping whatever extra data we received in the buffer. */
            memmove(*buffer, *buffer + objsize, *have_bytes - objsize);
            *have_bytes -= objsize;
            *err = 0;
            return message;
        }
        assert(error);
        if (!(error->domain == GWYFILE_ERROR_DOMAIN_DATA && error->code == GWYFILE_ERROR_TRUNCATED))
            goto bad_message;
        /* There is an incomplete message.  If it is incomplete because we would block, just pass it up.
         * Otherwise enlarge the buffer. */
        gwyfile_error_clear(&error);
        if (*have_bytes == *bufsize) {
            ensure_buffer_size(buffer, bufsize, 2*(*bufsize));
           // fprintf(stderr, "Enlarging buffer to %lu.\n", (unsigned long)(*bufsize));
        }
        first_try = 0;
    }
    /* If either read_size < 0 or we have still something in buffer then an error occurred. */
    *err = errno;
    printf("recv cycle done, errno: %s\n", strerror(errno));
    if (read_size < 0) {
        fprintf(stderr, "recv() 2 failed: %s.\n", strerror(errno));
        return NULL;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) { 
        *err = EAGAIN;
        fprintf(stderr, "Incomplete message so far.\n");
        return NULL;
    }

    /* Returning NULL with zero error means there is simply nothing more to read. */
    if (errno)
        fprintf(stderr, "We got zero from recv() with unexpected error %s.\n", strerror(errno));
    return NULL;

bad_message:
    fprintf(stderr, "Failed to read message. %s\n", error->message);
    fprintf(stderr, "Giving up.\n");
    gwyfile_error_clear(&error);
    *err = EINVAL;
    return NULL;
}

#ifdef __GNUC__
__attribute__((unused))
#endif
static int
send_gwyfile_message(GwyfileObject *message, int sock, char **buffer, size_t *bufsize)
{
    ssize_t pos, write_size;
    size_t len;

    len = gwyfile_object_size(message);
    ensure_buffer_size(buffer, bufsize, len);
    len = gwyfile_object_write_memory(message, *buffer, *bufsize, NULL);
    assert(len);
    pos = 0;
    while (len > 0) {
        write_size = write(sock, *buffer + pos, len);
        if (write_size < 0 && errno != EINTR) {
            int saved_errno = errno;
            fprintf(stderr, "send() failed: %s.\n", strerror(errno));
            errno = saved_errno;
            return -1;
        }
        len -= write_size;
        pos += write_size;
    }

    return 0;
}

/* vim: set cin et ts=4 sw=4 columns=120 tw=119 cino=>1s,e0,n0,f0,{0,}0,^0,\:1s,=0,g1s,h0,t0,+1s,c3,(0,u0 : */

