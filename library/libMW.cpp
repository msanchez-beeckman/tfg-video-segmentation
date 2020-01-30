//
//  mw.cpp
//  frdSources
//
//  Created by Antoni Buades on 12/09/2013.
//  Copyright (c) 2013 Antoni Buades. All rights reserved.
//

#include "libMW.h"


namespace libMW
{
    
    
    
int mwerrcnt = 0;

char *mwname = NULL;            /* Name of the module */
char *mwgroup = NULL;           /* Group of the module */
int mwdbg = FALSE;


void mwerror(int code, int exit_code, char *fmt, ...)
{
/*
    va_list marker;
    
    va_start(marker, fmt);
    
    switch (code)
    {
        case WARNING:
            fprintf(stderr, "megawave warning (%s) : ", mwname);
            vfprintf(stderr, fmt, marker);
            break;
        case ERROR:
            fprintf(stderr, "megawave error (%s) : ", mwname);
            vfprintf(stderr, fmt, marker);
            mwerrcnt++;
            break;
        case FATAL:
            fprintf(stderr, "megawave fatal (%s) : ", mwname);
            vfprintf(stderr, fmt, marker);
            fprintf(stderr, "Exit.\n");
            exit(exit_code);
            break;
        case INTERNAL:
            fprintf(stderr, "megawave internal (%s) : ", mwname);
            vfprintf(stderr, fmt, marker);
            fprintf(stderr, "Exit.\n");
            exit(exit_code);
            break;
        case USAGE:
            fprintf(stderr, "Bad parameter; use the '--help' option for details");
            exit(exit_code);
            break;
        default:
            mwerror(FATAL, 1, "Bad usage of mwerror : code %d is unknown\n",
                    code);
            break;
    }
    va_end(marker);
*/
}





void mwdebug(char *fmt, ...)
{
/*
    if (mwdbg)
    {
        va_list marker;
        
        va_start(marker, fmt);
        fprintf(stderr, "<dbg> ");
        vfprintf(stderr, fmt, marker);
        va_end(marker);
    }
*/
}





Fsignal mw_new_fsignal()
{
    Fsignal signal;
    
    if (!(signal = (Fsignal) (malloc(sizeof(struct fsignal)))))
    {
        mwerror(ERROR, 0, "[mw_new_fsignal] Not enough memory\n");
        return (NULL);
    }
    
    signal->size = 0;
    signal->allocsize = 0;
    signal->firstp = signal->lastp = 0;
    signal->param = 0.0;
    
    signal->scale = 1.0;
    signal->shift = 0.0;
    signal->gain = 1.0;
    signal->sgrate = 1.0;
    signal->bpsample = 8 * sizeof(float);
    
    strcpy(signal->cmt, "?");
    strcpy(signal->name, "?");
    
    signal->values = NULL;
    
    return (signal);
}

/* allocates the values array for N samples */

Fsignal mw_alloc_fsignal(Fsignal signal, int N)
{
    int mem_size;
    
    if (signal == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fsignal] cannot alloc array : "
                "fsignal structure is NULL\n");
        return (NULL);
    }
    
    mem_size = N * sizeof(float);
    if (mem_size <= 0)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fsignal] Attempts to alloc a fsignal "
                "with null size\n");
        return NULL;
    }
    
    if (signal->values != NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fsignal] Attempts to alloc a fsignal "
                "which is already allocated\n");
        return (NULL);
    }
    
    signal->values = (float *) malloc(mem_size);
    if (signal->values == NULL)
    {
        signal->size = 0;
        signal->allocsize = 0;
        mwerror(ERROR, 0, "[mw_alloc_fsignal] Not enough memory\n");
        return (NULL);
    }
    
    signal->size = N;
    signal->allocsize = mem_size;
    return (signal);
}

/* desallocate the array in the fsignal structure and the structure itself */

void mw_delete_fsignal(Fsignal signal)
{
    if (signal == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_delete_fsignal] cannot delete : "
                "fsignal structure is NULL\n");
        return;
    }
    
    if (signal->values != NULL)
        free(signal->values);
    signal->values = NULL;
    free(signal);
    signal = NULL;
}

/* Change the size of the allocated array */
/* May define the struct if not defined */
/* So you have to call it with signal = mw_change_fsignal(signal,...) */

Fsignal mw_change_fsignal(Fsignal signal, int N)
{
    int mem_size;
    
    if (signal == NULL)
        signal = mw_new_fsignal();
    if (signal == NULL)
        return (NULL);
    
    mem_size = N * sizeof(float);
    if (mem_size > signal->allocsize)
    {
        if (signal->values != NULL)
        {
            free(signal->values);
            signal->values = NULL;
        }
        if (mw_alloc_fsignal(signal, N) == NULL)
        {
            mw_delete_fsignal(signal);
            return (NULL);
        }
    }
    else
        signal->size = N;
    
    return (signal);
}

/* Clear the array of a fsignal with the value v */

void mw_clear_fsignal(Fsignal signal, float v)
{
    register float *ptr;
    register int l;
    
    if ((!signal) || (!signal->values))
    {
        mwerror(ERROR, 0,
                "[mw_clear_fsignal] NULL signal struct or NULL values array\n");
        return;
    }
    
    for (l = 1, ptr = signal->values; l <= signal->size; l++, ptr++)
        *ptr = v;
}

/* Copy the values array of a fsignal into another fsignal */

void mw_copy_fsignal_values(Fsignal in, Fsignal out)
{
    if ((!in) || (!out) || (!in->values) || (!out->values)
        || (in->size != out->size))
    {
        mwerror(ERROR, 0,
                "[mw_copy_fsignal_values] NULL input or output "
                "signal or signals of different sizes !\n");
        return;
    }
    
    memcpy(out->values, in->values, sizeof(float) * in->size);
}

/* Copy the header of a fsignal into another fsignal */

void mw_copy_fsignal_header(Fsignal in, Fsignal out)
{
    if ((!in) || (!out))
    {
        mwerror(ERROR, 0,
                "[mw_copy_fsignal_header] NULL input or output signal !\n");
        return;
    }
    out->firstp = in->firstp;
    out->lastp = in->lastp;
    out->param = in->param;
    out->scale = in->scale;
    out->shift = in->shift;
    out->gain = in->gain;
    out->sgrate = in->sgrate;
    out->bpsample = in->bpsample;
    strcpy(out->cmt, in->cmt);
}

/* Copy a fsignal into another fsignal */

void mw_copy_fsignal(Fsignal in, Fsignal out)
{
    if ((!in) || (!out) || (!in->values) || (!out->values)
        || (in->size != out->size))
    {
        mwerror(ERROR, 0,
                "[mw_copy_fsignal] NULL input or output signal "
                "or signals of different sizes !\n");
        return;
    }
    
    memcpy(out->values, in->values, sizeof(float) * in->size);
    mw_copy_fsignal_header(in, out);
}








Cimage mw_new_cimage()
{
    Cimage image;
    
    if (!(image = (Cimage) (malloc(sizeof(struct cimage)))))
    {
        mwerror(ERROR, 0, "[mw_new_cimage] Not enough memory\n");
        return (NULL);
    }
    
    image->nrow = image->ncol = 0;
    image->allocsize = 0;
    image->firstcol = image->lastcol = image->firstrow = image->lastrow = 0.0;
    
    image->scale = 1.0;
    strcpy(image->cmt, "?");
    strcpy(image->name, "?");
    
    image->gray = NULL;
    
    image->previous = NULL;
    image->next = NULL;
    
    return (image);
}

/* allocates the gray array */

Cimage mw_alloc_cimage(Cimage image, int nrow, int ncol)
{
    int size;
    
    if (image == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_cimage] cannot alloc plane : "
                "cimage structure is NULL\n");
        return (NULL);
    }
    
    size = nrow * ncol * sizeof(unsigned char);
    if (size <= 0)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_cimage] Attempts to alloc a cimage "
                "with null size\n");
        return (NULL);
    }
    
    if (image->gray != NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_cimage] Attempts to alloc a cimage "
                "which is already allocated\n");
        return (NULL);
    }
    
    image->gray = (unsigned char *) malloc(size);
    if (image->gray == NULL)
    {
        image->nrow = image->ncol = 0;
        image->allocsize = 0;
        mwerror(ERROR, 0, "[mw_alloc_cimage] Not enough memory\n");
        return (NULL);
    }
    
    image->nrow = nrow;
    image->ncol = ncol;
    image->allocsize = size;
    return (image);
}

/* desallocate the array in the cimage structure and the structure itself */

void mw_delete_cimage(Cimage image)
{
    if (image == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_delete_cimage] cannot delete : "
                "cimage structure is NULL\n");
        return;
    }
    if (image->gray != NULL)
        free(image->gray);
    image->gray = NULL;
    free(image);
    image = NULL;
}

/* Change the size of the allocated gray plane */
/* May define the struct if not defined */
/* So you have to call it with image = mw_change_cimage(image,...) */

Cimage mw_change_cimage(Cimage image, int nrow, int ncol)
{
    int size;
    
    if (image == NULL)
        image = mw_new_cimage();
    if (image == NULL)
        return (NULL);
    
    size = nrow * ncol * sizeof(unsigned char);
    if (size > image->allocsize)
    {
        if (image->gray != NULL)
        {
            free(image->gray);
            image->gray = NULL;
        }
        if (mw_alloc_cimage(image, nrow, ncol) == NULL)
        {
            mw_delete_cimage(image);
            return (NULL);
        }
    }
    else
    {
        image->nrow = nrow;
        image->ncol = ncol;
    }
    return (image);
}

/* Return the gray level value of a cimage at location (x,y) */
/* WARNING: this is a slow way to access to a pixel !        */

unsigned char mw_getdot_cimage(Cimage image, int x, int y)
{
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_getdot_cimage] NULL image struct "
                "or NULL gray plane... Return 0\n");
        return (0);
        
    }
    
    if ((x < 0) || (y < 0) || (x >= image->ncol) || (y >= image->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_getdot_cimage] Point (%d,%d) out of image... Return 0\n",
                x, y);
        return (0);
    }
    
    return (image->gray[y * image->ncol + x]);
}

/* Set the gray level value of a cimage at location (x,y) */
/* WARNING: this is a slow way to access to a pixel !     */

void mw_plot_cimage(Cimage image, int x, int y, unsigned char v)
{
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_plot_cimage] NULL image struct or NULL gray plane\n");
        return;
    }
    
    if ((x < 0) || (y < 0) || (x >= image->ncol) || (y >= image->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_plot_cimage] Point (%d,%d) out of image\n", x, y);
        return;
    }
    
    image->gray[y * image->ncol + x] = v;
}

/* Draw a connex line with gray level c between (a0,b0) and (a1,b1) */

void mw_draw_cimage(Cimage image, int a0, int b0, int a1, int b1,
                    unsigned char c)
{
    int bdx, bdy;
    int sx, sy, dx, dy, x, y, z;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_draw_cimage] NULL image struct or NULL gray plane\n");
        return;
    }
    
    bdx = image->ncol;
    bdy = image->nrow;
    
    if (a0 < 0)
        a0 = 0;
    else if (a0 >= bdx)
        a0 = bdx - 1;
    if (a1 < 0)
    {
        a1 = 0;
        /*
         * if (a0 == 0) mwerror(ERROR, 0,
         * "[mw_draw_cimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    else if (a1 >= bdx)
    {
        a1 = bdx - 1;
        /*
         * if (a0==bdx-1) mwerror(ERROR, 0,
         * "[mw_draw_cimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    if (b0 < 0)
        b0 = 0;
    else if (b0 >= bdy)
        b0 = bdy - 1;
    if (b1 < 0)
    {
        b1 = 0;
        /*
         * if (b0==0)
         * mwerror(ERROR, 0,
         * "[mw_draw_cimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    else if (b1 >= bdy)
    {
        b1 = bdy - 1;
        /*
         * if (b0==bdy-1)
         * mwerror(ERROR, 0,
         * "[mw_draw_cimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    
    if (a0 < a1)
    {
        sx = 1;
        dx = a1 - a0;
    }
    else
    {
        sx = -1;
        dx = a0 - a1;
    }
    if (b0 < b1)
    {
        sy = 1;
        dy = b1 - b0;
    }
    else
    {
        sy = -1;
        dy = b0 - b1;
    }
    x = 0;
    y = 0;
    
    if (dx >= dy)
    {
        z = (-dx) / 2;
        while (abs(x) <= dx)
        {
            image->gray[(y + b0) * bdx + x + a0] = c;
            x += sx;
            z += dy;
            if (z > 0)
            {
                y += sy;
                z -= dx;
            }
        }
    }
    else
    {
        z = (-dy) / 2;
        while (abs(y) <= dy)
        {
            image->gray[(y + b0) * bdx + x + a0] = c;
            y += sy;
            z += dx;
            if (z > 0)
            {
                x += sx;
                z -= dy;
            }
        }
    }
}

/* Clear the gray plane of a cimage with the value v */

void mw_clear_cimage(Cimage image, unsigned char v)
{
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_clear_cimage] NULL image struct or NULL gray plane\n");
        return;
    }
    memset(image->gray, v, image->ncol * image->nrow);
}

/* Copy the gray plane of a cimage into another cimage */

void mw_copy_cimage(Cimage in, Cimage out)
{
    if ((!in) || (!out) || (!in->gray) || (!out->gray)
        || (in->ncol != out->ncol) || (in->nrow != out->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_copy_cimage] NULL input or output image "
                "or images of different sizes\n");
        return;
    }
    
    strcpy(out->cmt, in->cmt);
    memcpy(out->gray, in->gray, in->ncol * in->nrow);
}

/* Return a tab T so that T[i][j] = image->gray[i*image->ncol+j] */

unsigned char **mw_newtab_gray_cimage(Cimage image)
{
    unsigned char **im;
    register unsigned long l;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_newtab_gray_cimage] NULL image struct "
                "or NULL gray plane\n");
        return (NULL);
    }
    
    im = (unsigned char **) malloc(image->nrow * sizeof(unsigned char *));
    if (im == NULL)
    {
        mwerror(ERROR, 0, "[mw_newtab_gray_cimage] Not enough memory\n");
        return (NULL);
    }
    
    im[0] = image->gray;
    /* FIXME: wrong types, dirty temporary fix */
    for (l = 1; l < (unsigned long) image->nrow; l++)
        im[l] = im[l - 1] + image->ncol;
    
    return (im);
}

/* Return 0 is this is not a binary image, a value > 0 if it is one.
 The value > 0 corresponds to the maxima value (the unique value != 0)
 */

unsigned char mw_isitbinary_cimage(Cimage image)
{
    unsigned char *p;
    int l, min, max;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_isitbinary_cimage] NULL image struct "
                "or NULL gray plane\n");
        return -1;
    }
    
    min = 1000;
    max = -1;
    for (l = 0, p = image->gray; l < image->ncol * image->nrow; l++, p++)
    {
        if (min > *p)
            min = *p;
        if (max < *p)
            max = *p;
    }
    if (min > 0)
        return (0);
    /* FIXME: wrong types, dirty temporary fix */
    for (l = 0, p = image->gray; l < image->ncol * image->nrow; l++, p++)
        if ((*p != min) && (*p != max))
            return (0);
    return (max);
}










Fimage mw_new_fimage()
{
    Fimage image;
    
    if (!(image = (Fimage) (malloc(sizeof(struct fimage)))))
    {
        mwerror(ERROR, 0, "[mw_new_fimage] Not enough memory\n");
        return (NULL);
    }
    
    image->nrow = image->ncol = 0;
    image->allocsize = 0;
    image->firstcol = image->lastcol = image->firstrow = image->lastrow = 0.0;
    
    image->scale = 0.0;
    strcpy(image->cmt, "?");
    strcpy(image->name, "?");
    
    image->gray = NULL;
    
    image->previous = NULL;
    image->next = NULL;
    
    return (image);
}

/* allocates the gray array */

Fimage mw_alloc_fimage(Fimage image, int nrow, int ncol)
{
    int size;
    
    if (image == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fimage] cannot alloc plane : "
                "fimage structure is NULL\n");
        return (NULL);
    }
    
    size = nrow * ncol * sizeof(float);
    if (size <= 0)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fimage] Attempts to alloc a fimage "
                "with null size\n");
        return NULL;
    }
    
    if (image->gray != NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_fimage] Attempts to alloc a fimage "
                "which is already allocated\n");
        return (NULL);
    }
    
    image->gray = (float *) malloc(size);
    if (image->gray == NULL)
    {
        image->nrow = image->ncol = 0;
        image->allocsize = 0;
        mwerror(ERROR, 0, "[mw_alloc_fimage] Not enough memory\n");
        return (NULL);
    }
    
    image->nrow = nrow;
    image->ncol = ncol;
    image->allocsize = size;
    return (image);
}

/* desallocate the array in the fimage structure and the structure itself */

void mw_delete_fimage(Fimage image)
{
    if (image == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_delete_fimage] cannot delete : "
                "fimage structure is NULL\n");
        return;
    }
    
    if (image->gray != NULL)
        free(image->gray);
    image->gray = NULL;
    free(image);
    image = NULL;
}

/* Change the size of the allocated gray plane */
/* May define the struct if not defined */
/* So you have to call it with image = mw_change_fimage(image,...) */

Fimage mw_change_fimage(Fimage image, int nrow, int ncol)
{
    int size;
    
    if (image == NULL)
        image = mw_new_fimage();
    if (image == NULL)
        return (NULL);
    
    size = nrow * ncol * sizeof(float);
    if (size > image->allocsize)
    {
        if (image->gray != NULL)
        {
            free(image->gray);
            image->gray = NULL;
        }
        if (mw_alloc_fimage(image, nrow, ncol) == NULL)
        {
            mw_delete_fimage(image);
            return (NULL);
        }
    }
    else
    {
        image->nrow = nrow;
        image->ncol = ncol;
    }
    return (image);
}

/* Return the gray level value of a fimage at location (x,y) */
/* WARNING: this is a slow way to access to a pixel !        */

float mw_getdot_fimage(Fimage image, int x, int y)
{
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_getdot_fimage] NULL image struct "
                "or NULL gray plane... Return 0\n");
        return (0);
        
    }
    
    if ((x < 0) || (y < 0) || (x >= image->ncol) || (y >= image->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_getdot_fimage] Point (%d,%d) out of image... "
                "Return 0.0\n", x, y);
        return (0.0);
    }
    
    return (image->gray[y * image->ncol + x]);
}

/* Set the gray level value of a fimage at location (x,y) */
/* WARNING: this is a slow way to access to a pixel !     */

void mw_plot_fimage(Fimage image, int x, int y, float v)
{
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_plot_fimage] NULL image struct or NULL gray plane\n");
        return;
    }
    
    if ((x < 0) || (y < 0) || (x >= image->ncol) || (y >= image->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_plot_fimage] Point (%d,%d) out of image\n", x, y);
        return;
    }
    
    image->gray[y * image->ncol + x] = v;
}

/* Draw a connex line with gray level c between (a0,b0) and (a1,b1) */

void mw_draw_fimage(Fimage image, int a0, int b0, int a1, int b1, float c)
{
    int bdx, bdy;
    int sx, sy, dx, dy, x, y, z;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_draw_fimage] NULL image struct or NULL gray plane\n");
        return;
    }
    
    bdx = image->ncol;
    bdy = image->nrow;
    
    if (a0 < 0)
        a0 = 0;
    else if (a0 >= bdx)
        a0 = bdx - 1;
    if (a1 < 0)
    {
        a1 = 0;
        /*
         * if (a0 == 0) mwerror(ERROR, 0,
         * "[mw_draw_fimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    else if (a1 >= bdx)
    {
        a1 = bdx - 1;
        /*
         * if (a0==bdx-1) mwerror(ERROR, 0,
         * "[mw_draw_fimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    if (b0 < 0)
        b0 = 0;
    else if (b0 >= bdy)
        b0 = bdy - 1;
    if (b1 < 0)
    {
        b1 = 0;
        /*
         * if (b0==0)
         * mwerror(ERROR, 0,
         * "[mw_draw_fimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    else if (b1 >= bdy)
    {
        b1 = bdy - 1;
        /*
         * if (b0==bdy-1)
         * mwerror(ERROR, 0,
         * "[mw_draw_fimage] Illegal parameters (%d,%d)-(%d,%d)\n",
         * a0,b0,a1,b1);
         */
    }
    
    if (a0 < a1)
    {
        sx = 1;
        dx = a1 - a0;
    }
    else
    {
        sx = -1;
        dx = a0 - a1;
    }
    if (b0 < b1)
    {
        sy = 1;
        dy = b1 - b0;
    }
    else
    {
        sy = -1;
        dy = b0 - b1;
    }
    x = 0;
    y = 0;
    
    if (dx >= dy)
    {
        z = (-dx) / 2;
        while (abs(x) <= dx)
        {
            image->gray[(y + b0) * bdx + x + a0] = c;
            x += sx;
            z += dy;
            if (z > 0)
            {
                y += sy;
                z -= dx;
            }
        }
    }
    else
    {
        z = (-dy) / 2;
        while (abs(y) <= dy)
        {
            image->gray[(y + b0) * bdx + x + a0] = c;
            y += sy;
            z += dx;
            if (z > 0)
            {
                x += sx;
                z -= dy;
            }
        }
    }
}

/* Clear the gray plane of a fimage with the value v */

void mw_clear_fimage(Fimage image, float v)
{
    register float *ptr;
    register int l;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_clear_fimage] NULL image struct or NULL gray plane\n");
        return;
    }
    
    for (l = 1, ptr = image->gray; l <= image->ncol * image->nrow; l++, ptr++)
        *ptr = v;
}

/* Copy the gray plane of a fimage into another fimage */

void mw_copy_fimage(Fimage in, Fimage out)
{
    if ((!in) || (!out) || (!in->gray) || (!out->gray)
        || (in->ncol != out->ncol) || (in->nrow != out->nrow))
    {
        mwerror(ERROR, 0,
                "[mw_copy_fimage] NULL input or output image "
                "or images of different sizes\n");
        return;
    }
    
    strcpy(out->cmt, in->cmt);
    memcpy(out->gray, in->gray, sizeof(float) * in->ncol * in->nrow);
}

/* Return a tab T so that T[i][j] = image->gray[i*image->ncol+j] */

float **mw_newtab_gray_fimage(Fimage image)
{
    float **im;
    register unsigned long l;
    
    if ((!image) || (!image->gray))
    {
        mwerror(ERROR, 0,
                "[mw_newtab_gray_fimage] NULL image struct "
                "or NULL gray plane\n");
        return (NULL);
    }
    
    im = (float **) malloc(image->nrow * sizeof(float *));
    if (im == NULL)
    {
        mwerror(ERROR, 0, "[mw_newtab_gray_fimage] Not enough memory\n");
        return (NULL);
    }
    
    im[0] = image->gray;
    /* FIXME: wrong types, dirty temporary fix */
    for (l = 1; l < (unsigned long) image->nrow; l++)
        im[l] = im[l - 1] + image->ncol;
    
    return (im);
}




/* Memory growth rate for reallocations */
#define MW_LIST_ENLARGE_FACTOR 1.5

/******************************** FLIST ********************************/

/* Create a new flist structure */

Flist mw_new_flist(void)
{
    Flist l;
    
    if (!(l = (Flist) (malloc(sizeof(struct flist)))))
    {
        mwerror(ERROR, 0, "[mw_new_flist] Not enough memory\n");
        return (NULL);
    }
    l->size = l->max_size = l->dim = l->data_size = 0;
    l->values = NULL;
    l->data = NULL;
    
    return (l);
}

/* (Re)allocate the values[] array in a flist structure */

Flist mw_realloc_flist(Flist l, int n)
{
    float *ptr;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_realloc_flist] flist structure is NULL\n");
        return (NULL);
    }
    
    if (n < l->size)
        mwerror(WARNING, 0, "[mw_realloc_flist] data may have been lost\n");
    ptr = (float *) realloc(l->values, n * l->dim * sizeof(float));
    if (!ptr && n != 0)
    {
        mwerror(ERROR, 0, "[mw_realloc_flist] Not enough memory\n");
        if (l->values)
            free(l->values);
        free(l);
        return (NULL);
    }
    l->values = ptr;
    l->max_size = n;
    return (l);
}

/* Enlarge the values[] array in a flist structure */

Flist mw_enlarge_flist(Flist l)
{
    int n;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_enlarge_flist] flist structure is NULL\n");
        return (NULL);
    }
    
    n = (int) (1 + (float) l->max_size * MW_LIST_ENLARGE_FACTOR);
    return (mw_realloc_flist(l, n));
}

/* Configure and (re)allocate a flist structure */

Flist mw_change_flist(Flist l, int max_size, int size, int dimension)
{
    if (!l)
    {
        l = mw_new_flist();
        if (!l)
            return (NULL);
    }
    
    l->dim = dimension;
    l->size = size;
    if (size < 0 || size > max_size)
    {
        mwerror(ERROR, 0, "[mw_change_flist] Inconsistant size\n");
        return (l);
    }
    
    /* (re)allocate values[] */
    if (max_size > l->max_size)
        return (mw_realloc_flist(l, max_size));
    
    return (l);
}

/* Delete a flist structure */

void mw_delete_flist(Flist l)
{
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_delete_flist] flist structure is NULL\n");
        return;
    }
    if (l->values)
        free(l->values);
    if (l->data_size > 0 && l->data)
        free(l->data);
    free(l);
}

/* Clear the values[] array in a flist structure */

void mw_clear_flist(Flist l, float v)
{
    int n;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_clear_flist] flist structure is NULL\n");
        return;
    }
    
    for (n = l->size * l->dim; n--;)
        l->values[n] = v;
}

/* Duplicate a flist structure */

Flist mw_copy_flist(Flist in, Flist out)
{
    int n;
    
    if (!in)
    {
        mwerror(ERROR, 0, "[mw_copy_flist] NULL input\n");
        return (NULL);
    }
    out = mw_change_flist(out, in->size, in->size, in->dim);
    if (out)
    {
        for (n = in->size * in->dim; n--;)
            out->values[n] = in->values[n];
        if (in->data_size && in->data)
        {
            if (out->data_size < in->data_size)
            {
                out->data = realloc(out->data, in->data_size);
                if (!out->data)
                {
                    mwerror(ERROR, 0, "[mw_copy_flist] Not enough memory\n");
                    return (out);
                }
            }
            memcpy(out->data, in->data, in->data_size);
            out->data_size = in->data_size;
        }
    }
    return (out);
}

/******************************** FLISTS ********************************/

/* Create a new flists structure */

Flists mw_new_flists(void)
{
    Flists ls;
    
    if (!(ls = (Flists) (malloc(sizeof(struct flists)))))
    {
        mwerror(ERROR, 0, "[mw_new_flists] Not enough memory\n");
        return (NULL);
    }
    ls->size = ls->max_size = ls->data_size = 0;
    ls->list = NULL;
    ls->data = NULL;
    
    return (ls);
}

/* (Re)allocate the list[] array in a flists structure */

Flists mw_realloc_flists(Flists ls, int n)
{
    Flist *ptr;
    int i;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_realloc_flists] flists structure is NULL\n");
        return (NULL);
    }
    
    if (n < ls->size)
        mwerror(WARNING, 0, "[mw_realloc_flists] data may have been lost\n");
    ptr = (Flist *) realloc(ls->list, n * sizeof(Flist));
    if (!ptr && n != 0)
    {
        mwerror(ERROR, 0, "[mw_realloc_flists] Not enough memory\n");
        free(ls);
        return (NULL);
    }
    
    /* clear new allocated Flists */
    ls->list = ptr;
    for (i = ls->max_size; i < n; i++)
        ls->list[i] = NULL;
    ls->max_size = n;
    return (ls);
}

/* Enlarge the list[] array in a flists structure */

Flists mw_enlarge_flists(Flists ls)
{
    int n;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_enlarge_flists] flists structure is NULL\n");
        return (NULL);
    }
    
    n = (int) (1 + (float) ls->max_size * MW_LIST_ENLARGE_FACTOR);
    return (mw_realloc_flists(ls, n));
}

/* Configure and (re)allocate a flists structure */

Flists mw_change_flists(Flists ls, int max_size, int size)
{
    if (!ls)
    {
        ls = mw_new_flists();
        if (!ls)
            return (NULL);
    }
    
    ls->size = size;
    if (size < 0 || size > max_size)
    {
        mwerror(ERROR, 0, "[mw_change_flists] Inconsistant size\n");
        return (ls);
    }
    
    /* (re)allocate list[] */
    if (max_size > ls->max_size)
        return (mw_realloc_flists(ls, max_size));
    
    return (ls);
}

/* Delete a flists structure (delete all associated Flist) */

void mw_delete_flists(Flists ls)
{
    int i;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_delete_flists] flists structure is NULL\n");
        return;
    }
    for (i = ls->size; i--;)
        mw_delete_flist(ls->list[i]);
    if (ls->list)
        free(ls->list);
    if (ls->data_size > 0 && ls->data)
        free(ls->data);
    free(ls);
}

/* Duplicate a flists structure */

Flists mw_copy_flists(Flists in, Flists out)
{
    int n;
    
    if (!in)
    {
        mwerror(ERROR, 0, "[mw_copy_flists] NULL input\n");
        return (NULL);
    }
    out = mw_change_flists(out, in->size, in->size);
    if (out)
    {
        for (n = in->size; n--;)
            out->list[n] = mw_copy_flist(in->list[n], out->list[n]);
        if (in->data_size && in->data)
        {
            if (out->data_size < in->data_size)
            {
                out->data = realloc(out->data, in->data_size);
                if (!out->data)
                {
                    mwerror(ERROR, 0, "[mw_copy_flists] Not enough memory\n");
                    return (out);
                }
            }
            memcpy(out->data, in->data, in->data_size);
            out->data_size = in->data_size;
        }
    }
    return (out);
}

/******************************** DLIST ********************************/

/* Create a new dlist structure */

Dlist mw_new_dlist(void)
{
    Dlist l;
    
    if (!(l = (Dlist) (malloc(sizeof(struct dlist)))))
    {
        mwerror(ERROR, 0, "[mw_new_dlist] Not enough memory\n");
        return (NULL);
    }
    l->size = l->max_size = l->dim = l->data_size = 0;
    l->values = NULL;
    l->data = NULL;
    
    return (l);
}

/* (Re)allocate the values[] array in a dlist structure */

Dlist mw_realloc_dlist(Dlist l, int n)
{
    double *ptr;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_realloc_dlist] dlist structure is NULL\n");
        return (NULL);
    }
    
    if (n < l->size)
        mwerror(WARNING, 0, "[mw_realloc_dlist] data may have been lost\n");
    
    ptr = (double *) realloc(l->values, n * l->dim * sizeof(double));
    if (!ptr && n != 0)
    {
        mwerror(ERROR, 0, "[mw_realloc_dlist] Not enough memory\n");
        if (l->values)
            free(l->values);
        free(l);
        return (NULL);
    }
    l->values = ptr;
    l->max_size = n;
    return (l);
}

/* Enlarge the values[] array in a dlist structure */

Dlist mw_enlarge_dlist(Dlist l)
{
    int n;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_enlarge_dlist] dlist structure is NULL\n");
        return (NULL);
    }
    
    n = (int) (1 + (double) l->max_size * MW_LIST_ENLARGE_FACTOR);
    return (mw_realloc_dlist(l, n));
}

/* Configure and (re)allocate a dlist structure */

Dlist mw_change_dlist(Dlist l, int max_size, int size, int dimension)
{
    if (!l)
    {
        l = mw_new_dlist();
        if (!l)
            return (NULL);
    }
    
    l->dim = dimension;
    l->size = size;
    if (size < 0 || size > max_size)
    {
        mwerror(ERROR, 0, "[mw_change_dlist] Inconsistant size\n");
        return (l);
    }
    
    /* (re)allocate values[] */
    if (max_size > l->max_size)
        return (mw_realloc_dlist(l, max_size));
    
    return (l);
}

/* Delete a dlist structure */

void mw_delete_dlist(Dlist l)
{
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_delete_dlist] dlist structure is NULL\n");
        return;
    }
    if (l->values)
        free(l->values);
    if (l->data_size > 0 && l->data)
        free(l->data);
    free(l);
}

/* Clear the values[] array in a dlist structure */

void mw_clear_dlist(Dlist l, double v)
{
    int n;
    
    if (!l)
    {
        mwerror(ERROR, 0, "[mw_clear_dlist] dlist structure is NULL\n");
        return;
    }
    
    for (n = l->size * l->dim; n--;)
        l->values[n] = v;
}

/* Duplicate a dlist structure */

Dlist mw_copy_dlist(Dlist in, Dlist out)
{
    int n;
    
    if (!in)
    {
        mwerror(ERROR, 0, "[mw_copy_dlist] NULL input\n");
        return (NULL);
    }
    out = mw_change_dlist(out, in->size, in->size, in->dim);
    if (out)
    {
        for (n = in->size * in->dim; n--;)
            out->values[n] = in->values[n];
        if (in->data_size && in->data)
        {
            if (out->data_size < in->data_size)
            {
                out->data = realloc(out->data, in->data_size);
                if (!out->data)
                {
                    mwerror(ERROR, 0, "[mw_copy_dlist] Not enough memory\n");
                    return (out);
                }
            }
            memcpy(out->data, in->data, in->data_size);
            out->data_size = in->data_size;
        }
    }
    return (out);
}

/******************************** DLISTS ********************************/

/* Create a new dlists structure */

Dlists mw_new_dlists(void)
{
    Dlists ls;
    
    if (!(ls = (Dlists) (malloc(sizeof(struct dlists)))))
    {
        mwerror(ERROR, 0, "[mw_new_dlists] Not enough memory\n");
        return (NULL);
    }
    ls->size = ls->max_size = ls->data_size = 0;
    ls->list = NULL;
    ls->data = NULL;
    
    return (ls);
}

/* (Re)allocate the list[] array in a dlists structure */

Dlists mw_realloc_dlists(Dlists ls, int n)
{
    Dlist *ptr;
    int i;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_realloc_dlists] dlists structure is NULL\n");
        return (NULL);
    }
    
    if (n < ls->size)
        mwerror(WARNING, 0, "[mw_realloc_dlists] data may have been lost\n");
    
    ptr = (Dlist *) realloc(ls->list, n * sizeof(Dlist));
    if (!ptr && n != 0)
    {
        mwerror(ERROR, 0, "[mw_realloc_dlists] Not enough memory\n");
        free(ls);
        return (NULL);
    }
    
    /* clear new allocated Dlists */
    ls->list = ptr;
    for (i = ls->max_size; i < n; i++)
        ls->list[i] = NULL;
    ls->max_size = n;
    return (ls);
}

/* Enlarge the list[] array in a dlists structure */

Dlists mw_enlarge_dlists(Dlists ls)
{
    int n;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_enlarge_dlists] dlists structure is NULL\n");
        return NULL;
    }
    
    n = (int) (1 + (double) ls->max_size * MW_LIST_ENLARGE_FACTOR);
    return (mw_realloc_dlists(ls, n));
}

/* Configure and (re)allocate a dlists structure */

Dlists mw_change_dlists(Dlists ls, int max_size, int size)
{
    if (!ls)
    {
        ls = mw_new_dlists();
        if (!ls)
            return (NULL);
    }
    
    ls->size = size;
    if (size < 0 || size > max_size)
    {
        mwerror(ERROR, 0, "[mw_change_dlists] Inconsistant size\n");
        return (ls);
    }
    
    /* (re)allocate list[] */
    if (max_size > ls->max_size)
        return (mw_realloc_dlists(ls, max_size));
    
    return (ls);
}

/* Delete a dlists structure (delete all associated Dlist) */

void mw_delete_dlists(Dlists ls)
{
    int i;
    
    if (!ls)
    {
        mwerror(ERROR, 0, "[mw_delete_dlists] dlists structure is NULL\n");
        return;
    }
    for (i = ls->size; i--;)
        mw_delete_dlist(ls->list[i]);
    if (ls->list)
        free(ls->list);
    if (ls->data_size > 0 && ls->data)
        free(ls->data);
    free(ls);
}

/* Duplicate a dlists structure */

Dlists mw_copy_dlists(Dlists in, Dlists out)
{
    int n;
    
    if (!in)
    {
        mwerror(ERROR, 0, "[mw_copy_dlists] NULL input\n");
        return (NULL);
    }
    out = mw_change_dlists(out, in->size, in->size);
    if (out)
    {
        for (n = in->size; n--;)
            out->list[n] = mw_copy_dlist(in->list[n], out->list[n]);
        if (in->data_size && in->data)
        {
            if (out->data_size < in->data_size)
            {
                out->data = realloc(out->data, in->data_size);
                if (!out->data)
                {
                    mwerror(ERROR, 0, "[mw_copy_dlists] Not enough memory\n");
                    return (out);
                }
            }
            memcpy(out->data, in->data, in->data_size);
            out->data_size = in->data_size;
        }
    }
    return (out);
}






/*--- Point_plane ---*/

/* creates a new point_plane structure */

Point_plane mw_new_point_plane(void)
{
    Point_plane point;
    
    if (!(point = (Point_plane) (malloc(sizeof(struct point_plane)))))
    {
        mwerror(ERROR, 0, "[mw_new_point_plane] Not enough memory\n");
        return (NULL);
    }
    point->x = point->y = -1;
    return (point);
}

/* Define the struct if it's not defined */

Point_plane mw_change_point_plane(Point_plane point)
{
    if (point == NULL)
        point = mw_new_point_plane();
    return (point);
}

/*--- Shape ---*/

/* Creates a new shape structure */

Shape mw_new_shape(void)
{
    Shape shape;
    
    if (!(shape = (Shape) (malloc(sizeof(struct shape)))))
    {
        mwerror(ERROR, 0, "[mw_new_shape] Not enough memory\n");
        return (NULL);
    }
    
    shape->inferior_type = 0;
    shape->value = 0.0;
    shape->open = 0;
    shape->area = 0;
    shape->removed = 0;
    shape->pixels = NULL;
    shape->boundary = NULL;
    shape->parent = shape->next_sibling = shape->child = NULL;
    shape->data = NULL;
    shape->data_size = 0;
    return (shape);
}

/* Define the struct if it's not defined */

Shape mw_change_shape(Shape sh)
{
    if (sh == NULL)
        sh = mw_new_shape();
    return (sh);
}

/* deallocate the shape structure */

void mw_delete_shape(Shape shape)
{
    if (shape == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_delete_shape] cannot delete : shape structure is NULL\n");
        return;
    }
    if (shape->pixels)
        free(shape->pixels);
    shape->pixels = NULL;
    shape->area = 0;
    if (shape->boundary)
        mw_delete_flist(shape->boundary);
    shape->boundary = NULL;
    if (shape->data)
        free(shape->data);
    free(shape);
    shape = NULL;
}

/* Get in the subtree of root sh a shape that is not removed, NULL if
 * all shapes are removed */

Shape mw_get_not_removed_shape(Shape sh)
{
    Shape NotSup = NULL;
    if ((sh == NULL) || (!sh->removed))
        return (sh);
    for (sh = sh->child; sh != NULL; sh = sh->next_sibling)
        if ((NotSup = mw_get_not_removed_shape(sh)) != NULL)
            break;
    return (NotSup);
}

/* Get the true parent, that is the greatest non removed ancestor */

Shape mw_get_parent_shape(Shape sh)
{
    if (sh == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_get_parent_shape] input shape structure is NULL\n");
        return (NULL);
    }
    if (sh->parent == NULL)     /* It is the root of the shape */
        return (NULL);
    
    do
        if ((sh = sh->parent) == NULL)
        {
            mwerror(ERROR, 0,
                    "[mw_get_parent_shape] the root of the shapes "
                    "is removed\n");
            return (NULL);
        }
    while (sh->removed);
    return (sh);
}

/* Get the first child, taking into account that some shapes are removed */

Shape mw_get_first_child_shape(Shape sh)
{
    Shape NotSup = NULL;
    if (sh == NULL)
    {
        mwerror(ERROR, 0, "[mw_get_first_child_shape] input shape is NULL\n");
        return (NULL);
    }
    for (sh = sh->child; sh != NULL; sh = sh->next_sibling)
        if ((NotSup = mw_get_not_removed_shape(sh)) != NULL)
            break;
    return (NotSup);
}

/* Get the next sibling, taking into account that some shapes are removed */

Shape mw_get_next_sibling_shape(Shape sh)
{
    Shape sh1 = NULL, sh2 = NULL;
    if (sh == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_get_next_sibling_shape] input shape is NULL\n");
        return (NULL);
    }
    /* First look at the siblings in the original tree */
    for (sh1 = sh->next_sibling; sh1 != NULL; sh1 = sh1->next_sibling)
        if ((sh2 = mw_get_not_removed_shape(sh1)) != NULL)
            return (sh2);
    if (sh->parent == NULL || !sh->parent->removed)
        return (NULL);
    /* The parent in the original tree is also the parent in the true
     * tree, nothing more to do */
    /* If not found, find the node in the original tree just before
     * the true parent */
    do
    {
        sh = sh->parent;
        /* Look at the siblings of this node */
        for (sh1 = sh->next_sibling; sh1 != NULL; sh1 = sh1->next_sibling)
            if ((sh2 = mw_get_not_removed_shape(sh1)) != NULL)
                return (sh2);
    }
    while (sh->parent->removed);
    return (NULL);
}

/* Return the smallest shape containing the given pixel */

Shape mw_get_smallest_shape(Shapes shs, int iX, int iY)
{
    Shape sh;
    
    if (shs == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_get_smallest_shape] input shapes structure is NULL\n");
        return (NULL);
    }
    sh = shs->smallest_shape[iY * shs->ncol + iX];
    if (sh == NULL)
    {
        mwerror(ERROR, 0, "[mw_get_smallest_shape] smallest shape is NULL\n");
        return (NULL);
    }
    if (sh->removed)
        sh = mw_get_parent_shape(sh);
    return (sh);
}

/*-- Shapes --*/

/* creates a new shapes structure */

Shapes mw_new_shapes(void)
{
    Shapes shapes;
    
    if (!(shapes = (Shapes) (malloc(sizeof(struct shapes)))))
    {
        mwerror(ERROR, 0, "[mw_new_shapes] Not enough memory\n");
        return (NULL);
    }
    shapes->the_shapes = NULL;
    shapes->smallest_shape = NULL;
    shapes->data = NULL;
    shapes->data_size = 0;
    shapes->nb_shapes = 0;
    shapes->nrow = shapes->ncol = 0;
    shapes->interpolation = 0;
    strcpy(shapes->cmt, "?");
    strcpy(shapes->name, "?");
    return (shapes);
}

/* Allocate the root of the shapes */

Shapes mw_alloc_shapes(Shapes shs, int nrow, int ncol, float value)
{
    int size, i;
    Shape root;
    
    if (shs == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_shapes] cannot alloc root : "
                "shapes structure is NULL\n");
        return (NULL);
    }
    
    size = nrow * ncol;
    if (size <= 0)
    {
        mwerror(ERROR, 0,
                "[mw_alloc_shapes] Attempts to alloc shapes with null size\n");
        return (NULL);
    }
    if ((shs->the_shapes != NULL) || (shs->smallest_shape != NULL))
    {
        mwerror(ERROR, 0,
                "[mw_alloc_shapes] Attempts to alloc root "
                "which is already allocated\n");
        return (NULL);
    }
    
    root = shs->the_shapes =
    (Shape) malloc((size + 1) * sizeof(struct shape));
    if (root == NULL)
    {
        mwerror(ERROR, 0, "[mw_alloc_shapes] Not enough memory\n");
        return (NULL);
    }
    root->inferior_type = 1;
    root->value = value;
    root->open = 1;
    root->area = size;
    root->removed = 0;
    root->pixels = NULL;
    root->boundary = NULL;
    root->parent = root->next_sibling = root->child = NULL;
    root->data = NULL;
    root->data_size = 0;
    
    shs->nb_shapes = 1;
    shs->nrow = nrow;
    shs->ncol = ncol;
    
    shs->smallest_shape = (Shape *) malloc(size * sizeof(Shape));
    if (shs->smallest_shape == NULL)
    {
        mwerror(ERROR, 0, "[mw_alloc_shapes] Not enough memory\n");
        free(root);
        return (NULL);
    }
    for (i = size - 1; i >= 0; i--)
        shs->smallest_shape[i] = root;
    
    return (shs);
}

/* Allocate the struct and the root if they are not defined */

Shapes mw_change_shapes(Shapes shs, int nrow, int ncol, float value)
{
    int i;
    
    if (shs == NULL)
        shs = mw_new_shapes();
    if (shs == NULL)
        return (NULL);
    
    /* Deallocate the shapes but the structure itself */
    for (i = shs->nb_shapes - 1; i > 0; i--)
        if (shs->the_shapes[i].boundary != NULL)
            mw_delete_flist(shs->the_shapes[i].boundary);
    
    if ((shs->the_shapes != NULL) && (shs->nb_shapes > 0))
        free(shs->the_shapes[0].pixels);
    if (shs->the_shapes != NULL)
        free(shs->the_shapes);
    if (shs->smallest_shape != NULL)
        free(shs->smallest_shape);
    
    shs = mw_alloc_shapes(shs, nrow, ncol, value);
    return (shs);
}

/* deallocate the shapes structure */

void mw_delete_shapes(Shapes shs)
{
    int i;
    
    if (shs == NULL)
    {
        mwerror(ERROR, 0,
                "[mw_delete_shapes] cannot delete : "
                "shapes structure is NULL\n");
        return;
    }
    
    for (i = shs->nb_shapes - 1; i > 0; i--)
        if (shs->the_shapes[i].boundary != NULL)
            mw_delete_flist(shs->the_shapes[i].boundary);
    
    if ((shs->the_shapes != NULL) && (shs->nb_shapes > 0))
        free(shs->the_shapes[0].pixels);
    if (shs->the_shapes != NULL)
        free(shs->the_shapes);
    if (shs->smallest_shape != NULL)
        free(shs->smallest_shape);
    free(shs);
}






/* The value set at a pixel that is not a saddle point. The most appropriate
 value would be NAN ('Not A Number'); unfortunately, this constant is not
 systematically defined in 'math.h'. */
#define NOT_A_SADDLE FLT_MAX

/* Compute the saddle values and return the number of saddle points */
int fsaddles(Fimage pImage, Fimage pSaddlesImage)
{
    int i, j, iNumber;
    double minDiag1, maxDiag1, minDiag2, maxDiag2, temp;
    float **tabtabPixels, **tabtabSaddleValues;
    
    if (mw_change_fimage(pSaddlesImage, pImage->nrow - 1, pImage->ncol - 1) ==
        NULL)
        mwerror(FATAL, 1, "Image allocation error");
    tabtabPixels = mw_newtab_gray_fimage(pImage);
    tabtabSaddleValues = mw_newtab_gray_fimage(pSaddlesImage);
    if (tabtabPixels == NULL || tabtabSaddleValues == NULL)
        mwerror(FATAL, 1, "Array of lines: allocation error");
    
    iNumber = 0;
    for (i = pImage->nrow - 2; i >= 0; i--)
        for (j = pImage->ncol - 2; j >= 0; j--)
        {
            if ((minDiag1 = tabtabPixels[i][j]) > (maxDiag1 =
                                                   tabtabPixels[i + 1][j +
                                                                       1]))
            {
                temp = minDiag1;
                minDiag1 = maxDiag1;
                maxDiag1 = temp;
            }
            if ((minDiag2 = tabtabPixels[i + 1][j]) > (maxDiag2 =
                                                       tabtabPixels[i][j +
                                                                       1]))
            {
                temp = minDiag2;
                minDiag2 = maxDiag2;
                maxDiag2 = temp;
            }
            if (minDiag1 > maxDiag2 || maxDiag1 < minDiag2)
            {
                temp = minDiag1 + maxDiag1 - (minDiag2 + maxDiag2);
                tabtabSaddleValues[i][j] =
                (minDiag1 * maxDiag1 - minDiag2 * maxDiag2) / temp;
                ++iNumber;
            }
            else
                tabtabSaddleValues[i][j] = NOT_A_SADDLE;
        }
    free(tabtabPixels);
    free(tabtabSaddleValues);
    
    return iNumber;
}




#ifndef POINT_T
#define POINT_T
typedef struct {
    float x, y;
} point_t;
#endif

#define NOT_A_SADDLE FLT_MAX    /* Value must be coherent with fsaddles.c */
#define is_a_saddle(x) ((x) != NOT_A_SADDLE)

#define QEAST  0
#define QNORTH 1
#define QWEST  2
#define QSOUTH 3
typedef struct {
    short int x, y;             /* Top left corner coordinates */
    unsigned char cDirection;   /* Direction along which we enter
                                 * the dual pixel */
} DualPixel;

/* Is the center of pixel (x+.5,y+.5) in the shape? */
static char point_in_shape(short int x, short int y, Shape pShape,
                           Shapes pTree)
{
    Shape pShapePoint = pTree->smallest_shape[y * pTree->ncol + x];
    return (pShapePoint->area <= pShape->area &&
            pShape->pixels <= pShapePoint->pixels &&
            pShapePoint->pixels < pShape->pixels + pShape->area);
}

/* If there is a saddle point in the dual pixel of top left corner (x,y) and
 this saddle point belongs to pShape, return 1 */
static char saddle_inside(short int x, short int y, Shape pShape,
                          float **tabtabSaddleValues)
{
    float v = tabtabSaddleValues[y - 1][x - 1];
    
    if (!is_a_saddle(v))
        return 0;
    if (pShape->inferior_type)
        return (char) (pShape->value >= v);
    return (char) (pShape->value <= v);
}

#define TURN_LEFT(dir) \
dir = (dir==QNORTH ? QWEST :\
(dir==QWEST ? QSOUTH :\
(dir==QSOUTH ? QEAST : QNORTH)))
#define TURN_RIGHT(dir) \
dir = (dir==QNORTH ? QEAST :\
(dir==QEAST ? QSOUTH :\
(dir==QSOUTH ? QWEST : QNORTH)))

/* Find the dual pixel following pDualPixel as we follow the shape boundary */
static void find_next_dual_pixel(DualPixel * pDualPixel, Shape pShape,
                                 Shapes pTree, float **tabtabSaddleValues)
{
    char bLeftIn, bRightIn;
    
    switch (pDualPixel->cDirection)
    {
        case QNORTH:
            bLeftIn =
            point_in_shape(pDualPixel->x - 1, pDualPixel->y - 1, pShape,
                           pTree);
            bRightIn =
            point_in_shape(pDualPixel->x, pDualPixel->y - 1, pShape, pTree);
            if (bLeftIn
                || saddle_inside(pDualPixel->x, pDualPixel->y, pShape,
                                 tabtabSaddleValues))
                if (bRightIn)
                {
                    ++pDualPixel->x;
                    TURN_RIGHT(pDualPixel->cDirection);
                }
                else
                    --pDualPixel->y;
                else
                {
                    --pDualPixel->x;
                    TURN_LEFT(pDualPixel->cDirection);
                }
            break;
        case QWEST:
            bLeftIn =
            point_in_shape(pDualPixel->x - 1, pDualPixel->y, pShape, pTree);
            bRightIn =
            point_in_shape(pDualPixel->x - 1, pDualPixel->y - 1, pShape,
                           pTree);
            if (bLeftIn
                || saddle_inside(pDualPixel->x, pDualPixel->y, pShape,
                                 tabtabSaddleValues))
                if (bRightIn)
                {
                    --pDualPixel->y;
                    TURN_RIGHT(pDualPixel->cDirection);
                }
                else
                    --pDualPixel->x;
                else
                {
                    ++pDualPixel->y;
                    TURN_LEFT(pDualPixel->cDirection);
                }
            break;
        case QSOUTH:
            bLeftIn = point_in_shape(pDualPixel->x, pDualPixel->y, pShape, pTree);
            bRightIn =
            point_in_shape(pDualPixel->x - 1, pDualPixel->y, pShape, pTree);
            if (bLeftIn
                || saddle_inside(pDualPixel->x, pDualPixel->y, pShape,
                                 tabtabSaddleValues))
                if (bRightIn)
                {
                    --pDualPixel->x;
                    TURN_RIGHT(pDualPixel->cDirection);
                }
                else
                    ++pDualPixel->y;
                else
                {
                    ++pDualPixel->x;
                    TURN_LEFT(pDualPixel->cDirection);
                }
            break;
        case QEAST:
            bLeftIn =
            point_in_shape(pDualPixel->x, pDualPixel->y - 1, pShape, pTree);
            bRightIn =
            point_in_shape(pDualPixel->x, pDualPixel->y, pShape, pTree);
            if (bLeftIn
                || saddle_inside(pDualPixel->x, pDualPixel->y, pShape,
                                 tabtabSaddleValues))
                if (bRightIn)
                {
                    ++pDualPixel->y;
                    TURN_RIGHT(pDualPixel->cDirection);
                }
                else
                    ++pDualPixel->x;
                else
                {
                    --pDualPixel->y;
                    TURN_LEFT(pDualPixel->cDirection);
                }
            break;
    }
}

/* Find the boundary of the shape, which is closed */
static int find_closed_boundary(Shapes pTree, Shape pShape,
                                float **tabtabSaddleValues, Flist pBoundary)
{
    short int x0, y0;
    DualPixel dualPixel;
    point_t *pPoint = (point_t *) pBoundary->values;
    
    /* 1) Find initial dual pixel, with NORTH direction */
    x0 = pShape->pixels[0].x;
    y0 = pShape->pixels[0].y;
    do
        ++x0;
    while (point_in_shape(x0, y0, pShape, pTree));
    
    /* 2) Follow the boundary */
    dualPixel.cDirection = QNORTH;
    dualPixel.x = x0;
    dualPixel.y = y0;
    do
    {
        pPoint[pBoundary->size].x = (float) dualPixel.x;
        pPoint[pBoundary->size++].y = (float) dualPixel.y;
        find_next_dual_pixel(&dualPixel, pShape, pTree, tabtabSaddleValues);
    }
    while (dualPixel.x != x0 || dualPixel.y != y0 ||
           dualPixel.cDirection != QNORTH);
    return 0;
}

/* Find an initial point (to follow the boundary) at the border of the image */
static void initial_point_border(DualPixel * pDualPixel, Shape pShape,
                                 Shapes pTree)
{
    short int iWidth = (short int) pTree->ncol, iHeight =
    (short int) pTree->nrow;
    short int x, y;
    
    /* Right border */
    pDualPixel->cDirection = QWEST;
    x = iWidth - 1;
    y = 0;
    if (point_in_shape(x, y++, pShape, pTree))
        while (y < iHeight && point_in_shape(x, y++, pShape, pTree));
    while (y < iHeight && !point_in_shape(x, y, pShape, pTree))
        ++y;
    if (y < iHeight)
    {
        pDualPixel->x = iWidth;
        pDualPixel->y = y;
        return;
    }
    /* Top border */
    pDualPixel->cDirection = QSOUTH;
    x = 0;
    y = 0;
    if (point_in_shape(x++, y, pShape, pTree))
        while (x < iWidth && point_in_shape(x++, y, pShape, pTree));
    while (x < iWidth && !point_in_shape(x, y, pShape, pTree))
        ++x;
    if (x < iWidth)
    {
        pDualPixel->x = x;
        pDualPixel->y = 0;
        return;
    }
    /* Left border */
    pDualPixel->cDirection = QEAST;
    x = 0;
    y = iHeight - 1;
    if (point_in_shape(x, y--, pShape, pTree))
        while (y >= 0 && point_in_shape(x, y--, pShape, pTree));
    while (y >= 0 && !point_in_shape(x, y, pShape, pTree))
        --y;
    if (y >= 0)
    {
        pDualPixel->x = 0;
        pDualPixel->y = y + 1;
        return;
    }
    /* Bottom border */
    pDualPixel->cDirection = QNORTH;
    x = iWidth - 1;
    y = iHeight - 1;
    if (point_in_shape(x--, y, pShape, pTree))
        while (x >= 0 && point_in_shape(x--, y, pShape, pTree));
    while (x >= 0 && !point_in_shape(x, y, pShape, pTree))
        --x;
    if (x >= 0)
    {
        pDualPixel->x = x + 1;
        pDualPixel->y = iHeight;
        return;
    }
    mwerror(FATAL, 1, "initial_point_border --> Coherency?");
}

/* Find an open boundary */
static int find_open_boundary(Shapes pTree, Shape pShape,
                              float **tabtabSaddleValues, Flist pBoundary)
{
    DualPixel dualPixel;
    short int iWidth = (short int) pTree->ncol, iHeight =
    (short int) pTree->nrow;
    point_t *pPoint = (point_t *) pBoundary->values;
    
    initial_point_border(&dualPixel, pShape, pTree);
    do
    {
        pPoint[pBoundary->size].x = (float) dualPixel.x;
        pPoint[pBoundary->size++].y = (float) dualPixel.y;
        find_next_dual_pixel(&dualPixel, pShape, pTree, tabtabSaddleValues);
    }
    while (0 < dualPixel.x && dualPixel.x < iWidth &&
           0 < dualPixel.y && dualPixel.y < iHeight);
    pPoint[pBoundary->size].x = (float) dualPixel.x;    /* We store the exit */
    pPoint[pBoundary->size++].y = (float) dualPixel.y;
    return 0;
}

/* Find boundary of the shape */
void flstb_dualchain(Shapes pTree, Shape pShape, Flist pBoundary,
                     char *ctabtabSaddleValues)
{
    struct fimage imageSaddles, *pImage;
    char bBuildSaddleValues = 0;
    float **tabtabSaddleValues = (float **) ctabtabSaddleValues;
    
    if (pTree->interpolation != 1)
        mwerror(USAGE, 1, "Please apply to a *bilinear* tree");
    if (tabtabSaddleValues == NULL)
    {
        bBuildSaddleValues = (char) 1;
        imageSaddles.nrow = imageSaddles.ncol = 0;
        imageSaddles.gray = NULL;
        imageSaddles.allocsize = 0;
        if ((pImage = mw_new_fimage()) == NULL)
            mwerror(FATAL, 1, "Void image allocation error!");
        flst_reconstruct(pTree, pImage);
        fsaddles(pImage, &imageSaddles);
        mw_delete_fimage(pImage);
        if ((tabtabSaddleValues =
             mw_newtab_gray_fimage(&imageSaddles)) == NULL)
            mwerror(FATAL, 1, "Lines of saddle values: allocation error");
    }
    if (pTree->the_shapes[0].pixels == NULL)
        flst_pixels(pTree);
    
    if (mw_change_flist(pBoundary, pShape->area * 4, 0, 2) != pBoundary)
        mwerror(FATAL, 1, "List allocation error");
    
    if (pShape->open)
        find_open_boundary(pTree, pShape, tabtabSaddleValues, pBoundary);
    else
        find_closed_boundary(pTree, pShape, tabtabSaddleValues, pBoundary);
    mw_realloc_flist(pBoundary, pBoundary->size);
    
    if (bBuildSaddleValues)
    {
        free(tabtabSaddleValues[0]);
        free(tabtabSaddleValues);
    }
}


void flst_reconstruct(Shapes pTree, Fimage pFloatImageOutput)
{
    int i;
    float *pOutputPixel;
    Shape *ppShapeOfPixel;
    Shape pShape;
    
    if (!mw_change_fimage(pFloatImageOutput, pTree->nrow, pTree->ncol))
        mwerror(FATAL, 1,
                "flst_reconstruct --> impossible "
                "to allocate the output image\n");
    
    pOutputPixel = pFloatImageOutput->gray;
    ppShapeOfPixel = pTree->smallest_shape;
    for (i = pTree->nrow * pTree->ncol - 1; i >= 0; i--)
    {
        pShape = *ppShapeOfPixel++;
        while (pShape->removed)
            pShape = pShape->parent;
        *pOutputPixel++ = pShape->value;
    }
}




/* For each shape, find its number of proper pixels */
static void compute_proper_pixels(Shapes pTree, int *tabNbOfProperPixels)
{
    Shape pShape;
    int *pNbOfProperPixels;
    int i;
    
    /* 1) Initialize by the area */
    pShape = pTree->the_shapes + pTree->nb_shapes - 1;
    pNbOfProperPixels = tabNbOfProperPixels + pTree->nb_shapes - 1;
    for (i = pTree->nb_shapes - 1; i >= 0; i--)
        *pNbOfProperPixels-- = (pShape--)->area;
    /* 2) For each shape, substract its area to its parent */
    pShape = pTree->the_shapes + pTree->nb_shapes - 1;
    for (i = pTree->nb_shapes - 1; i > 0; i--, pShape--)
        tabNbOfProperPixels[pShape->parent - pTree->the_shapes] -=
        pShape->area;
}

/* Allocate the array of pixels of each shape. Thanks to the tree structure,
 we allocate only memory for the pixels of the root, and other arrays are
 just pointers */
static void allocate_pixels(Shapes pTree, int *tabNbOfProperPixels)
{
    Point_plane tabPixelsOfRoot;
    Shape pShape, *tabpShapesOfStack;
    int i, iIndex, iSizeOfStack;
    
    /* 1) Memory allocation */
    tabPixelsOfRoot = pTree->the_shapes[0].pixels = (Point_plane)
    malloc(pTree->nrow * pTree->ncol * sizeof(struct point_plane));
    if (tabPixelsOfRoot == NULL)
        mwerror(FATAL, 1, "allocate_pixels --> Allocation of pixels\n");
    tabpShapesOfStack = (Shape *) malloc(pTree->nb_shapes * sizeof(Shape));
    if (tabpShapesOfStack == NULL)
        mwerror(FATAL, 1, "allocate_pixels --> Allocation of stack\n");
    
    /* 2) Enumeration of the tree in preorder, using the stack */
    pShape = &pTree->the_shapes[0];
    iSizeOfStack = 0;
    i = 0;
    while (1)
        if (pShape != NULL)
        {
            /* Write pixels of pShape */
            pShape->pixels = &tabPixelsOfRoot[i];
            iIndex = pShape - pTree->the_shapes;
            i += tabNbOfProperPixels[iIndex];
            tabpShapesOfStack[iSizeOfStack++] = pShape; /* Push */
            pShape = pShape->child;
        }
        else
        {
            if (iSizeOfStack == 0)
                break;
            pShape = tabpShapesOfStack[--iSizeOfStack]->next_sibling;
            /* Pop */
        }
    free(tabpShapesOfStack);
}

/* Associate to each shape its array of pixels. Fills the field PIXELS of
 the tree structure. From the command line, this function has no interest,
 since that field is not saved to the file. It is meant to be called from
 another module, when this field is needed */
void flst_pixels(Shapes pTree)
{
    Shape *ppShape;
    int *tabNbOfProperPixels;   /* For each shape,
                                 * its number of proper pixels */
    Point_plane pCurrentPoint;
    int i, j, iIndex;
    
    /* 1) Compute nb of proper pixels in each shape */
    if ((tabNbOfProperPixels =
         (int *) malloc(pTree->nb_shapes * sizeof(int))) == NULL)
        mwerror(FATAL, 1, "Allocation of array error");
    compute_proper_pixels(pTree, tabNbOfProperPixels);
    
    /* 2) Allocate array for the root and make others sub-arrays */
    allocate_pixels(pTree, tabNbOfProperPixels);
    
    /* 3) Fill the array */
    ppShape = pTree->smallest_shape + pTree->ncol * pTree->nrow - 1;
    for (i = pTree->nrow - 1; i >= 0; i--)
        for (j = pTree->ncol - 1; j >= 0; j--, ppShape--)
        {
            iIndex = (*ppShape) - pTree->the_shapes;
            pCurrentPoint =
            &(*ppShape)->pixels[--tabNbOfProperPixels[iIndex]];
            pCurrentPoint->x = j;
            pCurrentPoint->y = i;
        }
    free(tabNbOfProperPixels);
}



/* Optimization parameters. Probably should depend on image size, but these
 values seem good for most images. */
#define INIT_MAX_AREA 10
#define STEP_MAX_AREA 8

/* A 'local configuration' of the pixel is the logical 'or' of these values,
 stored in one byte. Corresponding bit is set if the neighbor is in region */
#define EAST         1
#define NORTH_EAST   2
#define NORTH        4
#define NORTH_WEST   8
#define WEST        16
#define SOUTH_WEST  32
#define SOUTH       64
#define SOUTH_EAST 128

/* Gives for each configuration of the neighborhood around the pixel the number
 of new cc's of the complement created (sometimes negative, since
 cc's can be deleted), when the pixel is appended to the region.
 Configurations are stored on one byte.
 tabPatterns[0]: set in 4-connectedness and complement in 8-connectedness.
 tabPatterns[1]: set in 8-connectedness and complement in 4-connectedness. */
static char tabPatterns[2][256] =
{ {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 1, 2, 1, 1,
    1, 2, 1, 2, 2, 3, 2, 2, 1, 2, 1, 2, 1, 2, 1, 1,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 1, 2, 1, 1,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0,
    1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0,
    0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0,
    0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
        0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1,
        0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 1, 0, 0, -1, 0, -1,
        0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 1, 0, 0, -1, 0, -1,
        0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        1, 1, 2, 1, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        1, 1, 2, 1, 2, 1, 2, 1, 2, 2, 3, 2, 2, 1, 2, 1,
        1, 1, 2, 1, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 1, 0, 0, -1, 0, -1,
        0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 2, 1, 1, 0, 1, 0,
        0, 0, 1, 0, 0, -1, 0, -1, 0, 0, 1, 0, 0, -1, 0, -1}
};

/* The structure representing the neighborhood of a region. It is a priority
 queue organized as a heap (a binary tree, where each node has a key larger,
 resp. smaller, than its two children), stored in an array.
 Index 1 is the root, its children are at index 2 and 3.
 Therefore, the parent of node at k is at k/2 and its children at 2*k and 2*k+1.
 Index 0 is only a temporary buffer to swap two nodes. */
enum TypeOfTree { AMBIGUOUS, MIN, MAX, INVALID };
typedef struct {
    struct point_plane point;   /* Neighbor pixel */
    float value;                /* Its gray level */
} Neighbor;
typedef struct {
    Neighbor *tabPoints;        /* The array of neighbors,
                                 * organized as a binary tree */
    int iNbPoints;              /* The size of the previous arrays */
    enum TypeOfTree type;       /* max- or min- oriented heap? */
    float otherBound;           /* Min gray level if max-oriented,
                                 * max if min-oriented */
} Neighborhood;

/* Structure to find connections between shapes. This is used when a monotone
 section is extracted. The goal is to find the parent of its largest shape. The
 gray level of the parent is known, so as a point of the parent, since we use
 in fact an image of connections */
typedef struct {
    Shape shape;                /* Largest shape of the monotone section */
    float level;                /* Connection level */
} Connection;

/* Well, here are global variables. Ugly, but avoids to weigh the code,
 since they are used almost everywhere */
static int iWidth, iHeight;
static int iMinArea, iMaxArea, iAreaImage, iHalfAreaImage, iPerimeterImage;
static int iExploration;        /* Used to avoid reinitializing images */
static Point_plane tabPointsInShape;
static int **tabtabVisitedNeighbors;    /* Exterior boundary */
static Shapes pGlobalTree;
static int iAtBorder;           /* #points meeting at border of image */

#define ARRAY_2D_ALLOC(array, iWidth, iHeight, type) \
(array) = (type**) malloc((iHeight)*sizeof(type*)); \
if((array) == NULL) mwerror(FATAL, 1, "ARRAY_2D_ALLOC --> memory"); \
(array)[0] = (type*) calloc((iWidth)*(iHeight), sizeof(type)); \
if((array)[0] == NULL) mwerror(FATAL, 1, "ARRAY_2D_ALLOC --> memory"); \
for(i = 1; i < (iHeight); i++) \
(array)[i] = (array)[i-1]+(iWidth);

/* ------------------------------------------------------------------------
 --------- Functions to manage the neighborhood structure ---------------
 ------------------------------------------------------------------------ */

/* Reinitialise the neighborhood, so that it will be used for a new region */
static void reinit_neighborhood(Neighborhood * pNeighborhood,
                                enum TypeOfTree type)
{
    pNeighborhood->iNbPoints = 0;
    pNeighborhood->type = type;
}

/* To allocate the structure representing the neighborhood of a region */
static void init_neighborhood(Neighborhood * pNeighborhood, int init_iMaxArea)
{
    init_iMaxArea = 4 * (init_iMaxArea + 1);
    if (init_iMaxArea > iWidth * iHeight)
        init_iMaxArea = iWidth * iHeight;
    
    pNeighborhood->tabPoints =
    (Neighbor *) malloc((init_iMaxArea + 1) * sizeof(Neighbor));
    if (pNeighborhood->tabPoints == NULL)
        mwerror(FATAL, 1,
                "init_neighborhood --> neighbors allocation error\n");
    reinit_neighborhood(pNeighborhood, AMBIGUOUS);
}

/* Free the structure representing the neighborhood of a region */
static void free_neighborhood(Neighborhood * pNeighborhood)
{
    free(pNeighborhood->tabPoints);
}

#define ORDER_MAX(k,l) (tabPoints[k].value > tabPoints[l].value)
#define ORDER_MIN(k,l) (tabPoints[k].value < tabPoints[l].value)
#define SWAP(k,l) tabPoints[0] = tabPoints[k]; \
tabPoints[k] = tabPoints[l]; \
tabPoints[l] = tabPoints[0];
/* Put the last neighbor at a position so that we fix the heap */
static void fix_up(Neighborhood * pNeighborhood)
{
    Neighbor *tabPoints = pNeighborhood->tabPoints;
    int k = pNeighborhood->iNbPoints, l;
    
    if (pNeighborhood->type == MAX)
        while (k > 1 && ORDER_MAX(k, l = k >> 1))
        {
            SWAP(k, l);
            k = l;
        }
    else
        while (k > 1 && ORDER_MIN(k, l = k >> 1))
        {
            SWAP(k, l);
            k = l;
        }
}

#define ORDER_MAX2(k,l) (tabPoints[k].value >= tabPoints[l].value)
#define ORDER_MIN2(k,l) (tabPoints[k].value <= tabPoints[l].value)
/* Put the first neighbor at a position so that we fix the heap */
static void fix_down(Neighborhood * pNeighborhood)
{
    Neighbor *tabPoints = pNeighborhood->tabPoints;
    int N = pNeighborhood->iNbPoints, k = 1, l;
    
    if (pNeighborhood->type == MAX)
        while ((l = k << 1) <= N)
        {
            if (l < N && ORDER_MAX(l + 1, l))
                ++l;
            if (ORDER_MAX2(k, l))
                break;
            SWAP(k, l);
            k = l;
        }
    else
        while ((l = k << 1) <= N)
        {
            if (l < N && ORDER_MIN(l + 1, l))
                ++l;
            if (ORDER_MIN2(k, l))
                break;
            SWAP(k, l);
            k = l;
        }
}

/* Add the pixel (x,y), of gray-level VALUE, to the neighbor pixels */
static void add_neighbor(Neighborhood * pNeighborhood, short int x,
                         short int y, float value)
{
    Neighbor *pNewNeighbor;
    
    /* 0) Tag the pixel as 'visited neighbor' */
    tabtabVisitedNeighbors[y][x] = iExploration;
    /* 1) Update the fields TYPE and OTHERBOUND of PNEIGHBORHOOD */
    if (pNeighborhood->iNbPoints == 0)
        pNeighborhood->otherBound = value;
    else
        switch (pNeighborhood->type)
    {
        case MIN:
            if (value > pNeighborhood->otherBound)
                pNeighborhood->otherBound = value;
            else if (value < pNeighborhood->tabPoints[1].value)
                pNeighborhood->type = INVALID;
            break;
        case MAX:
            if (value < pNeighborhood->otherBound)
                pNeighborhood->otherBound = value;
            else if (value > pNeighborhood->tabPoints[1].value)
                pNeighborhood->type = INVALID;
            break;
        case AMBIGUOUS:
            if (value != pNeighborhood->tabPoints[1].value)
            {
                pNeighborhood->type =
                (value < pNeighborhood->tabPoints[1].value) ? MAX : MIN;
                pNeighborhood->otherBound = value;
            }
            break;
        case INVALID:
            return;
    }
    if (pNeighborhood->type == INVALID)
        return;
    /* 2) Add the point in the heap and update it */
    pNewNeighbor = &pNeighborhood->tabPoints[++pNeighborhood->iNbPoints];
    pNewNeighbor->point.x = x;  /* Initialise the new neighbor point */
    pNewNeighbor->point.y = y;
    pNewNeighbor->value = value;
    fix_up(pNeighborhood);      /* Update the heap of neighbors */
}

/* Remove the neighbor at the top of the heap, that is the root of the tree. */
static void remove_neighbor(Neighborhood * pNeighborhood)
{
    Neighbor *pTop = &pNeighborhood->tabPoints[1];
    float value = pTop->value;
    
    if (pNeighborhood->type == INVALID)
        return;
    *pTop = pNeighborhood->tabPoints[pNeighborhood->iNbPoints--];
    if (pNeighborhood->iNbPoints == 0)
        return;
    fix_down(pNeighborhood);
    if (value != pTop->value && pTop->value == pNeighborhood->otherBound)
        pNeighborhood->type = AMBIGUOUS;
}

/* ------------------------------------------------------------------------
 --------- Allocations of structures used in the algorithm --------------
 ------------------------------------------------------------------------ */

/* Allocate image of the tags for visited pixels and the visited neighbors.
 Do not be afraid about the parameters: pointers to 2-D arrays. */
static void init_image_of_visited_pixels(int ***ptabtabVisitedPixels)
{
    int i;
    
    ARRAY_2D_ALLOC(*ptabtabVisitedPixels, iWidth, iHeight, int);
    ARRAY_2D_ALLOC(tabtabVisitedNeighbors, iWidth, iHeight, int);
}

static void free_image_of_visited_pixels(int **tabtabVisitedPixels)
{
    free(tabtabVisitedPixels[0]);       /* Actually a 1-D array */
    free(tabtabVisitedPixels);
    
    free(tabtabVisitedNeighbors[0]);
    free(tabtabVisitedNeighbors);
}

/* Initialize the output image */
static void init_output_image(float *tabPixelsIn,
                              float ***ptabtabPixelsOutput)
{
    int i;
    
    *ptabtabPixelsOutput = (float **) malloc(iHeight * sizeof(float *));
    if (*ptabtabPixelsOutput == 0)
        mwerror(FATAL, 1, "init_output_image --> allocation error\n");
    for (i = 0; i < iHeight; i++)
        (*ptabtabPixelsOutput)[i] = tabPixelsIn + i * iWidth;
}

static void free_output_image(float **tabtabPixelsOutput)
{
    free(tabtabPixelsOutput);
}

static void init_region(int init_region_iMaxArea)
{
    tabPointsInShape =
    (Point_plane) malloc(init_region_iMaxArea *
                         sizeof(struct point_plane));
    if (tabPointsInShape == NULL)
        mwerror(FATAL, 1,
                "init_region --> impossible to allocate the array\n");
}

static void free_region(void)
{
    free(tabPointsInShape);
}

/* ------------------------------------------------------------------------
 --------- The core extraction algorithm --------------------------------
 ------------------------------------------------------------------------ */

/* Is pixel (x, y) a local minimum? */
static char is_local_min(float **ou, short int x, short int y,
                         char b8Connected)
/* A 2-D array of the image */
{
    float v;
    char n = 0;
    
    v = ou[y][x];
    return (x == iWidth - 1 || (ou[y][x + 1] > v && ++n) || ou[y][x + 1] == v)
    && (x == 0 || (ou[y][x - 1] > v && ++n) || ou[y][x - 1] == v)
    && (y == iHeight - 1 || (ou[y + 1][x] > v && ++n)
        || ou[y + 1][x] == v) && (y == 0 || (ou[y - 1][x] > v && ++n)
                                  || ou[y - 1][x] == v)
    && (b8Connected == 0
        ||
        ((x == iWidth - 1 || y == 0 || (ou[y - 1][x + 1] > v && ++n)
          || ou[y - 1][x + 1] == v) && (x == iWidth - 1
                                        || y == iHeight - 1
                                        || (ou[y + 1][x + 1] > v && ++n)
                                        || ou[y + 1][x + 1] == v)
         && (x == 0 || y == iHeight - 1 || (ou[y + 1][x - 1] > v && ++n)
             || ou[y + 1][x - 1] == v) && (x == 0 || y == 0
                                           || (ou[y - 1][x - 1] > v
                                               && ++n)
                                           || ou[y - 1][x - 1] == v)))
    && n != 0;
}

/* Is pixel (x,y) a local maximum? */
static char is_local_max(float **ou, short int x, short int y,
                         char b8Connected)
/* A 2-D array of the image */
{
    float v;
    char n = 0;
    
    v = ou[y][x];
    return (x == iWidth - 1 || (ou[y][x + 1] < v && ++n) || ou[y][x + 1] == v)
    && (x == 0 || (ou[y][x - 1] < v && ++n) || ou[y][x - 1] == v)
    && (y == iHeight - 1 || (ou[y + 1][x] < v && ++n)
        || ou[y + 1][x] == v) && (y == 0 || (ou[y - 1][x] < v && ++n)
                                  || ou[y - 1][x] == v)
    && (b8Connected == 0
        ||
        ((x == iWidth - 1 || y == 0 || (ou[y - 1][x + 1] < v && ++n)
          || ou[y - 1][x + 1] == v) && (x == iWidth - 1
                                        || y == iHeight - 1
                                        || (ou[y + 1][x + 1] < v && ++n)
                                        || ou[y + 1][x + 1] == v)
         && (x == 0 || y == iHeight - 1 || (ou[y + 1][x - 1] < v && ++n)
             || ou[y + 1][x - 1] == v) && (x == 0 || y == 0
                                           || (ou[y - 1][x - 1] < v
                                               && ++n)
                                           || ou[y - 1][x - 1] == v)))
    && n != 0;
}

/* Set pixels and saddle points in `tabPoints' at level newGray */
static void levelize(float **tabtabPixelsOutput, Point_plane tabPoints,
                     int iNbPoints, float newGray)
{
    int i;
    for (i = iNbPoints - 1; i >= 0; i--)
        tabtabPixelsOutput[tabPoints[i].y][tabPoints[i].x] = newGray;
}

/* Return, coded in one byte, the local configuration around the pixel (x,y) */
static unsigned char configuration(int **tabtabVisitedPixels, short int x,
                                   short int y)
{
    short int iMaxX = iWidth - 1, iMaxY = iHeight - 1;
    unsigned char cPattern = 0;
    
    if (x != 0)
    {
        if (tabtabVisitedPixels[y][x - 1] == iExploration)
            cPattern = WEST;
        if ((y == 0 && iAtBorder) ||
            (y > 0 && tabtabVisitedPixels[y - 1][x - 1] == iExploration))
            cPattern |= NORTH_WEST;
        if ((y == iMaxY && iAtBorder) ||
            (y < iMaxY && tabtabVisitedPixels[y + 1][x - 1] == iExploration))
            cPattern |= SOUTH_WEST;
    }
    else if (iAtBorder)
        cPattern = SOUTH_WEST | WEST | NORTH_WEST;
    
    if (x != iMaxX)
    {
        if (tabtabVisitedPixels[y][x + 1] == iExploration)
            cPattern |= EAST;
        if ((y == 0 && iAtBorder) ||
            (y > 0 && tabtabVisitedPixels[y - 1][x + 1] == iExploration))
            cPattern |= NORTH_EAST;
        if ((y == iMaxY && iAtBorder) ||
            (y < iMaxY && tabtabVisitedPixels[y + 1][x + 1] == iExploration))
            cPattern |= SOUTH_EAST;
    }
    else if (iAtBorder)
        cPattern |= SOUTH_EAST | EAST | NORTH_EAST;
    
    if ((y == 0 && iAtBorder) ||
        (y > 0 && tabtabVisitedPixels[y - 1][x] == iExploration))
        cPattern |= NORTH;
    
    if ((y == iMaxY && iAtBorder) ||
        (y < iMaxY && tabtabVisitedPixels[y + 1][x] == iExploration))
        cPattern |= SOUTH;
    
    return cPattern;
}

/* Insert a new shape and its siblings in the tree, with parent pParent */
static void insert_children(Shape pParent, Shape pNewChildToInsert)
{
    Shape pSibling = pNewChildToInsert;
    while (pSibling->next_sibling != NULL)
    {
        pSibling->parent = pParent;
        pSibling = pSibling->next_sibling;
    }
    pSibling->parent = pParent;
    pSibling->next_sibling = pParent->child;
    pParent->child = pNewChildToInsert;
}

static Shape new_shape(int iCurrentArea, float currentGrayLevel,
                       char bOfInferiorType, Shape pChild)
/* Supposed to have no sibling. Can be NULL */
{
    Shape pNewShape = &pGlobalTree->the_shapes[pGlobalTree->nb_shapes++];
    
    pNewShape->inferior_type = bOfInferiorType;
    pNewShape->value = currentGrayLevel;
    pNewShape->open = (char) (iAtBorder != 0);
    pNewShape->area = iCurrentArea;
    pNewShape->removed = 0;
    pNewShape->pixels = NULL;
    pNewShape->data = NULL;
    pNewShape->boundary = (Flist) NULL;
    pNewShape->data_size = 0;
    /* Make links */
    pNewShape->parent = NULL;
    pNewShape->next_sibling = NULL;
    pNewShape->child = pChild;
    if (pChild != NULL)
        pChild->parent = pNewShape;
    return pNewShape;
}

/* Knowing that the last extracted shape contains the points, update,
 for each one, the smallest shape containing it */
static void update_smallest_shapes(Point_plane tabPoints, int iNbPoints)
{
    int i, iIndex;
    Shape pNewShape, pRoot = &pGlobalTree->the_shapes[0];
    
    pNewShape = &pGlobalTree->the_shapes[pGlobalTree->nb_shapes - 1];
    for (i = iNbPoints - 1; i >= 0; i--)
    {
        iIndex = tabPoints[i].y * iWidth + tabPoints[i].x;
        if (pGlobalTree->smallest_shape[iIndex] == pRoot)
            pGlobalTree->smallest_shape[iIndex] = pNewShape;
    }
}

/* Find children of the last constructed monotone section, which is composed
 of the interval between pSmallestShape and the last extracted shape. That is,
 find shapes in other monotone sections whose parent is inside this interval */
static void connect(Point_plane tabPoints, int iNbPoints,
                    Connection * tabConnections, Shape pSmallestShape)
{
    int i, iIndex;
    Shape pShape, pParent;
    float level;
    
    for (i = iNbPoints - 1; i >= 0; i--)
    {
        iIndex = tabPoints[i].y * iWidth + tabPoints[i].x;
        pShape = tabConnections[iIndex].shape;
        if (pShape != NULL)
        {
            level = tabConnections[iIndex].level;
            pParent = pSmallestShape;
            while (pParent->value != level)
            {
                assert(pParent !=
                       &pGlobalTree->the_shapes[pGlobalTree->nb_shapes - 1]);
                pParent = pParent->parent;
            }
            insert_children(pParent, pShape);
            tabConnections[iIndex].shape = NULL;
        }
    }
}

/* Make a new connection structure at the given point */
static void new_connection(Point_plane pPoint, float level,
                           Connection * tabConnections)
{
    int iIndex;
    Shape pSibling, pShape =
    &pGlobalTree->the_shapes[pGlobalTree->nb_shapes - 1];
    
    iIndex = pPoint->y * iWidth + pPoint->x;
    if (tabConnections[iIndex].shape == NULL)
    {
        tabConnections[iIndex].shape = pShape;
        tabConnections[iIndex].level = level;
    }
    else
    {
        assert(tabConnections[iIndex].level == level);
        pSibling = tabConnections[iIndex].shape;
        while (pSibling->next_sibling != NULL)
            pSibling = pSibling->next_sibling;
        pSibling->next_sibling = pShape;
    }
}

/* Is the neighbor pixel already stored for this exploration? */
#define NEIGHBOR_NOT_STORED(x,y) (tabtabVisitedNeighbors[y][x] < iExploration)

/* Store the 4-neighbors of pixel (x,y) */
static void store_4neighbors(float **ou, short int x, short int y,
                             Neighborhood * pNeighborhood)
{
    if (x > 0 && NEIGHBOR_NOT_STORED(x - 1, y))
        add_neighbor(pNeighborhood, x - 1, y, ou[y][x - 1]);
    if (x < iWidth - 1 && NEIGHBOR_NOT_STORED(x + 1, y))
        add_neighbor(pNeighborhood, x + 1, y, ou[y][x + 1]);
    if (y > 0 && NEIGHBOR_NOT_STORED(x, y - 1))
        add_neighbor(pNeighborhood, x, y - 1, ou[y - 1][x]);
    if (y < iHeight - 1 && NEIGHBOR_NOT_STORED(x, y + 1))
        add_neighbor(pNeighborhood, x, y + 1, ou[y + 1][x]);
}

/* Store the diagonal neighbors of pixel (x,y) */
static void store_8neighbors(float **ou, short int x, short int y,
                             Neighborhood * pNeighborhood)
{
    if (x > 0)
    {
        if (y > 0 && NEIGHBOR_NOT_STORED(x - 1, y - 1))
            add_neighbor(pNeighborhood, x - 1, y - 1, ou[y - 1][x - 1]);
        if (y < iHeight - 1 && NEIGHBOR_NOT_STORED(x - 1, y + 1))
            add_neighbor(pNeighborhood, x - 1, y + 1, ou[y + 1][x - 1]);
    }
    if (++x < iWidth)
    {
        if (y > 0 && NEIGHBOR_NOT_STORED(x, y - 1))
            add_neighbor(pNeighborhood, x, y - 1, ou[y - 1][x]);
        if (y < iHeight - 1 && NEIGHBOR_NOT_STORED(x, y + 1))
            add_neighbor(pNeighborhood, x, y + 1, ou[y + 1][x]);
    }
}

/* Add the points in the neighborhood of gray level currentGrayLevel to the
 region tabIsoPointsInShape and return 1 if a new shape is (maybe) detected.
 This "maybe" is linked to `pIgnoreHoles', indicating if we can count the
 holes. New points are added to `tabIsoPointsInShape'
 from position `pCurrentArea'.
 This value is changed at exit in case of success. */
static char add_iso_level(Point_plane tabIsoPointsInShape, int *pCurrentArea,
                          float currentGrayLevel,
                          Neighborhood * pNeighborhood, float **ou,
                          int **tabtabVisitedPixels, char *p8Connected,
                          char *pIgnoreHoles)
{
    short int x, y;
    Neighbor *pNeighbor;
    int iCurrentArea, iNbHoles;
    unsigned char cPattern;
    
    iNbHoles = 0;
    iCurrentArea = *pCurrentArea;
    pNeighbor = &pNeighborhood->tabPoints[1];
    do
    {                           /* 1) Neighbor is added to the region */
        x = pNeighbor->point.x;
        y = pNeighbor->point.y;
        tabIsoPointsInShape[iCurrentArea].x = x;
        tabIsoPointsInShape[iCurrentArea++].y = y;
        if (!*pIgnoreHoles)
        {
            cPattern = configuration(tabtabVisitedPixels, x, y);
            iNbHoles += tabPatterns[(int) *p8Connected][cPattern];
        }
        if (x == 0 || x == iWidth - 1 || y == 0 || y == iHeight - 1)
            ++iAtBorder;
        tabtabVisitedPixels[y][x] = iExploration;
        /* 2) Store new neighbors */
        store_4neighbors(ou, x, y, pNeighborhood);
        if (pNeighborhood->type == MAX)
        {
            if (!*p8Connected)
                *pIgnoreHoles = *p8Connected = (char) 1;
            store_8neighbors(ou, x, y, pNeighborhood);
        }
        remove_neighbor(pNeighborhood);
    }
    while (iCurrentArea <= iMaxArea &&
           pNeighbor->value == currentGrayLevel &&
           pNeighborhood->type != INVALID);
    
    if (iCurrentArea <= iMaxArea &&
        iAtBorder != iPerimeterImage &&
        (!iAtBorder || iCurrentArea <= iHalfAreaImage) &&
        pNeighborhood->type != INVALID && (*pIgnoreHoles || iNbHoles == 0))
    {
        *pCurrentArea = iCurrentArea;
        return (char) 1;
    }
    return 0;
}

/* Extract the terminal branch containing the point (x,y) */
static void find_terminal_branch(float **ou, int **tabtabVisitedPixels,
                                 short int x, short int y, char b8Connected,
                                 Neighborhood * pNeighborhood,
                                 Connection * tabConnections)
{
    float level;
    int iArea, iLastShapeArea;
    char bUnknownHoles;
    Shape pSmallestShape, pLastShape;
    
restart_branch:
    ++iExploration;
    iArea = iAtBorder = 0;
    bUnknownHoles = 0;
    pSmallestShape = pLastShape = NULL;
    level = ou[y][x];
    reinit_neighborhood(pNeighborhood, b8Connected ? MAX : MIN);
    add_neighbor(pNeighborhood, x, y, level);
    while (add_iso_level(tabPointsInShape, &iArea,
                         level, pNeighborhood, ou, tabtabVisitedPixels,
                         &b8Connected, &bUnknownHoles) != 0)
    {
        if (bUnknownHoles)
        {
            assert(iArea != 0);
            if (pLastShape != NULL)
            {
                iArea = pLastShape->area;
                connect(tabPointsInShape, iArea, tabConnections,
                        pSmallestShape);
                new_connection(&tabPointsInShape[iArea], level,
                               tabConnections);
            }
            levelize(ou, tabPointsInShape, iArea, level);
            goto restart_branch;
        }
        if (iMinArea <= iArea)
        {                       /* Store new shape? */
            iLastShapeArea = (pLastShape == NULL) ? 0 : pLastShape->area;
            pLastShape = new_shape(iArea, level, !b8Connected, pLastShape);
            if (pSmallestShape == NULL)
                pSmallestShape = pLastShape;
            update_smallest_shapes(tabPointsInShape + iLastShapeArea,
                                   iArea - iLastShapeArea);
        }
        if (iAtBorder && iArea == iHalfAreaImage)
            break;
        bUnknownHoles = (char) (b8Connected
                                && pNeighborhood->type == AMBIGUOUS);
        if (bUnknownHoles)
            b8Connected = 0;
        level = pNeighborhood->tabPoints[1].value;
    }
    if (pLastShape != NULL)
    {
        connect(tabPointsInShape, iArea, tabConnections, pSmallestShape);
        if (iAtBorder && iArea == iHalfAreaImage)
            insert_children(pGlobalTree->the_shapes, pLastShape);
        else if (iArea != 0)
            new_connection(&tabPointsInShape[iArea], level, tabConnections);
    }
    levelize(ou, tabPointsInShape, iArea, level);
}

/* Scan the image, calling a procedure to extract terminal branch at each
 (not yet visited) local extremum */
static void scan(float **tabtabPixelsOutput, int **tabtabVisitedPixels,
                 Neighborhood * pNeighborhood, Connection * tabConnections)
{
    short int i, j;
    char b8Connected = 0;
    int iExplorationInit;
    
    iExplorationInit = iExploration;
    for (i = 0; i < iHeight; i++)
        for (j = 0; j < iWidth; j++)
            if (tabtabVisitedPixels[i][j] <= iExplorationInit &&
                (is_local_min(tabtabPixelsOutput, j, i, (char) 0) ||
                 (is_local_max(tabtabPixelsOutput, j, i, (char) 1)
                  && (b8Connected = 1) == 1)))
            {
                find_terminal_branch(tabtabPixelsOutput, tabtabVisitedPixels,
                                     j, i, b8Connected, pNeighborhood,
                                     tabConnections);
                b8Connected = 0;
            }
}

/* ------------------------------------------------------------------------
 --------- The main function --------------------------------------------
 ------------------------------------------------------------------------ */
/* The "Fast Level Set Transform" gives the tree of interiors of level lines
 (named 'shapes') representing the image.
 Only shapes of area >= *pMinArea are put in the tree. pMinArea==NULL means 1.
 Output: *pTree is filled */
void flst(int *pMinArea, Fimage pImageInput, Shapes pTree)
{
    float **tabtabPixelsOutput; /* Array accessing pixels of output image */
    Neighborhood neighborhood;  /* The neighborhood of the current region */
    int **tabtabVisitedPixels;  /* Image of last visit for each pixel */
    Connection *tabConnections;
    int i;
    
    pGlobalTree = pTree;
    iWidth = pImageInput->ncol;
    iHeight = pImageInput->nrow;
    
    iAreaImage = iWidth * iHeight;
    iHalfAreaImage = (iAreaImage + 1) / 2;
    if (iWidth == 1)
        iPerimeterImage = iHeight;
    else if (iHeight == 1)
        iPerimeterImage = iWidth;
    else
        iPerimeterImage = (iWidth + iHeight - 2) * 2;
    iMinArea = (pMinArea != NULL) ? *pMinArea : 1;
    if (iMinArea > iAreaImage)
        mwerror(USAGE, 1, "min area > image");
    
    pGlobalTree =
    mw_change_shapes(pTree, iHeight, iWidth, pImageInput->gray[0]);
    pGlobalTree->interpolation = 0;
    tabConnections = (Connection *) malloc(iAreaImage * sizeof(Connection));
    if (tabConnections == NULL)
        mwerror(FATAL, 1, "Image of largest shapes: allocation error\n");
    for (i = iAreaImage - 1; i >= 0; i--)
        tabConnections[i].shape = NULL;
    
    init_output_image(pImageInput->gray, &tabtabPixelsOutput);
    init_image_of_visited_pixels(&tabtabVisitedPixels);
    init_neighborhood(&neighborhood, iAreaImage);
    init_region(iAreaImage);
    
    iExploration = 0;
    /* Loop with increasing iMaxArea: speed optimization. Result correct
     * with only one call to `scan' and iMaxArea = iAreaImage */
    iMaxArea = 0;
    do
    {
        if (iMaxArea == 0)
            iMaxArea = INIT_MAX_AREA;
        else
            iMaxArea <<= STEP_MAX_AREA;
        if (iMaxArea == 0 || iMaxArea >= iHalfAreaImage)
        /* iMaxArea==0: overflow */
            iMaxArea = iAreaImage - 1;
        scan(tabtabPixelsOutput, tabtabVisitedPixels, &neighborhood,
             tabConnections);
    }
    while (iMaxArea + 1 < iAreaImage);
    
    /* Make connections with root */
    pTree->the_shapes[0].value = tabtabPixelsOutput[0][0];
    for (i = iAreaImage - 1; i >= 0; i--)
        if (tabConnections[i].shape != NULL)
        {
            assert(tabConnections[i].level == pTree->the_shapes[0].value);
            insert_children(pTree->the_shapes, tabConnections[i].shape);
        }
    
    free(tabConnections);
    free_image_of_visited_pixels(tabtabVisitedPixels);
    free_region();
    free_neighborhood(&neighborhood);
    free_output_image(tabtabPixelsOutput);
}

    
    
    /* This removes the shapes from the tree associated to pFloatImageInput
     that are too small (threshold *pMinArea). As a consequence all the remaining
     shapes of pFloatImageOutput are of area larger or equal than *pMinArea */
    
    void mw_fgrain(int *pMinArea, Fimage pFloatImageInput, Fimage pFloatImageOutput)
    {
        int i;
        Shapes pTree;
        
        if(mw_change_fimage(pFloatImageOutput, pFloatImageInput->nrow,
                            pFloatImageInput->ncol) == NULL)
            mwerror(FATAL, 1,
                    "fgrain --> Not enough memory to allocate the output image");
        if((pTree = mw_new_shapes()) == NULL)
            mwerror(FATAL, 1,
                    "fgrain --> Not enough memory to allocate the tree of shapes");
        
        /* Compute the Level Sets Transform of the input image */
        flst(NULL, pFloatImageInput, pTree); //, NULL, NULL);
        
        /* Kill too small grains.
         Bound i>0 because it is forbidden to delete the root, at index 0 */
        for(i = pTree->nb_shapes-1; i > 0; i--)
            if(pTree->the_shapes[i].area < *pMinArea)
                pTree->the_shapes[i].removed = (char)1;
        
        /* Reconstruct in pFloatImageOutput the modified tree of shapes */
        flst_reconstruct(pTree, pFloatImageOutput);
        
        mw_delete_shapes(pTree);
    }
    
    
    
    /*in and out must be allocated*/
    void fgrain(int MinArea, float *in, int nx, int ny, float *out)
    {
       
        
        int i;
        Fimage mwin = mw_change_fimage(NULL,ny,nx);
        Fimage mwout = mw_new_fimage();
        for(i=0;i<nx*ny;i++) mwin->gray[i] = in[i]; 
        
        mw_fgrain( &MinArea, mwin, mwout);
        
        for(i=0;i<nx*ny;i++) out[i] = mwout->gray[i]; 
        mw_delete_fimage(mwin);
        mw_delete_fimage(mwout);
    }

    
    
    
    

}


