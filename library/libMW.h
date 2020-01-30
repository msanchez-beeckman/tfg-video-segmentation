//
//  mw.h
//  frdSources
//
//  Created by Antoni Buades on 12/09/2013.
//  Copyright (c) 2013 Antoni Buades. All rights reserved.
//

#ifndef __lib__mw__
#define __lib__mw__

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <assert.h>




namespace libMW
{

/* definitions.h */
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef M_PI
#define M_PI           3.14159265358979323846   /* pi */
#endif
#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923   /* pi/2 */
#endif
#ifndef M_PI_4
#define M_PI_4         0.78539816339744830962   /* pi/4 */
#endif

#define WARNING  0
#define ERROR    1
#define FATAL    2
#define USAGE    3
#define INTERNAL 4

#define MODEL_RGB 0
#define MODEL_YUV 1
#define MODEL_HSI 2
#define MODEL_HSV 3

#define _MW_DATA_ASCII_FILE_HEADER "MegaWave2 - DATA ASCII file -\n"

#define mw_not_an_argument 1.0e9
#define mw_not_a_magnitude -1.0

#define mw_mtype_size 20
#define mw_ftype_size 20
#define mw_cmtsize 255
#define mw_namesize 255





typedef struct cimage {
    int nrow;              /**< number of rows (dy)    */
    int ncol;              /**< number of columns (dx) */
    int allocsize;         /**< size allocated (in bytes) for the gray plane */
    unsigned char *gray;   /**< the gray level plane (may be NULL)           */
    
    float scale;             /**< scale of the picture */
    char cmt[mw_cmtsize];    /**< comments */
    char name[mw_namesize];
    /**< name of the image */
    
    /* defines the signifiant part of the picture */
    int firstcol;     /**< first col not affected by left side effect */
    int lastcol;      /**< last col not affected by right side effect */
    int firstrow;     /**< first row not aff. by upper side effect    */
    int lastrow;      /**< last row not aff. by lower side effect     */
    
    /* for use in movies */
    struct cimage *previous;   /**< pointer to the prev image */
    struct cimage *next;       /**< pointer to the next image */
} *Cimage;

typedef struct fimage {
    int nrow;       /**< number of rows (dy)    */
    int ncol;       /**< number of columns (dx) */
    int allocsize;
    /**< size allocated (in bytes) for the gray plane */
    float *gray;    /**< the gray level plane (may be NULL)           */
    
    float scale;             /**< scale of the picture */
    char cmt[mw_cmtsize];    /**< comments */
    char name[mw_namesize];
    /**< name of the image */
    
    /* defines the signifiant part of the picture */
    int firstcol;     /**< first col not affected by left side effect */
    int lastcol;      /**< last col not affected by right side effect */
    int firstrow;     /**< first row not aff. by upper side effect    */
    int lastrow;      /**< last row not aff. by lower side effect     */
    
    /* for use in movies */
    struct fimage *previous;   /**< pointer to the prev image */
    struct fimage *next;       /**< pointer to the next image */
} *Fimage;

typedef struct ccimage {
    int nrow;               /**< number of rows (dy)    */
    int ncol;               /**< number of columns (dx) */
    int allocsize;          /**< size allocated (in bytes) for the gray plane */
    
    unsigned char model;    /**< model of the color system           */
    unsigned char *red;     /**< the red   level plane (may be NULL) */
    unsigned char *green;   /**< the green level plane (may be NULL) */
    unsigned char *blue;    /**< the blue  level plane (may be NULL  */
    
    float scale;             /**< scale of the picture */
    char cmt[mw_cmtsize];    /**< comments             */
    char name[mw_namesize];
    /**< name of the image    */
    
    /* defines the signifiant part of the picture */
    int firstcol;     /**< first col not affected by left side effect */
    int lastcol;      /**< last col not affected by right side effect */
    int firstrow;     /**< first row not aff. by upper side effect    */
    int lastrow;      /**< last row not aff. by lower side effect     */
    
    /* for use in movies */
    struct ccimage *previous;   /**< pointer to the prev image */
    struct ccimage *next;       /**< pointer to the next image */
} *Ccimage;

typedef struct cfimage {
    int nrow;       /**< number of rows (dy)    */
    int ncol;       /**< number of columns (dx) */
    int allocsize;
    /**< size allocated (in bytes) for the gray plane */
    
    unsigned char model;  /**< model of the color system           */
    float *red;           /**< the red   level plane (may be NULL) */
    float *green;         /**< the green level plane (may be NULL) */
    float *blue;          /**< the blue  level plane (may be NULL) */
    
    float scale;             /**< scale of the picture */
    char cmt[mw_cmtsize];    /**< comments */
    char name[mw_namesize];
    /**< name of the image */
    
    /* defines the signifiant part of the picture */
    int firstcol;     /**< first col not affected by left side effect */
    int lastcol;      /**< last col not affected by right side effect */
    int firstrow;     /**< first row not aff. by upper side effect    */
    int lastrow;      /**< last row not aff. by lower side effect     */
    
    /* for use in movies */
    struct cfimage *previous;   /**< pointer to the prev image */
    struct cfimage *next;       /**< pointer to the next image */
} *Cfimage;

typedef struct point_curve {
    int x, y;
    /**< coordinates of the point */
    
    /* for use in curve */
    struct point_curve *previous;   /**< pointer to the prev point */
    struct point_curve *next;       /**< pointer to the next point */
} *Point_curve;

typedef struct curve {
    Point_curve first;  /** pointer to the first point of the curve */
    
    /* For use in curves */
    struct curve *previous;   /** pointer to the prev curve */
    struct curve *next;       /** pointer to the next curve */
} *Curve;
typedef struct curves {
    char cmt[mw_cmtsize];    /** comments                   */
    char name[mw_namesize];
    /** name of the set            */
    Curve first;             /** pointer to the first curve */
} *Curves;

typedef struct point_fcurve {
    float x, y;  /**< coordinates of the point */
    
    /* for use in curve */
    struct point_fcurve *previous;   /**< pointer to the prev point */
    struct point_fcurve *next;       /**< pointer to the next point */
} *Point_fcurve;

typedef struct fcurve {
    Point_fcurve first;  /** pointer to the first point of the fcurve */
    
    /* For use in curves */
    struct fcurve *previous;   /** pointer to the prev curve */
    struct fcurve *next;       /** pointer to the next curve */
} *Fcurve;
typedef struct fcurves {
    char cmt[mw_cmtsize];    /** comments                   */
    char name[mw_namesize];
    /** name of the set            */
    Fcurve first;             /** pointer to the first curve */
} *Fcurves;

typedef struct point_dcurve {
    double x, y;  /**< coordinates of the point */
    
    /* for use in curve */
    struct point_dcurve *previous;   /**< pointer to the prev point */
    struct point_dcurve *next;       /**< pointer to the next point */
} *Point_dcurve;

typedef struct dcurve {
    Point_dcurve first;  /** pointer to the first point of the fcurve */
    
    /* For use in curves */
    struct dcurve *previous;   /** pointer to the prev curve */
    struct dcurve *next;       /** pointer to the next curve */
} *Dcurve;
typedef struct dcurves {
    char cmt[mw_cmtsize];    /** comments                   */
    char name[mw_namesize];
    /** name of the set            */
    Dcurve first;             /** pointer to the first curve */
} *Dcurves;

typedef struct polygon {
    /* the number of elements is given by nb_channels */
    int nb_channels;    /**< number of channels */
    float *channel;     /**< tab to the channel */
    Point_curve first;  /**< pointer to the first point of the curve */
    
    /* for use in polygons only */
    struct polygon *previous;   /**< pointer to the prev poly. (may be NULL) */
    struct polygon *next;       /**< pointer to the next poly. (may be NULL) */
} *Polygon;
typedef struct polygons {
    char cmt[mw_cmtsize];    /**< comments                     */
    char name[mw_namesize];
    /**< name of the set              */
    Polygon first;           /**< pointer to the first polygon */
} *Polygons;

typedef struct fpolygon {
    /* the number of elements is given by nb_channels */
    int nb_channels;     /**< number of channels */
    float *channel;      /**< tab to the channel */
    Point_fcurve first;  /**< pointer to the first point of the curve */
    
    /* for use in polygons only */
    struct fpolygon *previous;   /**< pointer to the prev poly. (may be NULL) */
    struct fpolygon *next;       /**< pointer to the next poly. (may be NULL) */
} *Fpolygon;
typedef struct fpolygons {
    char cmt[mw_cmtsize];    /**< comments                     */
    char name[mw_namesize];
    /**< name of the set              */
    Fpolygon first;          /**< pointer to the first polygon */
} *Fpolygons;

typedef struct point_type {
    unsigned char type;  /**< Type of the point
                          *   0 : regular point
                          *   1 : point in the image's border
                          *   2 : T-junction
                          *   3 : Tau-junction
                          *   4 : X-junction
                          *   5 : Y-junction
                          */
    struct point_type *previous;   /**< pointer to the prev point */
    struct point_type *next;       /**< pointer to the next point */
} *Point_type;

typedef struct morpho_line {
    Point_curve first_point;  /**< pointer to the first point
                               *   of the morpho_line curve                 */
    Point_type first_type;    /**< pointer to the first Point_type          */
    float minvalue;           /**< min gray level value of this morpho line */
    float maxvalue;           /**< max gray level value of this morpho line */
    unsigned char open;       /**< 0 if the morpho line is closed
                               *   opened otherwise                 */
    float data;               /**< user-defined data field (saved)  */
    void *pdata;              /**< user-defined data field
                               *   pointer to something (not saved) */
    
    /* for use in mimage */
    struct morpho_sets *morphosets;   /**< pointer to the morpho sets */
    unsigned int num;                 /**< morpho line number         */
    struct morpho_line *previous;     /**< pointer to the prev m.l.   */
    struct morpho_line *next;         /**< pointer to the next m.l.   */
} *Morpho_line;

typedef struct fmorpho_line {
    Point_fcurve first_point;  /**< pointer to the first point
                                *   of the morpho_line curve                 */
    Point_type first_type;     /**< pointer to the first Point_type          */
    float minvalue;            /**< min gray level value of this morpho line */
    float maxvalue;            /**< max gray level value of this morpho line */
    unsigned char open;        /**< 0 if the morpho line is closed
                                *   opened otherwise                 */
    float data;                /**< user-defined data field (saved)  */
    void *pdata;               /**< user-defined data field
                                *   pointer to something (not saved) */
    
    /* for use in mimage */
    struct fmorpho_line *previous;     /**< pointer to the prev m.l.   */
    struct fmorpho_line *next;         /**< pointer to the next m.l.   */
} *Fmorpho_line;

typedef struct hsegment {
    int xstart;
    /**< left  x-coordinate of the segment */
    int xend;    /**< right x-coordinate of the segment */
    int y;       /**< y-coordinate of the segment       */
    struct hsegment *previous;   /**< pointer to the prev segment */
    struct hsegment *next;       /**< pointer to the next segment */
} *Hsegment;

typedef struct morpho_set {
    unsigned int num;        /**< morpho set number                */
    Hsegment first_segment;  /**< pointer to the first segment     */
    Hsegment last_segment;   /**< pointer to the last segment      */
    float minvalue;          /**< min gray level value of this set */
    float maxvalue;          /**< max gray level value of this set */
    unsigned char stated;    /**< 1 if this m.s. has already been stated,
                              *   0 otherwise                              */
    int area;                /**< area of the set
                              *   (number of pixels belonging to this set) */
    struct morpho_sets *neighbor;   /**< pointer to a chain
                                     *   of neighbor morpho sets */
} *Morpho_set;
typedef struct morpho_sets {
    Morpho_set morphoset;           /**< pointer to the current morpho set */
    struct morpho_sets *previous;   /**< pointer to the prev morpho sets   */
    struct morpho_sets *next;       /**< pointer to the next morpho sets   */
    /* for use in mimage */
    struct morpho_line *morpholine;    /**< pointer to the morpho line */
} *Morpho_sets;

typedef struct mimage {
    char cmt[mw_cmtsize];    /**< comments                          */
    char name[mw_namesize];
    /**< name of the set                   */
    int nrow;                /**< number of rows (dy)               */
    int ncol;                /**< number of columns (dx)            */
    float minvalue;          /**< min gray level value in the image */
    float maxvalue;          /**< max gray level value in the image */
    Morpho_line first_ml;    /**< pointer to the first morpho line  */
    Fmorpho_line first_fml;  /**< Pointer to the first morpho line  */
    Morpho_sets first_ms;    /**< Pointer to the first morpho sets  */
} *Mimage;

#include <float.h>
#define MORPHO_INFTY FLT_MAX

typedef struct color {
    unsigned char model;  /**< model of the colorimetric system   */
    float red;            /**< the red   value if model=MODEL_RGB */
    float green;          /**< the green value if model=MODEL_RGB */
    float blue;           /**< the blue  value if model=MODEL_RGB */
} Color;

typedef struct cmorpho_line {
    Point_curve first_point;  /**< pointer to the first point
                               *   of the cmorpho_line curve        */
    Point_type first_type;    /**< pointer to the first Point_type  */
    Color minvalue;           /**< min color of this cmorpho line   */
    Color maxvalue;           /**< max color of this cmorpho line   */
    unsigned char open;       /**< 0 if the cmorpho line is closed
                               *   opened otherwise                 */
    float data;               /**< user-defined data field (saved)  */
    void *pdata;              /**< user-defined data field
                               *   pointer to something (not saved) */
    
    /* for use in mimage */
    struct cmorpho_sets *cmorphosets;   /**< pointer to the cmorpho sets */
    unsigned int num;                   /**< cmorpho line number         */
    struct cmorpho_line *previous;      /**< pointer to the prev cm.l.   */
    struct cmorpho_line *next;          /**< pointer to the next cm.l.   */
} *Cmorpho_line;

typedef struct cfmorpho_line {
    Point_fcurve first_point;  /**< pointer to the first point
                                *   of the cmorpho_line curve        */
    Point_type first_type;     /**< pointer to the first Point_type  */
    Color minvalue;            /**< min color of this cmorpho line   */
    Color maxvalue;            /**< max color of this cmorpho line   */
    unsigned char open;        /**< 0 if the cmorpho line is closed
                                *   opened otherwise                 */
    float data;                /**< user-defined data field (saved)  */
    void *pdata;               /**< user-defined data field
                                *   pointer to something (not saved) */
    
    /* for use in mimage */
    struct cfmorpho_line *previous;    /**< pointer to the prev cm.l.  */
    struct cfmorpho_line *next;        /**< pointer to the next cm.l.  */
} *Cfmorpho_line;

typedef struct cmorpho_set {
    unsigned int num;        /**< cmorpho set number               */
    Hsegment first_segment;  /**< pointer to the first segment     */
    Hsegment last_segment;   /**< pointer to the last segment      */
    Color minvalue;          /**< min color of this set            */
    Color maxvalue;          /**< max color of this set            */
    unsigned char stated;    /**< 1 if this cm.s. has already been stated,
                              *   0 otherwise                               */
    int area;                /**< area of the set
                              *   (number of pixels belonging to this set)  */
    struct cmorpho_sets *neighbor;   /**< pointer to a chain
                                      *   of neighbor cmorpho sets */
} *Cmorpho_set;
typedef struct cmorpho_sets {
    Cmorpho_set cmorphoset;          /**< pointer to the current morpho set */
    struct cmorpho_sets *previous;   /**< pointer to the prev morpho sets   */
    struct cmorpho_sets *next;       /**< pointer to the next morpho sets   */
    /* for use in mimage */
    struct cmorpho_line *cmorpholine;    /**< pointer to the morpho line */
} *Cmorpho_sets;

/** morphological cimage */
typedef struct cmimage {
    char cmt[mw_cmtsize];     /**< comments                          */
    char name[mw_namesize];   /**< name of the set                   */
    int nrow;                 /**< number of rows (dy)               */
    int ncol;                 /**< number of columns (dx)            */
    Color minvalue;           /**< min color in the image            */
    Color maxvalue;           /**< max color in the image            */
    Cmorpho_line first_ml;    /**< pointer to the first cmorpho line */
    Cfmorpho_line first_fml;  /**< Pointer to the first cmorpho line */
    Cmorpho_sets first_ms;    /**< Pointer to the first cmorpho sets */
} *Cmimage;

typedef struct flist {
    int size;        /**< size (number of elements)                     */
    int max_size;    /**< currently allocated size (number of ELEMENTS) */
    int dim;         /**< dimension (number of components per element)  */
    
    float *values;   /**< values = size * dim array
                      *   nth element = values[n*dim+i], i=0..dim-1 */
    
    int data_size;
    /**< size of data[] in bytes    */
    void *data;      /**< user defined field (saved) */
} *Flist;
typedef struct flists {
    char cmt[mw_cmtsize];    /**< comments */
    char name[mw_namesize];
    /**< name     */
    
    int size;                /**< size (number of elements) */
    int max_size;            /**< currently alloc. size (number of ELEMENTS) */
    
    Flist *list;             /**< array of Flist             */
    
    int data_size;           /**< size of data[] in bytes    */
    void *data;              /**< user defined field (saved) */
} *Flists;

typedef struct dlist {
    int size;        /**< size (number of elements)                     */
    int max_size;    /**< currently allocated size (number of ELEMENTS) */
    int dim;         /**< dimension (number of components per element)  */
    
    double *values;   /**< values = size * dim array
                       *   nth element = values[n*dim+i], i=0..dim-1 */
    
    int data_size;
    /**< size of data[] in bytes    */
    void *data;      /**< user defined field (saved) */
} *Dlist;
typedef struct dlists {
    char cmt[mw_cmtsize];    /**< comments */
    char name[mw_namesize];
    /**< name     */
    
    int size;                /**< size (number of elements) */
    int max_size;            /**< currently alloc. size (number of ELEMENTS) */
    
    Dlist *list;             /**< array of Dlist             */
    
    int data_size;           /**< size of data[] in bytes    */
    void *data;              /**< user defined field (saved) */
} *Dlists;

typedef struct cmovie {
    float scale;             /**< time scale of the movie (should be 1) */
    char cmt[mw_cmtsize];    /**< comments                              */
    char name[mw_namesize];
    /**< name of the image                     */
    Cimage first;            /**< pointer to the first image            */
} *Cmovie;

typedef struct ccmovie {
    float scale;             /**< time scale of the movie (should be 1) */
    char cmt[mw_cmtsize];    /**< comments                              */
    char name[mw_namesize];
    /**< name of the movie                     */
    Ccimage first;           /**< pointer to the first image            */
} *Ccmovie;

typedef struct fmovie {
    float scale;             /**< time scale of the movie (should be 1) */
    char cmt[mw_cmtsize];    /**< comments                              */
    char name[mw_namesize];
    /**< name of the image                     */
    Fimage first;            /**< pointer to the first image            */
} *Fmovie;

typedef struct cfmovie {
    float scale;             /**< time scale of the movie (should be 1) */
    char cmt[mw_cmtsize];    /**< comments                              */
    char name[mw_namesize];
    /**< name of the image                     */
    Cfimage first;           /**< pointer to the first image            */
} *Cfmovie;

typedef struct fsignal {
    int size;        /**< number of samples                              */
    int allocsize;
    /**< size allocated (in bytes) for the values plane */
    float *values;   /**< the samples                                    */
    
    float scale;     /**< scale of the signal                            */
    float shift;     /**< shifting of the signal with respect to zero    */
    float gain;      /**< gain of the signal                             */
    float sgrate;    /**< sampling rate of the signal                    */
    int bpsample;    /**< number of bits per sample for audio drivers    */
    
    char cmt[mw_cmtsize];    /**< comments          */
    char name[mw_namesize];
    /**< name of the image */
    
    /* Defines the signifiant part of the signal : */
    int firstp;      /**< first point not aff. by left side effect */
    int lastp;       /**< last point not aff. by right side effect */
    float param;     /**< distance between two succesive uncorrelated points */
} *Fsignal;



typedef struct point_plane {
    short x, y;  /**< coordinates of the point */
} *Point_plane;

typedef struct shape {
    char inferior_type;
    /**< indicates if it is extracted from a superior
     * or inferior level set
     */
    float value;   /**< limiting gray-level of the level set                 */
    char open;     /**< indicates if the shape meets the border of the image */
    int area;      /**< area of the shape = area of the cc of level set
                    * + areas of the holes
                    */
    char removed;
    /**< indicates whether the shape exists or not        */
    Point_plane pixels;  /**< the array of pixels contained in the shape */
    Flist boundary;      /**< the boundary curve defining the shape      */
    
    struct shape *parent, *next_sibling, *child;
    /**< data to include it in a tree
     * it has a parent (the smallest containing shape),
     * children (the largest contained shapes,
     * whose first is pChild and the others are its siblings),
     * and siblings (the other children of its parent)
     */
    
    int data_size;
    /**< size of data[] in bytes */
    void *data;     /**< user defined field (saved) */
} *Shape;

typedef struct shapes {
    char cmt[mw_cmtsize];    /**< comments                            */
    char name[mw_namesize];
    /**< name of the set                     */
    int nrow;                /**< number of rows (dy) of the image    */
    int ncol;                /**< number of columns (dx) of the image */
    int interpolation;       /**< interpolation used for the level lines:
                              * - 0 nearest neighbor
                              * - 1 bilinear
                              */
    Shape the_shapes;  /**< array of the shapes.
                        * the root of the tree is at index 0
                        */
    
    int nb_shapes;
    /**< the number of shapes
     * (the size of the array the_shapes)
     */
    Shape *smallest_shape;   /**< image giving for each pixel
                              * the smallest shape containing it
                              */
    int data_size;
    /**< size of data[] in bytes    */
    void *data;     /**< user defined field (saved) */
} *Shapes;








void mwdebug(char *fmt, ...);
void mwerror(int code, int exit_code, char *fmt, ...);


Fsignal mw_new_fsignal(void);
Fsignal mw_alloc_fsignal(Fsignal signal, int N);
void mw_delete_fsignal(Fsignal signal);
Fsignal mw_change_fsignal(Fsignal signal, int N);
void mw_clear_fsignal(Fsignal signal, float v);
void mw_copy_fsignal_values(Fsignal in, Fsignal out);
void mw_copy_fsignal_header(Fsignal in, Fsignal out);
void mw_copy_fsignal(Fsignal in, Fsignal out);



Cimage mw_new_cimage(void);
Cimage mw_alloc_cimage(Cimage image, int nrow, int ncol);
void mw_delete_cimage(Cimage image);
Cimage mw_change_cimage(Cimage image, int nrow, int ncol);
unsigned char mw_getdot_cimage(Cimage image, int x, int y);
void mw_plot_cimage(Cimage image, int x, int y, unsigned char v);
void mw_draw_cimage(Cimage image, int a0, int b0, int a1, int b1, unsigned char c);
void mw_clear_cimage(Cimage image, unsigned char v);
void mw_copy_cimage(Cimage in, Cimage out);
unsigned char **mw_newtab_gray_cimage(Cimage image);
unsigned char mw_isitbinary_cimage(Cimage image);




Fimage mw_new_fimage(void);
Fimage mw_alloc_fimage(Fimage image, int nrow, int ncol);
void mw_delete_fimage(Fimage image);
Fimage mw_change_fimage(Fimage image, int nrow, int ncol);
float mw_getdot_fimage(Fimage image, int x, int y);
void mw_plot_fimage(Fimage image, int x, int y, float v);
void mw_draw_fimage(Fimage image, int a0, int b0, int a1, int b1, float c);
void mw_clear_fimage(Fimage image, float v);
void mw_copy_fimage(Fimage in, Fimage out);
float **mw_newtab_gray_fimage(Fimage image);







Flist mw_new_flist(void);
Flist mw_realloc_flist(Flist l, int n);
Flist mw_enlarge_flist(Flist l);
Flist mw_change_flist(Flist l, int max_size, int size, int dimension);
void mw_delete_flist(Flist l);
void mw_clear_flist(Flist l, float v);
Flist mw_copy_flist(Flist in, Flist out);
Flists mw_new_flists(void);
Flists mw_realloc_flists(Flists ls, int n);
Flists mw_enlarge_flists(Flists ls);
Flists mw_change_flists(Flists ls, int max_size, int size);
void mw_delete_flists(Flists ls);
Flists mw_copy_flists(Flists in, Flists out);
Dlist mw_new_dlist(void);
Dlist mw_realloc_dlist(Dlist l, int n);
Dlist mw_enlarge_dlist(Dlist l);
Dlist mw_change_dlist(Dlist l, int max_size, int size, int dimension);
void mw_delete_dlist(Dlist l);
void mw_clear_dlist(Dlist l, double v);
Dlist mw_copy_dlist(Dlist in, Dlist out);
Dlists mw_new_dlists(void);
Dlists mw_realloc_dlists(Dlists ls, int n);
Dlists mw_enlarge_dlists(Dlists ls);
Dlists mw_change_dlists(Dlists ls, int max_size, int size);
void mw_delete_dlists(Dlists ls);
Dlists mw_copy_dlists(Dlists in, Dlists out);


Point_plane mw_new_point_plane(void);
Point_plane mw_change_point_plane(Point_plane point);
Shape mw_new_shape(void);
Shape mw_change_shape(Shape sh);
void mw_delete_shape(Shape shape);
Shape mw_get_not_removed_shape(Shape sh);
Shape mw_get_parent_shape(Shape sh);
Shape mw_get_first_child_shape(Shape sh);
Shape mw_get_next_sibling_shape(Shape sh);
Shape mw_get_smallest_shape(Shapes shs, int iX, int iY);
Shapes mw_new_shapes(void);
Shapes mw_alloc_shapes(Shapes shs, int nrow, int ncol, float value);
Shapes mw_change_shapes(Shapes shs, int nrow, int ncol, float value);
void mw_delete_shapes(Shapes shs);





void flstb_dualchain(Shapes pTree, Shape pShape, Flist pBoundary,
                     char *ctabtabSaddleValues);
void flst_reconstruct(Shapes pTree, Fimage pFloatImageOutput);
int fsaddles(Fimage pImage, Fimage pSaddlesImage);
void flst_pixels(Shapes pTree);

void flst(int *pMinArea, Fimage pImageInput, Shapes pTree);



    
    void fgrain(int MinArea, float *in, int nx, int ny, float *out);

    
    
    
}

#endif

