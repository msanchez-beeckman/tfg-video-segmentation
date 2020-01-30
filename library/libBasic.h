#ifndef __library__
#define __library__


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cmath>
#include <cassert>
#include <vector>
#include <getopt.h>
#include <getopt.h>

#include <fftw3.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>


#include <sys/types.h>
#include <unistd.h>

namespace libUSTG
{
    
    
#ifndef PI
#define PI 3.14159265358979323846264338327
#endif
    
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif
    
#define dTiny 1e-10
#define fTiny 0.0000000001f
#define fLarge 10000000000.0f
#define dLarge 1e+10
    
#define MAX(i,j) ( (i)<(j) ? (j):(i) )
#define MIN(i,j) ( (i)<(j) ? (i):(j) )
    
    
    //! Forward declarations
    class laMatrix;
    class laVector;
    
    
    void src_exit(const char* message);
   
    
    std::string int2string(int number);
    
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//! Float Value operations
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void ipClear(int *fpI,int fValue, int iLength);
    
	
    void fpClear(float *fpI,float fValue, int iLength);
	void fpCopy(float *fpI,float *fpO, int iLength);
	
	
    void dpClear(double *fpI,double fValue, int iLength);
    void dpCopy(double *fpI,double *fpO, int iLength);
    
    
    float fpMax(float *u,int *pos, int size);
	float fpMin(float *u,int *pos,int size);
	
    
	float fpVar(float *u,int size);
	float fpMean(float *u,int size);
    
    float fpVar(float *u,float *m, int size);
    float fpMean(float *u, float *m, int size);
    
    float fpMedian(float *u,int size);
    
    float fpWeightedMedian(float *u,float *w, int size);
    double dpWeightedMedian(double *u,double *w, int size);
    
    
    void fpCombine(float *u,float a,float *v,float b, float *w,  int size);
    
    void fiImageDrawPoint(float *igray, int pi, int pj, int radius, float value, int width, int height);
    void fiImageDrawCircle(float *igray, int pi,int pj, float radius, float value, int width, int height);
    void fiImageDrawLine(float *igray, int a0, int b0, int a1, int b1, float value, int width, int height);
    void fiImageDrawCircumference(float *igray, int pi,int pj, float radius, float value, int width, int height);
    void fiImageDrawEllipse(float *igray, int pi, int pj, float a, float b, float alpha, float value, int width, int height);
    
    
    void fpBinarize(float *u, float *v, float value, int inverse, int size);
    
    
    float fpDistLp(float *u, float *v, int p, int size);
    float fpDistLp(float *u, float *v, float *m, int p, int size);
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//! Float pointer ordering
	//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    struct stf_qsort
    {
        float value;
        float index;
    };
    
    
    struct std_qsort
    {
        double value;
        double index;
    };
    
    int order_stf_qsort_increasing(const void *pVoid1, const void *pVoid2);
    

    
	void fpQuickSort(float *fpI, int iLength, int inverse = 0 );
	void fpQuickSort(float *fpI, float *fpO, int iLength, int inverse = 0);
	
    void dpQuickSort(double *fpI, int iLength, int inverse = 0 );
    void dpQuickSort(double *fpI, double *fpO, int iLength, int inverse = 0);
    
    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
	//! Image Conversion
	//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
#define COEFF_YR 0.299
#define COEFF_YG 0.587
#define COEFF_YB 0.114
    
    
	
    //! Attention cannot be called with input=output pointers
    void fiRgb2Yuv(float *r,float *g,float *b,float *y,float *u,float *v,int width,int height);
	void fiYuv2Rgb(float *r,float *g,float *b,float *y,float *u,float *v, int width,int height);
	
	
	void fiRgb2YuvO(float *r,float *g,float *b,float *y,float *u,float *v,int width,int height);
	void fiYuvO2Rgb(float *r,float *g,float *b,float *y,float *u,float *v, int width,int height);
	
	
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Patch Statistics
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void fiComputeIntegralImage(float *in, float *out, int width, int height);
    
    void fiPatchStatistics(float *fpIn, float *fpMinV, float *fpMaxV, float *fpMeanV, float *fpVarV, float *fpMedianV, float fRadius, int iWidth, int iHeight);
    void fiPatchStatistics(float *fpIn, float *fpMinV, float *fpMaxV, float *fpMeanV, float *fpVarV, float *fpMedianV, int rx, int ry, int iWidth, int iHeight);
    
    void fiPatchMin(float *fpIn, float *fpMinV, float fRadius, int iWidth, int iHeight);
    void fiPatchMax(float *fpIn, float *fpMaxV, float fRadius, int iWidth, int iHeight);
    void fiPatchMean(float *fpIn, float *fpMeanV, float fRadius, int iWidth, int iHeight);
    void fiPatchVar(float *fpIn, float *fpVarV, float fRadius, int iWidth, int iHeight);
    void fiPatchMedian(float *fpIn, float *fpMedianV, float fRadius, int iWidth, int iHeight);
    
    void fiPatchMean(float *fpIn, float *fpMeanV, int rx, int ry, int iWidth, int iHeight);
    void fiPatchVar(float *fpIn, float *fpVarV, int rx, int ry, int iWidth, int iHeight);
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
	//! Image Convolution
	//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

    
    
#define BOUNDARY_CONDITION_NEUMANN 0
#define BOUNDARY_CONDITION_SYMMETRIC 1
    
	float* fiFloatGaussKernel(float std, int & size);
	void fiFloatDirectionalGaussKernel(float xsigma, float ysigma, float angle, float *kernel, int kwidth, int kheight);
    float * fiFloatDirectionalGaussKernelS(float xsigma, float ysigma, float angle, float *kernel, int kwidth, int kheight, int sign);

    
    void fiFloatHorizontalConvolution(float *u, float *v, int width, int height, float *kernel, int ksize, int boundary);
    void fiFloatVerticalConvolution(float *u, float *v, int width, int height, float *kernel,int ksize, int boundary);
	void fiGaussianConvol(float *u, float *v, int width, int height, float sigma, int boundary);
	void fiConvol(float *u,float *v,int width,int height,float *kernel,int kwidth,int kheight, int boundary);
	void fiSepConvol(float *u,float *v,int width,int height,float *xkernel, int xksize, float *ykernel, int yksize, int boundary);
	
    
    //! JL Lisani Fast 2D convolution (non negative kernel) by keeping only values up to the sum larger than pkernel
    struct kernel_item {
        float v;
        int i, j;
    };
    
    struct sorted_kernel {
        int nitems;
        int w, h;
        int ncut;
        struct kernel_item *items;
    };
    void sort_kernel(float *u, int w, int h, struct sorted_kernel *skernel, float pkernel);
    void fiConvol_skernel(float *u,float *v,int width,int height, struct sorted_kernel *skernel, int boundary);
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Add Noise
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void fpAddNoiseGaussian(float *u, float *v, float std, long int randinit, int size);
    void fpAddNoiseGaussianAfine(float *u,float *v, float a,float b,long int randinit, int size);
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Gradient Computation
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void fiComputeImageGradient(float * fpI,float *fpXgrad, float *fpYgrad, float *fpGrad, float * fpOri, int iWidth, int iHeight, char cType = 'f');
    
    void fiComputeImageGradient(float * fpI, float *fpGrad, float * fpOri, int iWidth, int iHeight, char cType = 'f');
    
    void fiComputeImageGradient(float * fpI, float *fpGrad, int iWidth, int iHeight, char cType = 'f');
    
    void fiComputeImageLaplacian(float *input, float *out, float sigma, int width, int height);
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Sampling functions
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void fiImageSample(float *igray,float *ogray, int factor, int width, int height);
    
    float *fiImageSample(float *input, float sampling_factor, int high_width,
                         int high_height, int & low_width, int & low_height);
    
    void fiImageSampleAglomeration(float *igray,float *ogray, int factor, int width, int height);
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Interpolation functions
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    float bicubic_interpolation_at(
                                   const float *input, // image to be interpolated
                                   const float  uu,    // x component of the vector field
                                   const float  vv,    // y component of the vector field
                                   const int    nx,    // image width
                                   const int    ny,    // image height
                                   const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                   const float  bvalue // return this value outside the region
    );
    
    
    void bicubic_interpolation_zoom(const float *input, const int nx, const int ny,  const float  fFactor, const int binterpolation, const float bvalue, float *output );
    
    void bicubic_interpolation_zoom(
                                    const float *input,     // image to be warped
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    const float  fFactorx,     // zoom factor
                                    const float  fFactory,     // zoom factor
                                    const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                    const float bvalue,           // value outside the region
                                    float       *output    // image warped with bicubic interpolation
    );
    
    void bilinear_interpolation_translation(
                                            const float *input,     // image to be warped
                                            const float u,         // x component of the vector field
                                            const float v,         // y component of the vector field
                                            float       *output,    // image warped with bicubic interpolation
                                            const int    nx,        // image width
                                            const int    ny        // image height
    );
    
    void bicubic_interpolation_warp( const float *input, const float *u,  const float *v,  const int nx,  const int ny,  const int binterpolation, const float  bvalue, float *output);
    
    void bicubic_interpolation_translation(const float *input,const float u,const float v,const int    nx, const int    ny,const int    binterpolation, const float  bvalue, float *output   );
    
    void nn_interpolation_zoom(const float *input, const int    nx, const int    ny,  const float  fFactor, float *output);
    
    void bicubic_homography_interpolation(float *input, int width, int height, laMatrix &H, float bg, float *out, int nwidth, int nheight);
    
  
    float bilinear_interpolation_at(
                                    const float *input,     // image to be warped
                                    const float  uu,
                                    const float  vv,
                                    const int    width,        // image width
                                    const int    height         // image height
    );
    
    void bilinear_interpolation_zoom(
                                     const float *input,     // image to be warped
                                     const int    nx,        // image width
                                     const int    ny,        // image height
                                     const float  fFactor,     // zoom factor
                                     float       *output    // image warped with bicubic interpolation
    );
    
    void bilinear_interpolation_zoom(
                                     const float *input,     // image to be warped
                                     const int    nx,        // image width
                                     const int    ny,        // image height
                                     const float  fFactorx,     // zoom factor
                                     const float  fFactory,     // zoom factor
                                     float       *output    // image warped with bicubic interpolation
    );
    
    
    void bilinear_interpolation_warp(
                                     const float *input,     // image to be warped
                                     const float *u,         // x component of the vector field
                                     const float *v,         // y component of the vector field
                                     float       *output,    // image warped with bicubic interpolation
                                     const int    nx,        // image width
                                     const int    ny        // image height
    );
    
    
    void nn_interpolation_warp(
                               const float *input,     // image to be warped
                               const float *u,         // x component of the vector field
                               const float *v,         // y component of the vector field
                               float       *output,    // image warped with bicubic interpolation
                               const int    nx,        // image width
                               const int    ny        // image height
    );
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Tabulation
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
#define LUTMAX 30.0
#define dLUTMAX 3e+1
#define LUTMAXM1 29.0
#define dLUTMAXM1 29e+0
#define LUTPRECISION 1000.0
#define dLUTPRECISION 1e+3
    
    
    void  wxFillExpLut(float *lut, int size);
    void  wxFillExpLut(double *lut, int size);
    float wxSLUT(float dif, float *lut);
    double wxSLUT(double dif, double *lut);

    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Patch Distances
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //! Distances are not normalized by number of pixels or channels
    float fiL2FloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int width0, int width1);
    float fiL2FloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int width0, int width1);
    float fiL2FloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1);
    float fiL2FloatDist_NN(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1);
    float fiL2FloatWDist ( float * u0, float *u1, int i0, int j0,int i1,int j1,int xradius, int yradius, float *kernel, int width0, int width1);
    float fiL2FloatWDist ( float ** u0, float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, float *kernel, int channels, int width0, int width1);
    
    
    //
    //! Patch Double Distances
    //
    
    double fiL2DoubleDist(double *u0, double *u1, int i0, int j0, int i1, int j1, int xradius, int yradius, int width0, int width1);
    double fiL2DoubleDist(double *u0, double *u1, int i0, int j0, int i1, int j1, int xradius, int width0, int width1);
    double fiL2DoubleDist(double **u0, double **u1, int i0, int j0, int i1, int j1, int xradius, int yradius, int channels, int width0, int width1);
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Histogram
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
#define HSP_NB_CASES 500
    
    //! histogram of real values images
    void fpHisto(float* input, laVector &histo, float *iminim, float *imaxim, int *n, float *s, int size, char flag);
    
    //! histogram equalization for positive integer images
    void fk_fill_histo(float *Histo,float *HistoInverse, int iRangeMax, float *in, int width, int height);
    
    void fk_histogram_specification(float *in1, float *in2, float *out, int width1, int height1, int width2, int height2);
    void fk_histogram_specification_mask(float *in1, float *mask1, float *in2, float *mask2, float *out, int width1, int height1, int width2, int height2);

    
    void fk_histogram_midway(float *in1, float *in2, float *out1, float *out2, int width1, int height1, int width2, int height2);
    void fk_histogram_midway_mask(float *in1, float* mask1, float *in2, float *mask2, float *out1, float *out2, int width1, int height1, int width2, int height2);
    void fk_histogram_midway_sequence(float **in, float **out, int nImages, int width, int height);
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! OLD STUFF
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    
    
    
    

    
    
    
	
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
	//! Patch Distances
	//
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    

    
    float fiBCFloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1);
    float fiCFloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1);
    
   
    
    
    
    
    
  
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! FFT Stuff
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    void fft1d(float *Xr,float *Xi,float *Yr,float *Yi, int inverse, int n);
	void fft2d(float *in_re, float *in_im,float *out_re, float *out_im, int i_flag, int width, int height);
	void fiFFTShearPascal(float dAmountOfShear, float *pFloatImageInput,float *pFloatImageOutput, int iAxis, float fDelta, int PleaseDoNotCenter, int width, int height);
	void fiFFTZoom(float *in, float *out, float zoomx, float zoomy, int width, int height);
	
    
    //! Convolution
    void fft_Gaussian_convolution(float **convolved, float **data, float std, int num_channels, int width, int height);
    void fft_Gaussian_convolution(float *convolved, float *data, float std, int width, int height);
    void fft_convolution(float *convolved, float *data, float std, int width, int height);
    
    

    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Splines Stuff  OLD
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    float vMW(float *in,int x,int y,float bg, int width, int height);
    void keysMW(float *c,float t,float a);
    void spline3MW(float *c,float t);
    void init_splinenMW(float *a,int n);
    float ipowMW(float x,int n);
    void splinenMW(float *c,float t,float *a,int n);
    float initcausalMW(float *c,int n,float z);
    float initanticausalMW(float *c,int n,float z);
    void invspline1DMW(float *c,int size,float *z,int npoles);
	void finvsplineMW(float *in,int order,float *out, int width, int height);
    float evaluate_splineMW(float *input, float *ref, float xp, float yp, float *ak, int order, float bg, int width, int height);
    
    
    
    void apply_planar_homography(float *input, int width, int height, laMatrix &H, float bg, int order, float *out, float x0, float y0, int nwidth, int nheight);
    void apply_planar_homography_zoom(float *input, int width, int height, laMatrix &H, float bg, int order, float *out, float x0, float y0, int nwidth, int nheight, float fZoom);
   
    
    void spline_interpolation_zoom(
                                   float *input,     // image to be warped
                                   const int    nx,        // image width
                                   const int    ny,        // image height
                                   const float  fFactor,   // zoom factor
                                   const int    order,     // order of interpolation
                                   const float bvalue,     // value outside the region
                                   float       *output     // image warped with bicubic interpolation
    );

    

    

	
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Geometrical Transformations
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void compute_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, laMatrix &H);
    void compute_planar_homography_n_points_new_svd(float *x0, float *y0, float *x1, float *y1, int n, laMatrix &H);
    int compute_ransac_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, int niter, float tolerance, laMatrix &H, int *accorded, int inewSVD);
    int compute_ransac_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, int niter, float tolerance, laMatrix &H, int inewSVD);

    
    
    void SVD(laMatrix &A, laMatrix& U, laVector& S, laMatrix& V);
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Numerical Analysis Classes
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
  
    
    
    
    class laVector {
    protected:
        int d_n;	// size of array. upper index is nn-1
        float *d_v;
    public:
        
        laVector();
        explicit laVector(int n);								// Zero-based array
        laVector(const float &a, int n);						// initialize to constant value
        laVector(const float *a, int n);						// Initialize to array
        
        laVector(const laVector &rhs);							// Copy constructor
        laVector(const char * filename);						
        
        laVector & operator=(const laVector &rhs);				// assignment
        laVector & operator=(const float &a);					// assign a to every element
        
		float & operator[](const int i);				// i'th element
		const float & operator[](const int i) const;
        
        void create(int n);
        void erase();
        
		laVector copyBlock(int i0, int length);
		
        void sort(int decreasing_order);
        
		float * v();
		
		int size() const;
        ~laVector();
        
        friend class laMatrix;
        
    };
    
    
    
    
	
    class laMatrix
    {
        
    protected:
        int d_n;			// nrows
        int d_m;			// ncols
        float **d_v;		// pointer
        
    public:
        
        
        //! Construction / Destruction
        
		laMatrix();
        laMatrix(int n, int m);
        laMatrix(const float &a, int n, int m);
        laMatrix(const float *a, int n, int m);
        laMatrix(const laMatrix &rhs);
        
        laMatrix & operator=(const laMatrix &rhs);
        laMatrix & operator=(const float &a);
        
        ~laMatrix();
        
        
		
        //! Basic operators
        
        float * operator[](const int i);	//subscripting: pointer to row i
        const float * operator[](const int i) const;
        
        float ** v();
        
        int nrows() const;
        int ncols() const;
        
        
        void create(int n, int m);
        
        
        
        //! Non member Arithmetic Operations
		
		friend laMatrix operator*  (float a, const laMatrix& rhs);                               // scalar matrix product
		friend laMatrix operator/  (const laMatrix& lhs, float a);                               // matrix scalar division
		friend laMatrix operator+  (const laMatrix& lhs, const laMatrix& rhs);                   // matrix sum
		friend laMatrix operator-  (const laMatrix& lhs, const laMatrix& rhs);                   // matrix subtraction
		friend laMatrix operator*  (const laMatrix& lhs, const laMatrix& rhs);                   // matrix product
		
		friend laVector operator*  (const laMatrix& lhs, const laVector & rhs);                  // matrix vector product
		
        
		
		//! Other
		laMatrix transposed();
        
        laMatrix copyBlock(int i0, int j0, int rowb, int colb);
        
        friend class laVector;
        
    };
    
    
    
	void luinv(laMatrix &a, laMatrix &inv);
    void lusolve(laMatrix &a, laVector &x, laVector &b);
    void invCC(laMatrix &inv);          //! Inversion of definite positive matrices
    
    void compute_svd(laMatrix &A, laMatrix &m_U, laMatrix &m_V, laVector &m_W);
    
    void compute_svd_double_without_coefs(laMatrix &Ain, laMatrix &m_Uin, laMatrix &m_Vin, laVector &m_Win);
    void compute_svd_double(laMatrix &Ain, laMatrix &m_Uin, laMatrix &m_Vin, laVector &m_Win);
    
    void compute_pca_svd(laMatrix &X, laVector &S, laMatrix &V, laMatrix &U);
    
    
    void  l2_baricenter(float **X,float *baricenter,int n, int p);
    void  l1_baricenter(float **X,float *baricenter,int n, int p, int niter);
    
    
    void kmeans(float **data, float **means, int *labels, int nIter, int kCenters, int nVectors, int pCoordinates);
	void linear_fitting(float * tpX,float* tpY, float &da, float &db, int ilength);
    
    
    
    
    void  center_data_columns(float **X,float *baricenter,int n, int p);
    void  estimate_noise_pca_variances(int d, int n, float fSigma, float fDelta, int *osize, float *curve);
    
    
    
    //!
    //! Deprecated Numerical Analysis
    //!
    
	float ** allocate_float_matrix(int nrows, int ncols);
	void desallocate_float_matrix(float **matrix, int nrows, int ncols);
	
	
	/*- Householder reduction of a real symmetric matrix A[0..n-1][0..n_1]. On output, A is ---*/
	/*- replaced by the ortogonal matrix Q effecting the transformation. d[0..n-1] returns ----*/
	/*- the diagonal elements of the diagonal matrix, and e[0..n-1] the off-diagonal elements -*/
	/*- with e[0]=0. 									   */
	void symmetric_2_tridiag_HH(float **a,float *d,float *e,int n);
	
    
    /* QL with implicit shifts, to determine eigenvalues and eigenvectors of  a real, symmetric, tridiagonal matrix */
	/* d[0..n-1] contains the diagonal elements of the matrix and as output the returns the eigenvalues             */
	/* e[0..n-1] contains the sub-diagonal elements with e[0] arbitrary, on output e is destroyed			*/
	/* z[0..n-1][0..n-1] contain the identity matrix in input or the output of symmetric_2_tridiag_HH if previously */
	/* applied. 													*/
	
	int eigenvalues_QLI(float * d, float *e,float **z, int n);
	
    
    
	//void  pca_center_data(float **X,float *baricenter,int n, int p);
	//void order_decreasing(float *values, int *indexos, int size);
    
	float **  covariance_matrix(float **x,int n, int p);
    int  compute_pca_from_covariance(float **Mat_cov,float **Pcs, float *sVar, int p);
    
    
    
    

}



//
//! Parser
//

//! structure for parameters and options which are
//! optional in the code or they already have a default value
typedef struct optstruct
{
    char *gp;           //! string of two letters  "a:"  as necessary for using getopt afterwards
                        //! the ":" indicates that the activation of the option requires a value for the parameter
                        //! and "a" that this option is activated by "-a " in the command
    
    int flag;           //! flag indicating that the option has been activated
    
    char *defvalue;     //! default value for this parameter if not modified by console
    
    char *value;        //! value of the associated parameter to current option
    
    char *comment;      //! comment that appears by console
    	
} OptStruct;



//! structure for necessary parameters of the method
typedef struct parstruct
{
    char * name;
    char * value;       //! value of the parameter
    char * comment;     //! comment that appears by console
    
} ParStruct;



int parsecmdline(char *pname,
                 char *function,
                 int argc, char **argv,
                 std::vector <OptStruct*> & opt,
                 std::vector <ParStruct*> & par);



#endif
