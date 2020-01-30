#ifndef DUAL_TVL1_OPTIC_FLOW_H
#define DUAL_TVL1_OPTIC_FLOW_H

#include "../library/libBasic.h"

namespace libUSTGFLOW {

#define MAX_ITERATIONS 300
#define PRESMOOTHING_SIGMA 0.8
#define GRAD_IS_ZERO 1E-10


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


    void *xmalloc(size_t size);

    void image_normalization(
                             const float *I0,  // input image0
                             const float *I1,  // input image1
                             float *I0n,       // normalized output image0
                             float *I1n,       // normalized output image1
                             int size          // size of the image
    );


    void check_flow_reciprocity(float *uflow, float *vflow, float *uflow2, float *vflow2, float *omask,float fThreshold, int width, int height);
    

    void image_normalization(const float *I0, const float *I1, float *I0n, float *I1n, int size, int flag255);



    void gaussian(
                  float *I,             // input/output image
                  const int xdim,       // image width
                  const int ydim,       // image height
                  const double sigma    // Gaussian sigma
    );


    void zoom_size(
                   int nx,      // width of the orignal image
                   int ny,      // height of the orignal image
                   int *nxx,    // width of the zoomed image
                   int *nyy,    // height of the zoomed image
                   float factor // zoom factor between 0 and 1
    );

    void zoom_out(
                  const float *I,    // input image
                  float *Iout,       // output image
                  const int nx,      // image width
                  const int ny,      // image height
                  const float factor // zoom factor between 0 and 1
    );

void zoom_out_mask_integer_factor(
                                      const float *I,          // input image
                                      float *Iout,             // output image
                                      const int nx,            // image width
                                      const int ny,            // image height
                                      const float factor       // zoom factor between 0 and 1
                                      );

    //! nx and ny is the size of the entire one
    void zoom_out_mask_integer_factor_v2(
                                         const float *I,          // input image
                                         float *Iout,             // output image
                                         const int nx,            // image width
                                         const int ny,            // image height
                                         const float factor       // zoom factor between 0 and 1
    );
    
    void zoom_in(
                 const float *I, // input image
                 float *Iout,    // output image
                 int nx,         // width of the original image
                 int ny,         // height of the original image
                 int nxx,        // width of the zoomed image
                 int nyy         // height of the zoomed image
    );

    void centered_gradient(
                           const float *input,  //input image
                           float *dx,           //computed x derivative
                           float *dy,           //computed y derivative
                           const int nx,        //image width
                           const int ny         //image height
    );
    
    
    void forward_gradient(
                            const float *f, //input image
                            float *fx,      //computed x derivative
                            float *fy,      //computed y derivative
                            const int nx,   //image width
                            const int ny    //image height
    );
    
    void divergence(
                    const float *v1, // x component of the vector field
                    const float *v2, // y component of the vector field
                    float *div,      // output divergence
                    const int nx,    // image width
                    const int ny     // image height
    );


    float bicubic_interpolation_at(
                                   const float *input, //image to be interpolated
                                   const float  uu,    //x component of the vector field
                                   const float  vv,    //y component of the vector field
                                   const int    nx,    //image width
                                   const int    ny,    //image height
                                   bool         border_out //if true, return zero outside the region
    );


    void bicubic_interpolation_warp(
                                    const float *input,     // image to be warped
                                    const float *u,         // x component of the vector field
                                    const float *v,         // y component of the vector field
                                    float       *output,    // image warped with bicubic interpolation
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    bool         border_out // if true, put zeros outside the region
    );




    /**
     *
     * Function to compute the optical flow using multiple scales
     *
     **/
    void Dual_TVL1_optic_flow_multiscale(
                                         float *I0,           // source image
                                         float *I1,           // target image
                                         float *u1,           // x component of the optical flow
                                         float *u2,           // y component of the optical flow
                                         const int   nxx,     // image width
                                         const int   nyy,     // image height
                                         const float tau,     // time step
                                         const float lambda,  // weight parameter for the data term
                                         const float theta,   // weight parameter for (u - v)²
                                         const int   nscales, // number of scales
                                         const float zfactor, // factor for building the image piramid
                                         const int   warps,   // number of warpings per scale
                                         const float epsilon, // tolerance for numerical convergence
                                         const bool  verbose,  // enable/disable the verbose mode
                                         int iflagMedian,
                                         int iflagLastScale);


    /**
     *
     *  Procedure to handle the pyramidal approach.
     *  This procedure relies on the previous functions to calculate
     *  large optical flow fields using a pyramidal scheme.
     *
     */
    void horn_schunck_pyramidal(
                                const float *I1,              // source image
                                const float *I2,              // target image
                                float       *u,               // x component of optical flow
                                float       *v,               // y component of optical flow
                                const int    nx,              // image width
                                const int    ny,              // image height
                                const float  alpha,           // smoothing weight
                                const int    nscales,         // number of scales
                                const float  zfactor,         // zoom factor
                                const int    warps,           // number of warpings per scale
                                const float  TOL,             // stopping criterion threshold
                                const int    maxiter,         // maximum number of iterations
                                const bool   verbose          // switch on messages
                                );




    /**
     * Implementation of the Zach, Pock and Bischof dual TV-L1 optic flow method
     *
     * see reference:
     *  [1] C. Zach, T. Pock and H. Bischof, "A Duality Based Approach for Realtime
     *      TV-L1 Optical Flow", In Proceedings of Pattern Recognition (DAGM),
     *      Heidelberg, Germany, pp. 214-223, 2007
     *
     *
     * Details on the total variation minimization scheme can be found in:
     *  [2] A. Chambolle, "An Algorithm for Total Variation Minimization and
     *      Applications", Journal of Mathematical Imaging and Vision, 20: 89-97, 2004
     **/


    /**
     *
     * Function to compute the optical flow in one scale
     *
     **/
    void Dual_TVL1_optic_flow_gabrielle(
                                        float *I0,           // source image
                                        float *I1,           // target image
                                        float *u1,           // x component of the optical flow
                                        float *u2,           // y component of the optical flow
                                        const int   nx,      // image width
                                        const int   ny,      // image height
                                        const float tau,     // time step
                                        const float lambda,  // weight parameter for the data term
                                        const float theta,   // weight parameter for (u - v)²
                                        const int   warps,   // number of warpings per scale
                                        const float epsilon, // tolerance for numerical convergence
                                        const bool  verbose,  // enable/disable the verbose mode
                                        float       *init_u,         // x component of the input flow
                                        float       *init_v,         // y component of the input flow
                                        float       *weights,        // mask where the input flow is known
                                        float       *weightsofc,      // weights for the OFC
                                        float       robustparam,      // extra scalar parameter only used by robl1 for controlling the shape of the robust L1
                                        float       *weightstv
                                        );



    /**
     *
     * Function to compute the optical flow using multiple scales
     *
     **/
    void Dual_TVL1_optic_flow_multiscale_gabrielle(
                                                   float *I0,           // source image
                                                   float *I1,           // target image
                                                   float *u1,           // x component of the optical flow
                                                   float *u2,           // y component of the optical flow
                                                   const int   nxx,     // image width
                                                   const int   nyy,     // image height
                                                   const float tau,     // time step
                                                   const float lambda,  // weight parameter for the data term
                                                   const float theta,   // weight parameter for (u - v)²
                                                   const int   nscales, // number of scales
                                                   const float zfactor, // factor for building the image piramid
                                                   const int   warps,   // number of warpings per scale
                                                   const float epsilon, // tolerance for numerical convergence
                                                   const bool  verbose,  // enable/disable the verbose mode
                                                   float       *init_u1,          // x component of the input flow
                                                   float       *init_u2,          // y component of the input flow
                                                   float       *msk_init_uv,     // mask where the input flow is known
                                                   float       *msk_OFC,         // weights for the opt.flow.constr.
                                                   float       *weightsTV,
                                                   float       robustparam      // extra scalar parameter only used by robl1 for controlling the shape of the robust L1
                                                   );


    void Dual_TVL1_optic_flow_multiscale_julia(
                                                   float *I0,           // source image
                                                   float *I1,           // target image
                                                   float *u1,           // x component of the optical flow
                                                   float *u2,           // y component of the optical flow
                                                   const int   nxx,     // image width
                                                   const int   nyy,     // image height
                                                   const float tau,     // time step
                                                   const float lambda,  // weight parameter for the data term
                                                   const float theta,   // weight parameter for (u - v)²
                                                   const int   nscales, // number of scales
                                                   const float zfactor, // factor for building the image piramid
                                                   const int   warps,   // number of warpings per scale
                                                   const float epsilon, // tolerance for numerical convergence
                                                   const bool  verbose,  // enable/disable the verbose mode
                                                   float       *init_u1,          // x component of the input flow
                                                   float       *init_u2,          // y component of the input flow
                                                   float       *msk_init_uv,     // mask where the input flow is known
                                                   float       *msk_OFC,         // weights for the opt.flow.constr.
                                                   float       *weightsTV,
                                                   float       robustparam,      // extra scalar parameter only used by robl1 for controlling the shape of the robust L1
                                                   const float beta    // weight parameter for "u" term (new term)
                                                   );


    void Dual_TVL1_optic_flow_julia(
                                        float *I0,           // source image
                                        float *I1,           // target image
                                        float *u1,           // x component of the optical flow
                                        float *u2,           // y component of the optical flow
                                        const int   nx,      // image width
                                        const int   ny,      // image height
                                        const float tau,     // time step
                                        const float lambda,  // weight parameter for the data term
                                        const float theta,   // weight parameter for (u - v)²
                                        const int   warps,   // number of warpings per scale
                                        const float epsilon, // tolerance for numerical convergence
                                        const bool  verbose,  // enable/disable the verbose mode
                                        float       *init_u,         // x component of the input flow
                                        float       *init_v,         // y component of the input flow
                                        float       *weights,        // mask where the input flow is known
                                        float       *weightsofc,      // weights for the OFC
                                        float       robustparam,      // extra scalar parameter only used by robl1 for controlling the shape of the robust L1
                                        float       *weightstv,
                                        const float beta // weight parameter for u (the new term)
                                        );
}

#endif//DUAL_TVL1_OPTIC_FLOW_H
