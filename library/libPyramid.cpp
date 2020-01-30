//
//  libFusion.h
//  Sources
//
//  Created by antoni buades on 29/03/17.
//  Copyright (c) 2017 antoni buades. All rights reserved.
//

#include "libPyramid.h"


namespace libUSTG
{
    
    
    void Pyramid_filter1d(float *filter)
    {
        filter[0]=0.0625; filter[1]=0.25; filter[2]=0.375; filter[3]=0.25; filter[4]=0.0625;
    }
    
    
    void Pyramid_filter2d(float **filter)
    {
        float *filter1d=new float[5];
        Pyramid_filter1d(filter1d);
        
        for(int i=0;i< 5; i++)
            for(int j=0; j< 5; j++)
                filter[i][j]=filter1d[i]*filter1d[j];
        
        delete[] filter1d;
    }
    
    
    void Pyramid_Reduce(cflimage &out, cflimage &input)
    {
        int nx = input.w();
        int ny = input.h();
        
        int nx2 = out.w();
        int ny2 = out.h();
        
        float **filter=new float*[5];
        for (int i=0; i < 5; i++) filter[i] = new float[5];
        
        Pyramid_filter2d(filter);
        
        for (int n=0; n < input.c(); n++)
        {
            
            float *ptrI = input.v(n);
            float *ptrO = out.v(n);
            
            
            for(int i=0;i<nx2; i++)
                for(int j=0; j< ny2; j++)
                {
                    
                    ptrO[i+j*nx2]=0.0f;
                    for(int x=-2; x<=2; x++)
                        for(int y=-2; y<=2; y++)
                        {
                            int ii=2*i+x;
                            int jj=2*j+y;
                            if(2*i+x<0) ii=abs(2*i+x);
                            if(2*j+y <0) jj=abs(2*j+y);
                            if(2*i+x>= nx) ii=nx-1-abs(2*i+x-nx+1);
                            if(2*j+y>=ny) jj=ny-1-abs(2*j+y-ny+1);
                            
                            ptrO[i+j*nx2]+=filter[x+2][y+2]*ptrI[ii+jj*nx];
                        }
                    
                }
            
        }
        
        for (int i=0; i < 5; i++) delete[] filter[i];
        delete[] filter;
        
    }
    
    
    void Pyramid_Expand(cflimage  &out, cflimage &input)
    {
        int nx = input.w();
        int ny = input.h();
        
        int nx2 = out.w();
        int ny2 = out.h();
        
        float **filter=new float*[5];
        for (int i=0; i < 5; i++) filter[i] = new float[5];
        
        Pyramid_filter2d(filter);
        
        
        for (int n=0; n < input.c(); n++)
        {
            
            float *ptrI = input.v(n);
            float *ptrO = out.v(n);
            
            for(int i=0;i<nx2; i++)
                for(int j=0; j< ny2; j++)
                {
                    ptrO[i+j*nx2]=0.;
                    for(int x=-2; x<=2; x++)
                        for(int y=-2; y<=2; y++)
                        {
                            
                            int ii=(i+x)/2;
                            int jj=(j+y)/2;
                            if((i+x)/2<0) ii=abs((i+x)/2);
                            if((j+y)/2 < 0)  jj=abs((j+y)/2 );
                            if((i+x)/2>= nx) ii=nx-1-abs((i+x)/2-nx+1);
                            if((j+y)/2 >= ny) jj=ny-1-abs((j+y)/2-ny+1);
                            ptrO[i+j*nx2]+=filter[x+2][y+2]*ptrI[ii+jj*nx];
                            
                        }
                }
            
        }
        
        for (int i=0; i < 5; i++) delete[] filter[i];
        delete[] filter;
        
    }
    
    
    
    
    void BuildGaussianPyramid(cflimage &input, cflimage *GPyr, int nLevels)
    {
        
        GPyr[0] = input;
        for(int l=1; l< nLevels; l++)
        {
            int w = GPyr[l-1].w();
            int h = GPyr[l-1].h();
            
            GPyr[l].create((w-1)/2+1, (h-1)/2+1, input.c());
            Pyramid_Reduce(GPyr[l], GPyr[l-1]);
        }
        
    }
    
    
    
    void BuildLaplacianPyramid(cflimage &input, cflimage *LPyr, int nLevels)
    {
        
        BuildGaussianPyramid(input, LPyr,  nLevels);
        
        for(int l=0; l < nLevels-1; l++)
        {
            cflimage tmpImage(LPyr[l].w(), LPyr[l].h(), input.c());
            Pyramid_Expand(tmpImage, LPyr[l+1]);
            
            for(int n=0;n < LPyr[l].whc(); n++)  LPyr[l][n]=LPyr[l][n]-tmpImage[n];
        }
        
    }
    
    
    
    void ReconstructPyramidKeepPyr(cflimage &output, cflimage *LPyr, int nLevels)
    {
        cflimage tmpImage = LPyr[nLevels-1];
        for(int l=nLevels-2; l>=0; l--)
        {
            cflimage tmpImage2( LPyr[l].w(), LPyr[l].h(), LPyr[l].c() );
            Pyramid_Expand(tmpImage2, tmpImage);
            for(int n=0; n < tmpImage2.whc(); n++)
                tmpImage2[n]= LPyr[l][n]+tmpImage2[n];
            
            tmpImage = tmpImage2;
            
        }
        
        output=tmpImage;
    }
    
    
    void ReconstructPyramidRewrPyr(cflimage &output, cflimage *LPyr, int nLevels)
    {
        for(int l=nLevels-2; l>=0; l--)
        {
            cflimage tmpImage2( LPyr[l].w(), LPyr[l].h(), LPyr[l].c() );
            Pyramid_Expand(tmpImage2, LPyr[l+1]);
            for(int n=0; n < tmpImage2.whc(); n++)
                LPyr[l][n] = LPyr[l][n] + tmpImage2[n];
            
        }
        
        output=LPyr[0];
        
    }
    
    
    
    
    
}


