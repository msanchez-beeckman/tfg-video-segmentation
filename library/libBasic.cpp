#include "libBasic.h"
#include <algorithm>


namespace libUSTG
{
    
    
    
    void src_exit(const char* message)
    {
        
        std::string wmessage("exit :: ");
        std::string tmp(message);
        wmessage += message;
        std::cout << wmessage << std:: endl;
        exit(EXIT_FAILURE);
    }
  
    
    void src_warning(const char* message)
    {
        
        std::string wmessage("warning :: ");
        std::string tmp(message);
        wmessage += message;
        std::cout << wmessage << std:: endl;
    }
    
    std::string int2string(int number)
    {
        std::stringstream s;
        s << number;
        return s.str();
    }

    
    
    
    //
    //! Value operations
    //
    void ipClear(int *fpI,int fValue, int iLength)
    {
        assert(iLength > 0);
        for (int ii=0; ii < iLength; ii++) fpI[ii] = fValue;
    }
    
    void fpClear(float *fpI,float fValue, int iLength)
    {
        assert(iLength > 0);
        for (int ii=0; ii < iLength; ii++) fpI[ii] = fValue;
    }
    
    
    void fpCopy(float *fpI,float *fpO, int iLength)
    {
        assert(iLength > 0);
        if (fpI != fpO)  memcpy((void *) fpO, (const void *) fpI, iLength * sizeof(float));
    }

    
    void dpClear(double *fpI,double fValue, int iLength)
    {
        assert(iLength > 0);
        for (int ii=0; ii < iLength; ii++) fpI[ii] = fValue;
    }
    
    
    void dpCopy(double *fpI,double *fpO, int iLength)
    {
        assert(iLength > 0);
        if (fpI != fpO)  memcpy((void *) fpO, (const void *) fpI, iLength * sizeof(double));
    }

    
    
    
    
    float fpMax(float *u,int *pos, int size)
    {
        assert(size > 0);
        float max=u[0];
        if (pos) *pos=0;
        for(int i=1; i<size; i++)  if(u[i]>max){ max=u[i]; if (pos) *pos=i; }
        return max;
    }
    
    
    float fpMin(float *u,int *pos,int size)
    {
        assert(size > 0);
        float min=u[0];
        if (pos) *pos=0;
        for(int i=1;i<size;i++)	if(u[i]<min){ min=u[i]; if (pos) *pos=i; }
        return min;
    }
    
    
    
    
    
    float  fpMean(float *u,int size)
    {
        assert(size > 0);
        float *ptru=&u[0];
        float mean=0.0;
        for(int i=0; i<size; i++,ptru++)  mean +=  *ptru;
        mean/=(float) size;
        return mean;
    }
    
    
    float fpMean(float *u, float *m, int size)
    {
        assert(size > 0);
        float *ptru=&u[0];
        float *ptrm=&m[0];
        float mean=0.0;
        float npix=0.0f;
        for(int i=0; i<size; i++,ptru++,ptrm++) if (*ptrm>0.0f) { mean +=  *ptru; npix++; }
        mean/=(float) npix;
        return mean;
        
    }

    
    float  fpMedian(float *u,int size)
    {
        assert(size > 0);
        if (size == 1) return u[0];
        
        fpQuickSort(u, size);
        
        int dsize = (size - 1) / 2;
        if (size % 2 == 1) return u[dsize];
        else return 0.5 * (u[dsize]+u[dsize+1]);
    }
    
    
    float fpWeightedMedian(float *u,float *w, int size)
    {
    
        assert(size > 0);
        if (size == 1) return u[0];
        
        fpQuickSort(u, w, size);
        
        float sumW = w[0];
        for(int i=1; i < size; i++) sumW += w[i];
        
        float sumA = w[0];
        int i=1;
        for(; i < size && sumA < sumW/2.0f; i++)
        {
            sumA += w[i];
        }
        
        if (i==1)
        {
            return u[0];
        }
        else
        {
            float wAux = w[i-1];
            float A = sumA - w[i-1];
            float P = (sumW/2.0f - A)/wAux;
            float result = P * u[i-1] + (1.0f- P) * u[i-2];
            return result;
        }
        
    }

    
    double dpWeightedMedian(double *u,double *w, int size)
    {
        
        assert(size > 0);
        if (size == 1) return u[0];
        
        dpQuickSort(u, w, size);
        
        double sumW = w[0];
        for(int i=1; i < size; i++) sumW += w[i];
        
        double sumA = w[0];
        int i=1;
        for(; i < size && sumA < sumW/2; i++)
        {
            sumA += w[i];
        }
        
        if (i==1)
        {
            return u[0];
        }
        else
        {
            double wAux = w[i-1];
            double A = sumA - w[i-1];
            double P = (sumW/2.0f - A)/wAux;
            double result = P * u[i-1] + (1.0- P) * u[i-2];
            return result;
        }
        
    }

    
    
    
    
    
    
    float  fpVar(float *u,int size)
    {
        
        assert(size > 0);
        float *ptru=&u[0];
        float mean=0.0;
        float mean2 = 0.0;
        for(int i=0;i<size;i++,ptru++) { mean +=  *ptru; mean2 +=  *ptru *  (*ptru);}
        
        mean/=(float) size; mean2/=(float) size;
        float var = mean2- mean*mean;
        
        var = fabsf(var);
        return var;
    }
    
    
    
    float fpVar(float *u,float *m, int size)
    {
        assert(size > 0);
        float *ptru=&u[0];
        float *ptrm=&m[0];
        float mean=0.0;
        float mean2 = 0.0;
        float npix=0.0f;
        for(int i=0;i<size;i++,ptru++,ptrm++) if (*ptrm>0.0f){ mean +=  *ptru; mean2 +=  *ptru *  (*ptru); npix++;}
        
        mean/=(float) npix; mean2/=(float) npix;
        float var = mean2- mean*mean;
        
        var = fabsf(var);
        return var;
    }
    
    
    
    
    void fpCombine(float *u,float a,float *v,float b, float *w,  int size)
    {
        for(int i=0;i<size ;i++)   w[i]= (a *  u[i] + b* v[i]);
        
    }
    
    
    int val_coord(int i, int w) {
        int i_new = MIN(MAX(i, 0), w - 1);
        return i_new;
    }
    
    void fiImageDrawPoint(float *igray, int pi, int pj, int radius, float value, int width, int height)
    {
        
        //If point from previous image is in the current image, the point has valid coordinates at end
        if (pi >= 0 && pj >= 0 && pi < width && pj < height) {
            
            
            int i_min = val_coord(pi - radius, width);
            int i_max = val_coord(pi + radius, width);
            int j_min = val_coord(pj - radius, height);
            int j_max = val_coord(pj + radius, height);
            
            for (int ii = i_min; ii <= i_max; ii++) {
                for (int jj = j_min; jj <= j_max; jj++) {
                    igray[jj*width + ii] = value;
                }
            }
        }
    }
    
    
    
    
    void fiImageDrawCircle(float *igray, int pi,int pj, float radius, float value, int width, int height)
    {
        
        int mark = (int) rint(radius);
        float radius2 = radius * radius;
        
        for(int s = -mark ; s <= mark ;s++)
        for(int r = -mark ; r <= mark ;r++)
        if (pj+s>=0 && pi+r>= 0 && pj+s < height && pi+r < width && (float) (r*r + s*s) < radius2)
        {
            
            int l = (pj+s)*width+pi+r;
            igray[l] = value;
        }
        
    }

    void fiImageDrawCircumference(float *igray, int pi,int pj, float radius, float value, int width, int height)
    {
        
        float pi2 = 2.0f * PI;
        
        float px = (float) pi;
        float py = (float) pj;
        
        for(float alpha=0; alpha <= pi2; alpha+=0.001)
        {
            int ipx = rintf( px + radius * sinf(alpha));
            int ipy = rintf( py + radius * cosf(alpha));
            
            if (ipx>=0 && ipy>= 0 && ipx < width && ipy < height)
            {
                
                int l = (ipy)*width+ipx;
                igray[l] = value;
            }
            
        }

    }
    
    
    void fiImageDrawEllipse(float *igray, int pi, int pj, float a, float b, float alpha, float value, int width, int height)
    {
        
        
        float pi2 = 2.0f * PI;
        
        
        float px = (float) pi;
        float py = (float) pj;
        
        
        for(float theta = 0; theta <= pi2; theta += 0.001)
        {
            float s = sin(alpha);
            float c = cos(alpha);
            
            libUSTG::laMatrix Q(0.0, 2, 2);
            Q[0][0] = c; Q[0][1] = -s;
            Q[1][0] = s; Q[1][1] = c;
            
            libUSTG::laVector coord_at_origin(0.0, 2);
            coord_at_origin[0] = a*cos(theta);
            coord_at_origin[1] = b*sin(theta);
            
            libUSTG::laVector rotated_point = Q*coord_at_origin;
            
            //Point in cartesian coordinates
            int ipx = int(rintf( px + rotated_point[0]));
            int ipy = int(rintf( py + rotated_point[1]));
            
            
            //If point in immage, draw it
            if (ipx >= 0 && ipy >= 0 && ipx < width && ipy < height) {
                
                int l = (ipy)*width+ipx;
                igray[l] = value;
            }
        }
    }
    
    
    void fiImageDrawLine(float *igray, int a0, int b0, int a1, int b1, float value, int width, int height)
    {
        
        int bdx,bdy;
        float sx,sy,dx,dy,x,y,z;
        
        bdx = width;
        bdy = height;
        
        if (a0 < 0) a0=0;
        else if (a0>=bdx) a0=bdx-1;
        
        if (a1<0)  a1=0;
        else  if (a1>=bdx)   a1=bdx-1;
        
        if (b0<0) b0=0;
        else if (b0>=bdy) b0=bdy-1;
        
        if (b1<0) 	b1=0;
        else if (b1>=bdy) b1=bdy-1;
        
        float fEps = 0.1;
        if (a0<a1) { sx = fEps; dx = (float) a1-a0; } else { sx = -fEps; dx = (float) a0-a1; }
        if (b0<b1) { sy = fEps; dy = (float) b1-b0; } else { sy = -fEps; dy = (float) b0-b1; }
        x=0; y=0;
        
        
        if (dx>=dy)
        {
            z = (-dx) / 2.0;
            while (abs(x) <= dx)
            {
                
                int py = rintf(y+b0);
                int px = rintf(x+a0);
                
                if (px < 0) px = 0;
                if (px >= width) px = width-1;
                
                if (py < 0) py = 0;
                if (py >= height) py = height-1;
                
                int l =  py*bdx+px;
                igray[l] = value;
                
                x+=sx;
                z+=dy;
                if (z>0.0) { y+=sy; z-=dx; }
                
            }
            
        }
        else
        {
            z = (-dy) / 2.0;
            while (abs(y) <= dy) {
                
                int py = rintf(y+b0);
                int px = rintf(x+a0);
                
                if (px < 0) px = 0;
                if (px >= width) px = width-1;
                
                if (py < 0) py = 0;
                if (py >= height) py = height-1;
                
                int l =  py*bdx+px;
                igray[l] = value;
                
                y+=sy;
                z+=dx;
                if (z>0) { x+=sx; z-=dy; }
            }
        }
        
    }
    
    
    
    
    void fpBinarize(float *u, float *v, float value, int inverse, int size)
    {
        for(int i=0;i<size;i++){
            
            if (u[i] >= value && !inverse) 	v[i]= 1.0;
            else if (u[i] < value && inverse)  v[i]= 1.0;
            else v[i]= 0.0;
        }
    }
    
    
    
    
    float fpDistLp(float *u, float *v, int p, int size)
    {
        
        float fDist = 0.0f;
        
        for (int ii=0; ii < size; ii++)
        {
            
            float dif = fabsf(u[ii] - v[ii]);
            
            
            if (p == 0) fDist = MAX(fDist, dif);
            else if (p == 1)  fDist += dif;
            else if (p == 2)
            fDist += dif*dif;
            else
            {
                fDist += powf(dif, (float) p);
                
            }
            
        }
        
        fDist /= (float) size;
        
        if (p>0)
        fDist = powf(fDist, 1.0 / (float) p);
        
        
        return fDist;
        
    }
    
    
    
    float fpDistLp(float *u, float *v, float *m, int p, int size)
    {
        
        float fDist = 0.0f;
        int iCount = 0;
        
        for (int ii=0; ii < size; ii++)
        if (m[ii] > 0.0f)
        {
            
            float dif = fabsf(u[ii] - v[ii]);
            
            
            if (p == 0) fDist = MAX(fDist, dif);
            else if (p == 1)  fDist += dif;
            else if (p == 2)
            fDist += dif*dif;
            else
            {
                fDist += powf(dif, (float) p);
                
            }
            
            iCount++;
        }
        
        fDist /= (float) iCount;
        
        if (p>0)
        fDist = powf(fDist, 1.0 / (float) p);
        
        
        return fDist;
        
    }
    
    
    
   
    
    
    
    //
    //! Float pointer ordering
    //
    
    int order_float_increasing(const void *a, const void *b)
    {
        if ( *(float*)a  > *(float*)b ) return 1;
        else if ( *(float*)a  < *(float*)b ) return -1;
        
        return 0;
    }
    
    
    
    
    int order_float_decreasing(const void *a, const void *b)
    {
        if ( *(float*)a  > *(float*)b ) return -1;
        else if ( *(float*)a  < *(float*)b ) return 1;
        
        return 0;
    }
    
    
    
    
    
    
    void fpQuickSort(float *fpI, int iLength, int inverse)
    {
        
        if (inverse)
        qsort(fpI, iLength, sizeof(float), order_float_decreasing);
        else
        qsort(fpI, iLength, sizeof(float), order_float_increasing);
        
    }
    
    
    
    

    
    
    int order_stf_qsort_increasing(const void *pVoid1, const void *pVoid2)
    {
        struct stf_qsort *p1, *p2;
        
        p1=(struct stf_qsort *) pVoid1;
        p2=(struct stf_qsort *) pVoid2;
        
        if (p1->value < p2->value) return -1;
        if (p1->value > p2->value) return  1;
        
        return 0;
    }
    
    
    
    
    
    int order_stf_qsort_decreasing(const void *pVoid1, const void *pVoid2)
    {
        struct stf_qsort *p1, *p2;
        
        p1=(struct stf_qsort *) pVoid1;
        p2=(struct stf_qsort *) pVoid2;
        
        if (p1->value < p2->value) return 1;
        if (p1->value > p2->value) return  -1;
        
        return 0;
        
    }
    
    

    
    
    int order_std_qsort_increasing(const void *pVoid1, const void *pVoid2)
    {
        struct std_qsort *p1, *p2;
        
        p1=(struct std_qsort *) pVoid1;
        p2=(struct std_qsort *) pVoid2;
        
        if (p1->value < p2->value) return -1;
        if (p1->value > p2->value) return  1;
        
        return 0;
    }
    
    
    
    
    
    int order_std_qsort_decreasing(const void *pVoid1, const void *pVoid2)
    {
        struct std_qsort *p1, *p2;
        
        p1=(struct std_qsort *) pVoid1;
        p2=(struct std_qsort *) pVoid2;
        
        if (p1->value < p2->value) return 1;
        if (p1->value > p2->value) return  -1;
        
        return 0;
        
    }
    
    
    
    
    
    
    
    void fpQuickSort(float *fpI, float *fpS, int iLength, int inverse)
    {
        
        struct stf_qsort *vector = new stf_qsort[iLength];
        
        for (int i=0; i < iLength; i++)
        {
            vector[i].value = fpI[i];
            vector[i].index = fpS[i];
            
        }
        
        
        if (inverse)
        qsort(vector, iLength, sizeof(stf_qsort), order_stf_qsort_decreasing);
        else
        qsort(vector, iLength, sizeof(stf_qsort), order_stf_qsort_increasing);
        
        
        for (int i=0; i < iLength; i++)
        {
            fpI[i] = vector[i].value;
            fpS[i] = vector[i].index;
            
        }
        
        
        delete[] vector;
    }
 
    
    
    void dpQuickSort(double *fpI, int iLength, int inverse)
    {
        
        if (inverse)
        qsort(fpI, iLength, sizeof(double), order_float_decreasing);
        else
        qsort(fpI, iLength, sizeof(double), order_float_increasing);
        
    }
    
    
    
    void dpQuickSort(double *fpI, double *fpS, int iLength, int inverse)
    {
        
        struct std_qsort *vector = new std_qsort[iLength];
        
        for (int i=0; i < iLength; i++)
        {
            vector[i].value = fpI[i];
            vector[i].index = fpS[i];
            
        }
        
        
        if (inverse)
        qsort(vector, iLength, sizeof(std_qsort), order_std_qsort_decreasing);
        else
        qsort(vector, iLength, sizeof(std_qsort), order_std_qsort_increasing);
        
        
        for (int i=0; i < iLength; i++)
        {
            fpI[i] = vector[i].value;
            fpS[i] = vector[i].index;
            
        }
        
        
        delete[] vector;
    }
    
    
    
    
    
    
    
    //
    //! Image conversion
    //

    void fiRgb2Yuv(float *r,float *g,float *b,float *y,float *u,float *v,int width,int height)
    {
        int size=height*width;
        
        for(int i=0;i<size;i++){
            y[i] = ( COEFF_YR *  r[i] + COEFF_YG * g[i] + COEFF_YB * b[i]);
            u[i] =  ( r[i] - y[i]);
            v[i] =  ( b[i] - y[i]);
        }
        
    }
    
    
    
    
    
    void fiYuv2Rgb(float *r,float *g,float *b,float *y,float *u,float *v, int width,int height)
    {
        
        
        int iwh=height*width;
        
        for(int i=0;i<iwh;i++){
            
            g[i] =  ( y[i] - COEFF_YR * (u[i] + y[i]) - COEFF_YB * (v[i] +  y[i]) ) / COEFF_YG;
            r[i] =  ( u[i] + y[i]);
            b[i] =  ( v[i] + y[i]);
            
        }
        
    }
    
    
    void fiRgb2YuvO(float *r,float *g,float *b,float *y,float *u,float *v,int width,int height)
    {
        int size=height*width;
        
        for(int i=0;i<size;i++)
        {
            y[i] =    0.577350 * r[i] + 0.577350 * g[i] + 0.577350  * b[i];
            u[i] =    0.707106 * r[i]					- 0.707106  * b[i];
            v[i] =    0.408248 * r[i] - 0.816496 * g[i]	+ 0.408248  * b[i];
        }
        
    }
    
    
    
    
    void fiYuvO2Rgb(float *r,float *g,float *b,float *y,float *u,float *v, int width,int height)
    {
        
        
        int iwh=height*width;
        
        for(int i=0;i<iwh;i++)
        {
            
            r[i] =  0.577350 * y[i]	+ 0.707106 * u[i]	+  0.408248 * v[i];
            g[i] =  0.577350 * y[i]						-  0.816496 * v[i];
            b[i] =  0.577350 * y[i] - 0.707106 * u[i]	+  0.408248 * v[i];
            
        }
        
    }
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Patch Statistics
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    void fiPatchStatistics(float *fpIn, float *fpMinV, float *fpMaxV, float *fpMeanV, float *fpVarV, float *fpMedianV, float fRadius, int iWidth, int iHeight)
    {
        
        //! Parameters
        int iRadius = (int)(fRadius+1.0);
        int iNeigSize = (2*iRadius+1)*(2*iRadius+1);
        float fRadiusSqr = fRadius * fRadius;
        
        
        float * vector = new float[iNeigSize];
        
        
        
        //! For each pixel
        for(int x=0;x < iWidth;x++)
            for(int y=0;y< iHeight;y++)
            {
                
                int iCount=0;
                float fMin = fLarge;
                float fMax = -fLarge;
                
                
                for(int i=-iRadius;i<=iRadius;i++)
                    for(int j=-iRadius;j<=iRadius;j++)
                        if ((float) (i*i + j*j) <= fRadiusSqr)
                        {
                            
                            int x0=x+i;
                            int y0=y+j;
                            
                            if (x0 >= 0 && y0 >= 0 && x0 < iWidth && y0 < iHeight)
                            {
                                float fValue= fpIn[y0*iWidth+x0];
                                
                                if (fValue > fMax) fMax = fValue;
                                if (fValue < fMin) fMin = fValue;
                                
                                vector[iCount] = fValue;
                                iCount++;
                                
                            }
                            
                        }
                
                int l = y*iWidth+x;
                if (fpMinV) fpMinV[l] = fMin;
                if (fpMaxV) fpMaxV[l] = fMax;
                
                if (fpMeanV) fpMeanV[l] = fpMean(vector, iCount);
                if (fpVarV)  fpVarV[l] = fpVar(vector, iCount);
                
                
                if (fpMedianV)
                {
                    fpQuickSort(vector, iCount, 0);
                    fpMedianV[l] = vector[iCount / 2];
                }
                
                
                
            }
        
        delete[] vector;
    }
    
    
    void fiPatchStatistics(float *fpIn, float *fpMinV, float *fpMaxV, float *fpMeanV, float *fpVarV, float *fpMedianV, int rx, int ry, int iWidth, int iHeight)
    {
        
        //! Parameters
        int iNeigSize = (2*rx+1)*(2*ry+1);
        float * vector = new float[iNeigSize];
        
        
        
        //! For each pixel
        for(int x=0;x < iWidth;x++)
            for(int y=0;y< iHeight;y++)
            {
                
                int iCount=0;
                float fMin = fLarge;
                float fMax = -fLarge;
                
                
                for(int i=-rx;i<=rx;i++)
                    for(int j=-ry;j<=ry;j++)
                    {
                        
                        int x0=x+i;
                        int y0=y+j;
                        
                        if (x0 >= 0 && y0 >= 0 && x0 < iWidth && y0 < iHeight)
                        {
                            float fValue= fpIn[y0*iWidth+x0];
                            
                            if (fValue > fMax) fMax = fValue;
                            if (fValue < fMin) fMin = fValue;
                            
                            vector[iCount] = fValue;
                            iCount++;
                            
                        }
                        
                    }
                
                int l = y*iWidth+x;
                if (fpMinV) fpMinV[l] = fMin;
                if (fpMaxV) fpMaxV[l] = fMax;
                
                if (fpMeanV) fpMeanV[l] = fpMean(vector, iCount);
                if (fpVarV)  fpVarV[l] = fpVar(vector, iCount);
                
                
                if (fpMedianV)
                {
                    fpQuickSort(vector, iCount, 0);
                    fpMedianV[l] = vector[iCount / 2];
                }
                
                
                
            }
        
        delete[] vector;
    }
    
    
    
    
    
    void fiPatchMin(float *fpIn, float *fpMinV, float fRadius, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, fpMinV, NULL,  NULL,  NULL, NULL,  fRadius,  iWidth, iHeight);
        
    }
    
    
    void fiPatchMax(float *fpIn, float *fpMaxV, float fRadius, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, fpMaxV,  NULL,  NULL, NULL,  fRadius,  iWidth, iHeight);
        
    }
    
    
    void fiPatchMean(float *fpIn, float *fpMeanV, float fRadius, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, NULL,  fpMeanV,  NULL, NULL,  fRadius,  iWidth, iHeight);
        
    }
    
    
    void fiPatchVar(float *fpIn, float *fpVarV, float fRadius, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, NULL,  NULL,  fpVarV, NULL,  fRadius,  iWidth, iHeight);
        
    }
    
    
    void fiPatchMedian(float *fpIn, float *fpMedianV, float fRadius, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, NULL,  NULL,  NULL, fpMedianV,  fRadius,  iWidth, iHeight);
    }
    
    
    void fiPatchMean(float *fpIn, float *fpMeanV, int rx, int ry, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, NULL,  fpMeanV,  NULL, NULL,  rx,ry,  iWidth, iHeight);
        
    }
    
    void fiPatchVar(float *fpIn, float *fpVarV, int rx, int ry, int iWidth, int iHeight)
    {
        fiPatchStatistics( fpIn, NULL, NULL,  NULL,  fpVarV, NULL,  rx,ry,  iWidth, iHeight);
        
    }
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Image Convolution
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    //! Neumann boundary condition test
    int neumann_bc(int x, int nx, bool *out)
    {
        if(x < 0)
        {
            x = 0;
            *out = true;
        }
        else if (x >= nx)
        {
            x = nx - 1;
            *out = true;
        }
        
        return x;
    }
    
    
    //!Symmetric boundary condition test
    int symmetric_bc(int x, int nx, bool *out)
    {
        if(x < 0)
        {
            const int borde = nx - 1;
            const int xx = -x;
            const int n  = (int)(xx/borde) % 2;
            
            if ( n ) x = borde - ( xx % borde );
            else x = xx % borde;
            *out = true;
        }
        else if ( x >= nx )
        {
            const int borde = nx - 1;
            const int n = (int)(x/borde) % 2;
            
            if ( n ) x = borde - ( x % borde );
            else x = x % borde;
            *out = true;
        }
        
        return x;
    }
    

    
    float*  fiFloatGaussKernel(float std, int & size)
    {
        
        
        
        int n = 4 * ceilf(std) + 1;
        size = n;
        
        
        float* u = new float[n];
        
        
        if (n==1)  u[0]=1.0;
        else
        {
            
            int ishift = (n-1) / 2;
            
            for (int i=ishift; i < n; i++)
            {
                
                float v = (float)(i - ishift) / std;
                
                u[i] = u[n-1-i] = (float) exp(-0.5*v*v);
                
            }
            
        }
        
        
        // normalize
        float fSum = 0.0f;
        for (int i=0; i < n; i++) fSum += u[i];
        for (int i=0; i < n; i++)  u[i] /= fSum;
        
        
        return u;
        
    }
    
    
    
    void fiFloatDirectionalGaussKernel(float xsigma, float ysigma, float angle, float *kernel, int kwidth, int kheight)
    {
        
        assert(kwidth == kheight);
        int ksize = kwidth;
        
        
        float xsigma2 = xsigma*xsigma;
        float ysigma2 = ysigma*ysigma;
        
        int l2 = ksize/2;
        for(int y = -l2; y <= l2; y++)
        for(int x = -l2; x <= l2; x++)
        {
            
            float a = (float) angle * PI / 180.0f;
            float sina = sin(a);
            float cosa = cos(a);
            
            float ax = (float) x * cosa + (float) y * sina;
            float ay = -(float) x * sina + (float) y * cosa;
            kernel[(y+l2) * ksize + x + l2] =  exp(-(ax*ax)/(2.0f*xsigma2)  - (ay*ay)/(2.0f*ysigma2) );
            
        }
        
        
        float sum=0.0;
        for(int i=0; i < ksize*ksize; i++) sum += kernel[i];
        for(int i=0; i < ksize*ksize; i++) kernel[i] /= sum;

        
    }

    
    
    void fiFloatBufferConvolution(float *buffer,float *kernel,int size,int ksize)
    {
        
        for (int i = 0; i < size; i++) {
            
            float sum = 0.0;
            float *bp = &buffer[i];
            float *kp = &kernel[0];
            
            int k=0;
            for(;k + 4 < ksize;  bp += 5, kp += 5, k += 5)
            sum += bp[0] * kp[0] +  bp[1] * kp[1] + bp[2] * kp[2] +
            bp[3] * kp[3] +  bp[4] * kp[4];
            
            
            for(; k < ksize; bp++ , kp++, k++)  sum += *bp * (*kp);
            
            buffer[i] = sum;
        }
    }
    
    
    
    
    void fiFloatHorizontalConvolution(float *u, float *v, int width, int height, float *kernel, int ksize, int boundary)
    {
        
        int halfsize = ksize / 2;
        int buffersize = width + ksize;
        float *buffer = new float[buffersize];
        
        for (int r = 0; r < height; r++)
        {
            
            /// symmetry
            int l = r*width;
            if (boundary == BOUNDARY_CONDITION_SYMMETRIC)
            for (int i = 0; i < halfsize; i++)
            buffer[i] = u[l + halfsize - 1 - i ];
            else
            for (int i = 0; i < halfsize; i++)
            buffer[i] = u[l];
            
            
            for (int i = 0; i < width; i++)
            buffer[halfsize + i] = u[l + i];
            
            
            if (boundary == BOUNDARY_CONDITION_SYMMETRIC)
            for (int i = 0; i <  halfsize; i++)
            buffer[i + width + halfsize] = u[l + width - 1 - i];
            else
            for (int i = 0; i <  halfsize; i++)
            buffer[i + width + halfsize] = u[l+width-1];
            
            fiFloatBufferConvolution(buffer, kernel, width, ksize);
            for (int c = 0; c < width; c++)
            v[r*width+c] = buffer[c];
        }
        
        
        delete[] buffer;
        
    }
    
    
    
    void fiFloatVerticalConvolution(float *u, float *v, int width, int height, float *kernel,int ksize, int boundary)
    {
        int halfsize = ksize / 2;
        int buffersize = height + ksize;
        float *buffer = new float[buffersize];
        
        for (int c = 0; c < width; c++) {
            
            if (boundary == BOUNDARY_CONDITION_SYMMETRIC)
            for (int i = 0; i < halfsize; i++)
            buffer[i] = u[(halfsize-i-1)*width + c];
            else
            for (int i = 0; i < halfsize; i++)
            buffer[i] = u[c];
            
            for (int i = 0; i < height; i++)
            buffer[halfsize + i] = u[i*width + c];
            
            if (boundary == BOUNDARY_CONDITION_SYMMETRIC)
            for (int i = 0; i < halfsize; i++)
            buffer[halfsize + height + i] = u[(height - i - 1)*width+c];
            else
            for (int i = 0; i < halfsize; i++)
            buffer[halfsize + height + i] = u[(height-1) * width + c];
            
            fiFloatBufferConvolution(buffer, kernel, height, ksize);
            
            for (int r = 0; r < height; r++)
            v[r*width+c] = buffer[r];
            
        }
        
        delete[] buffer;
    }
    
    
    void fiSepConvol(float *u,float *v,int width,int height,float *xkernel, int xksize, float *ykernel, int yksize, int boundary)
    {
        
        if (u != v) memcpy(v, u, width*height*sizeof(float));
        
        fiFloatHorizontalConvolution(v, v, width, height, xkernel, xksize, boundary);
        fiFloatVerticalConvolution(v, v, width, height,  ykernel,  yksize, boundary);
        
    }
    
    
    
    
    void fiGaussianConvol(float *u, float *v, int width, int height, float sigma, int boundary)
    {
        
        int ksize;
        float *kernel;
        kernel = fiFloatGaussKernel(sigma,ksize);
        
        if (u != v) memcpy(v, u, width*height*sizeof(float));
        
        fiFloatHorizontalConvolution(v, v, width, height, kernel, ksize, boundary);
        fiFloatVerticalConvolution(v, v, width, height,  kernel,  ksize, boundary);
        
        delete[] kernel;
        
    }
    
    
    
    
    
    void fiConvol(float *u,float *v,int width,int height,float *kernel,int kwidth,int kheight, int boundary)
    {
        
        int K2 = kwidth / 2;
        int L2 = kheight / 2;
        bool b;
        
        for(int y=0 ; y < height; y++)
        for (int x=0 ; x < width; x++)
        {
            
            float S = 0.0;
            
            if (boundary == BOUNDARY_CONDITION_NEUMANN)
            {
                
                for (int l = -L2; l <= L2; l++)
                for (int k = -K2 ; k<= K2; k++)
                {
                    int px= neumann_bc(x+k, width, &b);
                    int py= neumann_bc(y+l, height, &b);
                    
                    S += kernel[kwidth*(l+L2) + k+K2] * u[width*py + px];
                    
                }
                
            } else
            {
                
                for (int l = -L2; l <= L2; l++)
                for (int k = -K2 ; k<= K2; k++)
                {
                    int px= symmetric_bc(x+k, width, &b);
                    int py= symmetric_bc(y+l, height, &b);
                    
                    S += kernel[kwidth*(l+L2) + k+K2] * u[width * py + px] ;
                    
                }
                
                
            }
            
            v[y*width+x] = (float) S;
            
        }
    }
    
    
    
    
    
    
    
    
    float * fiFloatDirectionalGaussKernelS(float xsigma, float ysigma, float angle, float *kernel, int kwidth, int kheight, int sign)
    {
        
        assert(kwidth == kheight);
        int ksize = kwidth;
        
        
        float xsigma2 = xsigma*xsigma;
        float ysigma2 = ysigma*ysigma;
        
        int l2 = ksize/2;
        for(int y = -l2; y <= l2; y++)
            for(int x = -l2; x <= l2; x++)
            {
                
                float a = (float) angle * PI / 180.0f;
                float sina = sin(a);
                float cosa = cos(a);
                
                float ax = (float) x * cosa + (float) y * sina;
                //float ay = -(float) x * sina + (float) y * cosa;
                
                float d2=x*x+y*y;
                
                if (ax*sign > 0)
                    kernel[(y+l2) * ksize + x + l2] =  exp(-d2/(2.0f*ysigma2)  - (ax*ax)/(2.0f*xsigma2) );
                else
                    kernel[(y+l2) * ksize + x + l2] =  exp(-d2/(2.0f*ysigma2) );
                
                
            }
        
        
        float sum=0.0;
        for(int i=0; i < ksize*ksize; i++) sum += kernel[i];
        for(int i=0; i < ksize*ksize; i++) kernel[i] /= sum;
        
        kwidth = ksize;
        kheight = ksize;
        
        return kernel;
    }
    
    
    
    int order_decreasing_kernel(const void *pVoid1, const void *pVoid2)
    {
        struct kernel_item *node1, *node2;
        
        node1=(struct kernel_item *) pVoid1;
        node2=(struct kernel_item *) pVoid2;
        
        if (node1->v < node2->v) return 1;
        if (node1->v > node2->v) return -1;
        return 0;
    }
    
    
    void sort_kernel(float *u, int w, int h, struct sorted_kernel *skernel, float pkernel)
    {
        for (int i=0; i < w; i++)
            for (int j=0; j < h; j++) {
                skernel->items[i+j*w].v=u[i+j*w];
                skernel->items[i+j*w].i=i;
                skernel->items[i+j*w].j=j;
            }
        
        qsort(skernel->items, skernel->nitems, sizeof(struct kernel_item), order_decreasing_kernel);
        
        double S=0;
        int ncut=0;
        while ((S < pkernel) && (ncut < skernel->nitems)) {
            S+=(double) skernel->items[ncut++].v;
        }
        skernel->ncut=ncut;
        //renormalize values
        S=0;
        for (int n=0; n < ncut; n++) S+=(double) skernel->items[n].v;
        for (int n=0; n < ncut; n++) skernel->items[n].v/=S;
        
    }
    
    
    void fiConvol_skernel(float *u,float *v,int width,int height, struct sorted_kernel *skernel, int boundary)
    {
        
        int K2 = skernel->w / 2;
        int L2 = skernel->h / 2;
        bool b;
        int l, k;
        int px, py;
        
        for(int y=0 ; y < height; y++)
            for (int x=0 ; x < width; x++)
            {
                
                float S = 0.0;
                
                if (boundary == BOUNDARY_CONDITION_NEUMANN)
                {
                    for (int n=0; n < skernel->ncut; n++) {
                        l=skernel->items[n].j-L2;
                        k=skernel->items[n].i-K2;
                        px= neumann_bc(x+k, width, &b);
                        py= neumann_bc(y+l, height, &b);
                        S += skernel->items[n].v * u[width*py + px];
                    }
                    
                } else
                {
                    
                    for (int n=0; n < skernel->ncut; n++) {
                        l=skernel->items[n].j-L2;
                        k=skernel->items[n].i-K2;
                        px= symmetric_bc(x+k, width, &b);
                        py= symmetric_bc(y+l, height, &b);
                        S += skernel->items[n].v * u[width*py + px];
                    }
                    
                }
                
                v[y*width+x] = (float) S;
                
            }
    }
    
    
    
    
    //
    //! Noise
    //
    
    void fpAddNoiseGaussian(float *u, float *v, float std, long int randinit, int size)
    {
        
        srand48( (long int) time (NULL) + (long int) getpid()  + (long int) randinit);
        
        for (int i=0; i< size; i++)
        {
            
            float a = drand48();
            float b = drand48();
            float z = (float)(std)*sqrt(-2.0*log(a))*cos(2.0*M_PI*b);
            
            v[i] =  u[i] + (float) z;
            
        }
        
    }
    
    
    
    void fpAddNoiseGaussianAfine(float *u,float *v, float a,float b,long int randinit, int size)
    {
        
        srand48( (long int) time (NULL) + (long int) getpid()  + (long int) randinit);
        
        // Gaussian noise
        for (int i=0; i< size; i++)
        {
            
            float std = (float) (a + b * u[i]);
            std = sqrt(std);
            
            float a0 = drand48();
            float b0 = drand48();
            float z = (float)(std)*sqrt(-2.0*log(a0))*cos(2.0*M_PI*b0);
            
            v[i] = u[i] + (float) z;
            
        }
        
    }
    
 
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Gradient Computation
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    void fiComputeImageGradient(float * fpI,float *fpXgrad, float *fpYgrad, float *fpGrad, float * fpOri, int iWidth, int iHeight, char cType)
    {
        
        assert(fpI != NULL);
        assert(cType == 'c' || cType == 'f');
        
        int iC, iN, iS, iW, iE;
        float xgrad, ygrad;
        
        for (int ih = 0; ih < iHeight; ih++)
            for (int iw = 0; iw < iWidth; iw++)
            {
                
                //! Indexos
                iC = ih * iWidth + iw;
                iN = iC - iWidth;
                iS = iC + iWidth;
                iW = iC - 1;
                iE = iC + 1;
                
                //! Boundary
                if (ih == 0) iN = iC;
                if (ih == iHeight-1) iS = iC;
                if (iw == 0) iW = iC;
                if (iw == iWidth-1) iE = iC;
                
                
                
                if (cType == 'f')
                {
                    //! Forward
                    xgrad = fpI[iE] - fpI[iC];
                    
                    ygrad = fpI[iS] - fpI[iC] ;
                    
                } else
                {
                    
                    //! Centered
                    xgrad = fpI[iE] - fpI[iW];
                    
                    ygrad = fpI[iS] - fpI[iN];
                    
                }
                
                
                if (fpXgrad) fpXgrad[ih * iWidth + iw] =  xgrad;
                
                if (fpYgrad) fpYgrad[ih * iWidth + iw] =  ygrad;
                
                if (fpGrad) fpGrad[ih * iWidth + iw] =  sqrtf(xgrad * xgrad + ygrad * ygrad);
                
                if (fpOri) fpOri[ih * iWidth + iw] =  atan2f(-ygrad,xgrad);
                
            }
        
        
    }
    
    
    
    
    void fiComputeImageGradient(float * fpI, float *fpGrad, float * fpOri, int iWidth, int iHeight, char cType)
    {
        fiComputeImageGradient( fpI, NULL, NULL, fpGrad,  fpOri, iWidth, iHeight, cType);
    }
    
    void fiComputeImageGradient(float * fpI, float *fpGrad, int iWidth, int iHeight, char cType)
    {
        fiComputeImageGradient( fpI, NULL, NULL, fpGrad, NULL , iWidth, iHeight, cType);
        
    }
    
    
    void fiComputeImageLaplacian(float *input, float *out, float sigma, int width, int height)
    {
        
        
        if (sigma > 0.0)
        {
            float * convolved = (float *) malloc(width*height*sizeof(float));
            
            fiGaussianConvol(input, convolved, width, height, sigma, BOUNDARY_CONDITION_SYMMETRIC);
            
            for(int i=0; i< width*height; i++) out[i] = convolved[i] - input[i];
            
            free(convolved);
            
        } else
        {
            
            
            for (int i=0; i < width; i++)
                for (int j=0; j < height; j++)
                {
                    int jm,im, j1,i1;
                    
                    if (j==0) jm=1; else jm=j-1;
                    if (j==height-1) j1=height-2; else j1=j+1;
                    
                    if (i==0) im=1; else im=i-1;
                    if (i==width-1) i1=width-2; else i1=i+1;
                    
                    out[j*width + i] =  - 4.0  * input[width*j+i] + input[width*j+im]+ input[width*j+i1]+input[width*jm + i] + input[width*j1 + i];
                }
        }
        
    }
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Sampling functions
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    void fiImageSample(float *igray,float *ogray, int factor, int width, int height)
    {
        
        int swidth = (int) floor( (float) width / (float) factor);
        int sheight = (int) floor( (float) height / (float) factor);
        
        for(int j=0; j < sheight; j++)
            for(int i=0; i < swidth; i++)
                ogray[j*swidth + i] = igray[ j * factor * width +  i*factor ];
        
    }
    
    
    
    
    float *fiImageSample(float *input, float sampling_factor, int high_width,
                         int high_height, int & low_width, int & low_height)
    {
        low_width = (int) floor((float) high_width / sampling_factor);
        low_height = (int) floor((float) high_height / sampling_factor);
        
        int low_dim = low_width * low_height;
        float *sampled = new float[low_dim];
        
        for(int j = 0; j < low_height; j++)
            for(int i = 0; i < low_width; i++)
                sampled[j*low_width+i] = input[(int)rintf((float)j*sampling_factor)
                                               * high_width
                                               + (int)rintf((float)i*sampling_factor)];
        
        return sampled;
    }
    
    
    
    
    void fiImageSampleAglomeration(float *igray,float *ogray, int factor, int width, int height)
    {
        
        int swidth = (int) floor( (float) width / (float) factor);
        int sheight = (int) floor( (float) height / (float) factor);
        
        float fFactor2 = (float) factor * (float) factor;
        
        for(int j=0; j < sheight; j++)
            for(int i=0; i < swidth; i++)
            {
                
                int pi = i * factor;
                int pj = j * factor;
                
                float fSum = 0.0f;
                for (int r=0; r < factor; r++)
                    for (int s=0; s < factor; s++)
                    {
                        fSum += igray[ (pj+s) * width + (pi+r)];
                    }
                
                
                ogray[j*swidth + i] = fSum / fFactor2;
                
            }
        
        
        
    }
    
    
    
    
    float bilinear_interpolation_at(
                                    const float *input,     // image to be warped
                                    const float  uu,
                                    const float  vv,
                                    const int    width,        // image width
                                    const int    height         // image height
    )
    {
        float  x =  (int) (uu);
        float  y =  (int) (vv);
        int ix = (int) x;
        int iy = (int) y;
        
        x=uu-x;
        y=vv-y;
        
        float c00, c01, c10, c11;
        int mx=ix+1;
        int my=iy+1;
        
        if (ix < 0) ix=0;
        else if (ix >= width) ix=width-1;
        
        if (iy < 0) iy=0;
        else if (iy >= height) iy=height-1;
        
        if (mx < 0) mx=0;
        else if (mx >= width) mx=width-1;
        
        if (my < 0) my=0;
        else if (my >= height) my=height-1;
        
        c00 = input[iy * width + ix];
        c10 = input[iy * width + mx];
        c01 = input[my * width + ix];
        c11 = input[my * width + mx];
        
        
        float  a =  c00 + x * (c10-c00); //c00 * (1.0 - x) + c10 * x;
        float  b =  c01 + x * (c11-c01); //c01 * (1.0 - x) + c11 * x;
        return  a + y * (b-a);  //a * (1 - y) + b * y;
    }
    
  
    void bilinear_interpolation_warp(
                                     const float *input,     // image to be warped
                                     const float *u,         // x component of the vector field
                                     const float *v,         // y component of the vector field
                                     float       *output,    // image warped with bicubic interpolation
                                     const int    nx,        // image width
                                     const int    ny        // image height
    )
    {
        for(int i = 0; i < nx; i++)
        {
            for(int j = 0; j < ny; j++)
            {
                const int p  = j * nx + i;
                const float uu = (float) i + u[p];
                const float vv = (float) j + v[p];
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[p] = bilinear_interpolation_at(input,
                                                      uu, vv, nx, ny);
            }
        }
    }
    
    
    void bilinear_interpolation_translation(
                                     const float *input,     // image to be warped
                                     const float u,         // x component of the vector field
                                     const float v,         // y component of the vector field
                                     float       *output,    // image warped with bicubic interpolation
                                     const int    nx,        // image width
                                     const int    ny        // image height
    )
    {
        for(int i = 0; i < nx; i++)
        {
            for(int j = 0; j < ny; j++)
            {
                const int p  = j * nx + i;
                const float uu = (float) i + u;
                const float vv = (float) j + v;
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[p] = bilinear_interpolation_at(input,
                                                      uu, vv, nx, ny);
            }
        }
    }
    
    
    
    
    
    /**
     *
     * Compute the bicubic interpolation of an image.
     *
     **/
    void bilinear_interpolation_zoom(
                                    const float *input,     // image to be warped
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    const float  fFactor,     // zoom factor
                                    float       *output    // image warped with bicubic interpolation
    )
    {
        
        int znx = (int)rintf( fFactor * (float) nx);
        int zny = (int)rintf( fFactor * (float) ny);
        
        for(int i = 0; i < znx; i++)
            for(int j = 0; j < zny; j++)
            {
                float uu = (float) i / fFactor;
                float vv = (float) j / fFactor;
                
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * znx + i] = bilinear_interpolation_at(input,
                                                               uu, vv, nx, ny);
            }
    }

    
    void nn_interpolation_warp(
                                     const float *input,     // image to be warped
                                     const float *u,         // x component of the vector field
                                     const float *v,         // y component of the vector field
                                     float       *output,    // image warped with bicubic interpolation
                                     const int    nx,        // image width
                                     const int    ny        // image height
    )
    {
        
        
        for(int i = 0; i < nx; i++)
        {
            int   p;
            int uu;
            int vv;
            
            for(int j = 0; j < ny; j++)
            {
                p  = j * nx + i;
                uu = (int) rintf( (float) i + u[p] );
                vv = (int) rintf( (float) j + v[p] );
           
                if (uu < 0) uu=0;
                if (uu > nx-1) uu = nx-1;
                
                if (vv < 0) vv=0;
                if (vv > ny-1) vv = ny-1;
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * nx + i] = input[ vv * nx + uu ];
                
                
            }
        }
    }
    

    
    
    void bilinear_interpolation_zoom(
                                     const float *input,     // image to be warped
                                     const int    nx,        // image width
                                     const int    ny,        // image height
                                     const float  fFactorx,     // zoom factor
                                     const float  fFactory,     // zoom factor
                                     float       *output    // image warped with bicubic interpolation
    )
    {
        
        int znx = (int)rintf( fFactorx * (float) nx);
        int zny = (int)rintf( fFactory * (float) ny);
        
        for(int i = 0; i < znx; i++)
            for(int j = 0; j < zny; j++)
            {
                float uu = (float) i / fFactorx;
                float vv = (float) j / fFactory;
                
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * znx + i] = bilinear_interpolation_at(input,
                                                                uu, vv, nx, ny);
            }
    }
    
    
    
    
    
    
    /**
     *
     * Cubic interpolation in one dimension
     *
     **/
    static double cubic_interpolation_cell (
                                            double v[4],  //interpolation points
                                            double x      //point to be interpolated
    )
    {
        return  v[1] + 0.5 * x * (v[2] - v[0] +
                                  x * (2.0 *  v[0] - 5.0 * v[1] + 4.0 * v[2] - v[3] +
                                       x * (3.0 * (v[1] - v[2]) + v[3] - v[0])));
    }
    
    
    /**
     *
     * Bicubic interpolation in two dimensions
     *
     **/
    static double bicubic_interpolation_cell (
                                              double p[4][4], //array containing the interpolation points
                                              double x,       //x position to be interpolated
                                              double y        //y position to be interpolated
    )
    {
        double v[4];
        v[0] = cubic_interpolation_cell(p[0], y);
        v[1] = cubic_interpolation_cell(p[1], y);
        v[2] = cubic_interpolation_cell(p[2], y);
        v[3] = cubic_interpolation_cell(p[3], y);
        return cubic_interpolation_cell(v, x);
    }
    
    
    /**
     *
     * Compute the bicubic interpolation of a point in an image.
     * Detect if the point goes outside the image domain.
     *
     **/
    float bicubic_interpolation_at(
                                   const float *input, // image to be interpolated
                                   const float  uu,    // x component of the vector field
                                   const float  vv,    // y component of the vector field
                                   const int    nx,    // image width
                                   const int    ny,    // image height
                                   const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                   const float  bvalue // return this value outside the region
    )
    {
        const int sx = (uu < 0)? -1: 1;
        const int sy = (vv < 0)? -1: 1;
        
        int x, y, mx, my, dx, dy, ddx, ddy;
        bool b=false;
        
        //apply the corresponding boundary conditions
        
        x   = symmetric_bc((int) uu, nx, &b);
        y   = symmetric_bc((int) vv, ny, &b);
        mx  = symmetric_bc((int) uu - sx, nx, &b);
        my  = symmetric_bc((int) vv - sx, ny, &b);
        dx  = symmetric_bc((int) uu + sx, nx, &b);
        dy  = symmetric_bc((int) vv + sy, ny, &b);
        ddx = symmetric_bc((int) uu + 2*sx, nx, &b);
        ddy = symmetric_bc((int) vv + 2*sy, ny, &b);
        
        
        
        if (b && !binterpolation)
            return bvalue;
        
        else
        {
            //obtain the interpolation points of the image
            const float p11 = input[mx  + nx * my];
            const float p12 = input[x   + nx * my];
            const float p13 = input[dx  + nx * my];
            const float p14 = input[ddx + nx * my];
            
            const float p21 = input[mx  + nx * y];
            const float p22 = input[x   + nx * y];
            const float p23 = input[dx  + nx * y];
            const float p24 = input[ddx + nx * y];
            
            const float p31 = input[mx  + nx * dy];
            const float p32 = input[x   + nx * dy];
            const float p33 = input[dx  + nx * dy];
            const float p34 = input[ddx + nx * dy];
            
            const float p41 = input[mx  + nx * ddy];
            const float p42 = input[x   + nx * ddy];
            const float p43 = input[dx  + nx * ddy];
            const float p44 = input[ddx + nx * ddy];
            
            //create array
            double pol[4][4] = {
                {p11, p21, p31, p41},
                {p12, p22, p32, p42},
                {p13, p23, p33, p43},
                {p14, p24, p34, p44}
            };
            
            //return interpolation
            return bicubic_interpolation_cell(pol, uu-x, vv-y);
        }
    }
    
    
    /**
     *
     * Compute the bicubic interpolation of an image.
     *
     **/
    void bicubic_interpolation_translation(
                                           const float *input,     // image to be warped
                                           const float u,         // x component of the vector field
                                           const float v,         // y component of the vector field
                                           const int    nx,        // image width
                                           const int    ny,        // image height
                                           const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                           const float  bvalue,   //if true, return this value outside the region
                                           float       *output    // image warped with bicubic interpolation
    )
    {
        
        
        for(int i = 0; i < ny; i++)
            for(int j = 0; j < nx; j++)
            {
                const int   p  = i * nx + j;
                const float uu = (float) (j + u);
                const float vv = (float) (i + v);
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[p] = bicubic_interpolation_at(input,
                                                     uu, vv, nx, ny, binterpolation, bvalue);
            }
    }
    
    
    
 
    
   
    void bicubic_interpolation_warp(
                                    const float *input,     // image to be warped
                                    const float *u,         // x component of the vector field
                                    const float *v,         // y component of the vector field
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                    const float  bvalue,   //if true, return this value outside the region
                                    float       *output    // image warped with bicubic interpolation
    )
    {
        
        
        for(int i = 0; i < ny; i++)
            for(int j = 0; j < nx; j++)
            {
                const int   p  = i * nx + j;
                const float uu = (float) j + u[p];
                const float vv = (float) i + v[p];
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[p] = bicubic_interpolation_at(input,
                                                     uu, vv, nx, ny, binterpolation, bvalue);
            }
    }
    
    
    
    
    
    /**
     *
     * Compute the bicubic interpolation of an image.
     *
     **/
    void bicubic_interpolation_zoom(
                                    const float *input,     // image to be warped
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    const float  fFactor,     // zoom factor
                                    const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                    const float bvalue,           // value outside the region
                                    float       *output    // image warped with bicubic interpolation
    )
    {
        
        int znx = (int)rintf( fFactor * (float) nx);
        int zny = (int)rintf( fFactor * (float) ny);
        
        for(int i = 0; i < znx; i++)
            for(int j = 0; j < zny; j++)
            {
                float uu = (float) i / fFactor;
                float vv = (float) j / fFactor;
                
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * znx + i] = bicubic_interpolation_at(input,
                                                               uu, vv, nx, ny, binterpolation, bvalue);
            }
    }
    
    
    
    
    void bicubic_interpolation_zoom(
                                    const float *input,     // image to be warped
                                    const int    nx,        // image width
                                    const int    ny,        // image height
                                    const float  fFactorx,     // zoom factor
                                    const float  fFactory,     // zoom factor
                                    const int    binterpolation,  // interpolate boundary if activated, else return bvalue
                                    const float bvalue,           // value outside the region
                                    float       *output    // image warped with bicubic interpolation
    )
    {
        
        int znx = (int)rintf( fFactorx * (float) nx);
        int zny = (int)rintf( fFactory * (float) ny);
        
        for(int i = 0; i < znx; i++)
            for(int j = 0; j < zny; j++)
            {
                float uu = (float) i / fFactorx;
                float vv = (float) j / fFactory;
                
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * znx + i] = bicubic_interpolation_at(input,
                                                               uu, vv, nx, ny, binterpolation, bvalue);
            }
    }
    
    
    
    
    void bicubic_homography_interpolation(float *input, int width, int height, laMatrix &H, float bg, float *out, int nwidth, int nheight)
    {
        
        
        //! Inverse matrix
        laMatrix V(3,3);
        luinv(H, V);
        
        float fwidth = (float) width - 1.0;
        float fheight = (float) height - 1.0;
        
        
        //! For each point in new image we compute its anti image and interpolate the new value
        for(int i=0; i < nwidth; i++)
            for(int j=0; j < nheight; j++)
            {
                
                
                //! Compute transformed vector
                laVector vec(3), vres(3);
                vec[0] = (float) i;
                vec[1] = (float) j;
                vec[2] = 1.0f;
                
                vres = V * vec;
                
                if (vres[2] != 0.0f)
                {
                    
                    vres[0] /= vres[2]; vres[1] /= vres[2];
                    
                    float xp =  (float) vres[0];
                    float yp =  (float) vres[1];
                    
                    
                    if (xp>=0 && xp < fwidth && yp>=0 && yp < fheight)
                    {
                        
                        float res =  bicubic_interpolation_at(input,
                                                              xp, yp, width, height, 0, bg);
                        
                        out[j*nwidth+i] = res;
                        
                    }else
                        out[j*nwidth+i] = bg;
                    
                } else
                {
                    out[j*nwidth+i] = bg;
                }
                
            }
        
    }
    
    
    
    
    void nn_interpolation_zoom(
                               const float *input,     // image to be warped
                               const int    nx,        // image width
                               const int    ny,        // image height
                               const float  fFactor,     // zoom factor
                               float       *output    // image warped with bicubic interpolation
    )
    {
        
        int znx = (int)rintf( fFactor * (float) nx);
        int zny = (int)rintf( fFactor * (float) ny);
        
        for(int i = 0; i < znx; i++)
            for(int j = 0; j < zny; j++)
            {
                int uu = (int) floorf((float) i / fFactor);
                int vv = (int) floorf((float) j / fFactor);
                
                if (uu < 0) uu=0;
                if (uu > nx-1) uu = nx-1;
                
                if (vv < 0) vv=0;
                if (vv > ny-1) vv = ny-1;
                
                // obtain the bicubic interpolation at position (uu, vv)
                output[j * znx + i] = input[ vv * nx + uu ];
                
            }
    }
  
    
    
    
    
    
    /**
     * \brief   Tabulates exp(-x) function
     *
     *
     * @param[in]  lut	vector
     * @param[in]  size	length of the vector
     *
     */
    
    void  wxFillExpLut(float *lut, int size)
    {
        for(int i=0; i< size;i++)   lut[i]=   expf( - (float) i / LUTPRECISION);
    }
    
    
    
    
    /**
     * \brief   Computes exp(-x) using lut table
     *
     *
     * @param[in]  dif	value
     * @param[in]  lut	lookup table
     *
     */
    float wxSLUT(float dif, float *lut)
    {
        
        if (dif >= (float) LUTMAXM1) return 0.0;
        
        int  x= (int) floor( (float) dif * (float) LUTPRECISION);
        
        float y1=lut[x];
        float y2=lut[x+1];
        
        return y1 + (y2-y1)*(dif*LUTPRECISION -  x);
    }
    
    
    void  wxFillExpLut(double *lut, int size)
    {
        for(int i=0; i< size;i++)   lut[i]=   expl( - (double) i / dLUTPRECISION);
    }
    

    
    double wxSLUT(double dif, double *lut)
    {
        
        if (dif >= (double) dLUTMAXM1) return 0e0;
        
        int  x= (int) floor( (double) dif * (double) dLUTPRECISION);
        
        double y1=lut[x];
        double y2=lut[x+1];
        
        return (double) (y1 + (y2-y1)*(dif*dLUTPRECISION - x));
    }
    
    
    
    
    
    
    //
    //! Patch distances
    //
    
    
    
    float fiL2FloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int width0, int width1)
    {
        
        
        
        float dist=0.0;
        for (int s=-yradius; s<= yradius; s++){
            
            int l = (j0+s)*width0 + (i0-xradius);
            float *ptr0 = &u0[l];
            
            l = (j1+s)*width1 + (i1-xradius);
            float *ptr1 = &u1[l];
            
            for(int r=-xradius;r<=xradius;r++,ptr0++,ptr1++){	float dif = (*ptr0 - *ptr1); dist += (dif*dif); }
            
        }
        
        return dist;
    }
    
    
    
    
    float fiL2FloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int width0, int width1)
    {
        
        
        return fiL2FloatDist(u0,u1,i0,j0,i1,j1, xradius, xradius,  width0,  width1);
    }
    
    
    

    float fiL2FloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1)
    {
        
        float dif = 0.0f;
        
        
        for (int ii=0; ii < channels; ii++) {
            
            dif += fiL2FloatDist(u0[ii],u1[ii],i0,j0,i1,j1,xradius,yradius,width0,width1);
            
        }
        
        dif /= (float) channels;
        return dif;
        
    }
    
    
    float fiL2FloatDist_NN(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1)
    {
        
        float dif = 0.0f;
        
        
        for (int ii=0; ii < channels; ii++) {
            
            dif += fiL2FloatDist(u0[ii],u1[ii],i0,j0,i1,j1,xradius,yradius,width0,width1);
            
        }
        
        return dif;
        
    }
    
    
    
    //
    //! Double patch distances
    //
    
    double fiL2DoubleDist(double *u0, double *u1, int i0, int j0, int i1, int j1, int xradius, int yradius, int width0, int width1)
    {
        double dist = 0e0;
        
        for(int s = -yradius; s <= yradius; s++)
        {
            int l = (j0 + s) * width0 + (i0 - xradius);
            double *ptr0 = &u0[l];
            
            l = (j1 + s) * width1 + (i1 - xradius);
            double *ptr1 = &u1[l];
            
            for(int r = -xradius; r <= xradius; r++, ptr0++, ptr1++)
            {
                double dif = (*ptr0 - *ptr1);
                dist += (dif*dif);
            }
            
        }
        
        return dist;
    }
    
    
    
    double fiL2DoubleDist(double *u0, double *u1, int i0, int j0, int i1, int j1, int xradius, int width0, int width1)
    {
        return fiL2DoubleDist(u0, u1, i0, j0, i1, j1, xradius, xradius, width0, width1);
    }
    
    
    
    double fiL2DoubleDist(double **u0, double **u1, int i0, int j0, int i1, int j1, int xradius, int yradius, int channels, int width0, int width1)
    {
        double dif = 0e0;
        
        for (int ii = 0; ii < channels; ii++)
        dif += fiL2DoubleDist(u0[ii], u1[ii], i0, j0, i1, j1, xradius, yradius, width0, width1);
        
        dif /= (double) channels;
        
        return dif;
    }
    
    
    
    
    
    float fiL2FloatWDist ( float * u0, float *u1, int i0, int j0,int i1,int j1,int xradius, int yradius, float *kernel, int width0, int width1)
    {
        
        float *ptrk=&kernel[0];
        float dist=0.0;
        for (int s=-yradius; s<= yradius; s++)
        {
            
            int l = (j0+s)*width0 + (i0-xradius);
            float *ptr0 = &u0[l];
            
            l = (j1+s)*width1 + (i1-xradius);
            float *ptr1 = &u1[l];
            
            for(int r=-xradius;r<=xradius;r++,ptr0++,ptr1++,ptrk++){ float dif = (*ptr0 - *ptr1); dist += *ptrk*(dif*dif); }
            
            
        }
        
        return dist;
    }
    
    
    
    
    
    float fiL2FloatWDist ( float ** u0, float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, float *kernel, int channels, int width0, int width1)
    {
        
        float dif = 0.0f;
        for (int ii=0; ii < channels; ii++) {
            
            dif += fiL2FloatWDist(u0[ii],u1[ii],i0,j0,i1,j1,xradius, yradius, kernel,width0,width1);
            
        }
        
        return dif;
    }
    
    
    
    
    
    
    
    //
    //! Histogram related
    //
    
    void fpHisto(float* input, laVector &histo, float *iminim, float *imaxim, int *n, float *s, int size, char flag)
    {
        
        assert(flag == 's' || flag == 'n');
        
        float minim;
        if (iminim) minim = *iminim;
        else minim = fpMin(input, NULL, size);
        
        float maxim;
        if (imaxim) maxim = *imaxim;
        else maxim = fpMax(input, NULL, size);
        
        int num;
        float step;
        if (flag == 'n')
        {
            num = *n;
            step = (maxim-minim)/ (float)num;
            *s = step;
            
        } else
        {
            step = *s;
            num = (int)(0.5+(maxim-minim)/step);
            *n = num;
        }
        
        histo.create(num); histo=0.0;
        
        for(int i=0; i < size; i++)
        {
            
            int cell = (int) floorf((input[i]-minim) / step);
            
            if (cell < 0) cell = 0;
            if (cell >= num) cell = num - 1;
            
            histo[cell]++;
        }
        
    }
    
    
    void fk_apply_histo(float *Histo,float *HistoInverse,int iRangeMax, float *src,float *srt, int width, int height)
    {
        
        
        
        int icounter=0;
        for (int adr=0; adr < width * height ; adr++) if ( (int) rintf(src[adr]) >= 0) icounter++;
        
        
        for (int adr=0; adr < width * height ; adr++)
        {
            
            int it = (int) rintf(src[adr]);
          
            if (it>=0)
            {
                
                if (it >= iRangeMax) it = iRangeMax -1;
                
                
                float x = Histo[it] * HSP_NB_CASES / (float) icounter;
                int k= (int) rintf(x);
                
                if (k == 0) k=1;
                if (k == HSP_NB_CASES) k = HSP_NB_CASES - 1;
                
                srt[adr]=HistoInverse[k];
                
            } else srt[adr] = src[adr];
            
            
            
        }
        
    }
    
    
    
    
    
    void fk_fill_histo(float *Histo,float *HistoInverse, int iRangeMax, float *in, int width, int height)
    {
        
        
        // clear data
        for (int i=0; i < iRangeMax; i++) Histo[i]=0.0f;
        
        
        // fill histogram
        int icounter = 0;
        for (int i=0; i < width * height; i++)
        {
            int ivalue = rintf(in[i]);
            if (ivalue >= 0)
            {
                if (ivalue >= iRangeMax) ivalue = iRangeMax - 1;
                Histo[ivalue]++;
                icounter++;
            }
        }
        
        
        // accumulate histogram H(i) in [0, iCounter]
        for (int i=1; i < iRangeMax;i++)
        Histo[i]+=Histo[i-1];
        
        // compute the inverse accumulated histogram
        HistoInverse[0]=0.0;
        
        float xp=0.0;
        for (int ii=1; ii <= HSP_NB_CASES;ii++)
        {
            int x=-1;
            float aux = (float) ii * (float) icounter / (float) (HSP_NB_CASES + 1);
            
            for (int k=0; k <  iRangeMax && Histo[k] < aux; k++) {x=k;}
            
            if (x==-1)  HistoInverse[ii]=0.0;
            else
            {
                float dif = Histo[x+1]-Histo[x];
                
                if( fabs(dif) < 0.01)  HistoInverse[ii] =    (float) x;
                else HistoInverse[ii] =    (float) x + ((aux - Histo[x]) / (dif));
                
                HistoInverse[ii] =    MAX( (float) xp,  HistoInverse[ii] );
            }
            
            xp=HistoInverse[ii];
        }
        
        
        
    }
    
    
    
    
    void fk_histogram_specification(float *in1, float *in2, float *out, int width1, int height1, int width2, int height2)
    {
        
        assert(in1 !=NULL && in2 != NULL);
        
        float fRangeMax;
        fRangeMax =   fpMax(in1, NULL, width1 * height1);
        fRangeMax =   MAX(fRangeMax, fpMax(in2, NULL, width2 * height2));
        
        float fRangeMin;
        fRangeMin =   fpMin(in1, NULL, width1 * height1);
        fRangeMin =   MIN(fRangeMin, fpMin(in2, NULL, width2 * height2));
        
        //if (fRangeMin < 0.0f)
        //printf("warning :: fk_histogram_specification :: negative values not being used\n");
        
        
        // memory
        int iRangeMax = (int) rintf(fRangeMax) + 1;
        
        assert(iRangeMax > 0);
        float *histo1 = new float[iRangeMax];
        float *histo2 = new float[iRangeMax];
        
        float *histoInverse1 = new float[HSP_NB_CASES + 1];
        float *histoInverse2 = new float[HSP_NB_CASES + 1];
        
        // compute histograms
        fk_fill_histo(histo1,histoInverse1, iRangeMax, in1, width1, height1);
        fk_fill_histo(histo2,histoInverse2, iRangeMax, in2, width2, height2);
        
        // specificate histogram
        fk_apply_histo(histo2,histoInverse1,iRangeMax,in2, out, width2, height2);
        
        delete[] histo1;
        delete[] histo2;
        
        delete[] histoInverse1;
        delete[] histoInverse2;
        
    }
    
    
    
    
    
    void fk_histogram_midway(float *in1, float *in2, float *out1, float *out2, int width1, int height1, int width2, int height2)
    {
        
        float fRangeMax;
        fRangeMax =   fpMax(in1, NULL, width1 * height1);
        fRangeMax =   MAX(fRangeMax, fpMax(in2, NULL, width1 * height1));
        
        float fRangeMin;
        fRangeMin =   fpMin(in1, NULL, width1 * height1);
        fRangeMin =   MIN(fRangeMin, fpMin(in2, NULL, width2 * height2));
        
        if (fRangeMin < 0.0f)
        printf("warning :: fk_histogram_specification :: negative values not being used\n");
        
        
        // memory
        int iRangeMax = (int) rintf(fRangeMax) + 1;
        
        float *histo1 = new float[iRangeMax];
        float *histo2 = new float[iRangeMax];
        
        float *histoInverse1 = new float[HSP_NB_CASES + 1];
        float *histoInverse2 = new float[HSP_NB_CASES + 1];
 
        //! compute histograms
        fk_fill_histo(histo1,histoInverse1, iRangeMax, in1, width1, height1);
        fk_fill_histo(histo2,histoInverse2, iRangeMax, in2, width2, height2);
        
        
        //! compute average histogram
        float *histoAverage = new float[HSP_NB_CASES+1];
        for (int i=0; i < HSP_NB_CASES + 1; i++) histoAverage[i] = 0.0f;

        for (int k=0; k < HSP_NB_CASES + 1; k++)
        {
            histoAverage[k] = 0.5f * (histoInverse1[k] + histoInverse2[k]);
        }
        
        
        fk_apply_histo(histo1,histoAverage,iRangeMax,in1, out1, width1, height1);
        fk_apply_histo(histo2,histoAverage,iRangeMax,in2, out2, width2, height2);
        
        
        delete[] histo1;
        delete[] histo2;
        
        delete[] histoInverse1;
        delete[] histoInverse2;

        delete[] histoAverage;
        
    }
    

    
    
    
    
    void fk_histogram_midway_sequence(float **in, float **out, int nImages, int width, int height)
    {
        
        
        float fRangeMax;
        fRangeMax =   fpMax(in[0], NULL, width * height);
        for (int ii=1; ii < nImages ; ii++)
        {
            fRangeMax =   MAX(fRangeMax, fpMax(in[ii], NULL, width * height));
        }
        
        
        
        //! memory
        int iRangeMax = (int) rintf(fRangeMax) + 1;
        
        
        float **histo = new float*[nImages];
        float **histoInverse = new float*[nImages];
        
        for (int ii=0; ii < nImages; ii++)
        {
            
            histo[ii] = new float[iRangeMax];
            histoInverse[ii] = new float[HSP_NB_CASES + 1];
            
        }
        
        
        //! compute histograms
        for (int ii=0; ii < nImages; ii++)
        {
            
            fk_fill_histo(histo[ii],histoInverse[ii], iRangeMax, in[ii], width, height);
            
        }
        
        
        
        //! compute average histogram
        float *histoAverage = new float[HSP_NB_CASES+1];
        for (int i=0; i < HSP_NB_CASES + 1; i++) histoAverage[i] = 0.0f;
        
        
        for (int k=0; k < HSP_NB_CASES + 1; k++)
        {
            for (int ii=0; ii < nImages; ii++)
            {
                
                histoAverage[k]  += histoInverse[ii][k];
            }
            
            
            histoAverage[k] /= (float) nImages;
            
        }
        
        
        
        
        //! specificate histograms
        for (int k=0; k < HSP_NB_CASES + 1; k++)
        {
            for (int ii=0; ii < nImages; ii++)
            {
                
                histoInverse[ii][k] =  histoAverage[k];
            }
        }
        
        
        for (int ii=0; ii < nImages; ii++)
        {
            
            fk_apply_histo(histo[ii],histoInverse[ii],iRangeMax, in[ii], out[ii], width, height);
            
        }
        
        
        
    }
    
    
    
    
    
    void fk_apply_histo_mask(float *Histo,float *HistoInverse,int iRangeMax, float *src, float *mask, float *srt, int width, int height)
    {
        
        int icounter=0;
        for (int adr=0; adr < width * height ; adr++) if ( mask[adr]>0.0f && (int) rintf(src[adr]) >= 0) icounter++;
        
        
        for (int adr=0; adr < width * height ; adr++)
            if ( mask[adr]>0.0f)
            {
                
                int it = (int) rintf(src[adr]);
                //if (it < 0) it=0;
                
                if (it>=0)
                {
                    
                    if (it >= iRangeMax) it = iRangeMax -1;
                    
                    
                    float x = Histo[it] * HSP_NB_CASES / (float) icounter;
                    int k= (int) rintf(x);
                    
                    if (k == 0) k=1;
                    if (k == HSP_NB_CASES) k = HSP_NB_CASES - 1;
                    
                    srt[adr]=HistoInverse[k];
                    
                } else srt[adr] = src[adr];
                
                
                
            }
        
    }
    
    
    
    
    
    
    void fk_fill_histo_mask(float *Histo,float *HistoInverse, int iRangeMax, float *in, float *mask, int width, int height)
    {
        
        
        // clear data
        for (int i=0; i < iRangeMax; i++) Histo[i]=0.0f;
        
        
        // fill histogram
        int icounter = 0;
        for (int i=0; i < width * height; i++)
            if (mask[i] > 0.0f)
            {
                int ivalue = rintf(in[i]);
                //if (ivalue < 0) ivalue = 0;
                
                if (ivalue >= 0)
                {
                    if (ivalue >= iRangeMax) ivalue = iRangeMax - 1;
                    Histo[ivalue]++;
                    icounter++;
                }
                
                
            }
        
        
        // accumulate histogram
        for (int i=1; i < iRangeMax;i++)
            Histo[i]+=Histo[i-1];
        
        
        //for (int i=0; i < iRangeMax;i++)
        //    printf("%d: %f\n",i,Histo[i]);
        
        
        // compute the inverse histogram
        HistoInverse[0]=0.0;
        
        
        float xp=0.0;
        
        
        for (int ii=1; ii <= HSP_NB_CASES;ii++)
        {
            
            int x=-1;
            
            float aux = (float) ii * (float) icounter / (float) (HSP_NB_CASES + 1);
            
            for (int k=0; k <  iRangeMax && Histo[k] < aux; k++) {x=k;}
            
            if (x==-1)  HistoInverse[ii]=0.0;
            else
            {
                float dif = Histo[x+1]-Histo[x];
                
                if( fabs(dif) < 0.01)  HistoInverse[ii] =    (float) x;
                else HistoInverse[ii] =    (float) x + ((aux - Histo[x]) / (dif));
                
                HistoInverse[ii] =    MAX( (float) xp,  HistoInverse[ii] );
            }
            
            xp=HistoInverse[ii];
        }
        
        
        
    }
    
    
    
    
    void fk_histogram_specification_mask(float *in1, float *mask1, float *in2, float *mask2, float *out, int width1, int height1, int width2, int height2)
    {
        
        assert(in1 !=NULL && in2 != NULL);
        
        float fRangeMax = -fLarge;
        for (int ii=0; ii < width1 * height1; ii++)
        {
            if (mask1[ii] > 0.0f && in1[ii] > fRangeMax) fRangeMax = in1[ii];
        }
        
        for (int ii=0; ii < width2 * height2; ii++)
        {
            if (mask2[ii] > 0.0f && in2[ii] > fRangeMax) fRangeMax = in2[ii];
        }
        
        
        
        // memory
        int iRangeMax = (int) rintf(fRangeMax) + 1;
        
        
        
        assert(iRangeMax > 0);
        float *histo1 = new float[iRangeMax];
        float *histo2 = new float[iRangeMax];
        
        float *histoInverse1 = new float[HSP_NB_CASES + 1];
        float *histoInverse2 = new float[HSP_NB_CASES + 1];
        
        
        
        // compute histograms
        fk_fill_histo_mask(histo1,histoInverse1, iRangeMax, in1, mask1, width1, height1);
        fk_fill_histo_mask(histo2,histoInverse2, iRangeMax, in2, mask2, width2, height2);
        
        
        
        
        
        // specificate histogram
        fk_apply_histo_mask(histo2,histoInverse1,iRangeMax,in2, mask2, out, width2, height2);
        
        delete[] histo1;
        delete[] histo2;
        
        delete[] histoInverse1;
        delete[] histoInverse2;
        
    }
    
    
    
    
    void fk_histogram_midway_mask(float *in1, float* mask1, float *in2, float *mask2, float *out1, float *out2, int width1, int height1, int width2, int height2)
    {
        
        float fRangeMax = -fLarge;
        float fRangeMin = fLarge;
        for (int ii=0; ii < width1 * height1; ii++)
        {
            if (mask1[ii] > 0.0f && in1[ii] > fRangeMax) fRangeMax = in1[ii];
            if (mask1[ii] > 0.0f && in1[ii] < fRangeMin) fRangeMin = in1[ii];
        }
        
        for (int ii=0; ii < width2 * height2; ii++)
        {
            if (mask2[ii] > 0.0f && in2[ii] > fRangeMax) fRangeMax = in2[ii];
            if (mask2[ii] > 0.0f && in2[ii] < fRangeMin) fRangeMin = in2[ii];
        }
        
            
        if (fRangeMin < 0.0f)
            printf("warning :: fk_histogram_midway :: negative values not being used\n");
        
        
        // memory
        int iRangeMax = (int) rintf(fRangeMax) + 1;
        
        float *histo1 = new float[iRangeMax];
        float *histo2 = new float[iRangeMax];
        
        float *histoInverse1 = new float[HSP_NB_CASES + 1];
        float *histoInverse2 = new float[HSP_NB_CASES + 1];
        
        //! compute histograms
        fk_fill_histo_mask(histo1,histoInverse1, iRangeMax, in1, mask1, width1, height1);
        fk_fill_histo_mask(histo2,histoInverse2, iRangeMax, in2, mask2, width2, height2);
        
        
        
        //! compute average histogram
        float *histoAverage = new float[HSP_NB_CASES+1];
        for (int i=0; i < HSP_NB_CASES + 1; i++) histoAverage[i] = 0.0f;
        
        for (int k=0; k < HSP_NB_CASES + 1; k++)
        {
            histoAverage[k] = 0.5f * (histoInverse1[k] + histoInverse2[k]);
        }
        
        
        fk_apply_histo_mask(histo1,histoAverage,iRangeMax,in1, mask1, out1, width1, height1);
        fk_apply_histo_mask(histo2,histoAverage,iRangeMax,in2, mask2, out2,width2, height2);
        
        delete[] histo1;
        delete[] histo2;
        
        delete[] histoInverse1;
        delete[] histoInverse2;
        
        delete[] histoAverage;
        
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! OLD STUFF
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    
    
    
    
    
    
    
    float fiCFloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int width0, int width1)
    {
        
        
        
        float dist=0.0;
        for (int s=-yradius; s<= yradius; s++){
            
            int l = (j0+s)*width0 + (i0-xradius);
            float *ptr0 = &u0[l];
            
            l = (j1+s)*width1 + (i1-xradius);
            float *ptr1 = &u1[l];
            
            for(int r=-xradius;r<=xradius;r++,ptr0++,ptr1++)
            {
                float dif = (*ptr0 - *ptr1);
                float adif = fabsf(*ptr0) + fabsf(*ptr1);
                if (adif>fTiny)
                dist += fabsf(dif) / adif;
                
            }
            
        }
        
        return dist;
    }
    
    
    
    float fiBCFloatDist(float *u0,float *u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int width0, int width1)
    {
        
        
        
        float dist=0.0;
        float adist = 0.0f;
        for (int s=-yradius; s<= yradius; s++){
            
            int l = (j0+s)*width0 + (i0-xradius);
            float *ptr0 = &u0[l];
            
            l = (j1+s)*width1 + (i1-xradius);
            float *ptr1 = &u1[l];
            
            for(int r=-xradius;r<=xradius;r++,ptr0++,ptr1++)
            {
                float dif = (*ptr0 - *ptr1);
                
                adist += fabsf(*ptr0 + *ptr1);
                dist += fabsf(dif);
                
            }
            
        }
        
        if (adist > fTiny)
        return dist / adist;
        else
        return fLarge;
    }
    
    
    
    
    
    
    
    
    
    
    float fiBCFloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1)
    {
        
        float dif = 0.0f;
        
        
        for (int ii=0; ii < channels; ii++) {
            
            dif += fiBCFloatDist(u0[ii],u1[ii],i0,j0,i1,j1,xradius,yradius,width0,width1);
            
        }
        
        dif /= (float) channels;
        return dif;
        
    }
    
    
    float fiCFloatDist(float **u0,float **u1,int i0,int j0,int i1,int j1,int xradius, int yradius, int channels, int width0, int width1)
    {
        
        float dif = 0.0f;
        
        
        for (int ii=0; ii < channels; ii++) {
            
            dif += fiCFloatDist(u0[ii],u1[ii],i0,j0,i1,j1,xradius,yradius,width0,width1);
            
        }
        
        dif /= (float) channels;
        return dif;
        
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////
    //! BEGIN Apply transformations
    ////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    void compute_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, laMatrix &H)
    {
        
        ////////////////// Compute Baricenter
        float lx = 0.0,  ly = 0.0;
        float rx = 0.0,  ry = 0.0;
        
        for (int i=0; i< n; i++) {
            
            lx += x0[i];
            ly += y0[i];
            
            rx += x1[i];
            ry += y1[i];
        }
        
        lx /= (float) n;
        ly /= (float) n;
        rx /= (float) n;
        ry /= (float) n;
        
        
        /////////////// Normalize points without modifying original vectors
        
        float *px0 = new float[n];
        float *py0 = new float[n];
        
        float *px1 = new float[n];
        float *py1 = new float[n];
        
        float spl= 0.0f, spr = 0.0f;
        for(int i=0; i < n; i++)
        {
            
            px0[i] = x0[i] - lx;
            py0[i] = y0[i] - ly;
            
            px1[i] = x1[i] - rx;
            py1[i] = y1[i] - ry;
            
            spl += sqrtf(px0[i] * px0[i] + py0[i]*py0[i]);
            spr += sqrtf(px1[i] * px1[i] + py1[i]*py1[i]);
            
        }
        
        
        spl = sqrtf(2.0f) / spl;
        spr = sqrtf(2.0f) / spr;
        
        for (int i=0; i< n; i++) {
            
            px0[i] *= spl;
            py0[i] *= spl;
            
            px1[i] *= spr;
            py1[i] *= spr;
            
        }
        
        
        /////////////////////////////////////////////////////// Minimization problem || Ah ||
        
        
        laMatrix Tpl(3,3), Timinv(3,3);
        
        // similarity transformation of the plane
        Tpl[0][0] = spl; Tpl[0][1] = 0.0; Tpl[0][2] = -spl*lx;
        Tpl[1][0] = 0.0; Tpl[1][1] = spl; Tpl[1][2] = -spl*ly;
        Tpl[2][0] = 0.0; Tpl[2][1] = 0.0; Tpl[2][2] = 1.0;
        
        // inverse similarity transformation of the image
        Timinv[0][0] = 1.0/spr; Timinv[0][1] =   0.0  ; Timinv[0][2] = rx;
        Timinv[1][0] =   0.0  ; Timinv[1][1] = 1.0/spr; Timinv[1][2] = ry;
        Timinv[2][0] =   0.0  ; Timinv[2][1] =   0.0  ; Timinv[2][2] = 1.0;
        
        
        ///////////////// System matrix
        
        laMatrix A(2*n,9);
        for(int i=0, eq=0; i < n; i++, eq++) {
            
            float	xpl = px0[i], ypl = py0[i],
            xim = px1[i], yim = py1[i];
            
            A[eq][0] = A[eq][1] = A[eq][2] = 0.0;
            A[eq][3] = -xpl;
            A[eq][4] = -ypl;
            A[eq][5] = -1.0;
            A[eq][6] =  yim * xpl;
            A[eq][7] =  yim * ypl;
            A[eq][8] =  yim;
            
            eq++;
            
            A[eq][0] =  xpl;
            A[eq][1] =  ypl;
            A[eq][2] =  1.0;
            A[eq][3] = A[eq][4] = A[eq][5] = 0.0;
            A[eq][6] = -xim * xpl;
            A[eq][7] = -xim * ypl;
            A[eq][8] = -xim;
        }
        
        
        ///////////////// SVD
        laMatrix U(2*n,9), V(9,9);
        laVector W(9);
        
        compute_svd_double(A,U,V,W);
        
        
        // Find the index of the least singular value
        int imin = 0;
        for (int i=1; i < 9; i++)
        if ( W[i] < W[imin] ) imin = i;
        
        
        ////////////////// Denormalize H = Timinv * V.col(imin)* Tpl;
        laMatrix matrix(3,3), result(3,3);
        
        
        int k=0;
        for(int i=0; i < 3; i++)
        for(int j=0; j < 3; j++, k++)
        matrix[i][j] = V[k][imin];
        
        
        result = Timinv * matrix;
        H = result * Tpl;
        
        
        
        delete[] px0;
        delete[] py0;
        delete[] px1;
        delete[] py1;
        
        
    }
    
    
    
    
    
    /// Compute homography using svd + Ransac
    int compute_ransac_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, int niter, float tolerance, laMatrix &H, int *accorded, int inewSVD)
    {
        
        
        // Initialize seed
        srand48( (long int) time (NULL) + (long int) getpid() );
        float tolerance2 = tolerance * tolerance;
        
        
        H.create(3,3);
        laMatrix Haux(3,3);
        
        
        
        int *paccorded = new int[n];
        int naccorded = 0;
        int pnaccorded = 0;
        
        for(int iter = 0; iter < niter; iter++)
        {
            
            
            // Choose 4 indexos from 1..n without repeated values
            int indexos[4];
            int acceptable = 0;
            while (!acceptable)
            {
                acceptable = 1;
                for(int i=0; i < 4; i++) indexos[i] = (int)  floor(drand48() * (float) n);
                
                // Check if indexos are repeated
                for(int i=0; i < 4; i++)
                for(int j=i+1; j < 4; j++)
                if (indexos[i] == indexos[j]) acceptable = 0;
            }
            
            
            
            
            
            // Store selected matches
            float px0[4] , py0[4], px1[4] , py1[4];
            for(int i=0; i < 4; i++)
            {
                px0[i] = x0[indexos[i]];
                py0[i] = y0[indexos[i]];
                
                px1[i] = x1[indexos[i]];
                py1[i] = y1[indexos[i]];
            }
            
            
            // Compute planar homography
            
            if (inewSVD)
                compute_planar_homography_n_points_new_svd(px0, py0, px1, py1, 4, Haux);
            else
                compute_planar_homography_n_points(px0, py0, px1, py1, 4, Haux);
            
            
            // Which matches are according to this transformation
            pnaccorded = 0;
            for(int i=0; i < n; i++)
            {
                
                laVector vec(3), res(3);
                vec[0] = x0[i];
                vec[1] = y0[i];
                vec[2] = 1.0f;
                
                res = Haux * vec;
                
                if (res[2] != 0.0f) {
                    
                    res[0] /= res[2]; res[1] /= res[2];
                    
                    float dif = (res[0] - x1[i]) * (res[0] - x1[i]) + (res[1] - y1[i]) * (res[1] - y1[i]);
                    
                    if (dif < tolerance2) {  paccorded[pnaccorded] = i; pnaccorded++; }
                    
                }
                
            }
            
            
            // printf("%d: %d (%d) \n", iter, pnaccorded, naccorded);
            
            // if more according points --> save positions
            if (pnaccorded > naccorded)
            {
                
                naccorded = pnaccorded;
                
                if (accorded != NULL)
                for(int i=0; i < naccorded; i++) accorded[i] = paccorded[i];
                
                H = Haux;
                
                
            }
            
        }
        
        
        delete[] paccorded;
        return naccorded;
        
    }
    
    
    
    int compute_ransac_planar_homography_n_points(float *x0, float *y0, float *x1, float *y1, int n, int niter, float tolerance, laMatrix &H, int inewSVD)
    {
        
        return  compute_ransac_planar_homography_n_points(x0, y0, x1, y1, n, niter, tolerance, H, NULL, inewSVD);
        
    }
    
    
    
    
    
  
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#define c_re(c) ((c)[0])
#define c_im(c) ((c)[1])
    
    
    void FFT_init (void) {
        FILE *wisdom_file;
        
        
        wisdom_file = fopen("/tmp/.fftwisdom","r");
        if (wisdom_file == NULL)
        fprintf(stderr, "Generating wisdom file...\n");
        else {
            int res;
            res = fftwf_import_wisdom_from_file(wisdom_file);
            fclose (wisdom_file);
        }
        
    }
    
    void FFT_end (void) {
        FILE *wisdom_file;
        
        // export wisdom for later use
        wisdom_file = fopen("/tmp/.fftwisdom","w");
        fftwf_export_wisdom_to_file(wisdom_file);
        fclose (wisdom_file);
    }
    
    
    // compute the inverse FFT and normalizes.
    void FFT_2d_inv (fftwf_complex* in, fftwf_complex* out, int nx, int ny)
    {
        fftwf_plan p;
        int i;
        int numpix = nx*ny;
        
        p = fftwf_plan_dft_2d(ny,nx, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftwf_execute(p);
        fftwf_destroy_plan(p);
        // normalize
        for(i=0; i<numpix;i++) { c_re(out[i]) /= numpix; c_im(out[i]) /= numpix; }
        
    }
    
    // compute the forward FFT.
    void FFT_2d_fwd (fftwf_complex* in,fftwf_complex* out, int nx, int ny)
    {
        fftwf_plan p;
        
        p = fftwf_plan_dft_2d(ny,nx, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
        fftwf_execute(p);
        fftwf_destroy_plan(p);
    }
    
    // compute the inverse FFT and normalizes.
    void FFT_1d_inv (fftwf_complex* in, fftwf_complex* out, int nx)
    {
        fftwf_plan p;
        int i;
        int numpix = nx;
        
        p = fftwf_plan_dft_1d(nx, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftwf_execute(p);
        fftwf_destroy_plan(p);
        // normalize
        for(i=0; i<numpix;i++) { c_re(out[i]) /= numpix; c_im(out[i]) /= numpix; }
        
    }
    
    // compute the forward FFT.
    void FFT_1d_fwd (fftwf_complex* in,fftwf_complex* out, int nx)
    {
        fftwf_plan p;
        
        p = fftwf_plan_dft_1d(nx, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
        fftwf_execute(p);
        fftwf_destroy_plan(p);
    }
    
    
    // 1d fftw
    void fft1d(float *in_re,float *in_im,float *out_re,float *out_im, int i_flag, int nx) {
        int i;
        fftwf_complex *in,*out;
        
        if ((!out_re) && (!out_im))
        {
            fprintf(stderr,"At least one output needed\n");
            return;
        }
        
        
        /***** allocate images *****/
        in  = new fftwf_complex[nx];
        out = new fftwf_complex[nx];
        
        /***** copy from input *****/
        if (!in_re) {
            fprintf(stderr,"No real part!?\n");
            return;
        }
        
        /* real part */
        for(i=0; i<nx;i++) {
            c_re(in[i]) = in_re[i];
            c_im(in[i]) = 0;
        }
        /* imaginary part (if present) */
        if ( in_im )
        for(i=0; i<nx;i++) {
            c_im(in[i]) = in_im[i];
        }
        
        /***** transform *****/
        //   FFT_init();
        
        if(!i_flag)
        FFT_1d_fwd (in, out, nx); /* F in */
        else
        FFT_1d_inv (in, out, nx); /* F^-1 in */
        
        //   FFT_end();
        
        /***** allocations *****/
        
        if(out_re)
        for(i=0; i<nx;i++)
        out_re[i] = c_re(out[i]);
        if(out_im)
        for(i=0; i<nx;i++)
        out_im[i] = c_im(out[i]);
        
        
        /***** free temporary *****/
        delete[] in;
        delete[] out;
        
    }
    
    
    
    // NEW FFT2d USES FFTW
    // 2d fftw
    //in_re, out_re and out_img has memory
    void fft2d(float *in_re, float *in_im,float *out_re,float *out_im,int i_flag, int width, int height)
    {
        int nx,ny,i;
        fftwf_complex *in,*out;
        
        nx = width;
        ny = height;
        
        if ((!out_re) && (!out_im))
        {
            fprintf(stderr,"At least one output needed\n");
            return;
        }
        
        
        /***** allocate images *****/
        in  = new fftwf_complex[nx*ny];
        out = new fftwf_complex[nx*ny];
        
        /***** copy from input *****/
        if (!in_re) {
            fprintf(stderr,"No real part!?\n");
            return;
        }
        
        /* real part */
        for(i=0; i<nx*ny;i++) {
            c_re(in[i]) = in_re[i];
            c_im(in[i]) = 0;
        }
        /* imaginary part (if present) */
        if ( in_im )
        for(i=0; i<nx*ny;i++) {
            c_im(in[i]) = in_im[i];
        }
        
        /***** transform *****/
        //   FFT_init();
        
        if(!i_flag)
        FFT_2d_fwd (in, out, nx, ny); /* F in */
        else
        FFT_2d_inv (in, out, nx, ny); /* F^-1 in */
        
        //   FFT_end();
        
        /***** allocations *****/
        
        if(out_re)
        for(i=0; i<nx*ny;i++)
        out_re[i] = c_re(out[i]);
        if(out_im)
        for(i=0; i<nx*ny;i++)
        out_im[i] = c_im(out[i]);
        
        
        /***** free temporary *****/
        delete[] in;
        delete[] out;
        
    }
    
    
    
    
    void fiFFTZoom(float *in, float *out, float zoomx, float zoomy, int width, int height)
    {
        
        
        
        assert((zoomx >= 1. && zoomy >= 1.) || (zoomx < 1. && zoomy < 1.) );
        
        int swidth = width / 2 + 1;
        
        
        // Perform FFT
        // FFTW are the left half image of the mw coefficients
        // MW coefficients are placed
        //								x y
        //								y x
        
        
        fftwf_complex  *fft_out;
        fftwf_plan p;
        
        fft_out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * (width/2+1) * height);
        
        p =  fftwf_plan_dft_r2c_2d(height, width, in, fft_out, FFTW_ESTIMATE);
        
        fftwf_execute(p);
        
        
        
        // Padding of FFT output
        int widthz = (int)rintf( (float) width *  (float) zoomx);
        int heightz = (int)rintf( (float) height *  (float) zoomy);
        
        int swidthz = (widthz / 2 + 1);
        
        
        fftwf_complex *fft_pout;
        fft_pout = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * (widthz/2+1) * heightz);
        
        
        
        for (int i=0; i < swidthz * heightz ; i++)
        {
            fft_pout[i][0] = 0.0f;
            fft_pout[i][1] = 0.0f;
        }
        
        
        
        if (zoomx<1. && zoomy<1.) {
            
            for (int x= 0 ; x < swidthz ;x++)
            for (int y=-heightz/2 ; y<= heightz/2;y++)
            {
                
                int adr  = x + ((y+ height )%height ) * swidth ;
                int adrz = x + ((y+ heightz)%heightz) * swidthz;
                
                float factor = zoomx * zoomy;
                
                fft_pout[adrz][0] = fft_out[adr][0] * factor;
                fft_pout[adrz][1] = fft_out[adr][1] * factor;
                
                
            }
            
            
        } else if (zoomx >= 1.0 && zoomy >= 1.0){
            
            
            int adr, adrz;
            for (int x=0; x < swidth ; x++)
            for (int y = -height/2; y <= height/2; y++)
            {
                adr = x + ( (y + height)% height ) * swidth ;
                adrz = x + ( (y + heightz) % heightz ) * swidthz ;
                
                float factor = zoomx * zoomy
                * ((2*y==height || 2*y==-height)?0.5:1.0);
                
                fft_pout[adrz][0] = fft_out[adr][0] * factor;
                fft_pout[adrz][1] = fft_out[adr][1] * factor;
                
            }
            
            
            
        }
        
        
        
        
        //   Backward fft
        p =  fftwf_plan_dft_c2r_2d(heightz, widthz, fft_pout, out, FFTW_ESTIMATE);
        
        fftwf_execute(p);
        
        
        for (int i=0; i < widthz * heightz; i++) out[i] /= (float) (widthz * heightz);
        
        
        
        fftwf_destroy_plan(p);
        fftwf_free(fft_out);
        fftwf_free(fft_pout);
        
    }
    
    
    
    void fiFFTShearPascal(float dAmountOfShear, float *pFloatImageInput,float *pFloatImageOutput, int iAxis, float fDelta, int PleaseDoNotCenter, int width, int height)
    {
        int i, j, iSize, iHalfSize, iOtherSize, iHalfOtherSize;
        fftwf_complex *pmxsignal, *pmxsignalTransformed;
        float dTranslation;
        float fCos, fSin, fReal;
        int odd,oddOther;
        
        
        // iSize is the size of the image in the direction of the axis of shear.
        // iOtherSize the size of the image in the orthogonal direction
        if(iAxis == 0) // HORIZONTAL
        {
            iHalfSize = (iSize = width) >> 1;
            iHalfOtherSize = (iOtherSize = height) >> 1;
        }
        else
        {
            
            //iHalfSize = (iSize = width) >> 1;
            //iHalfOtherSize = (iOtherSize = height) >> 1;
            
            iHalfSize = (iSize = height) >> 1;
            iHalfOtherSize = (iOtherSize = width) >> 1;
            
        }
        
        if ((iOtherSize & 1)!= 0) oddOther=1; else oddOther=0;
        if ((iSize & 1) !=0) odd=1; else odd=0;
        
        // Create temporary mxsignals to compute the fourier transform of a line
        // (or a column) of the image
        pmxsignal = new fftwf_complex[iSize]; // mw_change_fmxsignal(0, iSize);
        pmxsignalTransformed  = new fftwf_complex[iSize]; //= mw_new_fmxsignal();
        
        fftwf_plan pbwd;
        fftwf_plan pfwd;
        
        // THERE IS A SMALL INPROVEMENT IN RECYCLING THE PLANS
        // Naming of forward and backward fft's are reversed wrt fftw
#pragma omp critical (make_plan)
        pfwd = fftwf_plan_dft_1d(iSize, pmxsignal, pmxsignalTransformed, FFTW_BACKWARD, FFTW_ESTIMATE);
        
#pragma omp critical (make_plan)
        pbwd= fftwf_plan_dft_1d(iSize, pmxsignalTransformed, pmxsignal, FFTW_FORWARD, FFTW_ESTIMATE);
        
        
        for(i = 0; i < iOtherSize; i++)
        {
            memset(pmxsignal, 0, iSize * sizeof(fftwf_complex));
            
            if(iAxis == 0) {// HORIZONTAL
                for(j = 0; j < iSize; j++)
                c_re(pmxsignal[j]) = pFloatImageInput[i * iSize + j];
            }
            else {
                for(j = 0; j < iSize; j++)
                c_re(pmxsignal[j]) = pFloatImageInput[j * iOtherSize + i];
            }
            
            // Compute the FFT of the current line (or column) of the image
            //      FFT_1d_fwd (pmxsignal,pmxsignalTransformed, iSize);
            fftwf_execute(pfwd);          // RECYCLING PLANS
            
            
            if (PleaseDoNotCenter)
            dTranslation = - (i * dAmountOfShear + fDelta) * 2. * M_PI;
            else
            if (oddOther)
            dTranslation = - ((i - iHalfOtherSize ) * dAmountOfShear + fDelta) * 2. * M_PI;
            else
            dTranslation = - ((i - iHalfOtherSize + .5) * dAmountOfShear + fDelta) * 2. * M_PI;
            
            for(j = 1; j < iHalfSize+odd; j++)
            {
                fCos = (float) cos(j * dTranslation / iSize);
                fSin = (float) sin(j * dTranslation / iSize);
                fReal = c_re(pmxsignalTransformed[j]);
                c_re(pmxsignalTransformed[j]) = fCos * fReal - fSin * c_im(pmxsignalTransformed[j]);
                c_im(pmxsignalTransformed[j]) = fSin * fReal + fCos * c_im(pmxsignalTransformed[j]);
                c_re(pmxsignalTransformed[iSize - j]) = c_re(pmxsignalTransformed[j]);
                c_im(pmxsignalTransformed[iSize - j]) = -c_im(pmxsignalTransformed[j]);
            }
            
            if (odd == 0) {
                c_re(pmxsignalTransformed[iHalfSize]) = cos(dTranslation * .5) * c_re(pmxsignalTransformed[iHalfSize]) - sin(dTranslation * .5) * c_im(pmxsignalTransformed[iHalfSize]);
                c_im(pmxsignalTransformed[iHalfSize]) = 0.;
            }
            
            // Compute the inverse FFT of the current line (or column)
            //FFT_1d_inv (pmxsignalTransformed, pmxsignal,iSize);
            fftwf_execute(pbwd); // RECYCLING PLANS
            for(j=0; j<iSize;j++) { c_re(pmxsignal[j]) /= iSize; c_im(pmxsignal[j]) /= iSize; }
            
            if(iAxis == 0) {// HORIZONTAL
                for(j = 0; j < iSize; j++)
                pFloatImageOutput[i * iSize + j] = c_re(pmxsignal[j]);
            }
            else {
                for(j = 0; j < iSize; j++)
                pFloatImageOutput[j * iOtherSize + i] = c_re(pmxsignal[j]);
            }
        }
        
        // Delete the previously allocated temporary mxsignals
        fftwf_destroy_plan(pfwd); // RECYCLING PLANS
        fftwf_destroy_plan(pbwd);
        delete[] pmxsignalTransformed;
        delete[] pmxsignal;
    }
    
    
    
    
    
    // Color Convolution
    void fft_Gaussian_convolution(float **convolved, float **data, float std, int num_channels, int width, int height)
    {
        //! Convolving each channel
        for(int k = 0; k < num_channels; k++)
        fft_convolution(convolved[k], data[k], std, width, height);
    }
    
    
    // Grayscale Convolution
    void fft_Gaussian_convolution(float *convolved, float *data, float std, int width, int height)
    {
        fft_convolution(convolved, data, std, width, height);
    }
    
    
    // Apply FFT Gaussian Convolution to one-dimensional data
    void fft_convolution(float *convolved, float *data, float std, int width, int height)
    {
        //! Image sizes
        int dim = width * height;
        int dim4 = 4 * dim;
        
        
        //! Double variables
        double d_std = (double) std;
        double *d_data = new double[dim];
        double *d_convolved = new double[dim];
        
        for(int i = 0; i < dim; i++)
        d_data[i] = (double) data[i];
        
        
        //! FFT variables
        fftw_plan p;
        double *output = (double*) fftw_malloc(dim * sizeof(double));
        
        
        //! Compute FFT of data
        p = fftw_plan_r2r_2d(height, width, d_data, output, FFTW_REDFT10, FFTW_REDFT10, FFTW_ESTIMATE);
        fftw_execute(p);
        fftw_destroy_plan(p);
        
        
        //! Define the gaussian constants for the convolution
        double scale = (d_std * d_std) / 2e0;
        double normx = M_PI / (double) width;
        normx *= normx;
        double normy = M_PI / (double) height;
        normy *= normy;
        
        
        //! Convolve FFT of data with the FFT of Gaussian kernel (which is itself)
        for(int j = 0; j < height; j++)
        for(int i = 0; i < width; i++)
        output[j*width+i] *= exp((double)(-scale) * (normx * i * i + normy * j * j));
        
        
        //! Compute the inverse FFT of output
        p = fftw_plan_r2r_2d(height, width, output, d_convolved, FFTW_REDFT01, FFTW_REDFT01, FFTW_ESTIMATE);
        fftw_execute(p);
        fftw_destroy_plan(p);
        
        for(int i = 0; i < dim; i++)
        d_convolved[i] /= (double) dim4;
        
        
        //! Compute float solution
        for(int i = 0; i < dim; i++)
        convolved[i] = (float) d_convolved[i];
        
        
        //! Free memory
        fftw_free(output);
        delete[] d_data;
        delete[] d_convolved;
    }
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //! Patch Statistics
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    void fiComputeIntegralImage(float *in, float *out, int width, int height)
    {
        
        
        
        float * tmp_im = new float[width * height];
        
        // recursivity in the rows
        
        for(int jj=0; jj < height; jj++)
        for(int ii=0; ii < width; ii++)
        {
            if  (ii==0) tmp_im[width*jj+ii] = in[width*jj+ii];
            else tmp_im[width*jj+ii] = tmp_im[width*jj+ii-1] + in[width*jj+ii];
        }
        
        // recursivity in the columns
        for(int ii=0; ii < width; ii++)
        for(int jj=0; jj < height; jj++)
        {
            if(jj==0) out[width*jj+ii] = tmp_im[width*jj+ii];
            else out[width*jj+ii] = out[width*(jj-1)+ii] + tmp_im[width*jj+ii];
        }
        
        
        delete[] tmp_im;
        
    }
    
    
  
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////
    //! Begin Spline Interpolation MEGAWAVE
    ////////////////////////////////////////////////////////////////////////////////
    
    
    
    //! extract image value (even outside image domain)
    float vMW(float *in,int x,int y,float bg, int width, int height)
    {
        if (x<0 || x>=width || y<0 || y>=height)
        return bg;
        else return(in[y*width+x]);
    }
    
    
    
    
    
    //! c[] = values of interpolation function at ...,t-2,t-1,t,t+1,...
    //! coefficients for cubic interpolant (Keys' function)
    void keysMW(float *c,float t,float a)
    {
        float t2,at;
        
        t2 = t*t;
        at = a*t;
        c[0] = a*t2*(1.0-t);
        c[1] = (2.0*a+3.0 - (a+2.0)*t)*t2 - at;
        c[2] = ((a+2.0)*t - a-3.0)*t2 + 1.0;
        c[3] = a*(t-2.0)*t2 + at;
    }
    
    
    
    //! Coefficients for cubic spline
    void spline3MW(float *c,float t)
    {
        float tmp;
        
        tmp = 1.-t;
        c[0] = 0.1666666666*t*t*t;
        c[1] = 0.6666666666-0.5*tmp*tmp*(1.+t);
        c[2] = 0.6666666666-0.5*t*t*(2.-t);
        c[3] = 0.1666666666*tmp*tmp*tmp;
    }
    
    
    
    
    
    
    //! Init spline n
    void init_splinenMW(float *a,int n)
    {
        int k;
        
        a[0] = 1.;
        for (k=2;k<=n;k++) a[0]/=(float)k;
        for (k=1;k<=n+1;k++)
        a[k] = - a[k-1] *(float)(n+2-k)/(float)k;
    }
    
    
    //! Fast integral power function
    float ipowMW(float x,int n)
    {
        float res;
        
        for (res=1.;n;n>>=1) {
            if (n&1) res*=x;
            x*=x;
        }
        return(res);
    }
    
    
    //! coefficients for spline of order >3
    void splinenMW(float *c,float t,float *a,int n)
    {
        int i,k;
        float xn;
        
        memset((void *)c,0,(n+1)*sizeof(float));
        for (k=0;k<=n+1;k++) {
            xn = ipowMW(t+(float)k,n);
            for (i=k;i<=n;i++)
            c[i] += a[i-k]*xn;
        }
    }
    
    
    
    
    //! called by invspline1d
    //! takes c as input and output some kind of sum combining c and z
    float initcausalMW(float *c,int n,float z)
    {
        float zk,z2k,iz,sum;
        int k;
        
        zk = z; iz = 1./z;
        z2k = pow(z,(float)n-1.);
        sum = c[0] + z2k * c[n-1];
        z2k = z2k*z2k*iz;
        for (k=1;k<=n-2;k++) {
            sum += (zk+z2k)*c[k];
            zk *= z;
            z2k *= iz;
        }
        return (sum/(1.-zk*zk));
    }
    
    
    
    //! called by invspline1d
    //! takes c as input and output some kind of sum combining c and z
    float initanticausalMW(float *c,int n,float z)
    {
        return((z/(z*z-1.))*(z*c[n-2]+c[n-1]));
    }
    
    
    
    //! called by finvspline
    void invspline1DMW(float *c,int size,float *z,int npoles)
    {
        float lambda;
        int n,k;
        
        /* normalization */
        for (k=npoles,lambda=1.;k--;) lambda *= (1.-z[k])*(1.-1./z[k]);
        for (n=size;n--;) c[n] *= lambda;
        
        /*----- Loop on poles -----*/
        for (k=0;k<npoles;k++) {
            
            /* forward recursion */
            c[0] = initcausalMW(c,size,z[k]);
            for (n=1;n<size;n++)
            c[n] += z[k]*c[n-1];
            
            /* backwards recursion */
            c[size-1] = initanticausalMW(c,size,z[k]);
            for (n=size-1;n--;)
            c[n] = z[k]*(c[n+1]-c[n]);
            
        }
    }
    
    
    
    //! Main function filling coefficients
    void finvsplineMW(float *in,int order,float *out, int width, int height)
    {
        float *c,*d,z[5];
        int npoles,nx,ny,x,y;
        
        ny = height; nx = width;
        
        /* initialize poles of associated z-filter */
        switch (order)
        {
                case 2: z[0]=-0.17157288;  /* sqrt(8)-3 */
                break;
                
                case 3: z[0]=-0.26794919;  /* sqrt(3)-2 */
                break;
                
                case 4: z[0]=-0.361341; z[1]=-0.0137254;
                break;
                
                case 5: z[0]=-0.430575; z[1]=-0.0430963;
                break;
                
                case 6: z[0]=-0.488295; z[1]=-0.0816793; z[2]=-0.00141415;
                break;
                
                case 7: z[0]=-0.53528; z[1]=-0.122555; z[2]=-0.00914869;
                break;
                
                case 8: z[0]=-0.574687; z[1]=-0.163035; z[2]=-0.0236323; z[3]=-0.000153821;
                break;
                
                case 9: z[0]=-0.607997; z[1]=-0.201751; z[2]=-0.0432226; z[3]=-0.00212131;
                break;
                
                case 10: z[0]=-0.636551; z[1]=-0.238183; z[2]=-0.065727; z[3]=-0.00752819;
                z[4]=-0.0000169828;
                break;
                
                case 11: z[0]=-0.661266; z[1]=-0.27218; z[2]=-0.0897596; z[3]=-0.0166696;
                z[4]=-0.000510558;
                break;
                
            default:
                printf("finvspline: order should be in 2..11.\n");
                exit(-1);
        }
        
        npoles = order/2;
        
        /* initialize float array containing image */
        c = (float *)malloc(nx*ny*sizeof(float));
        d = (float *)malloc(nx*ny*sizeof(float));
        for (x=nx*ny;x--;)
        c[x] = (float)in[x];
        
        /* apply filter on lines */
        for (y=0;y<ny;y++)
        invspline1DMW(c+y*nx,nx,z,npoles);
        
        /* transpose */
        for (x=0;x<nx;x++)
        for (y=0;y<ny;y++)
        d[x*ny+y] = c[y*nx+x];
        
        /* apply filter on columns */
        for (x=0;x<nx;x++)
        invspline1DMW(d+x*ny,ny,z,npoles);
        
        /* transpose directy into image */
        for (x=0;x<nx;x++)
        for (y=0;y<ny;y++)
        out[y*nx+x] = (float)(d[x*ny+y]);
        
        /* free array */
        free(d);
        free(c);
    }
    
    
    
    float evaluate_splineMW(float *input, float *ref, float xp, float yp, float *ak, int order, float bg, int width, int height)
    {
        
        float  cx[12],cy[12];
        float p=-0.5;
        
        int xi = (int)floor((float)xp);
        int yi = (int)floor((float)yp);
        
        float res;
        int n1, n2;
        
        if (order == 0)
        {
            if (xi<0 || xi>=width || yi<0 || yi>=height)        res = bg;
            else res = input[yi*width+xi];
            
        } else
        {
            
            
            if (xp<0. || xp>=(float)width || yp<0. || yp>=(float)height) res=bg;
            else {
                
                //xp -= 0.5; yp -= 0.5;
                
                float ux = xp-(float)xi;
                float uy = yp-(float)yi;
                
                switch (order)
                {
                        case 1:
                        n2 = 1;
                        cx[0]=ux; cx[1]=1.-ux;
                        cy[0]=uy; cy[1]=1.-uy;
                        break;
                        
                        case -3:
                        n2 = 2;
                        keysMW(cx,ux,p);
                        keysMW(cy,uy,p);
                        break;
                        
                        case 3:
                        n2 = 2;
                        spline3MW(cx,ux);
                        spline3MW(cy,uy);
                        break;
                        
                    default:
                        n2 = (1+order)/2;
                        splinenMW(cx,ux,ak,order);
                        splinenMW(cy,uy,ak,order);
                        break;
                }
                
                res = 0.; n1 = 1-n2;
                if (xi+n1>=0 && xi+n2<width && yi+n1>=0 && yi+n2<height) {
                    
                    int adr = yi*width+xi;
                    for (int dy=n1;dy<=n2;dy++)
                    for (int dx=n1;dx<=n2;dx++)
                    res += cy[n2-dy]*cx[n2-dx]*ref[adr+width*dy+dx];
                } else
                
                for (int dy=n1;dy<=n2;dy++)
                for (int dx=n1;dx<=n2;dx++)
                res += cy[n2-dy]*cx[n2-dx]*vMW(ref,xi+dx,yi+dy,bg,width,height);
                
            }
            
            
        }
        
        
        return res;
        
        
    }
    
    
    ////////////////////////////////////////////////////////////////////////////////
    //! End Spline Interpolation
    ////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    
    
    void apply_planar_homography(float *input, int width, int height, laMatrix &H, float bg, int order, float *out, float x0, float y0, int nwidth, int nheight)
    {
        
        
        //! Inverse matrix
        laMatrix V(3,3);
        luinv(H, V);
        
        
        //! Spline interpolation
        float *coeffs;
        float *ref;
        
        float  ak[13];
        
        if (order!=0 && order!=1 && order!=-3 &&
            order!=3 && order!=5 && order!=7 && order!=9 && order!=11)
        {
            printf("unrecognized interpolation order.\n");
            exit(-1);
        }
        
        if (order>=3) {
            
            coeffs = new float[width*height];
            finvsplineMW(input,order,coeffs,width,height);
            ref = coeffs;
            if (order>3) init_splinenMW(ak,order);
            
        } else
        {
            coeffs = NULL;
            ref = input;
        }
        
        
        //! For each point in new image we compute its anti image and interpolate the new value
        for(int i=0; i < nwidth; i++)
        for(int j=0; j < nheight; j++)
        {
            
            
            //! Compute transformed vector
            laVector vec(3), vres(3);
            vec[0] = (float) i + x0;
            vec[1] = (float) j + y0;
            vec[2] = 1.0f;
            
            vres = V * vec;
            
            
            
            
            if (vres[2] != 0.0f)
            {
                
                vres[0] /= vres[2]; vres[1] /= vres[2];
                
                float xp =  (float) vres[0];
                float yp =  (float) vres[1];
                
                float res =  evaluate_splineMW(input,  ref,   xp,   yp, &ak[0],   order,   bg,   width,   height);
                
                out[j*nwidth+i] = res;
                
            } else
            {
                out[j*nwidth+i] = bg;
            }
            
            
        }
        
        if (coeffs != NULL )  delete[] coeffs;
        
    }
    
    
    
    
    
    
    void apply_planar_homography_zoom(float *input, int width, int height, laMatrix &H, float bg, int order, float *out, float x0, float y0, int nwidth, int nheight, float fZoom)
    {
        
        
        //! Inverse matrix
        laMatrix V(3,3);
        luinv(H, V);
        
        
        //! Spline interpolation
        float *coeffs;
        float *ref;
        
        float  ak[13];
        
        if (order!=0 && order!=1 && order!=-3 &&
            order!=3 && order!=5 && order!=7 && order!=9 && order!=11)
        {
            printf("unrecognized interpolation order.\n");
            exit(-1);
        }
        
        if (order>=3) {
            
            coeffs = new float[width*height];
            finvsplineMW(input,order,coeffs,width,height);
            ref = coeffs;
            if (order>3) init_splinenMW(ak,order);
            
        } else
        {
            coeffs = NULL;
            ref = input;
        }
        
        
        //! For each point in new image we compute its anti image and interpolate the new value
        for(int i=0; i < nwidth; i++)
        for(int j=0; j < nheight; j++)
        {
            
            
            
            //! Compute transformed vector
            laVector vec(3), vres(3);
            vec[0] = (float) i / fZoom + x0;
            vec[1] = (float) j / fZoom + y0;
            vec[2] = 1.0f;
            
            vres = V * vec;
            
            
            if (vres[2] != 0.0f)
            {
                
                vres[0] /= vres[2]; vres[1] /= vres[2];
                
                float xp =  (float) vres[0];
                float yp =  (float) vres[1];
                
                float res =  evaluate_splineMW(input,  ref,   xp,   yp, &ak[0],   order,   bg,   width,   height);
                
                out[j*nwidth+i] = res;
                
            } else
            {
                out[j*nwidth+i] = bg;
            }
            
            
        }
        
        if (coeffs != NULL) delete[] coeffs;
        
        
    }
    
    
    
    
    void spline_interpolation_zoom(
                                   float *input,     // image to be warped
                                   const int    nx,        // image width
                                   const int    ny,        // image height
                                   const float  fFactor,   // zoom factor
                                   const int    order,     // order of interpolation
                                   const float bvalue,     // value outside the region
                                   float       *output     // image warped with bicubic interpolation
    )
    {
        
        
        //! Spline interpolation
        float *coeffs;
        float *ref;
        
        float  ak[13];
        
        if (order!=0 && order!=1 && order!=-3 &&
            order!=3 && order!=5 && order!=7 && order!=9 && order!=11)
        {
            printf("unrecognized interpolation order.\n");
            exit(-1);
        }
        
        if (order>=3) {
            
            coeffs = new float[nx*ny];
            finvsplineMW(input,order,coeffs,nx,ny);
            ref = coeffs;
            if (order>3) init_splinenMW(ak,order);
            
        } else
        {
            coeffs = NULL;
            ref = input;
        }
        
        
        
        
        int znx = (int)rintf( fFactor * (float) nx);
        int zny = (int)rintf( fFactor * (float) ny);
        
        for(int i = 0; i < znx; i++)
        for(int j = 0; j < zny; j++)
        {
            float uu = (float) i / fFactor;
            float vv = (float) j / fFactor;
            
            
            // obtain the bicubic interpolation at position (uu, vv)
            output[j * znx + i] = evaluate_splineMW(input,  ref,   uu,  vv, &ak[0],   order,   bvalue,   nx,   ny);
            
        }
    }
    
    
    
    
   
    
    
    
    
    
    ////////////////////////////////////////////////////////////////////////////////
    //! BEGIN Numerics
    ////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    
    
    
    laVector::laVector() : d_n(0), d_v(0) {}
    
    
    
    laVector::laVector(int n) : d_n(n), d_v(new float[n]) {}
    
    
    laVector::laVector(const float& a, int n) : d_n(n), d_v(new float[n])
    {
        for(int i=0; i<n; i++)
        d_v[i] = a;
    }
    
    
    laVector::laVector(const float *a, int n) : d_n(n), d_v(new float[n])
    {
        for(int i=0; i<n; i++)
        d_v[i] = *a++;
    }
    
    
    laVector::laVector(const laVector &rhs) : d_n(rhs.d_n), d_v(new float[d_n])
    {
        for(int i=0; i<d_n; i++)
        d_v[i] = rhs[i];
    }
    
    
    laVector::laVector(const char * filename) : d_n(0), d_v(0)
    {
        
        //filename = new char[128];
        //strcpy(filename,ifilename);

        std::ifstream file(filename);
        if (!file.is_open())
        {
            printf("%s file not found or impossible to open\n", filename);
            exit(-1);
        }
        
        file >> d_n;
        d_v = new float[d_n];
        for(int i = 0; i < d_n ; i++ )
            file >> d_v[i];
    }

    
    
    
    laVector & laVector::operator=(const laVector &rhs)
    {
        if (this != &rhs)
        {
            if (d_n != rhs.d_n) {
                if (d_v != 0) delete [] d_v;
                d_n =rhs.d_n;
                d_v = new float[d_n];
            }
            
            for (int i=0; i<d_n; i++)
            d_v[i]=rhs[i];
        }
        return *this;
    }
    
    
    laVector & laVector::operator=(const float &a)	//assign a to every element
    {
        for (int i=0; i<d_n; i++)
        d_v[i]=a;
        return *this;
    }
    
    
    float & laVector::operator[](const int i)	//subscripting
    {
        return d_v[i];
    }
    
    
    const float & laVector::operator[](const int i) const	//subscripting
    {
        return d_v[i];
    }
    
    
    float * laVector::v() {return d_v;}
    
    
    
    int laVector::size() const
    {
        return d_n;
    }
    
    
    laVector::~laVector()
    {
        if (d_v != 0)
        delete[] d_v;
    }
    
    
    void laVector::erase()
    {
        d_n = 0;
        if (d_v) delete[] d_v;
        d_v=0;
    }
    
    
    void laVector::create(int n)
    {
        erase();
        d_n = n;
        d_v = new float[d_n];
        for (int ii=0; ii < d_n; ii++) d_v[ii] = 0.0f;
    }

    
    
    laVector laVector::copyBlock(int i0, int length)
    {
        
        laVector block(length);
        for (int ii=0; ii < length; ii++)  block.d_v[ii] = d_v[i0+ii];
        
        return block;
    }
    
    
    void laVector::sort(int decreasing_order)
    {

        if (decreasing_order)
            qsort(d_v, d_n, sizeof(float), order_float_decreasing);
        else
            qsort(d_v, d_n, sizeof(float), order_float_increasing);

    }
    
    
    
    
    
    
    laMatrix::laMatrix() : d_n(0), d_m(0), d_v(0) {}
    
    
    laMatrix::laMatrix(int n, int m) : d_n(n), d_m(m), d_v(new float*[n])
    {
        d_v[0] = new float[m*n];
        for (int i=1; i< n; i++)
        d_v[i] = d_v[i-1] + m;
    }
    
    
    
    void laMatrix::create(int n, int m)
    {
        
        if (d_v != 0) {
            delete[] d_v[0];
            delete[] d_v;
        }
        
        d_n = n;
        d_m = m;
        
        d_v = new float*[n];
        d_v[0] = new float[m*n];
        for (int i=1; i< n; i++)
        d_v[i] = d_v[i-1] + m;
    }
    
    
    
    
    
    laMatrix::laMatrix(const float &a, int n, int m) : d_n(n), d_m(m), d_v(new float*[n])
    {
        int i,j;
        d_v[0] = new float[m*n];
        for (i=1; i< n; i++)
        d_v[i] = d_v[i-1] + m;
        for (i=0; i< n; i++)
        for (j=0; j<m; j++)
        d_v[i][j] = a;
    }
    
    
    laMatrix::laMatrix(const float *a, int n, int m) : d_n(n), d_m(m), d_v(new float*[n])
    {
        int i,j;
        d_v[0] = new float[m*n];
        for (i=1; i< n; i++)
        d_v[i] = d_v[i-1] + m;
        for (i=0; i< n; i++)
        for (j=0; j<m; j++)
        d_v[i][j] = *a++;
    }
    
    
    laMatrix::laMatrix(const laMatrix &rhs) : d_n(rhs.d_n), d_m(rhs.d_m), d_v(new float*[rhs.d_n])
    {
        assert(d_m>0 && d_n>0);
        
        d_v[0] = new float[d_m * d_n];
        
        for (int i=1; i< d_n; i++)
        d_v[i] = d_v[i-1] + d_m;
        
        for (int i=0; i< d_n; i++)
        for (int j=0; j<d_m; j++)
        d_v[i][j] = rhs[i][j];
    }
    
    
    laMatrix & laMatrix::operator=(const laMatrix &rhs)
    {
        if (this != &rhs) {
            int i,j;
            if (d_n != rhs.d_n || d_m != rhs.d_m) {
                if (d_v != 0) {
                    delete[] (d_v[0]);
                    delete[] (d_v);
                }
                
                d_n=rhs.d_n;
                d_m=rhs.d_m;
                d_v = new float*[d_n];
                d_v[0] = new float[d_m*d_n];
                
                
            }
            
            
            for (i=1; i< d_n; i++)
            d_v[i] = d_v[i-1] + d_m;
            for (i=0; i< d_n; i++)
            for (j=0; j<d_m; j++)
            d_v[i][j] = rhs[i][j];
        }
        return *this;
    }
    
    
    laMatrix & laMatrix::operator=(const float &a)	//assign a to every element
    {
        for (int i=0; i< d_n; i++)
        for (int j=0; j<d_m; j++)
        d_v[i][j] = a;
        return *this;
    }
    
    
    
    float* laMatrix::operator[](const int i)	//subscripting: pointer to row i
    {
        return d_v[i];
    }
    
    
    const float* laMatrix::operator[](const int i) const
    {
        return d_v[i];
    }
    
    
    float ** laMatrix::v() {return d_v;}
    
    
    
    
    
    int laMatrix::nrows() const
    {
        return d_n;
    }
    
    
    int laMatrix::ncols() const
    {
        return d_m;
    }
    
    
    laMatrix::~laMatrix()
    {
        if (d_v != 0) {
            delete[] (d_v[0]);
            delete[] (d_v);
        }
    }
    
    
    
    
    
    
    
    laMatrix operator*(float a, const laMatrix& rhs)
    {
        
        laMatrix r(rhs.d_n, rhs.d_m);
        
        for (int k=r.d_n * r.d_m - 1; k>=0; k--)
        r.d_v[0][k] = a * rhs.d_v[0][k];
        
        return r;
    }
    
    
    
    laMatrix operator/(const laMatrix& lhs, float a)
    {
        laMatrix r(lhs.d_n, lhs.d_m);
        
        for (int k=r.d_n * r.d_m - 1; k>=0; k--)
        r.d_v[0][k] = lhs.d_v[0][k] / a;
        return r;
        
    }
    
    
    
    laMatrix operator+(const laMatrix& lhs, const laMatrix& rhs)
    {
        laMatrix r(lhs.d_n, lhs.d_m);
        for (int k=r.d_m * r.d_n -1; k>=0; k--)
        r.d_v[0][k] = lhs.d_v[0][k] + rhs.d_v[0][k];
        return r;
    }
    
    
    
    laMatrix operator-(const laMatrix& lhs, const laMatrix& rhs)
    {
        laMatrix r(lhs.d_m, lhs.d_n);
        for (int k=r.d_m * r.d_n - 1; k>=0; k--)
        r.d_v[0][k] = lhs.d_v[0][k] - rhs.d_v[0][k];
        return r;
    }
    
    
    
    laMatrix operator*(const laMatrix& lhs, const laMatrix& rhs)
    {
        float aux;
        
        laMatrix r(lhs.d_n, rhs.d_m);
        for (int i=0; i< r.d_n; i++)
        for (int j=0; j< r.d_m; j++)
        {
            aux = 0.0;
            for (int k=0; k< lhs.d_m; k++)
            aux += lhs.d_v[i][k] * rhs.d_v[k][j];
            
            r.d_v[i][j] = aux;
        }
        
        return r;
    }
    
    
    
    laVector operator*(const laMatrix& lhs, const laVector& rhs)
    {
        
        
        laVector r(lhs.d_n);
        for (int i=0; i < r.size(); i++)
        {
            
            r[i] = 0;
            for (int k=0; k< rhs.size(); k++)
            {   r[i] += lhs.d_v[i][k] * rhs[k];}
        }
        
        return r;
    }
    
    
    
    laMatrix laMatrix::transposed()
    {
        laMatrix r(d_m, d_n);
        
        for (int ii=0; ii < d_m; ii++)
        for (int jj=0; jj < d_n; jj++)
        {
            r.d_v[ii][jj] = d_v[jj][ii];
        }
        
        return r;
    }
    
    
    
    
    
    
    laMatrix laMatrix::copyBlock(int i0, int j0, int rowb, int colb)
    {
        
        laMatrix block(rowb, colb);
        
        for (int i=0; i < rowb; i++)
        for (int j=0; j < colb; j++)
        block.d_v[i][j] = d_v[i0+i][j0+j];
        
        return block;
    }
    
    
    
    void ludcmp(laMatrix &a, laVector &indx, float &d)
    {
        
        
        const float TINY=1.0e-20;
        int i,j,k,imax;
        float big,dum,sum,temp;
        
        int n=a.nrows();
        laVector vv(n);
        d=1.0;
        
        for (i=0;i<n;i++) {
            big=0.0;
            for (j=0;j<n;j++)
            if ((temp=fabs(a[i][j])) > big) big=temp;
            if (big == 0.0) {printf("Singular matrix in routine ludcmp"); exit(-1);}
            vv[i]=1.0/big;
        }
        for (j=0;j<n;j++) {
            for (i=0;i<j;i++) {
                sum=a[i][j];
                for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
                a[i][j]=sum;
            }
            big=0.0;
            for (i=j;i<n;i++) {
                sum=a[i][j];
                for (k=0;k<j;k++) sum -= a[i][k]*a[k][j];
                a[i][j]=sum;
                if ((dum=vv[i]*fabs(sum)) >= big) {
                    big=dum;
                    imax=i;
                }
            }
            if (j != imax) {
                for (k=0;k<n;k++) {
                    dum=a[imax][k];
                    a[imax][k]=a[j][k];
                    a[j][k]=dum;
                }
                d = -d;
                vv[imax]=vv[j];
            }
            indx[j]=(float) imax;
            if (a[j][j] == 0.0) a[j][j]=TINY;
            if (j != n-1) {
                dum=1.0/(a[j][j]);
                for (i=j+1;i<n;i++) a[i][j] *= dum;
            }
        }
    }
    
    
    /* Solves the set of n linear equations Ax=b. Here a[0..n-1][0..n-1] as input, not as the matrix A but rather as its LU decomposition,*/
    /* determined by the routine ludcmp. indx[0..n-1] is input as the permutation vector returned by ludcmp. b[0..n-1] is input as the    */
    /* right hand side vector and returns with the solution vector x. */
    void lubksb(laMatrix &a, laVector &indx, laVector &b)
    {
        int i,ii=0,ip,j;
        float sum;
        
        int n=a.nrows();
        for (i=0;i<n;i++) {
            ip = (int) indx[i];
            sum=b[ip];
            b[ip]=b[i];
            if (ii != 0)
            for (j=ii-1;j<i;j++) sum -= a[i][j]*b[j];
            else if (sum != 0.0)
            ii=i+1;
            b[i]=sum;
        }
        for (i=n-1;i>=0;i--) {
            sum=b[i];
            for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
            b[i]=sum/a[i][i];
        }
    }
    
    
    
    // Solves Ax=b by using lu decomposition
    void lusolve(laMatrix &a, laVector &x, laVector &b)
    {
        
        int n = b.size();
        
        laVector indx(n);
        laMatrix Aux = a;
        
        float d;
        ludcmp(Aux,indx,d);
        
        
        x=b;
        lubksb(Aux,indx,x);
        
    }
    
    
    
    
    // Computes the inverse of a column by column using the LU decomposition
    void luinv(laMatrix &a, laMatrix &inv)
    {
        
        
        int n = a.nrows();
        laVector col(n), indx(n);
        
        laMatrix invaux=a;
        inv=a;
        
        float d;
        ludcmp(invaux,indx,d);
        
        
        for(int j=0; j<n; j++)
        {
            
            for(int i=0; i<n; i++)  col[i]=0.0;
            col[j]=1.0;
            
            lubksb(invaux,indx,col);
            
            for(int i=0;i<n;i++) {inv[i][j] = col[i];}
            
            
        }
        
    }
    
    
    
    void invCC(laMatrix &inv)
    {
        
        float z,*p,*q,*r,*s,*t;
        int n= inv.nrows();
        int j,k;
        float *v = inv[0];
        
        for(j=0,p=v; j<n ;++j,p+=n+1){
            for(q=v+j*n; q<p ;++q) *p-= *q* *q;
            
            if(*p<=0.) { printf("error :: non defite positive matrix\n"); return;}
            
            *p=sqrt(*p);
            for(k=j+1,q=p+n; k<n ;++k,q+=n){
                for(r=v+j*n,s=v+k*n,z=0.; r<p ;) z+= *r++ * *s++;
                *q-=z; *q/= *p;
            }
            
        }
        
        
        inv = inv.transposed();
        
        for(j=0,p=v; j<n ;++j,p+=n+1){ *p=1./ *p;
            for(q=v+j,t=v; q<p ;t+=n+1,q+=n){
                for(s=q,r=t,z=0.; s<p ;s+=n) z-= *s * *r++;
                *q=z* *p; }
        }
        for(j=0,p=v; j<n ;++j,p+=n+1){
            for(q=v+j,t=p-j; q<=p ;q+=n){
                for(k=j,r=p,s=q,z=0.; k<n ;++k) z += *r++ * *s++;
                *t++ =(*q=z); }
        }
        
    }
    
    
    
    
    float withSignOf(float a, float b)
    { return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a); }
    
    float svdhypot(float a, float b)
    {
        a = fabsf(a);
        b = fabsf(b);
        if(a > b) {
            b /= a;
            return a*sqrt(1.0 + b*b);
        } else if(b) {
            a /= b;
            return b*sqrt(1.0 + a*a);
        }
        return 0.0;
    }
    
    
    void svdrotate(float& a, float& b, float c, float s)
    {
        float d = a;
        a = +d*c +b*s;
        b = -d*s +b*c;
    }
    
    
    
    void svdrotate_double(double& a, double& b, double c, double s)
    {
        double d = a;
        a = +d*c +b*s;
        b = -d*s +b*c;
    }
    
    
    
    
    
    
    
    // **********************************************
    //  Householder reduction to tridiagonal matrix
    // **********************************************
    
    inline float SQR(const float a) {return a*a;}
    
    
    inline float SIGN(const float &a, const float &b)
    {return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a);}
    
    
    inline void SWAP(float &a, float &b)
    {float dum=a; a=b; b=dum;}
    
    
    
    float **allocate_float_matrix(int nrows, int ncols)
    {
        float ** matrix;
        matrix = new float*[nrows];
        for(int i=0; i < nrows; i++) matrix[i] = new float[ncols];
        return matrix;
    }
    
    
    void desallocate_float_matrix(float **matrix, int nrows, int ncols)
    {
        if (matrix == NULL) return;
        
        if (ncols){;}	// Just to avoid the warning in the compilation for non use
        
        for(int i=0; i < nrows; i++) { delete[] matrix[i]; matrix[i] = 0;}
        matrix = 0;
    }
    
    
    
    
    
    void symmetric_2_tridiag_HH(float **a,float *d,float *e,int n)
    {
        int l,k,j,i;
        float scale,hh,h,g,f;
        
        
        for (i=n-1;i>0;i--) {
            l=i-1;
            h=scale=0.0;
            if (l > 0) {
                for (k=0;k<l+1;k++)
                scale += fabs(a[i][k]);
                if (scale == 0.0)
                e[i]=a[i][l];
                else {
                    for (k=0;k<l+1;k++) {
                        a[i][k] /= scale;
                        h += a[i][k]*a[i][k];
                    }
                    f=a[i][l];
                    g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
                    e[i]=scale*g;
                    h -= f*g;
                    a[i][l]=f-g;
                    f=0.0;
                    for (j=0;j<l+1;j++) {
                        // Next statement can be omitted if eigenvectors not wanted
                        a[j][i]=a[i][j]/h;
                        g=0.0;
                        for (k=0;k<j+1;k++)
                        g += a[j][k]*a[i][k];
                        for (k=j+1;k<l+1;k++)
                        g += a[k][j]*a[i][k];
                        e[j]=g/h;
                        f += e[j]*a[i][j];
                    }
                    hh=f/(h+h);
                    for (j=0;j<l+1;j++) {
                        f=a[i][j];
                        e[j]=g=e[j]-hh*f;
                        for (k=0;k<j+1;k++)
                        a[j][k] -= (f*e[k]+g*a[i][k]);
                    }
                }
            } else
            e[i]=a[i][l];
            d[i]=h;
        }
        
        // Next statement can be omitted if eigenvectors not wanted
        d[0]=0.0;
        e[0]=0.0;
        // Contents of this loop can be omitted if eigenvectors not
        //	wanted except for statement d[i]=a[i][i];
        for (i=0;i<n;i++) {
            l=i;
            if (d[i] != 0.0) {
                for (j=0;j<l;j++) {
                    g=0.0;
                    for (k=0;k<l;k++)
                    g += a[i][k]*a[k][j];
                    for (k=0;k<l;k++)
                    a[k][j] -= g*a[k][i];
                }
            }
            d[i]=a[i][i];
            a[i][i]=1.0;
            for (j=0;j<l;j++) a[j][i]=a[i][j]=0.0;
        }
    }
    
    
    
    // **********************************************
    //  QL decomposition algorithm to be used with tridiagonal matrices
    // **********************************************
    /*
     //! Computes
     float SQR(float a)
     {
     return a*a;
     }
     
     
     //! returns a or -a in function of the signe of a*b
     float SIGN(float a,float b)
     {
     if (b>=0)
     return  (a>=0 ? a : -a) ;
     else
     return  (a>=0 ? -a : a) ;
     
     }
     */
    
    
    /*- Computes (a*a+b*b)^1/2 without destructive underflow or overflow -*/
    float pythag(float a,float b)
    {
        float absa,absb;
        
        absa=fabs(a);
        absb=fabs(b);
        if (absa>absb) return absa*sqrt(1.0+SQR(absb/absa));
        else return (absb==0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
    }
    
    
    int eigenvalues_QLI(float * d, float *e,float **z, int n)
    {
        int m,l,iter,i,k;
        float s,r,p,g,f,dd,c,b;
        
        for (i=1;i<n;i++) e[i-1]=e[i];
        e[n-1]=0.0;
        
        for (l=0;l<n;l++)
        {
            iter=0;
            do {
                for (m=l;m<n-1;m++)
                {
                    dd=fabs(d[m])+fabs(d[m+1]);
                    if ( fabs(e[m])+dd == dd) break;
                }
                
                if (m != l)
                {
                    if (iter++ == 100) { printf("Too many iterations in eigenvalues_QLI\n"); return 0;}
                    g=(d[l+1]-d[l])/(2.0*e[l]);
                    r=pythag(g,1.0);
                    g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
                    s=c=1.0;
                    p=0.0;
                    for (i=m-1;i>=l;i--)
                    {
                        f=s*e[i];
                        b=c*e[i];
                        e[i+1]=(r=pythag(f,g));
                        if (r == 0.0) {
                            d[i+1] -= p;
                            e[m]=0.0;
                            break;
                        }
                        s=f/r;
                        c=g/r;
                        g=d[i+1]-p;
                        r=(d[i]-g)*s+2.0*c*b;
                        d[i+1]=g+(p=s*r);
                        g=c*r-b;
                        
                        // Next loop can be omitted if eigenvectors not wanted
                        for (k=0;k<n;k++)
                        {
                            f=z[k][i+1];
                            z[k][i+1]=s*z[k][i]+c*f;
                            z[k][i]=c*z[k][i]-s*f;
                        }
                    }
                    if (r == 0.0 && i >= l) continue;
                    d[l] -= p;
                    e[l]=g;
                    e[m]=0.0;
                }
            } while (m != l);
            
                        
        }
        
        return 1;
    }
    
    
    
    
    
    void  center_data_columns(float **X,float *baricenter,int n, int p)
    {
        
        
        for(int j=0;j<p;j++){
            
            float some=0.0;
            for(int i=0; i<n; i++) some += X[i][j];
            
            some/=(float) n;
            baricenter[j]=some;
            
            for(int i=0; i<n; i++) X[i][j]-=baricenter[j];
        }
        
    }
    
    
    
    
    /*
     void order_decreasing(float *values, int *indexos, int size)
     {
     
     for(int i=0; i < size; i++)
     {
     float dmax = values[i];
     int pos = i;
     
     for(int j=i+1; j < size; j++)
     if (values[j] > dmax) {dmax = values[j]; pos = j;}
     
     
     values[pos] = values[i];
     values[i] = dmax;
     
     int aux = indexos[i];
     
     indexos[i] = indexos[pos];
     indexos[pos] = aux;
     
     
     }
     
     }
     */
    
    
    
    
    float **  covariance_matrix(float **x,int n, int p)
    {
        
        int i,j,k;
        float **cov;
        float some;
        
        cov = allocate_float_matrix(p,p);
        
        /*--- Calculam la matriu de covariance cov[k][j] ---*/
        for(j=0;j<p;j++)
        for(k=0;k<=j;k++){
            some=0.0;
            for(i=0;i<n;i++)
            some+=(x[i][j])*(x[i][k]);
            some/=(float) (n-1);
            cov[k][j]=cov[j][k]=some;
        }
        
        
        return cov;
        
    }
    
    
    
    int  compute_pca_from_covariance(float **Mat_cov,float **Pcs, float *sVar, int p)
    {
        
        float  *ve, *vpdes, *posicio;
        int i,j,pos,k,l;
        float dmax,*vap;
        
        
        
        vpdes= (float *) malloc(p*sizeof(float));
        ve = (float *) malloc(p*sizeof(float));;
        
        
        
        symmetric_2_tridiag_HH(Mat_cov,vpdes,ve,p);
        
        if (eigenvalues_QLI(vpdes,ve,Mat_cov,p))
        {
            
            
            
            
            /*--------- Ordenam valors propis  -----------------*/
            vap=vpdes;
            posicio = (float *) malloc(p*sizeof(float));
            
            for(i=0;i<p;i++){
                
                dmax=vap[0];
                pos=0;
                
                for(j=1;j<p;j++)
                if (vap[j]>dmax){
                    dmax=vap[j];
                    pos=j;
                }
                
                sVar[i]=dmax;
                posicio[i]=(float) pos;
                vap[pos]=0.0;
                
            }
            
            
            /*---------------Ordenam vectors propis------------*/
            
            
            for(k=0;k<p;k++)
            for(l=0;l<p;l++){
                
                Pcs[l][k]=Mat_cov[l][(int) posicio[k]];
            }
            
            
            free(ve);
            free(posicio);
            free(vpdes);
            
            
            return 1;
            
        } else
        {
            
            free(ve);
            free(vpdes);
            
            printf("PCA failed.\n");
            return 0;
            
        }
        
        
    }
    
    
    
    
    
    
    double **laMatrix2double(laMatrix &Ain)
    {
        int rows = Ain.nrows();
        int cols = Ain.ncols();
        
        double **A=new double *[rows];
        A[0] = new double[rows*cols];
        for (int i=1; i< rows; i++) A[i]=A[i-1]+cols;
        
        //float **ptrAin = Ain.v();
        for (int i=0; i< rows; i++)
        for (int j=0; j< cols; j++) A[i][j]=(double) Ain[i][j];
        
        return A;
    }
    
    double *laVector2double(laVector &Vin)
    {
        int size = Vin.size();
        
        double *V=new double[size];
        //float *ptrVin = Vin.v();
        for (int i=0; i< size; i++) V[i]=(double) Vin[i];
        
        return V;
    }
    
    //includes delete[]
    void double2laMatrix(double **A, laMatrix &Ain)
    {
        int rows = Ain.nrows();
        int cols = Ain.ncols();
        
        //float **ptrAin = Ain.v();
        for (int i=0; i< rows; i++)
        for (int j=0; j< cols; j++) Ain[i][j]=(float) A[i][j];
        
        delete[] A[0];
        delete[] A;
    }
    
    //includes delete[]
    void double2laVector(double *V, laVector &Vin)
    {
        int size = Vin.size();
        
        //float *ptrVin = Vin.v();
        for (int i=0; i< size; i++) Vin[i]=(float) V[i];
        
        delete[] V;
    }
    
    
    
    //test function for computing svd without coefficients
    void compute_svd_double_without_coefs(laMatrix &Ain, laMatrix &m_Uin, laMatrix &m_Vin, laVector &m_Win)
    {
        int rows = Ain.nrows();
        int cols = Ain.ncols();
        
        double **A=laMatrix2double(Ain);
        double **m_U=laMatrix2double(m_Uin);
        double **m_V=laMatrix2double(m_Vin);
        double *m_W=laVector2double(m_Win);
        
        const double    EPSILON = 0.00001;
        const int SVD_MAX_ITS = 100;
        
        double g, scale, anorm;
        double * RV1 = new double[cols];
        
        
        for(int i=0; i < rows; i++)
            for(int j=0; j < cols; j++)
                m_U[i][j] = A[i][j];
        
        // Householder reduction to bidiagonal form:
        //only modifying m_U
        anorm = g = scale = 0.0;
        for (int i=0; i< cols; i++)
        {
            int l = i + 1;
            RV1[i] = scale*g;
            g = scale = 0.0;
            if(i < rows)
            {
                for (int k=i; k< rows; k++)
                    scale += fabsf(m_U[k][i]);
                if (scale != 0.0) {
                    double invScale=1.0/scale, s=0.0;
                    for (int k=i; k< rows; k++) {
                        m_U[k][i] *= invScale;
                        s += m_U[k][i] * m_U[k][i];
                    }
                    double f = m_U[i][i];
                    g = - withSignOf(sqrt(s),f);
                    double h = 1.0 / (f*g - s);
                    m_U[i][i] = f - g;
                    for (int j=l; j< cols; j++) {
                        s = 0.0;
                        for (int k=i; k< rows; k++)
                            s += m_U[k][i] * m_U[k][j];
                        f = s * h;
                        for (int k=i; k< rows; k++)
                            m_U[k][j] += f * m_U[k][i];
                    }
                    for (int k=i; k< rows; k++)
                        m_U[k][i] *= scale;
                }
            }
            m_W[i] = scale * g;
            g = scale = 0.0;
            if ( i< rows && i< cols-1 ) {
                for (int k=l; k< cols; k++)
                    scale += fabsf(m_U[i][k]);
                if (scale != 0.0) {
                    double invScale=1.0/scale, s=0.0;
                    for (int k=l; k< cols; k++) {
                        m_U[i][k] *= invScale;
                        s += m_U[i][k] * m_U[i][k];
                    }
                    double f = m_U[i][l];
                    g = - withSignOf(sqrt(s),f);
                    double h = 1.0 / (f*g - s);
                    m_U[i][l] = f - g;
                    for (int k=l; k< cols; k++)
                        RV1[k] = m_U[i][k] * h;
                    for (int j=l; j< rows; j++) {
                        s = 0.0;
                        for (int k=l; k< cols; k++)
                            s += m_U[j][k] * m_U[i][k];
                        for (int k=l; k< cols; k++)
                            m_U[j][k] += s * RV1[k];
                    }
                    for (int k=l; k< cols; k++)
                        m_U[i][k] *= scale;
                }
            }
            anorm = MAX(anorm, fabsf(m_W[i]) + fabsf(RV1[i]) );
        }
        
        // Accumulation of right-hand transformations:
        m_V[cols-1][cols-1] = 1.0;
        for (int i= cols-2; i>=0; i--) {
            m_V[i][i] = 1.0;
            int l = i+1;
            g = RV1[l];
            if (g != 0.0) {
                double invgUil = 1.0 / (m_U[i][l]*g);
                for (int j=l; j< cols; j++)
                    m_V[j][i] = m_U[i][j] * invgUil;
                for (int j=l; j< cols; j++){
                    double s = 0.0;
                    for (int k=l; k< cols; k++)
                        s += m_U[i][k] * m_V[k][j];
                    for (int k=l; k< cols; k++)
                        m_V[k][j] += s * m_V[k][i];
                }
            }
            for (int j=l; j< cols; j++)
                m_V[i][j] = m_V[j][i] = 0.0;
        }
        // Diagonalization of the bidiagonal form:
        for (int k=cols-1; k>=0; k--) { // Loop over singular values
            for (int its=1; its<=SVD_MAX_ITS; its++) {
                bool flag = false;
                int l  = k;
                int nm = k-1;
                while(l>0 && fabsf(RV1[l]) > EPSILON*anorm) { // Test for splitting
                    if(fabsf(m_W[nm]) <= EPSILON*anorm) {
                        flag = true;
                        break;
                    }
                    l--;
                    nm--;
                }
                if (flag) {    // Cancellation of RV1[l], if l > 0
                    double c=0.0, s=1.0;
                    for (int i=l; i< k+1; i++) {
                        double f = s * RV1[i];
                        RV1[i] = c * RV1[i];
                        if (fabsf(f)<=EPSILON*anorm)
                            break;
                        g = m_W[i];
                        double h = svdhypot(f,g);
                        m_W[i] = h;
                        
                        h = 1.0 / h;
                        c = g * h;
                        s = - f * h;
                    }
                }
                double z = m_W[k];
                if (l==k) {    // Convergence of the singular value
                    if (z< 0.0) {    // Singular value is made nonnegative
                        m_W[k] = -z;
                        for (int j=0; j< cols; j++)
                            m_V[j][k] = - m_V[j][k];
                    }
                    break;
                }
                
                // Exception if convergence to the singular value not reached:
                if(its==SVD_MAX_ITS) {printf("svd::convergence_error\n"); delete[] RV1; exit(-1);}
                double x = m_W[l]; // Get QR shift value from bottom 2x2 minor
                nm = k-1;
                double y = m_W[nm];
                g = RV1[nm];
                double h = RV1[k];
                double f = ( (y-z)*(y+z) + (g-h)*(g+h) ) / ( 2.0*h*y );
                g = svdhypot(f,1.0);
                f = ( (x-z)*(x+z) + h*(y/(f+withSignOf(g,f)) - h) ) / x;
                // Next QR transformation (through Givens reflections)
                double c=1.0, s=1.0;
                for (int j=l; j<=nm; j++) {
                    int i = j+1;
                    g = RV1[i];
                    y = m_W[i];
                    h = s * g;
                    g = c * g;
                    z = svdhypot(f,h);
                    RV1[j] = z;
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                    f = x*c + g*s;
                    g = g*c - x*s;
                    h = y * s;
                    y *= c;
                    for(int jj=0; jj < cols; jj++)
                        svdrotate_double(m_V[jj][j],m_V[jj][i], c,s);
                    z = svdhypot(f,h);
                    m_W[j] = z;
                    if (z!=0.0) { // Rotation can be arbitrary if z = 0.0
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = c*g + s*y;
                    x = c*y - s*g;
                }
                RV1[l] = 0.0;
                RV1[k] = f;
                m_W[k] = x;
            }
        }
        double2laMatrix(A, Ain); //include delete double array
        double2laMatrix(m_U, m_Uin); //include delete double array
        double2laMatrix(m_V, m_Vin); //include delete double array
        double2laVector(m_W, m_Win); //include delete double array
        delete[] RV1;
    }
    
    
    
    
    
    //internal computations in double precision
    //input and output: float
    void compute_svd_double(laMatrix &Ain, laMatrix &m_Uin, laMatrix &m_Vin, laVector &m_Win)
    {
        
        int rows = Ain.nrows();
        int cols = Ain.ncols();
        
        double **A=laMatrix2double(Ain);
        double **m_U=laMatrix2double(m_Uin);
        double **m_V=laMatrix2double(m_Vin);
        double *m_W=laVector2double(m_Win);
        
        const double	EPSILON = 0.00001;
        const int SVD_MAX_ITS = 100;
        
        double g, scale, anorm;
        double * RV1 = new double[cols];
        
        
        for(int i=0; i < rows; i++)
        for(int j=0; j < cols; j++)
        m_U[i][j] = A[i][j];
        
        // Householder reduction to bidiagonal form:
        anorm = g = scale = 0.0;
        for (int i=0; i< cols; i++)
        {
            int l = i + 1;
            RV1[i] = scale*g;
            g = scale = 0.0;
            if(i < rows)
            {
                for (int k=i; k< rows; k++)
                scale += fabsf(m_U[k][i]);
                if (scale != 0.0) {
                    double invScale=1.0/scale, s=0.0;
                    for (int k=i; k< rows; k++) {
                        m_U[k][i] *= invScale;
                        s += m_U[k][i] * m_U[k][i];
                    }
                    double f = m_U[i][i];
                    g = - withSignOf(sqrt(s),f);
                    double h = 1.0 / (f*g - s);
                    m_U[i][i] = f - g;
                    for (int j=l; j< cols; j++) {
                        s = 0.0;
                        for (int k=i; k< rows; k++)
                        s += m_U[k][i] * m_U[k][j];
                        f = s * h;
                        for (int k=i; k< rows; k++)
                        m_U[k][j] += f * m_U[k][i];
                    }
                    for (int k=i; k< rows; k++)
                    m_U[k][i] *= scale;
                }
            }
            
            
            m_W[i] = scale * g;
            g = scale = 0.0;
            if ( i< rows && i< cols-1 ) {
                for (int k=l; k< cols; k++)
                scale += fabsf(m_U[i][k]);
                if (scale != 0.0) {
                    double invScale=1.0/scale, s=0.0;
                    for (int k=l; k< cols; k++) {
                        m_U[i][k] *= invScale;
                        s += m_U[i][k] * m_U[i][k];
                    }
                    double f = m_U[i][l];
                    g = - withSignOf(sqrt(s),f);
                    double h = 1.0 / (f*g - s);
                    m_U[i][l] = f - g;
                    for (int k=l; k< cols; k++)
                    RV1[k] = m_U[i][k] * h;
                    for (int j=l; j< rows; j++) {
                        s = 0.0;
                        for (int k=l; k< cols; k++)
                        s += m_U[j][k] * m_U[i][k];
                        for (int k=l; k< cols; k++)
                        m_U[j][k] += s * RV1[k];
                    }
                    for (int k=l; k< cols; k++)
                    m_U[i][k] *= scale;
                }
            }
            anorm = MAX(anorm, fabsf(m_W[i]) + fabsf(RV1[i]) );
        }
        
        // Accumulation of right-hand transformations:
        m_V[cols-1][cols-1] = 1.0;
        for (int i= cols-2; i>=0; i--) {
            m_V[i][i] = 1.0;
            int l = i+1;
            g = RV1[l];
            if (g != 0.0) {
                double invgUil = 1.0 / (m_U[i][l]*g);
                for (int j=l; j< cols; j++)
                m_V[j][i] = m_U[i][j] * invgUil;
                for (int j=l; j< cols; j++){
                    double s = 0.0;
                    for (int k=l; k< cols; k++)
                    s += m_U[i][k] * m_V[k][j];
                    for (int k=l; k< cols; k++)
                    m_V[k][j] += s * m_V[k][i];
                }
            }
            for (int j=l; j< cols; j++)
            m_V[i][j] = m_V[j][i] = 0.0;
        }
        
        // Accumulation of left-hand transformations:
        for (int i=MIN(rows,cols)-1; i>=0; i--) {
            int l = i+1;
            g = m_W[i];
            for (int j=l; j< cols; j++)
            m_U[i][j] = 0.0;
            if (g != 0.0) {
                g = 1.0 / g;
                double invUii = 1.0 / m_U[i][i];
                for (int j=l; j< cols; j++) {
                    double s = 0.0;
                    for (int k=l; k< rows; k++)
                    s += m_U[k][i] * m_U[k][j];
                    double f = (s * invUii) * g;
                    for (int k=i; k< rows; k++)
                    m_U[k][j] += f * m_U[k][i];
                }
                for (int j=i; j< rows; j++)
                m_U[j][i] *= g;
            } else
            for (int j=i; j< rows; j++)
            m_U[j][i] = 0.0;
            m_U[i][i] = m_U[i][i] + 1.0;
        }
        
        // Diagonalization of the bidiagonal form:
        for (int k=cols-1; k>=0; k--) { // Loop over singular values
            for (int its=1; its<=SVD_MAX_ITS; its++) {
                bool flag = false;
                int l  = k;
                int nm = k-1;
                while(l>0 && fabsf(RV1[l]) > EPSILON*anorm) { // Test for splitting
                    if(fabsf(m_W[nm]) <= EPSILON*anorm) {
                        flag = true;
                        break;
                    }
                    l--;
                    nm--;
                }
                if (flag) {	// Cancellation of RV1[l], if l > 0
                    double c=0.0, s=1.0;
                    for (int i=l; i< k+1; i++) {
                        double f = s * RV1[i];
                        RV1[i] = c * RV1[i];
                        if (fabsf(f)<=EPSILON*anorm)
                        break;
                        g = m_W[i];
                        double h = svdhypot(f,g);
                        m_W[i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = - f * h;
                        for (int j=0; j< rows; j++)
                        svdrotate_double(m_U[j][nm],m_U[j][i], c,s);
                    }
                }
                double z = m_W[k];
                if (l==k) {		// Convergence of the singular value
                    if (z< 0.0) {	// Singular value is made nonnegative
                        m_W[k] = -z;
                        for (int j=0; j< cols; j++)
                        m_V[j][k] = - m_V[j][k];
                    }
                    break;
                }
                
                // Exception if convergence to the singular value not reached:
                if(its==SVD_MAX_ITS) {printf("svd::convergence_error\n"); delete[] RV1; exit(-1);}
                double x = m_W[l]; // Get QR shift value from bottom 2x2 minor
                nm = k-1;
                double y = m_W[nm];
                g = RV1[nm];
                double h = RV1[k];
                double f = ( (y-z)*(y+z) + (g-h)*(g+h) ) / ( 2.0*h*y );
                g = svdhypot(f,1.0);
                f = ( (x-z)*(x+z) + h*(y/(f+withSignOf(g,f)) - h) ) / x;
                // Next QR transformation (through Givens reflections)
                double c=1.0, s=1.0;
                for (int j=l; j<=nm; j++) {
                    int i = j+1;
                    g = RV1[i];
                    y = m_W[i];
                    h = s * g;
                    g = c * g;
                    z = svdhypot(f,h);
                    RV1[j] = z;
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                    f = x*c + g*s;
                    g = g*c - x*s;
                    h = y * s;
                    y *= c;
                    for(int jj=0; jj < cols; jj++)
                    svdrotate_double(m_V[jj][j],m_V[jj][i], c,s);
                    z = svdhypot(f,h);
                    m_W[j] = z;
                    if (z!=0.0) { // Rotation can be arbitrary if z = 0.0
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = c*g + s*y;
                    x = c*y - s*g;
                    for(int jj=0; jj < rows; jj++)
                    svdrotate_double(m_U[jj][j],m_U[jj][i], c,s);
                }
                RV1[l] = 0.0;
                RV1[k] = f;
                m_W[k] = x;
            }
        }
        
        
        double2laMatrix(A, Ain); //include delete double array
        double2laMatrix(m_U, m_Uin); //include delete double array
        double2laMatrix(m_V, m_Vin); //include delete double array
        double2laVector(m_W, m_Win); //include delete double array
        delete[] RV1;
        
    }
    
    
    
    
    
    void compute_svd(laMatrix &A, laMatrix &m_U, laMatrix &m_V, laVector &m_W)
    {
        
        int rows = A.nrows();
        int cols = A.ncols();
        
        const float	EPSILON = 0.00001;
        const int SVD_MAX_ITS = 100;
        
        float g, scale, anorm;
        float * RV1 = new float[cols];
        
        
        for(int i=0; i < rows; i++)
        for(int j=0; j < cols; j++)
        m_U[i][j] = A[i][j];
        
        // Householder reduction to bidiagonal form:
        anorm = g = scale = 0.0;
        for (int i=0; i< cols; i++)
        {
            int l = i + 1;
            RV1[i] = scale*g;
            g = scale = 0.0;
            if(i < rows)
            {
                for (int k=i; k< rows; k++)
                scale += fabsf(m_U[k][i]);
                if (scale != 0.0) {
                    float invScale=1.0/scale, s=0.0;
                    for (int k=i; k< rows; k++) {
                        m_U[k][i] *= invScale;
                        s += m_U[k][i] * m_U[k][i];
                    }
                    float f = m_U[i][i];
                    g = - withSignOf(sqrt(s),f);
                    float h = 1.0 / (f*g - s);
                    m_U[i][i] = f - g;
                    for (int j=l; j< cols; j++) {
                        s = 0.0;
                        for (int k=i; k< rows; k++)
                        s += m_U[k][i] * m_U[k][j];
                        f = s * h;
                        for (int k=i; k< rows; k++)
                        m_U[k][j] += f * m_U[k][i];
                    }
                    for (int k=i; k< rows; k++)
                    m_U[k][i] *= scale;
                }
            }
            
            
            m_W[i] = scale * g;
            g = scale = 0.0;
            if ( i< rows && i< cols-1 ) {
                for (int k=l; k< cols; k++)
                scale += fabsf(m_U[i][k]);
                if (scale != 0.0) {
                    float invScale=1.0/scale, s=0.0;
                    for (int k=l; k< cols; k++) {
                        m_U[i][k] *= invScale;
                        s += m_U[i][k] * m_U[i][k];
                    }
                    float f = m_U[i][l];
                    g = - withSignOf(sqrt(s),f);
                    float h = 1.0 / (f*g - s);
                    m_U[i][l] = f - g;
                    for (int k=l; k< cols; k++)
                    RV1[k] = m_U[i][k] * h;
                    for (int j=l; j< rows; j++) {
                        s = 0.0;
                        for (int k=l; k< cols; k++)
                        s += m_U[j][k] * m_U[i][k];
                        for (int k=l; k< cols; k++)
                        m_U[j][k] += s * RV1[k];
                    }
                    for (int k=l; k< cols; k++)
                    m_U[i][k] *= scale;
                }
            }
            anorm = MAX(anorm, fabsf(m_W[i]) + fabsf(RV1[i]) );
        }
        
        // Accumulation of right-hand transformations:
        m_V[cols-1][cols-1] = 1.0;
        for (int i= cols-2; i>=0; i--) {
            m_V[i][i] = 1.0;
            int l = i+1;
            g = RV1[l];
            if (g != 0.0) {
                float invgUil = 1.0 / (m_U[i][l]*g);
                for (int j=l; j< cols; j++)
                m_V[j][i] = m_U[i][j] * invgUil;
                for (int j=l; j< cols; j++){
                    float s = 0.0;
                    for (int k=l; k< cols; k++)
                    s += m_U[i][k] * m_V[k][j];
                    for (int k=l; k< cols; k++)
                    m_V[k][j] += s * m_V[k][i];
                }
            }
            for (int j=l; j< cols; j++)
            m_V[i][j] = m_V[j][i] = 0.0;
        }
        
        // Accumulation of left-hand transformations:
        for (int i=MIN(rows,cols)-1; i>=0; i--) {
            int l = i+1;
            g = m_W[i];
            for (int j=l; j< cols; j++)
            m_U[i][j] = 0.0;
            if (g != 0.0) {
                g = 1.0 / g;
                float invUii = 1.0 / m_U[i][i];
                for (int j=l; j< cols; j++) {
                    float s = 0.0;
                    for (int k=l; k< rows; k++)
                    s += m_U[k][i] * m_U[k][j];
                    float f = (s * invUii) * g;
                    for (int k=i; k< rows; k++)
                    m_U[k][j] += f * m_U[k][i];
                }
                for (int j=i; j< rows; j++)
                m_U[j][i] *= g;
            } else
            for (int j=i; j< rows; j++)
            m_U[j][i] = 0.0;
            m_U[i][i] = m_U[i][i] + 1.0;
        }
        
        // Diagonalization of the bidiagonal form:
        for (int k=cols-1; k>=0; k--) { // Loop over singular values
            for (int its=1; its<=SVD_MAX_ITS; its++) {
                bool flag = false;
                int l  = k;
                int nm = k-1;
                while(l>0 && fabsf(RV1[l]) > EPSILON*anorm) { // Test for splitting
                    if(fabsf(m_W[nm]) <= EPSILON*anorm) {
                        flag = true;
                        break;
                    }
                    l--;
                    nm--;
                }
                if (flag) {	// Cancellation of RV1[l], if l > 0
                    float c=0.0, s=1.0;
                    for (int i=l; i< k+1; i++) {
                        float f = s * RV1[i];
                        RV1[i] = c * RV1[i];
                        if (fabsf(f)<=EPSILON*anorm)
                        break;
                        g = m_W[i];
                        float h = svdhypot(f,g);
                        m_W[i] = h;
                        h = 1.0 / h;
                        c = g * h;
                        s = - f * h;
                        for (int j=0; j< rows; j++)
                        svdrotate(m_U[j][nm],m_U[j][i], c,s);
                    }
                }
                float z = m_W[k];
                if (l==k) {		// Convergence of the singular value
                    if (z< 0.0) {	// Singular value is made nonnegative
                        m_W[k] = -z;
                        for (int j=0; j< cols; j++)
                        m_V[j][k] = - m_V[j][k];
                    }
                    break;
                }
                
                // Exception if convergence to the singular value not reached:
                if(its==SVD_MAX_ITS) {printf("svd::convergence_error\n"); delete[] RV1; exit(-1);}
                float x = m_W[l]; // Get QR shift value from bottom 2x2 minor
                nm = k-1;
                float y = m_W[nm];
                g = RV1[nm];
                float h = RV1[k];
                float f = ( (y-z)*(y+z) + (g-h)*(g+h) ) / ( 2.0*h*y );
                g = svdhypot(f,1.0);
                f = ( (x-z)*(x+z) + h*(y/(f+withSignOf(g,f)) - h) ) / x;
                // Next QR transformation (through Givens reflections)
                float c=1.0, s=1.0;
                for (int j=l; j<=nm; j++) {
                    int i = j+1;
                    g = RV1[i];
                    y = m_W[i];
                    h = s * g;
                    g = c * g;
                    z = svdhypot(f,h);
                    RV1[j] = z;
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                    f = x*c + g*s;
                    g = g*c - x*s;
                    h = y * s;
                    y *= c;
                    for(int jj=0; jj < cols; jj++)
                    svdrotate(m_V[jj][j],m_V[jj][i], c,s);
                    z = svdhypot(f,h);
                    m_W[j] = z;
                    if (z!=0.0) { // Rotation can be arbitrary if z = 0.0
                        z = 1.0 / z;
                        c = f * z;
                        s = h * z;
                    }
                    f = c*g + s*y;
                    x = c*y - s*g;
                    for(int jj=0; jj < rows; jj++)
                    svdrotate(m_U[jj][j],m_U[jj][i], c,s);
                }
                RV1[l] = 0.0;
                RV1[k] = f;
                m_W[k] = x;
            }
        }
        
        
        
        delete[] RV1;
        
        
    }
    
    
    
    void compute_pca_svd(laMatrix &X, laVector &S, laMatrix &V, laMatrix &U)
    {
        
        int n = X.nrows();
        int p = X.ncols();
        
        compute_svd_double(X,U,V,S);
        
        // U contain new coefficients, which must be normalized by eigenvalues
        for(int i=0; i < n; i++)
        for(int j=0; j < p; j++)
        U[i][j] *= S[j];
        
        // Normalize eigenvalues
        float norm = (float) (n-1);
        for(int i=0; i < p; i++)
        S[i] = S[i] * S[i] / norm;
        
        // If n < p, principal component should be zero from n to p-1
        // Coefficients of these principal components should be zero
        if (n < p)
        {
            for(int i=n-1; i < p; i++) S[i] = 0.0f;
            
            for(int j=0; j < n; j++)
            for(int i=n-1; i < p; i++)
            U[j][i] = 0.0f;
        }
        
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    
    
    
    
    /////////////////Added code 29/12/2017 ////////////////
    
    // (auxiliary function)
    // Perform a QR reduction of a real bidiagonal matrix and update the
    // orthogonal transformation matrices U and V.
    template <typename T>
    static int qrbdv(T *dm, T *em, T *um, int mm, T *vm, int m)
    {
        int i, j, k, n, jj, nm;
        T u, x, y, a, b, c, s, t, w, *p, *q;
        for (j = 1, t = fabs(dm[0]); j < m; ++j)
        if((s = fabs(dm[j]) + fabs(em[j - 1])) > t) t = s;
        t *= 1.e-15; n = 5000*m; nm = m;
        for(j = 0; m > 1 && j < n; ++j){
            for(k = m - 1; k > 0; --k){
                if(fabs(em[k-  1]) < t) break;
                if(fabs(dm[k - 1]) < t){
                    for(i = k, s = 1., c= 0.; i < m; ++i){
                        a = s*em[i - 1]; b = dm[i]; em[i - 1]*=c;
                        dm[i] = u = sqrt(a*a + b*b); s = -a/u; c = b/u;
                        for(jj = 0, p = um + k- 1 ; jj < mm; ++jj, p += mm){
                            q = p + i - k + 1;
                            w = c* *p + s* *q; *q = c* *q - s* *p; *p = w;
                        }
                    }
                    break;
                }
            }
            y = dm[k]; x = dm[m - 1]; u = em[m - 2];
            a = (y + x)*(y - x) - u*u; s = y*em[k]; b = s + s;
            u = sqrt(a*a + b*b);
            if(u != 0.){
                if(a >= 0)
                a += u;
                else // Protect against dramatic cancellation
                a = (b*b)/(u - a);
                c = sqrt(a/(u + u));
                if(c != 0.) s /= (c*u); else s = 1.;
                for(i = k; i < m - 1; ++i){
                    b = em[i];
                    if(i > k){
                        a = s*em[i]; b *= c;
                        em[i - 1] = u = sqrt(x*x + a*a);
                        c = x/u; s = a/u;
                    }
                    a = c*y + s*b; b = c*b - s*y;
                    for(jj = 0, p = vm + i; jj < nm; ++jj, p += nm){
                        w = c* *p + s* *(p + 1); *(p + 1) = c* *(p + 1) - s* *p; *p = w;
                    }
                    s *= dm[i + 1]; dm[i] = u = sqrt(a*a + s*s);
                    y = c*dm[i + 1]; c = a/u; s /= u;
                    x = c*b + s*y; y = c*y - s*b;
                    for(jj = 0, p = um + i; jj < mm; ++jj, p += mm){
                        w =  c* *p + s* *(p + 1); *(p + 1) = c* *(p + 1)-s* *p; *p = w;
                    }
                }
            }
            em[m - 2] = x; dm[m - 1 ] = y;
            if(fabs(x) < t) --m;
            if(m == k + 1) --m;
        }
        return j;
    }
    
    // (auxiliary function)
    // Perform a left Householder transform matrix U from the vectors
    // specifying the Householder reflections.
    template <typename T>
    static void ldumat(T *a, T *u, int m, int n)
    {
        T *p0, *q0, *p, *q, *w;
        int i, j, k, mm;
        T s, h;
        w = (T *)calloc(m, sizeof(T));
        for(i = 0, mm = m*m, q = u; i < mm; ++i) *q++ = 0.;
        p0 = a + n*n - 1; q0 = u + m*m - 1; mm = m - n; i = n - 1;
        for(j = 0; j < mm; ++j, q0 -= m + 1) *q0 = 1.;
        if(mm == 0){ p0 -= n + 1; *q0 = 1.; q0 -= m + 1; --i; ++mm;}
        for(; i >= 0; --i, ++mm, p0 -= n + 1, q0 -= m + 1){
            if(*p0 != 0.){
                for(j = 0, p = p0 + n, h = 1.; j < mm; p += n) w[j++] = *p;
                h= *p0; *q0 = 1. - h;
                for(j = 0, q = q0 + m; j < mm; q += m) *q = -h*w[j++];
                for(k = i + 1, q = q0 + 1; k < m; ++k){
                    for(j = 0, p = q + m, s = 0.; j < mm; p += m) s += w[j++]* *p;
                    s *= h;
                    for(j = 0, p = q + m; j < mm; p += m) *p -= s*w[j++];
                    *q++ = -s;
                }
            }
            else{
                *q0 = 1.;
                for(j = 0, p = q0 + 1, q = q0 + m; j < mm; ++j, q += m) *q = *p++ = 0.;
            }
        }
        free(w);
    }
    
    // (auxiliary function)
    // Perform a right Householder transform matrix U from the vectors
    // specifying the Householder reflections.
    template <typename T>
    static void ldvmat(T *a, T *v, int n)
    {
        T *p0, *q0, *p, *q, *qq;
        T h, s;
        int i, j, k, mm;
        for(i = 0, mm = n*n, q = v; i < mm; ++i) *q++ = 0.;
        *v = 1.; q0 = v + n*n - 1; *q0 = 1.; q0 -= n + 1;
        p0 = a + n*n - n - n - 1;
        for(i = n - 2, mm = 1; i > 0; --i, p0 -= n + 1, q0 -= n+ 1, ++mm){
            if(*(p0 - 1) != 0.){
                for(j = 0, p = p0, h = 1.; j < mm; ++j, ++p) h += *p* *p;
                h = *(p0 - 1); *q0 = 1. - h;
                for(j = 0, q = q0 + n, p = p0; j < mm; ++j, q += n) *q = -h* *p++;
                for(k = i + 1, q = q0 + 1; k < n; ++k){
                    for(j = 0, qq = q + n, p = p0, s = 0.; j < mm; ++j, qq += n) s+= *qq* *p++;
                    s *= h;
                    for(j = 0, qq = q + n, p = p0; j < mm; ++j, qq += n) *qq -= s* *p++;
                    *q++ = -s;
                }
            }
            else{
                *q0 = 1.;
                for(j = 0, p = q0 + 1, q = q0 + n; j < mm; ++j, q += n) *q = *p++ = 0.;
            }
        }
    }
    // Compute the singular value decomposition A=U*D*V'
    // (efficient when m is much larger than n)
    template <typename T>
    int sv2uv(T *d, T *a, T *u, int m, T *v, int n)
    {
        T *p, *p1, *q, *pp, *w, *e;
        T s, t, h, r, sv;
        int i, j, k, mm, nm, ms;
        if(m < n) return -1;
        w = (T *)calloc(m + n, sizeof(T)); e = w + m;
        for(i = 0, mm = m, p = a; i < n; ++i, --mm, p += n + 1){
            if(mm > 1){ sv = h = 0.;
                for(j = 0, q = p, s = 0.; j < mm; ++j, q += n){
                    w[j] = *q; s+= *q* *q;
                }
                if(s > 0.){
                    h = sqrt(s); if(*p < 0.) h = -h;
                    s += *p*h; s = 1./s; t = 1./(w[0] += h);
                    sv = 1. + fabs(*p/h);
                    for(k = 1, ms = n - i; k < ms ; ++k){
                        for(j = 0, q = p + k, r = 0.; j < mm; q += n) r += w[j++]* *q;
                        r = r*s;
                        for(j = 0, q = p + k; j < mm; q += n) *q -= r*w[j++];
                    }
                    for(j = 1, q = p; j < mm ;) *(q += n) = w[j++]*t;
                }
                *p = sv; d[i] = -h;
            }
            if(mm == 1) d[i] = *p;
        }
        ldumat(a, u, m, n);
        for(i = 0, q = a; i < n; ++i){
            for(j = 0; j < n; ++j, ++q){
                if(j < i) *q = 0.;
                else if(j == i) *q = d[i];
            }
        }
        for(i = 0, mm = n, nm = n - 1, p = a; i < n; ++i, --mm, --nm, p += n + 1){
            if(i && mm > 1){ sv = h = 0.;
                for(j = 0, q = p, s = 0.; j < mm; ++j, q += n){
                    w[j] = *q; s += *q* *q;
                }
                if(s > 0.){
                    h = sqrt(s); if(*p < 0.) h = -h;
                    s += *p*h; s = 1./s; t = 1./(w[0] += h);
                    sv = 1. + fabs(*p/h);
                    for(k = 1, ms = n - i; k < ms; ++k){
                        for(j = 0, q = p + k, r = 0.; j < mm; q += n) r += w[j++]* *q;
                        for(j = 0, q = p + k, r *= s; j < mm; q += n) *q -= r*w[j++];
                    }
                    for(k = 0, p1 = u + i; k < m; ++k, p1 += m){
                        for(j = 0, q = p1, r = 0.; j < mm;) r += w[j++]* *q++;
                        for(j = 0, q = p1, r *= s; j < mm ;) *q++ -= r*w[j++];
                    }
                }
                *p = sv; d[i]= -h;
            }
            if(mm == 1) d[i]= *p;
            p1 = p + 1;
            if(nm > 1){ sv = h = 0.;
                for(j = 0, q = p1, s = 0.; j < nm; ++j, ++q) s += *q* *q;
                if(s > 0.){
                    h = sqrt(s); if(*p1 < 0.) h= -h;
                    sv = 1. + fabs(*p1/h);
                    s += *p1*h; s = 1./s; t = 1./(*p1 += h);
                    for(k = n, ms = n*(n - i); k < ms; k += n){
                        for(j = 0, q = p1, pp = p1 + k, r = 0.; j < nm; ++j) r += *q++ * *pp++;
                        for(j = 0, q = p1, pp = p1 + k, r *=s; j < nm; ++j) *pp++ -= r* *q++;
                    }
                    for(j = 1, q = p1 + 1; j < nm; ++j) *q++ *=t;
                }
                *p1 = sv; e[i]= -h;
            }
            if(nm == 1) e[i]= *p1;
        }
        ldvmat(a, v, n);
        qrbdv(d, e, u, m, v, n);
        for(i = 0; i < n; ++i){
            if(d[i] < 0.){ d[i]= -d[i];
                for(j = 0, p = v + i; j < n; ++j, p += n) *p= - *p;
            }
        }
        free(w);
        return 0;
    }
    
    // Compute the singular value decomposition A=U*D*V'
    // (calls the apropriate function above)
    template <typename T>
    void svd(T *d, T *a, T *u, int m, T *v, int n)
    {
        assert(m >= n);
        int out = sv2uv(d, a, u, m, v, n);
        if (out == -1){
            std::cerr << "Wrong dimensions for svd decomposition";
        }
    }

    
    
    /// Functor to sort elements of a vector by decreasing value.
    class SVDElement {
        public:
        SVDElement(const laVector& D, int i)
        : m_val(D[i]), m_i(i) {}
        bool operator<(const SVDElement& e) const
        { return (m_val>e.m_val); }
        
        float m_val;
        int m_i;
    };
    
    /*template <class T>
     void swap ( T& a, T& b )
     {
     T c(a); a = b; b = c;
     }*/
    
    //#define INDEX(i, j) ((i0 + i) * m_cols + (j0))
    /// Paste a matrix in another one, at position (\a i0,\a j0)
    /// \param i0 first row where to paste in
    /// \param j0 first column where to paste in
    /// \param matrix to paste
    void paste_col(int j0, laMatrix& A, std::vector<float> m)
    {
        assert(j0 >= 0 && j0 < A.ncols() && m.size() == A.nrows());
        
        
        for (int i = 0; i < A.nrows(); i++){
            A[i][j0] = m[i];
        }
        /*const float * in = m.v()[0];
         for(int i = 0; i < m.nrows(); i++) {
         float * out = A.v()[0] + ((i0 + i) * A.ncols() + (j0));
         for(int j = 0; j < m.ncols(); j++)
         *out++ = *in++;
         }*/
    }
    /// Copy column number j.
    std::vector<float> col_from_matrix(laMatrix A, int j)
    {
        assert(j >= 0 && j < A.ncols());
        std::vector<float> c(A.nrows());
        const float* in = A.v()[0] + j;
        for(int i = 0; i < A.nrows(); i++) {
            c[i] = *in;
            in += A.ncols();
        }
        return c;
    }
    
    
    /// Sort SVD by decreasing order of singular value.
    void sort(laMatrix& U, laVector& D, laMatrix& V)
    {
        std::vector<SVDElement> vec;
        for(int i = 0; i < D.size(); i++)
        vec.emplace_back(D, i);
        std::sort(vec.begin(), vec.end());
    
        // Apply permutation
        for(int i = std::min(U.ncols(), V.ncols()) - 1; i >= 0; i--) {
            if (vec[i].m_i != i) { // Find cycle of i
                const std::vector<float> colU = col_from_matrix(U, i);
                const std::vector<float> colV = col_from_matrix(V, i);
                const float w = D[i];
                int j = i;
                while (vec[j].m_i != i) {
                    paste_col(j, U, col_from_matrix(U, vec[j].m_i));
                    paste_col(j, V, col_from_matrix(V, vec[j].m_i));
                    D[j] = D[vec[j].m_i];
                    std::swap(j, vec[j].m_i);
                }
                vec[j].m_i = j;
                paste_col(j, U, colU);
                paste_col(j, V, colV);
                D[j] = w;
            }
        }
    }
    
    
    /// Singular Value Decomposition
//    void SVD(laMatrix &A, laMatrix& U, laVector& S, laMatrix& V){
//        assert(U.nrows() == A.nrows() && U.ncols() == A.nrows());
//        assert(S.size() == A.nrows() || S.size() == A.ncols());
//        assert(V.nrows() == A.ncols() && V.ncols() == A.ncols());
//
//
//        if(A.nrows() >= A.ncols()) {
//            //std::cout << "Hey you!\n";
//            svd(S.v(), A.v()[0], U.v()[0], A.nrows(), V.v()[0], A.ncols());
//        } else {
//            //std::cout << "Hey you2!\n";
//            laMatrix A_t = A.transposed(); //matrix<T> A(this->t()); //Transpose
//            svd(S.v(), A_t.v()[0], V.v()[0], A_t.nrows(), U.v()[0], A_t.ncols());
//        }
//
//        sort(U, S, V);
//
//    }
    
    
    /// Singular Value Decomposition
    void SVD(laMatrix &A, laMatrix& U, laVector& S, laMatrix& V){
        assert(U.nrows() == A.nrows() && U.ncols() == A.nrows());
        assert(S.size() == A.nrows() || S.size() == A.ncols());
        assert(V.nrows() == A.ncols() && V.ncols() == A.ncols());
        
        double **u = laMatrix2double(U);
        double **v = laMatrix2double(V);
        double * s = laVector2double(S);
        if(A.nrows() >= A.ncols()) {
            double * a = laMatrix2double(A)[0];
            svd(s, a, u[0], A.nrows(), v[0], A.ncols());
        } else {
            laMatrix A_t = A.transposed(); //matrix<T> A(this->t()); //Transpose
            double * a_t = laMatrix2double(A_t)[0];
            svd(s, a_t, v[0], A_t.nrows(), u[0], A_t.ncols());
        }
        double2laMatrix(u, U);
        double2laMatrix(v, V);
        double2laVector(s, S);
        sort(U, S, V);
    }
    
    
    
    
    void compute_planar_homography_n_points_new_svd(float *x0, float *y0, float *x1, float *y1, int n, laMatrix &H)
    {
        ////////////////// Compute Baricenter
        float lx = 0.0,  ly = 0.0;
        float rx = 0.0,  ry = 0.0;
        
        for (int i = 0; i < n; i++) {
            
            lx += x0[i];
            ly += y0[i];
            
            rx += x1[i];
            ry += y1[i];
        }
        
        lx /= (float) n;
        ly /= (float) n;
        rx /= (float) n;
        ry /= (float) n;
        
        
        /////////////// Normalize points without modifying original vectors
        
        float *px0 = new float[n];
        float *py0 = new float[n];
        
        float *px1 = new float[n];
        float *py1 = new float[n];
        
        float spl = 0.0f, spr = 0.0f;
        for(int i = 0; i < n; i++)
        {
            
            px0[i] = x0[i] - lx;
            py0[i] = y0[i] - ly;
            
            px1[i] = x1[i] - rx;
            py1[i] = y1[i] - ry;
            
            spl += sqrtf(px0[i] * px0[i] + py0[i]*py0[i]);
            spr += sqrtf(px1[i] * px1[i] + py1[i]*py1[i]);
            
        }
        
        
        spl = sqrtf(2.0f) / spl;
        spr = sqrtf(2.0f) / spr;
        
        for (int i = 0; i < n; i++) {
            
            px0[i] *= spl;
            py0[i] *= spl;
            
            px1[i] *= spr;
            py1[i] *= spr;
            
        }
        
        
        ////////////////////////// Minimization problem || Ah || /////////////////////////////
        
        laMatrix Tpl(3, 3), Timinv(3, 3);
        
        // similarity transformation of the plane
        Tpl[0][0] = spl; Tpl[0][1] = 0.0; Tpl[0][2] = -spl*lx;
        Tpl[1][0] = 0.0; Tpl[1][1] = spl; Tpl[1][2] = -spl*ly;
        Tpl[2][0] = 0.0; Tpl[2][1] = 0.0; Tpl[2][2] = 1.0;
        
        
        // inverse similarity transformation of the image
        Timinv[0][0] = 1.0f/spr; Timinv[0][1] =   0.0  ; Timinv[0][2] = rx;
        Timinv[1][0] =   0.0  ; Timinv[1][1] = 1.0f/spr; Timinv[1][2] = ry;
        Timinv[2][0] =   0.0  ; Timinv[2][1] =   0.0  ; Timinv[2][2] = 1.0;
        
        ///////////////// System matrix  ///////////////////////
        
        laMatrix A(2*n, 9);
        
        
        for(int i = 0, eq = 0; i < n; i++, eq++) {
            
            float	xpl = px0[i], ypl = py0[i],
            xim = px1[i], yim = py1[i];
            
            A[eq][0] = A[eq][1] = A[eq][2] = 0.0;
            A[eq][3] = -xpl;
            A[eq][4] = -ypl;
            A[eq][5] = -1.0f;
            A[eq][6] =  yim * xpl;
            A[eq][7] =  yim * ypl;
            A[eq][8] =  yim;
            
            eq++;
            
            A[eq][0] =  xpl;
            A[eq][1] =  ypl;
            A[eq][2] =  1.0f;
            A[eq][3] = A[eq][4] = A[eq][5] = 0.0;
            A[eq][6] = -xim * xpl;
            A[eq][7] = -xim * ypl;
            A[eq][8] = -xim;
        }
        
        
        ///////////////// SVD  /////////////////
        /* Previous svd, it does not work for 2*n > 9, but it works for 4 points
         
         laMatrix U_original(2*n, 9), V_original(9, 9);
         laVector W_original(9);
         compute_svd(A, U_original, V_original, W_original);
         // Find the index of the least singular value
         int imin = 0;
         for (int i = 1; i <  9; i++)
         if ( W_original[i] < W_original[imin] ) imin = i;
         */
        
        laMatrix U(2*n, 2*n), V(9, 9);
        laVector W(std::min(2*n, 9));
        
        SVD(A, U, W, V);
        
        
        ////////////////// Denormalize H = Timinv * V.col(imin)* Tpl;
        
        laMatrix matrix(3, 3), result(3, 3);
        int k = 0;
        for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++, k++)
        matrix[i][j] = V[k][8]; //If svd is sorted the result is always last column
        
        
        /* Previous svd, it does not work for 2*n > 9, but it works for 4 points
         int k = 0;
         for(int i = 0; i < 3; i++)
         for(int j = 0; j < 3; j++, k++)
         matrix[i][j] = V_original[k][imin];
         
         */
        
        
        //Undo normalization
        result = Timinv * matrix;
        H = result * Tpl;
        
        
        delete[] px0;
        delete[] py0;
        delete[] px1;
        delete[] py1;
    }
    

    
    
    
    ///////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    
    
    
    void linear_fitting(float * tpX,float* tpY, float &da, float &db, int ilength)
    {
        
        float dx = 0.0, dy = 0.0, dxx = 0.0, dxy = 0.0;
        for (int ii=0; ii < ilength; ii++) {
            
            dx += (float) tpX[ii];
            dy += (float) tpY[ii];
            dxy += (float) (tpX[ii] * tpY[ii]);
            dxx += (float) (tpX[ii] * tpX[ii]);
            
        }
        
        
        dx /= (float) ilength;
        dy /= (float) ilength;
        dxy /= (float) ilength;
        dxx /= (float) ilength;
        
        float delta = dxx - dx*dx;
        
        //assert( fabs(delta) > dTiny);
        if (fabs(delta) < dTiny)
        {	printf("tpX vector might be constant\n"); exit(-1);}
        
        
        
        
        da = (dy * dxx - dx * dxy) / delta;
        db = (dxy - dx * dy) / delta;
        
        
    }
    
    
    
    void  l2_baricenter(float **X,float *baricenter,int n, int p)
    {
        
        
        for(int j=0; j<p; j++)
        {
            
            float some=0.0;
            for(int i=0; i<n; i++) some += X[i][j];
            
            some/=(float) n;
            baricenter[j]=some;
            
        }
        
    }
    
    
    float fiL1floatDist(float *u0,float *u1,int size)
    {
        float dist=0.0;
        float dif;
        for (int ii=0 ; ii < size; ii++)
        {
            
            dif = u0[ii] - u1[ii];  dist += fabsf(dif);
            
        }
        
        return dist;
    }
    
    
    void  l1_baricenter(float **X,float *baricenter,int n, int p, int niter)
    {
        
        l2_baricenter(X, baricenter, n, p);
        
        
        float *fpAbar = new float[p];
        for (int ii=0; ii < niter; ii++)
        {
            
            fpClear(fpAbar, 0.0, p);
            
            
            float fSome = 0.0f;
            for (int in = 0; in < n ; in++)
            {
                
                float *fpV = X[in];
                
                float dist = fiL1floatDist(baricenter, fpV, p);
                if (dist < fTiny) dist = fTiny;
                dist = 1.0f / dist;
                fSome += dist;
                
                
                for (int ij=0; ij < p; ij++)		fpAbar[ij] += dist * fpV[ij];
                
            }
            
            
            if (fSome > fTiny)	for (int ij=0; ij < p; ij++)		fpAbar[ij] /= fSome;
            else
            {
                //printf("warning :: fsome < fTiny\n");
                delete[] fpAbar;
                return;
            }
            
            
            fpCopy(fpAbar, baricenter, p);
            
        }
        
        
        delete[] fpAbar;
        
    }
    
    
    
    
    
    
    void kmeans(float **data, float **means, int *labels, int nIter, int kCenters, int nVectors, int pCoordinates)
    {
        
        
        
        //! Initialization of means
        for (int i=0; i < kCenters; i++)
        {
            int iIndex = (int) rintf((double) nVectors * drand48());
            if (iIndex < 0 || iIndex >= nVectors) {printf("warning :: kmeans :: incorrect initialization\n");}
            
            for(int j=0; j < pCoordinates; j++)
            means[i][j] = data[iIndex][j];
        }
        
        
        //! Auxiliar Parameters
        float *nMembers = new float[kCenters];         //! Number of vectors per cluster
        
        //! Iterative process
        for (int n = 0; n < nIter; n++)
        {
            
            //! Assign each vector to its closest mean
            for (int i=0; i < nVectors; i++)
            {
                float difMin = fLarge;
                for (int k=0; k < kCenters; k++)
                {
                    float dif =  fpDistLp(data[i], means[k], 2, pCoordinates);
                    if (dif < difMin) { difMin = dif; labels[i] = k; }
                }
                
            }
            
            //! Clear means and nMembers
            fpClear(nMembers, 0.0, kCenters);
            for (int k=0; k < kCenters; k++)  fpClear(means[k], 0.0, pCoordinates);
            
            
            //! Recompute means
            for (int i=0; i < nVectors; i++)
            {
                for(int j=0; j < pCoordinates; j++)
                {
                    means[labels[i]][j] += data[i][j];
                }
                
                nMembers[labels[i]]++;
            }
            
            for (int k=0; k < kCenters; k++)
            {
                for(int j=0; j < pCoordinates; j++) means[k][j] /= nMembers[k];
            }
            
        }
        
        
        delete[] nMembers;
        
    }
    
    
    
    
    
    
    
    void  estimate_noise_pca_variances(int d, int n, float fSigma, float fDelta, int *osize, float *curve)
    {
        
        // ///////  d 			dimension
        // ///////  n    		samples
        // ///////  sigma		noise standard deviation
        // //  y = n / d;
        // //  a = a(y) =  sigma^2 * (1 − y^1/2)^2
        // //  b = b(y) =  sigma^2 * (1 + y^1/2)^2
        // //  p_{y,sigma} (x) =  (1 / 2 pi y x sigma^2) sqrt((b-x) (a-x)) 	if a <= x <= b
        // //			0						otherwise
        // //
        // //  p_{y, sigma} (x) =  p_{y,1} (x / sigma^2) / sigma^2
        // //
        // //  F_{sigma}(eta) = F_{1} (eta / sigma^2)
        
        int N = MIN(n,d);
        
        float y = (float) d / (float) n;
        float x;
        
        float fSigma2 = fSigma*fSigma;
        float a = 1.0 - sqrt(y);
        a = fSigma2 * a * a;  // Can be a*a > b*b? No because, then |1-sqrt(y)| > |1+sqrt(y)|
        
        float b = 1.0 + sqrt(y);
        b = fSigma2 * b * b;
        
        int isize = (int) ceilf( b / fDelta) + 2*11; // We begin at x=-10*delta .. b + 10*delta
        *osize = isize;
        
        float *cdf = new float[isize];
        
        
        float normal = 2.0f * 3.14159f * y * fSigma2;
        //printf("y: %f a: %f b: %f\n", y , a , b);
        
        if (y < 1)
        {
            
            float cumulative = 0.0;
            x = -10.0f * fDelta;
            for(int i = 0; i < isize; i++, x+=fDelta)
            {
                
                float value;
                
                if (a<=x && x<=b) value =  fDelta * sqrtf( (b-x)*(x-a)) / (x * normal);
                else value = 0.0;
                
                cumulative += value;
                
                cdf[i] =  (float) N * (1.0 - cumulative);
                //printf("%f: %f \n", x, cdf[i]);
            }
            
            
        }  else{
            
            
            float cumulative = 1.0  -  1.0 / y;
            
            cdf[0] =  (float) d *   (1.0 - cumulative);
            //printf("cumulative: %f n: %f cdf: %f \n", cumulative, (float) n, cdf[0]);
            
            x = fDelta;
            for(int i = 1; i < isize; i++,  x+=fDelta)
            {
                float value;
                if (a<=x && x<=b) value =  fDelta * sqrt( (b-x)*(x-a)) / (x * normal);
                else value = 0.0;
                
                cumulative +=   value;
                
                cdf[i] =  (float) d * (1.0 - cumulative);
            }
            
            
        }
        
        
        float minpositiu = (float)(N + 1);
        for (int i=0; i < isize; i++) {
            
            if (cdf[i] < minpositiu && cdf[i] >= 0.0)  minpositiu = cdf[i];
            
        }
        
        
        int nvalue = N;
        fpClear(curve, 0.0, N);
        x = -10.0 * fDelta;
        for(int i=0; nvalue > 0 && i < isize-1 ;i++, x += fDelta)
        {
            
            
            float ceps = 0.01;
            
            if (nvalue == N && cdf[i] > (float) N - ceps && cdf[i] != cdf[i+1])
            {
                curve[nvalue-1] = x;
                nvalue--;
                
            }
            /* else if (nvalue == 0 && cdf[i] < minpositiu + ceps && cdf[i] != cdf[i+1])
             {
             
             curve[nvalue] = x;
             nvalue--;
             
             } */
            else if (/*nvalue != 0 && */nvalue != N && cdf[i] >= (float) nvalue - ceps && cdf[i] < (float) nvalue + ceps && cdf[i] != cdf[i+1])
            {
                
                curve[nvalue-1] = x;
                nvalue--;
                
            }
            
        }
        
        
        delete[] cdf;
        
        
    }
    
    
    
    
    
}




void  printusage(char *pname,
                 char *gp,
                 std::vector<OptStruct*>  &opt,
                 std::vector<ParStruct*>  &par)
{
    
    
    int npar = par.size();
    
    //! print function prototype
    //! "usage:" function_name [-r r] [-g g] [-b b]  par1 par2 ... parn
    printf("\nusage: %s ", pname);
    for(int i=0; i < (int) strlen(gp); i++)
    if (gp[i] != ':')
    {
        printf("[-%c",gp[i]);
        
        if (i+1 < (int) strlen(gp) && gp[i+1] ==  ':') printf(" %c] ", gp[i]);
        else printf("] ");
        
    }
    
    for(int i=0; i < npar; i++)
    printf(" %s ", par[i]->name);
    
    printf("\n");
    
    
    //! print options with associated descriptions and defaulted values
    int j=0;
    for(int i=0; i < (int) strlen(gp); i++)
    if (gp[i] != ':')
    {
        printf("\t-%c",gp[i]);
        
        if (i+1 < (int) strlen(gp) && gp[i+1] ==  ':') {
            
            printf("  %c\t %s ", gp[i], opt[j]->comment);
            if (opt[j]->defvalue != NULL) printf("(Default: %s)",opt[j]->defvalue);
            
            printf("\n");
            
        }
        else printf("\t %s \n", opt[j]->comment);
        
        j++;
    }
    
    //! print mandatory parameters with associated descriptions
    for(int i=0; i < npar; i++)
    {
        printf("\t%s",par[i]->name);
        printf("\t %s\n", par[i]->comment);
    }
    
    
    
}


void print_call(int argc, char **argv)
{
    
    printf("\nCall:  ");
    for (int ii=0; ii < argc; ii++)  printf("%s ", argv[ii]);
    printf("\n");
    
}



int parsecmdline(char *pname,
                 char *function,
                 int argc, char **argv,
                 std::vector <OptStruct*> & opt,
                 std::vector <ParStruct*> & par)
{
    
    //! number of options and obligatory parameters
    int nopt = opt.size();
    int npar = par.size();
    
    
    //! size of auxiliar parameter needed by getopt
    //! the length is at maximum 2*nopt since each option contains one caracter
    //! or two if the option requires a values
    char *gp = new char[2*nopt+1];
    gp[0]='\0';
    
    
    //! clear options and concatenate option identifiers into gp vector
    for(int i=0; i < nopt; i++) { opt[i]->flag = 0; opt[i]->value=NULL; strcat(gp, opt[i]->gp);}
    
    //! clear necessary parameter values
    for(int i=0; i < npar; i++) { par[i]->value = NULL;}
    
    
    //! in this way getopt doesn't print any information and errors are
    //! treated and printed by this program.
    opterr = 0;
    
    
    //! getopt return the following option of the concatenated vector gp
    //! or -1 if all options have already been treated
    //! it also fills "optarg" with the associated value to this option as it is given in argv
    int c;
    while ((c = getopt (argc, argv, gp)) != -1)
    {
        
        //! set current option given by getopt
        //! if getopt finds an option which was not in the option of the program (vector gp)
        //! returns ?
        int j=0;
        for(unsigned int i=0; i < strlen(gp); i++)
        if (c == gp[i])
        {
            opt[j]->flag = 1;
            opt[j]->value = optarg;  // getopt fills "optarg" with the associated value to this option as it is given in argv
            
            break;
            
        } else if (gp[i] != ':') j++;
        
        
        //! option found in console command was not one of the parameters of our program
        //! or should have a mandatory values which is not provided
        if (c == '?')
        {
            
            
            //! when getopt encounters an unknown option character or an option with a missing required argument
            //! it stores that option character in optopt variable
            unsigned int i = 0;
            for(i=0; i < strlen(gp); i++)
            if (optopt == gp[i])
            {
                printf("\n%s: %s\n", pname, function);
                printf("\nerror: option -%c requires an argument.\n", optopt);
                break;
            }
            
            if (i == strlen(gp)) {
                printf("\n%s: %s\n", pname, function);
                printf ("\nerror: unknown option `-%c'.\n", optopt);
            }
            
            print_call(argc,argv);
            printusage(pname, gp,  opt,  par);
            delete[] gp;
            return 0;
            
        }
        
        
        
    }
    
    
    //! Setting default values for non selected options
    for(int j=0; j < nopt; j++)
    if (opt[j]->flag == 0 && opt[j]->defvalue != NULL) opt[j]->value =  opt[j]->defvalue;
    
    
    //! Check remaining words in command after reading option
    if (argc - optind != npar)
    {
        //printf("\n%s: %s\n", pname, function);
        printf("%s\n", function);
        print_call(argc,argv);
        fprintf (stderr, "\nerror: incorrect number of parameters\n");
        printusage(pname, gp,  opt,par);
        delete[] gp;
        return 0;
    }
    
    //! Read mandatory parameter values
    int i=0;
    for (int index = optind; index < argc ; index++, i++){
        par[i]->value = argv[index];
    }
    
    
    delete[] gp;
    return 1;
    
    
}




