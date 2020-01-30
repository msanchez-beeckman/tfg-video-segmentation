#include "libImage.h"


namespace libUSTG
{
    
    
	
	//////////////////////////////////////////////////////////////////
	//! Begin Cflimage
	//////////////////////////////////////////////////////////////////
	
	//! Constructors
	cflimage::cflimage() : d_c(0), d_w(0), d_h(0), d_wh(0), d_whc(0), d_v(0),  visuMin(0.0f), visuMax(255.f)
	{
	}
	
	
	
	cflimage::cflimage(int w, int h, int c) : d_c(c), d_w(w), d_h(h),d_wh(w*h), d_whc(c*w*h), d_v(new float[c*w*h]),  visuMin(0.0f), visuMax(255.f)
	{
		for (int ii=0; ii < d_whc; ii++) d_v[ii] = 0.0f;
		
	}
	
	
	
	cflimage::cflimage(int w, int h, float *igray) : d_c(1), d_w(w), d_h(h),d_wh(w*h), d_whc(w*h), d_v(new float[w*h]), visuMin(0.0f), visuMax(255.f)
	{
		memcpy(d_v, igray, w * h * sizeof(float));
	}
	
	
    cflimage::cflimage(int w, int h, unsigned char *igray) : d_c(1), d_w(w), d_h(h),d_wh(w*h), d_whc(w*h), d_v(new float[w*h]), visuMin(0.0f), visuMax(255.f)
    {
        for (int ii=0; ii < d_wh; ii++) d_v[ii] = (float) igray[ii];
    }
    
    
	cflimage::cflimage(int w, int h, float *ired, float *igreen, float *iblue) : d_c(3), d_w(w), d_h(h),d_wh(w*h), d_whc(3*w*h), d_v(new float[3*w*h]),  visuMin(0.0f), visuMax(255.f)
	{
		memcpy(d_v, ired, w * h * sizeof(float));
		memcpy(d_v + w*h, igreen, w * h * sizeof(float));
		memcpy(d_v + 2*w*h, iblue, w * h * sizeof(float));
	}
	
	
    cflimage::cflimage(int w, int h, unsigned char *ired, unsigned char *igreen, unsigned char *iblue) : d_c(3), d_w(w), d_h(h),d_wh(w*h), d_whc(3*w*h), d_v(new float[3*w*h]),  visuMin(0.0f), visuMax(255.f)
    {
        for (int ii=0; ii < d_wh; ii++)
        {
            d_v[ii] = (float) ired[ii];
            d_v[d_wh + ii] = (float) igreen[ii];
            d_v[2*d_wh + ii] = (float) iblue[ii];
        }
    }
    
    

    
	cflimage::cflimage(const flimage &red, const  flimage &green,const  flimage &blue) : d_c(0), d_w(0), d_h(0), d_wh(0), d_whc(0), d_v(0),  visuMin(0.0f), visuMax(255.f)
	{
		
		assert(red.d_w == green.d_w && green.d_w == blue.d_w);
		assert(red.d_h == green.d_h && green.d_h == blue.d_h);
		
		d_w = red.d_w;
		d_h = red.d_h;
		d_c = 3;
		
		d_wh = d_w * d_h;
		d_whc = d_wh * d_c;
		
		d_v = new float[3 * d_wh];
		memcpy(d_v, red.d_v, d_wh * sizeof(float));
		memcpy(d_v + d_wh, green.d_v, d_wh * sizeof(float));
		memcpy(d_v + 2 * d_wh, blue.d_v, d_wh * sizeof(float));
		
		
	}
	
	
	cflimage::cflimage(const cflimage& im) : d_c(im.d_c), d_w(im.d_w), d_h(im.d_h),d_wh(im.d_wh), d_whc(im.d_whc), d_v(0),  visuMin(0.0f), visuMax(255.f)
	{
		
		if (d_whc > 0)
		{
			d_v = new float[d_whc];
			memcpy((void *) d_v, (const void *) im.d_v, d_whc * sizeof(float));
			
		}
		
	}
	
	
	
	void cflimage::create(int w, int h, int c)
	{
		erase();
		d_c = c; d_w = w; d_h=h; d_wh = w*h; d_whc = c*w*h;
		d_v = new float[d_whc];
		visuMin=0.0f;
		visuMax=255.0f;
		
		for (int ii=0; ii < d_whc; ii++) d_v[ii] = 0.0f;
	}
	
	
	
	void cflimage::erase()
	{
		d_w = d_h = d_wh = d_whc = 0;
		if (d_v) delete[] d_v;
		d_v=0;
	}
	
	
	
	
	cflimage::~cflimage()
	{
    	erase();
	}
	
	
	
    
	
	
	//////////////////////////////////////////////////////////////////
	//! Begin Operators
	//////////////////////////////////////////////////////////////////
	
	
	
	
	cflimage&  cflimage::operator= (const cflimage& im)
	{
		if (&im == this)
		{
			return *this;
		}
		
		
		if (d_c != im.d_c || d_w != im.d_w || d_h != im.d_h)
		{
			erase();
			d_c = im.d_c; d_w = im.d_w; d_h=im.d_h; d_wh = d_w * d_h; d_whc=d_c * d_w * d_h;
			d_v = new float[d_whc];
		}
		
		
		memcpy((void *) d_v, (const void *) im.d_v, d_whc * sizeof(float));
		
		visuMin = im.visuMin;
		visuMax = im.visuMax;
		
		return *this;
		
	}
	
	
	
	
	
	
	
    
	//////////////////////////////////////////////////////////////////
	//! Begin Load/Save
	//////////////////////////////////////////////////////////////////
	
	
	
	void cflimage::load(const char *filename)
	{
		
		
		
		// erase current image
		erase();
		
		
		
		// look if it exists
        std::ifstream fileImage(filename);
        if (!fileImage.good())
		{
            std::cout << "... failed to read image " << filename << std::endl;
            exit(-1);
            
        } else fileImage.close();
		
        
		// try pmf format
        d_v = _load_pm(filename, d_c, d_w, d_h);
        if (d_v)
        {
            
            d_wh = d_w * d_h;
            d_whc = d_c * d_w * d_h;
            return;
        }
        
        
        
		// the sure bet
		d_v = iio_read_image_float_split(filename, &d_w, &d_h, &d_c);
		
		if (d_v)
		{
            if (d_c == 2) d_c = 1;
            //if (d_c > 3)  d_c = 3;
            
			d_wh = d_w * d_h;
			d_whc = d_c * d_w * d_h;
            
            
            
			return;
		}
		
		
		
        
		std::cout << "... failed to read image " << filename << std::endl;
		exit(-1);
		
		
	}
	
	
	
	void cflimage::save(const char *filename)
	{
		
		if (!d_v)
		{
			std::cout << "... failed to save image " << filename << std::endl;
			exit(-1);
		}
		
		std::string strname(filename);
		size_t pos = strname.find_last_of ('.');
		std::string extension = strname.substr(pos+1);
		
		if ( extension == "png" || extension == "PNG" || extension == "tif" || extension == "TIF" || extension == "tiff" || extension == "TIFF" )
		{
            
			iio_save_image_float_split((char*)filename, d_v, d_w, d_h, d_c);
			
		}
		else
		{
			
			if (_create_pm(filename,d_c,d_w,d_h,d_v) == 0)
            {
                std::cout << "... failed to save pmf image " << filename << std::endl;
                exit(-1);
                
            } else return;
			
		}
		
	}
	
	
    
    
    
    
	//////////////////////////////////////////////////////////////////
	//! Begin Get Basic Data
	//////////////////////////////////////////////////////////////////
	
	
	
	
	cflimage::operator  flimage()
	{
		return getGray();
	}
	
	
	
	
	
	int  cflimage::isSameSize(const  cflimage &inIm)
	{
		
		if (d_c != inIm.d_c || d_w != inIm.d_w || d_h != inIm.d_h) return 0;
		else return 1;
		
	}
	
	
	
	
	
	
	flimage cflimage::getChannel(int i)
	{
		
		assert(i < d_c);
		
		flimage image(d_w,d_h);
		
		for (int jj=0; jj < d_wh; jj++) image.d_v[jj] = d_v[ i * d_wh + jj];
		
		return image;
	}
	
	
    
    void cflimage::getChannel(int i, flimage *out)
	{
		
		assert(i < d_c);
        assert(d_v != NULL);
        
        if (out->d_w != d_w || out->d_h != d_h)
        {
            out->erase();
            out->create(d_w, d_h);
        }

    
		for (int jj=0; jj < d_wh; jj++) out->d_v[jj] = d_v[ i * d_wh + jj];
		
	
    }
	
	
	
	
	//////////////////////////////////////////////////////////////////
	//! Begin Math
	//////////////////////////////////////////////////////////////////
	
	
	
	cflimage& cflimage::operator= (float a)
	{
		if (d_v)  for (int j=0; j<d_whc ; j++) d_v[j] = a;
		
		return *this;
	}
	
	
	void cflimage::operator-= (float a)
	{
		if (d_v)  for (int j=0; j<d_whc ; j++) d_v[j] -= a;
		
	}
	
	
	void cflimage::operator+= (float a)
	{
		if (d_v)  for (int j=0; j<d_whc ; j++) d_v[j] += a;
		
	}
	
	
	void cflimage::operator*= (float a)
	{
		if (d_v)  for (int j=0; j<d_whc ; j++) d_v[j] *= a;
		
	}
	
	
	
	
	float  cflimage::max ()
	{
		
		assert(d_v != NULL);
		
		float fmax = d_v[0];
		for (int j=0; j < d_whc ; j++)  if (d_v[j] > fmax)  fmax = d_v[j];
		return fmax;
		
		
	}
	
	
	
	float  cflimage::min ()
	{
		assert(d_v != NULL);
		
		float fmin = d_v[0];
		for (int j=0; j < d_whc ; j++)  if (d_v[j] < fmin)  fmin = d_v[j];
		return fmin;
	}
	
	
	
	float  cflimage::min_channel (int i)
	{
		assert(d_v != NULL);
		assert(i>= 0 && i < d_c);
		
		float *ptr = &d_v[i * d_wh];
		float fmin = *ptr;
		
		for (int j=0; j < d_wh ; j++, ptr++)  if (*ptr < fmin)  fmin = *ptr;
		return fmin;
	}
	
	
	
	float  cflimage::max_channel (int i)
	{
		assert(d_v != NULL);
		assert(i>= 0 && i < d_c);
		
		float *ptr = &d_v[i * d_wh];
		float fmax = *ptr;
		
		for (int j=0; j < d_wh ; j++, ptr++)  if (*ptr > fmax)  fmax = *ptr;
		return fmax;
	}
	
	
	
	void  cflimage::min (float m)
	{
		assert(d_v != NULL);
		for (int j=0; j < d_whc ; j++)  if (d_v[j] > m)  d_v[j] = m;
	}
	
	
	
	void  cflimage::max (float M)
	{
		assert(d_v != NULL);
		for (int j=0; j < d_whc ; j++)  if (d_v[j] < M)  d_v[j] = M;
	}
	
	
	
	
	void cflimage::thre(float m, float M)
	{
		
		assert(d_v != NULL);
		for (int ii=0; ii < d_whc; ii++)
		{
			if (d_v[ii] >= M) 	d_v[ii]= M;
			else if (d_v[ii] <= m)  d_v[ii]= m;
		}
		
	}
	
	
	
	void cflimage::normalizeL1()
	{
		
		float fSum = 0.0f;
		for (int j=0; j < d_whc ; j++)    fSum += d_v[j];
		
		assert(fSum != 0.0f);
		float dfSum = 1.0 / fSum;
		for (int j=0; j < d_whc ; j++)    d_v[j] *= dfSum;
		
	}
	
	
	
	void cflimage::rint()
	{
		
		assert(d_v != NULL);
		for (int ii=0; ii < d_whc; ii++)		d_v[ii]= rintf(d_v[ii]);
		
	}
    
	
	void cflimage::abs()
	{
		
		assert(d_v != NULL);
		for (int ii=0; ii < d_whc; ii++)		d_v[ii]= fabsf(d_v[ii]);
		
	}
	
    
    
    
	
	///////////////////////////////////////////////
	//! Begin Color Conversion
	///////////////////////////////////////////////
	
	
	flimage cflimage::getGray()
	{
        
   
        
		flimage image(d_w, d_h);	image=0.0f;
		
		getGray(&image);
        
		return image;
	}
	

	
    
    
    void cflimage::getGray(flimage * out)
	{
        
        
       
        
        assert(d_v != NULL);
        if (out->d_w != d_w || out->d_h != d_h)
        {
            out->erase();
            out->create(d_w, d_h);
        }

		
		for (int i=0; i < d_whc; i++)
		{
			out->d_v[i % d_wh] += d_v[i];
		}
		
		for (int i=0; i < d_wh; i++)
			out->d_v[i] /= (float) d_c;
		
	}
	
    
	
	flimage cflimage::getGray(float wr, float wg, float wb)
	{
        
		//assert(d_c == 1  || d_c == 3);
		
		flimage image(d_w, d_h);	image=0.0f;
        
		getGray(wr,wg,wb,&image);
		
		return image;
	}
	
	
	
    
	
	void cflimage::getGray(float wr, float wg, float wb, flimage *out)
	{
		//assert(d_c == 1  || d_c == 3);
        assert(d_v != NULL);
        
        if (out->d_w != d_w || out->d_h != d_h)
        {
            out->erase();
            out->create(d_w, d_h);
        }
        
		      
		if (d_c == 1)  *out = (flimage) (*this);
		else
		{
			for (int i=0; i < d_wh; i++) out->d_v[i] = wr * d_v[i] + wg * d_v[d_wh + i] + wb * d_v[2*d_wh+i];
		}
		
		
	}
	
    
    
    int cflimage::isGray()
    {
        if (d_c == 1) return 1;
        
        for (int i=0; i < d_wh; i++)
        {
            float value = d_v[i];
            for (int j=1; j < d_c; j++)
            if ( d_v[ j * d_wh + i] != value  ) return 0;
        }
        
        return 1;
        
    }
	
	cflimage cflimage::binarize(float value, int inverse)
	{
		assert(d_v != NULL);
		cflimage binary(d_w,d_h,d_c);
		
        binarize(value, inverse, &binary);
        
		return binary;
	}
	
    

    
    void cflimage::binarize(float value, int inverse, cflimage *out)
	{
        
		assert(d_v != NULL);
        if (!isSameSize(*out))
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for (int ii=0; ii < d_whc; ii++)
        {
            if (d_v[ii] >= value && !inverse) 	out->d_v[ii]= 1.0;
            else if (d_v[ii] < value && inverse)  out->d_v[ii]= 1.0;
            else out->d_v[ii]= 0.0;
            
        }
        
    }
    
    
    
    
    
    

	
    
    void cflimage::Rgb2Yuv(int iflagOrto)
    {
        
        assert(d_c==3);
        float *r,*g,*b;
        r=d_v; g=&d_v[d_wh]; b=&d_v[2*d_wh];

        float vr, vg,vb;
        
        if (iflagOrto)
        {
            for(int i=0;i<d_wh;i++)
            {
                vr = r[i]; vg = g[i]; vb = b[i];
                
                r[i] =    0.577350 * vr + 0.577350 * vg + 0.577350  * vb;
                g[i] =    0.707106 * vr					- 0.707106  * vb;
                b[i] =    0.408248 * vr - 0.816496 * vg	+ 0.408248  * vb;
            }
        }
        else
        {
            for(int i=0;i<d_wh;i++)
            {
                vr = r[i]; vg = g[i]; vb = b[i];
                
                r[i] = ( 0.299 *  vr + 0.587 * vg + 0.114 * vb);
                g[i] =  ( vr - r[i]);
                b[i] =  ( vb - r[i]);
                
            }
        }
        
    }
    
    
 
    

    
    void  cflimage::Yuv2Rgb(int iflagOrto)
    {
        
        assert(d_c==3);
        assert(d_c==3);
        float *r,*g,*b;
        r=d_v; g=&d_v[d_wh]; b=&d_v[2*d_wh];
        
        float vr, vg,vb;
        
        if (iflagOrto)
        {
            for(int i=0;i<d_wh;i++)
            {
                vr = r[i]; vg = g[i]; vb = b[i];
                
                r[i] =    0.577350 * vr + 0.707106 * vg + 0.408248  * vb;
                g[i] =    0.577350 * vr					- 0.816496  * vb;
                b[i] =    0.577350 * vr - 0.707106 * vg	+ 0.408248  * vb;
            }
        }
        else
        {
            for(int i=0;i<d_wh;i++)
            {
                vr = r[i]; vg = g[i]; vb = b[i];
                
                g[i] =  ( vr - 0.299 * (vg + vr) - 0.114 * (vb +  vr) ) / 0.587;
                r[i] =  ( vg + vr);
                b[i] =  ( vb + vr);
                
            }
        }
        
        
        
    }
    
    
    
    

    
    
    
    
    //////////////////////////////////////////////////////////////////
    //! Begin Block Operations
    //////////////////////////////////////////////////////////////////
    
    

    cflimage cflimage::adjustResolution(int w, int h, float fValue)
    {
        
        cflimage image(w,h,d_c);
        image=fValue;
        int iwh = w*h;
        int mw = w; if (d_w < w) mw = d_w;
        int mh = h; if (d_h < h) mh = d_h;
        
        for (int ii=0; ii < d_c; ii++)
        {
            
            for(int j=0; j < mh; j++)
                for(int i=0; i < mw; i++)
                    image.d_v[ii * iwh + j * w + i] = d_v[ ii * d_wh + j * d_w + i];
            
        }
        
        return image;
    }
    
    
    cflimage cflimage::padding(int w, int h, float fValue)
    {
        
        assert(w >= d_w  && h >= d_h);
        
        cflimage image(w,h,d_c);
        image=fValue;
        
        for (int ii=0; ii < d_c; ii++)
        {
            
            for(int j=0; j < d_h; j++)
                for(int i=0; i < d_w; i++)
                    image.d_v[ii * image.d_wh + j * image.d_w + i] = d_v[ ii * d_wh + j * d_w + i];
            
        }
        
        return image;
    }
    
    
    
    
    
    
    cflimage cflimage::copy(int ipx, int ipy, int iw, int ih)
    {
        
        assert(iw>0 && ih>0);
        assert(ipx>=0 && ipy>=0);
        assert(ipx + iw - 1 < d_w && ipy + ih - 1 < d_h);
        
        cflimage image(iw, ih, d_c);
        
        int nn=0;
        for (int ii=0; ii < d_c; ii++)
        {
            
            int l = ii * d_wh +  ipy * d_w + ipx;
            
            for (int jj = 0; jj < ih; jj++)
            {
                
                for (int kk = 0; kk < iw; kk++,nn++,l++)
                    image[nn] = d_v[l];
                
                l += d_w - iw;
            }
            
            
        }
        
        return image;
    }
    
    
    
    
    void cflimage::paste(const cflimage &im, int ipx, int ipy)
    {
        
        assert(ipx>=0 && ipy>=0);
        assert(ipx + im.d_w - 1 < d_w && ipy + im.d_h - 1 < d_h);
        assert(d_c == im.d_c);
        
        
        for (int ii=0; ii < d_c; ii++)
        {
            
            
            int ll = ii * im.d_wh;
            int nn = ii * d_wh +  ipy * d_w + ipx;
            
            
            for (int jj = 0; jj < im.d_h; jj++)
            {
                
                for (int kk = 0; kk < im.d_w; ll++, nn++, kk++)
                    d_v[nn] = im.d_v[ll];
                
                nn += d_w - im.d_w;
            }
            
            
        }
        
    }
    
    
    
    
    
    
    cflimage cflimage::append(const  cflimage &imIn, int extension)
    {
        
        assert(d_c == imIn.d_c);
        
        //! create image
        cflimage image;
        if (extension == iipHorizontal)
            image.create(d_w + imIn.d_w, MAX(d_h, imIn.d_h), d_c);
        else
            image.create(MAX(d_w, imIn.d_w), d_h + imIn.d_h, d_c);
        
        
        image = 0.0f;
        image.paste(*this, 0, 0);
        if (extension == iipHorizontal)
            image.paste(imIn, d_w, 0);
        else
            image.paste(imIn, 0, d_h);
        
        return image;
    }
    
    
    
    
    
    //////////////////////////////////////////////////////////////////
    //! Begin Block Statistics
    //////////////////////////////////////////////////////////////////

    
    
    void cflimage::patchStatistics(float fRadius, cflimage *mean, cflimage *median, cflimage *var, cflimage *min, cflimage *max)
    {
        if (mean != NULL && (mean->d_w != d_w || mean->d_h != d_h || mean->d_c != d_c))
        {
            mean->erase();
            mean->create(d_w, d_h, d_c);
        }
        
    
        if (median != NULL && (median->d_w != d_w || median->d_h != d_h || median->d_c != d_c))
        {
            median->erase();
            median->create(d_w, d_h, d_c);
        }
    
        
        if (var != NULL && (var->d_w != d_w || var->d_h != d_h || var->d_c != d_c))
        {
            var->erase();
            var->create(d_w, d_h, d_c);
        }
        
        
        if (min != NULL && (min->d_w != d_w || min->d_h != d_h || min->d_c != d_c))
        {
            min->erase();
            min->create(d_w, d_h, d_c);
        }
        
        
        
        if (max != NULL && (max->d_w != d_w || max->d_h != d_h || max->d_c != d_c))
        {
            max->erase();
            max->create(d_w, d_h, d_c);
        }
        
        
        
        for (int i=0; i < d_c; i++)
        {
            
            float *fpmean = NULL;
            if (mean) fpmean = &(mean->d_v[i*d_wh]);

            float *fpmedian = NULL;
            if (median) fpmedian = &(median->d_v[i*d_wh]);

            float *fpmin = NULL;
            if (min) fpmin = &(min->d_v[i*d_wh]);

            float *fpmax = NULL;
            if (max) fpmax = &(max->d_v[i*d_wh]);

            float *fpvar = NULL;
            if (var) fpvar = &(var->d_v[i*d_wh]);

            
            fiPatchStatistics(&d_v[i*d_wh], fpmin, fpmax, fpmean, fpvar, fpmedian, fRadius,d_w, d_h);
            
        }
    }

    
    
    void  cflimage::patchMean(float fRadius, cflimage *out)
    {
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for (int i=0; i < d_c; i++)
        {
            fiPatchMean( &d_v[i*d_wh], &(out->d_v[i*d_wh]), fRadius, d_w, d_h);
        }
        
    }
    
    
    cflimage  cflimage::patchMean(float fRadius)
    {
        cflimage image(d_w, d_h, d_c);
        patchMean(fRadius, &image);
        return image;
    }
    
    
    void  cflimage::patchVar(float fRadius, cflimage *out)
    {
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        for (int i=0; i < d_c; i++)
		{
			fiPatchVar( &d_v[i*d_wh], &(out->d_v[i*d_wh]), fRadius, d_w, d_h);
			
		}
        
    }

    cflimage  cflimage::patchVar(float fRadius)
    {
        cflimage image(d_w, d_h, d_c);
        patchVar(fRadius, &image);
        return image;
    }

    
    
    
    
    void  cflimage::patchMin(float fRadius, cflimage *out)
    {
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for (int i=0; i < d_c; i++)
		{
			fiPatchMin( &d_v[i*d_wh], &(out->d_v[i*d_wh]), fRadius, d_w, d_h);
			
		}
        
    }

    
    cflimage  cflimage::patchMin(float fRadius)
    {
        cflimage image(d_w, d_h, d_c);
        patchMin(fRadius, &image);
        return image;
        
    }

    
    void  cflimage::patchMax(float fRadius, cflimage *out)
    {
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for (int i=0; i < d_c; i++)
        {
            fiPatchMax( &d_v[i*d_wh], &(out->d_v[i*d_wh]), fRadius, d_w, d_h);
            
        }
        
    }
    
    
    cflimage  cflimage::patchMax(float fRadius)
    {
        cflimage image(d_w, d_h, d_c);
        patchMax(fRadius, &image);
        return image;
        
    }

    
    
    
    void  cflimage::patchMedian(float fRadius, cflimage *out)
    {
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for (int i=0; i < d_c; i++)
        {
            fiPatchMedian( &d_v[i*d_wh], &(out->d_v[i*d_wh]), fRadius, d_w, d_h);
            
        }
        
    }
    
    
    cflimage  cflimage::patchMedian(float fRadius)
    {
        cflimage image(d_w, d_h, d_c);
        patchMedian(fRadius, &image);
        return image;
        
    }
    
    
    
    
    
    
    //////////////////////////////////////////////////////////////////
    //! Begin Gradient
    //////////////////////////////////////////////////////////////////

    
    cflimage  cflimage::gradient(char cType)
    {
        assert(d_v != NULL);
        
        cflimage grad(d_w, d_h, d_c);
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], &grad.d_v[ii * d_wh], NULL, d_w, d_h,cType);
        }
        
        return grad;
    }
    
    
    
    cflimage  cflimage::xgradient(char cType)
    {
        assert(d_v != NULL);
        
        cflimage grad(d_w, d_h, d_c);
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], &grad.d_v[ii * d_wh], NULL, NULL, NULL, d_w, d_h,cType);
        }
        
        return grad;
    }

    
    
    cflimage  cflimage::ygradient(char cType)
    {
        assert(d_v != NULL);
        
        cflimage grad(d_w, d_h, d_c);
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], NULL, &grad.d_v[ii * d_wh], NULL, NULL, d_w, d_h,cType);
        }
        
        return grad;
    }
    
    
    void  cflimage::xyderivatives(char cType, cflimage *xder, cflimage *yder)
    {
        
        if (xder->d_w != d_w || xder->d_h != d_h || xder->d_c != d_c)
        {
            xder->erase();
            xder->create(d_w, d_h, d_c);
        }
 
        if (yder->d_w != d_w || yder->d_h != d_h || yder->d_c != d_c)
        {
            yder->erase();
            yder->create(d_w, d_h, d_c);
        }
        
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], &(xder->d_v[ii * d_wh]), &(yder->d_v[ii * d_wh]), NULL, NULL, d_w, d_h,cType);
        }
        
    }
    
    
    
    cflimage  cflimage::gradient(cflimage &orientation, char cType)
    {
        assert(d_v != NULL);
        
        cflimage grad(d_w, d_h, d_c);
        if (!orientation.isSameSize(grad)) {orientation.erase(); orientation.create(d_w, d_h, d_c);}
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], &grad.d_v[ii * d_wh], &orientation.d_v[ii * d_wh], d_w, d_h,cType);
        }
        
        return grad;
    }
    
    
    
    cflimage  cflimage::gradient(cflimage &xgrad, cflimage &ygrad, cflimage &orientation, char cType)
    {
        assert(d_v != NULL);
        
        cflimage grad(d_w, d_h, d_c);
        if (!orientation.isSameSize(grad)) {orientation.erase(); orientation.create(d_w, d_h, d_c);}
        if (!xgrad.isSameSize(grad)) {xgrad.erase(); xgrad.create(d_w, d_h, d_c);}
        if (!ygrad.isSameSize(grad)) {ygrad.erase(); ygrad.create(d_w, d_h, d_c);}
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiComputeImageGradient(&d_v[ii * d_wh], &xgrad.d_v[ii * d_wh], &ygrad.d_v[ii * d_wh], &grad.d_v[ii * d_wh], &orientation.d_v[ii * d_wh], d_w, d_h,cType);
        }
        
        return grad;
    }
    
    
    
    
    
    
    
    
    ///////////////////////////////////////////////
    //! Begin Sampling and Convolution
    ///////////////////////////////////////////////
    
    
    
    
    
    
    void cflimage::subSample(int fFactor, cflimage *out)
    {
        
        assert(d_v != NULL);
        
        int sd_w = (int) floor((float) d_w / (float) fFactor);
        int sd_h = (int) floor((float) d_h / (float) fFactor);
        int sd_wh = sd_w * sd_h;
        
        
        if (out->d_w != sd_w || out->d_h != sd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(sd_w, sd_h, d_c);
        }
        
        
        
        for (int ii=0; ii < d_c; ii++)
        {
            for(int j=0; j < sd_h; j++)
                for(int i=0; i < sd_w; i++)
                    out->d_v[ ii * sd_wh + j * sd_w + i] = d_v[ ii*d_wh + j * fFactor * d_w +  i*fFactor ];
        }
        
    }
    
    
    
    
    cflimage cflimage::subSample(int fFactor)
    {
        
        assert(d_v != NULL);
        
        int sd_w = (int) floor((float) d_w / (float) fFactor);
        int sd_h = (int) floor((float) d_h / (float) fFactor);
        
        
        cflimage image(sd_w, sd_h, d_c);
        
        subSample(fFactor, &image);
        
        return image;
    }
    
    
    
    
    void cflimage::subSampleAglomeration(int iFactor, cflimage *out)
    {
        
        assert(d_v != NULL);
        
        int sd_w = (int) floor((float) d_w / (float) iFactor);
        int sd_h = (int) floor((float) d_h / (float) iFactor);
        //int sd_wh = sd_w * sd_h;
        
        
        if (out->d_w != sd_w || out->d_h != sd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(sd_w, sd_h, d_c);
        }
        
        
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiImageSampleAglomeration(&d_v[ii*d_wh], &(out->d_v[ii*out->d_wh]), iFactor, d_w, d_h);
        }
        
        
    }
    
    
    cflimage cflimage::subSampleAglomeration(int iFactor)
    {
        
        assert(d_v != NULL);
        
        int sd_w = (int) floor((float) d_w / (float) iFactor);
        int sd_h = (int) floor((float) d_h / (float) iFactor);
        
        
        cflimage image(sd_w, sd_h, d_c);
        
        
        for (int ii=0; ii < d_c; ii++)
        {
            fiImageSampleAglomeration(&d_v[ii*d_wh], &image.d_v[ii*image.d_wh], iFactor, d_w, d_h);
        }
        
        
        return image;
    }
    
    
    
    
    
    cflimage cflimage::subSampleConv(int fFactor, int boundary, float fSigma)
    {
        
        cflimage convolved;
        convolveGauss(fSigma, boundary, &convolved);
        
        cflimage image = convolved.subSample(fFactor);
        
        return image;
    }
    
    
    void cflimage::subSampleConv(int fFactor, int boundary, float fSigma, cflimage *out)
    {
        
        cflimage convolved;
        convolveGauss(fSigma, boundary, &convolved);
        
        convolved.subSample(fFactor, out);
        
    }
    
    
    
    void cflimage::convolveGauss(float fSigma, int boundary, cflimage *out)
    {
        
        assert(d_v != NULL);
        if (!isSameSize(*out))
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        int ksize;
        float *kernel;
        kernel = fiFloatGaussKernel(fSigma,ksize);
        
        
        for (int i=0; i < d_c; i++)
        {
            fiFloatHorizontalConvolution( &d_v[i*d_wh], &(out->d_v[i*d_wh]), d_w, d_h, kernel, ksize, boundary);
            fiFloatVerticalConvolution( &(out->d_v[i*d_wh]), &(out->d_v[i*d_wh]), d_w, d_h, kernel,  ksize, boundary);
        }
        
        
        delete[] kernel;
    }
    
    
    
    
    void cflimage::convolve(const flimage &kernel, int boundary, cflimage *out)
    {
        assert(d_v != NULL);
        if (!isSameSize(*out))
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        for (int i=0; i < d_c; i++)
        {
            fiConvol( &d_v[i*d_wh], &(out->d_v[i*d_wh]), d_w, d_h, kernel.d_v, kernel.d_w, kernel.d_h, boundary);
            
        }
        
    }
    
    
    
    
    ///////////////////////////////////////////////
    //! Begin Zooming
    ///////////////////////////////////////////////
    
    void cflimage::upSampleNN(const float fFactor, cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactor * (float) d_w);
        int zd_h = (int)rintf( fFactor * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            nn_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactor, &(out->d_v[i * out->d_wh]));
        
        
    }
    
   
    
    
    void cflimage::upSampleBilinear(const float fFactor,  cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactor * (float) d_w);
        int zd_h = (int)rintf( fFactor * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bilinear_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactor, &(out->d_v[i * out->d_wh]));
        
    }

    void cflimage::upSampleBilinear(const float fFactorx,const float fFactory,  cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactorx * (float) d_w);
        int zd_h = (int)rintf( fFactory * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bilinear_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactorx,fFactory, &(out->d_v[i * out->d_wh]));
        
    }
    

    
    
    
    
    
    void cflimage::warpBicubic(flimage &u, flimage &v,  int binterpolation,  float  bvalue,  cflimage *out)
    {
        
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bicubic_interpolation_warp(&d_v[i * d_wh], u.d_v, v.d_v,  d_w, d_h, binterpolation, bvalue, &(out->d_v[i * out->d_wh]));
        
        
    }
  
    void cflimage::warpBilinear(flimage &u, flimage &v,  cflimage *out)
    {
        
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bilinear_interpolation_warp(&d_v[i * d_wh], u.d_v, v.d_v, &(out->d_v[i * out->d_wh]), d_w, d_h);
        
    }
    
    void cflimage::warpNN(flimage &u, flimage &v,  cflimage *out)
    {
        
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            nn_interpolation_warp(&d_v[i * d_wh], u.d_v, v.d_v,  &(out->d_v[i * out->d_wh]), d_w, d_h);
        
    }
    

    
    
    void cflimage::upSampleCubicSplines(const float fFactorx,const float fFactory, const int bintFlag, const float bValue, cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactorx * (float) d_w);
        int zd_h = (int)rintf( fFactory * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bicubic_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactorx, fFactory, bintFlag, bValue, &(out->d_v[i * out->d_wh]));
    }
    
    
    
    void cflimage::upSampleCubicSplines(const float fFactor, const int bintFlag, const float bValue, cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactor * (float) d_w);
        int zd_h = (int)rintf( fFactor * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bicubic_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactor, bintFlag, bValue, &(out->d_v[i * out->d_wh]));
        
        
    }
    

    
    void cflimage::translateCubicSplines(const float tx, const float ty, const int bintFlag, const float bValue, cflimage *out)
    {
        
        if (out->d_w != d_w || out->d_h != d_h || out->d_c != d_c)
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
            bicubic_interpolation_translation(&d_v[i * d_wh], tx, ty, d_w, d_h, bintFlag, bValue, &(out->d_v[i * out->d_wh]));
        
        
    }
    
    
    cflimage cflimage::upSampleCubicSplines(const float fFactor, const int bintFlag ,const float bValue)
    {
        
        
        int zd_w = (int)rintf( fFactor * (float) d_w);
        int zd_h = (int)rintf( fFactor * (float) d_h);
        
        cflimage image(zd_w, zd_h, d_c);
        
        upSampleCubicSplines(fFactor,  bintFlag, bValue, &image);
        
        return image;
        
    }
    
    
    
    //////////////////////////////////////////////////////////////////
    //! Begin value operations
    //////////////////////////////////////////////////////////////////
    
    void  cflimage::addGaussianNoise(float std)
    {
        assert(d_v != NULL);
        fpAddNoiseGaussian(d_v, d_v, std, 0, d_whc);
    }
    
    
    void  cflimage::addGaussianNoiseSD(float std, float sdstd)
    {
        assert(d_v != NULL);
        fpAddNoiseGaussianAfine(d_v, d_v, std, sdstd, 0, d_whc);
    }
    
    
    
    
    
    //////////////////////////////////////////////////////////////////
    //! Begin block processing
    //////////////////////////////////////////////////////////////////
    
    
    //! Patch statistics
    void  cflimage::patchMean(int rx, int ry, cflimage *out)
    {
    
        assert(d_v != NULL);
        if (!isSameSize(*out))
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        for (int i=0; i < d_c; i++)
        {
            fiPatchMean(&d_v[i*d_wh], &(out->d_v[i*d_wh]), rx, ry, d_w,d_h);
        }
        
    }
    
    
    
    
    float distanceL2(const  cflimage &input1, const  cflimage &input2, int ipx, int ipy, int iqx, int iqy)
    {
        
        assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
        assert(ipx < input1.w() && ipy  < input1.h() && iqx  < input2.w() && iqy  < input2.h() );
        
        float fDif = 0.0f;
        float fDist = 0.0f;
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            fDif = *ptr1 - *ptr2;
            fDist += fDif * fDif;
            
        }
        
        return fDist;
    }
    
    
    
    
    float distancePatchL2_NN(const  cflimage &input1,const  cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h)
    {
        
        assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
        assert(ipx + r_w -1 < input1.w() && ipy + r_h - 1 < input1.h() && iqx + r_w - 1 < input2.w() && iqy + r_h - 1 < input2.h() );
        
        
        float fDist = 0.0f;
        float dif = 0.0f;
        
        int inc1 = input1.d_w - r_w;
        int inc2 = input2.d_w - r_w;
        
        
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            
            for (int jj = 0; jj < r_h; jj++)
            {
                
                for (int kk = 0; kk < r_w; kk++,ptr1++, ptr2++)
                {
                    dif = *ptr1 - *ptr2;
                    fDist += dif * dif;
                }
                
                
                ptr1 += inc1;
                ptr2 += inc2;
                
            }
            
            
            
        }
        
        return fDist;
    }
    
    
    
    float distancePatchL2(const  cflimage &input1,const  cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h)
    {
        
        float fSum = 1.0 / ((float) input1.d_c * (float) r_w * (float) r_h);
        float fDist = distancePatchL2_NN(input1,input2,  ipx,  ipy,  iqx,  iqy,  r_w,  r_h);
        return fSum * fDist;
    }
    
    
    
    

    
    
    float distancePatchL2M_NN(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h, cflimage &mean1, cflimage &mean2)
    {
        
        
        int sph = r_h / 2;
        int spw = r_w / 2;
        
        float fDist = 0.0f;
        float dif = 0.0f;
        
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
            float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
            float fMean = -fMean1 + fMean2;
            
            
            
            
            for (int jj = 0; jj < r_h; jj++)
            {
                
                for (int kk = 0; kk < r_w; kk++,ptr1++, ptr2++)
                {
                    dif = *ptr1 - *ptr2 + fMean;
                    fDist += dif * dif;
                }
                
                
                ptr1 += input1.d_w - r_w;
                ptr2 += input2.d_w - r_w;
                
            }
            
            
            
        }
        
        
        return fDist;
    }

    
    
    float distancePatchL2M_NN_tmp(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h, cflimage &mean1, cflimage &mean2, int sph, int spw)
    {
        
        float fDist = 0.0f;
        float dif = 0.0f;
        
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
            float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
            float fMean = -fMean1 + fMean2;
            
            
            
            
            for (int jj = 0; jj < r_h; jj++)
            {
                
                for (int kk = 0; kk < r_w; kk++,ptr1++, ptr2++)
                {
                    dif = *ptr1 - *ptr2 + fMean;
                    fDist += dif * dif;
                }
                
                
                ptr1 += input1.d_w - r_w;
                ptr2 += input2.d_w - r_w;
                
            }
            
            
            
        }
        
        
        return fDist;
    }
    
    
    
    
    
    float distancePatchL2M(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h, cflimage &mean1, cflimage &mean2)
    {
        //printf("computing normalized distance (%d,%d) (%d,%d) \n",ipx,ipy, iqx,iqy);
        
        float fSum = 1.0 / ((float) input1.d_c * (float) r_w * (float) r_h);
        float fDist = distancePatchL2M_NN(input1, input2,ipx, ipy,  iqx,  iqy,  r_w,  r_h, mean1, mean2);
        return fSum * fDist;
    }
    
    
    
    //! Non normalized distance
    float distancePatchL2_NN_Thresh(const  cflimage &input1,const  cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h, float fThresh)
    {
        
        assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
        assert(ipx + r_w -1 < input1.w() && ipy + r_h - 1 < input1.h() && iqx + r_w - 1 < input2.w() && iqy + r_h - 1 < input2.h() );
        
        float fDist = 0.0f;
        float dif = 0.0f;
        
        int inc1 = input1.d_w - r_w;
        int inc2 = input2.d_w - r_w;
        
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            for (int jj = 0; jj < r_h && fDist < fThresh; jj++)
            {
                
                for (int kk = 0; kk < r_w; kk++,ptr1++, ptr2++)
                {
                    dif = *ptr1 - *ptr2;
                    fDist += dif * dif;
                }
                
                
                ptr1 += inc1;
                ptr2 += inc2;
                
            }
            
            
            
        }
        
        
        return fDist;
    }
    
    
    float distancePatchL2M_NN_Thresh(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, int r_w, int r_h, cflimage &mean1, cflimage &mean2, float fThresh)
    {
        
       
        
        int sph = r_h / 2;
        int spw = r_w / 2;
        
        float fDist = 0.0f;
        float dif = 0.0f;
        //float fSum = 1.0 / ((float) input1.d_c * (float) r_w * (float) r_h);
        
        
        for (int ii=0; ii < input1.d_c; ii++)
        {
            
            float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx];
            float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx];
            
            float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
            float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
            float fMean = -fMean1 + fMean2;
            
            
            
            
            for (int jj = 0; jj < r_h && fDist < fThresh; jj++)
            {
                
                for (int kk = 0; kk < r_w; kk++,ptr1++, ptr2++)
                {
                    dif = *ptr1 - *ptr2 + fMean;
                    fDist += dif * dif;
                }
                
                
                ptr1 += input1.d_w - r_w;
                ptr2 += input2.d_w - r_w;
                
            }
            
            
            
        }
        
        
        //return fSum * fDist;
        return fDist;
    }
    

    
    
  
    
    
 
    
    
    
    
    
    
    
    
    //////////////////////
    //! Non revised stuff
    //////////////////////
    
    
    
    
    
    
    
    
    
    
	cflimage  cflimage::patchMean(flimage &kernel)
	{
		
		cflimage image(d_w, d_h, d_c);
		image = *this;
		
		int spd_w = kernel.d_w / 2;
		int spd_h = kernel.d_h / 2;
		int boundary =  MAX(kernel.d_h, kernel.d_w) + 1;
		
		
		for (int ipx = 0; ipx < d_w - boundary; ipx++)
			for (int ipy = 0; ipy < d_h - boundary ; ipy++)
			{
				
				for (int iC=0; iC < d_c; iC++)
				{
					
					float fMean = 0.0f;
					float *ptr = &d_v[ iC * d_wh + ipy *  d_w + ipx];
					float *ptrk = kernel.d_v;
					
					for (int s = 0; s < kernel.d_h; s++)
					{
						
						for(int r = 0 ; r < kernel.d_w; r++, ptr++, ptrk++)
						{
							fMean += *ptrk * (*ptr);
						}
						
						ptr += d_w - kernel.d_w;
					}
					
					
					image[iC * d_wh + (ipy + spd_h) * d_w + ipx + spd_w] = fMean;
					
				}
				
			}
		
		return image;
	}
	
	
	
	
	
	
	
	
	
	cflimage  cflimage::patchVar(flimage &kernel)
	{
		
		cflimage image(d_w, d_h, d_c);
		image = 0.0f;
		
		
		int spd_w = (kernel.d_w - 1) / 2;
		int spd_h = (kernel.d_h - 1) / 2;
		int boundary = MAX(spd_w, spd_h) + 1;
		
		
		for (int ipx = boundary; ipx < d_w - boundary; ipx++)
			for (int ipy = boundary; ipy < d_h - boundary ; ipy++)
			{
				
				for (int iC=0; iC < d_c; iC++)
				{
					
					float fMean = 0.0f;
					float fMean2 = 0.0f;
					
					float *ptr = &d_v[ iC * d_wh + (ipy - spd_h ) *  d_w + (ipx - spd_w)];
					float *ptrk = kernel.d_v;
					
					for (int s = 0; s < kernel.d_h; s++)
					{
						
						for(int r = 0 ; r < kernel.d_w; r++, ptr++, ptrk++)
						{
							fMean += *ptrk * (*ptr);
							fMean2 += *ptrk * (*ptr) * (*ptr);
						}
						
						
						ptr += d_w - kernel.d_w;
						
					}
					
					image[iC * d_wh + ipy * d_w + ipx] = fMean2 - fMean*fMean;
					
				}
				
			}
		
		return image;
	}
	

        
    
    
    
    
    
    
    
    
	float distancePatchWL2(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		float fDist = 0.0f;
		float dif = 0.0f;
        
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist / (float) input1.d_c;
	}
	
	
    
	float distancePatchWL2_NN(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		float fDist = 0.0f;
		float dif = 0.0f;
        
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
	}
	
	
    float distancePatchWL2_NN_Thresh(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel, float fThresh)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		
		float fDist = 0.0f;
		float dif = 0.0f;
        
        
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h && fDist < fThresh; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
	}
	

    
    
    
    float distancePatchWL2_NN_Thresh_Rob(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel, float fThresh, float fMinDist)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		assert(ipx >=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		float fMinDist2 = fMinDist * fMinDist;
		float fDist = 0.0f;
		float dif = 0.0f;
        
        
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h && fDist < fThresh; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2;
                    dif *= dif;
                    if (dif>fMinDist2)
                        fDist += *ptrk * fMinDist2;
                    else
                       fDist += *ptrk * dif * dif;
                    
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
	}
	

    
    
    
    
    
	
	
	
    
    
	
	
	
	float distancePatchWL2M(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel,  cflimage &mean1, cflimage &mean2)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		int sph = pd_h / 2 ;
		int spw = pd_w / 2 ;
		
        
        if (ipx < 0 || ipy < 0 || iqx < 0 || iqy < 0) printf("%d %d %d %d\n", ipx,ipy,iqx,iqy);
        
		assert(ipx>=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		
		float fDist = 0.0f;
		float dif = 0.0f;
		
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
			float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
			float fMean = -fMean1 + fMean2;
			
			
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2 + fMean;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist / (float) input1.d_c;
        
	}
	
    
    float distancePatchWL2M_NN(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel,  cflimage &mean1, cflimage &mean2)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		int sph = pd_h / 2 ;
		int spw = pd_w / 2 ;
		
        
        if (ipx < 0 || ipy < 0 || iqx < 0 || iqy < 0) printf("%d %d %d %d\n", ipx,ipy,iqx,iqy);
        
		assert(ipx>=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		
		float fDist = 0.0f;
		float dif = 0.0f;
		
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
			float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
			float fMean = -fMean1 + fMean2;
			
			
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2 + fMean;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
        
	}
    

    
    float distancePatchWL2M_NN_Rob(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel,  cflimage &mean1, cflimage &mean2, float fMinDist)
	{
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		int sph = pd_h / 2 ;
		int spw = pd_w / 2 ;
		
        
        if (ipx < 0 || ipy < 0 || iqx < 0 || iqy < 0) printf("%d %d %d %d\n", ipx,ipy,iqx,iqy);
        
		assert(ipx>=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		
		
		float fMinDist2 = fMinDist * fMinDist;
		float fDist = 0.0f;
		float dif = 0.0f;
		
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
			float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
			float fMean = -fMean1 + fMean2;
			
			
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2 + fMean;
                    dif *= dif;
                    
                    if (dif < fMinDist2)
                        fDist += *ptrk * dif * dif;
                    else
                        fDist += *ptrk * fMinDist2;
                                        
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
        
	}
    

    
    
    
    
       float distancePatchWL2M_NN_Thresh(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel,  cflimage &mean1, cflimage &mean2, float fThresh)
    {
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		int sph = pd_h / 2 ;
		int spw = pd_w / 2 ;
		
        
        if (ipx < 0 || ipy < 0 || iqx < 0 || iqy < 0) printf("%d %d %d %d\n", ipx,ipy,iqx,iqy);
        
		assert(ipx>=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		
		float fDist = 0.0f;
		float dif = 0.0f;
		
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
			float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
			float fMean = -fMean1 + fMean2;
			
			
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h && fDist < fThresh; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2 + fMean;
					fDist += *ptrk * dif * dif;
				}
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
        
	}
    
    
    
    
    float distancePatchWL2M_NN_Thresh_Rob(cflimage &input1, cflimage &input2, int ipx, int ipy, int iqx, int iqy, flimage &kernel,  cflimage &mean1, cflimage &mean2, float fThresh, float fMinDist)
    {
		
		int pd_w = kernel.d_w ;
		int pd_h = kernel.d_h ;
		
		int sph = pd_h / 2 ;
		int spw = pd_w / 2 ;
		
        
        if (ipx < 0 || ipy < 0 || iqx < 0 || iqy < 0) printf("%d %d %d %d\n", ipx,ipy,iqx,iqy);
        
		assert(ipx>=0 && ipy>=0 && iqx>=0 && iqy>=0);
		assert(ipx + pd_w < input1.w() && ipy + pd_h < input1.h() && iqx + pd_w < input2.w() && iqy + pd_h < input2.h() );
		
		
		float fMinDist2 = fMinDist * fMinDist;
		float fDist = 0.0f;
		float dif = 0.0f;
		
		for (int ii=0; ii < input1.d_c; ii++)
		{
			
			float *ptr1 = &input1.d_v[ii * input1.d_wh +  ipy  * input1.d_w + ipx ];
			float *ptr2 = &input2.d_v[ii * input2.d_wh +  iqy  * input2.d_w + iqx ];
			
			float fMean1 = mean1.d_v[ii * input1.d_wh +  (ipy + sph) * input1.d_w + ipx + spw];
			float fMean2 = mean2.d_v[ii * input2.d_wh +  (iqy + sph) * input2.d_w + iqx + spw];
			float fMean = -fMean1 + fMean2;
			
			
			
			float *ptrk = kernel.d_v;
			
			for (int jj = 0; jj < kernel.d_h && fDist < fThresh; jj++)
			{
				
				for (int kk = 0; kk < kernel.d_w; kk++,ptr1++, ptr2++, ptrk++)
				{
					dif = *ptr1 - *ptr2 + fMean;
                    dif *= dif;
                    
                    if (dif < fMinDist2)
                        fDist += *ptrk * dif * dif;
                    else
                        fDist += *ptrk * fMinDist2;
                    
                }
				
				
				ptr1 += input1.d_w - kernel.d_w;
				ptr2 += input2.d_w - kernel.d_w;
				
			}
			
			
			
		}
		
		
		return  fDist;
        
	}
    
    
    ///////////////////////////////////////////////
	//! Begin Geometrical transforms
	///////////////////////////////////////////////
	
	cflimage cflimage::mirror(int Orientation)
	{
		
		cflimage image;
		if (Orientation == iipHorizontal)
		{
			image.create(2 * d_w, d_h, d_c);
			
			for (int ic = 0; ic < d_c; ic++)
			{
				float *iptr = &d_v[ic * d_wh];
				float *optr = &image.d_v[ic * image.d_wh];
				
				for (int ij = 0; ij < d_h; ij++)
					for (int ii = 0; ii < d_w; ii++)
					{
						optr[ ij * 2 * d_w + ii] =  iptr[ ij * d_w + ii];
						optr[ ij * 2 * d_w + 2 * d_w - 1 -ii] =  iptr[ ij * d_w + ii];
					}
			}
			
			
		}else
		{
			image.create(d_w, 2 * d_h, d_c);
			
			for (int ic = 0; ic < d_c; ic++)
			{
				float *iptr = &d_v[ic * d_wh];
				float *optr = &image.d_v[ic * image.d_wh];
				
				for (int ij = 0; ij < d_h; ij++)
					for (int ii = 0; ii < d_w; ii++)
					{
						optr[ ij * d_w + ii] =  iptr[ ij * d_w + ii];
						optr[ (2 * d_h -1 - ij) * d_w + ii] =  iptr[ ij * d_w + ii];
					}
			}
			
		}
		
		
		return image;
		
	}
	

	
    
    
    
    
    
	

    
    
    
   
    
    
    
    
    
    
    
    
    void cflimage::convolve_skernel(struct sorted_kernel *skernel, int boundary, cflimage *out)
    {
        assert(d_v != NULL);
        if (!isSameSize(*out))
        {
            out->erase();
            out->create(d_w, d_h, d_c);
        }
        
        for (int i=0; i < d_c; i++)
        {
            fiConvol_skernel( &d_v[i*d_wh], &(out->d_v[i*d_wh]), d_w, d_h, skernel, boundary);
            
        }
        
    }
    
    
    
    

    
    
    
    
    
  
    
    
    void cflimage::upSampleGenericSplines(const float fFactor, const int order, const float bValue, cflimage *out)
    {
        
        int zd_w = (int)rintf( fFactor * (float) d_w);
        int zd_h = (int)rintf( fFactor * (float) d_h);
        
        if (out->d_w != zd_w || out->d_h != zd_h || out->d_c != d_c)
        {
            out->erase();
            out->create(zd_w, zd_h, d_c);
        }
        
        
        for(int i=0; i < d_c ;i++)
        spline_interpolation_zoom(&d_v[i * d_wh], d_w, d_h,  fFactor, order, bValue, &(out->d_v[i * out->d_wh]));
        
    }
    
    
    
    

    
    
    
    
    
    
    
	
	cflimage cflimage::UpSampleFFT(float fFactorx, float fFactory)
	{
		
		int zd_w = (int) rintf( fFactorx * (float) d_w);
		int zd_h = (int) rintf( fFactory * (float) d_h);
		
		cflimage image(zd_w, zd_h, d_c);
		
		for(int i=0; i < d_c ;i++)
		{
			fiFFTZoom(&d_v[i * d_wh], &image.d_v[i * image.d_wh], fFactorx, fFactory, d_w, d_h);
		}
		
		return image;
	}
	
    
	
	cflimage cflimage::UpSampleFFT(float fFactorx)
	{
		cflimage image = (*this).UpSampleFFT(fFactorx, fFactorx);
		return image;
	}
	
	
	
	
	void cflimage::fftRot(float angle, float xtrans , float ytrans, int flagNoCenter, int flagSymmetric)
	{
		
		angle = -(angle * M_PI / 180.);
		
		// The rotation is decomposed into three shears, two horizontal and one vertical
		(*this).fftShear(  tan(angle * .5),  iipHorizontal, 0.0,   flagNoCenter, flagSymmetric);
		(*this).fftShear(- sin(angle     ),  iipVertical, -(ytrans),  flagNoCenter, flagSymmetric);
		(*this).fftShear(  tan(angle * .5),  iipHorizontal, -(xtrans),  flagNoCenter, flagSymmetric);
		
	}
	
	
	
	void cflimage::fftTrans(float xtrans,float ytrans, int flagNoCenter, int flagSymmetric)
	{
		
		if (ytrans != 0.0f)
			(*this).fftShear(0.0, iipVertical,  -ytrans, flagNoCenter, flagSymmetric);
		
		if (xtrans != 0.0f)
			(*this).fftShear(0.0, iipHorizontal,  -(xtrans), flagNoCenter, flagSymmetric);
		
	}
	
	
    
    
	void cflimage::fftShear(float dAmountOfShear, int iOrientation, float fDelta, int flagNoCenter, int flagSymmetric)
	{
		
		cflimage extended;
		if (!flagSymmetric)
			extended = *this;
		else
			extended = (*this).mirror(iOrientation);
		
		for (int i=0; i < d_c; i++)
		{
			fiFFTShearPascal((float) dAmountOfShear, &extended.d_v[ i * extended.d_wh],&extended.d_v[i*extended.d_wh], iOrientation,  fDelta,  flagNoCenter,extended.d_w, extended.d_h);
		}
		
		
		if (!flagSymmetric) (*this)=extended;
		else (*this)=extended.copy(0, 0, d_w, d_h);
		
	}
    
    
    
    
    
    
	/// Class flimage
	
	flimage::flimage() : cflimage()
	{
	}
	
	
	
	
	flimage::flimage(int w, int h) : cflimage(w, h, 1)
	{
	}
	
	
	flimage::flimage(int w, int h, float *ptr) : cflimage(w, h, 1)
	{
		memcpy(this->d_v, ptr, w * h * sizeof(float));
	}

    flimage::flimage(int w, int h, unsigned char *ptr) : cflimage(w, h, ptr)
    {
    }

    
	
	
	flimage::flimage(const flimage& im)
	: cflimage(im)
	{}
	
	
	
	
	flimage& flimage::operator=(const flimage & im)
	{
		cflimage::operator=(im);
		return *this;
	}
	
	
	
	
	void flimage::create(int w, int h)
	{
		cflimage::create(w,h,1);
	}
    
    
    void flimage::load(const char* filename)
    {
        cflimage image; image.load(filename);
        image.getGray(this);
    }
    

    
    
    
    cflmovie::cflmovie():
    d_n(0), filename(NULL), fileadress(NULL), writable(0), strings(NULL), pos(-1), outfile(0)
    {}
    
    
    
    
    cflmovie::cflmovie(const char* ifilename,  int inframes)
    :  d_n(inframes), filename(NULL), fileadress(NULL), writable(0), strings(NULL), pos(-1), outfile(0)
    {
        
        filename = new char[128];
        strcpy(filename,ifilename);
        
        
        outfile.open(filename);
        if (!outfile.is_open())
        {
            printf("cflmovie file not writtable\n");
            exit(-1);
        }
        
        outfile << "cflmovie" << std::endl;
        outfile << d_n << std::endl;
        
        
        writable = true;
        pos = -1;
        
    }
    
    
    
    cflmovie::cflmovie(const cflmovie & imovie)
	:  d_n(imovie.d_n), filename(NULL), fileadress(NULL), writable(imovie.writable), strings(NULL), pos(imovie.pos), outfile(0)
    {
		
        strcpy(filename, imovie.filename);
        strcpy(fileadress, imovie.fileadress);
        
        if (imovie.strings)
        {
            strings = new char*[d_n];
            for (int i=0; i < d_n; i++)
            {
                strcpy(strings[i], imovie.strings[i]);
            }
            
        }
        
        
    }
    
    
    
    cflmovie::cflmovie(const char * ifilename)
	:  d_n(0), filename(NULL), fileadress(NULL), writable(0), strings(NULL), pos(-1), outfile(0)
    {
        
        /// Reading
        writable = false; pos = -1;
        fileadress = new char[128];
        filename = new char[128];
        
        strcpy(filename,ifilename);
        
        
        
        std::ifstream file(ifilename);
        if (!file.is_open())
        {
            printf("cflmovie %s file not found or impossible to open\n", ifilename);
            exit(-1);
        }
        
        /// Reading format
        char  iinfo[128];
        file >> iinfo;
        
        if( strcmp(iinfo, "cflmovie") == 0)
        {
            
			file >> d_n;
			
			
			strings = new char*[d_n];
			for(int i=0; i < d_n; i++) strings[i] = new char[128];
            
			for(int i = 0; i < d_n ; i++ )
			{
				file >>iinfo;
				strcpy(strings[i],iinfo);
				
			}
			
            
			
        }else{
			
			printf("Error: not a correct cflmovie list file\n");
			exit(-1);
			
        }
		
    }
    
    
    void cflmovie::load(const char * ifilename)
    {
        
        /// Reading
        writable = false; pos = -1;
        fileadress = new char[128];
        filename = new char[128];
        
        strcpy(filename,ifilename);
        
        
        
        std::ifstream file(ifilename);
        if (!file.is_open())
        {
             printf("cflmovie %s file not found or impossible to open\n", ifilename);
            exit(-1);
        }
        
        /// Reading format
        char  iinfo[128];
        file >> iinfo;
        
        if( strcmp(iinfo, "cflmovie") == 0)
        {
            
            file >> d_n;
            
            strings = new char*[d_n];
            for(int i=0; i < d_n; i++) strings[i] = new char[128];
            
            for(int i = 0; i < d_n ; i++ )
            {
                file >>iinfo;
                strcpy(strings[i],iinfo);
                
            }

            
            
        }else{
            
            printf("Error: not a correct cflmovie list file\n");
            exit(-1);
            
        }
        
    }
    
    
    
    
    
    cflmovie& cflmovie::operator= (const cflmovie& im)
    {
        printf("warning :: using cflmovie operator = which is not defined properly\n");
        
        if (&im == this)
        {
            return *this;
        }
        
        return *this;
    }
    
    
    
    
    cflmovie::~cflmovie()
    {
        
        if (outfile.is_open()) outfile.close();
        
    }
    
    
    
    
    
    cflimage cflmovie::getframe(int fpos)
    {
        
        /*
         if (strcmp(fileadress,"") != 0)
         {
         
         char *imname = new char[128];
         strcpy(imname, fileadress);
         strcat(imname, "/");
         strcat(imname, strings[fpos]);
         
         cflimage image;
         image.load(imname);
         
         if (DEBUG) printf("......END get frame %d\n", fpos);
         return image;
         
         } else
         */
        //{
		
		cflimage image;
		
		
		image.load(strings[fpos]);
		return image;
		
        //}
        
        
    }
    
    
    
    
    
    void cflmovie::write(cflimage &frame)
    {
        
        if (writable)
        {
            
            pos++;
            
            char* imfilename = new char[128];
            strcpy(imfilename,filename);
            
            char buf[128];
            sprintf(buf, "%d", pos);
            
            strcat(imfilename, "_");
            strcat(imfilename, buf);
            //strcat(imfilename, ".png");
            
			
            frame.save(imfilename);
            
            outfile <<  imfilename << std::endl;
            
        }
        
    }
   

    
    
    
    flData3D::flData3D() : npoints(0), dx(NULL), dy(NULL), dz(NULL), dr(NULL), dg(NULL), db(NULL),  ok(false), color(false)
    {
        
    }
	
    
    
    flData3D::flData3D(flData3D &inData)
    : npoints(0), dx(NULL), dy(NULL), dz(NULL), dr(NULL), dg(NULL), db(NULL),  ok(false), color(false)
    {
        
        
        
        if (inData.npoints > 0){
            
            npoints = inData.npoints;
            allocate_coordinates(npoints);
            
            color = inData.color;
            if (color)
                allocate_color(npoints);
            else{
                dr = dg = db = NULL;
            }
            
            for(int i = 0; i < npoints; i++)
            {
                
                dx[i] = inData.dx[i]; dy[i] = inData.dy[i]; dz[i] = inData.dz[i];
                
                if (color){dr[i] = inData.dr[i]; dg[i] = inData.dg[i]; db[i] = inData.db[i];}
            }
            
            ok = true;
        }
        
    }
    
    
    
    flData3D::flData3D(int inpoints, float *idx, float *idy, float *idz, float *idr, float *idg, float *idb)
    :  npoints(0), dx(NULL), dy(NULL), dz(NULL), dr(NULL), dg(NULL), db(NULL),ok(false), color(false)
    {
        
        
        if (inpoints > 0)
        {
            
            npoints = inpoints;
            allocate_coordinates(npoints);
            
            if (idr && idg && idb) color = true;
            else color = false;
            
            if (color)
                allocate_color(npoints);
            else{
                dr = dg = db = NULL;
            }
            
            for(int i = 0; i < npoints; i++)
            {
                
                dx[i] = idx[i]; dy[i] = idy[i]; dz[i] = idz[i];
                
                if (color){dr[i] = idr[i]; dg[i] = idg[i]; db[i] = idb[i];}
            }
            
            ok = true;
        }
        
    }
    
    
    
    
    
    
    flData3D::flData3D(const char * filename)
    :  npoints(0), dx(NULL), dy(NULL), dz(NULL), dr(NULL), dg(NULL), db(NULL),ok(false), color(false)
    {
        npoints = 0;
        dx = dy = dz = NULL;
        dr = dg = db = NULL;
        ok = false;
        color = false;
        
        loadFile(filename);
        
    }
    
    
    
    void flData3D::allocate_coordinates(int n)
    {
        
        if (!dx) dx = new float[n];
        if (!dy) dy = new float[n];
        if (!dz) dz = new float[n];
        
    }
    
    void flData3D::allocate_color(int n)
    {
        
        if (!dr) dr = new float[n];
        if (!dg) dg = new float[n];
        if (!db) db = new float[n];
        
    }
    
    void flData3D::desallocate_coordinates()
    {
        
        if (!dx) {delete[] dx; dx = NULL;}
        if (!dy) {delete[] dy; dy = NULL;}
        if (!dz) {delete[] dz; dz = NULL;}
    }
    
    
    
    void flData3D::desallocate_color()
    {
        
        if (!dr) {delete[] dr; dr = NULL;}
        if (!dg) {delete[] dg; dg = NULL;}
        if (!db) {delete[] db; db = NULL;}
    }
    
    
    
    
    
    
    
	
    
    flData3D::~flData3D()
    {
        
        if (dx) delete[] dx;
        if (dy) delete[] dy;
        if (dz) delete[] dz;
        
        if (dr) delete[] dr;
        if (dg) delete[] dg;
        if (db) delete[] db;
        
    }
    
    
    
    flData3D& flData3D::operator= (const flData3D &in)
    {
        
        if (npoints != in.npoints)
        {
            desallocate_color();
            desallocate_coordinates();
            
        }
        
        npoints = in.npoints;
        
        if (color && !in.color) desallocate_color();
        
        allocate_coordinates(npoints); // It only allocates if pointers are NULL
        if (color) allocate_color(npoints);
        
        for(int i = 0; i < npoints; i++)
        {
            dx[i] = in.dx[i]; dy[i] = in.dy[i]; dz[i] = in.dz[i];
            if (color){dr[i] = in.dr[i]; dg[i] = in.dg[i]; db[i] = in.db[i];}
        }
        
        return *this;
        
        
    }
    
    
    
    
    void flData3D::loadFile(const char * filename)
    {
        
        std::ifstream file(filename);
        if (!file.is_open())
        {
            printf("Exit(flData3D::loadData(wxString filename)): Surface file not found or impossible to open\n");
            ok = false;
            return;
            
        }
        
		
        char  iinfo[256];
        file >> iinfo;
		
        if( strcmp(iinfo, "flData3D") == 0)
        {
            
            if (dx) delete[] dx;
            if (dy) delete[] dy;
            if (dz) delete[] dz;
            
            if (dr) delete[] dr;
            if (dg) delete[] dg;
            if (db) delete[] db;
            
            
            file >> npoints;
			
            dx = new float[npoints];
            dy = new float[npoints];
            dz = new float[npoints];
            
            file >>	iinfo;
            
            if (strcmp(iinfo,"color") != 0)
            {
                printf("Warning (flData3D::loadData(wxString filename)): Color flag not found, not a correct 3ddata file.\n");
                ok = false;
                return;
            }
            
            int colorflag;
            file >> colorflag;
            
            if (colorflag) color = true;
            else color = false;
            
            if (color)
            {
                dr = new float[npoints];
                dg = new float[npoints];
                db = new float[npoints];
            } else
            {
                dr = dg = db = NULL;
            }
			
			
            for(int i=0; i < npoints; i++)
            {
                file >> dx[i];
                file >> dy[i];
                file >> dz[i];
                
                
                if (color)
                {
                    file >> dr[i];
                    file >> dg[i];
                    file >> db[i];
                    
                }
            }
            
            ok = true;
			
        } else
        {
			
			printf("Error: not a correct flData3D file\n");
			ok = false;
			return ;
			
        }
		
        
    }
    
    
    
    
    
    
    
    
    void flData3D::SaveFile(const char* filename)
    {
        
        std::ofstream outfile(filename);
        
        if (!outfile.is_open())
        {
            printf("Data file not writtable\n");
            exit(-1);
        }
        
        outfile << "flData3D" << std::endl;
        outfile << npoints << std::endl;
        outfile << "color" << std::endl;
        if (color) outfile << "1" << std::endl;
        else outfile << "0" << std::endl;
        
        
        for(int i=0; i < npoints; i++)
        {
            
            if (color)
                outfile << dx[i] << "  " << dy[i] << " " << dz[i] << " " << dr[i]  << "  " << dg[i] << " " << db[i] << std::endl;
            else
                outfile << dx[i] << "  " << dy[i] << " " << dz[i] << std::endl;
            
        }
        
    }
    
    
    
    
    
    
    void flData3D::normalize()
    {
        
        
        // Move to baricenter
        float bx, by,bz;
        
        bx=by=bz=0.0;
        for(int i=0; i < npoints; i++)
        {
            bx += dx[i];
            by += dy[i];
            bz += dz[i];
            
        }
        
        bx /= npoints;
        by /= npoints;
        bz /= npoints;
        
        
        for(int i=0; i < npoints; i++)
        {
            
            dx[i] -= bx;
            dy[i] -= by;
            dz[i] -= bz;
            
        }
        
        
        
        // Changing range to [-0.5,0.5]
        float max = fabsf(dx[0]);
        for(int i=0; i < npoints; i++)
        {
            
            if (fabsf(dx[i]) > max)  max = fabsf(dx[i]);
            if (fabsf(dy[i]) > max)  max = fabsf(dy[i]);
            if (fabsf(dz[i]) > max)  max = fabsf(dz[i]);
            
        }
        
        max = 2.0*max;
        
        for(int i=0; i < npoints; i++)
        {
            
            dx[i] /= max;
            dy[i] /= max;
            dz[i] /= max;
            
        }
        
        
    }

    
    
    
    

}
