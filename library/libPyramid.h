//
//  libFusion.h
//  Sources
//
//  Created by antoni buades on 29/03/17.
//  Copyright (c) 2017 antoni buades. All rights reserved.
//

#include "../library/libBasic.h"
#include "../library/libImage.h"


namespace libUSTG
{
    
    void BuildGaussianPyramid(cflimage &input, cflimage *GPyr, int nLevels);
    void BuildLaplacianPyramid(cflimage &input, cflimage *LPyr, int nLevels);
    void ReconstructPyramidKeepPyr(cflimage &output, cflimage *LPyr, int nLevels);
    void ReconstructPyramidRewrPyr(cflimage &output, cflimage *LPyr, int nLevels);
    

}
