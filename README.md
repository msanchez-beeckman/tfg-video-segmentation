Project dependencies
===========

The project uses the following external libraries:
* OpenCV 4.2.0
* Eigen 3.3
* Boost

It is recommended to use the versions above, since all the testing has been performed using them.
Earlier versions of Eigen may work, but the project uses some calls from OpenCV's API that do not exist in version 2 and have not been tested in version 3.

To compile the code, the following software is required:
* CMake 3.5

Additionally, some scripts are provided to automatize the different segmentation processes. These require:
* Bash (or some other shell implementation that can interpret the scripts)
* Python
* ImageMagick

Contents
===========

The project currently contains an implementation for three different video segmentation strategies:

1. Alon Faktor and Michal Irani
   Video segmentation by non-local consensus voting,
   Proceedings of the British Machine Vision Conference.
   BMVA Press, 2014.
   DOI: 10.5244/C.28.21

2. N. Shankar Nagaraja, F. R. Schmidt, and T. Brox
   Video segmentation with just a few strokes,
   ICCV, 2015.
   DOI: 10.1129/ICCV.2015.370

3. S. Wehrwein and R. Szeliski
   Video segmentation with background motion models,
   BMVC, 2017.
   DOI: 10.5244/C.31.96

[1] Requires the computation of optical flows between images of the dataset. Inside the *external* folder there are three executables to do so: **src_flow_tv_l1** outputs the flows in two .tiff files (horizontal and vertical flow), and is much faster than **src_flow_ldof**, which outputs the flows in a single .flo file.
Since the project's NLC implementation requires two .tiff files, **src_flow_read_flo** is provided to transform a .flo file into two .tiff files. The two methods to compute the optical flow are described in the following references:

4. TV_L1:
   C. Zach, T. Pock, and H. Bischof
   A duality based approach for realtime TV-L1 optical flow,
   Pattern Recognition (Proc. of the 29th DAGM-Symposium),
   Lecture Notes in Computer Science. Springer, 2007.
   DOI: 10.1007/978-3-540-74936-3_22

5. LDOF:
   T. Brox, C. Bregler, and J. Malik
   Large displacement optical flow
   CVPR, 2009.
   DOI: 10.1109/CVPR.2009.5206697

[2] and [3] use trajectories to perform the segmentations, and require files containing tracked points. By default, track files are searched in */test/tracks*, are assumed to have been computed using **src_point_tracking_flow**, and must have the same name as the dataset they are referencing.
A second format for the track file is accepted, consisting in the output .dat file of Brox's GPU Accelerated Point Tracking. If used, its name must be datasetnameBrox.dat (e.g. bearBrox.dat). Below are given a reference to the paper and indications on where to download the tracker.

6. N. Sundaram, T. Brox, and K. Keutzer
   Dense point trajectories by GPU-accelerated large displacement optical flow,
   ECCV, 2010.
   DOI: 10.1007/978-3-642-15549-9_32
   The executable can be found in https://lmb.informatik.uni-freiburg.de/resources/software.php

Moreover, the video segmentation defined by [2] is semi-supervised, so seeds are needed for it to work. Those should be placed in *data/datasetname/seeds*, with each seed named after the image the seed is for (e.g. *data/bear/seeds/00000.png*).

Setup
===========

TODO

Scripts
===========

TODO

