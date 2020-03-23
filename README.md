# Collection of video segmentation algorithms

## Project dependencies

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

## Contents

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

[1] Requires the computation of optical flows between images of the dataset. Inside the **external/** folder there are three executables to do so: **src_flow_tv_l1** outputs the flows in two .tiff files (horizontal and vertical flow), and is much faster than **src_flow_ldof**, which outputs the flows in a single .flo file.
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

[2] and [3] use trajectories to perform the segmentations, and require files containing tracked points. By default, track files are searched in **/test/tracks/**, are assumed to have been computed using **src_point_tracking_flow**, and must have the same name as the dataset they are referencing.
A second format for the track file is accepted, consisting in the output .dat file of Brox's GPU Accelerated Point Tracking. If used, its name must be datasetnameBrox.dat (e.g. *bearBrox.dat*). Below are given a reference to the paper and indications on where to download the tracker.

6. N. Sundaram, T. Brox, and K. Keutzer  
   Dense point trajectories by GPU-accelerated large displacement optical flow,  
   ECCV, 2010.  
   DOI: 10.1007/978-3-642-15549-9_32

   The executable can be found in https://lmb.informatik.uni-freiburg.de/resources/software.php

Moreover, the video segmentation defined by [2] is semi-supervised, so seeds are needed for it to work. Those should be placed in **data/&lt;datasetname&gt;/seeds/**, with each seed named after the image the seed is for (e.g. *data/bear/seeds/00000.png*).

Within the project folders there are some datasets included for testing, located inside **data/**. Every frame from each dataset is named with a five digit number, starting with 00000, and this convention should be used for other video sequences, since the scripts that come with the project rely on it.

## Setup

To compile the code, run the following commands from the terminal while inside the project's root folder:

```console
[user@pc tfg_video_segmentation]$ mkdir build
[user@pc tfg_video_segmentation]$ cd build
[user@pc tfg_video_segmentation]$ cmake .. -DCMAKE_BUILD_TYPE=Release
[user@pc tfg_video_segmentation]$ make
```

If a **build/** folder already existed when you got the project, delete it and follow the previous steps, since the Makefile inside it won't work in your PC.

You may need to modify the file **CMakeLists.txt** a bit, as apparently some systems have trouble locating OpenCV. If that's the case for you, add the following line to the file, just before trying to find OpenCV:

```cmake
set(OpenCV_DIR /path/to/OpenCV)
find_package(OpenCV REQUIRED)
```

After compiling the code, the executables will be found inside **bin/**.

## Scripts

The project includes the following scripts to automate the segmentation process:

* list_images.py  
  Creates a text file that lists the absolute path to every image in a dataset, so it can be used as input for the C++ executables. This script requires the images' names to consist of five digits, ordered by frame and starting with 00000. There should be no reason to use this script in a standalone way, since it is basically a helper called by the other scripts.
* computeShrunkFlows.sh  
  Shrinks the images of a dataset (by default to 854x480, modifiable using a -r flag), computes flows around each one of the frames (backwards and forwards), and lists the resulting files in a file called flows.txt.  
  By default both the flows and the text file are stored in **results/flows/&lt;datasetname&gt;/**.  
  Three parameters are required: the name of the dataset, the number of frames for which the flows should be computed, and the number of surrounding frames to be used as destination for the flow.  
  The default flow used is TV_L1, but LDOF can be used adding a -b flag.  
  Example usage:  
    ```console
    [user@pc tfg_video_segmentation]$ bash scripts/computeShrunkFlows.sh bear 31 3
    ```
* segmentFaktor.sh  
  Segments a video sequence using [1]. It calls list_images.py, then creates the results directories if needed, and performs the segmentation proposed by Faktor and Irani. This script assumes that the flows have been already computed and there exists a file flows.txt listing them, so computeShrunkFlows.sh should be called at least once before calling this script.  
  Results are stored in **results/nlcsegmentation/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to be segmented.  
  Optional flags can be set to modify default parameters of the implementation. Calling the script without arguments gives more information about them.  
  Example usage:  
    ```console
    [user@pc tfg_video_segmentation]$ bash scripts/segmentFaktor.sh bear 31
    ```
* trackSegmentNaveen.sh  
  Segments tracks of a video sequence using [2]. It calls list_images.py for both the data and the seeds, creates the result directories if necessary, and performs the track segmentation. To use this script, there must exist a text file in **test/tracks/** containing previously computed tracks, named after the dataset.  
  The label probabilities are stored in **results/walkerprobs/**, while the painted thresholded tracks can be found in **results/walkedseeds/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to segment.  
  An optional flag -b can be set to parse a track file that uses Brox's format.  
  Example usage:  
    ```console
    [user@pc tfg_video_segmentation]$ bash scripts/trackSegmentNaveen.sh bear 31
    ```
* trackSegmentSzeliski.sh  
  Segments tracks of a video sequence using [3]. To use this script, there must exist a text file in **test/tracks/** containing previously computed tracks, named after the dataset.  
  The background weights are stored in **results/weights/**, while the painted thresholded tracks can be found in **results/model/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to segment.  
  An optional flag -b can be set to parse a track file that uses Brox's format.  
  Example usage:  
    ```console
    [user@pc tfg_video_segmentation]$ bash scripts/trackSegmentSzeliski.sh bear 31
    ```

### Important

The shell scripts rely on knowing the absolute path to the root directory of the project. At the beginning of each of them there is a definition for a variable named **TFGLOCATION** that **must be modified to the path of the project in the local machine for the scripts to work correctly**.
The location where to find the data can also be modified, in addition to some other locations such as the directory where to find the tracks for the track segmentation scripts, and where to find the seeds for trackSegmentNaveen.sh.

