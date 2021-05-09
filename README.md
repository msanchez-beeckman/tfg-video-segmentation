# Collection of video segmentation algorithms

## Project dependencies

The project uses the following external libraries:
* OpenCV 4
* Eigen 3.3
* maxflow v3.04 (can be downloaded from <https://pub.ist.ac.at/~vnk/software.html>)

It is recommended to use the versions above, since all the testing has been performed using them.
Earlier versions of Eigen may work, but the project uses some calls from OpenCV's API that do not exist in version 2 and have not been tested in version 3.
As for maxflow v3.04, unzip the contents of the compressed folder so that the source code files are in **external/lib/maxflow-v3.04/** (relative to the project's root folder).

To compile the code, the following software is required:
* CMake 3.5

Additionally, some scripts are provided to automate the different segmentation processes. These require:
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

[1] Requires the computation of optical flows between images of the dataset. Inside the **external/** folder there are three Linux executables to do so: **src_flow_tv_l1** outputs the flows in two .tiff files (horizontal and vertical flow), and is much faster than **src_flow_ldof**, which outputs the flows in a single .flo file.
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

[2] and [3] use trajectories to perform the segmentations, and require files containing tracked points. An implementation of a dense point tracking strategy is also included inside the project, so as to compute these trajectories and use them in the segmentation approaches that need them.
It is important to note that point tracking requires to have previously computed the optical flows of a sequence. The following reference contains all the details regarding the tracking method.

6. N. Sundaram, T. Brox, and K. Keutzer  
   Dense point trajectories by GPU-accelerated large displacement optical flow,  
   ECCV, 2010.  
   DOI: 10.1007/978-3-642-15549-9_32

By default, track files are searched in **/results/tracks/**, are assumed to have been computed using **dense_tracking.sh**, and must have the same name as the dataset they are referencing, followed by the number of frames they encompass.
A second format for the track file is accepted, consisting in the output .dat file of Sundaram, Brox, and Keutzer's original executable for GPU Accelerated Point Tracking (which can be found in <https://lmb.informatik.uni-freiburg.de/resources/software.php>). If used, its name must be datasetnameNumberBrox.dat (e.g. *bearBrox82.dat*).

Moreover, the video segmentation defined by [2] is semi-supervised, so seeds (usually user scribbles) are needed for it to work. Those should be placed in **data/&lt;datasetname&gt;/seeds/**, with each seed named after the image the seed is for (e.g. *data/bear/seeds/00000.png*).
Another available option is to use annotations displaying the ground truth for some frames using the DAVIS' dataset format, in which case the corresponding flags must be set when calling the executables/scripts.

[2] and [3] divide the segmentation process in two phases: the segmentation of the tracked points, and a densification phase after which every pixel ends up segmented. Although both papers perform the densification differently, the strategy from one paper can be used for the other one.
In this project, only the densification from [3] has been implemented, which in turn adapts the bilateral grid approach found in the following paper to use it with point trajectories:

7. N. Maerki, F. Perazzi, O. Wang, and A. Sorkine-Hornung  
   Bilateral space video segmentation  
   CVPR, 2016  
   DOI: 10.1109/CVPR.2016.87

Within the project folders there are some video sequences included for testing, located inside **data/**, that belong to the DAVIS dataset. Every frame from each sequence is named with a five digit number, starting with 00000. This convention should be used for other video sequences, since the scripts that come with the project rely on it. Also some scribbles are provided, which follow the same naming standard.

## Setup

To compile the code, run the following commands from the terminal while inside the project's root folder:

```console
[user@pc tfg-video-segmentation]$ mkdir build
[user@pc tfg-video-segmentation]$ cd build
[user@pc tfg-video-segmentation]$ cmake .. -DCMAKE_BUILD_TYPE=Release
[user@pc tfg-video-segmentation]$ make
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
* compute_shrunk_flows.sh  
  Shrinks the images of a dataset (by default to 854x480, modifiable using the -r flag), computes flows around each one of the frames (backwards and forwards), and lists the resulting files in a file called surroundFlows.txt.  
  By default both the flows and the text file are stored in **results/flows/&lt;datasetname&gt;/**.  
  Three parameters are required: the name of the dataset, the number of frames for which the flows should be computed, and the number of surrounding frames to be used as destination for the flow.  
  The default flow used is TV_L1, but LDOF can be used adding a -b flag.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/compute_shrunk_flows.sh bear 31 3
    ```
* segment_faktor.sh  
  Segments a video sequence using [1]. It calls list_images.py, then creates the results directories if needed, and performs the segmentation proposed by Faktor and Irani. This script assumes that the flows have been already computed and there exists a file surroundFlows.txt listing them, so **compute_shrunk_flows.sh** should be called at least once before calling this script.  
  Results are stored in **results/nlcsegmentation/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to be segmented.  
  Optional flags can be set to modify default parameters of the implementation. Calling the script without arguments gives more information about them.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/segment_faktor.sh bear 31
    ```
* list_flows.py  
  Creates a text file that lists the files containing the optical flows for consecutive frames. It differs from the output text file of **compute_shrunk_flows.sh** in that it only lists the flows in one direction (forward or backward) and is formatted as a simple list instead of a list of sets for each frame. This script is used inside other scripts and there is no reason to call it in a standalone way.
* dense_tracking.sh  
  Tracks points in a video sequence using previously computed forward and backward optical flows. It calls list_flows.py to enumerate them, and then follows the path of a sample of pixels (modifiable with the -d option), checking the reliability of the flows before continuing or dropping a trajectory.  
  A text file storing the information of each track (to be used as input by other scripts) is stored in **results/tracks/**, as are the original images with superposed red points indicating which points are being tracked in each frame.  
  To use this script, the folder **results/flows/&lt;datasetname&gt;/** must contain precomputed forward and backward flows for the video sequence. An easy way to accomplish this is to call **compute_shrunk_flows.sh** with one surrounding frame before this script.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/denseTracking.sh -d 12 bear 82
    ```
* track_segment_naveen.sh  
  Segments tracks of a video sequence using [2]. It calls list_images.py for both the data and the seeds, creates the result directories if necessary, and performs the track segmentation.  
  To use this script, there must exist a text file in **results/tracks/&lt;datasetname&gt;/** containing previously computed tracks, named after the video sequence and the number of frames in it, so it is recommended to have called denseTracking.sh at least once before calling this script.  
  The label probabilities are stored in **results/weights/**, while the painted thresholded tracks can be found in **results/walkedseeds/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to segment.  
  An optional flag -b can be set to parse a track file that uses Brox's format, and a -D flag can be used to use DAVIS' ground truth as seeds instead of user scribbles. Calling the script without arguments gives more information about other options.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/track_segment_naveen.sh bear 31
    ```
* track_segment_szeliski.sh  
  Segments tracks of a video sequence using [3]. To use this script, there must exist a text file in **results/tracks/&lt;datasetname&gt;/** containing previously computed tracks, named after the dataset and the number of frames in it.  
  The background weights are stored in **results/weights/**, while the painted thresholded tracks can be found in **results/model/&lt;datasetname&gt;/**.  
  Two parameters are required: the name of the dataset, and the number of frames to segment.  
  An optional flag -b can be set to parse a track file that uses Brox's format.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/track_segment_szeliski.sh bear 31
    ```
* densify_maerki.sh  
  Densifies the track segmentation using [7]. This script needs the resulting track weights from either Szeliski's or Naveen's track segmentation, which must be stored in **results/weights/**, so either of them has to be called before this script. Keep in mind that only the last set of segmented trajectories will be densified.  
  The segmented images can be found in **results/bvsegmentation/&lt;datasetname&gt;/**  
  The script takes two parameters: the dataset name and the number of frames to segment, which has to be the same as the number used for the point tracking and track segmentation.  
  An optional flag -M can be used if the track weights have been obtained with Naveen's method, to segment multiple labels separately instead of doing a simple foreground/background division. A -b flag can (and must) also be used if the track file format is that of Brox's original executable. Calling the script without arguments gives more information about other options.  
  Example usage:  
    ```console
    [user@pc tfg-video-segmentation]$ bash scripts/densify_maerki.sh -M bmx-bumps 90
    ```
