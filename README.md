Overview
=========

M. B. de Paula and C. R. Jung, "Automatic Detection and Classification of Road Lane Markings Using Onboard Vehicular Cameras," in IEEE Transactions on Intelligent Transportation Systems, vol. 16, no. 6, pp. 3160-3169, Dec. 2015.

Please cite the above publication if you use this code. [DOI](http://dx.doi.org/10.1109/TITS.2015.2438714)


Requirements
============
The system was implemented and tested in C++ (GCC 4.8.2 compiler), using the Open source Computer Vision library (shortly OpenCV) Version 2.4.9 and the Open Multi-Processing (OpenMP) API.

The dataset used in this work with the corresponding ground truth data, as well as video sequences showing the results of our method, are publicly available at http://inf.ufrgs.br/~mbpaula/roadLaneMarkings/.

Configuration
=============
**xml file**
- path_in: path to the video file
- fileName: video file name
- initFrame: number of the first frame
- framesToTrain: number of frames used to train the linear parabolic model
- framesToTest: number of frames used to test the linear parabolic model

** RoadMark.cpp file **
```c
setNewPathOut("/Users/maubrapa/Documents/adc_rlm_ovc_output/"); // output directory
setPathToRoadMarkingsImages("./road_lane_images/"); // road lane images directory that will be used to show the type of the classified lane (dashed, solid, etc)
```

Running the code
================

Syntax
```
./adc_rlm_ovc <XML_config_file> <clipName>
```

Example
```
./adc_rlm_ovc i5s.xml clip_i5s_0096
```

Output
======

