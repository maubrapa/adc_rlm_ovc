//
//  vcvice.h
//  vcvice
//
//  Created by Maurício de Paula on 19/10/13.
//  Copyright (c) 2013 Maurício de Paula. All rights reserved.
//

#ifndef __vcvice__vcvice__
#define __vcvice__vcvice__

#include <iostream>

// Linear parabolic model
#include "CLane.h"
#include "_aux.h"
#include "LineSegment2D.h"

// Support classes
#include "VideoProcessor.h"
#include "ConfigParameters.h"
#include "Util.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Defined colors
const Scalar SPICYPINK = CV_RGB(255,28,174);
const Scalar RED = CV_RGB(255,0,0);
const Scalar GREEN = CV_RGB(0,255,0);
const Scalar BLUE = CV_RGB(0,0,255);
const Scalar CYAN = CV_RGB(0,255,255);
const Scalar SKYBLUE3 = CV_RGB(108,166,205);

// namespaces
//using namespace std;
//using namespace cv;

/* CLASS DEFINITION */
class vcvice {
    
public:
  
    /* PUBLIC VARIABLES */
    
    // ### Support classes ###
    ConfigParameters config;    // video config. parameters
    VideoProcessor* video;      // video
    CLane* clane;               // Linear parabolic model
    
    // ### For patches re and ri ###
    int mean_wl;
    float re_height;
    float re_width;
    float distanceBetween_re_ri;
    Point pt1_re, pt2_re, pt1_ri, pt2_ri;
    Mat img_re; //, img_re_, img_re__;
    Mat img_ri;
    
    // ### For CLane ###
    Mat allCoords;
    Mat leftCoords, rightCoords;
    CvPoint offset;
    
    // ### auxiliar variables ###

    /* PUBLIC MEMBER FUNCTIONS */
    vcvice(ConfigParameters _config);   // default constructor
    void getCoordLanes(ImageCoord* _coord);               // 1st. and last point
    void getAllCoords(int _step);                // all points!
    
    void initPatchesParameters();       // determine mean_wl, re_height, re_width values
    void getImg_re_ri(Mat _frame, ImageCoord* _coord);
    
    // graphic
    void drawBorderLanes(Mat _frame);
    void drawPatches_re_ri(Mat _frame);
    
};



#endif /* defined(__vcvice__vcvice__) */
