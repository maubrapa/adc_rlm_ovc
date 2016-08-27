/*
 *  VideoProcessor.h
 *  lane
 *
 *  Created by Maurício B. de Paula on 21/11/11.
 *  Copyright 2011 UFRGS. All rights reserved.
 *
 */

#ifndef VIDEOPROCESSOR_H
#define VIDEOPROCESSOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;
using namespace cv;

class VideoProcessor {

public:
	VideoProcessor(string inputFileName);	//default constructor
	~VideoProcessor();				//destructor
	int getActualFrameNumber();		//return the actual frame number
	Mat getCurrFrame();				//return the current frame
	Mat getPrevFrame();				//return the previous frame	
	double getDelay();				//get the delay between frames
	double getFPS();				//get the fps
    double getFOURCC();             //get FOURCC
	Size getImgSize();				//return the image size
	int getNumberOfFrames();		//get the total number of frames
	bool hasFrame();				//verify if exist a new frame
	bool readNextFrame();			//grab the next frame if exists; save the frame in currFrame
	void runVideo();				//run all video frames
	void setVideoFrame(int frameNumber);	//cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, (double) selectedFrame);
	
private:
	VideoCapture capture;
	Mat prevFrame;
	Mat currFrame;
	double fRate;		//frame rate	
	int delay;			//delay between each frame
	bool stop;			//has frame avaiable
	int totalFrames;	//number of
	Size imgsz;			//image dimension
};

#endif
