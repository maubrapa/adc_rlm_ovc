/*
 *  VideoProcessor.cpp
 *  lane
 *
 *  Created by Maurício B. de Paula on 21/11/11.
 *  Copyright 2011 UFRGS. All rights reserved.
 *
 */

#include "VideoProcessor.h"

VideoProcessor::VideoProcessor(string inputFileName) {
	this->stop = capture.open(inputFileName);	// opens the specified video file
	this->fRate = capture.get(CV_CAP_PROP_FPS);

    if(std::isnan(this->fRate)) {
        this->fRate = 29.97;
    }
	this->delay = 1000.0f/fRate;
	this->totalFrames = capture.get(CV_CAP_PROP_FRAME_COUNT);
	this->imgsz.width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	this->imgsz.height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

VideoProcessor::~VideoProcessor() {
	capture.release();
}

int VideoProcessor::getActualFrameNumber(){
	return capture.get(CV_CAP_PROP_POS_FRAMES);
}

Mat VideoProcessor::getCurrFrame() {
	return this->currFrame;
}

double VideoProcessor::getDelay() {
	return delay;
}

double VideoProcessor::getFOURCC() {
    return capture.get(CV_CAP_PROP_FOURCC);
}

double VideoProcessor::getFPS() {
	return this->fRate;
}

Size VideoProcessor::getImgSize() {
	return this->imgsz;
}

int VideoProcessor::getNumberOfFrames() {
	return this->capture.get(CV_CAP_PROP_FRAME_COUNT);
}

Mat VideoProcessor::getPrevFrame() {
	return this->prevFrame;
}

bool VideoProcessor::hasFrame() {
	return this->stop;
}

bool VideoProcessor::readNextFrame() {
	if (this->currFrame.empty()) {
		this->stop = capture.read(this->currFrame);
		//capture >> this->currFrame;
		this->currFrame.copyTo(prevFrame);
	} else {
		this->currFrame.copyTo(prevFrame);
		this->stop = capture.read(this->currFrame);
	}

	return this->stop;
}

void VideoProcessor::runVideo() {
	namedWindow("Video", CV_WINDOW_NORMAL);
	while (this->stop) {
		if (!readNextFrame()) {
			break;
		}
		imshow("Video", this->currFrame);
		if(waitKey(this->delay) >= 0) {
			break;
		}
	}	
}

void VideoProcessor::setVideoFrame(int frameNumber) {
	this->capture.set(CV_CAP_PROP_POS_FRAMES, (double) frameNumber);
}
