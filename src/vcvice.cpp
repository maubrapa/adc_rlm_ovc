//
//  vcvice.cpp
//  vcvice
//
//  Created by Maurício de Paula on 19/10/13.
//  Copyright (c) 2013 Maurício de Paula. All rights reserved.
//

#include "vcvice.h"

vcvice::vcvice(ConfigParameters _config):
config (_config)
{
    this->video = new VideoProcessor(this->config.path_in + this->config.fileName);
    
//    this->clane = new CLane(this->config.roi.x, this->config.roi.width, this->config.roi.y, this->config.roi.height, this->config.h/2, 0, this->config.framesToTrain, 10, 5);//10, 5 / 90, 30
    this->clane = new CLane(this->config.roi.x, this->config.roi.width, this->config.roi.y, this->config.roi.height, this->config.h/2, 0, this->config.framesToTrain, 90, 30);//10, 5 / 90, 30
    
    this->offset = cvPoint(this->config.roi.x, this->config.roi.y);
    
//    initPatchesParameters();

}


void vcvice::initPatchesParameters() {
    
    // ### Video ###
	Mat frame, gray, frameCopy;
    this->video->setVideoFrame(this->config.initFrame);
    
    // ### CLane ###
	IplImage frameIpl;
	this->clane->stateSystem = INITIAL_STATE;	// (see CLane.h - line 47)
	this->clane->contFrame = this->config.framesToTrain;
    
    Vec<float, 10> wlVec;
    int numberOf_wl = 10;

    this->video->setVideoFrame(this->config.initFrame);
    for (int frm = this->config.initFrame; frm < this->config.initFrame + numberOf_wl; ++frm) {
		this->video->readNextFrame();
		frame = this->video->getCurrFrame();

		// CLane
        cv::Mat imgFrom_gd_Run;
        frameIpl = frame;
        imgFrom_gd_Run = this->clane->gd_Run(&frameIpl);
        cvtColor(frame, gray, CV_BGR2GRAY);

        // -------------------------
//        frameCopy = frame.clone();
//        cvtColor(frame, gray, CV_BGR2GRAY);
//        rectangle(frameCopy, Point(config.roi.x, config.roi.y), Point(config.roi.width, config.roi.height), CV_RGB(0,255,255), 1, CV_AA);
//        imshow("frameCopy", frameCopy);
//        cv::flip(imgFrom_gd_Run, imgFrom_gd_Run, 0);
//        imshow("imgFrom_gd_Run", imgFrom_gd_Run);
//        waitKey(0);
        // -------------------------
        
        // Get the image coordinates of the edges
        ImageCoord coord[4];
        getCoordLanes(coord);
        
        // calc the wl distance, re_height and re_width #####
        int wl = coord[3].U - coord[1].U;
        
        wlVec[frm - this->config.initFrame] = wl;   // frm - this->config.initFrame: alternative to create a index from 0 to 9, i.e. 10 elements
        
    }

    this->mean_wl = mean(wlVec).val[0];
    this->re_height = 0.03 * mean_wl;   // original value: 0.025
    this->re_width = 0.20 * mean_wl;    // original value: 0.15
    this->distanceBetween_re_ri = 0.20;  // original = 0.3
    
}

void vcvice::getAllCoords(int _step) {
    
    int initialIndex = 0;
    int indexMax = this->clane->tamMatrixMain;
    
    Point ptLeft, ptRigth;
    
    this->allCoords.release();
    this->leftCoords.release();
    this->rightCoords.release();
    
    for (int i = initialIndex; i < indexMax; i += _step) {     //++i
        ptLeft.x = this->clane->matrixMainEsq[i].posJ + this->config.roi.x;
        ptLeft.y = this->clane->matrixMainEsq[i].posI + this->config.roi.y;
        
        ptRigth.x = this->clane->matrixMainDir[i].posJ + this->config.roi.x;
        ptRigth.y = this->clane->matrixMainDir[i].posI + this->config.roi.y;
        
        if (i%1 == 0) {
            Mat tmpMat;
            tmpMat = (Mat_<double>(1,2) << ptLeft.x, ptLeft.y);
            this->leftCoords.push_back(tmpMat);
            this->allCoords.push_back(tmpMat);
//            cout << tmpMat << endl;
            
            tmpMat = (Mat_<double>(1,2) << ptRigth.x, ptRigth.y);
            this->rightCoords.push_back(tmpMat);
            this->allCoords.push_back(tmpMat);
        }
    }
    
}

void vcvice::drawBorderLanes(Mat _frame) {
    
    // mark parameters
    int radius = 2;
    int thickness = 1;
    int lineType = CV_AA;
    
    int numberOfPoints = this->leftCoords.size().height;     // left = right

    for (int i = 0; i < numberOfPoints - 1; ++i) {
        Point ptLeft, ptRight;
        ptLeft.x = this->leftCoords.at<double>(i,2);
        ptLeft.y = this->leftCoords.at<double>(i,1);
        ptRight.x = this->rightCoords.at<double>(i,2);
        ptRight.y = this->rightCoords.at<double>(i,1);
        circle(_frame, ptLeft, radius, GREEN, thickness, lineType, 0);
        circle(_frame, ptRight, radius, GREEN, thickness, lineType, 0);
    }
    
}

void vcvice::getCoordLanes(ImageCoord* _coord) {
    int initialIndex = 0;
    int indexMax = this->clane->tamMatrixMain - 15; // -15: bring the patches inside the ROI
    
    // esq_sup - ref.: (0,0) = top left
    _coord[0].U = this->clane->matrixMainEsq[initialIndex].posJ + this->config.roi.x;
    _coord[0].V = this->clane->matrixMainEsq[initialIndex].posI + this->config.roi.y;
    // esq_inf
    _coord[1].U = this->clane->matrixMainEsq[indexMax].posJ + this->config.roi.x;
    _coord[1].V = this->clane->matrixMainEsq[indexMax].posI + this->config.roi.y;
    // dir_sup
    _coord[2].U = this->clane->matrixMainDir[initialIndex].posJ + this->config.roi.x;
    _coord[2].V = this->clane->matrixMainDir[initialIndex].posI + this->config.roi.y;
    // dir_inf
    _coord[3].U = this->clane->matrixMainDir[indexMax].posJ + this->config.roi.x;
    _coord[3].V = this->clane->matrixMainDir[indexMax].posI + this->config.roi.y;
}

void vcvice::getImg_re_ri(Mat _frame, ImageCoord* _coord) {
    Mat gray;
    cvtColor(_frame, gray, CV_BGR2GRAY);
    
    if (this->config.refLane == "left") {
        this->pt1_re = Point(_coord[1].U - this->re_width / 2, _coord[1].V);
        this->pt2_re = Point(_coord[1].U + this->re_width / 2, _coord[1].V + this->re_height);
        // deal with exceptions: when re is out of the image width
        if (this->pt1_re.x < 0)   this->pt1_re.x = 0;
        this->pt1_ri = Point(_coord[1].U + (this->distanceBetween_re_ri * this->mean_wl) - this->re_width / 2, _coord[1].V);
        this->pt2_ri = Point(_coord[1].U + (this->distanceBetween_re_ri * this->mean_wl) + this->re_width / 2, _coord[1].V + this->re_height);
    } else if (this->config.refLane == "right") {
        this->pt1_re = Point(_coord[3].U - this->re_width / 2, _coord[3].V);
        this->pt2_re = Point(_coord[3].U + this->re_width / 2, _coord[3].V + this->re_height);
        this->pt1_ri = Point(_coord[3].U - (this->distanceBetween_re_ri * this->mean_wl) - this->re_width / 2, _coord[3].V);
        this->pt2_ri = Point(_coord[3].U - (this->distanceBetween_re_ri * this->mean_wl) + this->re_width / 2, _coord[3].V + this->re_height);
    }
    // extract the re and ri ROI
    gray(Rect(pt1_re.x, pt1_re.y, this->re_width, this->re_height)).copyTo( this->img_re );
    gray(Rect(pt1_ri.x, pt1_ri.y, this->re_width, this->re_height)).copyTo( this->img_ri );    // size(re) == size(ri)
    
}

void vcvice::drawPatches_re_ri(Mat _frame) {
    
    // re and ri ROI
    int thickness = 1;
    int lineType = CV_AA;
//    Point pt1_re, pt2_re, pt1_ri, pt2_ri;
    if (this->config.refLane == "left") {
        // draw re
        rectangle(_frame, this->pt1_re, this->pt2_re, RED, thickness, lineType);
        // draw ri
        rectangle(_frame, this->pt1_ri, this->pt2_ri, BLUE, thickness, lineType);
    } else if (this->config.refLane == "right") {
        // draw re
        rectangle(_frame, this->pt1_re, this->pt2_re, RED, thickness, lineType);
        // draw ri
        rectangle(_frame, this->pt1_ri, this->pt2_ri, BLUE, thickness, lineType);
    }
    // draw the ROI
    rectangle(_frame, Point(this->config.roi.x, this->config.roi.y), Point(this->config.roi.width, this->config.roi.height), SKYBLUE3, 1, CV_AA);
    
}
