//
//  ConfigParameters.h
//  lane
//
//  Created by Maurício Braga de Paula on 11/21/12.
//  Copyright (c) 2012 Maurício Braga de Paula. All rights reserved.
//

#ifndef CONFIGPARAMETERS_H
#define CONFIGPARAMETERS_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "Util.h"

using namespace std;
using namespace cv;

class ConfigParameters {
    
public:
    
    struct extrinsicStruct {
        float alpha;
        float beta;
        float h;
    } extrinsic;
    
    ConfigParameters() {
		f_Mat = Mat::zeros(1, 2, CV_32F);
		c_Mat = Mat::zeros(1, 2, CV_32F);
        extrinsic_Mat = Mat::zeros(1, 3, CV_32F);
	}
    
	void writeConfigParameters(FileStorage& fs) const {}
    
    void readConfigParameters(const FileNode& node) {
		path_in = (string)node["path_in"];
        path_out = (string)node["path_out"];
		fileName = (string)node["fileName"];
        type_lane_marking = (string)node["type_lane_marking"];
		framesToTrain = (int)node["framesToTrain"];
        framesToTest = (int)node["framesToTest"];
		initFrame = (int)node["initFrame"];
		refLane = (string)node["refLane"];
		u = (int)node["u"];
		v = (int)node["v"];
		w = (int)node["w"];
		h = (int)node["h"];
		iV = (int)node["iV"];
		eV = (int)node["eV"];
		speedVeh = (float)node["speedVeh"];
		
		node["f"] >> f_Mat;
		node["c"] >> c_Mat;
        
		f.x = f_Mat.at<float>(0,0);
		f.y = f_Mat.at<float>(0,1);
		c.x = c_Mat.at<float>(0,0);
		c.y = c_Mat.at<float>(0,1);
		
		roi = cvRect(u, v, u+w, v+h);
		vSweep = cvScalar(iV, eV);
        
        node["extrinsic"] >> extrinsic_Mat;
        extrinsic.alpha = util::deg2rad(extrinsic_Mat.at<float>(0,0));
        extrinsic.beta = util::deg2rad(-1 * extrinsic_Mat.at<float>(0,1));
        extrinsic.h = extrinsic_Mat.at<float>(0,2);
        
    }
    
public:
    int framesToTrain;
    int framesToTest;
    int initFrame;
    int u;
    int v;
    int w;
    int h;
    int iV;
    int eV;
    float speedVeh;
    string path_in;
    string path_out;
    string fileName;
    string type_lane_marking;
    string refLane;
    Point2f f;
    Point2f c;
    CvRect roi;
    CvScalar vSweep;
    
private:
    
	Mat f_Mat;
	Mat c_Mat;
    Mat extrinsic_Mat;
    
};

#endif /* defined(__lane__ConfigParameters__) */






