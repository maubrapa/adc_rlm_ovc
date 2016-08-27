//
//  RoadMark.h
//  roadLaneMarking
//
//  Created by Mauricio de Paula on 28/07/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#ifndef __roadLaneMarking__RoadMark__
#define __roadLaneMarking__RoadMark__

#include <iostream>

#include "vcvice.h" //

#include <chrono>   // for date time
#include <ctime>

#if defined(_OPENMP)
#include <omp.h>
#endif

#include "Definitions.h"

#include "Timer.h"

#include <unistd.h> // for execl and vfork

#include <dirent.h> // for opendir

#include "GaussianClassifier.h" // for C1

#include "Tcc.h"

/* Class definition */
class RoadMark:private vcvice {
    
public:
    RoadMark(ConfigParameters _config);

    void classifyVideo();
    
    void setPathToRoadMarkingsImages(std::string _path);
    void setNewPathOut(std::string _path);
    
    void runVideoOnly();
    
private:
    int bound;  // LEFT, RIGHT, BOTH
    void coreProcessing(int _bound);

    cv::Mat predictedClass_left, predictedClass_right;

    void storeResultFromFrame_t(int _currFrameNumber, int _predictedClass, cv::Mat _prob, double _mahalDistance, bool _seeOnConsole, int _bound);
    
    /* Classifier 2*/
    void identifyLocationsForC2(cv::Mat _peaks, float _rowFromPatch, float _refPoint, cv::Mat* __x_pfl, int p);
    void classifier2(cv::Mat _x_pfl, int* _chosenClass, cv::Mat _np_t);
    void removeZeros(cv::Mat _in, cv::Mat* _out, Mat _mask);
    
    /* Outlier detection */
    bool useOutlierDetection;
    void outlierDetection(GaussianClassifier* _gc, cv::Mat _f1_t, int* _chosenClass, double* _mahalDistance, int _bound);
    int mapClass2C1(int _choosenClass);
    
    /* Gaussian Classifier */
    GaussianClassifier* gc_left;
    GaussianClassifier* gc_right;
    bool useTempInfo;
    void preparesDataForC1(cv::Mat _np_t, cv::Mat* _f1_t);

    /* For WCS (World Coordinate System) */
    int p_max;  // total number of patches - line segments
    float delta_x;
    float delta_z;
    float tau;  // distance between the internal (ri) and external (re) patches
    void calc_p_max();  // compute the number of patches
    
    void xi2xw();   // Xi to Xw coordinates
    float fcnXw(cv::Point2f _Xi);
    float fcnZw(cv::Point2f _Xi);
    float fcnIu(cv::Point3f _Xw);
    float fcnIv(cv::Point3f _Xw);
    
    /* Image profile, valid pair of peaks */
    void extractPeaksPerFrame(cv::Mat* _np_t, cv::Mat* _x_pfl, int _bound);
    void retrivePairOfPeaks(cv::Mat _dgI_pe, cv::Mat _dgI_pi, cv::Mat* _outputPeaks, int* _np, float* _sigma_pi, float* _threshold);

    /* Video */
    int totalFrames;
    cv::Mat Xw_left, Xw_right;  // WC for lane boundaries
    void grabLaneBoundaries(); // stores lane boundaries points (from linear-parabolid model) on Xw_left and Xw_right

    /* Visual */
    cv::Mat currFrameForVis; // Current image frame for vosualization
    void drawLaneBoundaries();
    void drawLineSegments(cv::Vec4f _reXi, cv::Vec4f _riXi);
    void drawMidLineSegments(int _p, cv::Vec4f _reXi, cv::Vec4f _riXi);
    void drawMarkings(cv::Mat _peaks, cv::Vec4f _reXi);
    void loadRoadMarkingsImageFiles(std::string _path);
    void insertClassifiedMark(int _type, int _bound);
    std::string pathToRoadMarkingsImages;
    std::vector<cv::Mat> roadMarkingsPool;
    void viewClassifiedFrames(int _delay);
    void saveClassifiedFrames();

    /* Debug */
    bool saveLineSegmentsToFile;

    /* I/O */
    void mat2File(cv::Mat _inputArray, std::string _fileName);
    void savePredictedClassesInFile(int _bound);

    /* General*/
    void setPaths();
    void initData();
    
    /* VideoWriter */
    cv::VideoWriter videoOut;
    void initVideoWriter();
    void displayInfo(std::string _text, cv::Mat _np_t, double _mahalDistance, int _bound);

    /* OpenMP */
    int numberOfThreads;

    /* Temporal coherence for choosen class*/
    Tcc* tcc_left;
    Tcc* tcc_right;

    /* Image processing */
    void rgb2gray(cv::Mat src, cv::Mat* dst);
};

#endif /* defined(__roadLaneMarking__RoadMark__) */
