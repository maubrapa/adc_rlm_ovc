//
//  GaussianClassifier.h
//  roadLaneMarking
//
//  Created by Mauricio de Paula on 31/07/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#ifndef __roadLaneMarking__GaussianClassifier__
#define __roadLaneMarking__GaussianClassifier__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Util.h"

/* Final classes: used in paper */
namespace class_t {
    enum class_t {
        DASHED = 1,
        DASHED_SOLID,
        SOLID_DASHED,
        SINGLE_SOLID,
        DOUBLE_SOLID,
        MERGED
    };
}

/* Class order for C1 */
namespace class_f1 {
    enum class_f1 {
        DASHED=0,
        DOUBLE_SOLID,
        MERGED,
        SINGLE_SOLID
    };
};

class GaussianClassifier {
    
public:
    GaussianClassifier();
    
    void getco(int _indexOfclass, cv::Mat* _coMatrix);
    void getmu(int _indexOfclass, cv::Mat* _muMatrix);
    void classifier(cv::Mat _featureVector, cv::Mat* _probClass, int* _chosenClass, bool _imposeTempCoher);
    void setwindowSize(int _T);
    int getwindowSize();
    
private:
    std::vector<cv::Mat> co, mu;
    cv::Mat tw; // temporal window
    int T;
    
    void loadModel();
    int verifyClass(cv::Mat _probClass); // retrieve class number $\omega_{1i}$
    void manageTemporalInformation(cv::Mat _inputProbClass, cv::Mat* _outputProbClass);
    
};


#endif /* defined(__roadLaneMarking__GaussianClassifier__) */
