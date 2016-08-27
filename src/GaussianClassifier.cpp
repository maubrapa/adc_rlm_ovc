//
//  GaussianClassifier.cpp
//  roadLaneMarking
//
//  Created by Mauricio de Paula on 31/07/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#include "GaussianClassifier.h"

GaussianClassifier::GaussianClassifier() {

    loadModel();
    this->T = 5;   // default value

}

void GaussianClassifier::classifier(cv::Mat _featureVector, cv::Mat* _probClass, int* _chosenClass, bool _imposeTempCoher) {
    
    cv::Mat fv;
    _featureVector.convertTo(fv, this->mu.at(0).type());
    int numberOfClasses = (int) this->co.size();
    cv::Mat likelihood, x;
    
    for (int i = 0; i < numberOfClasses; ++i) {
        cv::Mat co, mu;
        this->getco(i, &co);
        this->getmu(i, &mu);
        x = -0.5 * (fv - mu) * co.inv() * (fv - mu).t();
        double tmp = (1.0 / sqrt( 2.0 * CV_PI * cv::determinant(co) ) ) * exp( x.at<double>(0) );
        likelihood.push_back(tmp);
    }
    
    double priors = 1.0/numberOfClasses;
    double evidences = cv::sum(likelihood * priors)[0];
    *_probClass = (likelihood * priors) / evidences;
    
    if (_imposeTempCoher) {
        manageTemporalInformation(*_probClass, _probClass);
    }
    
    *_chosenClass = verifyClass(*_probClass);
    
}

void GaussianClassifier::manageTemporalInformation(cv::Mat _inputProbClass, cv::Mat* _outputProbClass) {
    
    _inputProbClass = _inputProbClass.t();
    if (this->tw.rows < this->T) {
        this->tw.push_back(_inputProbClass);
        util::meanArray(this->tw, _outputProbClass, VERT);
    } else {
        this->tw.push_back(_inputProbClass);
        cv::Point refBlock(0,0);
        cv::Size sz(tw.cols, this->T);
        cv::Mat roi = tw(cv::Rect(refBlock, sz));
        cv::Mat block;
        this->tw(cv::Range(1,tw.rows), cv::Range::all()).copyTo(block);
        block.copyTo(roi);
        this->tw.pop_back();
        util::meanArray(this->tw, _outputProbClass, VERT);
    }
    *_outputProbClass = _outputProbClass->t();

}

int GaussianClassifier::verifyClass(cv::Mat _probClass) {
    double minVal, maxVal;
    int minIdx, maxIdx;
    cv::minMaxIdx(_probClass, &minVal, &maxVal, &minIdx, &maxIdx);
    
    /* ### prob. class. ###
     *  1   0   0   0: (0) dashed
     *  0   1   0   0: (1) double-solid
     *  0   0   1   0: (2) dashed-solid or solid-dashed
     *  0   0   0   1: (3) single-solid
     */
    
    switch (maxIdx) {
        case 0:
            return class_t::DASHED;
            break;
        case 1:
            return class_t::DOUBLE_SOLID;
            break;
        case 2:     // dashed-solid or solid-dashed
            return class_t::MERGED;
            break;
        case 3:
            return class_t::SINGLE_SOLID;
            break;
            
        default:
            return -1;
            break;
    }
    
}

void GaussianClassifier::setwindowSize(int _T) {
    this->T = _T;
}

int GaussianClassifier::getwindowSize() {
    return this->T;
}

void GaussianClassifier::getco(int _indexOfclass, cv::Mat* _coMatrix) {
    this->co.at(_indexOfclass).copyTo(*_coMatrix);
}

void GaussianClassifier::getmu(int _indexOfclass, cv::Mat* _muMatrix) {
    this->mu.at(_indexOfclass).copyTo(*_muMatrix);
}

void GaussianClassifier::loadModel() {
    
    // Covariance matrix
    cv::Mat co1 = (cv::Mat_<double>(2,2) << 0.006571441382805, -0.006341660038461, -0.006341660038461, 0.006780972479305);
    cv::Mat co2 = (cv::Mat_<double>(2,2) << 0.000773092110667, 0.000270124348709, 0.000270124348709, 0.001960573928121);
    cv::Mat co3 = (cv::Mat_<double>(2,2) << 0.000092814638845, 0.000074217946972, 0.000074217946972, 0.004500648548049);
    cv::Mat co4 = (cv::Mat_<double>(2,2) << 1.084727110328870e-04, 1.264044992499868e-05, 1.264044992499868e-05, 7.759665176368341e-04);
    this->co.push_back(co1);
    this->co.push_back(co2);
    this->co.push_back(co3);
    this->co.push_back(co4);
    
    // Expected value (mean)
    cv::Mat mu1 = (cv::Mat_<double>(1,2) << 0.603938907877349, 0.394340066797810);
    cv::Mat mu2 = (cv::Mat_<double>(1,2) << 0.010321134468420, 0.021519842691324);
    cv::Mat mu3 = (cv::Mat_<double>(1,2) << -0.001008785854302, 0.677597649626667);
    cv::Mat mu4 = (cv::Mat_<double>(1,2) << 0.000811576881644, 0.985562287788531);
    this->mu.push_back(mu1);
    this->mu.push_back(mu2);
    this->mu.push_back(mu3);
    this->mu.push_back(mu4);
    
}
