/*
 *  Util.h
 *  lane
 *
 *  Created by Maurício B. de Paula on 02/03/12.
 *  Copyright 2012 UFRGS. All rights reserved.
 *
 */

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <vector>
#include <iterator>
#include <algorithm>
#include <numeric>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#ifndef PI
	#define PI 3.141592653589793
#endif

#define MAD_FACTOR 1.4826

/* Operations with ofstream file */
#define OP_APPEND   1
#define OP_NEW      2


/* Used for find peaks and plateuax */
enum MinMaxType {
    MAX,
    MIN
};
/* Type of regions for findPeaks */
const int PLATEAU = 0;
const int ASCENT = 1;
const int DESCENT = 2;

/* Dimensions of an array for meanArray */
const int VERT = 1;
const int HORZ = 2;

namespace util {

    // ### MATH ###
	template<typename T> void alphaTrimmedMean(const std::vector<T>* _vectorRef, float _alpha, double* _trimmedMean, double* _std);
	template<typename T> float mean(const std::vector<T>* vectorRef);
	template<typename T> float median(const std::vector<T>* vectorRef);
	double deg2rad(double d);
    double rad2deg(double r);
    template<typename T> float mad(const std::vector<T>* vectorRef);
	float madStdDev(float madValue);
    void meanArray(cv::Mat _inputArray, cv::Mat* _outputArray, int dim);

    // ### LANE ###
    double calcDZ(int velCar, int fps);	//calc the displacement
	template<typename T> void deleteFromTheIndexes(std::vector<T> &vectorRef, const std::vector<int> &idxVect);
	template<typename T> std::vector<int> idxToDeleteLessThan1D(const std::vector<T> &vectorRef, float threshold);
	template<typename T> std::vector<int> idxToDeleteGreaterThan1D(const std::vector<T> &vectorRef, float threshold);
	template<typename T> std::vector<int> idxToDeleteGreaterThan2D(const std::vector<T> &vectorRef, cv::Point2f threshold);	//for cv::Point_ structures

    // ### CONVERT ###
	std::string int2str(int _value, int _numberOfDigits);
    std::string num2str(double _value);
    std::string formatInt(int value, int numberOfDigits);    //format the number of int digits
    
    // ### I/O ###
    void matToFile(cv::Mat* matVector, std::string fileName, int typeOfIO, int matlabMatrixIndex = 0, int frameNumber = 0);	// Mat structure to text file
	template<typename T> void printVector(const std::vector<T>* vectorRef);
	template<typename T> void vectorToFile(const std::vector<T>* vectorRef, std::string fileName);
    
    // ### OTHERS ###
    void wait_for_key();
    
    // ### FIND PEAKS AND PLATEAU ###
    template<typename T> void findPeaks(cv::Mat_<T> _inputArray, cv::Mat_<T>* _peaks, int _type, float _cutoff);
    template<typename T> void findLocalPeaks(cv::Mat_<T> _inputArray, cv::Mat_<T>* _outputPeaks, float _cutoff);
    template<typename T> void findProclivity(cv::Mat_<T> _inputArray, cv::Mat_<T>* _outputProclivity);
    template<typename T> void findPlateuIndexes(cv::Mat_<T> _inputProclivity, cv::Mat_<T>* _outPlateauIndexes);
    template<typename T> void findRidgePlateau(cv::Mat_<T> _inputArray, cv::Mat_<T>* _inputPlateauIndexes, cv::Mat_<T>* _outputPlateauPeaks, float _cutoff);
    
}

/*  ################
    ##### CODE #####
    ################ */

template<typename T> void util::findPeaks(cv::Mat_<T> _inputArray, cv::Mat_<T>* _outputPeaks, int _type, float _cutoff) {
    
    // verify row or col _inputArray
    if (_inputArray.rows < _inputArray.cols) {
        cv::Mat tmp = _inputArray.t(); // transpose
        tmp.copyTo(_inputArray);
    }
    
    // invert _inputArray to find local minimum
    if (_type == MIN) {
        _inputArray = _inputArray * -1;
    }
    
    // find peaks
    findLocalPeaks(_inputArray, _outputPeaks, _cutoff);
    
    // find plateus
    cv::Mat_<float> outputProclivity, outIndexes;
    findProclivity(_inputArray, &outputProclivity);
    findPlateuIndexes(outputProclivity, &outIndexes);
    findRidgePlateau(_inputArray, &outIndexes, _outputPeaks, _cutoff);
    
    // multiply _outputPeaks by -1
    if (_type == MIN && !_outputPeaks->empty())
        _outputPeaks->col(1) = _outputPeaks->col(1) * -1;
    
}

template<typename T> void util::findLocalPeaks(cv::Mat_<T> _inputArray, cv::Mat_<T>* _outputPeaks, float _cutoff) {
    
    // find local min and max
    double maxVal, minVal;
	minMaxLoc(_inputArray, &minVal, &maxVal);
    
    _cutoff = _cutoff/100 * maxVal;
    for (int i = 1; i < _inputArray.rows-1; ++i) {
//    for (int i = 2; i < _inputArray.rows-2; ++i) {
//        bool cond_1 =   ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i-2) ) && ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i-1) );
//        bool cond_2 =   ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i+2) ) && ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i+1) );
        bool cond_1 =   ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i-1) );// && ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i-1) );
        bool cond_2 =   ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i+1) );// && ( _inputArray.template at<T>(i) > _inputArray.template at<T>(i+1) );
        bool cond_3 = _inputArray.template at<T>(i) > _cutoff;
        if (cond_1 && cond_2 && cond_3) {
            cv::Mat tmp = (cv::Mat_<T>(1,2) <<  i, _inputArray.template at<T>(i));
            _outputPeaks->push_back( tmp );
        }
    }
    
}

template<typename T> void util::findProclivity(cv::Mat_<T> _inputArray, cv::Mat_<T>* _outputProclivity) {
    
    for (int i = 1; i < _inputArray.rows; ++i) {
        double currentInput  = _inputArray.template at<T>(i);
        double previousInput = _inputArray.template at<T>(i-1);
        
        if (previousInput == currentInput )
            _outputProclivity->push_back((float)PLATEAU);
        else if (previousInput < currentInput )
            _outputProclivity->push_back((float)ASCENT);
        else if (previousInput > currentInput )
            _outputProclivity->push_back((float)DESCENT);
    }
    
}

template<typename T> void util::findPlateuIndexes(cv::Mat_<T> _inputProclivity, cv::Mat_<T>* _outPlateauIndexes) {
    
    bool isContinuos = false;
    int numberOfPlateus = 0;
    int startIdx = 0, endIdx = 0;
    
    for (int i = 0; i <= _inputProclivity.rows; ++i) {
        
        if (_inputProclivity.template at<T>(i) == PLATEAU) {
            
            if (isContinuos) {
                endIdx = i;
            } else {
                startIdx = i;
                endIdx = i;
                numberOfPlateus = numberOfPlateus + 1;
                isContinuos = true;
            }
        } else if( (isContinuos == true) && !( _inputProclivity.template at<T>(i) == PLATEAU) ) {
            cv::Mat tmpMat = (cv::Mat_<T>(1,2) << startIdx, endIdx+1);
            _outPlateauIndexes->push_back(tmpMat);
            isContinuos = false;
        }
        
        if ( (isContinuos == true) && (i == _inputProclivity.rows) ) {
            cv::Mat tmpMat = (cv::Mat_<T>(1,2) << startIdx, endIdx+1);
            _outPlateauIndexes->push_back(tmpMat);
        }
        
    }
    
}

template<typename T> void util::findRidgePlateau(cv::Mat_<T> _inputArray, cv::Mat_<T>* _inputPlateauIndexes, cv::Mat_<T>* _outputPlateauPeaks, float _cutoff) {
    
    //  Plateaux examples
    //  Bottom: \____/
    //        ____
    //  Top: /    \
    
    double maxVal, minVal;
	minMaxLoc(_inputArray, &minVal, &maxVal);
    
    int startIdx, endIdx;
    cv::Point meanPointPlateau;
    
    for (int r = 0; r < _inputPlateauIndexes->rows; ++r) {
        startIdx = _inputPlateauIndexes->template at<T>(r, 0);
        endIdx = _inputPlateauIndexes->template at<T>(r, 1);
        
        // ideal situation
        bool cond1 = startIdx != 0;
        bool cond2 = endIdx != _inputArray.rows;
        
        if (cond1 && cond2) {
            bool cond3 = _inputArray.template at<T>(startIdx-1) < _inputArray.template at<T>(startIdx);
            bool cond4 = _inputArray.template at<T>(endIdx+1) < _inputArray.template at<T>(startIdx);
            
            if (cond3 && cond4) {   // there is a ridge plateu
                double meanPlateauIndex = (endIdx-startIdx)/2.0f + startIdx;
                double valueAtThisPosition = _inputArray.template at<T>(startIdx);
                float _cutoffTop = _cutoff/100 * maxVal;
                
                if( valueAtThisPosition >= _cutoffTop ) {
//                if( meanPlateauIndex > _cutoffTop ) {
                    cv::Mat tmp = (cv::Mat_<T>(1,2) <<  meanPlateauIndex, valueAtThisPosition);
                    _outputPlateauPeaks->push_back( tmp );
                }
            }
        }
    }
    
}

/*  ################
    ###   MATH   ###
    ################ */

template<typename T> void util::alphaTrimmedMean(const std::vector<T>* _vectorRef, float _alpha, double* _trimmedMean, double* _std) {
	// see paper Alpha-trimmed means and their relationship to median filters in 
	double atm = 0;
	
	if (_vectorRef->empty()) {
		std::cerr << "Util::alphaTrimmedMean >> Vector is empty" << std::endl;
	} else {
		std::vector<T>* vectorRefCopy = new std::vector<T>();
		std::vector<T>* atmVector = new std::vector<T>();
		vectorRefCopy->resize(_vectorRef->size());
		copy(_vectorRef->begin(), _vectorRef->end(), vectorRefCopy->begin());
		sort(vectorRefCopy->begin(), vectorRefCopy->end());
		
		//elements to trim
		int N = (int)vectorRefCopy->size();
		double perMult = 1 / (N - 2.0 * floor(_alpha * N));
		double supLim = N - floor(_alpha * N) - 1;
		double infLim = floor(_alpha * N) + 1 - 1;
		int newLen = supLim - infLim + 1;
		atmVector->resize(newLen);
		copy(vectorRefCopy->begin() + infLim, vectorRefCopy->begin() + supLim + 1, atmVector->begin());
		double acc = std::accumulate(atmVector->begin(), atmVector->end(), 0.0f);
		atm = perMult * acc;
        
        // modification to calculate the standard deviation
        cv::Scalar mean_atmVector, std_atmVector;
        meanStdDev(*atmVector, mean_atmVector, std_atmVector);
        *_std = std_atmVector.val[0];
        *_trimmedMean = atm;
	}
}

template<typename T> float util::mad(const std::vector<T>* vectorRef) {
	if (vectorRef->empty()) {
		std::cout << "Vector is empty" << std::endl;
		return 0;
	} else {
		float medianValue = median(vectorRef);
		float madValue;
		std::vector<float> absDev;
		for (int idxAbsDev = 0; idxAbsDev < vectorRef->size(); ++idxAbsDev) {
			absDev.push_back(fabs(vectorRef->at(idxAbsDev) - medianValue));
		}
		madValue = median(&absDev);
		return madValue;
	}
}

template<typename T> float util::mean(const std::vector<T>* vectorRef) {
	if (vectorRef->empty()) {
		std::cout << "Util::mean >> Vector is empty" << std::endl;
        return 0;
	} else {
		return std::accumulate(vectorRef->begin(), vectorRef->end(), 0.0f ) / vectorRef->size();
	}
}

template<typename T> float util::median(const std::vector<T>* vectorRef) {
	std::vector<T>* vectorRefCopy = new std::vector<T>();
	vectorRefCopy->resize(vectorRef->size());
	copy(vectorRef->begin(), vectorRef->end(), vectorRefCopy->begin());
	if (vectorRefCopy->empty()) {
		std::cout << "Util::median >> Vector is empty" << std::endl;
        return 0;
	} else {
		int numberOfElements = (int)vectorRefCopy->size();
		double medianIdx;
		float medianValue;
		sort(vectorRefCopy->begin(), vectorRefCopy->end());
		//odd or even
		if (numberOfElements%2 == 1) {
			medianIdx = (numberOfElements+1)/2.0 - 1;	//-1 because vect index starts with zero
			medianValue = vectorRefCopy->at(medianIdx);
			return medianValue;
		} else {
			int upIdx, downIdx;
			medianIdx = (numberOfElements+1)/2.0;
			upIdx = ceil(medianIdx) - 1;	//-1 because vect index starts with zero
			downIdx = floor(medianIdx) - 1;	//-1 because vect index starts with zero
			medianValue = (vectorRefCopy->at(upIdx) + vectorRefCopy->at(downIdx)) / 2;
			return medianValue;
		}
	}
}

/*  ################
    ###   LANE   ###
    ################ */

template<typename T> void util::deleteFromTheIndexes(std::vector<T> &vectorRef, const std::vector<int> &idxVect) {
	if (vectorRef.empty()) {
//		cout << "Vector is empty" << endl;
	} else {
		int decValIdx = 0;
		for (int pos=0; pos < idxVect.size(); ++pos) {
			vectorRef.erase(vectorRef.begin() + idxVect.at(pos) + decValIdx);
			--decValIdx;
		}
	}
}

template<typename T> std::vector<int> util::idxToDeleteGreaterThan1D(const std::vector<T> &vectorRef, float threshold) {
	std::vector<int> idxVect;
	if (vectorRef.empty()) {
//		cout << "Vector is empty" << endl;
		return idxVect;
	} else {
		for (int idx = 0; idx < vectorRef.size(); ++idx) {
			if (vectorRef.at(idx) > threshold) {
				idxVect.push_back(idx);
			}
		}
	}
	return idxVect;
}

template<typename T> std::vector<int> util::idxToDeleteLessThan1D(const std::vector<T> &vectorRef, float threshold) {
	std::vector<int> idxVect;
	if (vectorRef.empty()) {
//		cout << "Vector is empty" << endl;
		return idxVect;
	} else {
		for (int idx = 0; idx < vectorRef.size(); ++idx) {
			if (vectorRef.at(idx) < threshold) {
				idxVect.push_back(idx);
			}
		}
	}
	return idxVect;
}

template<typename T> std::vector<int> util::idxToDeleteGreaterThan2D(const std::vector<T> &vectorRef, cv::Point2f threshold) {
	std::vector<int> idxVect;
	if (vectorRef.empty()) {
//		cout << "Vector is empty" << endl;
		return idxVect;
	} else {
		for (int idx = 0; idx < vectorRef.size(); ++idx) {
			if (vectorRef.at(idx).y > threshold.y) {
				idxVect.push_back(idx);
			}
		}
	}
	return idxVect;
}

template<typename T> void util::printVector(const std::vector<T>* vectorRef) {
	if (vectorRef->empty()) {
		std::cout << "Vector is empty" << std::endl;
	} else {
		std::ostream_iterator<T> out(std::cout, "; ");
		copy(vectorRef->begin(), vectorRef->end(), out);
		std::cout << std::endl;
	}
}

template<typename T> void util::vectorToFile(const std::vector<T>* vectorRef, std::string fileName) {
	std::ofstream out_cvMatToFile (fileName.c_str(), std::ios::out);
	
	for(int pos = 0; pos < vectorRef->size(); ++pos)
		out_cvMatToFile << vectorRef->at(pos) << "\n";
	
//	cout << fileName << " created" << endl;
}

#endif
