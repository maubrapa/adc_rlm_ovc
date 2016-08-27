/*
 *  Util.cpp
 *  lane
 *
 *  Created by Maurício B. de Paula on 05/03/12.
 *  Copyright 2012 UFRGS. All rights reserved.
 *
 */

#include "Util.h"

// ### MATH ###

/* Returns the mean values of the elements along different VERT or HORZ dimensions of an array. */
void util::meanArray(cv::Mat _inputArray, cv::Mat* _outputArray, int dim) {
    
    if(dim == VERT) {
        cv::Mat tmp;
        for (int c = 0; c < _inputArray.cols; ++c) {
            tmp.push_back( cv::mean(_inputArray.col(c))[0] );
        }
        tmp = tmp.t();
        tmp.copyTo(*_outputArray);
    } else if (dim == HORZ) {
        for (int r = 0; r < _inputArray.rows; ++r) {
            _outputArray->push_back( cv::mean(_inputArray.row(r))[0] );
        }
    }
    
}

double util::deg2rad(double d) {
	return (d * PI) / 180;
}

double util::rad2deg(double r) {
	return (180 * r) / PI;
}

float util::madStdDev(float madValue) {
	return MAD_FACTOR * madValue;
}

// ### I/O ###
void util::matToFile(cv::Mat* matVector, std::string fileName, int typeOfIO, int matlabMatrixIndex, int frameNumber) {
    std::ofstream out_cvMatToFile;
	if (typeOfIO == OP_APPEND)
        out_cvMatToFile.open(fileName.c_str(), std::ios::app);
    else if(typeOfIO == OP_NEW)
        out_cvMatToFile.open(fileName.c_str(), std::ios::out);
    
    //    if ( (matlabMatrixIndex != 0) && (frameNumber != 0) )
    if ( matlabMatrixIndex != 0 )
        out_cvMatToFile << "Xi(:,:," << int2str(matlabMatrixIndex, 4) << ") = " << *matVector << ";  % frame: " << frameNumber << std::endl;
    else
        out_cvMatToFile << *matVector << std::endl;
    
}

// ### CONVERT ###

std::string util::num2str(double _value) {
    //	stringstream ss;	//create a stringstream
    //	ss << value;		//add number to the stream
    //	return ss.str();//return a string with the contents of the stream
    
    std::stringstream ss;
    ss << _value;
    std::string tmpString = ss.str();
    
    return tmpString;
}

std::string util::int2str(int _value, int _numberOfDigits) {
    //	stringstream ss;	//create a stringstream
    //	ss << value;		//add number to the stream
    //	return ss.str();//return a string with the contents of the stream
    
    std::stringstream ss;
    ss << _value;
    std::string tmpString = ss.str();
    
    if (tmpString.size() < _numberOfDigits) {
        tmpString.insert(0, _numberOfDigits - tmpString.size(), '0');
    }
    
    return tmpString;
}

std::string util::formatInt(int value, int numberOfDigits) {
    std::stringstream valueFormated;
    valueFormated << std::setfill('0') << std::setw(numberOfDigits) << value;
    return valueFormated.str();
}

// ### LANE ###

double util::calcDZ(int velCar, int fps) {
	// km/h to m/s
	double kmhToms = (velCar * 1000.0) / 3600.0;
	return kmhToms / fps;
}

// ### OTHERS ###

// function from gnuplot-cpp
void util::wait_for_key() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
    std::cout << endl << "Press any key to continue..." << std::endl;
    
    FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
    _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    std::cout << std::endl << "Press ENTER to continue..." << std::endl;
    
    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
#endif
    return;
}
