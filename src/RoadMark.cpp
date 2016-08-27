//
//  RoadMark.cpp
//  roadLaneMarking
//
//  Created by Mauricio de Paula on 28/07/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#include "RoadMark.h"

RoadMark::RoadMark(ConfigParameters _config):vcvice(_config) {
    setPaths();
    initData();
#if defined(_OPENMP)
    omp_set_num_threads(this->numberOfThreads);
#endif
}

void RoadMark::initData() {
    
    loadRoadMarkingsImageFiles(this->pathToRoadMarkingsImages);
    this->totalFrames = this->config.initFrame + this->config.framesToTrain + this->config.framesToTest;
    this->delta_x = 1.0f;
    this->delta_z = 0.5f;
    this->tau = 0.12f;
    calc_p_max();
    this->useTempInfo = false;
    this->useOutlierDetection = true;
    
    this->numberOfThreads = 16;

    this->video->setVideoFrame(this->config.initFrame);

    gc_left = new GaussianClassifier();
    gc_right = new GaussianClassifier();

    tcc_left = new Tcc(T2);
    tcc_right = new Tcc(T2);

    initVideoWriter();
    
}

void RoadMark::setPaths() {
    /* Set path according the OS */
#ifdef __APPLE__
    setNewPathOut("/Users/maubrapa/Documents/adc_rlm_ovc_output/");
    setPathToRoadMarkingsImages("./road_lane_images/");
#elif __linux__
    setNewPathOut("/home/maubrapa/Documents/adc_rlm_ovc_output/");
    setPathToRoadMarkingsImages("./road_lane_images/");
#endif
}

void RoadMark::classifyVideo() {

    std::cout << "Video file: " << config.path_in + config.fileName << std::endl;
    std::cout << "Number of frames in the video file: " <<  video->getNumberOfFrames() << std::endl;

    this->bound = boundary::BOTH;
    
    for (int frm = this->config.initFrame; frm <= this->totalFrames; ++frm) {

        /* Linear-parabolic model */
        grabLaneBoundaries(); // in ICS

        xi2xw(); // map from ICS to WCS

        switch (this->bound) {
        case boundary::LEFT:
            coreProcessing(boundary::LEFT);
            break;
        case boundary::RIGHT:
            coreProcessing(boundary::RIGHT);
            break;
        default:
            coreProcessing(boundary::LEFT);
            coreProcessing(boundary::RIGHT);
            break;
        }

        saveClassifiedFrames();

    } // end for
    
    /* Save predicted classes */
    savePredictedClassesInFile(this->bound);
    
    /* Release data*/
    this->videoOut.release();
    
}

void RoadMark::coreProcessing(int _bound) {

    /* Line segments, image profiles and valid peaks */
    cv::Mat np_t = cv::Mat::zeros(1, this->p_max, CV_8UC1);
    cv::Mat x_pfl = cv::Mat::zeros(this->p_max, 2, CV_32F);

    extractPeaksPerFrame(&np_t, &x_pfl, _bound);

    /* Classifier C1 */
    int ksize = 3;
    cv::Mat np_tFiltered;
    cv::medianBlur(np_t, np_tFiltered, ksize);

    cv::Mat f1_t; // features for Classifier C1
    preparesDataForC1(np_tFiltered, &f1_t);
    cv::Mat prob;
    int chosenClass;
    if(_bound == boundary::LEFT)
        this->gc_left->classifier(f1_t, &prob, &chosenClass, this->useTempInfo);
    else if(_bound == boundary::RIGHT)
        this->gc_right->classifier(f1_t, &prob, &chosenClass, this->useTempInfo);

    /* Classifier C2 */
    if (chosenClass == class_t::MERGED) // merged
        classifier2(x_pfl, &chosenClass, np_t);

    /* Outlier detection */
    double mahalDistance;
    if(_bound == boundary::LEFT)
        outlierDetection(this->gc_left, f1_t, &chosenClass, &mahalDistance, _bound);
    else if(_bound == boundary::RIGHT)
        outlierDetection(this->gc_right, f1_t, &chosenClass, &mahalDistance, _bound);

    /* Temporal coherence of the class */
    if(_bound == boundary::LEFT)
        tcc_left->update(chosenClass, &chosenClass);
    else if(_bound == boundary::RIGHT)
        tcc_right->update(chosenClass, &chosenClass);

    /* Store result*/
    bool seeOnConsole = false;
    int frameNumber = this->video->getActualFrameNumber() - 1;
    storeResultFromFrame_t(frameNumber, chosenClass, prob, mahalDistance, seeOnConsole, _bound);

//    drawLaneBoundaries();
    displayInfo(util::int2str(frameNumber,4), np_t, mahalDistance, _bound);
    insertClassifiedMark(chosenClass, _bound);

}

void RoadMark::saveClassifiedFrames() {    
    this->videoOut << this->currFrameForVis;    
}

void RoadMark::viewClassifiedFrames(int _delay) {

    // Show image frame
    cv::namedWindow("output", CV_WINDOW_KEEPRATIO);
    cv::imshow("output", this->currFrameForVis);
    waitKey(30);

}

void RoadMark::savePredictedClassesInFile(int _bound) {
    
    int dotLocation = (int) this->config.fileName.find(".");
    std::string videoFileNameWithoutExt;
    if(_bound == boundary::LEFT) {
        videoFileNameWithoutExt = this->config.fileName.substr(0, dotLocation) + "_left.m";
        mat2File(this->predictedClass_left, videoFileNameWithoutExt);
    }
    else if(_bound == boundary::RIGHT) {
        videoFileNameWithoutExt = this->config.fileName.substr(0, dotLocation) + "_right.m";
        mat2File(this->predictedClass_right, videoFileNameWithoutExt);
    } else if(_bound == boundary::BOTH) {
        videoFileNameWithoutExt = this->config.fileName.substr(0, dotLocation) + "_left.m";
        mat2File(this->predictedClass_left, videoFileNameWithoutExt);
        videoFileNameWithoutExt = this->config.fileName.substr(0, dotLocation) + "_right.m";
        mat2File(this->predictedClass_right, videoFileNameWithoutExt);
    }
    
}

void RoadMark::storeResultFromFrame_t(int _currFrameNumber, int _predictedClass, cv::Mat _prob, double _mahalDistance, bool _seeOnConsole, int _bound) {
    
    cv::Mat out = cv::Mat::zeros(1, 7, CV_32F);
    out.at<float>(0,0) = (float) _currFrameNumber;
    _prob = _prob.t();
    _prob.copyTo(out(Range::all(), Range(1, 5)));
    out.at<float>(0,5) = _mahalDistance;
    out.at<float>(0,6) = _predictedClass;

    if(_bound == boundary::LEFT)
        this->predictedClass_left.push_back(out);
    else if(_bound == boundary::RIGHT)
        this->predictedClass_right.push_back(out);
    
    if (_seeOnConsole) {
        printf("%d; ", _currFrameNumber);
        for (int c = 0; c < _prob.cols; ++c) {
            printf("%.4f; ", _prob.at<double>(0,c));
        }
        printf("%.4f; ", _mahalDistance);
        printf("%d\n", _predictedClass);
    }

}

void RoadMark::insertClassifiedMark(int _type, int _bound) {
    
    cv::Mat roadMarkImage = this->roadMarkingsPool.at(_type - 1);

    int newWidth = (this->currFrameForVis.cols * roadMarkImage.cols) / 800;
    int newHeight = (this->currFrameForVis.rows * roadMarkImage.rows) / 640;
    resize(roadMarkImage, roadMarkImage, Size(newWidth, newHeight));
    
    // define ROI
    Point roadMarkImagePosition;
    if(_bound == boundary::LEFT) {
        roadMarkImagePosition = Point(roadMarkImage.cols, 0.5 * roadMarkImage.rows);
    } else if(_bound == boundary::RIGHT) {
        roadMarkImagePosition = Point(this->currFrameForVis.cols - 1.5 * roadMarkImage.cols, 0.5 * roadMarkImage.rows);
    }
    
    Mat imageROI = this->currFrameForVis(Rect(roadMarkImagePosition.x, roadMarkImagePosition.y, roadMarkImage.cols, roadMarkImage.rows));
    roadMarkImage.copyTo(imageROI);
    
}

void RoadMark::loadRoadMarkingsImageFiles(std::string _path) {
    
    std::vector<std::string> listOfFiles;
    struct dirent *drnt;
    DIR *dr = NULL;
    dr = opendir(_path.c_str());
    if (dr) {
        while( (drnt = readdir(dr)) ) {
            std::string fileName = drnt->d_name;
            listOfFiles.push_back(fileName);
        }
        std::sort(listOfFiles.begin(), listOfFiles.end());
        for(int l = 0; l < listOfFiles.size(); ++l) {
            std::string fileName = listOfFiles.at(l);
            if ( !(!fileName.compare(".") || !fileName.compare("..") || !fileName.compare(".DS_Store")) ) {
//                std::cout << "load " << _path + fileName.c_str() << " file." << std::endl;
                Mat logoTmp = imread(_path + fileName.c_str());
                this->roadMarkingsPool.push_back(logoTmp);                
            }
        }
    } else {
        printf("Can not open directory!\n");
    }
    
}

void RoadMark::setPathToRoadMarkingsImages(std::string _path) {
    this->pathToRoadMarkingsImages = _path;
}

void RoadMark::removeZeros(cv::Mat _in, cv::Mat* _out,cv::Mat _mask) {

    cv::Mat tmp;
    for(int r = 0; r < _in.rows; ++r) {
        if((int) _mask.at<unsigned char>(0,r) > 0)
            tmp.push_back(_in.at<float>(r,0));
    }
    int endIdx = round(tmp.rows * CUTOFF);
    tmp(cv::Range(0,endIdx), cv::Range::all()).copyTo(*_out);
}

void RoadMark::classifier2(cv::Mat _x_pfl, int* _chosenClass, cv::Mat _np_t) {
    
    cv::Mat x_pf, x_pl;
    removeZeros(_x_pfl.col(0), &x_pf, _np_t);
    removeZeros(_x_pfl.col(1), &x_pl, _np_t);
    
    cv::Scalar mu_x_pf, mu_x_pl;
    cv::Scalar stddev_x_pf, stddev_x_pl;
    cv::Scalar var_x_pf, var_x_pl;
    
    cv::meanStdDev(x_pf, mu_x_pf, stddev_x_pf);
    cv::meanStdDev(x_pl, mu_x_pl, stddev_x_pl);

    cv::pow(stddev_x_pf, 2, var_x_pf);
    cv::pow(stddev_x_pl, 2, var_x_pl);
    
    if (var_x_pf[0] < var_x_pl[0]) {
        *_chosenClass = class_t::SOLID_DASHED ;
    } else {
        *_chosenClass = class_t::DASHED_SOLID;
    }
    
}

int RoadMark::mapClass2C1(int _choosenClass) {
    
    switch (_choosenClass) {
        case class_t::DASHED:
            return class_f1::DASHED;
            break;
        case class_t::DASHED_SOLID:
            return class_f1::MERGED;
            break;
        case class_t::SOLID_DASHED:;
            return class_f1::MERGED;
            break;
        case class_t::SINGLE_SOLID:
            return class_f1::SINGLE_SOLID;
            break;
        case class_t::DOUBLE_SOLID:
            return class_f1::DOUBLE_SOLID;
            
        default:
            return -1;
            break;
    }

}

void RoadMark::outlierDetection(GaussianClassifier *_gc, cv::Mat _f1_t, int* _chosenClass, double* _mahalDistance, int _bound) {
    
    if (this->useOutlierDetection) {
        cv::Mat co;
        _gc->getco(mapClass2C1(*_chosenClass), &co);
        
        cv::Mat mu;
        _gc->getmu(mapClass2C1(*_chosenClass), &mu);
        
        _f1_t.convertTo(_f1_t, mu.type());
        
        *_mahalDistance = cv::Mahalanobis(_f1_t, mu, co.inv());
        if (_bound == boundary::LEFT && *_mahalDistance > CHI_SQUARE_LIMIT && this->predictedClass_left.rows > 1 ) {
            int previousPredictedClass = (int) this->predictedClass_left.at<float>(this->predictedClass_left.rows - 2, 6);
            *_chosenClass = previousPredictedClass;
            this->predictedClass_left.at<float>(this->predictedClass_left.rows-1, 1) = previousPredictedClass; // update with the previus predicted value
        } else if ( bound == boundary::RIGHT && *_mahalDistance > CHI_SQUARE_LIMIT && this->predictedClass_right.rows > 1 ) {
            int previousPredictedClass = (int) this->predictedClass_right.at<float>(this->predictedClass_right.rows - 2, 6);
            *_chosenClass = previousPredictedClass;
            this->predictedClass_right.at<float>(this->predictedClass_right.rows-1, 1) = previousPredictedClass; // update with the previus predicted value
        }
    }
    
}

void RoadMark::preparesDataForC1(cv::Mat _np_t, cv::Mat* _f1_t) {

    float f11_t = cv::countNonZero(_np_t == 0) / (float) this->p_max;
    float f12_t = cv::countNonZero(_np_t == 2) / (float) this->p_max;
    *_f1_t = (cv::Mat_<float>(1,2) << f11_t, f12_t);
    
}

void RoadMark::calc_p_max() {
    this->p_max = 1 + (1.5 * __l_mk + __l_adj) * pow(this->delta_z, -1);
}

void RoadMark::extractPeaksPerFrame(cv::Mat* _np_t, cv::Mat* _x_pfl, int _bound) {
    
    // Fits a line to a 2D point set
    cv::Vec4f fittedLine;

    if(_bound == boundary::LEFT)
        cv::fitLine(this->Xw_left, fittedLine, CV_DIST_L2, 0, 0.01,0.01);
    else
        cv::fitLine(this->Xw_right, fittedLine, CV_DIST_L2, 0, 0.01,0.01);

    float vx, vy, x0, y0;
    vx = fittedLine.val[0];
    vy = fittedLine.val[1];
    x0 = fittedLine.val[2];
    y0 = fittedLine.val[3];
    
    // compute z_min
    double z_min;
    int rows = this->Xw_left.cols;
    cv::minMaxLoc(this->Xw_left(Range::all(), Range(rows-1,rows)), &z_min);
        
    // define line segments
//#pragma omp parallel for
    for (int p = 0; p < this->p_max; p++) {
    
        // WCS (World Coordinate System)
        float y = z_min + p * this->delta_z;
        float xw = (vx * (y - y0) + x0 * vy) / vy;
        float zw = y;
        
        Point3f reXwLeft = Point3f(xw - this->delta_x/2 , 0, zw);
        Point3f reXwRight = Point3f(xw + this->delta_x/2 , 0, zw);
        
        Point3f riXwLeft;
        Point3f riXwRight;
        if(_bound == boundary::LEFT) {
            riXwLeft = Point3f(xw + this->delta_x/2 + this->tau, 0, zw);
            riXwRight = Point3f(xw + this->delta_x/2 + this->tau + this->delta_x, 0, zw);
        } else {
            riXwLeft = Point3f(xw - this->delta_x/2 - this->tau, 0, zw);
            riXwRight = Point3f(xw - this->delta_x/2 - this->tau - this->delta_x, 0, zw);
        }
        
        // ICS (Image Coordinate System)
        float reIuLeft = fcnIu(reXwLeft);
        float reIuRight = fcnIu(reXwRight);
        float reIvLeft = fcnIv(reXwLeft);
        float reIvRight = fcnIv(reXwRight);

        float riIuLeft = fcnIu(riXwLeft);
        float riIuRight = fcnIu(riXwRight);
        float riIvLeft = fcnIv(riXwLeft);
        float riIvRight = fcnIv(riXwRight);

        // Deal with outbound coordinates
        if (reIuLeft < 1)
            reIuLeft = 1;
        else if(reIuRight > this->video->getImgSize().width)
            reIuRight = this->video->getImgSize().width;
        
        int imgHeight = (this->video->getImgSize()).height;
        cv::Vec4f reXi = Vec4f(reIuLeft, imgHeight - reIvLeft, reIuRight, imgHeight - reIvRight);
        cv::Vec4f riXi = Vec4f(riIuLeft, imgHeight - riIvLeft, riIuRight, imgHeight - riIvRight);

        // Extract the image profile I_pe(u) and I_pi(u) along r_pe and r_pi
        float u1 = round(reXi.val[0]);
        float v1 = round(reXi.val[1]);
        float u2 = round(reXi.val[2]);
        float width = round(u2 - u1);
        float height = 1;
        cv::Mat I_pe(this->video->getCurrFrame(), cv::Rect(u1, v1, width, height));
        cv::cvtColor(I_pe, I_pe, CV_BGR2GRAY);  // convert I_pe from BGR to grayscale
//        rgb2gray(I_pe, &I_pe);

        u1 = round(riXi.val[0]);
        v1 = round(riXi.val[1]);
        u2 = round(riXi.val[2]);
        width = abs(round(u2 - u1));
        cv::Mat I_pi;
        if(_bound == boundary::LEFT)
            I_pi = cv::Mat(this->video->getCurrFrame(), cv::Rect(u1, v1, width, height));
        else
            I_pi = cv::Mat(this->video->getCurrFrame(), cv::Rect(u2, v1, width, height));

        cv::cvtColor(I_pi, I_pi, CV_BGR2GRAY);  // convert I_pi from BGR to grayscale
//        rgb2gray(I_pi, &I_pi);
        
        // Define sigma_p based on the line segment coord. and SIGMA_W
        Point3f sigmaXwLeft = Point3f(xw, 0, zw);
        Point3f sigmaXwRight = Point3f(xw + SIGMA_W , 0, zw);
        float sigmaIuLeft = fcnIu(sigmaXwLeft);
        float sigmaIuRight = fcnIu(sigmaXwRight);
        float sigma_p = sigmaIuRight - sigmaIuLeft;

        // Convolves the source image with the specified Gaussian kernel
        cv::Size kernelSize = cv::Size(7, 1);
        cv::Mat gI_pe, gI_pi;

        I_pe.convertTo(I_pe, CV_32F);
        I_pi.convertTo(I_pi, CV_32F);

        cv::GaussianBlur(I_pe, gI_pe, kernelSize, sigma_p);
        cv::GaussianBlur(I_pi, gI_pi, kernelSize, sigma_p);
                
        // Compute dgI_pe(u) and dgI_pe(u) derivative at gI_pe(u) and gI_pe(u)
        cv::Mat dgI_pe, dgI_pi;
        cv::Sobel(gI_pe, dgI_pe, CV_32F, 1, 0);
        cv::Sobel(gI_pi, dgI_pi, CV_32F, 1, 0);

        // Retrieve every pair of adjacent maximum-minimum peaks and the inter-peak height difference of the j-th pair valideted by k*sigma_pi
        cv::Mat peaks;
        int np = 0;
        float sigma_pi;
        float threshold;
        retrivePairOfPeaks(dgI_pe, dgI_pi, &peaks, &np, &sigma_pi, &threshold);
        
        // Identify locations for C2
        identifyLocationsForC2(peaks, v1, reIuLeft, _x_pfl, p);
        
        // Saves the number of peaks in np_t
        _np_t->at<unsigned char>(0,p) = np;

    } // end for

}

void RoadMark::retrivePairOfPeaks(cv::Mat _dgI_pe, cv::Mat _dgI_pi, cv::Mat* _outputPeaks, int* _np, float* _sigma_pi, float* _threshold) {
    
    // Compute the standard deviation sigma_pi
    cv::Scalar mu_pi, sigma_pi;
    cv::meanStdDev(_dgI_pi, mu_pi, sigma_pi);
    *_sigma_pi = sigma_pi.val[0];
    *_threshold = K_SIGMA_I * sigma_pi.val[0];
    
    cv::Mat_<float> localMinima, localMaxima, allPeaks, sortedPeaks;

    // find all peaks with VAREPSILON
    util::findPeaks(cv::Mat_<float>(_dgI_pe), &localMaxima, MAX, VAREPSILON);
    util::findPeaks(cv::Mat_<float>(_dgI_pe), &localMinima, MIN, VAREPSILON);
    allPeaks.push_back(localMaxima);
    allPeaks.push_back(localMinima);
    
    if (!allPeaks.empty()) {
        
        // sorts them
        cv::Mat idx;
        cv::sortIdx(allPeaks.col(0), idx, CV_SORT_ASCENDING + CV_SORT_EVERY_COLUMN);
        for (int i = 0; i < idx.rows; ++i) {
            int newIndex = idx.at<int>(i);
            sortedPeaks.push_back( allPeaks(Range(newIndex, newIndex+1), Range::all() ) );
        }
        
        // check for pair of peaks
        for (int r = 0; r < sortedPeaks.rows - 1; ++r) {
            
            bool cond1 = sortedPeaks.at<float>(r,1) > 0;
            bool cond2 = sortedPeaks.at<float>(r+1,1) < 0;
            if (cond1 && cond2) {
//                _outputPeaks->push_back( sortedPeaks(Range(r, r+2), Range::all() ) );
                // Compute the inter-peak height difference of the j-th pair
                float pk1 = sortedPeaks.at<float>(r, 1);
                float pk2 = sortedPeaks.at<float>(r+1, 1);
                float delta_h_pj = abs( pk1 - pk2 );
                // Validate the j-th peak pair based on k*sigma_pi
                if (delta_h_pj > *_threshold) {
                    _outputPeaks->push_back( sortedPeaks(Range(r, r+2), Range::all() ) );
                    *_np += 2;
                }
            }
        }
        
        if (*_np > 4) {
            *_np = 4;
        }
        
    }
    
}

void RoadMark::identifyLocationsForC2(cv::Mat _peaks, float _rowFromPatch, float _refPoint, cv::Mat* __x_pfl, int p) {

    cv::Point2f pt_f2D, pt_l2D;
    if (!_peaks.empty()) {

        int firstRow = 0;
        int lastRow = _peaks.rows - 1;
        int uPos = 0;
        pt_f2D.x = _peaks.at<float>(firstRow, uPos) + _refPoint;
        pt_f2D.y = _rowFromPatch;

        pt_l2D.x = _peaks.at<float>(lastRow, uPos) + _refPoint;
        pt_l2D.y = _rowFromPatch;

        float Xw_f = fcnXw(pt_f2D);
        float Xw_l = fcnXw(pt_l2D);

        __x_pfl->at<float>(p,0) = Xw_f;
        __x_pfl->at<float>(p,1) = Xw_l;

    }
}

void RoadMark::xi2xw() {
    
    float Xw, Zw;
    Mat tmp;
    this->Xw_left.release();
    this->Xw_right.release();
    
    for (int r = 0; r < this->leftCoords.rows; ++r) {
        cv::Point2f pt2D;
        pt2D.x = (float) this->leftCoords.at<double>(r, 0);
        pt2D.y = (float) this->leftCoords.at<double>(r, 1);
        Xw = fcnXw(pt2D);
        Zw = fcnZw(pt2D);
        tmp = (Mat_<float>(1,2) << Xw, Zw);
        this->Xw_left.push_back(tmp);

        pt2D.x = (float) this->rightCoords.at<double>(r, 0);
        pt2D.y = (float) this->rightCoords.at<double>(r, 1);
        Xw = fcnXw(pt2D);
        Zw = fcnZw(pt2D);
        tmp = (Mat_<float>(1,2) << Xw, Zw);
        this->Xw_right.push_back(tmp);
    }
    
}

float RoadMark::fcnXw(cv::Point2f _Xi) {

    float fu = this->config.f.x;
    float fv = this->config.f.y;
    float uo = this->config.c.x;
    float vo = this->config.c.y;
    float h = this->config.extrinsic.h;
    
    float u = _Xi.x;
    float v = 2 * vo - _Xi.y;
    
    float t2 = cos(this->config.extrinsic.beta);
    float t3 = cos(this->config.extrinsic.alpha);
    float t4 = t2*t2;
    float t5 = sin(this->config.extrinsic.beta);
    float t6 = t5*t5;
    float t7 = sin(this->config.extrinsic.alpha);
    float t8 = t3*t3;
    float t9 = t7*t7;
    
    float Xw = -(fu*fv*h*t3*t5+fv*h*t2*t8*u+fv*h*t2*t9*u-fv*h*t2*t8*uo-fv*h*t2*t9*uo-fu*h*t5*t7*v+fu*h*t5*t7*vo)/(fu*fv*t4*t7+fu*fv*t6*t7+fu*t3*t4*v+fu*t3*t6*v-fu*t3*t4*vo-fu*t3*t6*vo);
    
    return Xw;
}

float RoadMark::fcnZw(cv::Point2f _Xi) {

    float fu = this->config.f.x;
    float fv = this->config.f.y;
    float uo = this->config.c.x;
    float vo = this->config.c.y;
    float h = this->config.extrinsic.h;

    float u = _Xi.x;
    float v = 2 * vo - _Xi.y;

    float t2 = cos(this->config.extrinsic.beta);
    float t3 = cos(this->config.extrinsic.alpha);
    float t4 = t2*t2;
    float t5 = sin(this->config.extrinsic.beta);
    float t6 = t5*t5;
    float t7 = sin(this->config.extrinsic.alpha);
    float t8 = t3*t3;
    float t9 = t7*t7;
    
    float Zw = -(fu*fv*h*t2*t3-fv*h*t5*t8*u-fv*h*t5*t9*u+fv*h*t5*t8*uo+fv*h*t5*t9*uo-fu*h*t2*t7*v+fu*h*t2*t7*vo)/(fu*fv*t4*t7+fu*fv*t6*t7+fu*t3*t4*v+fu*t3*t6*v-fu*t3*t4*vo-fu*t3*t6*vo);
    
    return Zw;
}

float RoadMark::fcnIu(cv::Point3f _Xw) {
    
    float fu = this->config.f.x;
    float uo = this->config.c.x;
    float h = this->config.extrinsic.h;

    float t2 = cos(this->config.extrinsic.alpha);
    float t3 = cos(this->config.extrinsic.beta);
    float t4 = sin(this->config.extrinsic.beta);
    float t5 = sin(this->config.extrinsic.alpha);
    
    float X = _Xw.x;
    float Z = _Xw.z;
    float Iu = -(-X*(fu*t3+t2*t4*uo)+Z*(fu*t4-t2*t3*uo)+h*t5*uo)/(-h*t5+X*t2*t4+Z*t2*t3);
    return Iu;
}

float RoadMark::fcnIv(cv::Point3f _Xw) {
    
    float fv = this->config.f.y;
    float vo = this->config.c.y;
    float h = this->config.extrinsic.h;

    float t2 = cos(this->config.extrinsic.alpha);
    float t3 = cos(this->config.extrinsic.beta);
    float t4 = sin(this->config.extrinsic.alpha);
    float t5 = sin(this->config.extrinsic.beta);
    
    float X = _Xw.x;
    float Z = _Xw.z;
    float Iv = -(X*(fv*t4*t5-t2*t5*vo)+Z*(fv*t3*t4-t2*t3*vo)+fv*h*t2+h*t4*vo)/(-h*t4+X*t2*t5+Z*t2*t3);
    return Iv;
}

void RoadMark::drawLaneBoundaries() {
    drawBorderLanes(this->currFrameForVis);
}

void RoadMark::drawLineSegments(cv::Vec4f _reXi, cv::Vec4f _riXi) {
    
    Point pt1 = Point(_reXi.val[0], _reXi.val[1]);
    Point pt2 = Point(_reXi.val[2], _reXi.val[3]);
    cv::line(this->currFrameForVis, pt1, pt2, CV_RGB(255,0,0));
    
    Point pt3 = Point(_riXi.val[0], _riXi.val[1]);
    Point pt4 = Point(_riXi.val[2], _riXi.val[3]);
    cv::line(this->currFrameForVis, pt3, pt4, CV_RGB(0,0,255));
    
    int length = 3;
    Point pt5 = Point((_reXi.val[2] + _reXi.val[0]) / 2, (_reXi.val[3] + _reXi.val[1]) / 2 - length);
    Point pt6 = Point((_reXi.val[2] + _reXi.val[0]) / 2, (_reXi.val[3] + _reXi.val[1]) / 2 + length);
    cv::line(this->currFrameForVis, pt5, pt6, CV_RGB(255,255,255));
    
}
void RoadMark::drawMidLineSegments(int _p, cv::Vec4f _reXi, cv::Vec4f _riXi) {
    bool cond1 = _p == (int) round(p_max / 2.0);
    bool cond2 = _p == 0;
    bool cond3 = _p == p_max - 1;
    if(cond1 || cond2 || cond3) {
        Point pt1 = Point(_reXi.val[0], _reXi.val[1]);
        Point pt2 = Point(_reXi.val[2], _reXi.val[3]);
        cv::line(this->currFrameForVis, pt1, pt2, CV_RGB(255,0,0));

        Point pt3 = Point(_riXi.val[0], _riXi.val[1]);
        Point pt4 = Point(_riXi.val[2], _riXi.val[3]);
        cv::line(this->currFrameForVis, pt3, pt4, CV_RGB(0,0,255));

    }
}

void RoadMark::drawMarkings(cv::Mat _peaks, cv::Vec4f _reXi) {
    if (!_peaks.empty()) {
        cv::Scalar CYAN = CV_RGB(0, 255, 255);
//        cv::Scalar RED = CV_RGB(255, 0, 0);
        for (int p = 0; p < _peaks.rows; ++p) {
            Point pt1 = Point(_reXi.val[0] + _peaks.at<float>(p,0), _reXi.val[1]);
            Point pt2 = Point(_reXi.val[0] + _peaks.at<float>(p,0), _reXi.val[1]);
            cv::line(this->currFrameForVis, pt1, pt2, CYAN);
        }
    }
}

void RoadMark::grabLaneBoundaries() {
    
    cv::Mat currColorFrame;
    IplImage currIplColorFrame;

    this->video->readNextFrame();

    currColorFrame = this->video->getCurrFrame();
    currIplColorFrame = currColorFrame;

    this->clane->gd_Run(&currIplColorFrame);

    getAllCoords(3);
    
    // used for visualization
    this->currFrameForVis = currColorFrame.clone();
}

void RoadMark::setNewPathOut(std::string _path) {
    
    // ### Get the local date and hour ###
    auto now = chrono::system_clock::now();
    std::time_t now_c = chrono::system_clock::to_time_t(now);
    struct tm *parts = std::localtime(&now_c);
//    string dateTime = "-" + util::int2str(parts->tm_mday, 2) + util::int2str(1 + parts->tm_mon, 2) + "_" + util::int2str(parts->tm_hour, 2) + util::int2str(parts->tm_min, 2);
    string dateTime = "-" + util::int2str(parts->tm_mday, 2) + util::int2str(1 + parts->tm_mon, 2);
    
    // ### Create an output directory: video file name + current date and time ###
    int dotLocation = (int) this->config.fileName.find(".");
    std::string videoFileNameWithoutExt = this->config.fileName.substr(0, dotLocation);
    this->config.path_out = _path + videoFileNameWithoutExt + dateTime;

    // create dir with clip name
    std::string fullComand = "mkdir " + this->config.path_out;
    int retVal = system(fullComand.c_str());

}

void RoadMark::runVideoOnly() {
    
    for (int frm = this->config.initFrame; frm < this->totalFrames; ++frm) {
        /* Linear-parabolic model */
        grabLaneBoundaries(); // in ICS
        drawLaneBoundaries();
        
        cv::imshow("output", this->currFrameForVis);

        if (cv::waitKey(this->video->getDelay()) == 27) {
            break;
        }
    }
    
}

void RoadMark::mat2File(cv::Mat _inputArray, std::string _fileName) {
    std::ofstream outStream;
    _fileName = this->config.path_out + "/" + _fileName;
    outStream.open(_fileName.c_str(), std::ios::out);
    outStream << "xx = " << _inputArray << ";\n";
    outStream.close();
}

void RoadMark::initVideoWriter() {
    std::string extension = ".avi";
    int fourcc = CV_FOURCC('F','M','P','4');
    bool isColor = true;
    std::string fullFileName = this->config.path_out + "/out" + extension;

    cv::Size sz = this->video->getImgSize();
    this->videoOut.open(fullFileName, fourcc, this->video->getFPS(), this->video->getImgSize(), isColor);

}

void RoadMark::displayInfo(std::string _text, cv::Mat _np_t, double _mahalDistance, int _bound) {

    double fontScale = 1.5;
    int thickness = 2;
    int lineType = CV_AA;
    int fontFace = FONT_HERSHEY_PLAIN;
    cv::Scalar color = CV_RGB(255,255,255);

    // frame number
    cv::Size textsize = getTextSize(_text, fontFace, fontScale, thickness, 0);
    cv::Point org(10, 2 * textsize.height);
    cv::putText(this->currFrameForVis, _text, org, fontFace, fontScale, color, thickness, lineType );

    fontScale = 0.7;
    thickness = 1;
    if(this->currFrameForVis.rows > 480) {
        fontScale = 2.0;
        thickness = 2;
    }

}
