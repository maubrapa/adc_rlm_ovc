#include "Tcc.h"

Tcc::Tcc(int _windowSize) {
    this->counter = cv::Mat::zeros(1,5, CV_8UC1);
    this->T = _windowSize;
}

void Tcc::print_tw() {
    for(std::list<int>::iterator it = tw.begin(); it != tw.end(); ++it)
        std::cout << *it << " ";
    std::cout << std::endl;
}

void Tcc::print_counter() {
    std::cout << this->counter << std::endl;
}

void Tcc::update(int _c, int* _cout) {

    this->counter.at<uchar>(0,_c-1) = (int)this->counter.at<uchar>(0,_c-1) + 1;

    if(tw.size() <= this->T) {
        tw.push_back(_c);
    } else {
        tw.push_back(_c);
        int i = tw.front();
        this->counter.at<uchar>(0,i-1) = (int)this->counter.at<uchar>(0,i-1) - 1;
        tw.pop_front();
    }

    double maxVal;
    int maxIdx[2];
    cv::minMaxIdx(this->counter, NULL, &maxVal, NULL, maxIdx);
    *_cout = maxIdx[1] + 1;
}

