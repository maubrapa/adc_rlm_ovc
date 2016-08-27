#ifndef TCC_H
#define TCC_H

#include <iostream>
#include <list>
#include <opencv2/core/core.hpp>

class Tcc {

private:
    cv::Mat counter;
    int T; // temporal window size
    std::list<int> tw;

public:
    Tcc(int _windowSize);
    void update(int _c, int *_cout);
    void print_counter();
    void print_tw();

};

#endif // TCC_H
