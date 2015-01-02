#ifndef NORMEDGES_H_INCLUDED
#define NORMEDGES_H_INCLUDED



#endif // NORMEDGES_H_INCLUDED


// Pragmas
#pragma once

// Include Files
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>

// namespcaes declaration
using namespace cv;
using namespace std;


//! A class for Traversability Map
class NormEdges{
private:

protected:

public:
    int offset;
    Mat subArray;
    Mat finalArray;


// Constructor & Destructor
NormEdges();
~NormEdges();

// Public Function Prototypes
void NormaliseEdges(Mat array1D, int arrayLength, int windowSize);

};
