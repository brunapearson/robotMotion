#ifndef PREPROCESSING_H_INCLUDED
#define PREPROCESSING_H_INCLUDED



#endif // PREPROCESSING_H_INCLUDED

// Pragmas
#pragma once


// Include Files
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>

//#include "HistogramAnalysis.h"

// namespcaes declaration
using namespace cv;
using namespace std;


//! A class for pre processing image
class PreProcessing {
private:


protected:


public:
Mat hlsImage, hls_320x240, dst, dst2, dst3;
Mat satChannels[3];
Mat grad, grad_x, grad_y, gradDST, gradDST2, gradDST3, smoothS, hls2_320x240;
Mat abs_grad_x, abs_grad_y;
Mat ycrcb,abs_Cb,abs_Cr;
Mat chroma_1, chroma_2, chroma_3, meanChroma_320x240;
Mat chromaMap;
Mat gradC, gradC_x, gradC_y, gradC_DST, gradC_DST2, gradC_DST3, gradC_DST4, gradC_DST5, smoothC, chroma_320x240;
Mat abs_gradC_x, abs_gradC_y;
Mat chromaTextureMap;
Mat divog_320x240;


// Constructor & Destructor
PreProcessing();
~PreProcessing();


// Public Function Prototype
void Saturation(Mat image);
void SaturationTexture(Mat image);
void MeanChroma(Mat image);
void ChromaTexture(Mat image);
void DiVoG(const Mat& image);
};
