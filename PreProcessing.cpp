//Include Files
#include <fstream>
#include "PreProcessing.h"
#include "SaliencyDetector.h"

//namespace declaration
using namespace cv;
using namespace std;

SaliencyDetector sd;

//! Class constructor
PreProcessing::PreProcessing()
{

}

//! Class destructor
PreProcessing::~PreProcessing()
{

}

void PreProcessing::Saturation(Mat image)
{
//ofstream myfile;
//myfile.open("finalFrameFusedObs.txt");
//imshow("Original Image", image);
/*for(int i = 0; i < image.rows; i++)
{
    for(int j= 0; j < image.cols; j++)
    {
        cout  <<int(image.at<Vec3b>(0,0)[0]) << " " ;
        //myfile << int(image.at<Vec3b>(i,j)[0]) << " " ;
    }
    cout << endl;
}*/
//myfile.close();

//**********************************************************************************************************
//---Saturation (based on the S channel of the HSL colourspace)
//**********************************************************************************************************
//  by converting the RGB colourspace to HSL, the saturation channel is exetracted;
cvtColor(image, hlsImage,CV_BGR2HLS);

// the saturation channel is extracted
//Mat satChannels[3];
split(hlsImage,satChannels); //H is satChannels[0], L is satChannels[1], S is satChannels[2]

// and futher resized to a coarse 64x48 saturation intensity map by Guassian pyramid decomposition of
// the 320x240 input image
//imshow("Display before ", satChannels[2]);
pyrDown(satChannels[2], dst, Size(hlsImage.cols/2, hlsImage.rows/2));

//pyrDown(dst, dst2, Size(dst.cols/2, dst.rows/2));

// resize image
//pyrUp(dst2, dst3, Size( dst2.cols*2, dst2.rows*2 ));
pyrUp(dst, hls_320x240, Size( dst.cols*2, dst.rows*2 ));


//pyrUp(dst3, hls_320x240, Size( dst3.cols*2, dst3.rows*2 ));

image.release();
hlsImage.release();
dst.release();
dst2.release();
dst3.release();
}

//***********************************************************************************************************
//---Saturation-based texture
//***********************************************************************************************************
// This can be derived by applying an edge detector on the S channel of the HSL colourspace.
// Then the texture is defined as the density of edges in different parts of th image.
// Practically, this is achieved by Guassian pyramid decomposition of the output of the Sobel edge detector
// in order to generate a low-resolution 64x48 grid.

// Source: http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/sobel_derivatives/sobel_derivatives.html
///remove image you are not using it
void PreProcessing::SaturationTexture(Mat image)
{
//blur the S channel of the HSL colourspace to reduce noise with a kernel 3x3.
GaussianBlur(satChannels[2],smoothS, Size(3,3),0,0, BORDER_DEFAULT);
//blur(satChannels[2],smoothS, Size(3,3));

int scale = 1;
int delta = 0;
int ddepth = CV_16S;

// we calculate the "derivatives" in x and y directons using Sobel.
// Gradient X
//Sobel(hlsImage, grad_x, ddepth, 1,0,3,scale,delta,BORDER_DEFAULT);
Sobel(smoothS, grad_x, ddepth, 1,0,3,scale,delta,BORDER_DEFAULT);

// Gradient Y
//Sobel(hlsImage, grad_y, ddepth, 0,1,3,scale, delta,BORDER_DEFAULT);
Sobel(smoothS, grad_y, ddepth, 0,1,3,scale, delta,BORDER_DEFAULT);

// We convert our partial results to CV_8U
convertScaleAbs(grad_x,abs_grad_x);
convertScaleAbs(grad_y,abs_grad_y);

// We try to approximate the gradient by adding both directional gradients (note this is
// not an exact calculation).
addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

// Guassian pyramid decomposition of the output of the Sobel edge detector.
pyrDown(grad, gradDST, Size(grad.cols/2, grad.rows/2)); //this is not smaller enough
pyrDown(gradDST, gradDST2, Size(gradDST.cols/2, gradDST.rows/2));

//image resize
pyrUp(gradDST2, gradDST3, Size( gradDST2.cols*2, gradDST2.rows*2 ));
pyrUp(gradDST3, hls2_320x240, Size( gradDST3.cols*2, gradDST3.rows*2 ));

image.release();
smoothS.release();
grad_x.release();
grad_y.release();
abs_grad_x.release();
abs_grad_y.release();
grad.release();
gradDST.release();
gradDST2.release();
gradDST3.release();
}

//***********************************************************************************************************
//--- Mean Chroma (based on combining the Cb and Cr components of the YCbCr colourspace
//    with the A component of the LAB colourspace
//***********************************************************************************************************
// http://stackoverflow.com/questions/24404841/extracting-the-y-cb-cr-from-a-mat-in-c-and-create-a-mat-with-only-the-y-valu

void PreProcessing::MeanChroma(Mat image)
{
// convert image to YCbCr
cvtColor(image,ycrcb,CV_BGR2YCrCb);

Mat channels[3];
split(ycrcb,channels); //y is channels[0], cb is channels[1], cr is channels[2]

//scaled to fit the 0-255 (8-bit) range
convertScaleAbs(channels[1],abs_Cb);
convertScaleAbs(channels[2],abs_Cr);

//and subsequently their mean value is derived.
Mat compsCbsCr = ((abs_Cb+abs_Cr)/1.5);
Mat lab,abs_A;

//convert input image to Lab
cvtColor(image,lab,CV_BGR2Lab);

//split channels
Mat labChannels[3];
split(lab,labChannels);

//scaled to fit the 0-255 (8-bit) range
convertScaleAbs(labChannels[1],abs_A);

//Mat chromaMap;
chromaMap = (compsCbsCr+abs_A)/2;

pyrDown(chromaMap, chroma_1, Size(chromaMap.cols/2, chromaMap.rows/2));
pyrDown(chroma_1, chroma_2, Size(chroma_1.cols/2, chroma_1.rows/2));
//chromaMap.release();
chroma_1.release();
//image resize
pyrUp(chroma_2, chroma_3, Size( chroma_2.cols*2, chroma_2.rows*2 ));
pyrUp(chroma_3, meanChroma_320x240, Size( chroma_3.cols*2, chroma_3.rows*2 ));

image.release();
ycrcb.release();
abs_Cb.release();
abs_Cr.release();
compsCbsCr.release();
lab.release();
abs_A.release();
chroma_1.release();
chroma_2.release();
chroma_3.release();
channels[1].release();
channels[2].release();
//labChannels[3].release();

satChannels[1].release();
satChannels[2].release();
}

//***********************************************************************************************************
//--- Chroma-based texture (based on the Cb and Cr components of the YCbCr colourspace)
//***********************************************************************************************************
// This is derived by calculating the mean value of the Cb and Cr components to generate a new chroma map.
// The Sobel edge detector is subsequently applied to this map in order to calculate a chroma-baed texture
// density using the process described in the saturation-based texture above.
///you dont need image here
void PreProcessing::ChromaTexture(Mat image)
{
//blur the chromaMap to reduce noise with a kernel 3x3.
//GaussianBlur(chromaMap,smoothC, Size(3,3),0,0, BORDER_DEFAULT);
blur(chromaMap,smoothC, Size(3,3));

int scaleC = 1;
int deltaC = 0;
int ddepthC = CV_16S;

// we calculate the "derivatives" in x and y directons using Sobel.
// Gradient X
Sobel(smoothC, gradC_x, ddepthC, 1,0,3,scaleC,deltaC,BORDER_DEFAULT);

// Gradient Y
Sobel(smoothC, gradC_y, ddepthC, 0,1,3,scaleC, deltaC,BORDER_DEFAULT);

// We convert our partial results to CV_8U
convertScaleAbs(gradC_x,abs_gradC_x);
convertScaleAbs(gradC_y,abs_gradC_y);

// We try to approximate the gradient by adding both directional gradients (note this is
// not an exact calculation).
addWeighted( abs_gradC_x, 0.5, abs_gradC_y, 0.5, 0, gradC);

// Guassian pyramid decomposition of the output of the Sobel edge detector.
pyrDown(gradC, gradC_DST, Size(gradC.cols/2, gradC.rows/2));
pyrUp(gradC_DST, gradC_DST2, Size(gradC_DST.cols*2, gradC_DST.rows*2));

pyrDown(gradC_DST2, gradC_DST3, Size(gradC_DST2.cols/2, gradC_DST2.rows/2));
pyrDown(gradC_DST3, gradC_DST4, Size(gradC_DST3.cols/2, gradC_DST3.rows/2));

//image resize
pyrUp(gradC_DST4, gradC_DST5, Size( gradC_DST4.cols*2, gradC_DST4.rows*2 ));
pyrUp(gradC_DST5, chroma_320x240, Size( gradC_DST5.cols*2, gradC_DST5.rows*2 ));


//Mat chromaTextureMap;
chromaTextureMap = chroma_320x240/1.1;

image.release();
smoothC.release();
gradC_x.release();
gradC_y.release();
abs_gradC_x.release();
abs_gradC_y.release();
gradC.release();
gradC_DST.release();
gradC_DST2.release();
gradC_DST3.release();
gradC_DST4.release();
gradC_DST5.release();
chromaMap.release();
chroma_320x240.release();
}

//***********************************************************************************************************
//--- DIVoG
//***********************************************************************************************************
void PreProcessing::DiVoG(const Mat& image)
{
/* Using DiVoG */
Mat salient;   //dst and salient image

// convert from Mat to old style C IplImage object
IplImage* newImage = new IplImage(image);

salient = image.clone();
IplImage* divogMap = new IplImage(salient);
sd.DIVoG_Saliency(newImage,divogMap,3,true,true);

// convert from Iplimage to Mat
Mat imgMat(divogMap);

// Convert to single channel grayscale image
cvtColor(imgMat,divog_320x240,CV_BGR2GRAY);
}
