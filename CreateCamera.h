#ifndef CREATECAMERA_H_INCLUDED
#define CREATECAMERA_H_INCLUDED



#endif // CREATECAMERA_H_INCLUDED

// Include Files
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>

// namespcaes declaration
using namespace cv;
using namespace std;


//! A class for histogram analysis
class CreateCamera{
private:

protected:

public:
    double f; // focal length - [m]
    float PixelHeight; // Pixel Height[m]
    float PixelWidth; // Pixel Width [m]
    double Yaw; // Yaw angle [reads] (psi)
    int Roll; // Roll Angle [rads] (phi)
    double Pitch; //Pitch Angle [rads] (theta)
    int x; // camera Longitudinal Offset (from centre of vehicle) [m]
    int y; // camera Lateral Offset (from centre of vehicle) [m]
    double z; // camera Height 9from ground plane) [m]
    int XPixels; //image width - [pixels] (e.g. 640)
    int YPixels; //image height - [pixels] (e.g. 240)

    /*roll image */
    double cosRoll;
    double sinRoll;

    /*pitch camera */
    double tanPitch;
    double cosPitch;

    /*yaw */
    double cosYaw;
    double sinYaw;


// Constructor & Destructor
CreateCamera();
~CreateCamera();

// Public Function Prototypes
void sCamera(String Vehicle);

};
