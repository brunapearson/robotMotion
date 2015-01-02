#include "CreateCamera.h"

//namespace declaration
using namespace cv;
using namespace std;

//! Class constructor
CreateCamera::CreateCamera()
{

}

//! Class destructor
CreateCamera::~CreateCamera()
{

}

void CreateCamera::sCamera(String Vehicle)
{
    if(Vehicle =="Default"){
    f = 0.0017987; // focal length - [m]
    PixelHeight = 2.9634e-006; // Pixel Height[m]
    PixelWidth = 2.3423e-006; // Pixel Width [m]
    Yaw = 0.00015837; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.36665; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.58; // camera Height 9from ground plane) [m]
    XPixels = 320; //image width - [pixels] (e.g. 640)
    YPixels = 240; //image height - [pixels] (e.g. 240)
    }

    if(Vehicle=="Shrivenham31Oct2007")
    {
    f = 0.0017032; // focal length - [m]
    PixelHeight = 2.2208e-006; // Pixel Height[m]
    PixelWidth = 2.5714e-006; // Pixel Width [m]
    Yaw = 0.00015837; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.19875; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.74; // camera Height 9from ground plane) [m]
    XPixels = 720; //image width - [pixels] (e.g. 640)
    YPixels = 576; //image height - [pixels] (e.g. 240)
    }

    if(Vehicle=="2008_01_03-MGV_Calibration")
    {
    f = 0.0018011; // focal length - [m]
    PixelHeight = 1.9499e-006; // Pixel Height[m]
    PixelWidth = 2.4646e-006; // Pixel Width [m]
    Yaw = 0.00016313; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.19875; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.74; // camera Height 9from ground plane) [m]
    XPixels = 720; //image width - [pixels] (e.g. 640)
    YPixels = 576; //image height - [pixels] (e.g. 240)
    XPixels = 640;
    YPixels = 480;
    }

    if(Vehicle=="2008_02_06 Shirley Park")
    {
    f = 0.0017987; // focal length - [m]
    PixelHeight = 2.9634e-006; // Pixel Height[m]
    PixelWidth = 2.3423e-006; // Pixel Width [m]
    Yaw = 0.00016313; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.36665; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.58; // camera Height 9from ground plane) [m]
    XPixels = 640;
    YPixels = 480;
    }

    if(Vehicle=="DV Video")
    {
    f = 0.0017987; // focal length - [m]
    PixelHeight = 2.9634e-006; // Pixel Height[m]
    PixelWidth = 2.3423e-006; // Pixel Width [m]
    Yaw = 0.00015837; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.36665; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.58; // camera Height 9from ground plane) [m]
    XPixels = 720;
    YPixels = 576;
    }

    if(Vehicle =="2008_05_29 MGV")
    {
    f = 0.0017987; // focal length - [m]
    PixelHeight = 2.9634e-006; // Pixel Height[m]
    PixelWidth = 2.3423e-006; // Pixel Width [m]
    Yaw = 0.00016451; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.31841; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.65; // camera Height 9from ground plane) [m]
    XPixels = 640;
    YPixels = 480;
    }

    if(Vehicle =="LowRes")
    {
    f = 0.0017987; // focal length - [m]
    PixelHeight = 2.9634e-006; // Pixel Height[m]
    PixelWidth = 2.3423e-006; // Pixel Width [m]
    Yaw = 0.00016451; // Yaw angle [reads] (psi)
    Roll = 0; // Roll Angle [rads] (phi)
    Pitch = 0.31841; //Pitch Angle [rads] (theta)
    x = 0; // camera Longitudinal Offset (from centre of vehicle) [m]
    y = 0; // camera Lateral Offset (from centre of vehicle) [m]
    z = 0.65; // camera Height 9from ground plane) [m]
    XPixels = 320;
    YPixels = 240;
    }

    /*roll image */
    cosRoll = cos(Roll);
    sinRoll = sin(Roll);

    /*pitch camera */
    tanPitch = tan(Pitch);
    cosPitch = cos(Pitch);

    /*yaw */
    cosYaw = cos(Yaw);
    sinYaw = sin(Yaw)*10000;

}
