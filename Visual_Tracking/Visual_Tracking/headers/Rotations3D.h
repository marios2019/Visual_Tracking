#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath> 

using namespace cv;
using namespace std;

#define PI 3.14159265f

// Euler angles
// Return x-axis rotation matrix
Mat rotationX(float thetaX);

// Return y-axis rotation matrix
Mat rotationY(float thetaY);

// Return z-axis rotation matrix
Mat rotationZ(float thetaZ);

// Return rotation matrix according to Euler angles
Mat rotationEuler(float thetaX, float thetaY, float thetaZ);

// Return Euler angles from rotation matrix
Vec3f matrix2euler(Mat R);

// Degrees to radians
float deg2rad(float degrees);

// Radians to degrees
float rad2deg(float radians);

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat R);