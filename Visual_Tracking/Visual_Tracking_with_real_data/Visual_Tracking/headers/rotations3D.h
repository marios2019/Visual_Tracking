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
// Return x-axis rotation matrix, input angle expressed in radians
template <typename T> Mat rotationX(T thetaX)
{
	return (Mat_<T>(3, 3) << 1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
}

// Return y-axis rotation matrix, input angle expressed in radians
template <typename T> Mat rotationY(T thetaY)
{
	return (Mat_<T>(3, 3) << cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
}

// Return z-axis rotation matrix, input angle expressed in radians
template <typename T> Mat rotationZ(T thetaZ)
{
	return (Mat_<T>(3, 3) << cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
}

// Return rotation matrix according to Euler angles, angles expressed in radians
template <typename T> Mat eulerAngles2Matrix(T thetaX, T thetaY, T thetaZ)
{
	return  rotationY(thetaY) * rotationZ(thetaZ) * rotationX(thetaX);
}

// Return Euler angles from rotation matrix, expressed in radians
Vec3f matrix2euler(Mat R);

// Degrees to radians
template <typename T> T deg2rad(T degrees)
{
	return degrees * static_cast<T>(PI / 180.f);
}

// Radians to degrees
template <typename T> T rad2deg(T radians)
{
	return radians * static_cast<T>(180.f / PI);
}

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat R);

// Convert euler angles to axis angle
// Input angles are in radians
// Output axis is multiplied by the rotation angle, expressed in radians
Vec3f euler2AxisAngle(float thetaX = 0.f, float thetaY = 0.f, float thetaZ = 0.f);

// Axis angle to euler angle
// Input axis is multiplied by the rotation angle, expressed in radians
// Output euler angles are expressed in radians
Vec3f axisAngle2euler(Vec3f axis = Vec3f());

// Calculate rotation matrix from axis r which is multiplied
// by angle expressed in radians, using Rodrigues's formula
Mat axisAngle2Matrix(Vec3f axis = Vec3f());

// Convert rotation matrix to axis angle representation,
// where the output axis is multiplied by the rotation
// angle, expressed in radian
Vec3f matrix2AxisAngle(Mat R = Mat::eye(3, 3, CV_32F));

// Vec4f axis angle to Vec3f axis angle, 
// where the output axis angle is multiplied by the angle in radians
Vec3f axisAngleConversion(Vec4f axisAngle);


// Vec3f axis angle to Vec4f axis angle, 
// where the output axis angle is divided by the angle in radians
Vec4f axisAngleConversion(Vec3f axisAngle);
