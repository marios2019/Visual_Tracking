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

#define _PI 3.14159265f
#define _DEGREES 0
#define _RADIANS 1

// Euler angles
// Return x-axis rotation matrix
template <typename T> Mat rotationX(T thetaX, int type = _DEGREES)
{
	if (type == _DEGREES)
	{
		thetaX = deg2rad(thetaX);
	}
	else if (type != _RADIANS)
	{
		errorAngleType(__FILE__, __LINE__ - 6);
	}

	return (Mat_<T>(3, 3) << 1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
}

// Return y-axis rotation matrix
template <typename T> Mat rotationY(T thetaY, int type = _DEGREES)
{
	if (type == _DEGREES)
	{
		thetaY = deg2rad(thetaY);
	}
	else if (type != _RADIANS)
	{
		errorAngleType(__FILE__, __LINE__ - 6);
	}

	return (Mat_<T>(3, 3) << cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
}

// Return z-axis rotation matrix
template <typename T> Mat rotationZ(T thetaZ, int type = _DEGREES)
{
	if (type == _DEGREES)
	{
		thetaZ = deg2rad(thetaZ);
	}
	else if (type != _RADIANS)
	{
		errorAngleType(__FILE__, __LINE__ - 6);
	}

	return (Mat_<T>(3, 3) << cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
}

// Return rotation matrix according to Euler angles
template <typename T> Mat eulerAngles2Matrix(T thetaX, T thetaY, T thetaZ, int type = _DEGREES)
{
	if (type == _DEGREES)
	{
		thetaX = deg2rad(thetaX);
		thetaY = deg2rad(thetaY);
		thetaZ = deg2rad(thetaZ);
	}
	else if (type != _RADIANS)
	{
		errorAngleType(__FILE__, __LINE__ - 8);
	}
	return  rotationY(thetaY, type) * rotationZ(thetaZ, type) * rotationX(thetaX, type);
}

// Return Euler angles from rotation matrix, expressed in degrees or radians
Vec3f matrix2euler(Mat R, int type = _DEGREES);

// Degrees to radians
template <typename T> T deg2rad(T degrees)
{
	return degrees * static_cast<T>(_PI / 180.f);
}

// Radians to degrees
template <typename T> T rad2deg(T radians)
{
	return radians * static_cast<T>(180.f / _PI);
}

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat R);

// Convert euler angles to axis angle
// Input angles are in degrees or radians
// Output contains axis multiplied by rotation angle, expressed in degrees or radians
Vec3f euler2AxisAngle(float thetaX = 0.f, float thetaY = 0.f, float thetaZ = 0.f, int type = _DEGREES);

// Axis angle to euler angle
// Input contains axis multiplied by rotation angle, expressed in degrees or radians
// Output euler angles are expressed in degrees or radians
Mat axisAngle2euler(Vec3f axisAngle = Vec3f(), int type = _DEGREES);

// Calculate rotation matrix from axis r and angle
// expressed in degrees or radians, using Rodrigues's formula
Mat axisAngle2Matrix(Vec3f axisAngle = Vec3f(), int type = _DEGREES);

// Calculate axis angle representation
// by a rotation matrix. 
// Output contains axis multiplied by rotation angle, expressed in degrees or radians
Vec3f matrix2AxisAngle(Mat R = Mat::eye(3, 3, CV_32F), int type = _DEGREES);

// Axis angle ordered pair to axis angle
Vec3f axisOrdPair2AxisAngle(Vec4f axisPair);

// Axis angle to axis angle ordered pair
Vec4f axisAngle2AxisOrdPair(Vec3f axisAngle);

// Invalid angle type
void errorAngleType(string filename, int line);
