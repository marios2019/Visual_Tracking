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

// Euler angles
// Return x-axis rotation matrix
Mat getRotationX(float);
// Return y-axis rotation matrix
Mat getRotationY(float);
// Return z-axis rotation matrix
Mat getRotationZ(float);
// Return rotation matrix according to Euler angles
Mat getRotationEuler(float, float, float);
// Return Euler angles from rotation matrix
Vec3f matrixToEuler(Mat);

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat);

// Euler angles
// Return x-axis rotation matrix
Mat getRotationX(float thetaX)
{
	return (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
}

// Return y-axis rotation matrix
Mat getRotationY(float thetaY)
{
	return (Mat_<float>(3, 3) << cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
}

// Return z-axis rotation matrix
Mat getRotationZ(float thetaZ)
{
	return (Mat_<float>(3, 3) << cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
}

// Return rotation matrix according to Euler angles
Mat getRotationEuler(float thetaX, float thetaY, float thetaZ)
{
	return  getRotationY(thetaY) * getRotationZ(thetaZ) * getRotationX(thetaX);
}

// Return Euler angles from rotation matrix
Vec3f matrixToEuler(Mat R)
{
	if (!IsRotationMatrix(R))
	{
		cout << "Input matrix R is not a 3D rotation matrix." << endl;
		exit(EXIT_FAILURE);
	}

	Vec3f eulerAngles;
	if (abs(R.at<float>(1, 0)) != 1.f)
	{
		eulerAngles.val[0] = atan2f(-R.at<float>(1, 2), R.at<float>(1, 1));
		eulerAngles.val[1] = atan2f(-R.at<float>(2, 0), R.at<float>(0, 0));
		eulerAngles.val[2] = asinf(R.at<float>(1, 0));
	}
	else
	{
		eulerAngles.val[0] = 0.f;
		eulerAngles.val[1] = atan2f(R.at<float>(0, 2), R.at<float>(2, 2));
		eulerAngles.val[2] = asinf(R.at<float>(1, 0));
	}

	return eulerAngles;
}

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat R)
{
	Mat Iprog = R * R.t();
	Mat I = Mat::eye(3, 3, Iprog.type());

	return norm(I, Iprog, NORM_L2) < 1e-6;
}