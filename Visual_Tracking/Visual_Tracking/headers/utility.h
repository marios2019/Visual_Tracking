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

#include "projectiveGeometry.h"
#include "cameraCalibration.h"

// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f);
// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f);
// Compute euclidean distance between two points
float euclideanDistanceSquared(Point2i, Point2i);
// Return a skew symmetric matrix of a 3D vector, used for cross product
Mat skewMat(Vec3f);
// Decompose camera pose matrix
void decomposeCameraPose(Mat, Mat&, Vec3f&);
// Decompose euclidean transformation matrix expressed in homogeneous coordinates - matrix size 4 x 4
void decomposeEuclidean(Mat, Mat&, Vec3f&);
//  Create 4 x 4 camera pose matrix
void cameraPose(Mat, Mat, Mat&);

// Overload
// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f vec)
{
	float norm = sqrt(powf(vec.val[0], 2.f) + powf(vec.val[1], 2.f) + powf(vec.val[2], 2.f));

	return max(norm, 0.0001f);
}

// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f vec)
{
	return sqrt(powf(vec.val[0], 2.f) + powf(vec.val[1], 2.f));
}

// Compute euclidean distance between two points
float euclideanDistanceSquared(Point2i p, Point2i q)
{
	return (powf((float)(q.x - p.x), 2.f) + powf((float)(q.y - p.y), 2.f));
}

// Return a skew symmetric matrix of a 3D vector, used for cross product
Mat skewMat(Vec3f m)
{
	Mat sk = (Mat_<float>(3, 3) << 0, -m.val[2], m.val[1], m.val[2], 0, -m.val[0], -m.val[1], m.val[0], 0);

	return sk;
}

// Decompose camera pose matrix
void decomposeCameraPose(Mat E, Mat& R, Vec3f& t)
{
	// Check if matrix is 3 x 4
	Size size = E.size();
	int rows = size.height;
	int cols = size.width;

	if ((rows != 3) || (cols != 4))
	{
		cout << "The camera pose matrix must be 3 x 4." << endl;
		return;
	}

	// Extract 3D rotation matrix
	Mat Rd = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rd = E(r).clone();
	transpose(Rd, Rd);
	if (!IsRotationMatrix(Rd))
	{
		cout << "The input camera pose matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rd.copyTo(R);

	// Extract position vector
	Mat td;
	Vec3f Rt;
	Rt.val[0] = E.at<float>(0, 3);
	Rt.val[1] = E.at<float>(1, 3);
	Rt.val[2] = E.at<float>(2, 3);
	Rt = -Rt;
	td = R * Mat(Rt);
	t = td;
}

// Decompose euclidean transformation matrix expressed in homogeneous coordinates - matrix size 4 x 4
void decomposeEuclidean(Mat E, Mat &R, Vec3f &t)
{
	// Check if matrix is 4 x 4
	Size size = E.size();
	int rows = size.height;
	int cols = size.width;

	if ((rows != 4) || (cols != 4))
	{
		cout << "The euclidean trasformation matrix must be 4 x 4." << endl;
		return;
	}

	// Extract 3D rotation matrix
	Mat Rd = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rd = E(r).clone();
	if (!IsRotationMatrix(Rd))
	{
		cout << "The input euclidean transformation matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rd.copyTo(R);

	// Extract position vector
	t.val[0] = E.at<float>(0, 3);
	t.val[1] = E.at<float>(1, 3);
	t.val[2] = E.at<float>(2, 3);
}

//  Create 4 x 4 camera pose matrix
void cameraPose(Mat R, Mat t, Mat &E)
{
	if (!IsRotationMatrix(R))
	{
		cout << "The input matrix R is not a 3D rotation matrix." << endl;
		return;
	}

	hconcat(R.t(), -R.t() * t, E);
	Mat row = (Mat_<float>(1, 4) << 0.f, 0.f, 0.f, 1.f);
	E.push_back(row);
}