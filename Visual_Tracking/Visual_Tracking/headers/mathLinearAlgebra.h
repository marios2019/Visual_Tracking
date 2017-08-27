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

#define PI 3.14159265f

using namespace cv;
using namespace std;

#include "rotations3D.h"

// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f vec);

// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f vec);

// Compute squared euclidean distance between two 2D points
float euclideanDistanceSquared(Point2i p, Point2i q);

// Compute euclidean distance between two 2D points
float euclideanDistance(Point2f p, Point2f q);

// Return a skew symmetric matrix of a 3D vector, used for cross product
Mat skewMat(Vec3f m);

// Decompose camera pose matrix
void decomposeCameraPose(Mat E, Mat& R, Vec3f& t);

// Decompose euclidean transformation matrix expressed in homogeneous coordinates - matrix size 4 x 4
void decomposeEuclidean(Mat E, Mat &R, Vec3f &t);

//  Create 4 x 4 camera pose matrix
void cameraPose(Mat R, Mat t, Mat &E);

// Perspective projection
Point3f perspectiveProjection(Point3f point, Mat P);

// 3D Cartesian to 4D homogeneous coordinates conversion
Vec4f cart2hmgns(Vec3f carte);

// Check if the input matrix has the desirable dimensions
bool checkMatrixSize(Size size, int rows, int cols);