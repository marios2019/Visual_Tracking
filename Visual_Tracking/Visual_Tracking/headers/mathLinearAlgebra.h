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

enum PartialDeriv { Dx, Dy };

using namespace cv;
using namespace std;

#include "rotations3D.h"
#include "error.h"

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


// Vector calculus

// Parameters of the D(x) distance function 
enum Parameter { X0, X1, X2, X3, X4, X5 };

// Compute the Jacobian matrix of the perspective projection process
// in respect to the parameters xk. 
// Inputs: matrix V = { 3D points }
//		   matrix K = { camera extrinsics }
//         vector xk = { parameters }
//         vector x = { parameters values }
// Output: matrix Jvh = { Jacobian matrix with the 3D projection homogeneous points }
Mat jacobianPerspectiveProjection(Mat V, Mat K, Mat x, vector <Parameter> xk);

// Compute the Jacobian matrix of the pixel coordinates
// Inputs: matrix Vph = { 3D homogeneours projection points }
//		   matrix Jvh = { first derivatives of projection homogeneous coordinates }
// Output: matrix Jvp = { first derivatives of pixel coordinates }
Mat jacobianPixelCoordinates(Mat Vph, Mat Jvh);

// Compute the Jacobian matrix of the 2D edges
// Inputs: matrix Jvp = { 2D projection points first derivatives}
//		   vector edges2DPtr = { pointers to 2D projection vertices }
// Output: matrix Jep = { first derivatives of 2D edges }
Mat jacobianEdges(Mat Jvp, vector <vector <int>> edges2DPtr);

// Compute mijs first derivatives
Mat jacobianMijs(Mat Jep, int mNum);

// Compute first derivatives of distances dijs
Mat jacobianDijs(Mat mijs, Mat Jmijs, Mat dxDist, Mat dyDist);

// Compose camera's pose matrix first derivative
Mat cameraPoseFirstDerivative(Mat x, Parameter xk);

// Return camera position vector first derivative in respect of the state parameters
Mat cameraPositionFirstDerivative(Parameter xk);

// Return camera rotation matrix first derivative in respect of the state parameters
Mat cameraRotationFirstDerivative(Parameter xk, Mat x);

// Return rotation matrix first derivative in respect parameter xk
Mat rotationFirstDerivative(Parameter xk, float theta);

// Computer the first derivative of the inverse of 3D rotation matrix
Mat inverseRotationMatrixDerivative(Mat R, Mat dR);

// Image gradient to one direction
Mat imageGradient(Mat Img, PartialDeriv partial);
