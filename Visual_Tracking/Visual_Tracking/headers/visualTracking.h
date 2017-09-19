#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath> 
#include <chrono>  // for high_resolution_clock

using namespace cv;
using namespace std;

// Visual-tracking packages
#include "cameraCalibration.h"
#include "projectiveGeometry.h"
#include "mathLinearAlgebra.h"
#include "controls.h"

// Number of mijs
#define _MNUM 5

//#define _IMSHOW
//#define _COUNT_TIME
#define _DEMO

#ifdef _DEMO
// Virtual Camera default extrinsics parameters
#define _TX 27.f
#define _TY 18.f
#define _TZ 80.f
#define _RX 160.f
#define _RY -11.f
#define _RZ 0.f
#else
// Virtual Camera default extrinsics parameters
#define _TX -28.f
#define _TY 30.f
#define _TZ 80.f
#define _RX 160.f
#define _RY -30.f
#define _RZ 0.f
#endif

// Visual tracker main function
void visualTracker(Cuboid3D &model, Cuboid3D &Data, int width, int height);

// Read model data from .x file
void modelData(float &length, float &height, float &width);

// Render function
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera &camera, Mat &imagePlane, Mat &imagePlaneObj, string type);

// Dissimilarity between data and model object
void dissimilarity(Cuboid2D model, Mat &imagePlane, Mat dataImage, int mNum, Mat &mijs, Mat &dijs, Mat &distTransform);

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, Vec3b colour, int lineType);

// Check which parts of the object are visible from the camera's given viewpoint
void visibilityCulling(Cuboid3D &cuboid3D, Cuboid2D &cuboid2D, Camera camera, Size size);

// Edge clip_PIng
bool edgeClip(vector <Point2f> &edgePxl, Size size);

// Check if a surface is visible from the camera's viewpoint
int backFaceCulling(vector <int> surface, vector <Point3f> vertices, Point3f t);

// Fixed time frame update
Cuboid2D updateFrame(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, Mat &imagePlane, Mat &imagePlaneModel, Mat &imagePlaneData, Mat &dataImg);

// Create camera
Camera createCam(Vec3f t, Vec3f r, float fov, int width, int height, vector <State> state);

// Clear visibility of the 3D model
void resetVisibility(Cuboid3D &cuboid3D);

// Display image plane
void dis_PImagePlane(string windowName, Mat imagePlane);

// Convert 3D _VERTICES and parameters values to Mat, states to enum Parameters and extract intrisincs matrix 
void extractDataForDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat &V, Mat &Vph, Mat &K, Mat &x, vector <Parameter> &xk);

// Convert STL vector to OpenCV Mat
template <typename T>
Mat convertSTLvector2Mat(vector <T> vec, int rows, int cols, int type);

// Calculate edges vectors
Mat edgesVectors(vector <vector <Point2f>> edges);

// Calculate edges direction vectors
Mat edgesDirectionVectors(vector <vector <Point2f>> edges);

// Calculate edges normal unit vectors
Mat edgesNormalVectors(Mat edgesVec);

// Calculate the subintervals mijs for all edges
Mat edgesSubIntervals(Mat edgesVec, vector<vector<Point2f>> edgesVertices, int mNum);

// Calculate for normal vectors for all mijs
Mat subIntervalsNormals(Mat mijs, Mat edgesNormals, float offset, Size size);

// Draw mijs and mijs normal lines on each edge of the model
void drawMijs(Mat mijs, Mat & imagePlane);

// Compute the distance transform for the data object
Mat computeDistanceTransform(Mat dataImage);

// Convert to display distance transform
Mat normalise(Mat Img);

// Calculate distance from each mij to data
Mat calculateDistance(Mat mijs, Mat distTransform);

// Compute distance transform gradient
void distTransformImageGradient(Mat distTransform, Mat & dxdist, Mat & dydist);

// Compute dijs first derivatives
Mat computeModelFirstDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat mijs, Mat distTransform);

// Gauss - Newton non linear fitting
vector<float> fittingGaussNewton(Camera virtualCam, Mat Jdijs, Mat dijs);

// Dimensions of inputs, are not equal
template<typename T>
void errorSize(string input1, string input2, T size1, T size2, string filename, int line);

// Demo
bool demo(Camera & realCam);
