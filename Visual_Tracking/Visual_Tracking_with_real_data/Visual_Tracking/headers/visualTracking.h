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

#define _COUNT_TIME

// Visual tracker main function
void visualTracker(Cuboid3D &model);

// Read model data from .x file
void modelData(string filename, float &length, float &height, float &width);

// Read from .txt file camera parameters
void configCameraData(string configFilename, float & tX, float & tY, float & tZ, float & RX, float & RY, float & RZ, float & fov);

// Miscellaneous parameters	- image plane, fitting and Canny edge
void configParamsData(string configParamsFilename, int & imageWidth, int & imageHeight, int & mNum, int & maxIterations, float & threshold, float & ratio, int & kernel);

// Update filenames
void readFilenames(string &srcImageFilename, string &videoFilename, string &configParamsFilename, string & modelFilename, string & configCameraFilename, Mat & srcData);

// Render function
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera &camera, Mat &imagePlane, Vec3b colour);

// Dissimilarity between data and model object
void dissimilarity(vector <vector <Point2f>> edgesVertices, Mat &imagePlane, Mat dataImage, Mat distTransform, int mNum, Mat &mijs = Mat(), Mat &dijs = Mat());

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Vec3b colour, int lineType);

// Check which parts of the object are visible from the camera's given viewpoint
void visibilityCulling(Cuboid3D &cuboid3D, Cuboid2D &cuboid2D, Camera camera, Size size);

// Edge clip_PIng
bool edgeClip(vector <Point2f> &edgePxl, Size size);

// Check if a surface is visible from the camera's viewpoint
bool backFaceCulling(vector <int> surface, vector <Point3f> vertices, Point3f t);

// Front camera visibility
bool frontCameraVisibilty(vector<int> surface, vector<Point3f> vertices, Camera camera);

// Create camera
Camera createCam(Vec3f t, Vec3f r, float fov, int width, int height, vector <State> state);

// Clear visibility of the 3D model
void resetVisibility(Cuboid3D &cuboid3D);

// Display image plane
void displayImagePlane(string windowName, Mat imagePlane);

// Convert 3D _VERTICES and parameters values to Mat, states to enum Parameters and extract intrisincs matrix 
void extractDataForDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat &V, Mat &Vph, Mat &K, Mat &x, vector <Parameter> &xk);

// Convert STL vector to OpenCV Mat
template <typename T>
Mat convertSTLvector2Mat(vector <T> vec, int rows, int cols, int type);

// Calculate edges vectors
Mat edgesVectors(vector <vector <Point2f>> edges);

// Calculate edges direction vectors
Mat edgesDirectionVectors(vector <vector <Point2f>> edges);

// Calculate the subintervals mijs for all edges
Mat edgesSubIntervals(Mat edgesVec, vector<vector<Point2f>> edgesVertices, int mNum);

// Calculate for normal vectors for all mijs
Mat subIntervalsNormals(Mat mijs, Mat edgesNormals, float offset, Size size);

// Draw mijs and mijs normal lines on each edge of the model
void drawMijs(Mat mijs, Mat & imagePlane);

// Convert to display distance transform
Mat normalise(Mat Img);

// Calculate distance from each mij to data
Mat calculateDistance(Mat mijs, Mat distTransform);
// Der Tukey estimator
float tukeyEstimator(float residual, float threshold);

// Compute distance transform gradient
void distTransformImageGradient(Mat distTransform, Mat & dxdist, Mat & dydist);

// Compute dijs first derivatives
Mat computeModelFirstDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat mijs, Mat dxDist, Mat dyDist);

// Gauss - Newton non linear fitting
vector<float> fittingGaussNewton(Camera virtualCam, Mat Jdijs, Mat dijs);

// Dimensions of inputs, are not equal
template<typename T>
void errorSize(string input1, string input2, T size1, T size2, string filename, int line);

// Export fitting data
void exportFittingData(Mat m, Mat x);

// Detect edges using Canny edge detector
Mat detectEdges(Mat img, double threshold, double ratio, int kernel);

// Invert binary images
Mat invertBinaryImage(Mat binaryImg);
			  
// Play source video and track model
void playVideo(string videoFilename, Cuboid3D model, vector <float> params, float fov, int imageWidth, int imageHeight, int maxIterations, float threshold, float ratio, int kernel);
