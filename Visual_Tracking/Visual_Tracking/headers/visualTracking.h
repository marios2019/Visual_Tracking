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
#include <time.h>

using namespace cv;
using namespace std;

// Visual-tracking packages
#include "cameraCalibration.h"
#include "projectiveGeometry.h"
#include "mathLinearAlgebra.h"
#include "controls.h"

// Virtual Camera default extrinsics parameters
#define TX -28.f
#define TY 7.f
#define TZ 80.f
#define RX 160.f
#define RY -30.f
#define RZ 0.f

// Number of mijs
#define MNUM 3

// Visual tracker main function
void visualTracker(Cuboid3D &model, Cuboid3D &Data, int width, int height);

// Read model data from .x file
void modelData(float &length, float &height, float &width);

// Render function data
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, string type);

// Render function model
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, vector <Distance> &distances, string type);

// Dissimilarity between data and model object
void dissimilarity(Cuboid2D &model, Mat &imagePlane, Mat imagePlaneData, vector <Distance> &distances, DistClosedForm &distCF, int mNum);

// Calculate distance between model and data
void calcDist(vector <Distance> &distances, DistClosedForm &distCF, Vec2f normalOut, Vec2f vec, vector <Point2f> normalLine,
	Point2f pNormal, Point2f p_mi, float offsetOut, float offsetIn, Mat &imagePlane, Mat imagePlaneData, int idx, int j);

// Calculate new extrinsics matrix closer to data
void fitting(Camera& virtualCam, DistClosedForm& distCF, int edgesNum);

// Exponential map se(3) to SE(3)
Mat expMap(DistClosedForm &distCF);

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, string type);

// Check which parts of the object are visible from the camera's given viewpoint
void visibilityCulling(Cuboid3D &cuboid3D, Cuboid2D &cuboid2D, vector <Distance> &distances, Camera camera, Size size);

// Edge clipping
bool edgeClip(vector <Point2f> &edgePxl, Size size);

// Check if a surface is visible from the camera's viewpoint
int backFaceCulling(vector <int> surface, vector <Point3f> vertices, Point3f t);

// Fixed time frame update
Cuboid2D updateFrame(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, DistClosedForm &distCF, Mat &imagePlane, Mat &imagePlaneModel, Mat &imagePlaneData, vector <Distance> &distances);

// Create camera
Camera createCam(Vec3f t, Vec3f r, float fov, int width, int height, vector <State> state);

// Clear visibility of the 3D model
void resetVisibility(Cuboid3D &cuboid3D);

// Display image plane
void dispImagePlane(string windowName, Mat imagePlane);