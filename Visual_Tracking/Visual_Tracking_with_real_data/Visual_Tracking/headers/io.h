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

#include "error.h"

extern const string _VIDEO_DIR = "data\\cuboid\\";
extern const string _CONFIG_DIR = "config\\cuboid\\";
extern const string _MODEL_DIR = "models\\";

using namespace cv;
using namespace std;

// Read model data from .x file
void modelData(string modelFilename, float &length, float &height, float &width);

// Read from .txt file camera parameters
void configCameraData(string configFilename, float & tX, float & tY, float & tZ, float & RX, float & RY, float & RZ);

// Miscellaneous parameters	- image plane, fitting and Canny edge
void configParamsData(string configParamsFilename, int & imageWidth, int & imageHeight, float &fov, int &mNum, int &maxIterations, float &threshold, float & ratio, int & kernel);

// Update filenames
void readFilenames(string &srcImageFilename, string &videoFilename, string &configParamsFilename, string & modelFilename, string & configCameraFilename, Mat & srcData);
