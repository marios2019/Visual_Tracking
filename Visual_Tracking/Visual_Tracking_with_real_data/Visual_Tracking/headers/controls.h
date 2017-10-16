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

#include "cameraCalibration.h"
#include "projectiveGeometry.h"

// Dispaly camera parameters
void dispCamParams(Camera virtualCam);

void dispCamParamsVideo(Camera virtualCam);

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Cuboid3D &model, int &mNum, vector <float> defaultParams, bool &exitFlag, bool &updateFlag, bool &fitFlag, 
				     bool &readFilesFlag, bool &updateFilenamesFlag, bool &videoFlag);