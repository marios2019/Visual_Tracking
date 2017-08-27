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
void dispCamParams(Camera virtualCam, Camera realCam);

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, int &mNum, vector <float> defaultParams, bool &exitFlag, bool &updateFlag, bool &fitFlag);