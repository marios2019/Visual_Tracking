#include "opencv2/imgcodecs.hpp"
#include <iostream>

//visual-tracking package
#include "headers/visualTracking.h"

using namespace cv;
using namespace std;

// Image plane dimensions
#define WIDTH 400
#define HEIGHT 300
// Initialization
float length = 0.f, height = 0.f, width = 0.f;
// Model initialization
Cuboid3D model(length, height, width);
// Data initialization
Cuboid3D Data(length, height, width);

// Image plane 300x400 pixels
Mat imagePlane(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));
Mat imagePlaneModel(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));
Mat imagePlaneData(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));

// Virtual camera initialization
// Camera Extrinsics
int txVirtual = 2; // tx = -28
int tyVirtual = 60; // ty = 30
int tzVirtual = 50; // tz = 80
int rxVirtual = 32; // rx = 160
int ryVirtual = 30; // ry = -30
int rzVirtual = 36; // rx = 0
Vec3f tVirtual(0.f, 0.f, 80.f); // Camera position
Vec3f rVirtual(180.f, 0.f, 0.f); // Rotations over x, y, z axes
// Camera Intrinsics
float u0Virtual = WIDTH / 2, v0Virtual = HEIGHT / 2; // Principal point - center of image plane
float fovVirtual = 60.f; // F.O.V
float focalVirtual = (WIDTH / 2) / ((float)(tan((fovVirtual / 2) * PI / 180.0))); // Focal length
Camera virtualCam(tVirtual, rVirtual, Point2f(u0Virtual, v0Virtual), fovVirtual, focalVirtual);
// Real camera initialization
// Camera Extrinsics
int txReal = 4; // tx = -26
int tyReal = 60; // ty = 30
int tzReal = 50; // tz = 80
int rxReal = 32; // rx = 160 
int ryReal = 30; // ry = -30
int rzReal = 36; // rz = 0
Vec3f tReal(0.25f, 0.25f, 80.f); // Camera position
Vec3f rReal(180.f, 0.f, 0.f); // Rotations over x, y, z axes
// Camera Intrinsics
float u0Real = WIDTH / 2, v0Real = HEIGHT / 2; // Principal point - center of image plane
float fovReal = 60.f; // F.O.V
float focalReal = (WIDTH / 2) / ((float)(tan((fovReal / 2) * PI / 180.0))); // Focal length
Camera realCam(tReal, rReal, Point2f(u0Real, v0Real), fovReal, focalReal);

const char* virtualCamParams = "Virtual Camera Parameters";
const char* realCamParams = "Real Camera Parameters";
const char* virtualCamActual = "Virtual Camera Actual Values";
const char* realCamActual = "Real Camera Actual Values";

// Calling functions from functionality.h for rendering 
// and dissimilarity, using track bar values as inputs.
static void visualTracker(int, void*)
{
	// Model - data objects dimensions
	model.setDimensions(Point3f(length, height, width));
	Data.setDimensions(Point3f(length, height, width));

	// Get translation and rotation values from track bars
	Mat virtualValues(70, 200, CV_8UC3, Scalar::all(255));
	Mat realValues(70, 200, CV_8UC3, Scalar::all(255));

	// Virtual Camaera Parameters
	float x = (float)txVirtual - 30.f;
	float y = (float)tyVirtual - 30.f;
	float z = (float)tzVirtual + 30.f;
	string values = "Tx: " + to_string((int)x) + " Ty: " + to_string((int)y) + " Tz: " + to_string((int)z);
	putText(virtualValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	values = "Rx: " + to_string(rxVirtual * 5) + " Ry: " + to_string(ryVirtual * 5 - 180) + " Rz: " + to_string(rzVirtual * 5 - 180);
	putText(virtualValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	virtualCam.setPosition(Vec3f(x, y, z));
	virtualCam.setThetaX((float)rxVirtual * 5.f);
	virtualCam.setThetaY(-(float)ryVirtual * 5.f - 180.f);
	virtualCam.setThetaZ(-(float)rzVirtual * 5.f - 180.f);

	// Real Camaera Parameters
	x = (float)txReal - 30.f;
	y = (float)tyReal - 30.f;
	z = (float)tzReal + 30.f;
	values = "Tx: " + to_string((int)x) + " Ty: " + to_string((int)y) + " Tz: " + to_string((int)z);
	putText(realValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	values = "Rx: " + to_string(rxReal * 5) + " Ry: " + to_string(ryReal * 5 - 180) + " Rz: " + to_string(rzReal * 5 - 180);
	putText(realValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	realCam.setPosition(Vec3f(x, y, z));
	realCam.setThetaX((float)rxReal * 5.f);
	realCam.setThetaY(-(float)ryReal * 5.f - 180.f);
	realCam.setThetaZ(-(float)rzReal * 5.f - 180.f);

	// Render model
	Cuboid2D modelProjection(rendering(model, virtualCam, imagePlane, imagePlaneModel, "model"));
	// Render data
	Cuboid2D dataProjection(rendering(Data, realCam, imagePlane, imagePlaneData, "data"));

	// Disimillarity between data and model object
	float D = dissimilarity(modelProjection, virtualCam, imagePlane, imagePlaneData);
	cout << "Dissimilarity between model and data object: " << D << endl;

	// Display track bars and actual values
	Mat trackBarVirtual(600, 400, CV_8UC3, Scalar::all(255));
	imshow(virtualCamParams, trackBarVirtual);
	Mat trackBarReal(300, 500, CV_8UC3, Scalar::all(255));
	imshow(realCamParams, trackBarReal);
	imshow(virtualCamActual, virtualValues);
	imshow(realCamActual, realValues);
}

int main(int argc, char** argv)
{
	// Read from .x file
	modelData(length, height, width);

	// Virtual Camera Parameters track bars
	namedWindow(virtualCamParams, WINDOW_NORMAL);
	createTrackbar("x-axis position:", virtualCamParams, &txVirtual, 60, visualTracker);
	createTrackbar("y-axis position:", virtualCamParams, &tyVirtual, 60, visualTracker);
	createTrackbar("z-axis position:", virtualCamParams, &tzVirtual, 100, visualTracker);
	createTrackbar("rot - x:", virtualCamParams, &rxVirtual, 72, visualTracker);
	createTrackbar("rot - y:", virtualCamParams, &ryVirtual, 72, visualTracker);
	createTrackbar("rot - z:", virtualCamParams, &rzVirtual, 72, visualTracker);

	// Real Camera Parameters track bars
	namedWindow(realCamParams, WINDOW_NORMAL);
	createTrackbar("x-axis position:", realCamParams, &txReal, 60, visualTracker);
	createTrackbar("y-axis position:", realCamParams, &tyReal, 60, visualTracker);
	createTrackbar("z-axis position:", realCamParams, &tzReal, 100, visualTracker);
	createTrackbar("rot - x:", realCamParams, &rxReal, 72, visualTracker);
	createTrackbar("rot - y:", realCamParams, &ryReal, 72, visualTracker);
	createTrackbar("rot - z:", realCamParams, &rzReal, 72, visualTracker);

	visualTracker(0, 0);

	waitKey(0);
	exit(EXIT_SUCCESS);
}