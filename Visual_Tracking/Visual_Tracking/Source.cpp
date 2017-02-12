#include "opencv2/imgcodecs.hpp"
#include <iostream>

//visual-tracking package
#include "headers/visual-tracking.h"

// Image plane dimensions
#define WIDTH 400
#define HEIGHT 300

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	// Initialization
	float length = 0.f, height = 0.f, width = 0.f;
	// Read from .x file
	modelData(argc, argv, length, height, width);
	// Image plane 300x400 pixels
	Mat imagePlane(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));
	Mat imagePlaneModel(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));
	Mat imagePlaneData(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));

	// Model initialization
	Cuboid model(length, height, width);
	// Virtual camera initialization
	// Camera Extrinsics
	Vec3f t(0.f, 0.f, 80.f); // Camera position
	Vec3f r(180.f, 0.f, 0.f); // Rotations over x, y, z axes
	// Camera Intrinsics
	float u0 = WIDTH / 2, v0 = HEIGHT / 2; // Principal point - center of image plane
	float fov = 60.f; // F.O.V
	float focal = (WIDTH / 2) / ((float)(tan((fov / 2) * PI / 180.0))); // Focal length
	Camera virtualCam(t, r, Point2f(u0, v0), fov, focal);
	// Render model
	rendering(model, virtualCam, imagePlane, imagePlaneModel, "model");

	// Data initialization
	Cuboid data(length, height, width);
	// Real camera initialization
	t = Vec3f(0.25f, 0.25f, 80.f);
	Camera realCam(t, r, Point2f(u0, v0), fov, focal);
	// Render data
	rendering(data, realCam, imagePlane, imagePlaneData,"data");

	// Disimillarity between data and model object
	float D = dissimilarity(model, imagePlane, imagePlaneData);
	cout << "Dissimilarity between model and data object: " << D << endl;

	waitKey(0);
	exit(EXIT_SUCCESS);
}