//visual-tracking package
#include "../headers/visualTracking.h"

// Image plane dimensions
#define _WIDTH 400
#define _HEIGHT 300

int main(int argc, char** argv)
{
	// Initialization
	float length = 0.f, height = 0.f, width = 0.f;
	// Read from .x file
	modelData(length, height, width);
	// Model initialization
	Cuboid3D model(length, height, width);
	// Data initialization
	Cuboid3D Data(length, height, width);

	Mat src = imread("data/cuboid/cube3.jpg");
	
	visualTracker(model, Data, src, _WIDTH, _HEIGHT);
	
	exit(EXIT_SUCCESS);
}