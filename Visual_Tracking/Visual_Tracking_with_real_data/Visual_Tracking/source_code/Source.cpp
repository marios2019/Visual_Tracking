//visual-tracking package
#include "../headers/visualTracking.h"

int main(int argc, char** argv)
{
	// Initialization
	float length = 0.f, height = 0.f, width = 0.f;
	// Read from .x file
	modelData("cuboid3.x", length, height, width);
	// Model initialization
	Cuboid3D model(length, height, width);

	// Begin tracking of model	
	visualTracker(model);
	
	exit(EXIT_SUCCESS);
}