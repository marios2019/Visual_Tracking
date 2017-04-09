#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define PI 3.14159265f
#define VERTICES 8

using namespace cv;
using namespace std;

enum Type { DEGREES, RADIANS };
enum State {X, Y, Z, THETAX, THETAY, THETAZ};

class CameraStateSpace
{
public:
	CameraStateSpace(); // Empty Constructor
	CameraStateSpace(float, float, float, float, float, float, int); // Constructor
	~CameraStateSpace(); // Destructor

	void setTx(float); // Change x-axis position
	void setTy(float); // Change y-axis position
	void setTz(float); // Change z-axis position
	float getTx() const; // Return x-axis position
	float getTy() const; // Return y-axis position
	float getTz() const; // Return x-axis position
	void setPosition(Vec3f); // Change camera position
	Vec3f getPosition() const; // Return camera position

	void setThetaX(float); // Change x-axis rotation
	void setThetaY(float); // Change y-axis rotation
	void setThetaZ(float); // Change z-axis rotation
	float getThetaX(Type) const; // Return x-axis rotation
	float getThetaY(Type) const; // Return y-axis rotation
	float getThetaZ(Type) const; // Return x-axis rotation

	void setParams(vector <float>, vector <State>); // Set camera parameters
	vector <float> getParams(vector <State>); // Get camera parameters

	void setParamsNum(int); // Set the parameters number
	int getParamsNum() const; // Return the number of camera parameters

private:
	float tX, tY, tZ, thetaX, thetaY, thetaZ; // Camera state parameters
	int parametersNum; // Number of parameters
};

