#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// C++ libraries
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>

#define _PI 3.14159265f
#define _VERTICES 8
#define _DEG 360.f

using namespace cv;
using namespace std;

#include "mathLinearAlgebra.h"
#include "error.h"

enum Angle { DEGREES, RADIANS };
enum State {X, Y, Z, RX, RY, RZ};

class CameraStateSpace
{
public:
	CameraStateSpace(float tXVal = 0.f, float tYVal = 0.f, float tZVal = 0.f, float thetaXVal = 0.f, float thetaYVal = 0.f, float thetaZVal = 0.f,
					 vector <State> stateVal = { X, Y, Z, RX, RY, RZ }); // Constructor
	~CameraStateSpace(); // Destructor

	void setTx(float tXVal); // Change x-axis position
	void setTy(float tYVal); // Change y-axis position
	void setTz(float tZVal); // Change z-axis position
	float getTx() const; // Return x-axis position
	float getTy() const; // Return y-axis position
	float getTz() const; // Return x-axis position
	
	void setThetaX(float thetaXVal, Angle angle); // Change x-axis rotation
	void setThetaY(float thetaYVal, Angle angle); // Change y-axis rotation
	void setThetaZ(float thetaZVal, Angle angle); // Change z-axis rotation
	float getThetaX(Angle angle) const; // Return x-axis rotation
	float getThetaY(Angle angle) const; // Return y-axis rotation
	float getThetaZ(Angle angle) const; // Return x-axis rotation

	void setAxisAngle(Vec3f axisAngleVal, Angle type = DEGREES); // Set axis angle parameters
	Vec3f getAxisAngle() const; // Get axis angle vector, in radians

	vector <float> getParams(vector <State> stateVal); // Get specific camera parameters values
	vector <float> getParams(); // Get all camera parameters values

	void setState(vector <State> stateVal, Angle type); // Enable state parameters
	State getState(int idx); // Get specific state parameter
	vector <State> getStates() const; // Get all state parameters
	int getStateSize() const; // Get state parameters size

private:
	// Camera state parameters
	float tX, tY, tZ; // camera position
	float thetaX, thetaY, thetaZ; // camera orientation expressed in Euler angles, radians
	float r1, r2, r3; // camera orientation expressed in Axis angle representation, radians
	vector <State> state; // State parameters that are enabled

	void checkStateSize(size_t size); // Check state parameteres size
};