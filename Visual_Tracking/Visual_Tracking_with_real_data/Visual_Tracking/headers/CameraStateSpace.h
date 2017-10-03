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
#define _DEG 360.f

using namespace cv;
using namespace std;

#include "mathLinearAlgebra.h"
#include "error.h"

enum Angle { DEGREES, RADIANS };
enum State {X, Y, Z, R1, R2, R3};

class CameraStateSpace
{
public:
	CameraStateSpace(float tXVal = 0.f, float tYVal = 0.f, float tZVal = 0.f, float thetaX = 0.f, float thetaY = 0.f, float thetaZ = 0.f, vector <State> stateVal = { X, Y, Z, R1, R2, R3 }); // Constructor
	~CameraStateSpace(); // Destructor

	void setTx(float tXVal); // Change x-axis position
	void setTy(float tYVal); // Change y-axis position
	void setTz(float tZVal); // Change z-axis position
	float getTx() const; // Return x-axis position
	float getTy() const; // Return y-axis position
	float getTz() const; // Return x-axis position

	void setAxisAngle(Vec3f axisAngle); // Set r1, r2, r3, input is an axis angle representation,
										// where the axis is multiplied by the rotation angle, expressed in radians
	Vec3f getAxisAngle() const; // Return r1, r2, r3 in axis angle representation,
								// where the axis is multiplied by the rotation angle, expressed in radians
	
	void setThetaX(float thetaX, Angle angle); // Change x-axis rotation
	void setThetaY(float thetaY, Angle angle); // Change y-axis rotation
	void setThetaZ(float thetaZ, Angle angle); // Change z-axis rotation
	float getThetaX(Angle angle) const; // Return x-axis rotation
	float getThetaY(Angle angle) const; // Return y-axis rotation
	float getThetaZ(Angle angle) const; // Return x-axis rotation

	vector <float> getParams(vector <State> stateVal); // Get specific camera parameters values
	vector <float> getParams(); // Get all camera parameters values

	void setState(vector <State> stateVal); // Enable state parameters
	State getState(int idx); // Get specific state parameter
	vector <State> getStates() const; // Get all state parameters
	int getStateSize() const; // Get state parameters size

private:
	float tX, tY, tZ, r1, r2, r3; // Camera state parameters
	vector <State> state; // State parameters that are enabled

	void checkStateSize(size_t size); // Check state parameteres size
};