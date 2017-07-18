#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#include "CameraStateSpace.h"
#include "Rotations3D.h"
#include "utility.h"

#define PI 3.14159265f

using namespace cv;
using namespace std;

class Camera: public CameraStateSpace
{
public:
	Camera(); // Empty constructor
	Camera(Vec3f, Vec3f, Point2f, float = 0.f, float = 0.f, size_t = 0); // Constructor
	~Camera(); // Destructor

	void setPrincipal(Point2f); // Change principal point
	Point2f getPrincipal() const; // Return principal point

	void setFov(float); // Change fov
	float getFov() const; // Return fov

	void setFocal(float); // Change focal length
	float getFocal() const; // Return focal length

	void setPosition(Vec3f); // Change camera position
	Vec3f getPosition() const; // Return camera position
	
	void setRotation(Mat); // Set rotation matrix
	Mat getRotation() const; // Return rotation matrix
	void setThetaX(float); // Set thetaX value
	void setThetaY(float); // Set thetaY value
	void setThetaZ(float); // Set thetaZ value

	void setExtrinsics(); // Set camera extrinsics matrix
	void setExtrinsics(Mat); // Set camera extrinsics matrix
	Mat getIntrinsics() const; // Return camera intrinsics matrix
	Mat getExtrinsics(); // Return camera extrinsics matrix
	
	Mat getProjectionDerivative(int) const; // Return projection first derivative with respect of the i-th
											// se(3) Lie algebra generator.

	Mat getGenerator(int) const; // Get i-th se(3) Lie algebra generator

	void setParams(vector <float>, vector <State>); // Set camera parameters
	
private:
	// Camera Intrinsics
	Point2f principalPoint; // Principal point - center of image plane
	float fov; // F.O.V
	float focal; // Focal length
	// Camera Extrinsics
	Mat R; // Camera rotation matrix
	Mat E; // Camera extrinsics matrix
};

