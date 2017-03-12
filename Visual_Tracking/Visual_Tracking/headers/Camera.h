#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define PI 3.14159265f

using namespace cv;
using namespace std;

class Camera
{
public:
	Camera(); // Empty constructor
	Camera(Vec3f, Vec3f, Point2f, float = 0.f, float = 0.f); // Constructor
	~Camera(); // Destructor

	void setPosition(Vec3f); // Change camera position
	Vec3f getPosition() const; // Return camera position

	void setThetaX(float); // Change x-axis rotation
	void setThetaY(float); // Change y-axis rotation
	void setThetaZ(float); // Change z-axis rotation
	float getThetaX() const; // Return x-axis rotation
	float getThetaY() const; // Return y-axis rotation
	float getThetaZ() const; // Return x-axis rotation

	void setPrincipal(Point2f); // Change principal point
	Point2f getPrincipal() const; // Return principal point

	void setFov(float); // Change fov
	float getFov() const; // Return fov

	void setFocal(float); // Change focal length
	float getFocal() const; // Return focal length

	Mat getRotationX() const; // Return x-axis rotation matrix
	Mat getRotationY() const; // Return y-axis rotation matrix
	Mat getRotationZ() const; // Return z-axis rotation matrix
	Mat getRotation() const; // Return x, y, z axes rotation matrix

	Mat getIntrinsics() const; // Return camera intrinsics matrix
	Mat getExtrinsics() const; // Return camera extrinsics matrix
	
	Mat getExtrinDerivative(int); // Return the first derivative of the extrinsics matrix
								  // with respect of the i-th extrinsic parameter.
private:
	// Camera Extrinsics
	Vec3f t; // Camera position
	float thetaX, thetaY, thetaZ; // Camera rotation over x, y, z axes
	// Camera Intrinsics
	Point2f principalPoint; // Principal point - center of image plane
	float fov; // F.O.V
	float focal; // Focal length
	Vec3f getPositionDerivative(int); // Return the first derivative of the position vector t
	Mat getRotationDerivative(int); // Return the first derivative of the rotation matrix R
};

