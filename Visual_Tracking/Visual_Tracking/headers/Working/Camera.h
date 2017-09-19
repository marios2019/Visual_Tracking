#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#include "CameraStateSpace.h"

#define _PI 3.14159265f
#define _DEG 360.f
#define _Kp 500.f // _PIxels quantity per metric unit

using namespace cv;
using namespace std;

enum Rotation { EULER, AXISANGLE };

class Camera: public CameraStateSpace
{
public:
	Camera::Camera(Vec3f tVal = Vec3f(), Vec3f eulerAnglesVal = Vec3f(), Point2f principalVal = Point2f(), float fovVal = 0.f, 
					float focal_PIxelsVal = 0.f, vector <State> stateVal = { X, Y, Z, RX, RY, RZ }, Rotation RtypeVal = EULER); // Constructor
	~Camera(); // Destructor

	void setPrincipal(Point2f principalVal); // Change principal point
	Point2f getPrincipal() const; // Return principal point

	void setFov(float fovVal); // Change fov
	float getFov() const; // Return fov

	void setFocal_PIxels(float focal_PIxelsVal); // Change focal_PIxels
	float getFocal_PIxels() const; // Return focal_PIxels

	void setFocalMetric(float focalMetricVal); // Change focalMetric
	float getFocalMetric() const; // Return focalMetric

	void setPosition(Vec3f tVal); // Change camera position
	Vec3f getPosition() const; // Return camera position
	
	void setRotation(Mat RVal); // Set camera rotation
	Mat getRotation(Rotation type ) const; // Return rotation matrix
	
	void setExtrinsics(Mat EVal); // Set camera extrinsics
	Mat getIntrinsics() const; // Return camera intrinsics matrix
	Mat getExtrinsics(Rotation type); // Return camera extrinsics matrix
	
	Mat getLieAlgebraDerivative(int idx); // Return projection first derivative with respect of the i-th
											// se(3) Lie algebra generator.

	Mat getGenerator(int) const; // Get i-th se(3) Lie algebra generator

	void setParams(vector <float> paramsVal, vector <State> stateVal, Angle angle); // Set camera parameters
	
	Rotation getRotationType() const; // Get 3D rotation type
private:
	// Camera Intrinsics
	Point2f principalPoint; // Principal point - center of image plane
	float fov; // F.O.V
	float focal_PIxels; // Focal length in _PIxel coordinates
	float focalMetric; // Focal length in metric coordinates
	int width; // Image plane width
	Rotation Rtype; // 3D rotation represenation

	float checkFocal(float focalVal); // Check if focal length has non negative value
};

