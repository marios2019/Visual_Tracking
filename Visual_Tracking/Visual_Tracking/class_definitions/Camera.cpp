#include "..\headers\Camera.h"

Camera::Camera() : CameraStateSpace() // Empty constructor
{
	// Intrinsics initialization
	principalPoint = Point2f();
	fov = 0.f;
	focal = 0.f;
}

Camera::Camera(Vec3f tVal, Vec3f rVal, Point2f principalVal, float fovVal, float focalVal, size_t paramsNumVal) // Constructor
	: CameraStateSpace(tVal.val[0], tVal.val[1], tVal.val[2], rVal[0], rVal[1], rVal[2], (int) paramsNumVal)
{
	// Intrinsics initialization
	principalPoint = principalVal;
	fov = fovVal;
	focal = focalVal;
}

Camera::~Camera()
{
}

// Change principal point
void Camera::setPrincipal(Point2f principalVal)
{
	principalPoint = principalVal;
}

// Return principal point
Point2f Camera::getPrincipal() const
{
	return principalPoint;
}

// Change fov
void Camera::setFov(float fovVal)
{
	fov = fovVal;
}

// Return fov
float Camera::getFov() const
{
	return fov;
}

// Change focal length
void Camera::setFocal(float focalVal)
{
	focal = focalVal;
}

// Return focal length
float Camera::getFocal() const
{
	return focal;
}

// Return x-axis rotation matrix
Mat Camera::getRotationX() const
{
	return (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(getThetaX(RADIANS)), -sin(getThetaX(RADIANS)), 0, sin(getThetaX(RADIANS)), cos(getThetaX(RADIANS)));
}

// Return y-axis rotation matrix
Mat Camera::getRotationY() const
{
	return (Mat_<float>(3, 3) << cos(getThetaY(RADIANS)), 0, sin(getThetaY(RADIANS)), 0, 1, 0, -sin(getThetaY(RADIANS)), 0, cos(getThetaY(RADIANS)));
}

// Return z-axis rotation matrix
Mat Camera::getRotationZ() const
{
	return (Mat_<float>(3, 3) << cos(getThetaZ(RADIANS)), -sin(getThetaZ(RADIANS)), 0, sin(getThetaZ(RADIANS)), cos(getThetaZ(RADIANS)), 0, 0, 0, 1);
}

// Return x, y, z axes rotation matrix
Mat Camera::getRotation() const
{
	return getRotationX() * getRotationY() * getRotationZ();
}

// Return camera intrinsics matrix
Mat Camera::getIntrinsics() const
{
	return (Mat_<float>(3, 3) << focal, 0, principalPoint.x, 0, focal, principalPoint.y, 0, 0, 1);
}

Mat Camera::getExtrinsics() const
{
	// Trasnpose Rotation Matrix
	Mat Rt;
	transpose(getRotation(), Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position
	Mat tmp = -Rt * Mat(getPosition());
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);
	
	return E;
}

// Return the first derivative of the extrinsics matrix, with respect of the i-th extrinsic parameter.
Mat Camera::getExtrinsicsDerivative(int idx)
{
	if ((idx > 5) || (idx < 0))
	{
		cout << "Invalid extrinsic camera parameter." << endl;
		exit(EXIT_FAILURE);
	}

	// Inverse Rotation Matrix derivative
	Mat dR, Rt, dRt;
	transpose(getRotationDerivative(idx), dRt);
	transpose(getRotation(), Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position derivative
	Mat tmp = - (dRt * Mat(getPosition(), false)) - (Rt * Mat(getPositionDerivative(idx)));
	Vec3f dRtT = Vec3f(tmp);
	// Camera extrinsics derivative
	Mat dE = (Mat_<float>(3, 4) << dRt.at<float>(0, 0), dRt.at<float>(0, 1), dRt.at<float>(0, 2), dRtT[0], dRt.at<float>(1, 0), dRt.at<float>(1, 1), dRt.at<float>(1, 2), dRtT[1], dRt.at<float>(2, 0), dRt.at<float>(2, 1), dRt.at<float>(2, 2), dRtT[2]);
	
	return dE;
}

// Return the first derivative of the position vector t
Vec3f Camera::getPositionDerivative(int idx)
{
	switch (idx)
	{
		case 0: // x = tx
			return Vec3f(1, 0, 0);

		case 1: // x = ty
			return Vec3f(0, 1, 0);

		case 2: // x = tz
			return Vec3f(0, 0, 1);

		case 3: // x = thetax
			return Vec3f(0, 0, 0);

		case 4: // x = thetay
			return Vec3f(0, 0, 0);

		case 5: // x = thetaz
			return Vec3f(0, 0, 0);

		default:
			return Vec3f();
	}
}

// Return the first derivative of the rotation matrix R
Mat Camera::getRotationDerivative(int idx)
{
	switch (idx)
	{
		case 0: // x = tx
			return (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);

		case 1: // x = ty
			return (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);

		case 2: // x = tz
			return (Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);

		case 3: // x = thetax
		{
			Mat dRx = (Mat_<float>(3, 3) << 0, 0, 0, 0, -sin(getThetaX(RADIANS)), -cos(getThetaX(RADIANS)), 0, cos(getThetaX(RADIANS)), -sin(getThetaX(RADIANS)));
			return (dRx * getRotationY() * getRotationZ());
		}
		case 4: // x = thetay
		{
			Mat dRy = (Mat_<float>(3, 3) << -sin(getThetaY(RADIANS)), 0, cos(getThetaY(RADIANS)), 0, 0, 0, -cos(getThetaY(RADIANS)), 0, -sin(getThetaY(RADIANS)));
			return (getRotationX() * dRy * getRotationZ());
		}
		case 5: // x = thetaz
		{
			Mat dRz = (Mat_<float>(3, 3) << -sin(getThetaZ(RADIANS)), -cos(getThetaZ(RADIANS)), 0, cos(getThetaZ(RADIANS)), -sin(getThetaZ(RADIANS)), 0, 0, 0, 0);
			return (getRotationX() * getRotationY() * dRz);
		}

		default:
			return Mat();
	}
}