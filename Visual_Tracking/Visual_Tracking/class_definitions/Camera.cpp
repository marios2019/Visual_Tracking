#include "..\headers\Camera.h"

Camera::Camera() // Empty constructor
{
	// Extrinsics initialization
	t = Vec3f();
	thetaX = 0.f;
	thetaY = 0.f;
	thetaZ = 0.f;

	// Intrinsics initialization
	principalPoint = Point2f();
	fov = 0.f;
	focal = 0.f;
}

Camera::Camera(Vec3f tVal, Vec3f rVal, Point2f principalVal, float fovVal, float focalVal) // Constructor
{
	// Extrinsics initialization
	t = tVal;
	thetaX = rVal.val[0] * PI / 180.f;
	thetaY = rVal.val[1] * PI / 180.f;
	thetaZ = rVal.val[2] * PI / 180.f;

	// Intrinsics initialization
	principalPoint = principalVal;
	fov = fovVal;
	focal = focalVal;
}

Camera::~Camera()
{
}

// Change camera position
void Camera::setPosition(Vec3f tVal)
{
	t = tVal;
}

// Return camera position
Vec3f Camera::getPosition() const
{
	return t;
}

// Change x-axis rotation
void Camera::setThetaX(float thetaXVal)
{
	thetaX = thetaXVal * PI / 180.f;
}

// Change y-axis rotation
void Camera::setThetaY(float thetaYVal)
{
	thetaY = thetaYVal * PI / 180.f;
}

// Change z-axis rotation
void Camera::setThetaZ(float thetaZVal)
{
	thetaZ = thetaZVal * PI / 180.f;
}

// Return x-axis rotation
float Camera::getThetaX() const
{
	return thetaX * 180.f / PI;
}

// Return y-axis rotation
float Camera::getThetaY() const
{
	return thetaY * 180.f / PI;
}

// Return z-axis rotation
float Camera::getThetaZ() const
{
	return thetaZ * 180.f / PI;
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
	return (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
}

// Return y-axis rotation matrix
Mat Camera::getRotationY() const
{
	return (Mat_<float>(3, 3) << cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
}

// Return z-axis rotation matrix
Mat Camera::getRotationZ() const
{
	return (Mat_<float>(3, 3) << cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
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
	Mat tmp = -Rt * Mat(t);
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);
	
	return E;
}

// Return the first derivative of the extrinsics matrix, with respect of the i-th extrinsic parameter.
Mat Camera::getExtrinDerivative(int idx)
{
	if (idx > 5)
	{
		cout << "Invalid extrinsic camera parameter." << endl;
		exit(EXIT_FAILURE);
	}

	Mat dE = (Mat_<float>(3, 4));



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
			Mat dRx = (Mat_<float>(3, 3) << 0, 0, 0, 0, -sin(thetaX), -cos(thetaX), 0, cos(thetaX), -sin(thetaX));
			return (dRx * getRotationY() * getRotationZ());
		}
		case 4: // x = thetay
		{
			Mat dRy = (Mat_<float>(3, 3) << -sin(thetaY), 0, cos(thetaY), 0, 0, 0, -cos(thetaY), 0, -sin(thetaY));
			return (getRotationX() * dRy * getRotationZ());
		}
		case 5: // x = thetaz
		{
			Mat dRz = (Mat_<float>(3, 3) << -sin(thetaZ), -cos(thetaZ), 0, cos(thetaZ), -sin(thetaZ), 0, 0, 0, 0);
			return (getRotationX() * getRotationY() * dRz);
		}
	}
}
