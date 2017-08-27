#include "../headers/rotations3D.h"

// Euler angles
// Return x-axis rotation matrix
Mat rotationX(float thetaX)
{
	return (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
}

// Return y-axis rotation matrix
Mat rotationY(float thetaY)
{
	return (Mat_<float>(3, 3) << cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
}

// Return z-axis rotation matrix
Mat rotationZ(float thetaZ)
{
	return (Mat_<float>(3, 3) << cos(thetaZ), -sin(thetaZ), 0, sin(thetaZ), cos(thetaZ), 0, 0, 0, 1);
}

// Return rotation matrix according to Euler angles
Mat rotationEuler(float thetaX, float thetaY, float thetaZ)
{
	return  rotationY(thetaY) * rotationZ(thetaZ) * rotationX(thetaX);
}

// Return Euler angles from rotation matrix
Vec3f matrix2euler(Mat R)
{
	if (!IsRotationMatrix(R))
	{
		cout << "Input matrix R is not a 3D rotation matrix." << endl;
		return Vec3f();
	}

	Vec3f eulerAngles;
	if (abs(R.at<float>(1, 0)) != 1.f)
	{
		eulerAngles.val[0] = atan2f(-R.at<float>(1, 2), R.at<float>(1, 1));
		eulerAngles.val[1] = atan2f(-R.at<float>(2, 0), R.at<float>(0, 0));
		eulerAngles.val[2] = asinf(R.at<float>(1, 0));
	}
	else
	{
		eulerAngles.val[0] = 0.f;
		eulerAngles.val[1] = atan2f(R.at<float>(0, 2), R.at<float>(2, 2));
		eulerAngles.val[2] = asinf(R.at<float>(1, 0));
	}

	return eulerAngles;
}

// Degrees to radians
float deg2rad(float degrees)
{
	return degrees * PI / 180.f;
}

// Degrees to radians - overload
Vec3f deg2rad(Vec3f degrees)
{
	return degrees * PI / 180.f;
}

// Radians to degrees
float rad2deg(float radians)
{
	return radians * 180.f / PI;
}

// Radians to degrees - overload
Vec3f rad2deg(Vec3f radians)
{
	return radians * 180.f / PI;
}

// Check if input is 3D rotation matrix
bool IsRotationMatrix(Mat R)
{
	if ((R.rows != 3) || (R.cols != 3))
	{
		return 0;
	}

	Mat Iprog = R * R.t();
	Mat I = Mat::eye(3, 3, Iprog.type());

	return norm(I, Iprog, NORM_L2) < 1e-6;
}