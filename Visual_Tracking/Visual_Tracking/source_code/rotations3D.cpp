#include "../headers/rotations3D.h"
#include "../headers/mathLinearAlgebra.h"

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

// Convert euler angles to axis angle
// Input angles are in degrees
// Output axis is multiplied by the rotation angle, expressed in degrees
Vec3f euler2AxisAngle(float thetaX, float thetaY, float thetaZ)
{
	// Initialization
	float cos1 = cos(deg2rad(thetaY / 2.f)), cos2 = cos(deg2rad(thetaZ / 2.f)), cos3 = cos(deg2rad(thetaX / 2.f));
	float sin1 = sin(deg2rad(thetaY / 2.f)), sin2 = sin(deg2rad(thetaZ / 2.f)), sin3 = sin(deg2rad(thetaX / 2.f));

	// Rotation angle
	float angle = rad2deg(2.f * acos(cos1 * cos2 * cos3 - sin1 * sin2 * sin3));

	// Rotation axis
	Vec3f axis;
	axis.val[0] = sin1 * sin2 * cos3 + cos1 * cos2 * sin3;
	axis.val[1] = sin1 * cos2 * cos3 + cos1 * sin2 * sin3;
	axis.val[2] = cos1 * sin2 * cos3 - sin1 * cos2 * sin3;

	float norm = norm2(axis);
	// All euler angles are zeros - define an arbitary axis
	if (norm <= 0.0001f)
	{
		axis.val[0] = 1.f;
		axis.val[1] = 0.f;
		axis.val[2] = 0.f;
		return axis;
 	}
	// Normalise axis
	axis /= norm;
	
	return angle * axis;
}

// Axis angle to euler angle
// Input axis is multiplied by the rotation angle, expressed in degrees
Mat axisAngle2euler(Vec3f axis)
{
	float angle = norm2(axis);
	// Rotation angle is zero
	if (angle <= 0.0001f) 
	{
		return Mat::zeros(3, 1, CV_32F);
	}
	// Axis normalisation
	axis /= angle;

	// Get euler angles in the order tY, tZ, tX
	float x = axis.val[0], y = axis.val[1], z = axis.val[2];
	float sin1 = sin(deg2rad(angle)), cos1 = cos(deg2rad(angle)), t = 1.f - cos1;
	float thetaX, thetaY, thetaZ;
	// Singularites
	if ((x * y * t + z * sin1) > 0.998f) // Axis straight up
	{
		thetaY = 2.f * atan2f(x * sin(deg2rad(angle / 2.f)), cos(deg2rad(angle / 2.f)));
		thetaZ = PI / 2.f;
		thetaX = 0.f;
		return (Mat_<float>(3, 1) << rad2deg(thetaX), rad2deg(thetaY), rad2deg(thetaZ));
	}
	else if ((x * y * t + z * sin1) < -0.998f) // Axis straight down
	{
		thetaY = -2.f * atan2f(x * sin(deg2rad(angle / 2.f)), cos(deg2rad(angle / 2.f)));
		thetaZ = -PI / 2.f;
		thetaX = 0.f;
		return (Mat_<float>(3, 1) << rad2deg(thetaX), rad2deg(thetaY), rad2deg(thetaZ));
	}
	
	// Convert axis angle to euler angles
	thetaY = atan2f(y * sin1 - x * z * t, 1 - (y * y + z * z) * t);
	thetaZ = asin(x * y * t + z * sin1);
	thetaX = atan2f(x * sin1 - y * z * t, 1 - (x * x + z * z) * t);
	
	return (Mat_<float>(3, 1) << rad2deg(thetaX), rad2deg(thetaY), rad2deg(thetaZ));
}

// Calculate rotation matrix from axis r which is multiplied
// by angle expressed in degrees, using Rodrigues's formula
Mat axisAngle2Matrix(Vec3f axis)
{
	// Extract angle
	float angle = norm2(axis);
	// Normalise axis
	axis /= angle;

	// Calculate rotation matrix using Rodrigues' formula
	Mat Axisx = skewMat(axis);
	Mat R = Mat::eye(3, 3, CV_32F) + Axisx * sin(deg2rad(angle)) + Axisx * Axisx * (1 - cos(deg2rad(angle)));

	if (IsRotationMatrix(R))
	{
		return R;
	}
	else
	{
		return Mat::eye(3, 3, CV_32F);
	}
}