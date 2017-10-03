#include "../headers/rotations3D.h"
#include "../headers/mathLinearAlgebra.h"

// Return Euler angles from rotation matrix, expressed in radians
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

	return norm(I, Iprog, NORM_L2) < 0.0001f;
}

// Convert euler angles to axis angle
// Input angles are in radians
// Output axis is multiplied by the rotation angle, expressed in radians
Vec3f euler2AxisAngle(float thetaX, float thetaY, float thetaZ)
{
	// Initialization
	float cos1 = cos(thetaY / 2.f), cos2 = cos(thetaZ / 2.f), cos3 = cos(thetaX / 2.f);
	float sin1 = sin(thetaY / 2.f), sin2 = sin(thetaZ / 2.f), sin3 = sin(thetaX / 2.f);

	// Rotation angle
	float angle = 2.f * acos(cos1 * cos2 * cos3 - sin1 * sin2 * sin3);

	// Rotation axis
	Vec3f axis;
	axis.val[0] = sin1 * sin2 * cos3 + cos1 * cos2 * sin3;
	axis.val[1] = sin1 * cos2 * cos3 + cos1 * sin2 * sin3;
	axis.val[2] = cos1 * sin2 * cos3 - sin1 * cos2 * sin3;

	float norm = norm2(axis);
	// All euler angles are zeros - rotation angle is zero
	if (norm <= 0.0001f)
	{
		return Vec3f(0.f, 0.f, 0.f);
 	}
	// Normalise axis
	axis /= norm;
	return angle * Vec3f(axis.val[0], axis.val[1], axis.val[2]);
}

// Axis angle to euler angle
// Input axis is multiplied by the rotation angle, expressed in radians
// Output euler angles are expressed in radians
Vec3f axisAngle2euler(Vec3f axis)
{
	Vec4f axisAngle = axisAngleConversion(axis);
	float angle = axisAngle.val[3];
	// Rotation angle is zero
	if (angle <= 0.0001f) 
	{
		return Vec3f();
	}
	
	// Get euler angles in the order tY, tZ, tX
	float x = axisAngle.val[0], y = axisAngle.val[1], z = axisAngle.val[2];
	float sin1 = sin(angle), cos1 = cos(angle), t = 1.f - cos1;
	float thetaX, thetaY, thetaZ;
	// Singularites
	if ((x * y * t + z * sin1) > 0.998f) // Axis straight up
	{
		thetaY = 2.f * atan2f(x * sin(angle / 2.f), cos(angle / 2.f));
		thetaZ = PI / 2.f;
		thetaX = 0.f;
		return Vec3f(thetaX, thetaY, thetaZ);
	}
	else if ((x * y * t + z * sin1) < -0.998f) // Axis straight down
	{
		thetaY = -2.f * atan2f(x * sin(angle / 2.f), cos(angle / 2.f));
		thetaZ = -PI / 2.f;
		thetaX = 0.f;
		return Vec3f(thetaX, thetaY, thetaZ);
	}
	
	// Convert axis angle to euler angles
	thetaY = atan2f(y * sin1 - x * z * t, 1 - (y * y + z * z) * t);
	thetaZ = asin(x * y * t + z * sin1);
	thetaX = atan2f(x * sin1 - y * z * t, 1 - (x * x + z * z) * t);
	
	return Vec3f(thetaX, thetaY, thetaZ);;
}

// Calculate rotation matrix from axis r which is multiplied
// by angle expressed in radians, using Rodrigues's formula
Mat axisAngle2Matrix(Vec3f axis)
{
	Vec4f axisAngle = axisAngleConversion(axis);
	// Extract angle
	float angle = axisAngle.val[3];
	
	// Calculate rotation matrix using Rodrigues' formula
	Mat Axisx = skewMat(Vec3f(axisAngle.val[0], axisAngle.val[1], axisAngle.val[2]));
	Mat R = Mat::eye(3, 3, CV_32F) + Axisx * sin(angle) + Axisx * Axisx * (1 - cos(angle));

	if (IsRotationMatrix(R))
	{
		return R;
	}
	else
	{
		cout << "Couldn't calculate a 3D rotation matrix." << endl;
		return Mat::eye(3, 3, CV_32F);
	}
}


// Convert rotation matrix to axis angle representation,
// where the output axis is multiplied by the rotation
// angle, expressed in radian

Vec3f matrix2AxisAngle(Mat R)
{
	float x, y, z, angle;
	float epsilon = 0.01f; // margin to allow for rounding errors
	
	// Check if R is a rotation matrix
	if (!IsRotationMatrix(R))
	{
		cout << "Input matrix R is not a rotation matrix." << endl;
		return Vec3f();
	}

	const float *row0 = R.ptr<float>(0);
	const float *row1 = R.ptr<float>(1);
	const float *row2 = R.ptr<float>(2);
	if ((abs(row0[1] - row1[0]) < epsilon) && (abs(row0[2] - row2[0]) < epsilon) && (abs(row1[2] - row2[1]) < epsilon))
	{// Singularity is found
		cout << "Singularity found." << endl;
		// Check if R is an identity matrix
		if (norm(R, Mat::eye(3, 3, CV_32F), NORM_L2) < 0.01f)
		{// Angle = 0
			cout << "R is the identity matrix, so rotation axis and angle are zero (0)." << endl;
			return Vec3f();
		}

		// Angle = 180
		angle = PI;
		float xx = (row0[0] + 1.f) / 2.f;
		float yy = (row1[1] + 1.f) / 2.f;
		float zz = (row2[2] + 1.f) / 2.f;
		float xy = (row0[1] + row1[0]) / 4.f;
		float xz = (row0[2] + row2[0]) / 4.f;
		float yz = (row1[2] + row2[1]) / 4.f;
		if ((xx > yy) && (xx > zz)) 
		{ // m[0][0] is the largest diagonal term
			if (xx< epsilon) 
			{
				x = 0.f;
				y = 0.7071f;
				z = 0.7071f;
			}
			else
			{
				x = sqrt(xx);
				y = xy / x;
				z = xz / x;
			}
		}
		else if (yy > zz) 
		{ // m[1][1] is the largest diagonal term
			if (yy< epsilon) 
			{
				x = 0.7071f;
				y = 0.f;
				z = 0.7071f;
			}
			else 
			{
				y = sqrt(yy);
				x = xy / y;
				z = yz / y;
			}
		}
		else 
		{ // m[2][2] is the largest diagonal term so base result on this
			if (zz< epsilon) 
			{
				x = 0.7071f;
				y = 0.7071f;
				z = 0.f;
			}
			else 
			{
				z = sqrt(zz);
				x = xz / z;
				y = yz / z;
			}
		}
		return angle * Vec3f(x, y, z); // return 180 deg rotation
	}

	// Convert rotation matrix to axis angle representation
	float s = sqrt((row2[1] - row1[2])*(row2[1] - row1[2]) + (row0[2] - row2[0])*(row0[2] - row2[0]) + (row1[0] - row0[1])*(row1[0] - row0[1])); // used to normalise
	if (abs(s) < 0.001f)
	{// prevent divide by zero, should not happen if matrix is orthogonal and should be
	 // caught by singularity test above, but I've left it in just in case
		s = 1.f;
	}
	angle = acos((row0[0] + row1[1] + row2[2] - 1.f) / 2.f);
	x = (row2[1] - row1[2]) / s;
	y = (row0[2] - row2[0]) / s;
	z = (row1[0] - row0[1]) / s;
	
	return angle * Vec3f(x, y, z);
}

// Vec4f axis angle to Vec3f axis angle, 
// where the output axis angle is multiplied by the angle in radians
Vec3f axisAngleConversion(Vec4f axisAngle)
{
	return axisAngle.val[3] * Vec3f(axisAngle.val[0], axisAngle.val[1], axisAngle.val[2]);
}

// Vec3f axis angle to Vec4f axis angle, 
// where the output axis angle is divided by the angle in radians
Vec4f axisAngleConversion(Vec3f axisAngle)
{
	// Extract angle
	float angle = norm2(axisAngle);
	// Normalise axis
	axisAngle /= angle;

	return Vec4f(axisAngle.val[0], axisAngle.val[1], axisAngle.val[2], angle);
}
