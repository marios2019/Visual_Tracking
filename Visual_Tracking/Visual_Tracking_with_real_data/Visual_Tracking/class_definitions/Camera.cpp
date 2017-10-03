#include "..\headers\Camera.h"

Camera::Camera(Vec3f tVal, Vec3f eulerAnglesVal, Point2f principalVal, float fovVal, float focalPixelsVal, vector <State> stateVal, Rotation RtypeVal) // Constructor
	: CameraStateSpace(tVal.val[0], tVal.val[1], tVal.val[2], eulerAnglesVal.val[0], eulerAnglesVal.val[1], eulerAnglesVal.val[2], stateVal)
{
	// Intrinsics initialization
	principalPoint = principalVal;
	fov = fmod(fmod(fovVal, _DEG) + _DEG, _DEG);
	focalPixels = checkFocal(focalPixelsVal);
	focalMetric = focalPixelsVal / _Kp;
	width = static_cast<int>(2.f * focalPixelsVal * tan(deg2rad(fovVal / 2.f)));
	Rtype = RtypeVal;
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
	fov = fmod(fmod(fovVal, _DEG) + _DEG, _DEG);
	focalPixels = static_cast<float>(width) / (2.f * tan(deg2rad(fov / 2.f)));
	focalMetric = focalPixels / _Kp;
}

// Return fov
float Camera::getFov() const
{
	return fov;
}

// Change focalPixels
void Camera::setFocalPixels(float focalPixelsVal)
{
	focalPixels = checkFocal(focalPixelsVal);
	focalMetric = focalPixels / _Kp;
	fov = 2.f * rad2deg(atan2f(static_cast<float>(width), 2.f * focalPixels));
}

// Return focalPixels
float Camera::getFocalPixels() const
{
	return focalPixels;
}

// Change focalMetric
void Camera::setFocalMetric(float focalMetricVal)
{
	focalMetric = checkFocal(focalMetricVal);
	focalPixels = _Kp * focalMetric;
	fov = 2.f * rad2deg(atan2f(static_cast<float>(width), 2.f * focalPixels));
}

// Return focalMetric
float Camera::getFocalMetric() const
{
	return focalMetric;
}

// Change camera position
void Camera::setPosition(Vec3f tVal)
{
	setTx(tVal.val[0]);
	setTy(tVal.val[1]);
	setTz(tVal.val[2]);
}

// Return camera position
Vec3f Camera::getPosition() const
{
	return Vec3f(getTx(), getTy(), getTz());
}

// Set rotation matrix
void Camera::setRotation(Mat RVal)
{
	if (IsRotationMatrix(RVal))
	{
		setAxisAngle(matrix2AxisAngle(RVal));
	}
	else
	{
		cout << "The input matrix is not a 3D rotation matrix." << endl;
	}
}

// Return rotation matrix
Mat Camera::getRotation(Rotation type) const
{
	if (type == EULER) // Get rotation matrix from euler angles
	{
		return eulerAngles2Matrix(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS));
	}
	else // Get rotation matrix from axis angle
	{
		return axisAngle2Matrix(getAxisAngle());
	}
}

// Set camera extrinsics matrix - overload
void Camera::setExtrinsics(Mat EVal)
{
	Mat R;
	Vec3f t;
	decomposeCameraPose(EVal, R, t);
	
	if (IsRotationMatrix(R))
	{
		// Update camera position and orientation
		setRotation(R);
		setPosition(t);
	}
	else
	{
		cout << "The input matrix must contain a 3D rotation matrix at the upper left region." << endl;
		return;
	}
}

// Return camera intrinsics matrix
Mat Camera::getIntrinsics() const
{
	return (Mat_<float>(3, 3) << focalPixels, 0.f, principalPoint.x, 0.f, focalPixels, principalPoint.y, 0.f, 0.f, 1.f);
}

// Return camera extrinsics matrix
Mat Camera::getExtrinsics(Rotation type)
{
	// Trasnpose Rotation Matrix
	Mat Rt;
	transpose(getRotation(type), Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position
	Mat tmp = -Rt * Mat(getPosition());
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	return (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);
}

// Set camera parameters
void Camera::setParams(vector <float> paramsVal, vector <State> stateVal)
{
	// Check number of states
	if (stateVal.size() > 6)
	{
		cout << "Up to 6 state parameters are allowed." << endl;
		return;
	}

	// Check if the two inputs match
	if (stateVal.size() != paramsVal.size())
	{
		cout << "Parameters values and states STL vectors must be of the same size." << endl;
		return;
	}
	
	// Update camera state space
	float r1, r2, r3;
	bool r1Flag = false, r2Flag = false, r3Flag = false;
	for (int i = 0; i < stateVal.size(); i++)
	{
		switch (stateVal[i])
		{
		case X:
			setTx(paramsVal[i]);
			break;
		case Y:
			setTy(paramsVal[i]);
			break;
		case Z:
			setTz(paramsVal[i]);
			break;
		case R1:
		{
			r1 = paramsVal[i];
			r1Flag = true;
			break;
		}
		case R2:
		{
			r2 = paramsVal[i];
			r2Flag = true;
			break;
		}
		case R3:
		{
			r3 = paramsVal[i];
			r3Flag = true;
			break;
		}
		default:
			break;
		}
	}

	// Update r1, r2, r3
	if ((r1Flag == true) && (r2Flag == true) && (r3Flag == true))
	{
		setAxisAngle(Vec3f(r1, r2, r3));
	}
}

// Get 3D rotation type
Rotation Camera::getRotationType() const
{
	return Rtype;
}

// Check if focal length has non negative value
float Camera::checkFocal(float focalVal)
{
	if (focalVal < 0.f)
	{
		cout << "Focal length has to be a positive float; value has changed from negative to positive." << endl;
		return -focalVal;
	}

	return focalVal;
}
