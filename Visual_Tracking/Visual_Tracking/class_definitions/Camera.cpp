#include "..\headers\Camera.h"

Camera::Camera(Vec3f tVal, Vec3f rVal, Point2f principalVal, float fovVal, float focalPixelsVal, vector <State> stateVal) // Constructor
	: CameraStateSpace(tVal.val[0], tVal.val[1], tVal.val[2], rVal.val[0], rVal.val[1], rVal.val[2], stateVal)
{
	// Intrinsics initialization
	principalPoint = principalVal;
	fov = fmod(fmod(fovVal, DEG) + DEG, DEG);
	focalPixels = checkFocal(focalPixelsVal);
	focalMetric = focalPixelsVal / K;
	width = static_cast<int>(2.f * focalPixelsVal * tan(deg2rad(fovVal / 2.f)));
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
	fov = fmod(fmod(fovVal, DEG) + DEG, DEG);
	focalPixels = static_cast<float>(width) / (2.f * tan(deg2rad(fov / 2.f)));
	focalMetric = focalPixels / K;
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
	focalMetric = focalPixels / K;
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
	focalPixels = K * focalMetric;
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
		Vec3f eulerAngles(matrix2euler(RVal));
		setThetaX(eulerAngles.val[0], RADIANS);
		setThetaY(eulerAngles.val[1], RADIANS);
		setThetaZ(eulerAngles.val[2], RADIANS);
	}
	else
	{
		cout << "The input matrix is not a 3D rotation matrix." << endl;
	}
}

// Return rotation matrix
Mat Camera::getRotation() const
{
	return rotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS));
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
Mat Camera::getExtrinsics()
{
	// Trasnpose Rotation Matrix
	Mat Rt;
	transpose(getRotation(), Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position
	Mat tmp = -Rt * Mat(getPosition());
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	return (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);
}

// Return projection first derivative with respect of the i-th
// se(3) Lie algebra generator.
Mat Camera::getLieAlgebraDerivative(int idx)
{
	return getIntrinsics() * getExtrinsics() * getGenerator(idx);
}

// Get i-th se(3) Lie algebra generator
Mat Camera::getGenerator(int idx) const
{
	if ((idx > 5) || (idx < 0))
	{
		cout << "se(3) Lie algebra contains only six generators." << endl;
		return Mat();
	}

	Mat G = Mat::zeros(Size(4, 4), CV_32F);
	switch (idx)
	{
		case 0: // generator - derivative with respect of tx at identity transformation
		{
			G.at<float>(0, 3) = 1.f;
			break;
		}
		case 1: // generator - derivative with respect of ty at identity transformation
		{
			G.at<float>(1, 3) = 1.f;
			break;
		}
		case 2: // generator - derivative with respect of tz at identity transformation
		{
			G.at<float>(2, 3) = 1.f;
			break;
		}
		case 3: // generator - derivative with respect of thetax at identity transformation
		{
			G.at<float>(1, 2) = -1.f;
			G.at<float>(2, 1) = 1.f;
			break;
		}
		case 4: // generator - derivative with respect of thetay at identity transformation
		{
			G.at<float>(0, 2) = 1.f;
			G.at<float>(2, 0) = -1.f;
			break;
		}
		case 5: // generator - derivative with respect of thetaz at identity transformation
		{
			G.at<float>(0, 1) = -1.f;
			G.at<float>(1, 0) = 1.f;
			break;
		}
	}

	return G;
}

// Set camera parameters
void Camera::setParams(vector <float> paramsVal, vector <State> stateVal, Angle angle)
{
	if (stateVal.size() > 6)
	{
		cout << "Up to 6 state parameters are allowed." << endl;
		return;
	}

	if (stateVal.size() != paramsVal.size())
	{
		cout << "Parameters values and states STL vectors must be of the same size." << endl;
		return;
	}

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
		case THETAX:
			setThetaX(paramsVal[i], angle);
			break;
		case THETAY:
			setThetaY(paramsVal[i], angle);
			break;
		case THETAZ:
			setThetaZ(paramsVal[i], angle);
			break;
		default:
			break;
		}
	}
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
