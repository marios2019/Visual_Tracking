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
	
	// Set rotation matrix
	R = getRotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS));
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
		RVal.copyTo(R);
	}
	else
	{
		cout << "The input matrix is not a 3D rotation matrix." << endl;
	}
}

// Return rotation matrix
Mat Camera::getRotation() const
{
	return R;
}

// Set thetaX value
void Camera::setThetaX(float thetaXVal)
{
	CameraStateSpace::setThetaX(thetaXVal);
	// Update rotation matrix
	setRotation(getRotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS)));
}

// Set thetaY value
void Camera::setThetaY(float thetaYVal)
{
	CameraStateSpace::setThetaY(thetaYVal);
	// Update rotation matrix
	setRotation(getRotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS)));
}

// Set thetaZ value
void Camera::setThetaZ(float thetaZVal)
{
	CameraStateSpace::setThetaZ(thetaZVal);
	// Update rotation matrix
	setRotation(getRotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS)));
}

// Return camera intrinsics matrix
Mat Camera::getIntrinsics() const
{
	return (Mat_<float>(3, 3) << focal, 0, principalPoint.x, 0, focal, principalPoint.y, 0, 0, 1);
}

// Return camera extrinsics matrix
Mat Camera::getExtrinsics()
{
	setExtrinsics();

	return E;
}

// Set camera extrinsics matrix
void Camera::setExtrinsics()
{
	// Trasnpose Rotation Matrix
	Mat Rt;
	transpose(getRotation(), Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position
	Mat tmp = -Rt * Mat(getPosition());
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);

	E.copyTo(this->E);
}

// Set camera extrinsics matrix - overload
void Camera::setExtrinsics(Mat EVal)
{
	Mat R;
	Vec3f t;
	decomposeCameraPose(EVal, R, t);
	setTx(t.val[0]);
	setTy(t.val[1]);
	setTz(t.val[2]);

	if (IsRotationMatrix(R))
	{
		Vec3f eulerAngles(matrixToEuler(R));
		CameraStateSpace::setThetaX(eulerAngles.val[0]);
		CameraStateSpace::setThetaY(eulerAngles.val[1]);
		CameraStateSpace::setThetaZ(eulerAngles.val[2]);
		setRotation(getRotationEuler(getThetaX(RADIANS), getThetaY(RADIANS), getThetaZ(RADIANS)));
	}
	else
	{
		cout << "The input matrix must contain a 3D rotation matrix at the upper left region." << endl;
	}
}

// Return projection first derivative with respect of the i-th
// se(3) Lie algebra generator.
Mat Camera::getProjectionDerivative(int idx) const
{
	Mat dP;
	
	dP = getIntrinsics() * E * getGenerator(idx);
	return dP;
}

// Get i-th se(3) Lie algebra generator
Mat Camera::getGenerator(int idx) const
{
	if ((idx > 5) || (idx < 0))
	{
		cout << "se(3) Lie algebra contains only six generators." << endl;
		exit(EXIT_FAILURE);
	}

	Mat G = Mat::zeros(Size(4, 4), CV_32F);
	switch (idx)
	{
		case 0: // generator - derivative with respect of tx at identity transformation
		{
			G.at<float>(0, 3) = 1;
			break;
		}
		case 1: // generator - derivative with respect of ty at identity transformation
		{
			G.at<float>(1, 3) = 1;
			break;
		}
		case 2: // generator - derivative with respect of tz at identity transformation
		{
			G.at<float>(2, 3) = 1;
			break;
		}
		case 3: // generator - derivative with respect of thetax at identity transformation
		{
			G.at<float>(1, 1) = -1;
			G.at<float>(2, 0) = 1;
			break;
		}
		case 4: // generator - derivative with respect of thetay at identity transformation
		{
			G.at<float>(0, 2) = 1;
			G.at<float>(2, 0) = -1;
			break;
		}
		case 5: // generator - derivative with respect of thetaz at identity transformation
		{
			G.at<float>(0, 1) = -1;
			G.at<float>(1, 0) = 1;
			break;
		}
	}

	return G;
}

// Set camera parameters
void Camera::setParams(vector <float> paramsVal, vector <State> stateVal)
{
	if (stateVal.size() > 6)
	{
		cout << "Up to 6 state parameters are allowed!" << endl;
		exit(EXIT_FAILURE);
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
			setThetaX(paramsVal[i]);
			break;
		case THETAY:
			setThetaY(paramsVal[i]);
			break;
		case THETAZ:
			setThetaZ(paramsVal[i]);
			break;
		default:
			break;
		}
	}
}