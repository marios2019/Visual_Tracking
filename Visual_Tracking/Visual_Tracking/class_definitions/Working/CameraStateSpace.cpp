#include "..\headers\CameraStateSpace.h"

// CameraStateSpace class definition

// Constructor
CameraStateSpace::CameraStateSpace(float tXVal, float tYVal, float tZVal, float thetaXVal, float thetaYVal, float thetaZVal, vector <State> stateVal)
{
	// Parameters initialization
	tX = tXVal;
	tY = tYVal;
	tZ = tZVal;
	thetaX = fmod(thetaXVal, _DEG) * _PI / 180.f;
	thetaY = fmod(thetaYVal, _DEG) * _PI / 180.f;
	thetaZ = fmod(thetaZVal, _DEG) * _PI / 180.f;
	// Axis angle
	Vec3f axisAngle = euler2AxisAngle(thetaX, thetaY, thetaZ);
	r1 = axisAngle.val[0];
	r2 = axisAngle.val[1];
	r3 = axisAngle.val[2];
	checkStateSize(stateVal.size());
	state = stateVal;
	sort(state.begin(), state.end());
}

// Destructor
CameraStateSpace::~CameraStateSpace()
{
}

// Change x-axis position
void CameraStateSpace::setTx(float tXVal)
{
	tX = tXVal;
}

// Change y-axis position
void CameraStateSpace::setTy(float tYVal)
{
	tY = tYVal;
}

// Change z-axis position
void CameraStateSpace::setTz(float tZVal)
{
	tZ = tZVal;
}

// Return x-axis position
float CameraStateSpace::getTx() const
{
	return tX;
}

// Return y-axis position
float CameraStateSpace::getTy() const
{
	return tY;
}

// Return z-axis position
float CameraStateSpace::getTz() const
{
	return tZ;
}

// Change x-axis rotation
void CameraStateSpace::setThetaX(float thetaXVal, Angle angle)
{
	if (angle == DEGREES)
	{// Degrees
		thetaX = deg2rad(fmod(thetaXVal, _DEG));
		
	}
	else
	{// Radians
		thetaX = fmod(thetaXVal, 2.f * _PI);
	}

	setAxisAngle(euler2AxisAngle(thetaX, thetaY, thetaZ, angle), angle);
}

// Change y-axis rotation
void CameraStateSpace::setThetaY(float thetaYVal, Angle angle)
{
	if (angle == DEGREES)
	{// Degrees
		thetaY = deg2rad(fmod(thetaYVal, _DEG));

	}
	else
	{// Radians
		thetaY = fmod(thetaYVal, 2.f * _PI);
	}

	setAxisAngle(euler2AxisAngle(thetaX, thetaY, thetaZ, angle), angle);
}

// Change z-axis rotation
void CameraStateSpace::setThetaZ(float thetaZVal, Angle angle)
{
	if (angle == DEGREES)
	{// Degrees
		thetaZ = deg2rad(fmod(thetaZVal, _DEG));

	}
	else
	{// Radians
		thetaZ = fmod(thetaZVal, 2.f * _PI);
	}

	setAxisAngle(euler2AxisAngle(thetaX, thetaY, thetaZ, angle), angle);
}

// Return x-axis rotation
float CameraStateSpace::getThetaX(Angle angle) const
{
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(thetaX);
	}

	// Radians
	return thetaX;
}

// Return y-axis rotation
float CameraStateSpace::getThetaY(Angle angle) const
{
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(thetaY);
	}
	
	// Radians
	return thetaY;
}

// Return z-axis rotation
float CameraStateSpace::getThetaZ(Angle angle) const
{
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(thetaZ);
	}

	// Radians
	return thetaZ;
}

// Set axis angle parameters
void CameraStateSpace::setAxisAngle(Vec3f axisAngleVal, Angle type)
{
	float angle = norm2(axisAngleVal);
	if (type == DEGREES)
	{
		angle = deg2rad(angle);
	}

	// Normalise angle between [0, 2Pi], if needed
	axisAngleVal /= angle;
	float angle = fmod(angle, 2 * _PI);
	axisAngleVal *= angle;

	r1 = axisAngleVal.val[0];
	r2 = axisAngleVal.val[1];
	r3 = axisAngleVal.val[0];

	// Update euler angles
	Mat eulerAngles = axisAngle2euler(axisAngleVal, type);
	setThetaX(eulerAngles.at<float>(0, 0), type);
	setThetaY(eulerAngles.at<float>(1, 0), type);
	setThetaZ(eulerAngles.at<float>(2, 0), type);
}

// Get axis angle vector, in radians
Vec3f CameraStateSpace::getAxisAngle() const
{
	return Vec3f(r1, r2, r3);
}


// Get specific camera parameters values
vector <float> CameraStateSpace::getParams(vector <State> stateVal)
{
	vector <float> params;
	sort(stateVal.begin(), stateVal.end());

	for (int i = 0; i < state.size(); i++)
	{
		if (getState(i) == stateVal[i])
		{
			switch (stateVal[i])
			{
			case X:
				params.push_back(getTx());
				break;
			case Y:
				params.push_back(getTy());
				break;
			case Z:
				params.push_back(getTz());
				break;
			case THETAX:
				params.push_back(getThetaX(DEGREES));
				break;
			case THETAY:
				params.push_back(getThetaY(DEGREES));
				break;
			case RZ:
				params.push_back(getThetaZ(DEGREES));
				break;
			default:
				break;
			}
		}
		else
		{
			cout << "Parameter " << stateVal[i] << " is disabled." << endl;
		}
	}

	return params;
}

// Get all camera parameters values
vector <float> CameraStateSpace::getParams()
{
	return	{ getTx(), getTy(), getTz(), getThetaX(DEGREES), getThetaY(DEGREES), getThetaZ(DEGREES) };
}

// Enable state parameters
void CameraStateSpace::setState(vector <State> stateVal)
{
	// Check size
	checkStateSize(stateVal.size());
	state.clear();
	state = stateVal;
	sort(state.begin(), state.end());
}

// Get specific state parameter
State CameraStateSpace::getState(int idx)
{
	// Check if idx is valid
	checkIdx("CameraStateSpace::state", idx, state.size());

	return state[idx];
}

// Get all state parameters
vector <State> CameraStateSpace::getStates() const
{
	return state;
}

// Get state parameters size
int CameraStateSpace::getStateSize() const
{
	return static_cast<int>(state.size());
}

// Check state parameters size
void CameraStateSpace::checkStateSize(size_t size)
{
	if ((size == 0) && (size > 6))
	{
		errorExit("CameraStateSpace::state", 6, LIMIT);
	}	
}
