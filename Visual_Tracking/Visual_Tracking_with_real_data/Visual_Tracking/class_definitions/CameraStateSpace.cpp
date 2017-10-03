#include "..\headers\CameraStateSpace.h"

// CameraStateSpace class definition

// Constructor
CameraStateSpace::CameraStateSpace(float tXVal, float tYVal, float tZVal, float thetaXVal, float thetaYVal, float thetaZVal, vector <State> stateVal)
{
	// Parameters initialization
	tX = tXVal;
	tY = tYVal;
	tZ = tZVal;
	Vec3f axisAngle = euler2AxisAngle(thetaXVal, thetaYVal, thetaZVal);
	// Check if the axis angle is corresponds to a rotation matrix
	if (IsRotationMatrix(axisAngle2Matrix(axisAngle)))
	{
		r1 = axisAngle.val[0];
		r2 = axisAngle.val[1];
		r3 = axisAngle.val[2];
	}
	else
	{
		cout << "The input axis angle doesn't correspond to a rotation matrix: " << axisAngle << endl;
		cout << "r1, r2, r3 are set to zero." << endl;
		r1 = 0.f;
		r2 = 0.f;
		r3 = 0.f;
	}
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

// Set r1, r2, r3, input is an axis angle representation,
// where the axis is multiplied by the rotation angle, expressed in radians
void CameraStateSpace::setAxisAngle(Vec3f axisAngle)
{
	// Check if the axis angle is corresponds to a rotation matrix
	if (IsRotationMatrix(axisAngle2Matrix(axisAngle)))
	{
		r1 = axisAngle.val[0];
		r2 = axisAngle.val[1];
		r3 = axisAngle.val[2];
	}
	else
	{
		cout << "The input axis angle doesn't correspond to a rotation matrix: " << axisAngle << endl;
		cout << "r1, r2, r3 are set to zero." << endl;
		r1 = 0.f;
		r2 = 0.f;
		r3 = 0.f;
	}
}

// Return r1, r2, r3 in axis angle representation,
// where the axis is multiplied by the rotation angle, expressed in radians
Vec3f CameraStateSpace::getAxisAngle() const
{
	return Vec3f(r1, r2, r3);
}

// Change x-axis rotation
void CameraStateSpace::setThetaX(float thetaX, Angle angle)
{
	// Degrees
	if (angle == DEGREES)
	{
		thetaX = deg2rad(fmod(thetaX, _DEG));
	}
	else if (angle == RADIANS)
	{
		// Radians
		thetaX = fmod(thetaX, 2.f * _PI);
	}

	// Convert euler angles to axis angle
	Vec3f axisAngle = euler2AxisAngle(thetaX, getThetaY(RADIANS), getThetaZ(RADIANS));
	setAxisAngle(axisAngle);
}

// Change y-axis rotation
void CameraStateSpace::setThetaY(float thetaY, Angle angle)
{
	// Degrees
	if (angle == DEGREES)
	{
		thetaY = deg2rad(fmod(thetaY, _DEG));
	}
	else if (angle == RADIANS)
	{
		// Radians
		thetaY = fmod(thetaY, 2.f * PI);
	}

	// Convert euler angles to axis angle
	Vec3f axisAngle = euler2AxisAngle(getThetaX(RADIANS), thetaY, getThetaZ(RADIANS));
	setAxisAngle(axisAngle);
}

// Change z-axis rotation
void CameraStateSpace::setThetaZ(float thetaZ, Angle angle)
{
	// Degrees
	if (angle == DEGREES)
	{
		thetaZ = deg2rad(fmod(thetaZ, _DEG));
	}
	else if (angle == RADIANS)
	{
		// Radians
		thetaZ = fmod(thetaZ, 2.f * _PI);
	}

	// Convert euler angles to axis angle
	Vec3f axisAngle = euler2AxisAngle(getThetaX(RADIANS), getThetaY(RADIANS), thetaZ);
	setAxisAngle(axisAngle);
}

// Return x-axis rotation
float CameraStateSpace::getThetaX(Angle angle) const
{
	// Convert r1, r2, r3 to euler angles
	Vec3f eulerAngles = axisAngle2euler(getAxisAngle());
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(eulerAngles.val[0]);
	}

	// Radians
	return eulerAngles.val[0];
}

// Return y-axis rotation
float CameraStateSpace::getThetaY(Angle angle) const
{
	// Convert r1, r2, r3 to euler angles
	Vec3f eulerAngles = axisAngle2euler(getAxisAngle());
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(eulerAngles.val[1]);
	}

	// Radians
	return eulerAngles.val[1];
}

// Return z-axis rotation
float CameraStateSpace::getThetaZ(Angle angle) const
{
	// Convert r1, r2, r3 to euler angles
	Vec3f eulerAngles = axisAngle2euler(getAxisAngle());
	// Degrees
	if (angle == DEGREES)
	{
		return rad2deg(eulerAngles.val[2]);
	}

	// Radians
	return eulerAngles.val[2];
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
			case R1:
				params.push_back(r1);
				break;
			case R2:
				params.push_back(r2);
				break;
			case R3:
				params.push_back(r3);
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
	return	{ getTx(), getTy(), getTz(), r1, r2, r3 };
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
