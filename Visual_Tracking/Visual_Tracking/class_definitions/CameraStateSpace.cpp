#include "..\headers\CameraStateSpace.h"

// CameraStateSpace class definition

// Constructor
CameraStateSpace::CameraStateSpace(float tXVal, float tYVal, float tZVal, float thetaXVal, float thetaYVal, float thetaZVal, vector <State> stateVal)
{
	// Parameters initialization
	tX = tXVal;
	tY = tYVal;
	tZ = tZVal;
	thetaX = fmod(thetaXVal, DEG) * PI / 180.f;
	thetaY = fmod(thetaYVal, DEG) * PI / 180.f;
	thetaZ = fmod(thetaZVal, DEG) * PI / 180.f;
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
	// Degrees
	if (angle == DEGREES)
	{
		thetaX = fmod(thetaXVal, DEG) * PI / 180.f;
		return;
	}

	// Radians
	thetaX = fmod(thetaXVal, 2.f * PI);
}

// Change y-axis rotation
void CameraStateSpace::setThetaY(float thetaYVal, Angle angle)
{
	// Degrees
	if (angle == DEGREES)
	{
		thetaY = fmod(thetaYVal, DEG) * PI / 180.f;
		return;
	}

	// Radians
	thetaY = fmod(thetaYVal, 2.f * PI);
}

// Change z-axis rotation
void CameraStateSpace::setThetaZ(float thetaZVal, Angle angle)
{
	// Degrees
	if (angle == DEGREES)
	{
		thetaZ = fmod(thetaZVal, DEG) * PI / 180.f;
		return;
	}

	// Radians
	thetaZ = fmod(thetaZVal, 2.f * PI);
}

// Return x-axis rotation
float CameraStateSpace::getThetaX(Angle angle) const
{
	// Degrees
	if (angle == DEGREES)
	{
		return thetaX * 180.f / PI;
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
		return thetaY * 180.f / PI;
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
		return thetaZ * 180.f / PI;
	}

	// Radians
	return thetaZ;
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
			case THETAZ:
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
	return getParams(state);
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
