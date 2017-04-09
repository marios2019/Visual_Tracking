#include "..\headers\CameraStateSpace.h"

// CameraStateSpace class definition

// Empty Constructor
CameraStateSpace::CameraStateSpace()
{
	// Parameters initialization
	tX = 0.f;
	tY = 0.f;
	tZ = 0.f;
	thetaX = 0.f;
	thetaY = 0.f;
	thetaZ = 0.f;
}

// Constructor
CameraStateSpace::CameraStateSpace(float tXVal, float tYVal, float tZVal, float thetaXVal, float thetaYVal, float thetaZVal, int paramsNumVal)
{
	// Parameters initialization
	tX = tXVal;
	tY = tYVal;
	tZ = tZVal;
	thetaX = thetaXVal * PI / 180.f;
	thetaY = thetaYVal * PI / 180.f;
	thetaZ = thetaZVal * PI / 180.f;
	parametersNum = paramsNumVal;
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

// Change camera position
void CameraStateSpace::setPosition(Vec3f tVal)
{
	tX = tVal.val[0];
	tY = tVal.val[1];
	tZ = tVal.val[2];
}

// Return camera position
Vec3f CameraStateSpace::getPosition() const
{
	return Vec3f(getTx(), getTy(), getTz());
}

// Change x-axis rotation
void CameraStateSpace::setThetaX(float thetaXVal)
{
	thetaX = thetaXVal * PI / 180.f;
}

// Change y-axis rotation
void CameraStateSpace::setThetaY(float thetaYVal)
{
	thetaY = thetaYVal * PI / 180.f;
}

// Change z-axis rotation
void CameraStateSpace::setThetaZ(float thetaZVal)
{
	thetaZ = thetaZVal * PI / 180.f;
}

// Return x-axis rotation
float CameraStateSpace::getThetaX(Type type) const
{
	if (type == DEGREES)
	{
		return thetaX * 180.f / PI;
	}
	else if (type = RADIANS)
	{
		return thetaX;
	}
	else
	{
		cout << "Metic unit must be DEGREES of RADIANS" << endl;
		exit(EXIT_FAILURE);
	}
}

// Return y-axis rotation
float CameraStateSpace::getThetaY(Type type) const
{
	if (type == DEGREES)
	{
		return thetaY * 180.f / PI;
	}
	else if (type == RADIANS)
	{
		return thetaY;
	}
	else
	{
		cout << "Metic unit must be DEGREES of RADIANS" << endl;
		exit(EXIT_FAILURE);
	}
}

// Return z-axis rotation
float CameraStateSpace::getThetaZ(Type type) const
{
	if (type == DEGREES)
	{
		return thetaZ * 180.f / PI;
	}
	else if (type == RADIANS)
	{
		return thetaZ;
	}
	else
	{
		cout << "Metic unit must be DEGREES of RADIANS" << endl;
		exit(EXIT_FAILURE);
	}
}

// Set camera parameters
void CameraStateSpace::setParams(vector <float> paramsVal, vector <State> stateVal)
{
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

// Get camera parameters
vector <float> CameraStateSpace::getParams(vector <State> stateVal)
{
	vector <float> params;
	for (int i = 0; i < stateVal.size(); i++)
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

	return params;
}

// Set the parameters number
void CameraStateSpace::setParamsNum(int paramsNumVal)
{
	if (paramsNumVal > 0)
	{
		parametersNum = paramsNumVal;
	}
}

// Return the number of camera parameters
int CameraStateSpace::getParamsNum() const
{
	return parametersNum;
}