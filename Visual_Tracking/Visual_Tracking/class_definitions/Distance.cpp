#include "..\headers\Distance.h"

// Distance class definition

// Contructor
Distance::Distance()
{
	mijNum = 0;
}

// Destructor
Distance::~Distance()
{
}

// Set vertex derivative of the cuboid2d, in homogeneous coordinates
void Distance::setVerticeDeriv(Point3f vertexDerivVal, Point3f homogeneousVal)
{
	if (verticesDerivatives.size() <= VERTICES)
	{
		verticesDerivatives.push_back(vertexDerivVal);
		setPxlDeriv(vertexDerivVal, homogeneousVal);
	}
}

// Returns a homogeneous vertex of the cuboid2d
Point3f Distance::getVerticeDeriv(int idx)
{
	// Check if idx is between 0..VERTICES=8
	checkIdx(idx, VERTICES);

	return verticesDerivatives[idx];
}

// Returns a vector of cuboid2d homogeneous vertices derivatives
vector<Point3f> Distance::getVerticesDeriv() const
{
	return verticesDerivatives;
}

// Returns the number of vertices homogeneous derivatives
int Distance::getVerticesDerivSize() const
{
	return VERTICES;
}

// Returns a vertex derivative of the cuboid2d, in pixel coordinates
Point2f Distance::getPxlDeriv(int idx)
{
	// Check if idx is valid
	checkIdx(idx, pixelDerivatives.size());

	return pixelDerivatives[idx];
}

// Returns a vector of cuboid2d vertices derivatives, in pixel coordinates
vector <Point2f> Distance::getPxlDeriv() const
{
	return pixelDerivatives;
}

// Returns the number of pixelDerivatives
size_t Distance::getPxlDerivSize() const
{
	return pixelDerivatives.size();
}

// Set edge derivative
void Distance::setEdgeDeriv(vector <Point2f> edgeDerivVal)
{
	edgesDerivatives.push_back(edgeDerivVal);
}

// Returns an edge derivative of the cuboid2d
vector <Point2f> Distance::getEdgeDeriv(int idx)
{
	// Check if idx is valid
	checkIdx(idx, edgesDerivatives.size());
	
	return edgesDerivatives[idx];
}

// Returns a vector of cuboid2d edges derivatives
vector <vector <Point2f>> Distance::getEdgesDeriv() const
{
	return edgesDerivatives;
}

// Returns the number of edgesDerivatives
size_t Distance::getEdgesDerivSize() const
{
	return edgesDerivatives.size();
}

// Set edge's subinterval
void Distance::setInterval(Point2f mijVal)
{
	mij.push_back(mijVal);
}

// Returns an edge's subinterval of the cuboid2d
Point2f Distance::getInterval(int idx)
{
	// Check if idx is valid
	checkIdx(idx, mij.size());

	return mij[idx];
}

// Returns a vector of cuboid2d edges subintervals
vector <Point2f> Distance::getIntervals() const
{
	return mij;
}

// Returns the number of mij
size_t Distance::getIntervalsSize() const
{
	return mij.size();
}

// Set edge's subinterval derivative
void Distance::setIntervalDeriv(Point2f mijDerivativeVal)
{
	mijDerivatives.push_back(mijDerivativeVal);
}

// Returns an edge's subinterval derivative of the cuboid2d
Point2f Distance::getIntervalDeriv(int idx)
{
	// Check if idx is valid
	checkIdx(idx, mijDerivatives.size());

	return mijDerivatives[idx];
}

// Set error for one pair mij - sij
void Distance::setError(Vec2f errorsVal)
{
	errors.push_back(errorsVal.val[0]);
	errors.push_back(errorsVal.val[1]);
}

// Returns a vector of errors for a specific parameter derivative
vector <float> Distance::getErrors() const
{
	return errors;
}

// Returns  the size of the error vector = 2(distance vector) * M (number of rendered model edges) * N (number of subintervals on each rendered model edge)
size_t Distance::getErrorsSize() const
{
	return errors.size();
}

// Set error derivative for one pair mij - sij
void Distance::setErrorDeriv(Vec2f errorDerivVal)
{
	errorsDeriv.push_back(errorDerivVal.val[0]);
	errorsDeriv.push_back(errorDerivVal.val[1]);
}

// Returns a vector of errors derivatives
vector <float> Distance::getErrorsDeriv() const
{
	return errorsDeriv;
}

// Returns  the size of the error vector derivatives = 2(distance vector) * M (number of rendered model edges) * N (number of subintervals on each rendered model edge)
size_t Distance::getErrorsDerivSize() const
{
	return errorsDeriv.size();
}

// Returns a vector of cuboid2d edges subintervals derivatives
vector <Point2f> Distance::getIntervalsDeriv() const
{
	return mijDerivatives;
}

// Returns the number of mij
size_t Distance::getIntervalsDerivSize() const
{
	return mijDerivatives.size();
}

// Set edge's subinterval number
void Distance::setIntervalNum(int mijNumVal)
{
	if (mijNumVal < 0)
	{
		cout << "Subintervals number must be positive." << endl;
		exit(EXIT_FAILURE);
	}

	mijNum = mijNumVal;
}

// Returns edge's subinterval number
int Distance::getIntervalNum() const
{
	return mijNum;
}

// Set vertex derivative of the cuboid2d, in pixel coordinate
void Distance::setPxlDeriv(Point3f vertexDerivVal, Point3f homogeneousVal)
{
	Point2f tmp;
	Point2f pixelDeriv;
	if (homogeneousVal.z <= 0.f)
	{
		homogeneousVal.z = min(homogeneousVal.z, -0.0001f);
	}
	else
	{
		homogeneousVal.z = max(homogeneousVal.z, 0.0001f);
	}

	tmp.x = (vertexDerivVal.x * homogeneousVal.z - homogeneousVal.x * vertexDerivVal.z) / powf(homogeneousVal.z, 2.f);
	tmp.y = (vertexDerivVal.y * homogeneousVal.z - homogeneousVal.y * vertexDerivVal.z) / powf(homogeneousVal.z, 2.f);
	pixelDeriv = tmp;
	pixelDerivatives.push_back(pixelDeriv);
}

// Check for invalid memory access
void Distance::checkIdx(int idx, size_t limit)
{
	if ((idx >= limit) || (idx < 0))
	{
		cout << "ERROR, invalid memory access." << endl;
		exit(EXIT_FAILURE);
	}
}
