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
void Distance::setVertexDeriv(Point3f vertexDerivVal, Point3f homogeneousVal)
{
	if (verticesDerivatives.size() <= VERTICES)
	{
		verticesDerivatives.push_back(vertexDerivVal);
		setPxlDeriv(vertexDerivVal, homogeneousVal);
	}
}

// Returns a homogeneous vertex of the cuboid2d
Point3f Distance::getVertexDeriv(int idx)
{
	// Check if idx is between 0..VERTICES=8
	checkIdx("Distance::verticesDerivatives", idx, VERTICES);

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
	checkIdx("Distance::pixelDerivatives", idx, pixelDerivatives.size());

	return pixelDerivatives[idx];
}

// Returns a vector of cuboid2d vertices derivatives, in pixel coordinates
vector <Point2f> Distance::getPxlsDeriv() const
{
	return pixelDerivatives;
}

// Returns the number of pixelDerivatives
size_t Distance::getPxlsDerivSize() const
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
	checkIdx("Distance::edgesDerivatives", idx, edgesDerivatives.size());
	
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
	checkIdx("Distance::mij", idx, mij.size());

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
	checkIdx("Distance::mijDerivatives", idx, mijDerivatives.size());

	return mijDerivatives[idx];
}

// Returns a vector of cuboid2d edges subintervals derivatives
vector <Point2f> Distance::getIntervalsDeriv() const
{
	return mijDerivatives;
}

// Return the number of mij derivatives
size_t Distance::getIntervalsDerivSize() const
{
	return mijDerivatives.size();
}

// Set error for one pair mij - sij
void Distance::setError(float errorsVal)
{
	errors.push_back(errorsVal);
}

// Returns a specific error
float Distance::getErrors(int idx)
{
	// Check if idx is valid
	checkIdx("Distance::errors", idx, errors.size());

	return errors[idx];
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
	Point2f pixelDeriv;
	if (homogeneousVal.z <= 0.f)
	{
		homogeneousVal.z = min(homogeneousVal.z, -0.0001f);
	}
	else
	{
		homogeneousVal.z = max(homogeneousVal.z, 0.0001f);
	}

	pixelDeriv.x = (vertexDerivVal.x * homogeneousVal.z - homogeneousVal.x * vertexDerivVal.z) / powf(homogeneousVal.z, 2.f);
	pixelDeriv.y = (vertexDerivVal.y * homogeneousVal.z - homogeneousVal.y * vertexDerivVal.z) / powf(homogeneousVal.z, 2.f);
	pixelDerivatives.push_back(pixelDeriv);
}