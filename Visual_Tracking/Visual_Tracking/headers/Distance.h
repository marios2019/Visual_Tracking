#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define VERTICES 8

using namespace cv;
using namespace std;

#include "error.h"

class Distance
{
public:
	Distance(); // Empty contructor
	~Distance(); // Destructor

	void setVertexDeriv(Point3f, Point3f); // Set vertex derivative of the cuboid2d, in homogeneous coordinates
	Point3f getVertexDeriv(int); // Returns a homogeneous vertex of the cuboid2d
	vector <Point3f> getVerticesDeriv() const; // Returns a vector of cuboid2d homogeneous vertices derivatives
	int getVerticesDerivSize() const; // Returns the number of vertices homogeneous derivatives

	Point2f getPxlDeriv(int); // Returns a vertex derivative of the cuboid2d, in pixel coordinates
	vector <Point2f> getPxlsDeriv() const; // Returns a vector of cuboid2d vertices derivatives, in pixel coordinates
	size_t getPxlsDerivSize() const; // Returns the number of pixelDerivatives

	void setEdgeDeriv(vector <Point2f>); // Set edge derivative
	vector <Point2f> getEdgeDeriv(int); // Returns an edge derivative of the cuboid2d
	vector <vector <Point2f>> getEdgesDeriv() const; // Returns a vector of cuboid2d edges derivatives
	size_t getEdgesDerivSize() const; // Returns the number of edgesDerivatives

	void setInterval(Point2f); // Set edge's subinterval
	Point2f getInterval(int); // Returns an edge's subinterval of the cuboid2d
	vector <Point2f> getIntervals() const; // Returns a vector of cuboid2d edges subintervals
	size_t getIntervalsSize() const; // Returns the number of mij

	void setIntervalDeriv(Point2f); // Set edge's subinterval derivative
	Point2f getIntervalDeriv(int); // Returns an edge's subinterval derivative of the cuboid2d
	vector <Point2f> getIntervalsDeriv() const; // Returns a vector of cuboid2d edges subintervals derivatives
	size_t getIntervalsDerivSize() const; // Return the number of mij derivatives

	void setError(float); // Set error for one pair mij - sij
	float getErrors(int); // Returns a specific error
	vector <float> getErrors() const; // Returns a vector of errors
	size_t getErrorsSize() const; // Returns  the size of the error vector = 2(distance vector) * M (number of rendered model edges) * N (number of subintervals on each rendered model edge)

	void setIntervalNum(int); // Set edge's subinterval number
	int getIntervalNum() const; // Returns edge's subinterval number
	
private:
	vector <Point3f> verticesDerivatives; // vertices derivatives of the cuboid2d, in homogeneous coordinates
	vector <Point2f> pixelDerivatives; // vertices derivatives of the cuboid2d, in pixel coordinates
	vector <vector <Point2f>> edgesDerivatives; // edges derivatives of the cuboid2d
	vector <Point2f> mij; // edge's subintervals of the cuboid2d
	vector <Point2f> mijDerivatives; // edge's subintervals derivatives of the cuboid2d
	vector <float> errors; // distances between model and data object edges, from mij points to model edges to normal intersection at data edges - sij
	int mijNum; // number of subintervals on each edge

	void setPxlDeriv(Point3f, Point3f); // Set vertex derivative of the cuboid2d, in pixel coordinates
};