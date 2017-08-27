#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

#include "error.h"

class DistClosedForm
{
public:
	DistClosedForm(); // Empty Contsructor
	~DistClosedForm(); // Destructor

	void setDij(float); // Set dij
	float getDij(int); // Get dij
	vector <float> getDijs() const; // Get dijs
	size_t getDijsSize() const; // Get dijs vector size

	void setFij(float); // Set fij
	float getFij(int); // Get fij
	vector <float> getFijs() const; // Get fijs
	size_t getFijsSize() const; // Get fijs vector size

	void calcFijWeights(int, int); // Calculate fijs linear combination coefficients

	float getW(int); // Get weight
	vector <float> getWs() const; // Get weights

	void setWnum(int); // Set wNum
	int getWnum() const; // Get wNum

	void Reset(); // Clear all data
private:
	vector <float> dij; // Measurable distance between model and data
	vector <float> fij; // mij derivatives projection onto edges normal vectors
	vector <float> w; // Coefficients of the linear combination of the
					  // partial derivative of fijs in respect to the i-th generator
					  // which form the dijProg
	int wNum; // Number of coefficients equals to the number of the generators
};

