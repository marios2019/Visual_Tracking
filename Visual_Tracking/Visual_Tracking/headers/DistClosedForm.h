#pragma once
// OpenCV libraries
#include "opencv2/imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

class DistClosedForm
{
public:
	DistClosedForm(); // Empty Contsructor
	~DistClosedForm(); // Destructor

	void setDij(float); // Set dij
	float getDij(int); // Get dij
	vector <float> getDijs() const; // Get dijs

	void setFij(float); // Set fij
	float getFij(int); // Get fij
	vector <float> getFijs() const; // Get fijs

	void calcDijsProg(int); // Calculate dijsProg
	float getDijProg(int); // Get dijProg
	vector <float> getDijsProg() const; // Get dijsProg

	float getW(int); // Get w
	vector <float> getWs() const; // Get ws

	void setWnum(int); // Set wNum
	int getWnum() const; // Get wNum

private:
	vector <float> dij; // Measurable distance between model and data
	vector <float> fij; // mij derivatives projection onto edges normal vectors
	vector <float> dijProg; // Closed form distances
	vector <float> w; // Coefficients of the linear combination of the
					  // partial derivative of fijs in respect to the i-th generator
					  // which form the dijProg
	int wNum; // Number of coefficients equals to the number of the generators

	void checkIdx(int, size_t); // Check for invalid memory access
};

