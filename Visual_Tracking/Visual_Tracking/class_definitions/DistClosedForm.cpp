#include "..\headers\DistClosedForm.h"

// DistClosedForm class definition

DistClosedForm::DistClosedForm()
{
	wNum = -1;
}


DistClosedForm::~DistClosedForm()
{
}

// Set dij
void DistClosedForm::setDij(float dijVal)
{
	dij.push_back(abs(dijVal));
}

// Get dij
float DistClosedForm::getDij(int idx)
{
	// Check if idx is valid
	checkIdx("DistClosedForm::dij", idx, dij.size());

	return dij[idx];
}

// Get dijs
vector <float> DistClosedForm::getDijs() const
{
	return dij;
}

// Get dijs vector size
size_t DistClosedForm::getDijsSize() const
{
	return dij.size();
}

// Set fij
void DistClosedForm::setFij(float fijVal)
{
	fij.push_back(fijVal);
}

// Get fij
float DistClosedForm::getFij(int idx)
{
	// Check if idx is valid
	checkIdx("DistClosedForm::fij", idx, fij.size());

	return fij[idx];
}

// Get fijs
vector<float> DistClosedForm::getFijs() const
{
	return fij;
}

// Get fijs vector size
size_t DistClosedForm::getFijsSize() const
{
	return fij.size();
}

// Calculate fijs linear combination coefficients
void DistClosedForm::calcFijWeights(int edgesNum, int wnum)
{
	if (getWnum() > 0)
	{
		Mat Fij = Mat(wnum * edgesNum, getWnum(), CV_32F);
		Mat dij = Mat((int)getDijs().size(), 1, CV_32F);
		Mat w = Mat(getWnum(), 1, CV_32F);

		// Initialise Fij
		for (int i = 0; i < edgesNum * wnum; i++)
		{
			for (int j = 0; j < getWnum(); j++)
			{
				Fij.at<float>(i, j) = getFij(i*getWnum() + j);
			}
		}

		// Initialise dij
		memcpy(dij.data, getDijs().data(), getDijs().size() * sizeof(float));

		// Calculate weights
		Mat FijInverse;
		invert(Fij.t() * Fij, FijInverse);
		w = FijInverse * Fij.t() * dij;
		for (int i = 0; i < getWnum(); i++)
		{
			this->w.push_back(w.at<float>(i));
		}
	}
	else
	{
		cout << "wNum must be a positive integer." << endl;
		exit(EXIT_FAILURE);
	}
}

// Get weight
float DistClosedForm::getW(int idx)
{
	// Check if idx is valid
	checkIdx("DistClosedForm::w", idx, w.size());

	return w[idx];
}

// Get weights
vector<float> DistClosedForm::getWs() const
{
	return w;
}

// Set wNum
void DistClosedForm::setWnum(int wNumVal)
{
	if (wNumVal >= 0)
	{
		wNum = wNumVal;
		return;
	}
	
	cout << "wNum must be a positive integer." << endl;
	return;
}

// Get wNum
int DistClosedForm::getWnum() const
{
	return wNum;
}

// Clear all data
void DistClosedForm::Reset()
{
	dij.clear();
	fij.clear();
	w.clear();
	wNum = -1;
}