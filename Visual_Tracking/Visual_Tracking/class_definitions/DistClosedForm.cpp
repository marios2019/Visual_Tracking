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
	dij.push_back(dijVal);
}

// Get dij
float DistClosedForm::getDij(int idx)
{
	// Check if idx is valid
	checkIdx(idx, dij.size());

	return dij[idx];
}

// Get dijs
vector <float> DistClosedForm::getDijs() const
{
	return dij;
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
	checkIdx(idx, fij.size());

	return fij[idx];
}

// Get fijs
vector<float> DistClosedForm::getFijs() const
{
	return fij;
}

// Calculate dijsProg
void DistClosedForm::calcDijsProg(int edgesNum)
{
	if (getWnum() >= 0)
	{
		Mat Fij = Mat(edgesNum, getWnum(), CV_32F);
		Mat dij = Mat((int)getDijs().size(), 1, CV_32F);
		Mat W = Mat(getWnum(), 1, CV_32F);

		// Initialise Fij
		for (int i = 0; i < edgesNum; i++)
		{
			for (int j = 0; j < getWnum(); j++)
			{
				Fij.at<float>(i, j) = getFij(i*getWnum() + j);
			}
		}

		// Initialise dij
		memcpy(dij.data, getDijs().data(), getDijs().size() * sizeof(float));

		// Calculate w
		Mat FijInverse;
		invert(Fij.t() * Fij, FijInverse);
		W = FijInverse * Fij.t() * dij;
		for (int i = 0; i < getWnum(); i++)
		{
			w.push_back(W.at<float>(i));
		}
	}
	else
	{
		cout << "wNum must be a positive integer." << endl;
		exit(EXIT_FAILURE);
	}
}

// Get dijProg
float DistClosedForm::getDijProg(int idx)
{
	// Check if idx is valid
	checkIdx(idx, dijProg.size());

	return dijProg[idx];
}

// Get dijsProg
vector<float> DistClosedForm::getDijsProg() const
{
	return dijProg;
}

// Get w
float DistClosedForm::getW(int idx)
{
	// Check if idx is valid
	checkIdx(idx, w.size());

	return w[idx];
}

// Get ws
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
	}
	else
	{
		cout << "wNum must be a positive integer." << endl;
		exit(EXIT_FAILURE);
	}
}

// Get wNum
int DistClosedForm::getWnum() const
{
	return wNum;
}

// Check for invalid memory access
void DistClosedForm::checkIdx(int idx, size_t limit)
{
	if ((idx >= limit) || (idx < 0))
	{
		cout << "ERROR, invalid memory access." << endl;
		exit(EXIT_FAILURE);
	}
}


