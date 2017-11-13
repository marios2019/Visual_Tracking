#include "../headers/mathLinearAlgebra.h"

// Overload
// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f vec)
{
	float norm = sqrt(powf(vec.val[0], 2.f) + powf(vec.val[1], 2.f) + powf(vec.val[2], 2.f));

	return max(norm, 0.0001f);
}

// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f vec)
{
	float norm = sqrt(powf(vec.val[0], 2.f) + powf(vec.val[1], 2.f));

	return max(norm, 0.0001f);
}

// Compute squared euclidean distance between two 2D points
float euclideanDistanceSquared(Point2i p, Point2i q)
{
	return (powf((float)(q.x - p.x), 2.f) + powf((float)(q.y - p.y), 2.f));
}

// Compute euclidean distance between two 2D points
float euclideanDistance(Point2f p, Point2f q)
{
	return powf((q.x - p.x), 2.f) + powf((q.y - p.y), 2.f);
}

// Return a skew symmetric matrix of a 3D vector, used for cross product
Mat skewMat(Vec3f m)
{
	Mat sk = (Mat_<float>(3, 3) << 0, -m.val[2], m.val[1], m.val[2], 0, -m.val[0], -m.val[1], m.val[0], 0);

	return sk;
}

// Decompose camera pose matrix
void decomposeCameraPose(Mat E, Mat &R, Vec3f &t)
{
	// Check if matrix is 3 x 4
	if (checkMatrixSize(E.size(), 3, 4))
	{
		return;
	}
	
	// Extract 3D rotation matrix
	Mat Rtemp = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rtemp = E(r).clone();
	transpose(Rtemp, Rtemp);
	if (!IsRotationMatrix(Rtemp))
	{
		cout << "The input matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rtemp.copyTo(R);

	// Extract position vector
	Mat ttemp;
	Vec3f Rt;
	Rt.val[0] = E.at<float>(0, 3);
	Rt.val[1] = E.at<float>(1, 3);
	Rt.val[2] = E.at<float>(2, 3);
	Rt = -Rt;
	ttemp = R * Mat(Rt);
	t = ttemp;
}

// Decompose euclidean transformation matrix expressed in homogeneous coordinates - matrix size 4 x 4
void decomposeEuclidean(Mat E, Mat &R, Vec3f &t)
{
	// Check if matrix is 4 x 4
	if (checkMatrixSize(E.size(), 4, 4))
	{
		return;
	}

	// Extract 3D rotation matrix
	Mat Rtemp = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rtemp = E(r).clone();
	if (!IsRotationMatrix(Rtemp))
	{
		cout << "The input euclidean transformation matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rtemp.copyTo(R);

	// Extract position vector
	t.val[0] = E.at<float>(0, 3);
	t.val[1] = E.at<float>(1, 3);
	t.val[2] = E.at<float>(2, 3);
}

//  Create 4 x 4 camera pose matrix
void cameraPose(Mat R, Mat t, Mat &E)
{
	if (!IsRotationMatrix(R))
	{
		cout << "The input matrix R is not a 3D rotation matrix." << endl;
		return;
	}

	hconcat(R.t(), -R.t() * t, E);
	Mat row = (Mat_<float>(1, 4) << 0.f, 0.f, 0.f, 1.f);
	E.push_back(row);
}

// Perspective projection
Point3f perspectiveProjection(Point3f point, Mat P)
{
	// Convert point cartesian coordinates to homogeneous coordinates
	Vec4f homogeneousPoint4D = cart2hmgns(point);
	// Perspective projection
	Mat homogeneousPoint3D = P * Mat(homogeneousPoint4D);

	return Point3f(homogeneousPoint3D);
}

// 3D Cartesian to 4D homogeneous coordinates conversion
Vec4f cart2hmgns(Vec3f cartesian)
{
	return Vec4f(cartesian.val[0], cartesian.val[1], cartesian.val[2], 1.f);
}

// Check if the input matrix has the desirable dimensions
bool checkMatrixSize(Size size, int rows, int cols)
{
	if ((rows != size.height) || (cols != size.width))
	{
		cout << "The dimensions of the input matrix must be " << rows << " x " << cols <<"." << endl;
		return 1;
	}

	return 0;
}


// Vector calculus

// Compute the Jacobian matrix of the perspective projection process
// in respect to the parameters xk. 
// Inputs: matrix V = { 3D points }
//		   matrix K = { camera extrinsics }
//         vector xk = { parameters }
//         vector x = { parameters values }
// Output: matrix Jvh = { Jacobian matrix with the 3D projection homogeneous points }
Mat jacobianPerspectiveProjection(Mat V, Mat K, Mat x, vector <Parameter> xk)
{
	Mat Jvph(V.rows, static_cast<int>(xk.size()), CV_32FC3);
	
	// Compute Jacobian
	for (int i = 0; i < xk.size(); i++)
	{
		// Perpective projection firt derivative in respect with i-th x parameter
		Mat dP = K * cameraPoseFirstDerivative(x, xk[i]);

		for (int j = 0; j < V.rows; j++)
		{
			// Extract one row from V at a time and convert to 4D homogeneous
			Vec4f vh = cart2hmgns(static_cast<Vec3f>(V.row(j)));

			Mat dvph = dP * Mat(vh);
			Jvph.at<Vec3f>(j, i) = dvph;
		}
	}

	return Jvph;
}

// Compute the Jacobian matrix of the pixel coordinates
// Inputs: matrix Vph = { 3D homogeneours projection points }
//		   matrix Jvh = { first derivatives of projection homogeneous coordinates }
// Output: matrix Jvp = { first derivatives of _PIxel coordinates }
Mat jacobianPixelCoordinates(Mat Vph, Mat Jvph)
{
	Mat Jvp(Jvph.rows, Jvph.cols, CV_32FC2);

	// Compute Jacobian
	for (int i = 0; i < Jvp.rows; i++)
	{
		// Extract one vertex from V at a time
		Vec3f vph = static_cast<Vec3f>(Vph.row(i));
		if (vph.val[2] <= 0.f)
		{
			vph.val[2] = min(vph.val[2], -0.0001f);
		}
		else
		{
			vph.val[2] = max(vph.val[2], 0.0001f);
		}
		for (int j = 0; j < Jvp.cols; j++)
		{
			// Extract one homogeneous projection vertex derivative from Jvh at a time
			Vec3f dvph = Jvph.at<Vec3f>(i, j);

			// _PIxel coordinate first derivative
			Vec2f dvp;
			dvp.val[0] = (dvph.val[0] * vph.val[2] - vph.val[0] * dvph.val[2]) / powf(vph.val[2], 2.f);
			dvp.val[1] = (dvph.val[1] * vph.val[2] - vph.val[1] * dvph.val[2]) / powf(vph.val[2], 2.f);
			Jvp.at<Vec2f>(i, j) = dvp;
		}
	}

	return Jvp;
}

// Compute the Jacobian matrix of the 2D edges
// Inputs: matrix Jvp = { 2D projection points first derivatives}
//		   vector edges2DPtr = { pointers to 2D projection _VERTICES }
// Output: matrix Jep = { first derivatives of 2D edges }
Mat jacobianEdges(Mat Jvp, vector <vector <int>> edges2DPtr)
{
	Mat Jep(static_cast<int>(edges2DPtr.size()), Jvp.cols, CV_32FC4);

	// Compute Jacobian
	for (int i = 0; i < Jep.rows; i++)
	{
		vector <int> edge2DPtr = { edges2DPtr[i] };

		for (int j = 0; j < Jep.cols; j++)
		{
			// Extract the _VERTICES of the edge
			Vec2f dvp1 = Jvp.at<Vec2f>(edge2DPtr[0], j), dvp2 = Jvp.at<Vec2f>(edge2DPtr[1], j);

			// Edge first derivative
			Vec4f dep = Vec4f(dvp1.val[0], dvp1.val[1], dvp2.val[0], dvp2.val[1]);
			Jep.at<Vec4f>(i, j) = dep;
		}
	}

	return Jep;
}

// Compute mijs first derivatives
Mat jacobianMijs(Mat Jep, int mNum)
{
	Mat Jmijs(Jep.rows * (mNum - 2), Jep.cols, CV_32FC2);

	// Compute Jacobian
	for (int i = 0; i < Jep.rows; i++)
	{
		for (int j = 0; j < Jep.cols; j++)
		{
			for (int k = 0; k < (mNum - 2); k++)
			{
				int x = i * (mNum - 2) + k , y = j;
				Vec4f dedge = Jep.at<Vec4f>(i, j);
				Vec2f dp1(dedge.val[0], dedge.val[1]), dp2(dedge.val[2], dedge.val[3]), dvec = dp2 - dp1;
				Jmijs.at<Vec2f>(x, y) = dp1 + static_cast<float>(k + 1) * dvec / static_cast<float>(mNum - 1);
			}
		}
	}

	return Jmijs;
}

// Compute first derivatives of distances dijs
Mat jacobianDijs(Mat mijs, Mat Jmijs, Mat dxDist, Mat dyDist)
{
	Mat Jdijs(Jmijs.size(), CV_32F);
	Mat mijsVec(mijs.rows * mijs.cols, 1, CV_32FC2);
	for (int j = 0; j < mijs.cols; j++)
	{
		for (int i = 0; i < mijs.rows; i++)
		{
			mijsVec.at<Point2f>((j * mijs.rows) + i, 0) = mijs.at<Point2f>(i, j);
		}
	}


	for (int i = 0; i < Jdijs.rows; i++)
	{
		for (int j = 0; j < Jdijs.cols; j++)
		{
			Point2i mij = mijsVec.at<Point2f>(i, 0);
			Point2f dmij = Jmijs.at<Point2f>(i, j);
			Mat dxdyDist = (Mat_<float>(2, 1) << dxDist.at<float>(mij.y, mij.x), dyDist.at<float>(mij.y, mij.x));
			Jdijs.at<float>(i, j) = static_cast<float>(dxdyDist.dot(Mat(dmij)));
		}
	}

	return Jdijs;
}

// Compose camera's pose matrix first derivative
Mat cameraPoseFirstDerivative(Mat x, Parameter xk)
{
	Mat dE;
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	// Camera position vector
	Mat t;
	for (int i = 0; i < 3; i++)
	{
		t.push_back(x.row(i));
	}
	// Camera rotation matrix
#ifdef _EULER
	Vec3f eulerAngles = axisAngle2euler(Vec3f(x.at<float>(3, 0), x.at<float>(4, 0), x.at<float>(5, 0)));
	Mat R = eulerAngles2Matrix(eulerAngles.val[0], eulerAngles.val[1], eulerAngles.val[2]);
#endif
#ifdef _AXISANGLE
	Vec3f axisAngle(x.at<float>(3, 0), x.at<float>(4, 0), x.at<float>(5, 0));
	Mat R = axisAngle2Matrix(axisAngle);
#endif

	// Camera's position vector and rotation matrix first derivatives
	Mat dt = cameraPositionFirstDerivative(xk);
	Mat dR = cameraRotationFirstDerivative(xk, x);
	Mat dRinv = inverseRotationMatrixDerivative(R, dR);
	hconcat(dRinv, -(dRinv * t + R.t() * dt), dE);

	return dE;
}

// Return camera position vector first derivative in respect of the state parameters
Mat cameraPositionFirstDerivative(Parameter xk)
{
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	switch (parameter)
	{
		case 0: // x = tx
			return (Mat_<float>(3, 1) << 1.f, 0.f, 0.f);
		case 1: // x = ty
			return (Mat_<float>(3, 1) << 0.f, 1.f, 0.f);;
		case 2: // x = tz
			return (Mat_<float>(3, 1) << 0.f, 0.f, 1.f);
		case 3: // x = thetax
			return Mat::zeros(3, 1, CV_32F);
		case 4: // x = thetay
			return Mat::zeros(3, 1, CV_32F);
		case 5: // x = thetaz
			return Mat::zeros(3, 1, CV_32F);
		default:
		{
			cout << "Not valid state parameter." << endl;
			return Mat();
		}
	}
}

#ifdef _EULER
// Return camera rotation matrix first derivative in respect of the state parameters
Mat cameraRotationFirstDerivative(Parameter xk, Mat x)
{
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	Vec3f eulerAngles = axisAngle2euler(Vec3f(x.at<float>(3, 0), x.at<float>(4, 0), x.at<float>(5, 0)));
	float thetaX = eulerAngles.val[0], thetaY = eulerAngles.val[1], thetaZ = eulerAngles.val[2];

	switch (parameter)
	{
		case 0: // x = tx
			return rotationFirstDerivative(xk, 0.f);
		case 1: // x = ty
			return rotationFirstDerivative(xk, 0.f);
		case 2: // x = tz
			return rotationFirstDerivative(xk, 0.f);
		case 3: // x = thetax
			return rotationY(thetaY) * rotationZ(thetaZ) * rotationFirstDerivative(xk, thetaX);
		case 4: // x = thetay
			return rotationFirstDerivative(xk, thetaY) * rotationZ(thetaZ) * rotationX(thetaX);
		case 5: // x = thetaz
			return rotationY(thetaY) * rotationFirstDerivative(xk, thetaZ) * rotationX(thetaX);
		default:
		{
			cout << "Not valid state parameter." << endl;
			return Mat();
		}
	}
}

// Return rotation matrix first derivative in respect parameter xk
Mat rotationFirstDerivative(Parameter xk, float theta)
{
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	switch (parameter)
	{
		case 0: // x = tx
			return Mat::zeros(3, 3, CV_32F);
		case 1: // x = ty
			return Mat::zeros(3, 3, CV_32F);
		case 2: // x = tz
			return Mat::zeros(3, 3, CV_32F);
		case 3: // x = thetax
			return (Mat_<float>(3, 3) << 0, 0, 0, 0, -sin(theta), -cos(theta), 0, cos(theta), -sin(theta));
		case 4: // x = thetay
			return (Mat_<float>(3, 3) << -sin(theta), 0, cos(theta), 0, 0, 0, -cos(theta), 0, -sin(theta));
		case 5: // x = thetaz
			return (Mat_<float>(3, 3) << -sin(theta), -cos(theta), 0, cos(theta), -sin(theta), 0, 0, 0, 0);
		default:
		{
			cout << "Not valid state parameter." << endl;
			return Mat();
		}
	}
}
#endif

// Computer the first derivative of the inverse of 3D rotation matrix
Mat inverseRotationMatrixDerivative(Mat R, Mat dR)
{
	// Check if R is a rotation matrix
	if (!IsRotationMatrix(R))
	{
		cout << "The input matrix R is not a 3D rotation matrix." << endl;
		return Mat();
	}

	// Check if dR matrix is 3 x 3
	if (checkMatrixSize(dR.size(), 3, 3))
	{
		return Mat();
	}

	return -R.t() * dR * R.t();
}

// Image gradient to one direction
Mat imageGradient(Mat Img, PartialDeriv partial)
{
	Mat dImg = Mat(Img.size(), CV_32F);
	// Choose derivative
	if (partial == Dx)
	{
		Sobel(Img, dImg, CV_32F, 1, 0, 3, 1, 0);
	}
	else
	{
		Sobel(Img, dImg, CV_32F, 0, 1, 3, 1, 0);
	}

	return dImg;
}

#ifdef _AXISANGLE
// Return camera rotation matrix first derivative in respect of the state parameters
Mat cameraRotationFirstDerivative(Parameter xk, Mat x)
{
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	Vec3f axisAngle(x.at<float>(3, 0), x.at<float>(4, 0), x.at<float>(5, 0));

	switch (xk) // Choose parameter
	{
		case X0: // x = tx
			return Mat::zeros(3, 3, CV_32F);
		case X1: // x = ty
			return Mat::zeros(3, 3, CV_32F);
		case X2: // x = tz
			return Mat::zeros(3, 3, CV_32F);
		case X3: // x = r1
			return axisAngleFirstDerivative(axisAngle, xk);
		case X4: // x = r2
			return axisAngleFirstDerivative(axisAngle, xk);
		case X5: // x = r3
			return axisAngleFirstDerivative(axisAngle, xk);
		default:
		{
			cout << "Not valid state parameter." << endl;
			return Mat();
		}
	}	
}

// Return axis angle matrix derivative
Mat axisAngleFirstDerivative(Vec3f axisAngle, Parameter xk)
{	
	Mat dR(3, 3, CV_32F);
	
	for (int i = 0; i < dR.rows; i++)
	{
		for (int j = 0; j < dR.cols; j++)
		{
			if (i == j)
			{// Calculate derivative of diagonal elements
				dR.at<float>(i, j) = diagonalElementsDerivatives(axisAngle, xk, i);
			}
			else
			{// Calculate derivative of the non diagonal elements - the matrix is like a skew symmetric
				dR.at<float>(i, j) = elementsDerivatives(axisAngle, xk, i, j);
			}
		}
	}

	return dR;
}

// First derivvative of diagonal elements of axis angle matrix
float diagonalElementsDerivatives(Vec3f axisAngle, Parameter xk, int elem)
{
	if ((xk < X3) || (xk > X5))
	{
		cout << "Invalid state parameter: " << xk << "; should be between X3(3) and X5(5)" << endl;
		return 0.f;
	}

	// diagonal element first derivative
	int parameter = static_cast<int>(xk) - 3;
	Vec3f e = basis3DVectors(elem);
	float theta = norm2(axisAngle);
	float drii = 2.f * (e.val[parameter] * theta * theta - axisAngle.val[elem] * axisAngle.val[elem] * axisAngle.val[parameter]) * (1.f - cos(theta));
	drii += axisAngle.val[parameter] * theta * sin(theta) * (axisAngle.val[elem] * axisAngle.val[elem] - theta * theta);
	theta = max(theta, 0.1f);
	drii /= pow(theta, 4.f);

	return drii;
}

// First derivative of non diagonal elements of axis angle matrix - it's similar to a skew symmetric matrix
float elementsDerivatives(Vec3f axisAngle, Parameter xk, int row, int col)
{
	if ((xk < X3) || (xk > X5))
	{
		cout << "Invalid state parameter: " << xk << "; should be between X3(3) and X5(5)" << endl;
		return 0.f;
	}

	// non diagonal element first derivative
	int parameter = static_cast<int>(xk) - 3;
	int lastElem = 3 - (row + col);
	Vec3f ei = basis3DVectors(row);
	Vec3f ej = basis3DVectors(col);
	Vec3f elast = basis3DVectors(lastElem);
	float theta = norm2(axisAngle);
	float drijFirstTerm = (ei.val[parameter] * theta * theta - axisAngle.val[row] * axisAngle.val[parameter]) * axisAngle.val[col];
	drijFirstTerm += (ej.val[parameter] * theta * theta - axisAngle.val[col] * axisAngle.val[parameter]) * axisAngle.val[row];
	drijFirstTerm /= pow(max(theta, 0.1f), 4.f);
	drijFirstTerm *= (1.f - cos(theta));
	float drijSecondTerm = 0.f;
	if (((row == 0) && (col == 1)) || ((row == 1) && (col == 2)) || ((row == 2) && (col == 0)))
	{
		drijSecondTerm += (axisAngle.val[row] * axisAngle.val[col] - axisAngle.val[lastElem]) * axisAngle.val[parameter] / pow(max(theta, 0.01f), 2.f);
		drijSecondTerm -= (elast.val[parameter] * theta * theta - axisAngle.val[lastElem] * axisAngle.val[parameter]) / pow(max(theta, 0.1f), 3.f);
		drijSecondTerm *= sin(theta);
	}
	else
	{
		drijSecondTerm += (axisAngle.val[row] * axisAngle.val[col] + axisAngle.val[lastElem]) * axisAngle.val[parameter] / pow(max(theta, 0.01f), 2.f);
		drijSecondTerm += (elast.val[parameter] * theta * theta - axisAngle.val[lastElem] * axisAngle.val[parameter]) / pow(max(theta, 0.1f), 3.f);
		drijSecondTerm *= sin(theta);
	}

	return drijFirstTerm + drijSecondTerm;
}

// Axis angle rotation matrix partial derivative - first term ([normalised(r)]x * sin(theta))
Mat axisAngleFirstDerivative_term1(Vec3f axisAngle, Parameter xk)
{
	Vec4f axisNormalised = axisAngleConversion(axisAngle);
	// Normalised axis skew symmetric matrix
	Mat skew_r = skewMat(Vec3f(axisNormalised.val[0], axisNormalised.val[1], axisNormalised.val[2]));
	// Normalised axis skew symmetric matrix first derivative
	Mat Dskew_r;
	// Rotation angle
	float theta = axisNormalised.val[3], norm = theta;
	// Norm first derivative
	float dnorm;
	// dsin(theta), where theta is a function of axisAngle
	float dsin;

	if ((xk >= X3) && (xk <= X5))
	{
		Dskew_r = skewMatFirstDerivative(static_cast<int>(xk) - 3);
		dnorm = normFirstDerivative(axisAngle, static_cast<int>(xk) - 3);
		dsin = sinFirstDerivative(axisAngle, static_cast<int>(xk) - 3);
	}
	else
	{
		cout << "Not valid axis angle parameter." << endl;
		return Mat();
	}

	// The derivative of the first term as state above is
	// the derivative of the product of two subsequent terms
	Mat Dproduct1 = sin(theta) * ((Dskew_r * norm - skewMat(axisAngle) * dnorm) / axisAngle.dot(axisAngle));
	Mat Dproduct2 = skew_r * dsin;

	return (Dproduct1 + Dproduct2); // First term construction
}

// Axis angle rotation matrix partial derivative - second term ([normalised(r)]x^2 * (1 - cos(theta)))
Mat axisAngleFirstDerivative_term2(Vec3f axisAngle, Parameter xk)
{
	Vec4f axisNormalised = axisAngleConversion(axisAngle);
	// Normalised axis skew symmetric matrix
	Mat skew_r = skewMat(Vec3f(axisNormalised.val[0], axisNormalised.val[1], axisNormalised.val[2]));
	// Normalised axis skew symmetric matrix first derivative
	Mat Dskew_rSqrd; 
	// Rotation angle
	float theta = axisNormalised.val[3], norm = theta;
	// Dot product derivative
	float ddot;
	// dcos(theta), where theta is a function of axisAngle
	float dcos;

	if ((xk >= X3) && (xk <= X5))
	{
		Dskew_rSqrd = skewMatSqrdFirstDerivative(axisAngle, static_cast<int>(xk) - 3);
		ddot = dotProductFirstDerivative(axisAngle, static_cast<int>(xk) - 3);
		dcos = cosFirstDerivative(axisAngle, static_cast<int>(xk) - 3);
	}
	else
	{
		cout << "Not valid axis angle parameter." << endl;
		return Mat();
	}

	// The derivative of the second term as state above is
	// the derivative of the product of two subsequent terms
	float dot = axisAngle.dot(axisAngle);
	Mat Dproduct1 = (1.f - cos(theta)) * ((Dskew_rSqrd * dot - skewMat(axisAngle) * skewMat(axisAngle) * ddot) / (dot * dot));
	Mat Dproduct2 = skew_r * skew_r * (-dcos);

	return (Dproduct1 + Dproduct2);
}
#endif

// 3 x 3 skew symmetric matrix first derivative
Mat skewMatFirstDerivative(int vi)
{
	switch (vi) // Choose one of three elements that are used to built the 3 x 3 skew symmetric matrix
	{
		case 0: // vi = v0
			return (Mat_<float>(3, 3) << 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 1.f, 0.f);
		case 1: // vi = v1
			return (Mat_<float>(3, 3) << 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f);
		case 2: // vi = v2
			return (Mat_<float>(3, 3) << 0.f, -1.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
		default:
		{
			cout << "Not valid element." << endl;
			return Mat();
		}
	}
}

// 3 x 3 skew symmetric matrix squared, first derivative
Mat skewMatSqrdFirstDerivative(Vec3f v, int vi)
{
	switch (vi) // Choose one of three elements that are used to built the 3 x 3 skew symmetric matrix
	{
	case 0: // vi = v0
		return (Mat_<float>(3, 3) << 0.f, v.val[1], v.val[2], v.val[1], -2.f*v.val[0], 0.f, v.val[2], 0.f, -2.f*v.val[0]);
	case 1: // vi = v1
		return (Mat_<float>(3, 3) << -2.f*v.val[1], v.val[0], 0.f, v.val[0], 0.f, v.val[2], 0.f, v.val[2], -2.f*v.val[1]);
	case 2: // vi = v2
		return (Mat_<float>(3, 3) << -2.f*v.val[2], 0.f, v.val[0], 0.f, -2.f*v.val[2], v.val[1], v.val[0], v.val[1], 0.f);
	default:
	{
		cout << "Not valid element." << endl;
		return Mat();
	}
	}
}

// First derivative of the euclidean norm of 3 x 1 vector
float normFirstDerivative(Vec3f v, int vi)
{
	float norm = norm2(v);
	// Dot product first derivative
	float ddot = dotProductFirstDerivative(v, vi);
	// Norm first derivative in respect to the dot product
	float dnorm_dot = 1.f / (2.f * norm);

	// Chain rule
	return dnorm_dot * ddot;
}

// First derivative of the dot product v . v
float dotProductFirstDerivative(Vec3f v, int vi)
{
	switch (vi) // Choose one of three elements of v
	{
		case 0: // vi = v0
			return (2 * v.val[0]);
		case 1: // vi = v1
			return (2 * v.val[1]);
		case 2: // vi = v2
			return (2 * v.val[2]);
		default:
		{
			cout << "Not valid element." << endl;
			return 0.f;
		}
	}
}

// First derivative of sin(theta), where theta is a function of v
float sinFirstDerivative(Vec3f v, int vi)
{
	float dnorm = normFirstDerivative(v, vi);
	float theta = norm2(v);

	// Chain rule
	return cos(theta) * dnorm;
}

// First derivative of sin(theta), where theta is a function of v
float cosFirstDerivative(Vec3f v, int vi)
{
	float dnorm = normFirstDerivative(v, vi);
	float theta = norm2(v);

	// Chain rule
	return -sin(theta) * dnorm;
}

// Return the basis vectors of the 3d space
Vec3f basis3DVectors(int dim)
{
	if ((dim < 0) || (dim > 2))
	{
		cout << "Invalid dimension dim:" << dim << " ;this a basis of the 3d space" << endl;
		return Vec3f(1.f, 0.f, 0.f);
	}
	Mat I = Mat::eye(3, 3, CV_32F);

	return I.at<Vec3f>(dim, 0);
}

// Calculate pose of the object, through 2D projected points
// and their corresponding 3D model points
void poseEstimation2D_3D(const vector <Point3f> *const xw, const vector <Point2f> *const xp, Mat K, 
						 Mat * const R, Vec3f * const t)
{
	Mat A, b;
	float f = K.at<float>(0, 0);
	float u0 = K.at<float>(0, 2), v0 = K.at<float>(1, 2);
	for (size_t i = 0; i < (*xw).size(); i++)
	{
		Mat rowXp = (Mat_<float>(1, 12) << f * (*xw)[i].x, f * (*xw)[i].y, f * (*xw)[i].z, 0.f, 0.f, 0.f,
			u0 * (*xw)[i].x - (*xw)[i].x * (*xp)[i].x, u0 * (*xw)[i].y - (*xw)[i].y * (*xp)[i].x,
			u0 * (*xw)[i].z - (*xw)[i].z * (*xp)[i].x, f, 0.f, u0);
									
	}

}

// Get 3D rotation representation type
int rotation3Dtype()
{
#ifdef _EULER
	return 0;
#endif

#ifdef _AXISANGLE
	return 1;
#endif
}