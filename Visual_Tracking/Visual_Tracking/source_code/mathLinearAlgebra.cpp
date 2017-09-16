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
void decomposeCameraPose(Mat E, Mat& R, Vec3f& t)
{
	// Check if matrix is 3 x 4
	if (checkMatrixSize(E.size(), 3, 4))
	{
		return;
	}
	
	// Extract 3D rotation matrix
	Mat Rd = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rd = E(r).clone();
	transpose(Rd, Rd);
	if (!IsRotationMatrix(Rd))
	{
		cout << "The input matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rd.copyTo(R);

	// Extract position vector
	Mat td;
	Vec3f Rt;
	Rt.val[0] = E.at<float>(0, 3);
	Rt.val[1] = E.at<float>(1, 3);
	Rt.val[2] = E.at<float>(2, 3);
	Rt = -Rt;
	td = R * Mat(Rt);
	t = td;
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
	Mat Rd = Mat(3, 3, CV_32F);
	Rect r(0, 0, 3, 3);
	Rd = E(r).clone();
	if (!IsRotationMatrix(Rd))
	{
		cout << "The input euclidean transformation matrix doesn't contains a 3D rotation matrix at the upper left 3 x 3 region." << endl;
		return;
	}
	Rd.copyTo(R);

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
// Output: matrix Jvp = { first derivatives of pixel coordinates }
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

			// Pixel coordinate first derivative
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
//		   vector edges2DPtr = { pointers to 2D projection vertices }
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
			// Extract the vertices of the edge
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
	Mat R = rotationEuler(deg2rad(x.at<float>(3, 0)), deg2rad(x.at<float>(4, 0)), deg2rad(x.at<float>(5, 0)));

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
			return Mat();
	}
}

// Return camera rotation matrix first derivative in respect of the state parameters
Mat cameraRotationFirstDerivative(Parameter xk, Mat x)
{
	int parameter = static_cast<int>(xk);
	checkIdx("State parameters", parameter, 6);

	float thetaX = deg2rad(x.at<float>(3, 0)), thetaY = deg2rad(x.at<float>(4, 0)), thetaZ = deg2rad(x.at<float>(5, 0));

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
			return Mat();
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
			return Mat();
	}
}

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
	Mat kernel = (Mat_<float>(1, 2) << -1.f, 1.f);
	int i = 0, j = 0, dx = 0, dy = 0, *ix, *iy, height = Img.rows, width = Img.cols, iRange, jRange;

	// Choose partial derivative's direction - dimension
	switch (partial)
	{
		case Dx: // Horizontal direction
		{
			ix = &j;
			iy = &i;
			dx = 1;
			Mat column = Img.col(Img.cols - 1);
			hconcat(Img, column, Img); // Image padding with an extra column
			jRange = Img.cols - 1;
			iRange = Img.rows;
			break;
		}
		case Dy: // Vertical direction
		{
			ix = &i;
			iy = &j;
			dy = 1;
			Mat row = Img.row(Img.rows - 1); // Image padding with an extra row
			Img.push_back(row);
			jRange = Img.rows - 1;
			iRange = Img.cols;
			break;
		}
		default:
		{
			cout << "The two independent variables are x and y; available partial derivatives are dx and dy." << endl;
			system("PAUSE");
			exit(EXIT_FAILURE);
		}
	}

	// Convolution of img with the derivative kernel
	for (i = 0 ; i < iRange; i++)
	{
		for (j = 0 ; j < jRange; j++)
		{
			dImg.at<float>(*iy, *ix) = Img.at<float>(*iy, *ix) * kernel.at<float>(0, 0) + Img.at<float>(*iy + dy, *ix + dx) * kernel.at<float>(0, 1);
		}
	}

	return dImg;
}