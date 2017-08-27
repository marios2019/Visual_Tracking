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
	return sqrt(powf(vec.val[0], 2.f) + powf(vec.val[1], 2.f));
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
// Compute the Jacobian matrix of the input vector v that holds 3D homogeneous
// points that are , 
// according to the input vector x that holds the state parameters of the