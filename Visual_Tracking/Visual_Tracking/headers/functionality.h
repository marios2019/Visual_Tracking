#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath> 

using namespace cv;
using namespace std;

#include "Cuboid3D.h"
#include "Camera.h"

// Virtual Camera default extrinsics parameters
#define TX -28
#define TY 30
#define TZ 80
#define RX 160
#define RY 30
#define RZ 0

//#define F_DEBUG

// Main functions declarations
void visualTracker(Cuboid3D&, Cuboid3D&, int, int);
// Read model data from .x file
void modelData(float&, float&, float&);
// Render function data
Cuboid2D rendering(Cuboid3D&, Camera, Mat&, Mat&, string);
// Render function model
Cuboid2D rendering(Cuboid3D&, Camera, Mat&, Mat&, vector <Distance>&, string);
// Dissimilarity between data and model object
void dissimilarity(Cuboid2D&, Camera, Mat&, Mat, vector <Distance>&);

// Secondary functions declarations
// Draw object on image planes
void drawObj(Cuboid2D, Mat&, Mat&, string);
// Object clipping
void clipping(Cuboid2D&, int, int);
// Check if a surface is being occluded by another one
int occlusionSurf(vector <int>, vector <Point3f>, Point3f);
// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f);
// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f);
// Compute euclidean distance between two points
float euclideanDistanceSquared(Point2i, Point2i);
// Keys pressed handler - case sensitive
void keyboardHandler(Camera&, Camera&, bool&, bool&, bool&);

// Main functions definitions
// Visual tracker main function
void visualTracker(Cuboid3D& model, Cuboid3D& Data, int WIDTH, int HEIGHT)
{
	bool exitFlag = false; // If true, exit application
	bool updateFlag = true; // If true render again cuboids
	bool fitFlag = false; // If true start non linear fitting
	int iterations = 0; // Number of non linear fitting iterations
	float squareDiff = 0.f; // Square difference between xi+1 and xi state parameters
	vector <State> state = { X, Y, Z, THETAX, THETAY };
	Mat Jglobal, Eglobal;

	// Image plane WIDTHxHEIGHT pixels
	Mat imagePlane(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneModel(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneData(HEIGHT, WIDTH, CV_8UC3, CV_RGB(255, 255, 255));

	// Virtual camera initialization
	// Camera Extrinsics
	Vec3f tVirtual(TX, TY, TZ); // Camera position
	Vec3f rVirtual(RX, RY, RZ); // Rotations over x, y, z axes
	// Camera Intrinsics
	float u0Virtual = (float) WIDTH / 2, v0Virtual = (float) HEIGHT / 2; // Principal point - center of image plane
	float fovVirtual = 60.f; // F.O.V
	float focalVirtual = (WIDTH / 2) / ((float)(tan((fovVirtual / 2) * PI / 180.0))); // Focal length
	Camera virtualCam(tVirtual, rVirtual, Point2f(u0Virtual, v0Virtual), fovVirtual, focalVirtual, state.size());
	
	// Real camera initialization
	// Camera Extrinsics
	Vec3f tReal(TX + 2, TY, TZ); // Camera position
	Vec3f rReal(RX, RY, RZ); // Rotations over x, y, z axes
	// Camera Intrinsics
	float u0Real = (float) WIDTH / 2, v0Real = (float) HEIGHT / 2; // Principal point - center of image plane
	float fovReal = 60.f; // F.O.V
	float focalReal = (WIDTH / 2) / ((float)(tan((fovReal / 2) * PI / 180.0))); // Focal length
	Camera realCam(tReal, rReal, Point2f(u0Real, v0Real), fovReal, focalReal, state.size());
	
	while (!exitFlag)
	{
		if (updateFlag)
		{
			// Render model
			vector <Distance> distances;
			for (int i = 0; i < virtualCam.getParamsNum(); i++)
			{
				distances.push_back(Distance());
			}
			Cuboid2D modelProjection(rendering(model, virtualCam, imagePlane, imagePlaneModel, distances, "model"));
			// Render data
			Cuboid2D dataProjection(rendering(Data, realCam, imagePlane, imagePlaneData, "data"));
			// Disimillarity between data and model object
			dissimilarity(modelProjection, virtualCam, imagePlane, imagePlaneData, distances);

			// Display camera parameters
			Mat virtualValues(70, 400, CV_8UC3, Scalar::all(255));
			Mat realValues(70, 400, CV_8UC3, Scalar::all(255));
			// Virtual Camaera Parameters
			string values = "Tx: " + to_string(virtualCam.getTx()) + " Ty: " + to_string(virtualCam.getTy()) + " Tz: " + to_string(virtualCam.getTz());
			putText(virtualValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
			values.clear();
			values = "Rx: " + to_string(virtualCam.getThetaX(DEGREES)) + " Ry: " + to_string(-virtualCam.getThetaY(DEGREES)) + " Rz: " + to_string(-virtualCam.getThetaZ(DEGREES));
			putText(virtualValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
			values.clear();
			// Real Camaera Parameters
			values = "Tx: " + to_string(realCam.getTx()) + " Ty: " + to_string(realCam.getTy()) + " Tz: " + to_string(realCam.getTz());
			putText(realValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
			values.clear();
			values = "Rx: " + to_string(realCam.getThetaX(DEGREES)) + " Ry: " + to_string(-realCam.getThetaY(DEGREES)) + " Rz: " + to_string(-realCam.getThetaZ(DEGREES));
			putText(realValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
			// Show camera parameters
			const char* virtualCamParams = "Virtual Camera Parameters";
			const char* realCamParams = "Real Camera Parameters";
			imshow(virtualCamParams, virtualValues);
			imshow(realCamParams, realValues);
			waitKey(1);

			// Errors vector
			vector <float> errors;
			errors = distances[0].getErrors();
			Mat E = Mat((int)errors.size(), 1, CV_32FC1);
			memcpy(E.data, errors.data(), errors.size() * sizeof(float));
			E.copyTo(Eglobal);
			// Jacobian of errors vector for each state parameter
			vector <vector <float>> Jacobian;
			for (int i = 0; i < distances.size(); i++)
			{
				Jacobian.push_back(distances[i].getErrorsDeriv());
			}
			Mat J = Mat((int)Jacobian[0].size(), (int)Jacobian.size(), CV_32FC1);
			for (int i = 0; i < Jacobian.size(); i++)
			{
				Mat Ji = Mat((int)Jacobian[i].size(), 1, CV_32FC1);
				memcpy(Ji.data, Jacobian[i].data(), Jacobian[i].size() * sizeof(float));
				Ji.col(0).copyTo(J.col(i));
			}
			J.copyTo(Jglobal);
			cout << "Dissimilarity between model and data object: " << E.dot(E) << endl;
		}
		// Non linear fitting - Gauss Newton
		if (fitFlag)
		{
			vector <float> tmp;
			tmp = virtualCam.getParams(state);
			Mat x(virtualCam.getParamsNum(), 1, CV_32FC1);
			Mat xNew(virtualCam.getParamsNum(), 1, CV_32FC1);
			Mat Jinv;
			Mat Jt;
			memcpy(x.data, tmp.data(), tmp.size() * sizeof(float));
			transpose(Jglobal, Jt);
			invert(Jt*Jglobal, Jinv);
			Mat Jgn = Jinv * Jt;
			xNew = x - Jgn * Eglobal;
			vector <float> xi;
			for (int i = 0; i < virtualCam.getParamsNum(); i++)
			{
				xi.push_back(xNew.at<float>(i));
			}
			virtualCam.setParams(xi, state);
			++iterations;
			squareDiff = (float) abs(xNew.dot(xNew) - x.dot(x));
			// Iterations limit
			if (iterations == 100)
			{
				cout << "Number of iterations: " << iterations << ", state parameters: ";
				for (int i = 0; i < virtualCam.getParamsNum(); i++)
				{
					cout << " x" << i << ": " << virtualCam.getParams(state)[i];
				}
				cout << "\nSquare difference between xi+1 and xi state parameters: " << squareDiff <<  endl;
				fitFlag = false;
				iterations = 0;
			}
		}
		else
		{
			// Keys pressed handler
			keyboardHandler(virtualCam, realCam, exitFlag, updateFlag, fitFlag);
		}
	}
}

// Read model data from .x file
void modelData(float &length, float &height, float &width)
{
	
	ifstream file("cuboid.x");
	// Check if input is empty or the file is empty
	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
#ifdef F_DEBUG
		cout << "Provide input images file with .x extension and " << endl;
		cout << "make sure it's not empty: ";
#endif
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	string line;
	// Read Cuboid3D dimensions from .x file
	while (getline(file, line))
	{
		size_t prev = 0;
		size_t next = 0;
		vector <String> data;

		while ((next = line.find_first_of(" ", prev)) != string::npos)
		{
			if (next - prev != 0)
			{
				data.push_back(line.substr(prev, next - prev));
			}
			prev = next + 1;
		}
		if (prev < line.size())
		{
			data.push_back(line.substr(prev));
		}

		//Extract Vertices
		if (!(data[0].compare("length")))
		{
			length = stof(data[1]);
		}
		else if (!(data[0].compare("height")))
		{
			height = stof(data[1]);
		}
		else if (!(data[0].compare("width")))
		{
			width = stof(data[1]);
		}
	}
}

// Render function data
Cuboid2D rendering(Cuboid3D &Cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	// Initialization
	Mat tmp(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	tmp.copyTo(imagePlaneObj);
	vector <int>::iterator iter1, iter2;
	vector <int> edge, checkEdge;
	vector <Point3f> homogeneousVertices;
	vector <int> surface;
	vector <Point2f> pointsPxl;
	for (int i = 0; i < Cuboid3D.getEdgesSize(); i++)
	{
		Cuboid3D.setEdgeVisibility(i, false);
	}
	for (int i = 0; i < Cuboid3D.getSurfacesSize(); i++)
	{
		Cuboid3D.setSurfaceVisibility(i, false);
	}

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics();
#ifdef F_DEBUG
	cout << "Camera Position: \n t = " << camera.getPosition() << endl;
	cout << "\nRotation angles: \n theta_x = " << camera.getThetaX(DEGREES) << ", theta_y = " << camera.getThetaY(DEGREES) << ", theta_z = " << camera.getThetaZ(DEGREES) << endl;
	cout << "\nCamera Intrinsics:  K = \n" << camera.getIntrinsics() << endl;
	cout << "\nCamera Extrinsics:  E = [ Rt | -RtT ] =\n" << camera.getExtrinsics() << endl;
	cout << "\nPerspective Projection Matrix: P =  K * E =\n" << P << endl;

	// Project vertices to image plane
	cout << "\nGlobal coordinates <---> Pixel coordinates: " << endl;
	cout << "-------------------------------------------" << endl;
#endif
	vector <float> homogeneous;
	for (int i = 0; i < Cuboid3D.getVerticesSize(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		homogeneous.push_back(Cuboid3D.getVertice(i).x);
		homogeneous.push_back(Cuboid3D.getVertice(i).y);
		homogeneous.push_back(Cuboid3D.getVertice(i).z);
		homogeneous.push_back(1.f);

		// Perspective projection
		Mat tmp = P * Mat(homogeneous, false);
		Point3f projection(tmp);
		homogeneousVertices.push_back(projection);
		homogeneous.clear();
	}

	// Create cuboid2d - projection of cuboid3d
	Cuboid2D objProjection(homogeneousVertices);
	homogeneousVertices.clear();
#ifdef F_DEBUG
	for (int i = 0; i < Cuboid3D.getVerticesSize(); i++)
	{
		cout << Cuboid3D.getVertice(i) << "                " << objProjection.getVerticePxl(i) << endl;
	}
#endif
	// Surface occlusion
#ifdef F_DEBUG
	cout << "\nAngle between camera direction and surface's normal:" << endl;
	cout << "------------------------------------------------------" << endl;
#endif
	for (int i = 0; i < Cuboid3D.getSurfacesSize(); i++)
	{
#ifdef F_DEBUG
		cout << "Surface " << i << " -->";
#endif
		// Check if surface is not occluded
		if (!occlusionSurf(Cuboid3D.getSurface(i), Cuboid3D.getVertices(), camera.getPosition()))
		{
			// Surface i is visible
			Cuboid3D.setSurfaceVisibility(i, true);
			surface = Cuboid3D.getSurface(i);
			objProjection.setSurface(surface);
			for (int j = 0; j < surface.size(); j++)
			{
				// Egde j is visible
				edge.push_back(surface[j]);
				edge.push_back(surface[(j + 1) % surface.size()]);
				for (int k = 0; k < Cuboid3D.getEdgesSize(); k++)
				{
					if (!(Cuboid3D.getEdgeVisibility(k)))
					{
						checkEdge = Cuboid3D.getEdge(k);
						iter1 = find(checkEdge.begin(), checkEdge.end(), edge[0]);
						iter2 = find(checkEdge.begin(), checkEdge.end(), edge[1]);
						if ((iter1 != checkEdge.end()) && (iter2 != checkEdge.end()))
						{
							Cuboid3D.setEdgeVisibility(k, true);
							pointsPxl.push_back(objProjection.getVertexPxl(edge[0]));
							pointsPxl.push_back(objProjection.getVertexPxl(edge[1]));
							objProjection.setEdge(pointsPxl);
							checkEdge.clear();
							pointsPxl.clear();
							break;
						}
						checkEdge.clear();
					}
				}
				edge.clear();
			}
			surface.clear();
		}
	}

	// Object clipping
	clipping(objProjection, imagePlane.rows, imagePlane.cols);

	// Render object
	drawObj(objProjection, imagePlane, imagePlaneObj, type);

	// Display image plane object
	string windowName = "Image Plane";
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);

	// Display image plane object
	windowName = "Cuboid3D " + type;
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlaneObj);
#ifdef F_DEBUG
	cout << endl;
#endif
	return objProjection;
}

// Render function model
Cuboid2D rendering(Cuboid3D &Cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, vector <Distance> &distances, string type)
{
	// Initialization
	Mat tmp(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	tmp.copyTo(imagePlaneObj);
	tmp.copyTo(imagePlane);
	vector <int>::iterator iter1, iter2;
	vector <int> edge, checkEdge;
	vector <Point3f> homogeneousVertices;
	vector <int> surface;
	vector <Point2f> pointsPxl;
	for (int i = 0; i < Cuboid3D.getEdgesSize(); i++)
	{
		Cuboid3D.setEdgeVisibility(i, false);
	}
	for (int i = 0; i < Cuboid3D.getSurfacesSize(); i++)
	{
		Cuboid3D.setSurfaceVisibility(i, false);
	}
	
	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics();
#ifdef F_DEBUG
	cout << "Camera Position: \n t = " << camera.getPosition() << endl;
	cout << "\nRotation angles: \n theta_x = " << camera.getThetaX(DEGREES) << ", theta_y = " << camera.getThetaY(DEGREES) << ", theta_z = " << camera.getThetaZ(DEGREES) << endl;
	cout << "\nCamera Intrinsics:  K = \n" << camera.getIntrinsics() << endl;
	cout << "\nCamera Extrinsics:  E = [ Rt | -RtT ] =\n" << camera.getExtrinsics() << endl;
	cout << "\nPerspective Projection Matrix: P =  K * E =\n" << P << endl;

	// Project vertices to image plane
	cout << "\nGlobal coordinates <---> Pixel coordinates: " << endl;
	cout << "-------------------------------------------" << endl;
#endif
	vector <float> homogeneous;
	for (int i = 0; i < Cuboid3D.getVerticesSize(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		homogeneous.push_back(Cuboid3D.getVertice(i).x);
		homogeneous.push_back(Cuboid3D.getVertice(i).y);
		homogeneous.push_back(Cuboid3D.getVertice(i).z);
		homogeneous.push_back(1.f);

		// Perspective projection
		Mat tmp = P * Mat(homogeneous, false);
		Point3f projection(tmp);
		homogeneousVertices.push_back(projection);
		// Perspective projection derivative
		for (int j = 0; j < camera.getParamsNum(); j++)
		{
			// Perspective projection derivative matrix
			Mat dP = camera.getIntrinsics() * camera.getExtrinsicsDerivative(j);
			Mat tmpDeriv = dP * Mat(homogeneous, false);
			Point3f homogeneousDeriv(tmpDeriv);
			distances[j].setVerticeDeriv(homogeneousDeriv, projection);
		}
		homogeneous.clear();
	}

	// Create cuboid2d - projection of cuboid3d
	Cuboid2D objProjection(homogeneousVertices);
	homogeneousVertices.clear();
#ifdef F_DEBUG
	for (int i = 0; i < Cuboid3D.getVerticesSize(); i++)
	{
		cout << Cuboid3D.getVertice(i) << "                " << objProjection.getVerticePxl(i) << endl;
	}
#endif

	// Surface occlusion
#ifdef F_DEBUG
	cout << "\nAngle between camera direction and surface's normal:" << endl;
	cout << "------------------------------------------------------" << endl;
#endif
	for (int i = 0; i < Cuboid3D.getSurfacesSize(); i++)
	{
#ifdef F_DEBUG
		cout << "Surface " << i << " -->";
#endif
		// Check if surface is not occluded
		if (!occlusionSurf(Cuboid3D.getSurface(i), Cuboid3D.getVertices(), camera.getPosition()))
		{
			// Surface i is visible
			Cuboid3D.setSurfaceVisibility(i, true);
			surface = Cuboid3D.getSurface(i);
			objProjection.setSurface(surface);
			for (int j = 0; j < surface.size(); j++)
			{
				// Egde j is visible
				edge.push_back(surface[j]);
				edge.push_back(surface[(j + 1) % surface.size()]);
				for (int k = 0; k < Cuboid3D.getEdgesSize(); k++)
				{
					if (!(Cuboid3D.getEdgeVisibility(k)))
					{
						checkEdge = Cuboid3D.getEdge(k);
						iter1 = find(checkEdge.begin(), checkEdge.end(), edge[0]);
						iter2 = find(checkEdge.begin(), checkEdge.end(), edge[1]);
						if ((iter1 != checkEdge.end()) && (iter2 != checkEdge.end()))
						{
							Cuboid3D.setEdgeVisibility(k, true);
							pointsPxl.push_back(objProjection.getVertexPxl(edge[0]));
							pointsPxl.push_back(objProjection.getVertexPxl(edge[1]));
							objProjection.setEdge(pointsPxl);
							// Edges derivatives
							for (int s = 0; s < distances.size(); s++)
							{
								vector <Point2f> edgeDerivative;
								edgeDerivative.push_back(distances[s].getPxlDeriv(edge[0]));
								edgeDerivative.push_back(distances[s].getPxlDeriv(edge[1]));
								distances[s].setEdgeDeriv(edgeDerivative);
							}
							checkEdge.clear();
							pointsPxl.clear();
							break;
						}
						checkEdge.clear();
					}
				}
				edge.clear();
			}
			surface.clear();
		}
	}

	// Object clipping
	clipping(objProjection, imagePlane.rows, imagePlane.cols);

	// Render object
	drawObj(objProjection, imagePlane, imagePlaneObj, type);

	// Display image plane object
	string windowName = "Image Plane";
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);

	// Display image plane object
	windowName = "Cuboid3D " + type;
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlaneObj);
#ifdef F_DEBUG
	cout << endl;
#endif

	return objProjection;
}

// Dissimilarity between data and model object
void dissimilarity(Cuboid2D &model, Camera virtualCam, Mat &imagePlane, Mat imagePlaneData, vector <Distance> &distances)
{
	// Initialization
	float offsetOut, offsetIn, normVec, normNormalVec;
	float normalOffset, maxValue = 765;
	int mNum = 7;
	Vec2f vec, normalOut, normalIn, normalVec, checkNormal;
	Point2f  vi, vj, p, tmp, pN, pC, p_mi, pNormal, pCheck, intersection;
	vector <Point2f> normalLine, edge;
	Vec3b pixel;
	Rect rectangle(0, 0, imagePlane.cols, imagePlane.rows);
	int length;

	// Set mijNum = mNum - 2
	for (int i = 0; i < distances.size(); i++)
	{
		distances[i].setIntervalNum(mNum - 2);
	}

	for (int i = 0; i < model.getEdgesSize(); i++)
	{

#ifdef F_DEBUG
		if (i == 0)
		{

			cout << "Edge segmentation and normals" << endl;
			cout << "-----------------" << endl;
		}
#endif

		// Check if edge is visible
		edge = model.getEdge(i);
		// Compute equal distance points on each edge
#ifdef F_DEBUG
		cout << "Edge " << edge[0] << " --> " << edge[1];
#endif
		vi = edge[0];
		vj = edge[1];
		vec = vj - vi;
		normVec = norm2(vec);
#ifdef F_DEBUG
		cout << " edge length = " << normVec;
#endif
		if (normVec > 0.f)
		{
			// In and Out normal for each edge
			normalOut.val[0] = -(vj.y - vi.y);
			normalOut.val[1] = vj.x - vi.x;
			normalOut = normalOut / max(norm2(normalOut), 0.0001f);
			normalIn.val[0] = -(vi.y - vj.y);
			normalIn.val[1] = vi.x - vj.x;
			normalIn = normalIn / max(norm2(normalIn), 0.0001f);
#ifdef F_DEBUG
			cout << ", normalOut: " << normalOut << ", normalIn: " << normalIn << ", mi's:" << endl;
#endif
			// For each mi on vivj edge compute normal vector and distance from data object
			for (int j = 1; j < (mNum - 1); j++)
			{
				offsetOut = 10.f;
				offsetIn = 10.f;
				// Calculate mi on edge vivj
				p = vec * ((float)j / (float)(mNum - 1));
				p = vi + p;
				p_mi = p;
#ifdef F_DEBUG
				cout << p_mi << " ";
#endif
				// Draw Out normal from current mi
				tmp.x = p_mi.x + offsetOut * normalOut.val[0];
				tmp.y = p_mi.y + offsetOut * normalOut.val[1];
				pNormal = tmp;
				// Edge near image plane
				if (!(pNormal.inside(rectangle)))
				{
					checkNormal = (Point2f)(pNormal - p_mi);
					normNormalVec = norm2(checkNormal);
					length = (int)normNormalVec;
					if (length > 0)
					{
						checkNormal = checkNormal / normNormalVec;
						// Check if the whole egde is outside the image plane
						for (int k = 0; k <= length; k++)
						{
							pC = checkNormal * ((float)k / (float)(length)) * normNormalVec;
							pC = (Point2f)p_mi + pC;
							pCheck = pC;
							if (pCheck.inside(rectangle))
							{
								pNormal = pCheck;
								offsetOut = (float) k;
								break;
							}
						}
					}
					else
					{
						pNormal = p_mi;
					}
				}
				normalLine.push_back(pNormal);
				arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);
				
				// Draw In normal from current mi
				tmp.x = p_mi.x + offsetIn * normalIn.val[0];
				tmp.y = p_mi.y + offsetIn * normalIn.val[1];
				pNormal = tmp;
				// Edge near image plane
				if (!(pNormal.inside(rectangle)))
				{
					checkNormal = (Point2f)(pNormal - p_mi);
					normNormalVec = norm2(checkNormal);
					length = (int)normNormalVec;
					if (length > 0)
					{
						checkNormal = checkNormal / normNormalVec;
						// Check if the whole egde is outside the image plane
						for (int k = 0; k <= length; k++)
						{
							pC = checkNormal * ((float)k / (float)(length)) * normNormalVec;
							pC = (Point2f)p_mi + pC;
							pCheck = pC;
							if (pCheck.inside(rectangle))
							{
								pNormal = pCheck;
								offsetIn = (float) k;
								break;
							}
						}
					}
					else
					{
						pNormal = p_mi;
					}
				}
				normalLine.push_back(pNormal);
				arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);
				circle(imagePlane, p_mi, 2, CV_RGB(0, 0, 0), -1, 8, 0);

				// Calculate edge distance between model and data object
				normalOffset = offsetOut + offsetIn;
				normalVec = normalLine[1] - normalLine[0];
				vector <float> pixelValue, weights;
				vector <Point2f> sijPoints;
				for (int k = 0; k <= normalOffset; k++)
				{
					pN = normalVec * ((float)k / (float)(normalOffset));
					pN = normalLine[0] + pN;
					pNormal = pN;
					// Find the pixel with the smallest value
					pixel = imagePlaneData.at<Vec3b>(pNormal);
					pixelValue.push_back((float) pixel.val[0] + pixel.val[1] + pixel.val[2]);
					if (pixelValue.back() < maxValue)
					{
						weights.push_back(pixelValue.back() / 765);
						sijPoints.push_back(pNormal);
					}
				}
				// Calculate euclidean distance between model and data edge
				if (sijPoints.size() > 0)
				{
					// Find exact sij point on the data edge - weighted average
					float weightsSum = 0.f;
					for (int k = 0; k < sijPoints.size(); k++)
					{
						intersection += (1 - weights[k]) * sijPoints[k];
						weightsSum += (1 - weights[k]);
					}
					intersection.x = intersection.x / weightsSum;
					intersection.y = intersection.y / weightsSum;
					circle(imagePlane, intersection, 2, CV_RGB(255, 0, 0), -1, CV_AA, 0);
					// Assign mij, calculate mij derivatives, assign dij and calculate dij derivatives
					for (int s = 0; s < (int)distances.size(); s++)
					{
						distances[s].setInterval(p_mi);
						Point2f dp_mi, dsij = Point2f(0.f, 0.f);
						Vec2f dvec, dij, ddij;
						dvec = distances[s].getEdgeDeriv(i)[1] - distances[s].getEdgeDeriv(i)[0];
						dp_mi = ((float)j / (float)(mNum - 1)) * dvec;
						dp_mi += distances[s].getEdgeDeriv(i)[0];
						distances[s].setIntervalDeriv(dp_mi);
						dij = (Point2f) (intersection - p_mi);
						ddij = dsij - dp_mi;
						distances[s].setError(dij);
						distances[s].setErrorDeriv(ddij);
					}
				}
				else
				{
					// Assign mij, calculate mij derivatives, assign dij and calculate dij derivatives
					for (int s = 0; s < distances.size(); s++)
					{
						distances[s].setInterval(p_mi);
						Point2f dp_mi, dsij = Point2f(0.f, 0.f);
						Vec2f dvec, dij, ddij;
						dvec = distances[s].getEdgeDeriv(i)[1] - distances[s].getEdgeDeriv(i)[0];
						dp_mi = ((float)j / (float)(mNum - 1)) * dvec;
						dp_mi += distances[s].getEdgeDeriv(i)[0];
						distances[s].setIntervalDeriv(dp_mi);
						dij.val[0] = 10.f;
						dij.val[1] = 0.f;
						ddij = dsij - dp_mi;
						distances[s].setError(dij);
						distances[s].setErrorDeriv(ddij);
					}
				}
				intersection = Point2f (0.f, 0.f);
				normalLine.clear();
			}
		}
#ifdef F_DEBUG
		cout << endl << endl;
#endif
		edge.clear();
	}

	string window_name = "Model - Data dissimilarity";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, imagePlane);
}

// Secondary functions declarations
// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
#ifdef F_DEBUG
	cout << "\nEdges to be rendered:" << endl;
	cout << "-----------------------" << endl;
#endif
	for (int i = 0; i < objProjection.getEdgesSize(); i++)
	{
		Point2i p1(objProjection.getEdge(i)[0]);
		Point2i p2(objProjection.getEdge(i)[1]);
#ifdef F_DEBUG
		cout << "Edge " << i << ": " << p1 << " --> " << p2 << endl;
#endif
		if (!(type.compare("model")))
		{
			// Draw vertices
			imagePlane.at<uchar>(p1.y, p1.x) = (0, 0, 0);
			imagePlane.at<uchar>(p2.y, p2.x) = (0, 0, 0);
			imagePlaneObj.at<uchar>(p1.y, p1.x) = (0, 0, 0);
			imagePlaneObj.at<uchar>(p2.y, p2.x) = (0, 0, 0);
			// Draw edges
			line(imagePlane, p1, p2, CV_RGB(0, 0, 0), 1, CV_AA, 0);
			line(imagePlaneObj, p1, p2, CV_RGB(0, 0, 0), 1, CV_AA, 0);
		}
		else if (!(type.compare("data")))
		{
			// Draw vertices
			imagePlane.at<uchar>(p1.y, p1.x) = (0, 0, 255);
			imagePlane.at<uchar>(p2.y, p2.x) = (0, 0, 255);
			imagePlaneObj.at<uchar>(p1.y, p1.x) = (0, 0, 255);
			imagePlaneObj.at<uchar>(p2.y, p2.x) = (0, 0, 255);
			// Draw edges
			line(imagePlane, p1, p2, CV_RGB(255, 0, 0), 1, CV_AA, 0);
			line(imagePlaneObj, p1, p2, CV_RGB(255, 0, 0), 1, CV_AA, 0);
		}
	}
}

// Object clipping
void clipping(Cuboid2D &obj, int height, int width)
{
	// Initialization
	vector <Point2f> edgePxl, pointsPxl;
	Vec2f edgeVec;
	Point2f pE, p1, p2;
	Point2i pEdge;
	Rect rectangle(0, 0, width, height);
	float normEdgeVec;
	int length;

	for (int i = ((int)obj.getEdgesSize() - 1); i >= 0; i--)
	{
		edgePxl = obj.getEdge(i);
		p1 = edgePxl[0];
		p2 = edgePxl[1];
		// Check if one of the vertices of the edge is outside the image plane
		if (!(p1.inside(rectangle)) || !(p2.inside(rectangle)))
		{
			edgeVec = p2 - p1;
			normEdgeVec = norm2(edgeVec);
			length = (int)normEdgeVec;
			if (length > 0)
			{
				edgeVec = edgeVec / normEdgeVec;
				// Check if the whole egde is outside the image plane
				if (!(p1.inside(rectangle)) && !(p2.inside(rectangle)))
				{					
					for (int j = 0; j <= length; j++)
					{
						pE = edgeVec * ((float)j / (float)(length)) * normEdgeVec;
						pE = p1 + pE;
						pEdge = pE;
						if (pEdge.inside(rectangle))
						{
							pointsPxl.push_back(pEdge);
							break;
						}
					}
					edgeVec = p1 - p2;
					edgeVec = edgeVec / normEdgeVec;
					for (int j = 0; j <= length; j++)
					{
						pE = edgeVec * ((float)j / (float)(length)) * normEdgeVec;
						pE = p2 + pE;
						pEdge = pE;
						if (pEdge.inside(rectangle))
						{
							pointsPxl.push_back(pEdge);
							break;
						}
					}
					if (pointsPxl.empty())
					{
						obj.destroyEdge(i);
					}
					else
					{
						obj.setEdge(pointsPxl, i);
					}
				}
				// Check if p1 edge's vertice is outside the image plane
				else if (!(p1.inside(rectangle)))
				{
					for (int j = 0; j <= length; j++)
					{
						pE = edgeVec * ((float)j / (float)(length)) * normEdgeVec;
						pE = p1 + pE;
						pEdge = pE;
						if (pEdge.inside(rectangle))
						{
							pointsPxl.push_back(pEdge);
							pointsPxl.push_back((Point2i)p2);
							break;
						}
					}
					obj.setEdge(pointsPxl, i);
				}
				// Check if p2 edge's vertice is outside the image plane
				else if (!(p2.inside(rectangle)))
				{
					edgeVec = p1 - p2;
					edgeVec = edgeVec / normEdgeVec;
					for (int j = 0; j <= length; j++)
					{
						pE = edgeVec * ((float)j / (float)(length)) * normEdgeVec;
						pE = p2 + pE;
						pEdge = pE;
						if (pEdge.inside(rectangle))
						{
							pointsPxl.push_back((Point2i)p1);
							pointsPxl.push_back(pEdge);
							break;
						}
					}
					obj.setEdge(pointsPxl, i);
				}
				pointsPxl.clear();
			}
			else
			{
				obj.destroyEdge(i);
			}
			edgePxl.clear();
		}
		
	}
		
}

// Check if a surface is being occluded by another one
int occlusionSurf(vector <int> surface, vector <Point3f> vertices, Point3f t)
{
	// Vectors begining from down right vertex of surface
	Vec3f vec1 = vertices[surface[0]] - vertices[surface[1]];
	Vec3f vec2 = vertices[surface[2]] - vertices[surface[1]];
	// Normal vector of surface
	Vec3f normal = vec2.cross(vec1);
	normal = normal / norm2(normal);
	// Vector between camera position and down right vertex of surface
	Vec3f look_vector = t - vertices[surface[1]];
	look_vector = look_vector / norm2(look_vector);
	// Angle between look vector and normal of surface
	float theta = acos(normal.dot(look_vector)) * 180.f / PI;

#ifdef F_DEBUG
	cout << " Angle: " << theta;
#endif

	// If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta < 90.f)
	{
#ifdef F_DEBUG
		cout << " --> Visible" << endl;
#endif
		return 0;
	}
	else
	{
#ifdef F_DEBUG
		cout << " --> Occluded" << endl;
#endif
		return 1;
	}
}

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

// Compute euclidean distance between two points
float euclideanDistanceSquared(Point2i p, Point2i q)
{
	return (powf((float)(q.x - p.x), 2.f) + powf((float)(q.y - p.y), 2.f));
}

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Camera &realCam, bool &exitFlag, bool &updateFlag, bool &fitFlag)
{

	switch (waitKey(0))
	{
		// Virtual Camera handler
		case 119: // 'w' key pressed - tyVirtual++
		{
			float tyVirtual = virtualCam.getTy();
			virtualCam.setTy(++tyVirtual);
			updateFlag = true;
			break;
		}
		case 115: // 's' key pressed - tyVirtual--
		{
			float tyVirtual = virtualCam.getTy();
			virtualCam.setTy(--tyVirtual);
			updateFlag = true;
			break;
		}
		case 97: // 'a' key pressed - txVirtual--
		{
			float txVirtual = virtualCam.getTx();
			virtualCam.setTx(--txVirtual);
			updateFlag = true;
			break;
		}
		case 100: // 'd' key pressed - txVirtual++
		{
			float txVirtual = virtualCam.getTx();
			virtualCam.setTx(++txVirtual);
			updateFlag = true;
			break;
		}
		case 101: // 'e' key pressed - tzVirtual++
		{
			float tzVirtual = virtualCam.getTz();
			virtualCam.setTz(++tzVirtual);
			updateFlag = true;
			break;
		}
		case 113: // 'q' key pressed - tzVirtual--
		{
			float tzVirtual = virtualCam.getTz();
			virtualCam.setTz(--tzVirtual);
			updateFlag = true;
			break;
		}
		case 122: // 'z' key pressed - ++ryVirtual
		{
			float ryVirtual = virtualCam.getThetaY(DEGREES);
			ryVirtual += 0.5f;
			virtualCam.setThetaY(ryVirtual);
			updateFlag = true;
			break;
		}
		case 120: // 'x' key pressed - --ryVirtual
		{
			float ryVirtual = virtualCam.getThetaY(DEGREES);
			ryVirtual -= 0.5f;
			virtualCam.setThetaY(ryVirtual);
			updateFlag = true;
			break;
		}
		case 99: // 'c' key pressed - ++rxVirtual
		{
			float rxVirtual = virtualCam.getThetaX(DEGREES);
			rxVirtual += 0.5f;
			virtualCam.setThetaX(rxVirtual);
			updateFlag = true;
			break;
		}
		case 118: // 'v' key pressed - --rxVirtual
		{
			float rxVirtual = virtualCam.getThetaX(DEGREES);
			rxVirtual -= 0.5f;
			virtualCam.setThetaX(rxVirtual);
			updateFlag = true;
			break;
		}
		case 98: // 'b' key pressed - ++rzVirtual
		{
			float rzVirtual = virtualCam.getThetaZ(DEGREES);
			virtualCam.setThetaZ(++rzVirtual);
			updateFlag = true;
			break;
		}
		case 110: // 'n' key pressed - --rzVirtual
		{
			float rzVirtual = virtualCam.getThetaZ(DEGREES);
			virtualCam.setThetaZ(--rzVirtual);
			updateFlag = true;
			break;
		}
		// Real Camera handler
		case 91: // '[{' key pressed - tyReal++
		{
			float tyReal = realCam.getTy();
			realCam.setTy(++tyReal);
			updateFlag = true;
			break;
		}
		case 39: // '"'' key pressed - tyReal--
		{
			float tyReal = realCam.getTy();
			realCam.setTy(--tyReal);
			updateFlag = true;
			break;
		}
		case 59: // ';:' key pressed - txReal--
		{
			float txReal = realCam.getTx();
			realCam.setTx(--txReal);
			updateFlag = true;
			break;
		}
		case 92: // '\|' key pressed - txReal++
		{
			float txReal = realCam.getTx();
			realCam.setTx(++txReal);
			updateFlag = true;
			break;
		}
		case 93: // ']}' key pressed - tzReal++
		{
			float tzReal = realCam.getTz();
			realCam.setTz(++tzReal);
			updateFlag = true;
			break;
		}
		case 112: // 'p' key pressed - tzReal--
		{
			float tzReal = realCam.getTz();
			realCam.setTz(--tzReal);
			updateFlag = true;
			break;
		}
		case 102: // 'f' key pressed - ++ryReal
		{
			float ryReal = realCam.getThetaY(DEGREES);
			realCam.setThetaY(++ryReal);
			updateFlag = true;
			break;
		}
		case 103: // 'g' key pressed - --ryReal
		{
			float ryReal = realCam.getThetaY(DEGREES);
			realCam.setThetaY(--ryReal);
			updateFlag = true;
			break;
		}
		case 104: // 'h' key pressed - ++rxReal
		{
			float rxReal = realCam.getThetaX(DEGREES);
			realCam.setThetaX(++rxReal);
			updateFlag = true;
			break;
		}
		case 106: // 'j' key pressed - --rxReal
		{
			float rxReal = realCam.getThetaX(DEGREES);
			realCam.setThetaX(--rxReal);
			updateFlag = true;
			break;
		}
		case 107: // 'k' key pressed - ++rzReal
		{
			float rzReal = realCam.getThetaZ(DEGREES);
			realCam.setThetaZ(++rzReal);
			updateFlag = true;
			break;
		}
		case 108: // 'l' key pressed - --rzReal
		{
			float rzReal = realCam.getThetaZ(DEGREES);
			realCam.setThetaZ(--rzReal);
			updateFlag = true;
			break;
		}
		case 27: // ESC key pressed - exit
		{
			exitFlag = true;
			break;
		}
		case 13: // ENTER key pressed - start fitting
		{
			fitFlag = true;
			updateFlag = true;
			break;
		}
		case 32: // SPACE key pressed - set camera to default extrinsic parameters
		{
			vector <float> params = { TX, TY, TZ, RX, RY, RZ };
			vector <State> state = { X, Y, Z, THETAX, THETAY, THETAZ };
			virtualCam.setParams(params, state);
			params[0] += 2.f;
			realCam.setParams(params, state);
			updateFlag = true;
			break;
		}
		default:
		{
			updateFlag = false;
			break;
		}
	}
}