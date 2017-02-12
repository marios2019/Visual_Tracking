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

#include "Cuboid.h"
#include "Camera.h"

// Functions declarations
// Read model data from .x file
void modelData(int, char**, float&, float&, float&);
// Render function
void rendering(Cuboid&, Camera&, Mat&, Mat&, string);
// Check if a surface is being occluded by another one
int occlusionSurf(vector <int>, vector <Point3f>, Point3f);
// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f);
// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f);
// Dissimilarity between data and model object
float dissimilarity(Cuboid&, Mat&, Mat);
// Compute euclidean distance between two points
float euclideanDistance(Point2i, Point2i);

//Functions definitions
// Read model data from .x file
void modelData(int argc, char **argv, float &length, float &height, float &width)
{
	
	// Check if input .x file has been provided
	if (argc < 2)
	{
		cout << "Provide input model file with .x extension." << endl;
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	// Check for too many arguments
	if (argc > 2)
	{
		cout << "Provide only one input file." << endl;
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	ifstream file(argv[1]);
	// Check if input is empty or the file is empty
	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
		cout << "Provide input images file with .x extension and " << endl;
		cout << "make sure it's not empty: ";
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	string line;
	// Read cuboid dimensions from .x file
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

// Render function
void rendering(Cuboid &cuboid, Camera &camera, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	// Initialization
	vector <int>::iterator iter1, iter2;
	vector <int> edge, checkEdge;

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics();

	cout << "Camera Position: \n t = " << camera.getPosition() << endl;
	cout << "\nRotation angles: \n theta_x = " << camera.getThetaX() << ", theta_y = " << camera.getThetaY() << ", theta_z = " << camera.getThetaZ() << endl;
	cout << "\nCamera Intrinsics:  K = \n" << camera.getIntrinsics() << endl;
	cout << "\nCamera Extrinsics:  E = [ Rt | -RtT ] =\n" << camera.getExtrinsics() << endl;
	cout << "\nPerspective Projection Matrix: P =  K * E =\n" << P << endl;

	// Project vertices to image plane
	cout << "\nGlobal coordinates <---> Pixel coordinates: " << endl;
	cout << "-------------------------------------------" << endl;
	vector <float> homogeneous;
	for (int i = 0; i < cuboid.getVerticesSize(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		homogeneous.push_back(cuboid.getVertice(i).x);
		homogeneous.push_back(cuboid.getVertice(i).y);
		homogeneous.push_back(cuboid.getVertice(i).z);
		homogeneous.push_back(1.f);

		// Perspective projection
		Mat tmp = P * Mat(homogeneous, false);
		Point3f projection(tmp);

		// Pixel coordinates
		Point2f tmp2((projection.x / projection.z), (projection.y / projection.z));
		Point2i pixelCoord(tmp2);
		// Clip pixel coordinates if smaller than zero
		if (pixelCoord.x < 0)
		{
			pixelCoord.x = 0;
		}
		if (pixelCoord.y < 0)
		{
			pixelCoord.y = 0;
		}
		// Clip pixel coordinates if larger than image plane dimensions
		if (pixelCoord.x >= imagePlane.cols)
		{
			pixelCoord.x = imagePlane.cols - 1;
		}
		if (pixelCoord.y >= imagePlane.rows)
		{
			pixelCoord.y = imagePlane.rows - 1;
		}
		cuboid.setVerticePxl(pixelCoord, i);
		cout << cuboid.getVertice(i) << "                " << cuboid.getVerticePxl(i) << endl;
		homogeneous.clear();
	}

	// Draw surfaces
	cout << "\nAngle between camera direction and surface's normal:" << endl;
	cout << "------------------------------------------------------" << endl;
	vector <int> surface;
	for (int i = 0; i < cuboid.getSurfacesSize(); i++)
	{
		cout << "Surface " << i << " -->";
		// Draw if surface is not occluded
		if (!occlusionSurf(cuboid.getSurface(i), cuboid.getVertices(), camera.getPosition()))
		{
			surface = cuboid.getSurface(i);
			// Surface i is visible
			cuboid.setSurfaceVisibility(i, true);
			for (int j = 0; j < surface.size(); j++)
			{
				// Egde j is visible
				edge.push_back(surface[j]);
				edge.push_back(surface[(j + 1) % surface.size()]);
				for (int k = 0; k < cuboid.getEdgesSize(); k++)
				{
					if (!(cuboid.getEdgeVisibility(k)))
					{
						checkEdge = cuboid.getEdge(k);
						iter1 = find(checkEdge.begin(), checkEdge.end(), edge[0]);
						iter2 = find(checkEdge.begin(), checkEdge.end(), edge[1]);
						if ((iter1 != checkEdge.end()) && (iter2 != checkEdge.end()))
						{
							cuboid.setEdgeVisibility(k, true);
							checkEdge.clear();
							break;
						}
						checkEdge.clear();
					}
				}
				// Draw edges
				Point2i p1(cuboid.getVerticePxl(edge[0]));
				Point2i p2(cuboid.getVerticePxl(edge[1]));
				if (!(type.compare("model")))
				{
					// Draw vertices
					imagePlane.at<uchar>(cuboid.getVerticePxl(edge[0]).y, cuboid.getVerticePxl(edge[1]).x) = (0, 0, 0);
					line(imagePlane, p1, p2, CV_RGB(0, 0, 0), 1, CV_AA, 0);
					line(imagePlaneObj, p1, p2, CV_RGB(0, 0, 0), 1, CV_AA, 0);
				}
				else if (!(type.compare("data")))
				{
					// Draw vertices
					imagePlane.at<uchar>(cuboid.getVerticePxl(edge[0]).y, cuboid.getVerticePxl(edge[1]).x) = (0, 0, 255);
					line(imagePlane, p1, p2, CV_RGB(255, 0, 0), 1, 8, 0);
					line(imagePlaneObj, p1, p2, CV_RGB(255, 0, 0), 1, 8, 0);
				}
				edge.clear();
			}
			surface.clear();
		}
	}
	// Display image plane object
	string windowName = "Image Plane";
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);

	// Display image plane object
	windowName = "Cuboid " + type;
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlaneObj);
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

	cout << " Angle: " << theta;

	// If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta < 90.f)
	{
		cout << " --> Visible" << endl;
		return 0;
	}
	else
	{
		cout << " --> Occluded" << endl;
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

// Dissimilarity between data and model object
float dissimilarity(Cuboid &model, Mat &imagePlane, Mat imagePlaneData)
{
	// Initialization
	float offset = 10.f, normVec, normNormalVec;
	float D = 0.f, mNum = 5, normalNum = 2 * offset, pixelValue, maxValue;
	vector <int> edge, surface;
	Vec2f vec, normalOut, normalIn, normalVec;
	Point2f vi, vj, p, tmp, pN;
	Point2i p_mi, pNormal, intersection;
	vector <Point2f> normalLine;
	Vec3b pixel;
		
	for (int i = 0; i < model.getEdgesSize(); i++)
	{
		if (i == 0)
		{
			cout << "Edge segmentation and normals" << endl;
			cout << "-----------------" << endl;
		}

		// Check if edge is visible
		edge = model.getEdge(i);
		if (model.getEdgeVisibility(i))
		{
			// Compute equal distance points on each edge
			cout << "Edge " << edge[0] << " --> " << edge[1];
			vi = model.getVerticePxl(edge[0]);
			vj = model.getVerticePxl(edge[1]);
			vec = vj - vi;
			normVec = norm2(vec);
			cout << " edge length = " << normVec;
			if (normVec > 0.f)
			{
				normVec = max(normVec, 0.0001f);
				vec = vec / normVec;
				// In and Out normal for each edge
				normalOut.val[0] = -(vj.y - vi.y);
				normalOut.val[1] = vj.x - vi.x;
				normalOut = normalOut / max(norm2(normalOut), 0.0001f);
				normalIn.val[0] = -(vi.y - vj.y);
				normalIn.val[1] = vi.x - vj.x;
				normalIn = normalIn / max(norm2(normalIn), 0.0001f);
				cout << ", normalOut: " << normalOut << ", normalIn: " << normalIn << ", mi's:" << endl;
				// For each mi on vivj edge compute normal vector and distance from data object
				for (int j = 0; j < mNum; j++)
				{
					// Calculate mi on edge vivj
					p = vec * ((float)j / (float)(mNum - 1)) * normVec;
					p = vi + p;
					p_mi = p;
					cout << p_mi << " ";
					// Draw Out normal from current mi
					tmp.x = p_mi.x + offset * normalOut.val[0];
					tmp.y = p_mi.y + offset * normalOut.val[1];
					pNormal = tmp;
					normalLine.push_back(pNormal);
					arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);
					// Draw In normal from current mi
					tmp.x = p_mi.x + offset * normalIn.val[0];
					tmp.y = p_mi.y + offset * normalIn.val[1];
					pNormal = tmp;
					normalLine.push_back(pNormal);
					arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);
					circle(imagePlane, p_mi, 2, CV_RGB(0, 0, 0), -1, 8, 0);

					// Calculate edge distance between model and data object
					normalVec = normalLine[1] - normalLine[0];
					normNormalVec = norm2(normalVec);
					normalVec = normalVec / normNormalVec;
					maxValue = 765;
					for (int k = 0; k < normalNum; k++)
					{
						pN = normalVec * ((float)k/ (float)(normalNum - 1)) * normNormalVec;		
						pN = normalLine[0] + pN;
						pNormal = pN;
						// Find the pixel with the smallest value
						pixel = imagePlaneData.at<Vec3b>(pNormal);
						pixelValue = pixel.val[0] + pixel.val[1] + pixel.val[2];
						if(pixelValue < maxValue)
						{
							maxValue = pixelValue;
							intersection = pNormal;
						}
					}
					// Calculate euclidean distance between model and data edge
					if (maxValue < 765)
					{
						circle(imagePlane, intersection, 2, CV_RGB(255, 0, 0), -1, 8, 0);
						D += euclideanDistance(p_mi, intersection);
					}

					normalLine.clear();
				}
			}
			cout << endl << endl;
		}
		edge.clear();
	}

	string window_name = "Model - Data dissimilarity";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, imagePlane);

	return D;
}

// Compute euclidean distance between two points
float euclideanDistance(Point2i p, Point2i q)
{
	return sqrt(powf((q.x - p.x), 2.f) + powf((q.y - p.y), 2.f));
}