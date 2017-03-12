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

// Main functions declarations
// Read model data from .x file
void modelData(float&, float&, float&);
// Render function
Cuboid2D rendering(Cuboid2D&, Camera, Mat&, Mat&, string);
// Dissimilarity between data and model object
float dissimilarity(Cuboid3D&, Camera, Mat&, Mat);

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

// Main functions definitions
// Read model data from .x file
void modelData(float &length, float &height, float &width)
{
	
	ifstream file("cuboid.x");
	// Check if input is empty or the file is empty
	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
		cout << "Provide input images file with .x extension and " << endl;
		cout << "make sure it's not empty: ";
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

// Render function
Cuboid2D rendering(Cuboid3D &Cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	// Initialization
	Mat tmp(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	tmp.copyTo(imagePlaneObj);
	if (!(type.compare("model")))
	{
		tmp.copyTo(imagePlane);
	}
	vector <int>::iterator iter1, iter2;
	vector <int> edge, checkEdge;
	vector <Point3f> homogeneousVertices;
	vector <int> surface;
	vector <Point2i> pointsPxl;
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

	cout << "Camera Position: \n t = " << camera.getPosition() << endl;
	cout << "\nRotation angles: \n theta_x = " << camera.getThetaX() << ", theta_y = " << camera.getThetaY() << ", theta_z = " << camera.getThetaZ() << endl;
	cout << "\nCamera Intrinsics:  K = \n" << camera.getIntrinsics() << endl;
	cout << "\nCamera Extrinsics:  E = [ Rt | -RtT ] =\n" << camera.getExtrinsics() << endl;
	cout << "\nPerspective Projection Matrix: P =  K * E =\n" << P << endl;

	// Project vertices to image plane
	cout << "\nGlobal coordinates <---> Pixel coordinates: " << endl;
	cout << "-------------------------------------------" << endl;
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
	for (int i = 0; i < Cuboid3D.getVerticesSize(); i++)
	{
		cout << Cuboid3D.getVertice(i) << "                " << objProjection.getVerticePxl(i) << endl;
	}

	// Surface occlusion
	cout << "\nAngle between camera direction and surface's normal:" << endl;
	cout << "------------------------------------------------------" << endl;
	for (int i = 0; i < Cuboid3D.getSurfacesSize(); i++)
	{
		cout << "Surface " << i << " -->";
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
							pointsPxl.push_back(objProjection.getVerticePxl(edge[0]));
							pointsPxl.push_back(objProjection.getVerticePxl(edge[1]));
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

	cout << endl;

	return objProjection;
}

// Dissimilarity between data and model object
float dissimilarity(Cuboid2D &model, Camera virtualCam, Mat &imagePlane, Mat imagePlaneData)
{
	// Initialization
	float offsetOut, offsetIn, normVec, normNormalVec;
	float D = 0.f, mNum = 7, normalNum, pixelValue, maxValue;
	vector <Point2i> edge;
	Vec2f vec, normalOut, normalIn, normalVec, checkNormal;
	Point2f vi, vj, p, tmp, pN, pC;
	Point2i p_mi, pNormal, pCheck, intersection;
	vector <Point2f> normalLine;
	Vec3b pixel;
	Rect rectangle(0, 0, imagePlane.cols, imagePlane.rows);
	int length;
	
	for (int i = 0; i < model.getEdgesSize(); i++)
	{
		if (i == 0)
		{
			cout << "Edge segmentation and normals" << endl;
			cout << "-----------------" << endl;
		}

		// Check if edge is visible
		edge = model.getEdge(i);
		// Compute equal distance points on each edge
		cout << "Edge " << edge[0] << " --> " << edge[1];
		vi = edge[0];
		vj = edge[1];
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
			for (int j = 1; j < (mNum - 1); j++)
			{
				offsetOut = 10.f;
				offsetIn = 10.f;
				// Calculate mi on edge vivj
				p = vec * ((float)j / (float)(mNum - 1)) * normVec;
				p = vi + p;
				p_mi = p;
				cout << p_mi << " ";
				// Draw Out normal from current mi
				tmp.x = p_mi.x + offsetOut * normalOut.val[0];
				tmp.y = p_mi.y + offsetOut * normalOut.val[1];
				pNormal = tmp;
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
								offsetOut = k;
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
								offsetIn = k;
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
				normalNum = offsetOut + offsetIn;
				normalVec = normalLine[1] - normalLine[0];
				normNormalVec = norm2(normalVec);
				normalVec = normalVec / normNormalVec;
				pixelValue = 0.f;
				maxValue = 765;
				for (int k = 0; k <= normalNum; k++)
				{
					pN = normalVec * ((float)k / (float)(normalNum)) * normNormalVec;
					pN = normalLine[0] + pN;
					pNormal = pN;
					// Find the pixel with the smallest value
					pixel = imagePlaneData.at<Vec3b>(pNormal);
					pixelValue = pixel.val[0] + pixel.val[1] + pixel.val[2];
					if (pixelValue < maxValue)
					{
						maxValue = pixelValue;
						intersection = pNormal;
					}
				}
				// Calculate euclidean distance between model and data edge
				if (maxValue < 765)
				{
					circle(imagePlane, intersection, 2, CV_RGB(255, 0, 0), -1, 8, 0);
					D += euclideanDistanceSquared(p_mi, intersection);
				}
				else
				{
					D += normalNum;
				}
				normalLine.clear();
			}
		}
		cout << endl << endl;
		edge.clear();
	}

	string window_name = "Model - Data dissimilarity";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, imagePlane);

	if (model.getEdgesSize() < 4)
	{
		D = 9999.f;
	}

	return D;
}

// Secondary functions declarations
// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	vector <Point2i> edgePxl;
	cout << "\nEdges to be rendered:" << endl;
	cout << "-----------------------" << endl;
	for (int i = 0; i < objProjection.getEdgesSize(); i++)
	{
		edgePxl = objProjection.getEdge(i);
		Point2i p1(edgePxl[0]);
		Point2i p2(edgePxl[1]);
		cout << "Edge " << i << ": " << p1 << " --> " << p2 << endl;
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
			line(imagePlane, p1, p2, CV_RGB(255, 0, 0), 1, 4, 0);
			line(imagePlaneObj, p1, p2, CV_RGB(255, 0, 0), 1, 4, 0);
		}
		edgePxl.clear();
	}
}

// Object clipping
void clipping(Cuboid2D &obj, int height, int width)
{
	// Initialization
	vector <Point2i> edgePxl, pointsPxl;
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

// Compute euclidean distance between two points
float euclideanDistanceSquared(Point2i p, Point2i q)
{
	return (powf((q.x - p.x), 2.f) + powf((q.y - p.y), 2.f));
}