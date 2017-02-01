#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <math.h> 

// Image plane dimensions
#define WIDTH 400
#define HEIGHT 300

#define PI 3.14159265f

using namespace cv;
using namespace std;

// Render function
void rendering(vector <Point3f>, vector <vector <int>>, vector <Point2i>&, float, float, float, float, float, float, Mat, Mat &, vector <bool>&, string);
// Check if a surface is being occluded by another one
int occlusion_surf(vector <int>, vector <Point3f>, Point3f);
// Returns the Euclidean norm of a 3D vector
float norm2(Vec3f);
// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f);
// Dissimilarity between data and model object
int dissimilarity(vector <Point2i>, vector <Point2i>, vector <vector <int>>, vector <vector <int>>, vector <bool>, vector <vector <Point2i>>&, Mat);
// Read model data from .x file
void model_data(int, char**, vector <Point3f>&, vector <vector <int>>&, vector <vector <int>>&);

int main(int argc, char** argv)
{
	// Initialization local coordinates
	vector <Point3f> vertices;
	vector <vector <int>> edges;
	vector <vector <int>> surfaces;
	// Read from .x file
	model_data(argc, argv, vertices, edges, surfaces);
	vector <bool> modelVisible;
	vector <bool> dataVisible;
	// Image plane 300x400 pixels
	Mat image_plane(HEIGHT, WIDTH, CV_8UC3, Scalar::all(255));

	// Camera Extrinsics
	// Camera position
	float t_x = 0.f;
	float t_y = 0.f;
	float t_z = 80.f;
	// Rotation x-axis
	float theta_x = 180.f * PI / 180.f;
	// Rotation y-axis
	float theta_y = 0.f * PI / 180.f;
	// Rotation z-axis
	float theta_z = 0.f * PI / 180.f;
	// Camera Intrinsics
	// F.O.V
	float fov = 60.f;
	// Principal point - center of image plane
	float u_0 = WIDTH / 2;
	float v_0 = HEIGHT / 2;
	// Focal length
	float f = (WIDTH / 2) / ((float) (tan((fov / 2) * PI / 180.0)));
	// Camera intrinsics Matrix
	Mat K = (Mat_<float>(3, 3) << f, 0, u_0, 0, f, v_0, 0, 0, 1);
	
	// Render model
	vector <Point2i> model_cam_vertices;
	rendering(vertices, surfaces, model_cam_vertices, theta_x, theta_y, theta_z, t_x, t_y, t_z, K, image_plane, modelVisible, "model");
	cout << endl;
	// Render data
	// Camera position
	t_x = 0.5f;
	t_y = 0.5f;
	vector <Point2i> data_cam_vertices;
	rendering(vertices, surfaces, data_cam_vertices, theta_x, theta_y, theta_z, t_x, t_y, t_z, K, image_plane, dataVisible, "data");
	cout << endl;

	// Compute dissimilarity between data and model
	vector <vector <Point2i>> mi;
	int D = dissimilarity(data_cam_vertices, model_cam_vertices, surfaces, edges, modelVisible, mi, image_plane);

	waitKey(0);
	exit(EXIT_SUCCESS);
}

// Render function
void rendering(vector <Point3f> vertices, vector <vector <int>> surfaces, vector <Point2i> &vertices_camera,float theta_x, float theta_y, float theta_z, float t_x, float t_y, float t_z, Mat K, Mat &image_plane, vector <bool> &visible, string type)
{
	// Camera calibration
	// Translation Vector;
	Vec3f t = Vec3f(t_x, t_y, t_z);
	// Rotation x-axis Matrix
	Mat R_x = (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
	// Rotation y-axis Matrix
	Mat R_y = (Mat_<float>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
	// Rotation z-axis Matrix
	Mat R_z = (Mat_<float>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
	// Rotation Matrix
	Mat R = R_z * R_y * R_x;
	// Trasnpose Rotation Matrix
	Mat Rt;
	transpose(R, Rt);
	// Trasnpose Rotation Matrix multiplied by vector T - camera position
	Mat tmp = -Rt * Mat(t);
	Vec3f RtT = Vec3f(tmp);
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << Rt.at<float>(0, 0), Rt.at<float>(0, 1), Rt.at<float>(0, 2), RtT[0], Rt.at<float>(1, 0), Rt.at<float>(1, 1), Rt.at<float>(1, 2), RtT[1], Rt.at<float>(2, 0), Rt.at<float>(2, 1), Rt.at<float>(2, 2), RtT[2]);
	// Perpective Projection Matrix
	Mat	P = K * E;

	cout << "Camera Position: \n t = " << t << endl;
	cout << "\nRotation angles: \n theta_x = " << theta_x * 180.f / PI << ", theta_y = " << theta_y * 180.f / PI << ", theta_z = " << theta_z * 180.f / PI << endl;
	cout << "\nCamera Intrinsics:  K = \n" << K << endl;
	cout << "\nCamera Extrinsics:  E = [ RtT | -RtT ] =\n" << E << endl;
	cout << "\nPerspective Projection Matrix: P =  K * E =\n" << P << endl;

	// Project vertices to image plane using perspective projection - camera coordinates
	vector <Point3f> vertices_hmgns;
	cout << "\nGlobal coordinates <---> Pixel coordinates: " << endl;
	cout << "-------------------------------------------" << endl;
	vector <float> tmp1;
	for (int i = 0; i < vertices.size(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		tmp1.push_back(vertices[i].x);
		tmp1.push_back(vertices[i].y);
		tmp1.push_back(vertices[i].z);
		tmp1.push_back(1.f);
		
		// Perspective projection
		Mat tmp2 = P * Mat(tmp1, false);
		Point3f tmp3(tmp2);
		vertices_hmgns.push_back(tmp3);

		// Pixel coordinates
		Point2f tmp4((tmp3.x / tmp3.z), (tmp3.y / tmp3.z));
		Point2i tmp5(tmp4);
		// Clip pixel coordinates if smaller than zero
		if (tmp5.x < 0)
		{
			tmp5.x = 0;
		}
		if (tmp5.y < 0)
		{
			tmp5.y = 0;
		}
		// Clip pixel coordinates if larger than image plane dimensions
		if (tmp5.x >= WIDTH)
		{
			tmp5.x = WIDTH - 1;
		}
		if (tmp5.y >= HEIGHT)
		{
			tmp5.y = HEIGHT - 1;
		}
		vertices_camera.push_back(tmp5);
		cout << vertices[i] << "                " << vertices_camera[i] << endl;
		tmp1.clear();
	}

	// Draw surfaces
	string window_name = "Cuboid " + type;
	namedWindow(window_name, WINDOW_AUTOSIZE);
	cout << "\nAngle between camera direction and surface's normal:" << endl;
	cout << "------------------------------------------------------" << endl;
	vector <int> surface;
	for (int i = 0; i < surfaces.size(); i++)
	{
		cout << "Surface " << i << " -->";
		// Draw if surface is not occluded
		if (!occlusion_surf(surfaces[i], vertices, t))
		{
			surface = surfaces[i];
			visible.push_back(true);
			for (int j = 0; j < surface.size(); j++)
			{
				// Draw vertices
				image_plane.at<uchar>(vertices_camera[surface[j]].y, vertices_camera[surface[j]].x) = 0;
				// Draw edges
				Point2i p1(vertices_camera[surface[j]]);
				Point2i p2(vertices_camera[surface[(j + 1) % 4]]);
				if (!(type.compare("model")))
				{
					line(image_plane, p1, p2, Scalar::all(0), 1, CV_AA, 0);
				}
				else if (!(type.compare("data")))
				{
					line(image_plane, p1, p2, Scalar(0, 0, 255), 1, CV_AA, 0);
				}
			}
			surface.clear();
		}
		else
		{
			visible.push_back(false);
		}
	}
	// Display model
	imshow(window_name, image_plane);
}

// Check if a surface is being occluded by another one
int occlusion_surf(vector <int> surface, vector <Point3f> vertices, Point3f t)
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
	
	cout << " Angle: " <<  theta;

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
	float norm = sqrt(vec.val[0] * vec.val[0] + vec.val[1] * vec.val[1] + vec.val[2] * vec.val[2]);

	return max(norm, 0.0001f);
}

// Returns the Euclidean norm of a 2D vector
float norm2(Vec2f vec)
{
	float norm = sqrt(vec.val[0] * vec.val[0] + vec.val[1] * vec.val[1]);

	return norm;
}

// Dissimilarity between data and model object
int dissimilarity(vector <Point2i> data, vector <Point2i> model, vector <vector <int>> surfaces, vector <vector <int>> edges, vector <bool> visibleSurf, vector <vector <Point2i>> &mi, Mat imgPlane)
{
	int D = 0, mNum = 5;
	vector <int> edge, surface;
	vector <Point2i> m;
	Vec2f vec, normalOut, normalIn;
	Point2f vi, vj, p, tmp;
	Point2i p_mi;
	bool visibleEdge[12];
	vector <Vec2f> normalsOut, normalsIn;
	vector <int>::iterator iter1, iter2;

	// Check which edges are visible
	for (int i = 0; i < 12; i++)
	{
		visibleEdge[i] = false;
		edge = edges[i];
		for (int j = 0; j < surfaces.size(); j++)
		{
			if (visibleSurf[j])
			{
				surface = surfaces[j];
				iter1 = find(surface.begin(), surface.end(), edge[0]);
				iter2 = find(surface.begin(), surface.end(), edge[1]);
				if ((iter1 != surface.end()) && (iter2 != surface.end()))
				{
					visibleEdge[i] = true;
				}
				surface.clear();
			}
		}		
		edge.clear();
	}

	for (int i = 0; i < edges.size(); i++)
	{
		if (i == 0)
		{
			cout << "Edge segmentation and normals" << endl;
			cout << "-----------------" << endl;
		}

		// Check if edge is visible
		if (visibleEdge[i])
		{
			// Compute equal distance points on each edge
			edge = edges[i];
			cout << "Edge " << edge[0] << " --> " << edge[1];
			vi = model[edge[0]];
			vj = model[edge[1]];
			vec = vj - vi;
			float norm = norm2(vec);
			cout << " edge length = " << norm;
			if (norm > 0.f)
			{
				norm = max(norm, 0.0001f);
				vec = vec / norm;
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
					p = vec * ((float)j / (float)(mNum - 1)) * norm;
					p = vi + p;
					m.push_back(p);
					cout << m[j] << " ";
					tmp.x = m[j].x + 10.f * normalOut.val[0];
					tmp.y = m[j].y + 10.f * normalOut.val[1];
					p_mi = tmp;
					line(imgPlane, m[j], p_mi, Scalar::all(0), 1, 8, 0);
					tmp.x = m[j].x + 10.f * normalIn.val[0];
					tmp.y = m[j].y + 10.f * normalIn.val[1];
					p_mi = tmp;
					line(imgPlane, m[j], p_mi, Scalar::all(0), 1, 8, 0);
					circle(imgPlane, m[j], 2, Scalar::all(0), -1, CV_AA, 0);
				}
				normalsOut.push_back(normalOut);
				normalsIn.push_back(normalIn);
				mi.push_back(m);
				m.clear();
			}
			edge.clear();
			cout << endl << endl;
		}
	}

	string window_name = "Model - Data dissimilarity";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, imgPlane);
	
	return D;
}

// Read model data from .x file
void model_data(int argc, char **argv, vector<Point3f>& vertices, vector<vector<int>>& edges, vector<vector<int>>& surfaces)
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
	// Read vertices and lines from .x file
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
		if (!(data[0].compare("vertex")))
		{
			vector <int> v;

			for (int i = 1; i < data.size(); i++)
			{
				v.push_back(stoi(data[i]));
			}

			Point3f tmp((float)v[0], (float)v[1], (float)v[2]);
			vertices.push_back(tmp);
		}
		else if (!(data[0].compare("edge")))
		{
			vector <int> tmp;
			for (int i = 1; i < data.size(); i++)
			{
				tmp.push_back(stoi(data[i]));
			}
			edges.push_back(tmp);
		}
		else if (!(data[0].compare("surface")))
		{
			vector <int> tmp;
			for (int i = 1; i < data.size(); i++)
			{
				tmp.push_back(stoi(data[i]));
			}
			surfaces.push_back(tmp);
		}
	}
}