#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h> 

// Image plane dimensions
#define WIDTH 400
#define HEIGHT 300

#define PI 3.14159265f

using namespace cv;
using namespace std;

// Read model data from .x file
void model_data(int, char**, vector <Point3f>&, vector <vector <int>>&);
// Render function
void rendering(vector <Point3f>&, vector <vector <int>>&, float, float, float, float, float, float, Mat);
// Check if a surface is being occluded by another one
int occlusion_surf(vector <int>, vector <Point3f>, Point3f);

int main(int argc, char** argv)
{
	// Initialization local coordinates
	vector <Point3f> vertices;
	vector <vector <int>> surfaces;

	// Read from .x file
	model_data(argc, argv, vertices, surfaces);

	// Camera Extrinsics
	// Camera position
	float t_x = 1.f;
	float t_y = 1.f;
	float t_z = 100.f;
	// Rotation x-axis
	float theta_x = 20.f * PI / 180.f;
	// Rotation y-axis
	float theta_y = 140.f * PI / 180.f;
	// Rotation z-axis
	float theta_z = 180.f * PI / 180.f;
	// Camera Intrinsics
	// F.O.V
	float fov = 50.f;
	// Principal point - center of image plane
	float u_0 = WIDTH / 2;
	float v_0 = HEIGHT / 2;
	// Focal length
	float f = (WIDTH / 2) / (float) (tan((fov / 2) * PI / 180.0));
	// Camera intrinsics Matrix
	Mat K = (Mat_<float>(3, 3) << f, 0, u_0, 0, f, v_0, 0, 0, 1);
	
	// Render model
	rendering(vertices, surfaces, theta_x, theta_y, theta_z, t_x, t_y, t_z, K);

	exit(EXIT_SUCCESS);
}

// Render function
void rendering(vector <Point3f>& vertices, vector <vector <int>>& surfaces, float theta_x, float theta_y, float theta_z, float t_x, float t_y, float t_z, Mat K)
{
	// Image plane 300x400 pixels
	Mat model(HEIGHT, WIDTH, CV_8U, Scalar::all(255));
	
	// Camera calibration
	// Translation Vector;
	Vec3f T = Vec3f(t_x, t_y, t_z);
	// Rotation x-axis Matrix
	Mat R_x = (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
	// Rotation y-axis Matrix
	Mat R_y = (Mat_<float>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
	// Rotation z-axis Matrix
	Mat R_z = (Mat_<float>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
	// Rotation Matrix
	Mat R = R_z * R_y * R_x;
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), T[0], R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), T[1], R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), T[2]);
	// Perpective Projection Matrix
	Mat	P = K * E;

	// Project vertices to image plane using perspective projection - camera coordinates
	vector <Point3f> vertices_hmgns;
	vector <Point2i> vertices_camera;
	cout << "Pixel coordinates: " << endl;
	for (int i = 0; i < vertices.size(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		vector <float> tmp1;
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
		cout << vertices_camera[i] << endl;
	}
	
	/*Mat t = (Mat_<float>(4, 1) << T.val[0], T.val[1], T.val[2], 1);
	t = P * t;*/
	Mat t = R * Mat(T);
	Vec3f T_hmgns = t;
	// Draw surfaces
	for (int i = 0; i < surfaces.size(); i++)
	{
		// Draw if surface is not occluded
		if (!occlusion_surf(surfaces[i], vertices_hmgns, T_hmgns))
		{
			vector <int> tmp;
			tmp = surfaces[i];
			for (int j = 0; j < tmp.size(); j++)
			{
				// Draw vertices
				model.at<uchar>(vertices_camera[tmp[j]].y, vertices_camera[tmp[j]].x) = 0;
				// Draw edges
				Point2i p1(vertices_camera[tmp[j]]);
				Point2i p2(vertices_camera[tmp[(j + 1) % 4]]);
				line(model, p1, p2, Scalar::all(0), 1, CV_AA, 0);
			}
		}
	}

	// Display model
	const char* window_name = "Cuboid";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, model);
	waitKey(0);
}

// Check if a surface is being occluded by another one
int occlusion_surf(vector <int> surface, vector <Point3f> vertices_hmgns, Point3f T)
{
	// Vectors begining from down right vertex of surface
	Vec3f vec1 = vertices_hmgns[surface[0]] - vertices_hmgns[surface[1]];
	Vec3f vec2 = vertices_hmgns[surface[2]] - vertices_hmgns[surface[1]];
	// Normal vector of surface
	vec1 = vec1 / norm(vec1, NORM_L2);
	//normalize(vec2, vec2, 1, NORM_L2);
	vec2 = vec2 / norm(vec2, NORM_L2);
	Vec3f norm_vec = vec2.cross(vec1);
	//normalize(norm_vec, norm_vec, 1, NORM_L2);
	norm_vec = norm_vec / norm(norm_vec, NORM_L2);
	// Vector between camera position and down right vertex of surface
	Vec3f vec_look = T - vertices_hmgns[surface[1]];
	//normalize(vec_look, vec_look, 1, NORM_L2);
	vec_look = vec_look / norm(vec_look, NORM_L2);
	// Angle between look vector and normal of surface
	float theta = acos(norm_vec.dot(vec_look)) * 180.f / PI;

	cout << theta << endl;

	//return 0;

	// If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta <= 90.f)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

// Read model data from .x file
void model_data(int argc, char **argv, vector<Point3f>& vertices, vector<vector<int>>& surfaces)
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