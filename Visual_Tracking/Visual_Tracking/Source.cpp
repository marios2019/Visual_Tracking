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

int main(int argc, char** argv)
{
	// Initialization local coordinates
	vector <Point3f> vertices;
	vector <vector <int>> edges;

	// Read from .x file
	model_data(argc, argv, vertices, edges);

	// Camera Extrinsics
	// Camera position
	float t_x = 1.f;
	float t_y = 1.f;
	float t_z = 90.f;
	// Rotation x-axis
	float theta_x = 20.f * PI / 180.f;
	// Rotation y-axis
	float theta_y = 180.f * PI / 180.f;
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
	rendering(vertices, edges, theta_x, theta_y, theta_z, t_x, t_y, t_z, K);

	exit(EXIT_SUCCESS);
}

// Read model data from .x file
void model_data(int argc, char **argv, vector<Point3f>& vertices, vector<vector<int>>& edges)
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
	}
}

// Render function
void rendering(vector <Point3f>& vertices, vector <vector <int>>& edges, float theta_x, float theta_y, float theta_z, float t_x, float t_y, float t_z, Mat K)
{
	// Image plane 300x400 pixels
	Mat model(HEIGHT, WIDTH, CV_8U, Scalar::all(255));
	
	// Camera calibration
	// Translation Vector;
	Mat T = (Mat_<float>(3, 1) << t_x, t_y, t_z);
	// Rotation x-axis Matrix
	Mat R_x = (Mat_<float>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
	// Rotation y-axis Matrix
	Mat R_y = (Mat_<float>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
	// Rotation z-axis Matrix
	Mat R_z = (Mat_<float>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
	// Rotation Matrix
	Mat R = R_z * R_y * R_x;
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), T.at<float>(0, 0), R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), T.at<float>(1, 0), R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), T.at<float>(2, 0));
	// Perpective Projection Matrix
	Mat	P = K * E;

	// Project vertices to image plane using perspective projection - camera coordinates
	vector <Point3f> vertices_homo;
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

	// Draw vertices
	for (int i = 0; i < vertices_camera.size(); i++)
	{
		model.at<uchar>(vertices_camera[i].y, vertices_camera[i].x) = 0;
	}
	// Draw edges
	for (int i = 0; i < edges.size(); i++)
	{
		vector <int> tmp;
		tmp = edges[i];
		Point2i p1(vertices_camera[tmp[0]]);
		Point2i p2(vertices_camera[tmp[1]]);
		line(model, p1, p2, Scalar::all(0), 1, CV_AA, 0);
	}

	// Display model
	const char* window_name = "Cuboid";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, model);
	waitKey(0);
}