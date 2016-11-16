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

// Render function
void rendering(vector <Point3f>&, vector <vector <int>>&, Mat);

int main(int, char** argv)
{
	// Initialization local coordinates
	vector <Point3f> vertices;
	vector <vector <int>> edges;
	
	ifstream file(argv[1]);
	// Check if input is empty or the file is empty
	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
		cout << "Provide input images file with .x extension and " << endl;
		cout << "make sure it's not empty: ";
		system("PAUSE");
		return (0);
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

			Point3f tmp((float) v[0], (float) v[1], (float) v[2]);
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

	// Camera calibration
	// Camera position
	float t_x = 3.f;
	float t_y = 1.f;
	float t_z = 80.f;
	// F.O.V
	float fov = 50.f;
	// Rotation x-axis
	float theta_x = 20.f * PI / 180.f;
	// Principal point - center of image plane
	float u_0 = WIDTH / 2;
	float v_0 = HEIGHT / 2;
	// Focal length
	float f = (WIDTH / 2) / (float) (tan((fov / 2) * PI / 180.0));
	// Camera extrinsics
	Mat E = (Mat_<float>(3, 4) << 1, 0, 0, t_x, 0, cos(theta_x), -sin(theta_x), t_y, 0, sin(theta_x), cos(theta_x), t_z);
	// Camera intrinsics
	Mat K = (Mat_<float>(3, 3) << f, 0, u_0, 0, f, v_0, 0, 0, 1);
	// Perpective Projection Matrix
	Mat P(3, 4, CV_32FC1, Scalar::all(0));
	P = K * E;

	cout << P << endl;
	
	// Render model
	rendering(vertices, edges, P);

	return 0;
}

// Render function
void rendering(vector <Point3f>& vertices, vector <vector <int>>& edges, Mat P)
{
	// Image plane 300x400 pixels
	Mat cuboid(HEIGHT, WIDTH, CV_8U, Scalar::all(255));
	
	// Project vertices to image plane using perspective projection - camera coordinates
	vector <Point3f> vertices_homo;
	vector <Point2i> vertices_camera;
	
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
		vertices_homo.push_back(tmp3);
		
		// Pixel coordinates
		Point2f tmp4((tmp3.x / tmp3.z), (tmp3.y / tmp3.z));
		//

		Point2i tmp5(tmp4);
		vertices_camera.push_back(tmp5);
		cout << vertices_camera[i] << endl;
	}

	/*perspectiveTransform(vertices, vertices_camera, P);*/
	
	// Draw vertices
	for (int i = 0; i < vertices_camera.size(); i++)
	{
		cuboid.at<uchar>(vertices_camera[i].y, vertices_camera[i].x) = 0;
	}
	// Draw edges
	for (int i = 0; i < edges.size(); i++)
	{
		vector <int> tmp;
		tmp = edges[i];
		Point2i p1(vertices_camera[tmp[0]]);
		Point2i p2(vertices_camera[tmp[1]]);
		line(cuboid, p1, p2, Scalar::all(0), 1, CV_AA, 0);
	}

	// Display model
	const char* window_name = "Cuboid";
	namedWindow(window_name, WINDOW_AUTOSIZE);
	imshow(window_name, cuboid);
	waitKey(0);
}