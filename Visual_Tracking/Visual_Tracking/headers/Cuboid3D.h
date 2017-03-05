#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define VERTICES 8
#define EDGES 12
#define SURFACES 6

using namespace cv;
using namespace std;

// Cuboid3D class declaration. Each Cuboid3D instance
// is a 3d cuboid in Euclidean geometry, with the dimensions
// provided, length - x-axis, height - y-axis, width - z-axis.

class Cuboid3D
{
public:
	Cuboid3D(float = 0.f, float = 0.f, float = 0.f); // Constructor
	~Cuboid3D(); // Destructor

	void setLength(float); // Change the length of the cuboid3d
	float getLength() const; // Returns the length of the cuboid3d

	void setHeight(float); // Change the height of the cuboid3d
	float getHeight() const; // Returns the height of the cuboid3d

	void setWidth(float); // Change the width of the cuboid3d
	float getWidth() const; // Returns the width of the cuboid3d

	void setDimensions(Point3f); // Change the dimensions of the cuboid3d
	Point3f getDimensions() const; // Returns the dimensions of the cuboid3d

	Point3f getVertice(int); // Returns a vertice of the cuboid3d, in local coordinates
	vector <Point3f> getVertices() const; // Returns a vector of cuboid3d vertices, in local coordinates
	int getVerticesSize() const; // Returns the number of vertices
	
	vector <int> getEdge(int); // Returns an edge of the cuboid3d
	vector <vector <int>> getEdges() const; // Returns a vector of cuboid3d edges
	int getEdgesSize() const; // Returns the number of edges
	
	vector <int> getSurface(int); // Returns a surface of the cuboid3d
	vector <vector <int>> getSurfaces() const; // Returns a vector of cuboid3d surfaces
	int getSurfacesSize() const; // Returns the number of surfaces

	void setEdgeVisibility(int, bool); // Change the edge visibility
	bool getEdgeVisibility(int); // Returns the edge visibility
	vector <bool> getEdgesVisibility() const; // Returns the vector of edges visibility

	void setSurfaceVisibility(int, bool); // Change the surface visibility
	bool getSurfaceVisibility(int); // Returns the surface visibility
	vector <bool> getSurfacesVisibility() const; // Returns the vector of surfaces visibility
	
private:
	vector <Point3f> vertices; // vertices of the cuboid, in local coordinates
	vector <vector <int>> edges, surfaces; // 12 edges and 6 surfaces of the cuboid
	vector <bool> edgesVisibility, surfacesVisibility; // Check the visibility of edges and surfaces according to the camera
	
	int checkIdx(int, int); // Check for invalid memory access
};

