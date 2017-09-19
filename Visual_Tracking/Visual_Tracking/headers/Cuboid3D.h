#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define _VERTICES 8
#define _EDGES 12
#define _SURFACES 6

using namespace cv;
using namespace std;

#include "error.h"

// Cuboid3D class declaration. Each Cuboid3D instance
// is a 3d cuboid in Euclidean geometry, with the dimensions
// provided, length - x-axis, height - y-axis, width - z-axis.

class Cuboid3D
{
public:
	Cuboid3D(float length = 0.f, float height = 0.f, float width = 0.f); // Constructor
	~Cuboid3D(); // Destructor

	void setLength(float length); // Change the length of the cuboid3d
	float getLength() const; // Returns the length of the cuboid3d

	void setHeight(float height); // Change the height of the cuboid3d
	float getHeight() const; // Returns the height of the cuboid3d

	void setWidth(float width); // Change the width of the cuboid3d
	float getWidth() const; // Returns the width of the cuboid3d

	void setDimensions(Point3f dimensionsVal); // Change the dimensions of the cuboid3d
	Point3f getDimensions() const; // Returns the dimensions of the cuboid3d

	Point3f getVertex(int idx); // Returns a vertex of the cuboid3d, in local coordinates
	vector <Point3f> getVertices() const; // Returns a vector of cuboid3d _VERTICES, in local coordinates
	int getVerticesSize() const; // Returns the number of _VERTICES
	
	vector <int> getEdge(int idx); // Returns an edge of the cuboid3d
	vector <vector <int>> getEdges() const; // Returns a vector of cuboid3d edges
	int getEdgesSize() const; // Returns the number of edges
	
	vector <int> getSurfaceVertices(int idx); // Returns a surface (with pointers to _VERTICES) of the cuboid3d
	vector <vector <int>> getSurfacesVertices() const; // Returns a vector of cuboid3d surfaces (with pointers to _VERTICES)
	vector <int> getSurfaceEdges(int idx); // Returns a surface (with pointers to edges) of the cuboid3d
	vector <vector <int>> getSurfacesEdges() const; // Returns a vector of cuboid3d surfaces (with pointers to _VERTICES)
	int getSurfacesSize() const; // Returns the number of surfaces

	void setEdgeVisibility(int idx, bool visible); // Change the edge visibility
	bool getEdgeVisibility(int idx); // Returns the edge visibility
	vector <bool> getEdgesVisibility() const; // Returns the vector of edges visibility

	void setSurfaceVisibility(int idx, bool visible); // Change the surface visibility
	bool getSurfaceVisibility(int idx); // Returns the surface visibility
	vector <bool> getSurfacesVisibility() const; // Returns the vector of surfaces visibility

	// Overload
	void setEdgesRendered(bool render); // Set all edgesRendered elements to true or false
	void setEdgesRendered(int edgeVal); // Set which edges will be rendered

private:
	vector <Point3f> vertices; // _VERTICES of the cuboid, in local coordinates
	vector <vector <int>> edges, surfacesVertices, surfacesEdges; // 12 edges and 6 surfaces of the cuboid
	vector <bool> edgesVisibility, surfacesVisibility; // Check the visibility of edges and surfaces according to the camera
	vector <bool> edgesRendered; // Define which edges are going to be rendered
};
