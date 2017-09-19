#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define _VERTICES 8

using namespace cv;
using namespace std;

#include "error.h"

// Cuboid2D class declaration. Each Cuboid3D instance
// is a 2d cuboid in Projective geometry, produced from
// the perspective projection of a Cuboid3D into an
// image plane, from the desirable pose of a Camera instance.

class Cuboid2D
{
public:
	Cuboid2D(vector <Point3f> homoVerticesVal); // Constructor
	~Cuboid2D(); // Destructor

	Point3f getHomogeneousVertex(int idx); // Returns a homogeneous vertex of the cuboid2d
	vector <Point3f> getHomogeneousVertices() const; // Returns a vector of cuboid2d homogeneous _VERTICES
	int getHomogeneousVerticesSize() const; // Returns the number of _VERTICES homogeneous
	
	void setEdge(vector <Point2f> edgeVal, vector <int> edgeVerticesVal); // Set edge to be rendered
	void destroyEdge(int idx); // Destroy edge[idx]
	vector <Point2f> getEdge(int idx); // Returns an edge of the cuboid2d
	vector <vector <Point2f>> getEdges() const; // Returns a vector of cuboid2d edges
	size_t getEdgesSize() const; // Returns the number of edges

	vector <int> getEdgePtr(int idx); // Get edgeVertices[idx]
	vector <vector <int>> getEdgesPtr() const; // Get all edgeVertices
	
	void setSurface(vector <int> surfaceVal); // Set surface to be rendered
	vector <int> getSurface(int idx); // Returns a surface of the cuboid2d
	vector <vector <int>> getSurfaces() const; // Returns a vector of cuboid2d surfaces
	size_t getSurfacesSize() const; // Returns the number of surfaces

	void setVerticesPxl(vector <Point2f> vertices_PIxelVal); // Change vertices_PIxel vector
	void setVertexPxl(Point2f vertices_PIxelVal, int idx); // Change vertex_PIxel[idx] value
	Point2f getVertexPxl(int idx); // Returns a vertex of the cuboid2d, in _PIxel coordinates
	vector <Point2f> getVerticesPxl() const; // Returns a vector of cuboid2d _VERTICES, in _PIxel coordinates
	size_t getVerticesPxlSize() const; // Returns the number of vertices_PIxel

private:
	vector <Point3f> homogeneousVertices; // _VERTICES of the cuboid2d, in homogeneous coordinates
	vector <vector <Point2f>> edges; // edges of the cuboid2d, projected in the image plane
	vector <vector <int>> surfaces; // surfaces of the cuboid2d, projected in the image plane
	vector <vector <int>> edgesPtr; // pointers to the _VERTICES of the cuboid2d
	vector <Point2f> vertices_PIxel; // _VERTICES of the cuboid2d, in _PIxel coordinates

	bool setEdgePtr(vector <int> edgeVerticesVal); // Set edgeVertices pointer to vertices_PIxel
};