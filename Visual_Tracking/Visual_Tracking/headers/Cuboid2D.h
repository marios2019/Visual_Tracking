#pragma once
// OpenCV libraries
#include "opencv2\imgcodecs.hpp"

// C++ libraries
#include <iostream>
#include <vector>

#define VERTICES 8

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
	vector <Point3f> getHomogeneousVertices() const; // Returns a vector of cuboid2d homogeneous vertices
	int getHomogeneousVerticesSize() const; // Returns the number of vertices homogeneous
	
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

	void setVerticesPxl(vector <Point2f> verticesPixelVal); // Change verticesPixel vector
	void setVertexPxl(Point2f verticesPixelVal, int idx); // Change vertexPixel[idx] value
	Point2f getVertexPxl(int idx); // Returns a vertex of the cuboid2d, in pixel coordinates
	vector <Point2f> getVerticesPxl() const; // Returns a vector of cuboid2d vertices, in pixel coordinates
	size_t getVerticesPxlSize() const; // Returns the number of verticesPixel

private:
	vector <Point3f> homogeneousVertices; // vertices of the cuboid2d, in homogeneous coordinates
	vector <vector <Point2f>> edges; // edges of the cuboid2d, projected in the image plane
	vector <vector <int>> surfaces; // surfaces of the cuboid2d, projected in the image plane
	vector <vector <int>> edgesPtr; // pointers to the vertices of the cuboid2d
	vector <Point2f> verticesPixel; // vertices of the cuboid2d, in pixel coordinates

	bool setEdgePtr(vector <int> edgeVerticesVal); // Set edgeVertices pointer to verticesPixel
};