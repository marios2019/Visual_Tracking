#include "..\headers\Cuboid2D.h"

// Cuboid2D class definition

// Constructor
Cuboid2D::Cuboid2D(vector <Point3f> homoVerticesVal)
{
	if ((homoVerticesVal.size() < VERTICES) || (homoVerticesVal.size() > VERTICES))
	{
		cout << "Incompatible homogeneous vertices vector size (size = 8 is mandatory).";
		exit(EXIT_FAILURE);
	}

	// HomogeneousVertices initialization
	homogeneousVertices = homoVerticesVal;

	// VerticesPixel and verticesVisibility initialization
	Point2f vertexPixel;
	for (int i = 0; i < homogeneousVertices.size(); i++)
	{
		vertexPixel.x = homogeneousVertices[i].x / homogeneousVertices[i].z;
		vertexPixel.y = homogeneousVertices[i].y / homogeneousVertices[i].z;
		verticesPixel.push_back(vertexPixel);
	}
}


Cuboid2D::~Cuboid2D()
{
}

// Returns a vertex of the Cuboid2D
Point3f Cuboid2D::getHomogeneousVertex(int idx)
{
	// Check if idx is between 0..VERTICES=8
	checkIdx("Cuboid2D::homogeneousVertices", idx, VERTICES);

	return homogeneousVertices[idx];
}

// Returns a vector of Cuboid2D vertices
vector <Point3f> Cuboid2D::getHomogeneousVertices() const
{
	return homogeneousVertices;
}

// Returns the number of vertices
int Cuboid2D::getHomogeneousVerticesSize() const
{
	return VERTICES;
}

// Set edge to be rendered
void Cuboid2D::setEdge(vector <Point2f> edgeVal, vector <int> edgeVerticesVal)
{
	if (setEdgePtr(edgeVerticesVal))
	{
		edges.push_back(edgeVal);
	}
}

// Destroy edge[idx]
void Cuboid2D::destroyEdge(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::edges", idx, edges.size());

	edges[idx].swap(edges.back());
	edges.pop_back();
}

// Returns an edge of the Cuboid2D
vector <Point2f> Cuboid2D::getEdge(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::edges", idx, edges.size());
	
	return edges[idx];
}

// Returns a vector of Cuboid2D edges
vector <vector <Point2f>> Cuboid2D::getEdges() const
{
	return edges;
}

// Returns the number of edges
size_t Cuboid2D::getEdgesSize() const
{
	return edges.size();
}

// Set edgeVertices pointer to verticesPixel
bool Cuboid2D::setEdgePtr(vector <int> edgePtrVal)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::edgesVertices", edgePtrVal[0], verticesPixel.size());
	checkIdx("Cuboid2D::edgesVertices", edgePtrVal[1], verticesPixel.size());
	
	// Check for duplicates
	for (int i = 0; i < edgesPtr.size(); i++)
	{
		if (edgesPtr[i] == edgePtrVal)
		{
			return false;
		}
	}

	edgesPtr.push_back(edgePtrVal);

	return true;
}

// Get edgeVertices[idx]
vector <int> Cuboid2D::getEdgePtr(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::edgesVertices", idx, edgesPtr.size());

	return edgesPtr[idx];
}

// Get all edgeVertices
vector <vector <int>> Cuboid2D::getEdgesPtr() const
{
	return edgesPtr;
}

// Set surface to be rendered
void Cuboid2D::setSurface(vector <int> surfaceVal)
{
	surfaces.push_back(surfaceVal);
}

// Returns a surface of the Cuboid2D
vector <int> Cuboid2D::getSurface(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::surfaces", idx, surfaces.size());

	return surfaces[idx];
}

// Returns a vector of Cuboid2D surfaces
vector <vector <int>> Cuboid2D::getSurfaces() const
{
	return surfaces;
}

// Returns the number of surfaces
size_t Cuboid2D::getSurfacesSize() const
{
	return surfaces.size();
}

// Change verticesPixel vector
void Cuboid2D::setVerticesPxl(vector <Point2f> verticesPixelVal)
{
	verticesPixel = verticesPixelVal;
}

// Change vertexPixel[idx] value
void Cuboid2D::setVertexPxl(Point2f vertexPixelVal, int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::verticesPixel", idx, verticesPixel.size());

	verticesPixel[idx] = vertexPixelVal;
}

// Returns a vertex of the Cuboid2D, in pixel coordinates 
Point2f Cuboid2D::getVertexPxl(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboi2D::verticesPixel", idx, verticesPixel.size());

	return verticesPixel[idx];
}

// Returns a vector of Cuboid2D vertices, in pixel coordinates
vector <Point2f> Cuboid2D::getVerticesPxl() const
{
	return verticesPixel;
}

// Returns the number of verticesPixel
size_t Cuboid2D::getVerticesPxlSize() const
{
	return verticesPixel.size();
}