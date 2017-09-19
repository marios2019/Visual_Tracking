#include "..\headers\Cuboid2D.h"

// Cuboid2D class definition

// Constructor
Cuboid2D::Cuboid2D(vector <Point3f> homoVerticesVal)
{
	if ((homoVerticesVal.size() < _VERTICES) || (homoVerticesVal.size() > _VERTICES))
	{
		cout << "Incompatible homogeneous _VERTICES vector size (size = 8 is mandatory).";
		exit(EXIT_FAILURE);
	}

	// HomogeneousVertices initialization
	homogeneousVertices = homoVerticesVal;

	// Vertices_PIxel and verticesVisibility initialization
	Point2f vertex_PIxel;
	for (int i = 0; i < homogeneousVertices.size(); i++)
	{
		vertex_PIxel.x = homogeneousVertices[i].x / homogeneousVertices[i].z;
		vertex_PIxel.y = homogeneousVertices[i].y / homogeneousVertices[i].z;
		vertices_PIxel.push_back(vertex_PIxel);
	}
}


Cuboid2D::~Cuboid2D()
{
}

// Returns a vertex of the Cuboid2D
Point3f Cuboid2D::getHomogeneousVertex(int idx)
{
	// Check if idx is between 0.._VERTICES=8
	checkIdx("Cuboid2D::homogeneousVertices", idx, _VERTICES);

	return homogeneousVertices[idx];
}

// Returns a vector of Cuboid2D _VERTICES
vector <Point3f> Cuboid2D::getHomogeneousVertices() const
{
	return homogeneousVertices;
}

// Returns the number of _VERTICES
int Cuboid2D::getHomogeneousVerticesSize() const
{
	return _VERTICES;
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
	checkIdx("Cuboid2D::_EDGES", idx, edges.size());

	edges[idx].swap(edges.back());
	edges.pop_back();
}

// Returns an edge of the Cuboid2D
vector <Point2f> Cuboid2D::getEdge(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::_EDGES", idx, edges.size());
	
	return edges[idx];
}

// Returns a vector of Cuboid2D _EDGES
vector <vector <Point2f>> Cuboid2D::getEdges() const
{
	return edges;
}

// Returns the number of _EDGES
size_t Cuboid2D::getEdgesSize() const
{
	return edges.size();
}

// Set edgeVertices pointer to vertices_PIxel
bool Cuboid2D::setEdgePtr(vector <int> edgePtrVal)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::edgesVertices", edgePtrVal[0], vertices_PIxel.size());
	checkIdx("Cuboid2D::edgesVertices", edgePtrVal[1], vertices_PIxel.size());
	
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
	checkIdx("Cuboid2D::_SURFACES", idx, surfaces.size());

	return surfaces[idx];
}

// Returns a vector of Cuboid2D _SURFACES
vector <vector <int>> Cuboid2D::getSurfaces() const
{
	return surfaces;
}

// Returns the number of _SURFACES
size_t Cuboid2D::getSurfacesSize() const
{
	return surfaces.size();
}

// Change vertices_PIxel vector
void Cuboid2D::setVerticesPxl(vector <Point2f> vertices_PIxelVal)
{
	vertices_PIxel = vertices_PIxelVal;
}

// Change vertex_PIxel[idx] value
void Cuboid2D::setVertexPxl(Point2f vertex_PIxelVal, int idx)
{
	// Check if idx is valid
	checkIdx("Cuboid2D::vertices_PIxel", idx, vertices_PIxel.size());

	vertices_PIxel[idx] = vertex_PIxelVal;
}

// Returns a vertex of the Cuboid2D, in _PIxel coordinates 
Point2f Cuboid2D::getVertexPxl(int idx)
{
	// Check if idx is valid
	checkIdx("Cuboi2D::vertices_PIxel", idx, vertices_PIxel.size());

	return vertices_PIxel[idx];
}

// Returns a vector of Cuboid2D _VERTICES, in _PIxel coordinates
vector <Point2f> Cuboid2D::getVerticesPxl() const
{
	return vertices_PIxel;
}

// Returns the number of vertices_PIxel
size_t Cuboid2D::getVerticesPxlSize() const
{
	return vertices_PIxel.size();
}