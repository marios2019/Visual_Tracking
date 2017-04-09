#include "..\headers\Cuboid2D.h"

// Cuboid2D class definition

// Empty contructor
Cuboid2D::Cuboid2D()
{
	cout << "Incompatible homogeneous vertices vector size (size = 8 is mandatory).";
	exit(EXIT_FAILURE);
}

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
	Point2f tmp;
	Point2i verticePixel;
	for (int i = 0; i < homogeneousVertices.size(); i++)
	{
		tmp.x = homogeneousVertices[i].x / homogeneousVertices[i].z;
		tmp.y = homogeneousVertices[i].y / homogeneousVertices[i].z;
		verticePixel = tmp;
		verticesPixel.push_back(verticePixel);
	}
}


Cuboid2D::~Cuboid2D()
{
}

// Returns a vertice of the Cuboid2D
Point3f Cuboid2D::getHomogeneousVertice(int idx)
{
	// Check if idx is between 0..VERTICES=8
	checkIdx(idx, VERTICES);

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
void Cuboid2D::setEdge(vector <Point2f> edgeVal)
{
	edges.push_back(edgeVal);
}

// Change edge[idx] value
void Cuboid2D::setEdge(vector <Point2f> edge, int idx)
{
	// Check if idx is valid
	checkIdx(idx, (int)edges.size());

	edges[idx] = edge;
}

// Destroy edge[idx]
void Cuboid2D::destroyEdge(int idx)
{
	// Check if idx is valid
	checkIdx(idx, (int)edges.size());

	edges.erase(edges.begin() + idx);
}

// Returns an edge of the Cuboid2D
vector <Point2f> Cuboid2D::getEdge(int idx)
{
	// Check if idx is valid
	checkIdx(idx, (int) edges.size());
	
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

// Set surface to be rendered
void Cuboid2D::setSurface(vector <int> surfaceVal)
{
	surfaces.push_back(surfaceVal);
}

// Returns a surface of the Cuboid2D
vector <int> Cuboid2D::getSurface(int idx)
{
	// Check if idx is valid
	checkIdx(idx, (int) surfaces.size());

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
void Cuboid2D::setVertexPxl(int idx, Point2f verticePixelVal)
{
	// Check if idx is valid
	checkIdx(idx, verticesPixel.size());

	verticesPixel[idx] = verticePixelVal;
}

// Returns a vertex of the Cuboid2D, in pixel coordinates 
Point2f Cuboid2D::getVertexPxl(int idx)
{
	// Check if idx is valid
	checkIdx(idx, verticesPixel.size());

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

bool Cuboid2D::checkIdx(int idx, size_t limit)
{
	if ((idx >= limit) || (idx < 0))
	{
		cout << "ERROR, invalid memory access." << endl;
		exit(EXIT_FAILURE);
	}
	
	return true;
}