#include "..\headers\Cuboid.h"

// Constructor
Cuboid::Cuboid(float length, float height, float width)
{
	// Vertices and verticesPixel initialization
	Point3f vertice;
	int j = 0;
	for (int i = 0; i < VERTICES; i++)
	{
		if (i < 4)
		{
			vertice = Point3f((i % 3 > 0 ? length : 0.f), (i % 4 > 1 ? height : 0.f), 0.f);
			vertices.push_back(vertice);
		}
		else
		{
			vertice = Point3f((j % 3 > 0 ? length : 0.f), (j % 4 > 1 ? height : 0.f), width);
			vertices.push_back(vertice);
			++j;
		}

		verticesPixel.push_back(Point2i());
	}

	// Edges initialization
	edges.push_back(vector <int>{0, 1});
	edges.push_back(vector <int>{1, 2});
	edges.push_back(vector <int>{2, 3});
	edges.push_back(vector <int>{3, 0});
	edges.push_back(vector <int>{4, 5});
	edges.push_back(vector <int>{5, 6});
	edges.push_back(vector <int>{6, 7});
	edges.push_back(vector <int>{7, 4});
	edges.push_back(vector <int>{4, 0});
	edges.push_back(vector <int>{5, 1});
	edges.push_back(vector <int>{6, 2});
	edges.push_back(vector <int>{7, 3});
	
	// Surfaces initialization
	surfaces.push_back(vector <int>{1, 0, 3, 2});
	surfaces.push_back(vector <int>{0, 4, 7, 3});
	surfaces.push_back(vector <int>{4, 5, 6, 7});
	surfaces.push_back(vector <int>{5, 1, 2, 6});
	surfaces.push_back(vector <int>{7, 6, 2, 3});
	surfaces.push_back(vector <int>{0, 1, 5, 4});

	// EdgesVisibility initializagtion
	for (int i = 0; i < EDGES; i++)
	{
		edgesVisibility.push_back(false);
	}

	// SurfacesVisibility initializagtion
	for (int i = 0; i < SURFACES; i++)
	{
		surfacesVisibility.push_back(false);
	}
}

// Destructor
Cuboid::~Cuboid()
{
}

// Change the length of the cuboid
void Cuboid::setLength(float length)
{
	// Check for non negative value of length
	if (length < 0.f)
	{
		cout << "Length must have a positive value." << endl;
		return;
	}

	// Change the x-axis value to match the new length
	int j = 0;
	for (int i = 0; i < VERTICES; i++)
	{
		if (i < 4)
		{
			vertices[i].x = (i % 3 > 0 ? length : 0.f);
		}
		else
		{
			vertices[i].x = (j % 3 > 0 ? length : 0.f);
			++j;
		}
	}
}

// Returns the length of the cuboid
float Cuboid::getLength() const
{
	return vertices[1].x;
}

// Change the height of the cuboid
void Cuboid::setHeight(float height)
{
	// Check for non negative value of length
	if (height < 0.f)
	{
		cout << "Height must have a positive value." << endl;
		return;
	}

	// Change the y-axis value to match the new height
	int j = 0;
	for (int i = 0; i < VERTICES; i++)
	{
		if (i < 4)
		{
			vertices[i].y = (i % 4 > 1 ? height : 0.f);
		}
		else
		{
			vertices[i].y = (i % 4 > 1 ? height : 0.f);
			++j;
		}
	}
}

// Returns the height of the cuboid
float Cuboid::getHeight() const
{
	return vertices[2].y;
}

// Change the width of the cuboid
void Cuboid::setWidth(float width)
{
	// Check for non negative value of width
	if (width < 0.f)
	{
		cout << "Width must have a positive value." << endl;
		return;
	}

	// Change the z-axis value to match the new width
	for (int i = 4; i < VERTICES; i++)
	{
		vertices[i].z = width;
	}
}

// Returns the width of the cuboid
float Cuboid::getWidth() const
{
	return vertices[4].z;
}

// Change the dimensions of the cuboid
void Cuboid::setDimensions(Point3f dimensionsVal)
{
	// Check for non negative value of length
	if (dimensionsVal.x < 0.f)
	{
		cout << "Length must have a positive value." << endl;
		return;
	}
	// Check for non negative value of height
	if (dimensionsVal.y < 0.f)
	{
		cout << "Height must have a positive value." << endl;
		return;
	}
	// Check for non negative value of width
	if (dimensionsVal.z < 0.f)
	{
		cout << "Width must have a positive value." << endl;
		return;
	}

	// Change the new dimensions
	int j = 0;
	for (int i = 0; i < VERTICES; i++)
	{
		if (i < 4)
		{
			vertices[i] = Point3f((i % 3 > 0 ? dimensionsVal.x : 0.f), (i % 4 > 1 ? dimensionsVal.y : 0.f), 0.f);
		}
		else
		{
			vertices[i] = Point3f((j % 3 > 0 ? dimensionsVal.x : 0.f), (j % 4 > 1 ? dimensionsVal.y : 0.f), dimensionsVal.z);
			++j;
		}
	}
}

// Returns the dimensions of the cuboid
Point3f Cuboid::getDimensions() const
{
	return Point3f(getLength(), getHeight(), getWidth());
}

// Returns a vertice of the cuboid, in local coordinates
Point3f Cuboid::getVertice(int idx)
{
	// Check if idx is between 0..VERTICES=8
	return vertices[checkIdx(idx, VERTICES)];
}

// Returns a vector of cuboid vertices, in local coordinates
vector <Point3f> Cuboid::getVertices() const
{
	return vertices;
}

// Returns the number of vertices
int Cuboid::getVerticesSize() const
{
	return VERTICES;
}

// Returns an edge of the cuboid
vector <int> Cuboid::getEdge(int idx)
{
	// Check if idx is between 0..EDGES=12
	return edges[checkIdx(idx, EDGES)];
}

// Returns a vector of cuboid edges
vector <vector <int>> Cuboid::getEdges() const
{
	return edges;
}

// Returns the number of edges
int Cuboid::getEdgesSize() const
{
	return EDGES;
}

// Returns a surface of the cuboid
vector <int> Cuboid::getSurface(int idx)
{
	// Check if idx is between 0..SURFACES=6
	return surfaces[checkIdx(idx, SURFACES)];
}

// Returns a vector of cuboid surfaces
vector <vector <int>> Cuboid::getSurfaces() const
{
	return surfaces;
}

// Returns the number of surfaces
int Cuboid::getSurfacesSize() const
{
	return SURFACES;
}

// Change the edge visibility
void Cuboid::setEdgeVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= EDGES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	edgesVisibility[idx] = visible;
}

bool Cuboid::getEdgeVisibility(int idx)
{
	// Check if idx is between 0..EDGES=12
	return edgesVisibility[checkIdx(idx, EDGES)];
}

// Returns the vector of edges visibility
vector <bool> Cuboid::getEdgesVisibility() const
{
	return edgesVisibility;
}

// Change the surface visibility
void Cuboid::setSurfaceVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= SURFACES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	surfacesVisibility[idx] = visible;
}

// Returns the surface visibility
bool Cuboid::getSurfaceVisibility(int idx)
{
	// Check if idx is between 0..SURFACES=6
	return surfacesVisibility[checkIdx(idx, SURFACES)];
}

// Returns the vector of surfaces visibility
vector <bool> Cuboid::getSurfacesVisibility() const
{
	return surfacesVisibility;
}

// Change verticesPixel vector
void Cuboid::setVerticesPxl(vector <Point2i> verticesPixelVal)
{
	if (verticesPixelVal.size() != VERTICES)
	{
		cout << "The vector must contain exactly 8 Point2i elements." << endl;
		return;
	}

	for (int i = 0; i < VERTICES; i++)
	{
		verticesPixel[i] = verticesPixelVal[i];
	}
}

// Change verticePixel
void Cuboid::setVerticePxl(Point2i verticePixelVal, int idx)
{
	if ((idx < 0) || (idx >= VERTICES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	verticesPixel[idx] = verticePixelVal;
}

// Returns a vertice of the cuboid, in pixel coordinates 
Point2i Cuboid::getVerticePxl(int idx)
{
	// Check if idx is between 0..VERTICES=8
	return verticesPixel[checkIdx(idx, VERTICES)];
}

// Returns a vector of cuboid vertices, in pixel coordinates
vector<Point2i> Cuboid::getVerticesPxl() const
{
	return verticesPixel;
}

int Cuboid::checkIdx(int idx, int limit)
{
	if (idx >= limit)
	{
		cout << "WARNING, invalid memory access; the last element will be returned." << endl;
		return (limit - 1);
	}
	if (idx < 0)
	{
		cout << "WARNING, invalid memory access; the first element will be returned." << endl;
		return 0;
	}

	return idx;
}
