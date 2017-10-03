#include "..\headers\Cuboid3D.h"

// Cuboid3D class defintion.

// Constructor
Cuboid3D::Cuboid3D(float length, float height, float width)
{
	// _VERTICES initialization
	Point3f vertex;
	for (int i = 0; i < _VERTICES; i++)
	{
			vertex = Point3f(((i % 4 != 0) && (i % 4 != 3) ? length : 0.f), (i % 4 >= 2 ? height : 0.f), (i >= 4 ? width : 0.f));
			vertices.push_back(vertex);
	}

	// _EDGES initialization
	edges.push_back({ 0, 1 }); // 0
	edges.push_back({ 1, 2 }); // 1
	edges.push_back({ 2, 3 }); // 2
	edges.push_back({ 3, 0 }); // 3
	edges.push_back({ 4, 5 }); // 4
	edges.push_back({ 5, 6 }); // 5
	edges.push_back({ 6, 7 }); // 6
	edges.push_back({ 7, 4 }); // 7
	edges.push_back({ 4, 0 }); // 8
	edges.push_back({ 5, 1 }); // 9
	edges.push_back({ 6, 2 }); // 10
	edges.push_back({ 7, 3 }); // 11
	
	// _SURFACES initialization
	surfacesVertices.push_back({ 1, 0, 3, 2 }); // 0
	surfacesVertices.push_back({ 0, 4, 7, 3 }); // 1
	surfacesVertices.push_back({ 4, 5, 6, 7 }); // 2
	surfacesVertices.push_back({ 5, 1, 2, 6 }); // 3
	surfacesVertices.push_back({ 7, 6, 2, 3 }); // 4
	surfacesVertices.push_back({ 0, 1, 5, 4 }); // 5
	surfacesEdges.push_back({ 0, 1, 2, 3 }); // 0
	surfacesEdges.push_back({ 8, 3, 11, 7 }); // 1
	surfacesEdges.push_back({ 4, 5, 6, 7 }); // 2
	surfacesEdges.push_back({ 9, 1, 10, 5 }); // 3
	surfacesEdges.push_back({ 6, 10, 2, 11 }); // 4
	surfacesEdges.push_back({ 0, 9, 4, 8 }); // 5

	// EdgesVisibility initializagtion
	for (int i = 0; i < _EDGES; i++)
	{
		edgesVisibility.push_back(false);
		edgesRendered.push_back(true);
	}

	// SurfacesVisibility initializagtion
	for (int i = 0; i < _SURFACES; i++)
	{
		surfacesVisibility.push_back(false);
	}
}

// Destructor
Cuboid3D::~Cuboid3D()
{
}

// Change the length of the Cuboid3D
void Cuboid3D::setLength(float length)
{
	// Check for non negative value of length
	if (length < 0.f)
	{
		cout << "Length must have a positive value." << endl;
		return;
	}

	// Change the x-axis value to match the new length
	for (int i = 0; i < _VERTICES; i++)
	{
		// Check which _VERTICES must change their x value
		vertices[i].x = ((i % 4 != 0) && (i % 4 != 3) ? length : 0.f);
	}
}

// Returns the length of the Cuboid3D
float Cuboid3D::getLength() const
{
	return vertices[1].x;
}

// Change the height of the Cuboid3D
void Cuboid3D::setHeight(float height)
{
	// Check for non negative value of length
	if (height < 0.f)
	{
		cout << "Height must have a positive value." << endl;
		return;
	}

	// Change the y-axis value to match the new height
	for (int i = 0; i < _VERTICES; i++)
	{
		// Check which _VERTICES must change their y value
		vertices[i].y = (i % 4 >= 2 ? height : 0.f);
	}
}

// Returns the height of the Cuboid3D
float Cuboid3D::getHeight() const
{
	return vertices[2].y;
}

// Change the width of the Cuboid3D
void Cuboid3D::setWidth(float width)
{
	// Check for non negative value of width
	if (width < 0.f)
	{
		cout << "Width must have a positive value." << endl;
		return;
	}

	// Change the z-axis value to match the new width
	for (int i = 4; i < _VERTICES; i++)
	{
		vertices[i].z = width;
	}
}

// Returns the width of the Cuboid3D
float Cuboid3D::getWidth() const
{
	return vertices[4].z;
}

// Change the dimensions of the Cuboid3D
void Cuboid3D::setDimensions(Point3f dimensionsVal)
{
	// Update dimensions
	setLength(dimensionsVal.x);
	setHeight(dimensionsVal.y);
	setWidth(dimensionsVal.z);
}

// Returns the dimensions of the Cuboid3D
Point3f Cuboid3D::getDimensions() const
{
	return Point3f(getLength(), getHeight(), getWidth());
}

// Returns a vertex of the Cuboid3D, in local coordinates
Point3f Cuboid3D::getVertex(int idx)
{
	// Check if idx is between 0.._VERTICES=8
	checkIdx("Cuboid3D::_VERTICES", idx, _VERTICES);

	return vertices[idx];
}

// Returns a vector of Cuboid3D _VERTICES, in local coordinates
vector <Point3f> Cuboid3D::getVertices() const
{
	return vertices;
}

// Returns the number of _VERTICES
int Cuboid3D::getVerticesSize() const
{
	return _VERTICES;
}

// Returns an edge of the Cuboid3D
vector <int> Cuboid3D::getEdge(int idx)
{
	// Check if idx is between 0.._EDGES=12
	checkIdx("Cuboid3D::_EDGES", idx, _EDGES);

	return edges[idx];
}

// Returns a vector of Cuboid3D _EDGES
vector <vector <int>> Cuboid3D::getEdges() const
{
	return edges;
}

// Returns the number of _EDGES
int Cuboid3D::getEdgesSize() const
{
	return _EDGES;
}

// Returns a surface of the Cuboid3D (with pointers to _VERTICES)
vector <int> Cuboid3D::getSurfaceVertices(int idx)
{
	// Check if idx is between 0.._SURFACES=6
	checkIdx("Cuboid3D::surfacesVertices", idx, _SURFACES);

	return surfacesVertices[idx];
}

// Returns a vector of Cuboid3D _SURFACES (with pointers to _VERTICES)
vector <vector <int>> Cuboid3D::getSurfacesVertices() const
{
	return surfacesVertices;
}

// Returns a surface (with pointers to _EDGES) of the cuboid3d
vector <int> Cuboid3D::getSurfaceEdges(int idx)
{
	// Check if idx is between 0.._SURFACES=6
	checkIdx("Cuboid3D::surfacesEdges", idx, _SURFACES);

	return surfacesEdges[idx];
}

// Returns a vector of cuboid3d _SURFACES (with pointers to _VERTICES)
vector <vector <int>> Cuboid3D::getSurfacesEdges() const
{
	return surfacesEdges;
}

// Returns the number of _SURFACES
int Cuboid3D::getSurfacesSize() const
{
	return _SURFACES;
}

// Change the edge visibility
void Cuboid3D::setEdgeVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= _EDGES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	if (edgesRendered[idx] == true)
	{
		edgesVisibility[idx] = visible;
	}
	else
	{ 
		edgesVisibility[idx] = false;
	}
}

bool Cuboid3D::getEdgeVisibility(int idx)
{
	// Check if idx is between 0.._EDGES=12
	checkIdx("Cuboid3D::edgesVisibility", idx, _EDGES);

	return edgesVisibility[idx];
}

// Returns the vector of _EDGES visibility
vector <bool> Cuboid3D::getEdgesVisibility() const
{
	return edgesVisibility;
}

// Change the surface visibility
void Cuboid3D::setSurfaceVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= _SURFACES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	surfacesVisibility[idx] = visible;
	// Set surface's _EDGES to visibility = visible
	vector <int> surface = surfacesEdges[idx];
	for (int i = 0; i < surface.size(); i++)
	{
		setEdgeVisibility(surface[i], visible);
	}
}

// Returns the surface visibility
bool Cuboid3D::getSurfaceVisibility(int idx)
{
	// Check if idx is between 0.._SURFACES=6
	checkIdx("Cuboid3D::surfacesVisibility", idx, _SURFACES);

	return surfacesVisibility[idx];
}

// Returns the vector of _SURFACES visibility
vector <bool> Cuboid3D::getSurfacesVisibility() const
{
	return surfacesVisibility;
}

// Set all edgesRendered elements to true or false
void Cuboid3D::setEdgesRendered(bool render)
{
	// Set first edgeRendered = true and everything else to false
	if (render == false)
	{
		setEdgesRendered(0);
		return;
	}

	// Update edgesRendered vector
	for (int i = 0; i < _EDGES; i++)
	{
		edgesRendered[i] = render;
	}
}

// Set which _EDGES will be rendered
void Cuboid3D::setEdgesRendered(int edgeVal)
{
	// Set all _EDGES to be rendered to false
	for (int i = 0; i < _EDGES; i++)
	{
			edgesRendered[i] = false;
	}

	if ((edgeVal >= 0) && (edgeVal < _EDGES)) // Choose which edge is going to be rendered
	{
		edgesRendered[edgeVal] = true;
	}
	else
	{
		cout << "Not valid edge." << endl;
	}
}