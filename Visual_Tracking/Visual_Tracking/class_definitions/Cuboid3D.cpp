#include "..\headers\Cuboid3D.h"

// Cuboid3D class defintion.

// Constructor
Cuboid3D::Cuboid3D(float length, float height, float width)
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
		edgesRendered.push_back(true);
	}
	Nedges = EDGES;

	// SurfacesVisibility initializagtion
	for (int i = 0; i < SURFACES; i++)
	{
		surfacesVisibility.push_back(false);
		surfacesRendered.push_back(true);
	}
	Nsurfaces = SURFACES;

	primitives = 0;
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
	for (int i = 4; i < VERTICES; i++)
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

// Returns the dimensions of the Cuboid3D
Point3f Cuboid3D::getDimensions() const
{
	return Point3f(getLength(), getHeight(), getWidth());
}

// Returns a vertice of the Cuboid3D, in local coordinates
Point3f Cuboid3D::getVertice(int idx)
{
	// Check if idx is between 0..VERTICES=8
	return vertices[checkIdx(idx, VERTICES)];
}

// Returns a vector of Cuboid3D vertices, in local coordinates
vector <Point3f> Cuboid3D::getVertices() const
{
	return vertices;
}

// Returns the number of vertices
int Cuboid3D::getVerticesSize() const
{
	return VERTICES;
}

// Returns an edge of the Cuboid3D
vector <int> Cuboid3D::getEdge(int idx)
{
	// Check if idx is between 0..EDGES=12
	return edges[checkIdx(idx, EDGES)];
}

// Returns a vector of Cuboid3D edges
vector <vector <int>> Cuboid3D::getEdges() const
{
	return edges;
}

// Returns the number of edges
int Cuboid3D::getEdgesSize() const
{
	return EDGES;
}

// Returns a surface of the Cuboid3D
vector <int> Cuboid3D::getSurface(int idx)
{
	// Check if idx is between 0..SURFACES=6
	return surfaces[checkIdx(idx, SURFACES)];
}

// Returns a vector of Cuboid3D surfaces
vector <vector <int>> Cuboid3D::getSurfaces() const
{
	return surfaces;
}

// Returns the number of surfaces
int Cuboid3D::getSurfacesSize() const
{
	return SURFACES;
}

// Change the edge visibility
void Cuboid3D::setEdgeVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= EDGES))
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
	// Check if idx is between 0..EDGES=12
	return edgesVisibility[checkIdx(idx, EDGES)];
}

// Returns the vector of edges visibility
vector <bool> Cuboid3D::getEdgesVisibility() const
{
	return edgesVisibility;
}

// Change the surface visibility
void Cuboid3D::setSurfaceVisibility(int idx, bool visible)
{
	if ((idx < 0) || (idx >= SURFACES))
	{
		cout << "Invalid memory access." << endl;
		return;
	}

	if (surfacesRendered[idx] == true)
	{
		surfacesVisibility[idx] = visible;
	}
	else
	{
		surfacesVisibility[idx] = false;
	}
}

// Returns the surface visibility
bool Cuboid3D::getSurfaceVisibility(int idx)
{
	// Check if idx is between 0..SURFACES=6
	return surfacesVisibility[checkIdx(idx, SURFACES)];
}

// Returns the vector of surfaces visibility
vector <bool> Cuboid3D::getSurfacesVisibility() const
{
	return surfacesVisibility;
}

// Set which edges will be rendered
void Cuboid3D::setEdgesRendered()
{
	// Render all edges
	if (Nedges == EDGES) 
	{
		for (int i = 0; i < Nedges; i++)
		{
			edgesRendered[i] = true;
		}
	}
	else
	{
		// Set all edges to be rendered to false
		for (int i = 0; i < EDGES; i++)
		{
			edgesRendered[i] = false;
		}

		// Render edges belong only to one surface
		if (Nedges == 4)
		{
			// Find which edges belong to the surface which is going to be rendered
			for (int i = 0; i < SURFACES; i++)
			{
				if (surfacesRendered[i] == true)
				{
					vector <int> surface = getSurface(i);
					for (int j = 0; j < surface.size(); j++)
					{
						edgesRendered[surface[j]] = true;
					}
					break;
				}
			}
		}
	}
}

// Set which edges will be rendered
void Cuboid3D::setEdgesRendered(int edgeVal)
{
	if (Nedges == 1) // Render only one egde
	{
		// Set all edges to be rendered to false
		for (int i = 0; i < EDGES; i++)
		{
			edgesRendered[i] = false;
		}

		if ((edgeVal >= 0) && (edgeVal < EDGES)) // Choose which edge is going to be rendered
		{
			edgesRendered[edgeVal] = true;
		}
		else
		{
			cout << "Not valid edge." << endl;
		}
	}
}

// Set which surface will be rendered
void Cuboid3D::setSurfacesRendered()
{
	// Render all surfaces
	if (Nsurfaces == SURFACES)
	{
		for (int i = 0; i < Nsurfaces; i++)
		{
			surfacesRendered[i] = true;
		}
	}
	else if (Nsurfaces == 0)
	{
		// Set all surfaces to be rendered to false
		for (int i = 0; i < SURFACES; i++)
		{
			surfacesRendered[i] = false;
		}
	}
}

// Set which surface will be rendered
void Cuboid3D::setSurfacesRendered(int surfaceVal)
{
	if (Nsurfaces == 1) // Render only one surface
	{
		// Set all surfaces to be rendered to false
		for (int i = 0; i < SURFACES; i++)
		{
			surfacesRendered[i] = false;
		}

		if ((surfaceVal >= 0) && (surfaceVal < SURFACES)) // Choose which edge is going to be rendered
		{
			surfacesRendered[surfaceVal] = true;
		}
		else
		{
			cout << "Not valid surface." << endl;
		}
	}
}

// Set which type of primitives will be rendered
void Cuboid3D::setPrimitives(int primitiveVal)
{
	switch (primitiveVal)
	{
		case 0: // ALL
		{
			primitives = 0;
			Nsurfaces = SURFACES;
			Nedges = EDGES;
			setSurfacesRendered();
			setEdgesRendered();
			break;
		}
		case 1: // SURFACES
		{
			primitives = 1;
			Nsurfaces = 1;
			Nedges = 4;
			setSurfacesRendered(0);
			setEdgesRendered();
			break;
		}
		case 2: // EDGES
		{
			primitives = 2;
			Nsurfaces = 0;
			Nedges = 1;
			setSurfacesRendered();
			setEdgesRendered(0);
			break;
		}
		default:
		{
			cout << "Not valid type of primitives." << endl;
			break;
		}
	}
}

int Cuboid3D::checkIdx(int idx, int limit)
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
