#include "../headers/visualTracking.h"

// Visual tracker main function
void visualTracker(Cuboid3D &model, Cuboid3D &Data, int width, int height)
{
	// Initialization
	vector <State> state = { X, Y, Z, R1, R2, R3 };
	float fov = 60.f;
	Vec3f defaultAxisAngle = euler2AxisAngle(deg2rad(_RX), deg2rad(_RY), deg2rad(_RZ));
	vector <float> defaultParams = { _TX, _TY, _TZ, defaultAxisAngle.val[0], defaultAxisAngle.val[1], defaultAxisAngle.val[2], _MNUM };
	int maxIterations = 10; // Number of non linear fitting iterations
	bool exitFlag = false; // If true, exit application
	bool updateFlag = true; // If true render again cuboids
	bool fitFlag = false; // If true start non linear fitting
	bool demoFlag = false; // If true start demo
	int mNum = _MNUM;
	
	// Virtual camera initialization
	Camera virtualCam = createCam(Vec3f(_TX, _TY, _TZ), Vec3f(deg2rad(_RX), deg2rad(_RY), deg2rad(_RZ)), fov, width, height, state);
	// Real camera initialization	
	Camera realCam = createCam(Vec3f(_TX + 2, _TY, _TZ), Vec3f(deg2rad(_RX), deg2rad(_RY), deg2rad(_RZ)), fov, width, height, state);

	// Image plane WIDTHxHEIGHT _PIxels
	Mat imagePlane(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneModel(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneData(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	Mat dataImage(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	
	// Distance transform and it's first spatial derivatives
	Mat distTransform, dxDist, dyDist;
	// Pointer to a Cuboid2D object
	Cuboid2D *modelProjection;

#ifdef _LOG
	// Vector of errors for each fitting
	Mat err;
	// Matrix of virtual camera parameters for each iteration
	Mat x;
#endif

	while (!exitFlag)
	{
		// Render model
		modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlane, imagePlaneModel, "model"));
		//Cuboid2D modelProjection = rendering(model, virtualCam, imagePlane, imagePlaneModel, "model");
		// Render data
		Cuboid2D dataProjection = rendering(Data, realCam, imagePlane, imagePlaneData, "data");
		if (updateFlag) // Update image plane
		{
			//Update frame
			// Draw aliassed image
			dataImage = Mat(imagePlaneData.size(), dataImage.type(), CV_RGB(255, 255, 255));
			drawObj(dataProjection, dataImage, dataImage, Vec3b(0, 0, 0), 8);
			// Display camera parameters
			dispCamParams(virtualCam, realCam);

			// Calculate distance transform of data and it's first derivatives
			distTransform = computeDistanceTransform(dataImage);
			distTransformImageGradient(distTransform, dxDist, dyDist);
			// Display distance transform image
			displayImagePlane("Distance Transform of data image", normalise(distTransform));
		}

		// Disimillarity between data and model object
		Mat mijs, dijs;
		dissimilarity(modelProjection->getEdges(), imagePlane, dataImage, distTransform, mNum, mijs, dijs);
		// Display Model - Data dissimilarity
		displayImagePlane("Model - Data image plane", imagePlane);
		

		if (fitFlag) // Fit model to data
		{
#ifdef _COUNT_TIME
			auto start = chrono::high_resolution_clock::now();
#endif			
#ifdef _LOG
			x.push_back(convertSTLvector2Mat(virtualCam.getParams(), static_cast<int>(virtualCam.getParams().size()), 1, CV_32F).t());
#endif
			for (int i = 0; i < maxIterations; i++)
			{

				// Update image plane
				delete modelProjection;
				modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlane, imagePlaneModel, "model"));
			
				// Disimillarity between data and model object
				Mat mijs, dijs;
				dissimilarity(modelProjection->getEdges(), imagePlane, dataImage, distTransform, mNum, mijs, dijs);
#ifdef _LOG
				err.push_back(dijs.t() * dijs);
#endif

				// Compute first derivatives of 3D homogeneous projection model coordinates
				// in respect to the state parameters of the camera - extrinsics parameters
				Mat Jdijs = computeModelFirstDerivatives(model, *modelProjection, virtualCam, mijs, dxDist, dyDist);

				// Use non linear fitting to estimate new parameters for virtual camera
				vector <float> xNew = fittingGaussNewton(virtualCam, Jdijs, dijs);
				virtualCam.setParams(xNew, virtualCam.getStates());
#ifdef _LOG
				x.push_back(convertSTLvector2Mat(virtualCam.getParams(), static_cast<int>(virtualCam.getParams().size()), 1, CV_32F).t());
#endif
			}

#ifdef _COUNT_TIME
			auto finish = chrono::high_resolution_clock::now();
			chrono::duration<double> elapsed = finish - start; // Time elapsed between maxIterations
			cout << "Elapsed time: " << elapsed.count() << " s\n";
#endif
			fitFlag = false;
			updateFlag = true;

#ifdef _LOG // Update model to extract the last minimum error
			// Update image plane
			delete modelProjection;
			modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlane, imagePlaneModel, "model"));

			// Disimillarity between data and model object
			Mat mijs, dijs;
			dissimilarity(modelProjection->getEdges(), imagePlane, dataImage, distTransform, mNum, mijs, dijs);
			err.push_back(dijs.t() * dijs);
			exportFittingData(err, x);
			err.release();
			x.release();
#endif

#ifdef _DEMO
			if (demoFlag) // Demo
			{
				demoFlag = demo(realCam);
				fitFlag = true;
				updateFlag = true;
			}				
#endif
		}
		else
		{
			// Keys pressed handler
			keyboardHandler(virtualCam, realCam, model, Data, mNum, defaultParams, exitFlag, updateFlag, fitFlag, demoFlag);
		}
	}
}

// Read model data from .x file
void modelData(float &length, float &height, float &width)
{
	string filename = "cuboid.x";
	// Check if input or the file are empty
	checkFile(filename);
	ifstream file(filename);
	
	// Read Cuboid3D dimensions from .x file
	string line;
	while (getline(file, line))
	{
		size_t prev = 0;
		size_t next = 0;
		vector <String> data;

		while ((next = line.find_first_of(" ", prev)) != string::npos)
		{
			if (next - prev != 0)
			{
				data.push_back(line.substr(prev, next - prev));
			}
			prev = next + 1;
		}
		if (prev < line.size())
		{
			data.push_back(line.substr(prev));
		}

		// Extract 3D cuboid dimensions
		if (!(data[0].compare("length")))
		{
			length = stof(data[1]);
		}
		else if (!(data[0].compare("height")))
		{
			height = stof(data[1]);
		}
		else if (!(data[0].compare("width")))
		{
			width = stof(data[1]);
		}
	}
}

// Render function
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera &camera, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	// Initialization
	Mat img(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	img.copyTo(imagePlaneObj);
	if (!(type.compare("model")))
	{
		img.copyTo(imagePlane);
	}
	vector <Point3f> homogeneousVertices;

	// Reset visibility
	resetVisibility(cuboid3D);

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics(static_cast<Rotation>(rotation3Dtype()));

	// Project _VERTICES to image plane
	for (int i = 0; i < cuboid3D.getVerticesSize(); i++)
	{
		homogeneousVertices.push_back(perspectiveProjection(cuboid3D.getVertex(i), P));
	}

	// Create cuboid2d - projection of cuboid3d
	Cuboid2D objProjection(homogeneousVertices);
	
	// Object visibility culling
	visibilityCulling(cuboid3D, objProjection, camera, imagePlane.size());

	// Render object
	Vec3b colour = Vec3b(0, 0, 0);
	if (!(type.compare("data")))
	{
		colour.val[2] = 255;
	}
	drawObj(objProjection, imagePlane, imagePlaneObj, colour, CV_AA);

	return objProjection;
}

// Dissimilarity between data and model object
void dissimilarity(vector <vector <Point2f>> edgesVertices, Mat &imagePlane, Mat dataImage, Mat distTransform, int mNum, Mat &mijs, Mat &dijs)
{
	// Initialization
	float offset = 10.f;

	// Construct edges vectors
	Mat edgesVec = edgesVectors(edgesVertices);

	// Calculate edges normals vectors
	Mat edgesNormals = edgesNormalVectors(edgesVec);

	// Calculate edges subintervals
	mijs = edgesSubIntervals(edgesVec, edgesVertices, mNum);

	// Draw mijs
	drawMijs(mijs, imagePlane);

	// Calculate dijs
	dijs = calculateDistance(mijs, distTransform);
}

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, Vec3b colour, int lineType)
{
	if (imagePlane.size() != imagePlaneObj.size())
	{
		errorSize("imagePlane", "imagePlaneObj", imagePlane.size(), imagePlaneObj.size(), __FILE__, __LINE__ - 4);
	}

	for (int i = 0; i < objProjection.getEdgesSize(); i++)
	{
		Point2i p1(objProjection.getEdge(i)[0]);
		Point2i p2(objProjection.getEdge(i)[1]);
		// Draw _VERTICES
		imagePlane.at<Vec3b>(p1.y, p1.x) = colour;
		imagePlane.at<Vec3b>(p2.y, p2.x) = colour;
		imagePlaneObj.at<Vec3b>(p1.y, p1.x) = colour;
		imagePlaneObj.at<Vec3b>(p2.y, p2.x) = colour;
		// Draw edges
		line(imagePlane, p1, p2, colour, 1, lineType, 0);
		line(imagePlaneObj, p1, p2, colour, 1, lineType, 0);
	}
}

// Check which parts of the object are visible from the camera's given viewpoint
void visibilityCulling(Cuboid3D &cuboid3D, Cuboid2D &cuboid2D, Camera camera, Size size)
{
	int edge;
	vector <int> surfaceVertices, surfaceEdges, edgeVertices;
	vector <Point2f> pointsPxl;

	// Check if the object is in front of the camera
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		surfaceVertices = cuboid3D.getSurfaceVertices(i);
		if (!frontCameraVisibilty(surfaceVertices, cuboid3D.getVertices(), camera))
		{
			return;
		}
		surfaceVertices.clear();
	}

	// Visibility culling
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		// Check if the surface is facing the camera
		surfaceVertices = cuboid3D.getSurfaceVertices(i);
		if (backFaceCulling(surfaceVertices, cuboid3D.getVertices(), camera.getPosition()))
		{
			// Surface i is visible
			cuboid3D.setSurfaceVisibility(i, true);
			cuboid2D.setSurface(surfaceVertices);
			surfaceEdges = cuboid3D.getSurfaceEdges(i);
			for (int j = 0; j < surfaceEdges.size(); j++)
			{
				// Egde j is visible
				edge = surfaceEdges[j];
				edgeVertices = cuboid3D.getEdge(edge);
				pointsPxl.push_back(cuboid2D.getVertexPxl(edgeVertices[0]));
				pointsPxl.push_back(cuboid2D.getVertexPxl(edgeVertices[1]));
				// If rendered == true and current edge belongs to the image plane
				if (cuboid3D.getEdgeVisibility(edge) && edgeClip(pointsPxl, size))
				{
					cuboid2D.setEdge(pointsPxl, edgeVertices);
				}
				pointsPxl.clear();
				edgeVertices.clear();
			}
		}
		surfaceEdges.clear();
		surfaceVertices.clear();
	}
}

// Edge clip_PIng
bool edgeClip(vector <Point2f> &edgePxl, Size size)
{
	// Initialization
	Point2i p1(edgePxl[0]), p2(edgePxl[1]);

	// Clip lines inside the image plane
	if (clipLine(size, p1, p2))
	{
		// Check if edge has been clipped (euclidean distance > 1.f)  or if the original points of the edge are near the image plane limits
		if ((abs(static_cast<float>(p1.x) - edgePxl[0].x) > 1.f) || (edgePxl[0].x < 0.f) || (edgePxl[0].x > static_cast<float>(size.width - 1)))
		{
			edgePxl[0].x = static_cast<float>(p1.x);
		}

		if ((abs(static_cast<float>(p1.y) - edgePxl[0].y) > 1.f) || (edgePxl[0].y < 0.f) || (edgePxl[0].y > static_cast<float>(size.height - 1)))
		{
			edgePxl[0].y = static_cast<float>(p1.y);
		}

		if ((abs(static_cast<float>(p2.x) - edgePxl[1].x) > 1.f) || (edgePxl[1].x < 0.f) || (edgePxl[1].x > static_cast<float>(size.width - 1)))
		{
			edgePxl[1].x = static_cast<float>(p2.x);
		}

		if ((abs(static_cast<float>(p2.y) - edgePxl[1].y) > 1.f) || (edgePxl[1].y < 0.f) || (edgePxl[1].y > static_cast<float>(size.height - 1)))
		{
			edgePxl[1].y = static_cast<float>(p2.y);
		}
	}
	else
	{
		return false;
	}
	
	return true;
}

// Check if a surface is visible from the camera's viewpoint
bool backFaceCulling(vector <int> surface, vector <Point3f> vertices, Point3f t)
{
	// Vectors begining from down right vertex of surface
	Vec3f vec1 = vertices[surface[0]] - vertices[surface[1]];
	Vec3f vec2 = vertices[surface[2]] - vertices[surface[1]];
	// Normal vector of surface
	Vec3f normal = vec2.cross(vec1);
	normal = normal / norm2(normal);
	// Vector between camera position and down right vertex of surface
	Vec3f look_vector = t - vertices[surface[1]];
	look_vector = look_vector / norm2(look_vector);
	// Angle between look vector and normal of surface
	float theta = acos(normal.dot(look_vector)) * 180.f / _PI;

	// If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta < 90.f)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Front camera visibility
bool frontCameraVisibilty(vector <int> surface, vector <Point3f> vertices, Camera camera)
{
	Vec3f vw;
	Mat e3 = (Mat_<float>(3, 1) << 0.f, 0.f, 1.f);
	// Check if the distance of the object to the camera is smaller
	// than the distance of the image plane of the camera to the camera - focal length in metric units
	for (int i = 0; i < surface.size(); i++)
	{
		vw = vertices[surface[i]];
		Mat nw = camera.getRotation(static_cast<Rotation>(rotation3Dtype())) * e3;
		Mat xw = Mat(vw - camera.getPosition());
		float d = static_cast<float>(xw.dot(nw));
		if (d <= camera.getFocalMetric())
		{
			return false;
		}
	}

	return true;
}

// Fixed time frame update
Cuboid2D updateFrame(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, Mat &imagePlane, Mat &imagePlaneModel, Mat &imagePlaneData, Mat &dataImg)
{
	// Render model
	Cuboid2D modelProjection = rendering(model, virtualCam, imagePlane, imagePlaneModel, "model");

	// Render data
	Cuboid2D dataProjection = rendering(Data, realCam, imagePlane, imagePlaneData, "data");

	// Draw aliassed image
	dataImg = Mat(dataImg.size(), dataImg.type(), CV_RGB(255, 255, 255));
	drawObj(dataProjection, dataImg, dataImg, Vec3b(0, 0, 0), 8);

	// Display camera parameters
	dispCamParams(virtualCam, realCam);

	return modelProjection;
}

// Create camera
Camera createCam(Vec3f t, Vec3f r, float fov, int width, int height, vector <State> state)
{
	// Camera Intrinsics
	float u0 = static_cast<float>(width) / 2.f, v0 = static_cast<float>(height) / 2.f; // Principal point - center of image plane
	float focal = (static_cast<float>(width) / 2.f) / (tan((fov / 2.f) * _PI / 180.f)); // Focal length

	return Camera(t, r, Point2f(u0, v0), fov, focal, state, static_cast<Rotation>(rotation3Dtype()));
}

// Clear visibility of the 3D model
void resetVisibility(Cuboid3D &cuboid3D)
{
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		cuboid3D.setSurfaceVisibility(i, false);
	}
}

// Display image plane
void displayImagePlane(string windowName, Mat imagePlane)
{
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);
	// waitKey(1);
}

// Convert 3D _VERTICES and parameters values to Mat, states to enum Parameters and extract intrisincs matrix
void extractDataForDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat &V, Mat &Vph, Mat &K, Mat &x, vector <Parameter> &xk)
{
	// Cuboi3D _VERTICES in 3D cartesian coordinates
	V = convertSTLvector2Mat(model.getVertices(), model.getVerticesSize(), 3, CV_32F);
	// Cuboid2D _VERTICES in 3D homogeneous projection coordinates
	Vph = convertSTLvector2Mat(modelProjection.getHomogeneousVertices(), static_cast<int>(modelProjection.getHomogeneousVertices().size()), 3, CV_32F);
	// Virtual camera intrinscs matrix
	K = virtualCam.getIntrinsics();
	// Virtual camera state parameters values
	x = convertSTLvector2Mat(virtualCam.getParams(), static_cast<int>(virtualCam.getParams().size()), 1, CV_32F);
	// Virtual camera state parameters
	for (int i = 0; i < virtualCam.getStateSize(); i++)
	{
		xk.push_back(static_cast<Parameter>(virtualCam.getState(i)));
	}
}

// Convert STL vector to OpenCV Mat
template <typename T>
Mat convertSTLvector2Mat(vector <T> vec, int rows, int cols, int type)
{
	int size = rows;
	Mat M = Mat(rows, cols, type);
	memcpy(M.data, vec.data(), size * sizeof(T));

	return M;
}

// Calculate edges vectors
Mat edgesVectors(vector <vector <Point2f>> edges)
{
	Mat edgesVec(static_cast<int>(edges.size()), 1, CV_32FC2);

	// Calculate each edge vector
	for (int i = 0; i < edgesVec.rows; i++)
	{
		vector <Point2f> edge = { edges[i] };
		edgesVec.at<Vec2f>(i, 0) = edge[1] - edge[0];
	}

	return edgesVec;
}

// Calculate edges direction vectors
Mat edgesDirectionVectors(vector <vector <Point2f>> edges)
{
	Mat edgesDirection(static_cast<int>(edges.size()), 1, CV_32FC2);

	// Calculate each edge vector
	for (int i = 0; i < edgesDirection.rows; i++)
	{
		vector <Point2f> edge = { edges[i] };
		edgesDirection.at<Vec2f>(i, 0) = (edge[1] - edge[0]) / norm2(edge[1] - edge[0]);
	}

	return edgesDirection;
}

// Calculate edges normal unit vectors
Mat edgesNormalVectors(Mat edgesVec)
{
	Mat edgesNormals(edgesVec.rows, 1, CV_32FC2);

	// Calculate normal vector for each edge
	for (int i = 0; i < edgesNormals.rows; i++)
	{
		Vec2f edgeVec = edgesVec.at<Vec2f>(i, 0);
		Vec2f edgeNormal(-edgeVec.val[1], edgeVec.val[0]);
		edgesNormals.at<Vec2f>(i, 0) = edgeNormal / norm2(edgeNormal);
	}

	return edgesNormals;
}

// Calculate the subintervals mijs for all edges
Mat edgesSubIntervals(Mat edgesVec, vector <vector <Point2f>> edgesVertices, int mNum)
{
	Mat mijs(mNum - 2, edgesVec.rows, CV_32FC2);
	int first = 1, last = mNum - 1;
	
	if (edgesVec.rows != static_cast<int>(edgesVertices.size()))
	{
		errorSize("edgesVector", "edgesVertices", edgesVec.rows, static_cast<int>(edgesVertices.size()), __FILE__, __LINE__ - 7);
	}

	for (int i = 0; i < mijs.cols; i++)
	{
		Vec2f edgeVec = edgesVec.at<Vec2f>(i, 0);
		vector <Point2f> edgeVertices = { edgesVertices[i] };
		for (int j = first; j < last; j++)
		{
			mijs.at<Vec2f>(j - 1, i) = static_cast<Vec2f>(edgeVertices[0]) + static_cast<float>(j) * edgeVec / static_cast<float>(mNum - 1);
		}
	}

	return mijs;
}

// Calculate for normal vectors for all mijs
Mat subIntervalsNormals(Mat mijs, Mat edgesNormals, float offset, Size size)
{
	Mat mijsNormals(mijs.size(), CV_32FC4);

	if (mijs.cols != edgesNormals.rows)
	{
		errorSize("mijs", "edgesNormals", mijs.cols, edgesNormals.rows, __FILE__, __LINE__ - 6);
	}

	for (int i = 0; i < mijsNormals.rows; i++)
	{
		for (int j = 0; j < mijsNormals.cols; j++)
		{
			vector <Point2f> normalLine;
			normalLine.push_back(mijs.at<Point2f>(i, j) + offset * edgesNormals.at<Point2f>(j, 0));
			normalLine.push_back(mijs.at<Point2f>(i, j) - offset * edgesNormals.at<Point2f>(j, 0));
			edgeClip(normalLine, size);
			mijsNormals.at<Vec4f>(i, j) = Vec4f(normalLine[0].x, normalLine[0].y, normalLine[1].x, normalLine[1].y);
		}
	}

	return mijsNormals;
}

// Draw mijs and mijs normal lines on each edge of the model
void drawMijs(Mat mijs, Mat &imagePlane)
{
	Vec3b colour = Vec3b(0, 0, 0);

	for (int j = 0; j < mijs.cols; j++)
	{
		for (int i = 0; i < mijs.rows; i++)
		{
			circle(imagePlane, mijs.at<Point2f>(i, j), 2, colour, -1, 8);
		}
	}
}

// Compute the distance transform for the data object
Mat computeDistanceTransform(Mat dataImage)
{
	// Create binary data image
	Mat binaryImage;
	cvtColor(dataImage, binaryImage, CV_BGR2GRAY);
	
	// Calculate distance trasnform
	Mat distTransform;
	distanceTransform(binaryImage, distTransform, CV_DIST_L2, 3);
	
	return distTransform;
}

// Convert to display distance transform
Mat normalise(Mat Img)
{
	Mat normaliseImg(Img.size(), CV_32FC1);
	double min, max;
	float offset = 0.f;
	minMaxLoc(Img, &min, &max);
	if (min < 0.0)
	{
		offset = static_cast<float>(-min);
		max += -min;
	}
	normaliseImg = (Img + offset) / max;
		
	return normaliseImg;
}

// Calculate distance from each mij to data
Mat calculateDistance(Mat mijs, Mat distTransform)
{
	Mat dijs(mijs.rows * mijs.cols, 1, CV_32FC1);
	int x, y;

	for (int j = 0; j < mijs.cols; j++)
	{
		for (int i = 0; i < mijs.rows; i++)
		{
			x = static_cast<int>(round(mijs.at<Point2f>(i, j).y));
			y = static_cast<int>(round(mijs.at<Point2f>(i, j).x));
			dijs.at<float>((j * mijs.rows) + i, 0) = distTransform.at<float>(x, y);
		}
	}

	return dijs;
}

// Compute distance transform gradient
void distTransformImageGradient(Mat distTransform, Mat &dxdist, Mat &dydist)
{
	dxdist = imageGradient(distTransform, Dx);

	dydist = imageGradient(distTransform, Dy);
}

// Compute dijs first derivatives
Mat computeModelFirstDerivatives(Cuboid3D model, Cuboid2D modelProjection, Camera virtualCam, Mat mijs, Mat dxDist, Mat dyDist)
{
	Mat V, Vph, K, x;
	vector <Parameter> xk;
	extractDataForDerivatives(model, modelProjection, virtualCam, V, Vph, K, x, xk);
	Mat Jvph = jacobianPerspectiveProjection(V, K, x, xk);
	// Compute first derivatives of 2D projection model _PIxel coordinates
	Mat Jvp = jacobianPixelCoordinates(Vph, Jvph);
	// Compute first derivatives of 2D projection model edges
	Mat Jep = jacobianEdges(Jvp, modelProjection.getEdgesPtr());
	// Compute first derivatives of mijs
	Mat Jmijs = jacobianMijs(Jep, mijs.rows + 2);
	// Compute first derivatives of distances dijs
	Mat Jdijs = jacobianDijs(mijs, Jmijs, dxDist, dyDist);

	return Jdijs;
}

// Gauss - Newton non linear fitting
vector <float> fittingGaussNewton(Camera virtualCam, Mat Jdijs, Mat dijs)
{
	vector <float> params = virtualCam.getParams();
	vector <State> states = virtualCam.getStates();
	Mat xNew;
	Mat x = convertSTLvector2Mat(params, static_cast<int>(params.size()), 1, CV_32F);
	Mat Jinv;
	invert(Jdijs.t() * Jdijs, Jinv); // Pseudo inverse
	Mat err = Jinv * Jdijs.t() * dijs; // error

	Vec3f eulerAngles = axisAngle2euler(Vec3f(x.at<float>(3, 0), x.at<float>(4, 0), x.at<float>(5, 0)));
	Vec3f eulerAngles_err = axisAngle2euler(Vec3f(err.at<float>(3, 0), err.at<float>(4, 0), err.at<float>(5, 0)));
	Vec3f eulerAngles_New = eulerAngles - eulerAngles_err;
	Vec3f axisAngle = euler2AxisAngle(eulerAngles_New.val[0], eulerAngles_New.val[1], eulerAngles_New.val[2]);
	
	xNew = x - err; // Remove error from parameters
	xNew.at<float>(3, 0) = axisAngle.val[0];
	xNew.at<float>(4, 0) = axisAngle.val[1];
	xNew.at<float>(5, 0) = axisAngle.val[2];

	return xNew;
}

// Dimensions of inputs, are not equal
template <typename T>
void errorSize(string input1, string input2, T size1, T size2, string filename, int line)
{
	int pos = static_cast<int>(filename.find_last_of("\\"));
	
	cout << input1 << " and " << input2 << " matrices must have the same size; " << endl;
	cout << input1 << ".size = " << size1 << " != " << input2 <<".size = " << size2 << endl;
	cout << "File: " << filename.substr(pos + 1, string::npos) << "; Line: " << line << endl;
	system("PAUSE");
	exit(EXIT_FAILURE);
}

// Demo
bool demo(Camera &realCam)
{
	static int iter = 0;
	int maxIter = 280;
	
	++iter;
	if (iter == maxIter)
	{// stop demo
		iter = 0;
		return false;
	}
	else
	{// Move realCam
		
		vector <float> params = realCam.getParams();
		Vec3f eulerAngles = axisAngle2euler(Vec3f(params[3], params[4], params[5]));
		if (iter < (maxIter / 2))
		{
			params[0] -= 0.5f;
		}
		else
		{
			params[0] -= 0.5f;
			eulerAngles.val[1] -= deg2rad(0.75f);
			params[1] += 0.25f;
			eulerAngles.val[2] -= deg2rad(0.25f);
			params[2] -= 0.25f;
		}
		Vec3f axisAngle = euler2AxisAngle(eulerAngles.val[0], eulerAngles.val[1], eulerAngles.val[2]);
		params[3] = axisAngle.val[0];
		params[4] = axisAngle.val[1];
		params[5] = axisAngle.val[2];

		realCam.setParams(params, { X, Y, Z, R1, R2, R3 });
		
		return true;
	}
}

// Export fitting data
void exportFittingData(Mat m, Mat x)
{
	ofstream out;
	out.open("data/fitting_data.txt", ofstream::out | ofstream::app);
	if (!out.is_open())
	{
		cout << "Problem with opening the file." << endl;
		return;
	}
	string sM = "E = [";
	string sX = "X = [";
	if (x.rows == m.rows)
	{
		for (int i = 0; i < m.rows; i++)
		{
			ostringstream convert1;
			convert1 << m.at<float>(i, 0);
			sM += convert1.str();
			for (int j = 0; j < x.cols; j++)
			{
				ostringstream convert2;
				convert2 << x.at<float>(i, j);
				sX += convert2.str();
				if (j < (x.cols - 1))
				{
					sX += " ";
				}
			}
			if (i < (m.rows - 1))
			{
				sM += " ";
				sX += ";\n";
			}
		}
	}
	else
	{
		cout << "The two input matrices must have the same number of rows." << endl;
		out.close();
		return;
	}
	sM += "];";
	sX += "];";
	out << sM;
	out << endl;
	out << sX;
	out << endl;
	out.flush();
	out.close();
}