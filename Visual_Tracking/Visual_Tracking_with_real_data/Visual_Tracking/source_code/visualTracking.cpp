#include "../headers/visualTracking.h"

// Chosen 2D points from image plane, using mouse left click
vector <Point2f> points2DMouse;

string _VIDEO_DIR = "data/cuboid3/";
string _CONFIG_DIR = "config/cuboid3/";
string _MODEL_DIR = "models/";

// Visual tracker main function
void visualTracker(Cuboid3D &model)
{
	// Initialization
	vector <State> state = { X, Y, Z, R1, R2, R3 };
	float tX, tY, tZ, RX, RY, RZ, fov = 62.3f; // Camera parameters
	int imageWidth, imageHeight; // Image plane parameters
	int maxIterations, mNum; // Fitting parameters
	float threshold, ratio; int kernel; // Canny edge parameters
	string configParamsFilename = "config_params.txt"; // Miscellaneous parameters
	string modelFilename = "cuboid3.x";  // 3D model
	string srcImageFilename = "frame15.jpg"; // Source image
	string videoFilename = "video15.mp4"; // Source video
	Mat srcData = imread(_VIDEO_DIR + srcImageFilename); // Cube image
	string configCameraFilename = "video15.txt";	// Camera parameters
	configCameraData(configCameraFilename, tX, tY, tZ, RX, RY, RZ); // Read camera parameters
	configParamsData(configParamsFilename, imageWidth, imageHeight, mNum, maxIterations, threshold, ratio, kernel); // Miscellaneous parameters
	
	// Default parameters
	Vec3f defaultAxisAngle = euler2AxisAngle(deg2rad(RX), deg2rad(RY), deg2rad(RZ));
	vector <float> defaultParams = { tX, tY, tZ, defaultAxisAngle.val[0], defaultAxisAngle.val[1], defaultAxisAngle.val[2], static_cast<float>(mNum) };

	// Flags
	bool exitFlag = false; // If true, exit application
	bool updateModelFlag = true; // If true render model
	bool fitFlag = false; // If true start non linear fitting
	bool readFilesFlag = false; // Read again model .x and parameters .txt files
	bool updateFilenamesFlag = false; // Change filenames
	bool videoFlag = false; // Fitting on video
	bool writeFlag = false; // Write camera parameters
	bool captureFlag = false; // Write capture image
	bool poseEstimationFlag = false; // Pose estimation through corresponding 2D and 3D points
	
	// Model colour
	Vec3b modelColour = Vec3b(0, 255, 0);

	// Virtual camera initialization
	Camera virtualCam = createCam(Vec3f(tX, tY, tZ), Vec3f(deg2rad(RX), deg2rad(RY), deg2rad(RZ)), fov, imageWidth, imageHeight, state);
	
	// Distance transform and it's first spatial derivatives
	Mat distTransform, dxDist, dyDist;
	// Pointer to a Cuboid2D object
	Cuboid2D *modelProjection;
	
	// Image plane WIDTHxHEIGHT pixels
	Mat imagePlane(Size(imageWidth, imageHeight), CV_8UC3, CV_RGB(255, 255, 255));
	// Cube image
	Mat cube;
	// Inverted binary 
	Mat cubeEdgesInv;

	while (!exitFlag)
	{
#ifdef _COUNT_TIME
		auto start = chrono::high_resolution_clock::now();
#endif			
		// Render src data image
		// Resize src image to _HEIGHT and _WIDTH of the image plane of the virtual camera
		resize(srcData, cube, Size(imageWidth, imageHeight));
		cube.copyTo(imagePlane);
		
		// Detect data object edges
		Mat cubeEdges = detectEdges(cube, threshold, ratio, kernel);
		// Invert binary image for distance transform
		cubeEdgesInv.release();
		cubeEdgesInv = invertBinaryImage(cubeEdges);
		// Calculate distance transform of data and it's first derivatives
		distanceTransform(cubeEdgesInv, distTransform, CV_DIST_L2, 3);
		distTransformImageGradient(distTransform, dxDist, dyDist);
		
		if (updateModelFlag) // Update model
		{
			// Render model
			modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlane, modelColour));
			// Disimillarity between data and model object
			dissimilarity(modelProjection->getEdges(), imagePlane, cubeEdgesInv, distTransform, mNum);
		}		  	

		if (fitFlag) // Fit model to data
		{
			for (int i = 0; i < maxIterations; i++)
			{
				// Update model
				Mat imagePlaneModel(Size(imageWidth, imageHeight), CV_8UC3, CV_RGB(255, 255, 255));
				delete modelProjection;
				modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlaneModel, modelColour));
			
				// Disimillarity between data and model object
				Mat mijs, dijs;
				dissimilarity(modelProjection->getEdges(), imagePlaneModel, cubeEdgesInv, distTransform, mNum, mijs, dijs);
				// Compute first derivatives of 3D homogeneous projection model coordinates
				// in respect to the state parameters of the camera - extrinsics parameters
				Mat Jdijs = computeModelFirstDerivatives(model, *modelProjection, virtualCam, mijs, dxDist, dyDist);
				// Use non linear fitting to estimate new parameters for virtual camera
				vector <float> xNew = fittingGaussNewton(virtualCam, Jdijs, dijs);
				virtualCam.setParams(xNew, virtualCam.getStates());
			}
			// Render src data image
			cube.copyTo(imagePlane);
			// Render model
			delete modelProjection;
			modelProjection = new Cuboid2D(rendering(model, virtualCam, imagePlane, modelColour));
			dissimilarity(modelProjection->getEdges(), imagePlane, cubeEdgesInv, distTransform, mNum);
			fitFlag = false;
		}

		// Display Model - Data dissimilarity
		displayImagePlane(srcImageFilename, imagePlane);

#ifdef _DEBUG
		// Display cube edges
		displayImagePlane("Canny edges", cubeEdges);
		// Display distance tranform
		displayImagePlane("Distance Transform", normalise(distTransform));
		// Display dxDist
		displayImagePlane("Dx Distance Transform", dxDist);
		// Display dyDist
		displayImagePlane("Dy Distance Transform", dyDist);
		// Display virtual camera parameters
		dispCamParams(virtualCam);
		// Set the callback function for left click event
		setMouseCallback("Model - Data image plane - " + srcImageFilename, CallBackFunc, (void*)&points2DMouse);
#endif // _DEBUG

#ifdef _COUNT_TIME
		auto finish = chrono::high_resolution_clock::now();
		chrono::duration<double> elapsed = finish - start; // Time elapsed between maxIterations
		cout << "Elapsed time: " << elapsed.count() << " s\n";
#endif // _COUNT_TIME

		// Keys pressed handler
		keyboardHandler(virtualCam, model, mNum, defaultParams, exitFlag, updateModelFlag, fitFlag, readFilesFlag, updateFilenamesFlag, videoFlag, writeFlag, captureFlag, poseEstimationFlag);

		// Write captured image to disk
		if (captureFlag)
		{
			string imageCaptured;
			cout << "Name of captured image: ";
			cin >> imageCaptured;
			imwrite("data/exported_images/" + imageCaptured, imagePlane);
			cout << "Image saved." << endl;
			captureFlag = false;
		}

		// Write camera parameters
		if (writeFlag)
		{
			cout << "Name of camera parameters file: ";
			cin >> configCameraFilename;
			vector <float> parameters = virtualCam.getParams();
			writeCamParams(configCameraFilename, parameters);
			writeFlag = false;
			readFilesFlag = true;
		}
		// Calculate object orientation and translation
		// from 2D and 3D corresponding points
		if (poseEstimationFlag)
		{
			// Select 2D projected points and their 3D corresponding model points
			vector <Point3f> modelPoints;
			vector <Point2f> projectedPoints;
			get2D_3DCorrespondingPoints(&model.getVertices(), &modelPoints, &projectedPoints);
			// Estimate position and orientation of the object
			Mat R; Vec3f t;
			//poseEstimation2D_3D(&modelPoints, &projectedPoints, &R, &t);
			poseEstimationFlag = false;
		}

		// Play video
		if (videoFlag)
		{
			playVideo(videoFilename, model, defaultParams, fov, imageWidth, imageHeight, maxIterations, threshold, ratio, kernel);
			videoFlag = false;
		}

		// Update filenames
		if (updateFilenamesFlag)
		{
			readFilenames(srcImageFilename, videoFilename, configParamsFilename, modelFilename, configCameraFilename, srcData);
			updateFilenamesFlag = false;
		}

		// Update model and parameters
		if (readFilesFlag)
		{
			// Read from .x file
			float length, height, width;
			modelData(modelFilename, length, height, width);
			// Update model dimensions
			model.setLength(length); model.setHeight(height); model.setWidth(width);
			// Read camera parameters
			configCameraData(configCameraFilename, tX, tY, tZ, RX, RY, RZ);
			// Miscellaneous parameters
			configParamsData(configParamsFilename, imageWidth, imageHeight, mNum, maxIterations, threshold, ratio, kernel);
			// Default parameters
			Vec3f defaultAxisAngle = euler2AxisAngle(deg2rad(RX), deg2rad(RY), deg2rad(RZ));
			defaultParams = { tX, tY, tZ, defaultAxisAngle.val[0], defaultAxisAngle.val[1], defaultAxisAngle.val[2], static_cast<float>(mNum) };
			virtualCam.setParams({ defaultParams.begin(), defaultParams.end() - 1 }, state);
			cout << "Model, camera, image plane and fitting parameters are updated." << endl;
			destroyAllWindows();
			readFilesFlag = false;
		}
	}
}

// Read model data from .x file
void modelData(string filename, float &length, float &height, float &width)
{
	// Check if input or the file are empty
	string dir = _MODEL_DIR;
	if (!checkFileModel(dir + filename))
	{
		return;
	}
	ifstream file(dir + filename);
	
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

// Read from .txt file camera
void configCameraData(string configFilename, float &tX, float &tY, float &tZ, float &RX, float &RY, float &RZ)
{
	// Check if input or the file are empty
	string dir = _CONFIG_DIR;
	string filename = dir + configFilename;
	if (!checkFileConfig(filename))
	{
		return;
	}
	ifstream file(filename);

	// Read parameters from .txt file
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

		// Extract camera parameters
		if (!(data[0].compare("TX")))
		{
			tX = stof(data[1]);
		}
		else if (!(data[0].compare("TY")))
		{
			tY = stof(data[1]);
		}
		else if (!(data[0].compare("TZ")))
		{
			tZ = stof(data[1]);
		}
		else if (!(data[0].compare("RX")))
		{
			RX = stof(data[1]);
		}
		else if (!(data[0].compare("RY")))
		{
			RY = stof(data[1]);
		}
		else if (!(data[0].compare("RZ")))
		{
			RZ = stof(data[1]);
		}
	}
}

// Miscellaneous parameters	- image plane, fitting and Canny edge
void configParamsData(string configParamsFilename, int &imageWidth, int &imageHeight, int &mNum, int &maxIterations, float &threshold, float &ratio, int &kernel)
{
	// Check if input or the file are empty
	string dir = _CONFIG_DIR;
	string filename = dir + configParamsFilename;
	if (!checkFileConfig(filename))
	{
		return;
	}
	ifstream file(filename);

	// Read parameters from .txt file
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

		if (!(data[0].compare("WIDTH")))
		{// Image plane parameters
			imageWidth = stoi(data[1]);
		}
		else if (!(data[0].compare("HEIGHT")))
		{
			imageHeight = stoi(data[1]);
		}
		else if (!(data[0].compare("mNum")))
		{// Fitting parameters
			mNum = stoi(data[1]);
		}
		else if (!(data[0].compare("maxIterations")))
		{
			maxIterations = stoi(data[1]);
		}
		if (!(data[0].compare("threshold")))
		{// Canny edge parameters
			threshold = stof(data[1]);
		}
		if (!(data[0].compare("ratio")))
		{
			ratio = stof(data[1]);
		}
		if (!(data[0].compare("kernel")))
		{
			kernel = stoi(data[1]);
		}
	}
}

// Update filenames
void readFilenames(string &srcImageFilename, string &videoFilename, string &configParamsFilename, string &modelFilename, string &configCameraFilename, Mat &srcData)
{
	char choice[] = "\0";
	bool check = false;
	// Source image filename
	while (!check)
	{
		cout << "Source image file is: " << srcImageFilename << "; do you want to give new file? [Y/N]";
		cin >> choice;
		if (!strcmp("Y", choice))
		{
			cout << "Give new source image file: ";
			string tmpFile;
			cin >> tmpFile;
			Mat tmpMat = imread(_VIDEO_DIR + tmpFile);
			if (tmpMat.data == NULL)
			{
				cout << "Error opening image: " << tmpFile << endl;
				check = false;
			}
			else
			{
				srcImageFilename = tmpFile;
				srcData = imread(_VIDEO_DIR + srcImageFilename);
				check = true;
			}
		}
		else if (strcmp("N", choice) != 0)
		{
			cout << "Invalid option; Try again." << endl;
			check = false;
		}
		else
		{
			check = true;
		}
	}

	// Source video filename
	check = false;
	while (!check)
	{
		cout << "Source video file is: " << videoFilename << "; do you want to give new file? [Y/N]";
		cin >> choice;
		if (!strcmp("Y", choice))
		{
			cout << "Give new source video file: ";
			string tmpFile;
			cin >> tmpFile;
			// Check if camera opened successfully
			VideoCapture cap(_VIDEO_DIR + tmpFile);
			if (!cap.isOpened()) 
			{
				cout << "Error opening video stream: " << _VIDEO_DIR + tmpFile << endl;
				check = false;
			}
			else
			{
				videoFilename = tmpFile;
				check = true;
			}
		}
		else if (strcmp("N", choice) != 0)
		{
			cout << "Invalid option; Try again." << endl;
			check = false;
		}
		else
		{
			check = true;
		}
	}

	// Model
	check = false;
	while (!check)
	{
		cout << "Model file is: " << modelFilename << "; do you want to give a new file? [Y/N]";
		cin >> choice;
		if (!strcmp("Y", choice))
		{
			cout << "Give new model file: ";
			string tmpFile;
			cin >> tmpFile;
			if (!checkFileModel(_MODEL_DIR + tmpFile))
			{
				check = false;
			}
			else
			{
				modelFilename = tmpFile;
				check = true;
			}
		}
		else if (strcmp("N", choice) != 0)
		{
			cout << "Invalid option; Try again." << endl;
			check = false;
		}
		else
		{
			check = true;
		}
	}

	// Camera parameters
	check = false;
	while (!check)
	{
		cout << "Camera parameters file is: " << configCameraFilename << "; do you want to give a new file? [Y/N]";
		cin >> choice;
		if (!strcmp("Y", choice))
		{
			cout << "Give new camera parameters file: ";
			string tmpFile;
			cin >> tmpFile;
			if (!checkFileConfig(_CONFIG_DIR + tmpFile))
			{
				check = false;
			}
			else
			{
				configCameraFilename = tmpFile;
				check = true;
			}
		}
		else if (strcmp("N", choice) != 0)
		{
			cout << "Invalid option; Try again." << endl;
			check = false;
		}
		else
		{
			check = true;
		}
	}

	// Miscellaneous parameters
	check = false;
	while (!check)
	{
		cout << "Miscellaneous parameters file is: " << configParamsFilename << "; do you want to give a new file? [Y/N]";
		cin >> choice;
		if (!strcmp("Y", choice))
		{
			cout << "Give new miscellaneous parameters file: ";
			string tmpFile;
			cin >> tmpFile;
			if (!checkFileConfig(_CONFIG_DIR + tmpFile))
			{
				check = false;
			}
			else
			{
				configParamsFilename = tmpFile;
				check = true;
			}
		}
		else if (strcmp("N", choice) != 0)
		{
			cout << "Invalid option; Try again." << endl;
			check = false;
		}
		else
		{
			check = true;
		}
	}
}

// Render function
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera &camera, Mat &imagePlane, Vec3b colour)
{
	// Initialization
	vector <Point3f> homogeneousVertices;

	// Reset visibility
	resetVisibility(cuboid3D);

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics(AXISANGLE);

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
	drawObj(objProjection, imagePlane, colour, CV_AA);

	return objProjection;
}

// Dissimilarity between data and model object
void dissimilarity(vector <vector <Point2f>> edgesVertices, Mat &imagePlane, Mat dataImage, Mat distTransform, int mNum, Mat &mijs, Mat &dijs)
{
	// Initialization
	float offset = 10.f;

	// Construct edges vectors
	Mat edgesVec = edgesVectors(edgesVertices);

	// Calculate edges subintervals
	mijs = edgesSubIntervals(edgesVec, edgesVertices, mNum);

	// Draw mijs
	drawMijs(mijs, imagePlane);

	// Calculate dijs
	dijs = calculateDistance(mijs, distTransform);
}

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Vec3b colour, int lineType)
{
	for (int i = 0; i < objProjection.getEdgesSize(); i++)
	{
		Point2i p1(objProjection.getEdge(i)[0]);
		Point2i p2(objProjection.getEdge(i)[1]);
		// Draw _VERTICES
		imagePlane.at<Vec3b>(p1.y, p1.x) = colour;
		imagePlane.at<Vec3b>(p2.y, p2.x) = colour;
		// Draw edges
		line(imagePlane, p1, p2, colour, 1, lineType, 0);
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
	Vec3b colour = Vec3b(0, 255, 0);

	for (int j = 0; j < mijs.cols; j++)
	{
		for (int i = 0; i < mijs.rows; i++)
		{
			circle(imagePlane, mijs.at<Point2f>(i, j), 2, colour, -1, 8);
		}
	}
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
	// Extract distance from mij to data, using distance transform image
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

	// Convert to euler angles from axis angle
	// to accumulate the error - axis angle not linear
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

// Export fitting data
void exportFittingData(Mat m, Mat x)
{
	ofstream out;
	out.open("data/fitting_data.txt", ofstream::out | ofstream::trunc);
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

// Detect edges using Canny edge detector
Mat detectEdges(Mat img, double threshold, double ratio, int kernel)
{
	// Convert image to grayscale
	Mat imgGray;
	cvtColor(img, imgGray, CV_BGR2GRAY);
	// Reduse noise using a 3 x 3 kernel
	Mat detectedEdges;
	blur(imgGray, detectedEdges, Size(3, 3));
	// Canny detector
	Canny(detectedEdges, detectedEdges, threshold, threshold * ratio, kernel);

	return detectedEdges;
}

// Invert binary images
Mat invertBinaryImage(Mat binaryImg)
{
	Mat invertedImg(binaryImg.size(), CV_8UC1);
	for (int i = 0; i < invertedImg.rows; i++)
	{
		for (int j = 0; j < invertedImg.cols; j++)
		{
			int pixel = binaryImg.at<uchar>(i, j);
			if (pixel == 0)
			{
				invertedImg.at<uchar>(i, j) = 255;
			}
			else if (pixel == 255)
			{
				invertedImg.at<uchar>(i, j) = 0;
			}
		}
	}

	return invertedImg;
}

// Play source video and track model
void playVideo(string videoFilename, Cuboid3D model, vector <float> params, float fov, int imageWidth, int imageHeight, int maxIterations, float threshold, float ratio, int kernel)
{
	// Create a VideoCapture object and open the input file
	// If the input is the web camera, pass 0 instead of the video file name
	string dir = _VIDEO_DIR;
	VideoCapture cap(dir + videoFilename);

	// Virtual camera initialization
	Camera virtualCam = createCam(Vec3f(params[0], params[1], params[2]), axisAngle2euler(Vec3f(params[3], params[4], params[5])), fov, imageWidth, imageHeight, vector <State> {X, Y, Z, R1, R2, R3});
	// Model colour
	Vec3b modelColour(0, 255, 0);

	int count = 0;
	while (1) 
	{
		// Capture frame-by-frame
		Mat frame;
		cap >> frame;

		// If the frame is empty, break immediately
		if (frame.empty())
			break;

#ifdef _CAP_FIRST_FRAME
		if (count == 0)
		{
			size_t pos = videoFilename.find_last_of(".");
			string filename = dir + videoFilename.substr(0, pos);
			filename += "FirstFrame.jpg";
			imwrite(filename, frame);
		}
		++count;
#endif // _CAP_FIRST_FRAME
		
		// Render frame
		// Resize frame to _HEIGHT and _WIDTH of the image plane of the virtual camera
		// Image plane WIDTHxHEIGHT pixels
		Mat imagePlane(Size(imageWidth, imageHeight), CV_8UC3, CV_RGB(255, 255, 255));
		Mat frameResize;
		resize(frame, frameResize, Size(imageWidth, imageHeight));
		frameResize.copyTo(imagePlane);
				
		// Detect data object edges
		Mat cubeEdges = detectEdges(imagePlane, threshold, ratio, kernel);
		// Invert binary image for distance transform
		Mat cubeEdgesInv = invertBinaryImage(cubeEdges);
		// Calculate distance transform of data and it's first derivatives
		Mat distTransform, dxDist, dyDist;
		distanceTransform(cubeEdgesInv, distTransform, CV_DIST_L2, 3);
		distTransformImageGradient(distTransform, dxDist, dyDist);

		// Render model
		Cuboid2D modelProjection = rendering(model, virtualCam, imagePlane, modelColour);
		dissimilarity(modelProjection.getEdges(), imagePlane, cubeEdgesInv, distTransform, static_cast<int>(params[6]));

		// Fitting
		for (int i = 0; i < maxIterations; i++)
		{
			// Update model
			Mat imagePlaneModel(Size(imageWidth, imageHeight), CV_8UC3, CV_RGB(255, 255, 255));
			Cuboid2D modelProjection = rendering(model, virtualCam, imagePlaneModel, modelColour);

			// Disimillarity between data and model object
			Mat mijs, dijs;
			dissimilarity(modelProjection.getEdges(), imagePlaneModel, cubeEdgesInv, distTransform, static_cast<int>(params[6]), mijs, dijs);
			// Compute first derivatives of 3D homogeneous projection model coordinates
			// in respect to the state parameters of the camera - extrinsics parameters
			Mat Jdijs = computeModelFirstDerivatives(model, modelProjection, virtualCam, mijs, dxDist, dyDist);
			// Use non linear fitting to estimate new parameters for virtual camera
			vector <float> xNew = fittingGaussNewton(virtualCam, Jdijs, dijs);
			virtualCam.setParams(xNew, virtualCam.getStates());
		}
		// Render frame
		frameResize.copyTo(imagePlane);
		// Render model
		Cuboid2D modelProjectionNew = rendering(model, virtualCam, imagePlane, modelColour);
		dissimilarity(modelProjectionNew.getEdges(), imagePlane, cubeEdgesInv, distTransform, static_cast<int>(params[6]));

		// Display Model - Data dissimilarity
		displayImagePlane(videoFilename, imagePlane);
		
#ifdef _DEBUG
		// Display cube edges
		displayImagePlane("Canny edges video", cubeEdges);
		// Display virtual camera parameters
		dispCamParamsVideo(virtualCam);
#endif // _DEBUG

		// Press  ESC on keyboard to exit
		if (waitKey(5) == 27)
			break;
	}
}

// Use mouse to select 2D points and then give their corresponding 3D points
void get2D_3DCorrespondingPoints(const vector <Point3f> *const modelPoints, vector <Point3f> *const points3D,
								 vector <Point2f> *const points2D)
{
	// Check size of selected 2d points
	if (points2DMouse.size() != 6)
	{
		cout << "You should select 6 2d points from the image plane" << endl;
		return;
	}
	(*points2D) = points2DMouse;
	points2DMouse.clear();
	// Choose 6 corresponding 3D model points
	cout << "Give the order of the 6 corresponding 3D model points:" << endl;
	cout << "Vertices 0 - " << (*modelPoints).size() - 1 << ": ";
	while ((*points3D).size() < 6)
	{
		unsigned int choice;
		cin >> choice;
		if (choice < static_cast<int>((*modelPoints).size()))
		{
			(*points3D).push_back((*modelPoints)[choice]);
		}
		else
		{
			cout << "The cube has only 8 vertices, choose again." << endl;
		}
	}
	
	cout << "Chosen 2D - 3D corresponding points: ";
	for (size_t i = 0; i < (*points2D).size(); i++)
	{
		cout << "\n" << (*points2D)[i] << " --> " << (*points3D)[i];
	}
	cout << endl;
}

// Write camera parameters to file
void writeCamParams(string filename, vector <float> parameters)
{
	// Check parameters size
	if (static_cast<int>(parameters.size()) != 6)
	{
		cout << "There should be 6 camera parameters." << endl;
		return;
	}

	// Open file
	ofstream out;
	string path = _CONFIG_DIR;
	out.open(path + filename, ofstream::out | ofstream::trunc);
	if (!out.is_open())
	{
		cout << "Problem with opening the file." << endl;
		return;
	}

	// Axis angle to euler angles
	Vec3f eulerAngles = axisAngle2euler(Vec3f(parameters[3], parameters[4], parameters[5]));
	parameters[3] = rad2deg(eulerAngles.val[0]);
	parameters[4] = rad2deg(eulerAngles.val[1]);
	parameters[5] = rad2deg(eulerAngles.val[2]);

	string s;
	for (int i = 0; i < static_cast<int>(parameters.size()); i++)
	{
		ostringstream convert1;
		switch (i)
		{
			case 0:
				s += "TX ";
				break;
			case 1:
				s += "TY ";
				break;
			case 2:
				s += "TZ ";
				break;
			case 3:
				s += "RX ";
				break;
			case 4:
				s += "RY ";
				break;
			case 5:
				s += "RZ ";
				break;
			default:
				break;
		}
		convert1 << parameters[i];
		s += convert1.str();
		s += "\n";

	}

	out << s;
	out.flush();
	out.close();
}