#include "../headers/visualTracking.h"

// Visual tracker main function
void visualTracker(Cuboid3D &model, Cuboid3D &Data, int width, int height)
{
	// Initialization
	vector <State> state = { X, Y, Z, THETAX, THETAY, THETAZ };
	float fov = 60.f;
	vector <float> defaultParams = { TX, TY, TZ, RX, RY, RZ, MNUM };
	int iterations = 0; // Number of non linear fitting iterations
	bool exitFlag = false; // If true, exit application
	bool updateFlag = true; // If true render again cuboids
	bool fitFlag = false; // If true start non linear fitting
	int mNum = MNUM, edgesNum = 0;

	// Virtual camera initialization
	Camera virtualCam = createCam(Vec3f(TX, TY, TZ), Vec3f(RX, RY, RZ), fov, width, height, state);
	// Real camera initialization	
	Camera realCam = createCam(Vec3f(TX+2, TY, TZ), Vec3f(RX, RY, RZ), fov, width, height, state);

	// Image plane WIDTHxHEIGHT pixels
	Mat imagePlane(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneModel(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	Mat imagePlaneData(height, width, CV_8UC3, CV_RGB(255, 255, 255));
	DistClosedForm distCF;
	
	while (!exitFlag)
	{
		if (updateFlag)
		{
			vector <Distance> distances;
			for (int i = 0; i < virtualCam.getStateSize(); i++)
			{
				distances.push_back(Distance());
			}
			//Update frame
			Cuboid2D modelProjection = updateFrame(virtualCam, realCam, model, Data, distCF, imagePlane, imagePlaneModel, imagePlaneData, distances);
			edgesNum = static_cast<int>(modelProjection.getEdgesSize());
			//// Disimillarity between data and model object
			//dissimilarity(modelProjection, imagePlane, imagePlaneData, distances, distCF, mNum);
			//// Errors vector
			//vector <float> errors;
			//errors = distances[0].getErrors();
			//if (!errors.empty())
			//{
			//	Mat E = Mat(static_cast<int>(errors.size()), 1, CV_32F);
			//	memcpy(E.data, errors.data(), errors.size() * sizeof(float));
			//	cout << "Dissimilarity between model and data object: " << sum(E)[0] << endl;
			//}
		}
		if (fitFlag)
		{
			// Calculate new extrinsics matrix with smaller error according to data
			/*fitting(virtualCam, distCF, edgesNum);
			++iterations;
			if (iterations == 1)
			{
				fitFlag = false;
				iterations = 0;
			}*/
		}
		else
		{
			// Keys pressed handler
			keyboardHandler(virtualCam, realCam, model, Data, mNum, defaultParams, exitFlag, updateFlag, fitFlag);
			// Clear distCF
			distCF.Reset();
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

// Render function - data
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	// Initialization
	Mat tmp(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	tmp.copyTo(imagePlaneObj);
	vector <int>::iterator iter1, iter2;
	vector <int> edge, checkEdge;
	vector <Point3f> homogeneousVertices;
	vector <int> surface;
	vector <Point2f> pointsPxl;
	for (int i = 0; i < cuboid3D.getEdgesSize(); i++)
	{
		cuboid3D.setEdgeVisibility(i, false);
	}
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		cuboid3D.setSurfaceVisibility(i, false);
	}

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics();

	// Project vertices to image plane
	vector <float> homogeneous;
	for (int i = 0; i < cuboid3D.getVerticesSize(); i++)
	{
		// Convert vertex coordinates to homogeneous coordinates
		homogeneous.push_back(cuboid3D.getVertex(i).x);
		homogeneous.push_back(cuboid3D.getVertex(i).y);
		homogeneous.push_back(cuboid3D.getVertex(i).z);
		homogeneous.push_back(1.f);

		// Perspective projection
		Mat tmp = P * Mat(homogeneous, false);
		Point3f projection(tmp);
		homogeneousVertices.push_back(projection);
		homogeneous.clear();
	}

	// Create cuboid2d - projection of cuboid3d
	Cuboid2D objProjection(homogeneousVertices);
	homogeneousVertices.clear();
	
	// Surface occlusion
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		// Check if surface is not occluded
		if (!backFaceCulling(cuboid3D.getSurfaceVertices(i), cuboid3D.getVertices(), camera.getPosition()))
		{
			// Surface i is visible
			cuboid3D.setSurfaceVisibility(i, true);
			surface = cuboid3D.getSurfaceVertices(i);
			if (cuboid3D.getSurfaceVisibility(i))
			{
				objProjection.setSurface(surface);
			}
			for (int j = 0; j < surface.size(); j++)
			{
				// Egde j is visible
				edge.push_back(surface[j]);
				edge.push_back(surface[(j + 1) % surface.size()]);
				for (int k = 0; k < cuboid3D.getEdgesSize(); k++)
				{
					if (!(cuboid3D.getEdgeVisibility(k)))
					{
						checkEdge = cuboid3D.getEdge(k);
						iter1 = find(checkEdge.begin(), checkEdge.end(), edge[0]);
						iter2 = find(checkEdge.begin(), checkEdge.end(), edge[1]);
						if ((iter1 != checkEdge.end()) && (iter2 != checkEdge.end()))
						{
							cuboid3D.setEdgeVisibility(k, true);
							pointsPxl.push_back(objProjection.getVertexPxl(edge[0]));
							pointsPxl.push_back(objProjection.getVertexPxl(edge[1]));
							if (cuboid3D.getEdgeVisibility(k) && edgeClip(pointsPxl, imagePlane.size()))
							{
								objProjection.setEdge(pointsPxl);
								checkEdge.clear();
								pointsPxl.clear();
								break;
							}
						}
						checkEdge.clear();
					}
				}
				edge.clear();
			}
			surface.clear();
		}
	}

	// Render object
	drawObj(objProjection, imagePlane, imagePlaneObj, type);

	// Display image plane object
	string windowName = "Image Plane";
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);

	// Display image plane object
	windowName = "Cuboid3D " + type;
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlaneObj);
	
	return objProjection;
}

// Render function - model
Cuboid2D rendering(Cuboid3D &cuboid3D, Camera camera, Mat &imagePlane, Mat &imagePlaneObj, vector <Distance> &distances, string type)
{
	// Initialization
	Mat img(imagePlane.rows, imagePlane.cols, CV_8UC3, CV_RGB(255, 255, 255));
	img.copyTo(imagePlaneObj);
	img.copyTo(imagePlane);
	vector <Point3f> homogeneousVertices;

	// Reset visibility
	resetVisibility(cuboid3D);

	// Perspective projection matrix
	Mat P = camera.getIntrinsics() * camera.getExtrinsics();

	// Project vertices to image plane
	for (int i = 0; i < cuboid3D.getVerticesSize(); i++)
	{
		homogeneousVertices.push_back(perspectiveProjection(cuboid3D.getVertex(i), P));
		// Perspective projection derivative
		for (int j = 0; j < camera.getStateSize(); j++)
		{
			// Perspective projection derivative matrix
			Mat dP = camera.getLieAlgebraDerivative(j);
			Mat tmpDeriv = dP * Mat(cart2hmgns(cuboid3D.getVertex(i)), false);
			Point3f homogeneousDeriv(tmpDeriv);
			distances[j].setVertexDeriv(homogeneousDeriv, homogeneousVertices[i]);
		}
	}

	// Create cuboid2d - projection of cuboid3d
	Cuboid2D objProjection(homogeneousVertices);
	
	// Object visibility culling
	visibilityCulling(cuboid3D, objProjection, distances, camera, imagePlane.size());

	// Render object
	drawObj(objProjection, imagePlane, imagePlaneObj, type);

	// Display image plane object
	dispImagePlane("Image Plane", imagePlane);

	// Display image plane object
	dispImagePlane("Cuboid3D " + type, imagePlaneObj);

	return objProjection;
}

// Dissimilarity between data and model object
void dissimilarity(Cuboid2D &model, Mat &imagePlane, Mat imagePlaneData, vector <Distance> &distances, DistClosedForm &distCF, int mNum)
{
	// Initialization
	float offsetOut, offsetIn, normVec, normNormalVec;
	Vec2f vec, normalOut, normalIn, checkNormal;
	Point2f  vi, vj, p, tmp, pC, p_mi, pNormal, pCheck;
	vector <Point2f> normalLine, edge;
	Vec3b pixel;
	Rect rectangle(0, 0, imagePlane.cols, imagePlane.rows);
	int length;

	// Set mijNum = mNum - 2
	for (int i = 0; i < distances.size(); i++)
	{
		distances[i].setIntervalNum(mNum - 2);
	}

	for (int i = 0; i < model.getEdgesSize(); i++)
	{
		// Check if edge is visible
		edge = model.getEdge(i);
		// Compute equal distance points on each edge
		vi = edge[0];
		vj = edge[1];
		vec = vj - vi;
		normVec = norm2(vec);

		if (normVec > 0.f)
		{
			// In and Out normal for each edge
			normalOut.val[0] = -(vj.y - vi.y);
			normalOut.val[1] = vj.x - vi.x;
			normalOut = normalOut / max(norm2(normalOut), 0.0001f);
			normalIn.val[0] = -(vi.y - vj.y);
			normalIn.val[1] = vi.x - vj.x;
			normalIn = normalIn / max(norm2(normalIn), 0.0001f);

			// For each mi on vivj edge compute normal vector and distance from data object
			for (int j = 1; j < (mNum - 1); j++)
			{
				offsetOut = 10.f;
				offsetIn = 10.f;
				// Calculate mi on edge vivj
				p = vec * ((float)j / (float)(mNum - 1));
				p = vi + p;
				p_mi = p;

				// Draw Out normal from current mi
				tmp.x = p_mi.x + offsetOut * normalOut.val[0];
				tmp.y = p_mi.y + offsetOut * normalOut.val[1];
				pNormal = tmp;
				// Edge near image plane
				if (!(pNormal.inside(rectangle)))
				{
					checkNormal = (Point2f)(pNormal - p_mi);
					normNormalVec = norm2(checkNormal);
					length = (int)normNormalVec;
					if (length > 0)
					{
						checkNormal = checkNormal / normNormalVec;
						// Check if the whole egde is outside the image plane
						for (int k = 0; k <= length; k++)
						{
							pC = checkNormal * ((float)k / (float)(length)) * normNormalVec;
							pC = (Point2f)p_mi + pC;
							pCheck = pC;
							if (pCheck.inside(rectangle))
							{
								pNormal = pCheck;
								offsetOut = (float)k;
								break;
							}
						}
					}
					else
					{
						pNormal = p_mi;
					}
				}
				normalLine.push_back(pNormal);
				arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);

				// Draw In normal from current mi
				tmp.x = p_mi.x + offsetIn * normalIn.val[0];
				tmp.y = p_mi.y + offsetIn * normalIn.val[1];
				pNormal = tmp;
				// Edge near image plane
				if (!(pNormal.inside(rectangle)))
				{
					checkNormal = (Point2f)(pNormal - p_mi);
					normNormalVec = norm2(checkNormal);
					length = (int)normNormalVec;
					if (length > 0)
					{
						checkNormal = checkNormal / normNormalVec;
						// Check if the whole egde is outside the image plane
						for (int k = 0; k <= length; k++)
						{
							pC = checkNormal * ((float)k / (float)(length)) * normNormalVec;
							pC = (Point2f)p_mi + pC;
							pCheck = pC;
							if (pCheck.inside(rectangle))
							{
								pNormal = pCheck;
								offsetIn = (float)k;
								break;
							}
						}
					}
					else
					{
						pNormal = p_mi;
					}
				}
				normalLine.push_back(pNormal);
				arrowedLine(imagePlane, p_mi, pNormal, CV_RGB(0, 0, 0), 1, CV_AA, 0, 0.5);
				circle(imagePlane, p_mi, 2, CV_RGB(0, 0, 0), -1, 8, 0);

				// Calculate distanse
				calcDist(distances, distCF, normalOut, vec, normalLine, pNormal, p_mi, offsetOut, offsetIn, imagePlane, imagePlaneData, i, j);

				normalLine.clear();
			}
		}
		edge.clear();
	}

	// Display Model - Data dissimilarity
	dispImagePlane("Model - Data dissimilarity", imagePlane);
}

// Calculate distance between model and data
void calcDist(vector <Distance> &distances, DistClosedForm &distCF, Vec2f normalOut, Vec2f vec, vector <Point2f> normalLine,
	Point2f pNormal, Point2f p_mi, float offsetOut, float offsetIn, Mat &imagePlane, Mat imagePlaneData, int idx, int j)
{
	Point2f intersection, pN;
	float normalOffset, maxValue = 765.f;
	Vec2f normalVec;
	Vec3b pixel;
	int mNum = distances[0].getIntervalNum() + 2;

	// Calculate edge distance between model and data object
	normalOffset = offsetOut + offsetIn;
	normalVec = normalLine[1] - normalLine[0];
	vector <float> weights;
	float pixelValue;
	vector <Point2f> sijPoints;
	int pointer = 0, continuous = 0;
	bool first = false;
	for (int k = 0; k <= normalOffset; k++)
	{
		pN = normalVec * ((float)k / (float)(normalOffset));
		pN = normalLine[0] + pN;
		pNormal = pN;
		// Find the pixel with the smallest value
		pixel = imagePlaneData.at<Vec3b>(pNormal);
		pixelValue = (float)(pixel.val[0] + pixel.val[1] + pixel.val[2]);
		if (pixelValue < maxValue)
		{
			// First interpolation
			if (!first)
			{
				first = true;
			}
			pointer = k;
			if (pointer == continuous + 1)
			{
				weights.push_back(pixelValue / 765.f);
				sijPoints.push_back(pNormal);
				continuous = k;
			}
		}
		else
		{
			if (!first)
			{
				continuous = k;
			}
		}
	}
	// Calculate euclidean distance between model and data edge
	if (sijPoints.size() > 0)
	{
		// Intersection is found
		// Find exact sij point on the data edge - weighted average
		float weightsSum = 0.f;
		for (int k = 0; k < sijPoints.size(); k++)
		{
			intersection += (1 - weights[k]) * sijPoints[k];
			weightsSum += (1 - weights[k]);
		}
		intersection.x = intersection.x / weightsSum;
		intersection.y = intersection.y / weightsSum;
		circle(imagePlane, intersection, 2, CV_RGB(255, 0, 0), -1, CV_AA, 0);
		// Assign mij, calculate mij derivatives, assign dij and calculate dij derivatives
		for (int s = 0; s < (int)distances.size(); s++)
		{
			distances[s].setInterval(p_mi);
			Point2f dp_mi, dsij = Point2f(0.f, 0.f);
			Vec2f dvec, dij, lij;
			dvec = distances[s].getEdgeDeriv(idx)[1] - distances[s].getEdgeDeriv(idx)[0];
			dp_mi = ((float)j / (float)(mNum - 1)) * dvec;
			dp_mi += distances[s].getEdgeDeriv(idx)[0];
			distances[s].setIntervalDeriv(dp_mi);
			dij = (Point2f)(intersection - p_mi);
			distances[s].setError(norm2(dij));
			lij = dp_mi;
			if (s == 0)
			{
				distCF.setDij(norm2(dij));
			}
			distCF.setFij(lij.dot(normalVec / norm2(normalVec)));
		}
	}
	else
	{
		// Assign mij, calculate mij derivatives, assign dij and calculate dij derivatives
		// Intersection not found
		for (int s = 0; s < distances.size(); s++)
		{
			distances[s].setInterval(p_mi);
			float lambda = offsetOut;
			Point2f dp_mi;
			Vec2f dvec, dij, normal, lij;
			normal = normalOut;
			dvec = distances[s].getEdgeDeriv(idx)[1] - distances[s].getEdgeDeriv(idx)[0];
			dp_mi = ((float)j / (float)(mNum - 1)) * dvec;
			dp_mi += distances[s].getEdgeDeriv(idx)[0];
			distances[s].setIntervalDeriv(dp_mi);
			dij.val[0] = p_mi.x + lambda * normal.val[0];
			dij.val[1] = p_mi.y + lambda * normal.val[1];
			distances[s].setError(norm2(dij));
			lij = dp_mi;
			if (s == 0)
			{
				distCF.setDij(norm2(dij));
			}
			distCF.setFij(lij.dot(normalVec / norm2(normalVec)));
		}
	}
}

// Calculate new extrinsics matrix closer to the data
void fitting(Camera& virtualCam, DistClosedForm& distCF, int edgesNum)
{
	distCF.setWnum(virtualCam.getStateSize());
	distCF.calcFijWeights(edgesNum, MNUM - 2);
	Mat M = expMap(distCF);
	Mat R;
	Vec3f t;
	decomposeEuclidean(M, R, t);
	cout << matrix2euler(R) * 180.f / PI << endl;
	Mat dcamPose;
	cameraPose(R, Mat(t), dcamPose);
	cout << dcamPose << endl;

	Mat E = virtualCam.getExtrinsics() * dcamPose;
	virtualCam.setExtrinsics(E);
}

// Exponential map se(3) to SE(3)
Mat expMap(DistClosedForm &distCF)
{
	Mat I, R, V;
	Mat M = Mat::zeros(4, 4, CV_32F);
	vector <float> params = distCF.getWs();
	Vec3f u = Vec3f(params[0], params[1], params[2]);
	Vec3f w = Vec3f(params[3], params[4], params[5]);
	I = Mat::eye(3, 3, CV_32F);
	float theta = max(sqrtf(w.dot(w)), 0.0001f);
	float a = sin(theta) / theta;
	float thetaSQR = theta * theta;
	float b = (1 - cos(theta)) / thetaSQR;
	float c = (1 - a) / thetaSQR;

	Mat Wx = skewMat(w);

	R = I + a * Wx + b * (Wx * Wx);
	V = I + b * Wx + c * (Wx * Wx);

	Rect r(0, 0, 3, 3);
	R.copyTo(M(r));
	Mat Vu = V * Mat(u);
	M.at<float>(0, 3) = Vu.at<float>(0);
	M.at<float>(1, 3) = Vu.at<float>(1);
	M.at<float>(2, 3) = Vu.at<float>(2);
	M.at<float>(3, 3) = 1.f;

	return M;
}

// Draw object on image planes
void drawObj(Cuboid2D objProjection, Mat &imagePlane, Mat &imagePlaneObj, string type)
{
	Vec3b colour = Vec3b(0, 0, 0);
	if (!(type.compare("data")))
	{
		colour.val[2] = 255;
	}
	
	for (int i = 0; i < objProjection.getEdgesSize(); i++)
	{
		Point2i p1(objProjection.getEdge(i)[0]);
		Point2i p2(objProjection.getEdge(i)[1]);
		// Draw vertices
		imagePlane.at<Vec3b>(p1.y, p1.x) = colour;
		imagePlane.at<Vec3b>(p2.y, p2.x) = colour;
		imagePlaneObj.at<Vec3b>(p1.y, p1.x) = colour;
		imagePlaneObj.at<Vec3b>(p2.y, p2.x) = colour;
		// Draw edges
		line(imagePlane, p1, p2, colour, 1.5, CV_AA, 0);
		line(imagePlaneObj, p1, p2, colour, 1, CV_AA, 0);
	}
}

// Check which parts of the object are visible from the camera's given viewpoint
void visibilityCulling(Cuboid3D &cuboid3D, Cuboid2D &cuboid2D, vector <Distance> &distances, Camera camera, Size size)
{
	int edge;
	vector <int> surfaceVertices, surfaceEdges, edgeVertices;
	vector <Point2f> pointsPxl;

	// Visibility culling
	for (int i = 0; i < cuboid3D.getSurfacesSize(); i++)
	{
		// Check if the surface is facing the camera
		surfaceVertices = cuboid3D.getSurfaceVertices(i);
		if (!backFaceCulling(surfaceVertices, cuboid3D.getVertices(), camera.getPosition()))
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
				// If rendered == true
				if (cuboid3D.getEdgeVisibility(edge) && edgeClip(pointsPxl, size)) 
				{
					cuboid2D.setEdge(pointsPxl);
					// Edges derivatives
					for (int s = 0; s < distances.size(); s++)
					{
						vector <Point2f> edgeDerivative;
						edgeDerivative.push_back(distances[s].getPxlDeriv(edgeVertices[0]));
						edgeDerivative.push_back(distances[s].getPxlDeriv(edgeVertices[1]));
						distances[s].setEdgeDeriv(edgeDerivative);
					}
				}
				pointsPxl.clear();
				edgeVertices.clear();
			}
		}
		surfaceVertices.clear();
		surfaceEdges.clear();
	}
}

// Edge clipping
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
int backFaceCulling(vector <int> surface, vector <Point3f> vertices, Point3f t)
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
	float theta = acos(normal.dot(look_vector)) * 180.f / PI;

	// If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta < 90.f)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

// Fixed time frame update
Cuboid2D updateFrame(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, DistClosedForm &distCF, Mat &imagePlane, Mat &imagePlaneModel, Mat &imagePlaneData, vector <Distance> &distances)
{
	// Render model
	Cuboid2D modelProjection = rendering(model, virtualCam, imagePlane, imagePlaneModel, distances, "model");
	// Render data
	Cuboid2D dataProjection = rendering(Data, realCam, imagePlane, imagePlaneData, "data");

	// Display camera parameters
	dispCamParams(virtualCam, realCam);

	return modelProjection;
}

// Create camera
Camera createCam(Vec3f t, Vec3f r, float fov, int width, int height, vector <State> state)
{
	// Camera Intrinsics
	float u0 = static_cast<float>(width) / 2.f, v0 = static_cast<float>(height) / 2.f; // Principal point - center of image plane
	float focal = (static_cast<float>(width) / 2.f) / (tan((fov / 2.f) * PI / 180.f)); // Focal length

	return Camera(t, r, Point2f(u0, v0), fov, focal, state);
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
void dispImagePlane(string windowName, Mat imagePlane)
{
	namedWindow(windowName, WINDOW_AUTOSIZE);
	imshow(windowName, imagePlane);
	// waitKey(1);
}