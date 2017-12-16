#include "../headers/io.h"

// Read model data from .x file
void modelData(string modelFilename, float &length, float &height, float &width)
{
	// Check if input or the file are empty
	if (!checkFileModel(modelFilename))
	{
		return;
	}
	ifstream file(modelFilename);

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
	if (!checkFileConfig(configFilename))
	{
		return;
	}
	ifstream file(configFilename);

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
void configParamsData(string configParamsFilename, int &imageWidth, int &imageHeight, float &fov, int &mNum, int &maxIterations, float &threshold, float &ratio, int &kernel)
{
	// Check if input or the file are empty
	if (!checkFileConfig(configParamsFilename))
	{
		return;
	}
	ifstream file(configParamsFilename);

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
		else if (!(data[0].compare("fov")))
		{// Camera fov
			fov = stof(data[1]);
		}
		else if (!(data[0].compare("mNum")))
		{// Fitting parameters
			mNum = stoi(data[1]);
		}
		else if (!(data[0].compare("maxIterations")))
		{
			maxIterations = stoi(data[1]);
		}
		else if (!(data[0].compare("threshold")))
		{// Canny edge parameters
			threshold = stof(data[1]);
		}
		else if (!(data[0].compare("ratio")))
		{
			ratio = stof(data[1]);
		}
		else if (!(data[0].compare("kernel")))
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
				videoFilename = _VIDEO_DIR + tmpFile;
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
				modelFilename = _MODEL_DIR + tmpFile;
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
				configCameraFilename = _CONFIG_DIR + tmpFile;
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
				configParamsFilename = _CONFIG_DIR + tmpFile;
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