#include "../headers/controls.h"

// Display camera parameters
void dispCamParams(Camera virtualCam)
{
	// Attributes
	HersheyFonts font = FONT_HERSHEY_COMPLEX_SMALL;
	double fontScale = 0.6;
	Scalar colour = (0, 0, 0);
	int thickness = 1;
	int lineType = CV_AA;

	// Display camera parameters
	Mat virtualValues(70, 400, CV_8UC3, Scalar::all(255));
	// Virtual Camaera Parameters
	string values = "Tx: " + to_string(virtualCam.getTx()) + " Ty: " + to_string(virtualCam.getTy()) + " Tz: " + to_string(virtualCam.getTz());
	putText(virtualValues, values, cvPoint(0, 20), font, fontScale, colour, thickness, lineType);
	values.clear();
	values = "Rx: " + to_string(virtualCam.getThetaX(DEGREES)) + " Ry: " + to_string(virtualCam.getThetaY(DEGREES)) + " Rz: " + to_string(virtualCam.getThetaZ(DEGREES));
	putText(virtualValues, values, cvPoint(0, 50), font, fontScale, colour, thickness, lineType);
	values.clear();
	// Show camera parameters
	const char* virtualCamParams = "Virtual Camera Parameters";
	imshow(virtualCamParams, virtualValues);
	waitKey(1);
}

// Display camera parameters for video
void dispCamParamsVideo(Camera virtualCam)
{
	// Attributes
	HersheyFonts font = FONT_HERSHEY_COMPLEX_SMALL;
	double fontScale = 0.6;
	Scalar colour = (0, 0, 0);
	int thickness = 1;
	int lineType = CV_AA;

	// Display camera parameters
	Mat virtualValues(70, 400, CV_8UC3, Scalar::all(255));
	// Virtual Camaera Parameters
	string values = "Tx: " + to_string(virtualCam.getTx()) + " Ty: " + to_string(virtualCam.getTy()) + " Tz: " + to_string(virtualCam.getTz());
	putText(virtualValues, values, cvPoint(0, 20), font, fontScale, colour, thickness, lineType);
	values.clear();
	values = "Rx: " + to_string(virtualCam.getThetaX(DEGREES)) + " Ry: " + to_string(virtualCam.getThetaY(DEGREES)) + " Rz: " + to_string(virtualCam.getThetaZ(DEGREES));
	putText(virtualValues, values, cvPoint(0, 50), font, fontScale, colour, thickness, lineType);
	values.clear();
	// Show camera parameters
	const char* virtualCamParams = "Virtual Camera Parameters Video";
	imshow(virtualCamParams, virtualValues);
	waitKey(1);
}

// Mouse Handler
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	// Left click
	if (event == EVENT_LBUTTONDOWN)
	{
		cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
}

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Cuboid3D &model, int &mNum, vector <float> defaultParams, bool &exitFlag, bool &updateModelFlag, bool &fitFlag, 
					 bool &readFilesFlag, bool &updateFilenamesFlag, bool &videoFlag, bool &writeFlag, bool &poseEstimationFlag)
{
	switch (waitKey(0))
	{
		// Virtual Camera handler
		case 119: // 'w' key pressed - tyVirtual++
		{
			float tyVirtual = virtualCam.getTy();
			virtualCam.setTy(++tyVirtual);
			updateModelFlag = true;
			break;
		}
		case 115: // 's' key pressed - tyVirtual--
		{
			float tyVirtual = virtualCam.getTy();
			virtualCam.setTy(--tyVirtual);
			updateModelFlag = true;
			break;
		}
		case 97: // 'a' key pressed - txVirtual--
		{
			float txVirtual = virtualCam.getTx();
			virtualCam.setTx(--txVirtual);
			updateModelFlag = true;
			break;
		}
		case 100: // 'd' key pressed - txVirtual++
		{
			float txVirtual = virtualCam.getTx();
			virtualCam.setTx(++txVirtual);
			updateModelFlag = true;
			break;
		}
		case 101: // 'e' key pressed - tzVirtual++
		{
			float tzVirtual = virtualCam.getTz();
			virtualCam.setTz(++tzVirtual);
			updateModelFlag = true;
			break;
		}
		case 113: // 'q' key pressed - tzVirtual--
		{
			float tzVirtual = virtualCam.getTz();
			virtualCam.setTz(--tzVirtual);
			updateModelFlag = true;
			break;
		}
		case 122: // 'z' key pressed - ++ryVirtual
		{
			float ryVirtual = virtualCam.getThetaY(DEGREES);
			ryVirtual += 0.5f;
			virtualCam.setThetaY(ryVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 120: // 'x' key pressed - --ryVirtual
		{
			float ryVirtual = virtualCam.getThetaY(DEGREES);
			ryVirtual -= 0.5f;
			virtualCam.setThetaY(ryVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 99: // 'c' key pressed - ++rxVirtual
		{
			float rxVirtual = virtualCam.getThetaX(DEGREES);
			rxVirtual += 0.5f;
			virtualCam.setThetaX(rxVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 118: // 'v' key pressed - --rxVirtual
		{
			float rxVirtual = virtualCam.getThetaX(DEGREES);
			rxVirtual -= 0.5f;
			virtualCam.setThetaX(rxVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 98: // 'b' key pressed - ++rzVirtual
		{
			float rzVirtual = virtualCam.getThetaZ(DEGREES);
			virtualCam.setThetaZ(++rzVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 110: // 'n' key pressed - --rzVirtual
		{
			float rzVirtual = virtualCam.getThetaZ(DEGREES);
			virtualCam.setThetaZ(--rzVirtual, DEGREES);
			updateModelFlag = true;
			break;
		}
		case 114: // 'r' key pressed - read files
		{
			readFilesFlag = true;
			updateModelFlag = true;
			break;
		}
		case 108: // 'l' key pressed - read files
		{
			updateFilenamesFlag = true;
			readFilesFlag = true;
			updateModelFlag = true;
			break;
		}
		case 112: // 'p' key pressed - read files
		{
			videoFlag = true;
			updateModelFlag = true;
			break;
		}
		case 102: // 'f' key pressed - write files
		{
			writeFlag = true;
			break;
		}
		case 105: // 'i' key pressed - poseEstimation
		{
			poseEstimationFlag = true;
			break;
		}
		case 27: // ESC key pressed - exit
		{
			exitFlag = true;
			break;
		}
		case 13: // ENTER key pressed - start fitting
		{
			fitFlag = true;
			updateModelFlag = false;
			break;
		}
		case 32: // SPACE key pressed - set camera to default extrinsic parameters
		{
			vector <State> defaultState = { X, Y, Z, R1, R2, R3 };
			virtualCam.setParams({ defaultParams.begin(), defaultParams.end() - 1 }, defaultState);
			updateModelFlag = true;
			break;
		}
		default:
		{
			updateModelFlag = true;
			fitFlag = false;
			break;
		}
	}
}