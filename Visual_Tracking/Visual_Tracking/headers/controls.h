#pragma once
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath> 

using namespace cv;
using namespace std;

#include "projectiveGeometry.h"
#include "cameraCalibration.h"

// Dispaly camera parameters
void dispCamParams(Camera, Camera);
// Keys pressed handler - case sensitive
void keyboardHandler(Camera&, Camera&, Cuboid3D&, Cuboid3D&, int&, vector <State>&, vector <float>, bool&, bool&, bool&);

// Display camera parameters
void dispCamParams(Camera virtualCam, Camera realCam)
{
	// Display camera parameters
	Mat virtualValues(70, 400, CV_8UC3, Scalar::all(255));
	Mat realValues(70, 400, CV_8UC3, Scalar::all(255));
	// Virtual Camaera Parameters
	string values = "Tx: " + to_string(virtualCam.getTx()) + " Ty: " + to_string(virtualCam.getTy()) + " Tz: " + to_string(virtualCam.getTz());
	putText(virtualValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	values = "Rx: " + to_string(virtualCam.getThetaX(DEGREES)) + " Ry: " + to_string(-virtualCam.getThetaY(DEGREES)) + " Rz: " + to_string(-virtualCam.getThetaZ(DEGREES));
	putText(virtualValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	// Real Camaera Parameters
	values = "Tx: " + to_string(realCam.getTx()) + " Ty: " + to_string(realCam.getTy()) + " Tz: " + to_string(realCam.getTz());
	putText(realValues, values, cvPoint(0, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	values.clear();
	values = "Rx: " + to_string(realCam.getThetaX(DEGREES)) + " Ry: " + to_string(-realCam.getThetaY(DEGREES)) + " Rz: " + to_string(-realCam.getThetaZ(DEGREES));
	putText(realValues, values, cvPoint(0, 50), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(0, 0, 0), 1, CV_AA);
	// Show camera parameters
	const char* virtualCamParams = "Virtual Camera Parameters";
	const char* realCamParams = "Real Camera Parameters";
	imshow(virtualCamParams, virtualValues);
	imshow(realCamParams, realValues);
	waitKey(1);
}

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, int &mNum, vector <State>& state, vector <float> defaultParams, bool &exitFlag, bool &updateFlag, bool &fitFlag)
{
	static int primitives = 0, surface = 0, edge = 0;

	switch (waitKey(0))
	{
		// Virtual Camera handler
	case 119: // 'w' key pressed - tyVirtual++
	{
		float tyVirtual = virtualCam.getTy();
		virtualCam.setTy(++tyVirtual);
		updateFlag = true;
		break;
	}
	case 115: // 's' key pressed - tyVirtual--
	{
		float tyVirtual = virtualCam.getTy();
		virtualCam.setTy(--tyVirtual);
		updateFlag = true;
		break;
	}
	case 97: // 'a' key pressed - txVirtual--
	{
		float txVirtual = virtualCam.getTx();
		virtualCam.setTx(--txVirtual);
		updateFlag = true;
		break;
	}
	case 100: // 'd' key pressed - txVirtual++
	{
		float txVirtual = virtualCam.getTx();
		virtualCam.setTx(++txVirtual);
		updateFlag = true;
		break;
	}
	case 101: // 'e' key pressed - tzVirtual++
	{
		float tzVirtual = virtualCam.getTz();
		virtualCam.setTz(++tzVirtual);
		updateFlag = true;
		break;
	}
	case 113: // 'q' key pressed - tzVirtual--
	{
		float tzVirtual = virtualCam.getTz();
		virtualCam.setTz(--tzVirtual);
		updateFlag = true;
		break;
	}
	case 122: // 'z' key pressed - ++ryVirtual
	{
		float ryVirtual = virtualCam.getThetaY(DEGREES);
		ryVirtual += 0.5f;
		virtualCam.setThetaY(ryVirtual);
		updateFlag = true;
		break;
	}
	case 120: // 'x' key pressed - --ryVirtual
	{
		float ryVirtual = virtualCam.getThetaY(DEGREES);
		ryVirtual -= 0.5f;
		virtualCam.setThetaY(ryVirtual);
		updateFlag = true;
		break;
	}
	case 99: // 'c' key pressed - ++rxVirtual
	{
		float rxVirtual = virtualCam.getThetaX(DEGREES);
		rxVirtual += 0.5f;
		virtualCam.setThetaX(rxVirtual);
		updateFlag = true;
		break;
	}
	case 118: // 'v' key pressed - --rxVirtual
	{
		float rxVirtual = virtualCam.getThetaX(DEGREES);
		rxVirtual -= 0.5f;
		virtualCam.setThetaX(rxVirtual);
		updateFlag = true;
		break;
	}
	case 98: // 'b' key pressed - ++rzVirtual
	{
		float rzVirtual = virtualCam.getThetaZ(DEGREES);
		virtualCam.setThetaZ(++rzVirtual);
		updateFlag = true;
		break;
	}
	case 110: // 'n' key pressed - --rzVirtual
	{
		float rzVirtual = virtualCam.getThetaZ(DEGREES);
		virtualCam.setThetaZ(--rzVirtual);
		updateFlag = true;
		break;
	}
	// Real Camera handler
	case 91: // '[{' key pressed - tyReal++
	{
		float tyReal = realCam.getTy();
		realCam.setTy(++tyReal);
		updateFlag = true;
		break;
	}
	case 39: // '"'' key pressed - tyReal--
	{
		float tyReal = realCam.getTy();
		realCam.setTy(--tyReal);
		updateFlag = true;
		break;
	}
	case 59: // ';:' key pressed - txReal--
	{
		float txReal = realCam.getTx();
		realCam.setTx(--txReal);
		updateFlag = true;
		break;
	}
	case 92: // '\|' key pressed - txReal++
	{
		float txReal = realCam.getTx();
		realCam.setTx(++txReal);
		updateFlag = true;
		break;
	}
	case 93: // ']}' key pressed - tzReal++
	{
		float tzReal = realCam.getTz();
		realCam.setTz(++tzReal);
		updateFlag = true;
		break;
	}
	case 112: // 'p' key pressed - tzReal--
	{
		float tzReal = realCam.getTz();
		realCam.setTz(--tzReal);
		updateFlag = true;
		break;
	}
	case 102: // 'f' key pressed - ++ryReal
	{
		float ryReal = realCam.getThetaY(DEGREES);
		realCam.setThetaY(++ryReal);
		updateFlag = true;
		break;
	}
	case 103: // 'g' key pressed - --ryReal
	{
		float ryReal = realCam.getThetaY(DEGREES);
		realCam.setThetaY(--ryReal);
		updateFlag = true;
		break;
	}
	case 104: // 'h' key pressed - ++rxReal
	{
		float rxReal = realCam.getThetaX(DEGREES);
		realCam.setThetaX(++rxReal);
		updateFlag = true;
		break;
	}
	case 106: // 'j' key pressed - --rxReal
	{
		float rxReal = realCam.getThetaX(DEGREES);
		realCam.setThetaX(--rxReal);
		updateFlag = true;
		break;
	}
	case 107: // 'k' key pressed - ++rzReal
	{
		float rzReal = realCam.getThetaZ(DEGREES);
		realCam.setThetaZ(++rzReal);
		updateFlag = true;
		break;
	}
	case 108: // 'l' key pressed - --rzReal
	{
		float rzReal = realCam.getThetaZ(DEGREES);
		realCam.setThetaZ(--rzReal);
		updateFlag = true;
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
		updateFlag = true;
		break;
	}
	case 32: // SPACE key pressed - set camera to default extrinsic parameters
	{
		vector <State> defaultState = { X, Y, Z, THETAX, THETAY, THETAZ };
		virtualCam.setParams(defaultParams, defaultState);
		defaultParams[0] += 2.f;
		realCam.setParams(defaultParams, defaultState);
		updateFlag = true;
		break;
	}
	case 9: // TAB key pressed - choose primitives to be rendered
	{
		primitives = (primitives + 1) % 3;
		model.setPrimitives(primitives);
		Data.setPrimitives(primitives);
		if (primitives == 0)
		{
			mNum = (int)defaultParams[6];
		}
		else if ((primitives == 1) && (mNum < 5))
		{
			mNum = 5;
		}
		else if ((primitives == 2) && (mNum < 7))
		{
			mNum = 7;
		}
		updateFlag = true;
		break;
	}
	case 45: // - key pressed - choose egde or surface to be rendered
	{
		if (primitives == 1) // surfaces
		{
			surface = (surface - 1) % 6;
			model.setSurfacesRendered(surface);
			Data.setSurfacesRendered(surface);
			model.setEdgesRendered();
			Data.setEdgesRendered();
		}
		else if (primitives == 2) // edges
		{
			edge = (edge - 1) % 12;
			model.setSurfacesRendered();
			Data.setSurfacesRendered();
			model.setEdgesRendered(edge);
			Data.setEdgesRendered(edge);
		}
		updateFlag = true;
		break;
	}
	case 61: // = key pressed - choose egde or surface to be rendered
	{
		if (primitives == 1) // surfaces
		{
			surface = (surface + 1) % 6;
			model.setSurfacesRendered(surface);
			Data.setSurfacesRendered(surface);
			model.setEdgesRendered();
			Data.setEdgesRendered();
		}
		else if (primitives == 2) // edges
		{
			edge = (edge + 1) % 12;
			model.setSurfacesRendered();
			Data.setSurfacesRendered();
			model.setEdgesRendered(edge);
			Data.setEdgesRendered(edge);
		}
		updateFlag = true;
		break;
	}
	case 126: // ` key pressed - choose number of camera parameters
	{
		int numParams, parameters;
		vector <State> tmpStates;
		cout << "Choose number of camera parameters: ";
		cin >> numParams;
		cout << "\n Choose which camera parameters: ";
		for (int i = 0; i < numParams; i++)
		{
			cin >> parameters;
			tmpStates.push_back((State)parameters);
		}
		sort(tmpStates.begin(), tmpStates.end());
		state.clear();
		state = tmpStates;
		virtualCam.setParamsNum(numParams);
	}
	default:
	{
		updateFlag = false;
		break;
	}
	}
}