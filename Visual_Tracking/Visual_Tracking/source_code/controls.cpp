#include "../headers/controls.h"

// Display camera parameters
void dispCamParams(Camera virtualCam, Camera realCam)
{
	// Attributes
	HersheyFonts font = FONT_HERSHEY_COMPLEX_SMALL;
	double fontScale = 0.6;
	Scalar colour = (0, 0, 0);
	int thickness = 1;
	int lineType = CV_AA;

	// Display camera parameters
	Mat virtualValues(70, 400, CV_8UC3, Scalar::all(255));
	Mat realValues(70, 400, CV_8UC3, Scalar::all(255));
	// Virtual Camaera Parameters
	string values = "Tx: " + to_string(virtualCam.getTx()) + " Ty: " + to_string(virtualCam.getTy()) + " Tz: " + to_string(virtualCam.getTz());
	putText(virtualValues, values, cvPoint(0, 20), font, fontScale, colour, thickness, lineType);
	values.clear();
	values = "Rx: " + to_string(virtualCam.getThetaX(DEGREES)) + " Ry: " + to_string(virtualCam.getThetaY(DEGREES)) + " Rz: " + to_string(virtualCam.getThetaZ(DEGREES));
	putText(virtualValues, values, cvPoint(0, 50), font, fontScale, colour, thickness, lineType);
	values.clear();
	// Real Camaera Parameters
	values = "Tx: " + to_string(realCam.getTx()) + " Ty: " + to_string(realCam.getTy()) + " Tz: " + to_string(realCam.getTz());
	putText(realValues, values, cvPoint(0, 20), font, fontScale, colour, thickness, lineType);
	values.clear();
	values = "Rx: " + to_string(realCam.getThetaX(DEGREES)) + " Ry: " + to_string(realCam.getThetaY(DEGREES)) + " Rz: " + to_string(realCam.getThetaZ(DEGREES));
	putText(realValues, values, cvPoint(0, 50), font, fontScale, colour, thickness, lineType);
	// Show camera parameters
	const char* virtualCamParams = "Virtual Camera Parameters";
	const char* realCamParams = "Real Camera Parameters";
	imshow(virtualCamParams, virtualValues);
	imshow(realCamParams, realValues);
	waitKey(1);
}

// Keys pressed handler - case sensitive
void keyboardHandler(Camera &virtualCam, Camera &realCam, Cuboid3D &model, Cuboid3D &Data, int &mNum, vector <float> defaultParams, bool &exitFlag, bool &updateFlag, bool &fitFlag, bool &demoFlag)
{
	static bool renderEdges = true;
	static int edge = 0;

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
		virtualCam.setThetaY(ryVirtual, DEGREES);
		updateFlag = true;
		break;
	}
	case 120: // 'x' key pressed - --ryVirtual
	{
		float ryVirtual = virtualCam.getThetaY(DEGREES);
		ryVirtual -= 0.5f;
		virtualCam.setThetaY(ryVirtual, DEGREES);
		updateFlag = true;
		break;
	}
	case 99: // 'c' key pressed - ++rxVirtual
	{
		float rxVirtual = virtualCam.getThetaX(DEGREES);
		rxVirtual += 0.5f;
		virtualCam.setThetaX(rxVirtual, DEGREES);
		updateFlag = true;
		break;
	}
	case 118: // 'v' key pressed - --rxVirtual
	{
		float rxVirtual = virtualCam.getThetaX(DEGREES);
		rxVirtual -= 0.5f;
		virtualCam.setThetaX(rxVirtual, DEGREES);
		updateFlag = true;
		break;
	}
	case 98: // 'b' key pressed - ++rzVirtual
	{
		float rzVirtual = virtualCam.getThetaZ(DEGREES);
		virtualCam.setThetaZ(++rzVirtual, DEGREES);
		updateFlag = true;
		break;
	}
	case 110: // 'n' key pressed - --rzVirtual
	{
		float rzVirtual = virtualCam.getThetaZ(DEGREES);
		virtualCam.setThetaZ(--rzVirtual, DEGREES);
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
		realCam.setThetaY(++ryReal, DEGREES);
		updateFlag = true;
		break;
	}
	case 103: // 'g' key pressed - --ryReal
	{
		float ryReal = realCam.getThetaY(DEGREES);
		realCam.setThetaY(--ryReal, DEGREES);
		updateFlag = true;
		break;
	}
	case 104: // 'h' key pressed - ++rxReal
	{
		float rxReal = realCam.getThetaX(DEGREES);
		realCam.setThetaX(++rxReal, DEGREES);
		updateFlag = true;
		break;
	}
	case 106: // 'j' key pressed - --rxReal
	{
		float rxReal = realCam.getThetaX(DEGREES);
		realCam.setThetaX(--rxReal, DEGREES);
		updateFlag = true;
		break;
	}
	case 107: // 'k' key pressed - ++rzReal
	{
		float rzReal = realCam.getThetaZ(DEGREES);
		realCam.setThetaZ(++rzReal, DEGREES);
		updateFlag = true;
		break;
	}
	case 108: // 'l' key pressed - --rzReal
	{
		float rzReal = realCam.getThetaZ(DEGREES);
		realCam.setThetaZ(--rzReal, DEGREES);
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
		virtualCam.setParams({ defaultParams.begin(), defaultParams.end() - 1 }, defaultState, DEGREES);
		defaultParams[0] += 2.f;
		realCam.setParams({ defaultParams.begin(), defaultParams.end() - 1 }, defaultState, DEGREES);
		updateFlag = true;
		break;
	}
	case 8: // BACKSPACE key pressed - start demo
	{
		fitFlag = true;
		demoFlag = true;
		break;
	}
	case 9: // TAB key pressed - choose primitives to be rendered
	{
		renderEdges = !renderEdges;
		model.setEdgesRendered(renderEdges);
		Data.setEdgesRendered(renderEdges);
		if (renderEdges)
		{
			mNum = (int)defaultParams[6];
			edge = 0;
		}
		else
		{
			mNum = 9;
		}
		updateFlag = true;
		break;
	}
	case 45: // - key pressed - choose egde which edge is going to be rendered at the next frame
	{
		if (!renderEdges)
		{
			int sizeEdges = model.getEdgesSize();
			edge = (--edge % sizeEdges + sizeEdges) % sizeEdges;
			cout << edge << endl;
			model.setEdgesRendered(edge);
			Data.setEdgesRendered(edge);
		}
		updateFlag = true;
		break;
	}
	case 61: // = key pressed - choose egde which edge is going to be rendered at the next frame
	{
		if (!renderEdges)
		{
			edge = ++edge % model.getEdgesSize();
			cout << edge << endl;
			model.setEdgesRendered(edge);
			Data.setEdgesRendered(edge);
		}
		updateFlag = true;
		break;
	}
	case 126: // ` key pressed - choose number of camera parameters
	{
		int numParams, parameters;
		vector <State> state;
		cout << "Choose number of camera parameters: ";
		cin >> numParams;
		cout << "\n Choose which camera parameters: ";
		for (int i = 0; i < numParams; i++)
		{
			cin >> parameters;
			state.push_back(static_cast<State>(parameters));
		}
		virtualCam.setState(state);
	}
	default:
	{
		updateFlag = false;
		break;
	}
	}
}