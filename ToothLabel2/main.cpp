// gum boolean operation.
//
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#ifdef WIN32
#include <windows.h>
#endif
#include <iostream>
#include <strstream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include "windows.h"   
#include <io.h>

#include <GL/freeglut.h>
#include "TrackBall.h"
#include "FileManager.h"
#include "ToothSegmentation/ToothSegmentation.h"

using namespace std;
using namespace MeshSegmentation;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

enum EnumDisplayMode
{
	FLATSHADED = 50,
	COLORSMOOTHSHADED,
	SEGMENTATION,
	LABELMAPPING,
	SIMPLIFYRESULT,
	TRAINMODEL,
	TESTMODEL,
	TESTPREDICT,
	TESTREFINE,
	TESTFUZZY,
	LABELBOUNDARY,
	PREDICT,
	REFINE,
	PREDICT0,
	PREDICT1,
	PREDICT2,
	PREDICT3,
	PREDICT4,
	PREDICT5,
	REFINE0,
	REFINE1,
	REFINE2,
	REFINE3,
	REFINE4,
	REFINE5,
	FINALRESULT,
	REFINEBOUNDARY,
	GEODESICDISTANCE,
	FUZZYRESULT,
	PREDIT_PROB_DISTRIBUTION,
	BLEND_PROB_DISTRIBUTION,
	BOUNdARY_LEVEL,
	CANCAVE_REGION,
	SEED_POINTS,
	ROUGH_RESULT,
	SIMPLIFY_RESULT,
	PCAANALYSIS
};

// global variables
int displayMode = FLATSHADED;					// current display mode
int mainMenu, displayMenu;						// glut menu handlers
int winWidth = 1200, winHeight = 900;
double winAspect;
string winName = "Tooth Segmentation";

double zNear = 1.0, zFar = 100.0;				// clipping
double g_fov = 45.0;
Vector3d g_center;
double g_sdepth, sdepth = 10, xtrans = 0, ytrans = 0, lastNccX = 0.0, lastNccY = 0.0;
double zDelta;									// zoom in, zoom out
int viewport[4];
bool isLeftDown = false, isRightDown = false, isMidDown = false;
TrackBall trackball;

//string rootFilePath = "D:/tooth_seg/tooth_data2/part1/5/";
string rootFilePath = "E:/tooth_data/part1/5/"; // 2/1-4;

//string pcaDataFile = "D:/tooth_seg/forbat/pca.txt";
string pcaDataFile = "E:/tooth_data/parameters/pca.txt";
size_t currentFileIdx = 24; //part2/3, idx=16 

int classNum = 8;			// ÖÐÇÐÑÀ²àÇÐÑÀ: 31; ¼âÑÀ: 32; Ç°Ä¥ÑÀ: 33; Ä¥ÑÀ: 34
bool justLoadLabel = false;

string filePath = "E:\\LAB\\Tooth\\Model\\";
string modelName = "LXB2L.obj";
string oldModelName = "U125_train.obj";

string trainModelName = "U200_train.obj";
string testModelName = "L200_test.obj";

vector<string> modelPathList;

Mesh tooth, tooth_train, tooth_test, tooth_simplified, U125_train;
bool simplifyOnoff = false;
float lambda = 50.0f;
float lambda2 = 50.0f;
vector<int> labelList;
vector<Vector3d> colorList;

int pcaToothIdx = 1;
ToothPCAInfoList toothPCAList;


// tmp
FaceList gumfFlist;

// functions
void ScreenToNCC(int x, int y, float & nccX, float & nccY);
void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax);
void InitGL();
void InitMenu();
void LoadMesh(Mesh & mesh, string & modelName, bool simplify_flag = false);
void LoadLabelsGTPr(Mesh & mesh, string & modelName);
bool LoadLabels(const char * filename, vector<int> & labels, vector<vector<float>> & probList);
void LoadNewModel();
void CreateColorList();
void InitPara();
void ReadFiles();


// window related 
void MenuCallback(int value);
void ReshapeFunc(int width, int height);

// rendering functions
void DisplayFunc();
void DrawSmoothShaded(Mesh & mesh);
void DrawFlatShaded(Mesh & mesh, int groupID = -1);
void DrawColorSmoothShaded(Mesh & mesh);

// groupID = -1: all draw; flag = 0: label; flag = 1, p_label, flag = 2, r_label;
void DrawSegmentationFlatShaded(Mesh & mesh, int groupID = -1, int flag = 0);
void DrawLayerSegmentationFlatShaded(Mesh & mesh, int layer, int flag);
void DrawLabelBoundaryFlatShaded(Mesh & mesh);
void DrawRefineBoundaryFlatShaded(Mesh & mesh, int level);
void DrawFuzzyResultFlatShaded(Mesh & mesh);
void DrawProbFlatShaded(Mesh & mesh, int dim, int flag);
void DrawGeodesicDistanceFlatShaded(Mesh & mesh);
void DrawBoundaryLevelFlatShaded(Mesh & mesh, int start_level, int end_level);
void DrawCancavePointsFlatShaded(Mesh & mesh);
void DrawSeedPointsFlatShaded(Mesh & mesh);
void DrawTmpLabelFlatShaded(Mesh & mesh);
void DrawPCAFlatShaded(ToothPCAInfoList & toothPCAList, int & pcaToothIdx);

// input related glut functions
void KeyboardFunc(unsigned char ch, int x, int y);
void SpecialKeyFcn(GLint specialKey, GLint xMouse, GLint yMouse);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void MouseWheelFunc(int button, int dir, int x, int y);


int main(int argc, char *argv[]) {
	CreateColorList();
	InitPara();
	ReadFiles();
	// glut initialization functions:
	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	LoadMesh(tooth, filePath+modelName, simplifyOnoff);
	SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
	//LoadMesh(U125_train, filePath + oldModelName);
	
	//CompareNegativeCurvatureAndCancaveRegion();
	//ExportCurvatureAndBoundaryLevel(filePath + modelName);
	//SaveBoundaryFace(filePath + modelName);
	//tooth.FindToothGumBoundary(boundaryPropagate);
	//ExportToothGum4Labels(filePath + modelName);
	//Specifiy2Class4();
	//SaveRefineLabel(filePath + modelName);
	//tooth.WriteObjFile((filePath + "U_stl2obj.obj").c_str());
	//tooth.SaveMaxConnectedComponet((filePath + "U_stl2obj.obj").c_str());
	//MeshSimplify();
	//MeshSimplify1();
	

	//LoadMesh(tooth_test, testModelName);
	//Mapping(tooth, tooth_test);

	// start glutMainLoop -- infinite loop
	glutMainLoop();
	return 0;
}

void InitPara()
{
	switch (classNum)
	{
	case 2:
		labelList = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		//labelList = { 0, 11 };
		lambda = 5;
		break;
	case 24:
		labelList = { 0, 1, 2, 3 };
		lambda = 10;
		break;
	case 4:
		labelList = { 0, 11, 13, 14, 16 };	// 4 class(ÑÀö¸   ÇÐÑÀ   ¼âÑÀ   Ç°Ä¥ÑÀ   Ä¥ÑÀ)
		lambda = 30;
		break;
	case 8: 
		labelList = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		lambda = 100;
		break;
	case 9:
		labelList = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		lambda = 20;
		break;
	case 17:
		labelList = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		lambda = 20;
		break;
	case 31:
		labelList = { 0, 11, 12 };		// ÇÐÑÀ
		lambda = 30;
		break;
	case 32:
		labelList = { 0, 13};			// ¼âÑÀ
		lambda = 30;
		break;
	case 33:
		labelList = { 0, 14, 15 };		// Ç°Ä¥ÑÀ
		lambda = 30;
		break;
	case 34:
		labelList = { 0, 16, 17, 18 };	// Ä¥ÑÀ
		lambda = 100;
		break;
	default:
		break;
	}
}

void ReadFiles()
{

	struct _finddata_t fa;
	long long fHandle;
	string ss = rootFilePath + "*";
	if ((fHandle = _findfirst(ss.c_str(), &fa)) == -1L)//ÕâÀï¿ÉÒÔ¸Ä³ÉÐèÒªµÄÄ¿Â¼ 
	{
		printf("there is no file\n");
	}
	else
	{
		do
		{
			string ss(fa.name);
			if (ss != "." && ss != "..")
			{
				string s = rootFilePath + ss+"/";
				modelPathList.push_back(s);
				//cout << s << endl;
			}
		} while (_findnext(fHandle, &fa) == 0);
		_findclose(fHandle);
		if (modelPathList.size() > 0)
			filePath = modelPathList[currentFileIdx];
	}
}


void ScreenToNCC(int x, int y, float & nccX, float & nccY)
{
	nccX = (2.0f*x - viewport[2]) / viewport[2];
	nccY = (viewport[3] - 2.0f*y) / viewport[3];
}

void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax)
{
	double radius = bmax.Distance(bmin);
	g_center = 0.5 * (bmin + bmax);
	zNear = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
	zFar = zNear + 2.0 * radius;
	g_sdepth = zNear + radius;
	zNear *= 0.01;
	zFar *= 10;
	sdepth = g_sdepth;
	zDelta = sdepth / 40;
}


void InitGL()
{
	GLfloat light0Position[] = { 0, 1, 0, 1.0 };

	// initialize GLUT stuffs
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(winWidth, winHeight);
	glutCreateWindow(winName.c_str());

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glPolygonOffset(1.0, 1.0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glLightfv(GL_LIGHT0, GL_POSITION, light0Position);
	glEnable(GL_LIGHT0);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glutReshapeFunc(ReshapeFunc);
	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutSpecialFunc(SpecialKeyFcn);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
	glutMouseWheelFunc(MouseWheelFunc);
}

// init right-click menu
void InitMenu()
{
	displayMenu = glutCreateMenu(MenuCallback);
	glutAddMenuEntry("Flat Shaded", FLATSHADED);
	glutAddMenuEntry("Color Smooth Shaded", COLORSMOOTHSHADED);
	glutAddMenuEntry("Ground Truth Flat Shaded", SEGMENTATION);
	glutAddMenuEntry("Simplify Flat Shaded", SIMPLIFYRESULT);
	glutAddMenuEntry("Train Data Flat Shaded", TRAINMODEL);
	glutAddMenuEntry("Test Data Flat Shaded", TESTMODEL);
	glutAddMenuEntry("Test Data Predict Flat Shaded", TESTPREDICT);
	glutAddMenuEntry("Test Data Refine Flat Shaded", TESTREFINE);
	glutAddMenuEntry("Test Data Fuzzy Flat Shaded", TESTFUZZY);
	//glutAddMenuEntry("Boundary Flat Shaded", LABELBOUNDARY);
	glutAddMenuEntry("Predict Flat Shaded", PREDICT);
	glutAddMenuEntry("Refine Flat Shaded", REFINE);
	//glutAddMenuEntry("Refine Boundary Flat Shaded", REFINEBOUNDARY);
	//glutAddMenuEntry("Geodesic Boundary Flat Shaded", GEODESICDISTANCE);
	glutAddMenuEntry("Fuzzy Result Flat Shaded", FUZZYRESULT);
	//glutAddMenuEntry("Predict Distribution Flat Shaded", PREDIT_PROB_DISTRIBUTION);
	//glutAddMenuEntry("Blending Distribution Flat Shaded", BLEND_PROB_DISTRIBUTION);
	//glutAddMenuEntry("Boundary Level Flat Shaded", BOUNdARY_LEVEL);
	//glutAddMenuEntry("Cancave points Flat Shaded", CANCAVE_REGION);
	glutAddMenuEntry("Seed points Flat Shaded", SEED_POINTS);
	glutAddMenuEntry("Rough Result Flat Shaded", ROUGH_RESULT);
	glutAddMenuEntry("PCA Flat Shaded", PCAANALYSIS);


	mainMenu = glutCreateMenu(MenuCallback);
	glutAddSubMenu("Display", displayMenu);
	glutAddMenuEntry("Load New Model", 100);
	glutAddMenuEntry("Ground Truth Flat Shaded", SEGMENTATION);
	glutAddMenuEntry("Label Mapping Flat Shaded", LABELMAPPING);
	glutAddMenuEntry("2 Class Predict Result", PREDICT0);
	glutAddMenuEntry("2 Class Refine Result", REFINE0);
	glutAddMenuEntry("4 Class Predict Result", PREDICT1);
	glutAddMenuEntry("4 Class Refine Result", REFINE1);
	glutAddMenuEntry("8 Class Predict Result", PREDICT2);
	glutAddMenuEntry("8 Class Refine Result", REFINE2);
	glutAddMenuEntry("17 Class Refine Result", REFINE3);
	glutAddMenuEntry("Directly 8 Class Predict Result", PREDICT4);
	glutAddMenuEntry("Directly 8 Class Refine Result", REFINE4);
	glutAddMenuEntry("Directly 17 Class Refine Result", REFINE5);
	glutAddMenuEntry("Exit", 99);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void LoadNewModel()
{
	OpenModelFile openModel;
	string fullPath;
	BOOL bSel = openModel.OpenNewModel(NULL, fullPath, modelName);
	filePath = fullPath.substr(0, fullPath.find_last_of("\\"))+'/';
	if (bSel)
	{
		LoadMesh(tooth, filePath + modelName, simplifyOnoff);
		SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
		//LoadMesh(U125_train, filePath + oldModelName);		
	}
}

// Load mesh file
void LoadMesh(Mesh & mesh, string & modelName, bool simplfy_flag)
{
	string modelExtensions = modelName.substr(modelName.length() - 3, modelName.length());

	if (!mesh.LoadModel(modelName.c_str()))
		return;

	mesh.DisplayMeshInfo();

	/*
	if (mesh.fList.size() > 0)
	{
		LoadLabelsGTPr(mesh, modelName);
	}

	if (strcmp(modelExtensions.c_str(), "stl") == 0)
	{
		
		//LoadMesh(tooth_train, filePath + trainModelName);
		LoadMesh(tooth_test, filePath + testModelName);
	}

	if (justLoadLabel) return;
	
	ToothSegmentation ts;
	SimplifyParameters sp;
	MultiLabelGraphCutParameters mlgcp;
	ImprovedFuzzyClusteringParameters ifcp;
	LeftRightSeparationParameters lrsp;
	SplitJointedTeethParameters sjtp;
	BoundarySmoothingParameters bsp;
	
	int p = max(0, (int)(modelName.find_last_of("/") + 1));
	char type = modelName[p];
	sp.modelType = type;
	lrsp.modelType = type;
	sjtp.modelType = type;

	if (simplfy_flag)
	{
		sp.labelBased = false;
		cout << sp.modelType << endl;
		if (strcmp(modelExtensions.c_str(), "stl") == 0)
		{
			ts.Simplify(mesh, tooth_simplified, sp);
			for (size_t i = 0; i < tooth_simplified.fList.size(); i++)
			{
				Face * f = tooth_simplified.fList[i];
				f->SetColor(colorList[f->Label() % 20]);
			}
		}
	}
	
	if (strcmp(modelExtensions.c_str(), "stl") == 0 && !tooth_test.fList.empty())
	{
		ts.MeshANNMapping(mesh, tooth_test);

		mlgcp.ResetParameters();
		mlgcp.lambda = lambda2;
		ts.MultiLabelOptimizeBoundary(mesh, mlgcp);

		//ifcp.propagate = 16;
		ifcp.angDistThreshold = 0.3f;
		ifcp.yitaAngDist = 0.2f;
		ifcp.yita = 0.1f;

		//ifcp.geodesicSigma = 3.0f;
		//ifcp.yita = 0.5f;
		//ifcp.useAngDist = false;
		ts.MultiLabelFuzzyOptimizeBoundary(mesh, ifcp);
		ts.FillGapBetweenTeeth(mesh);
		ts.RemoveSmallComponent(mesh, 3, 500.0f);
		ts.SmoothBoundary(mesh, bsp);
		ts.RemoveSmallComponent(mesh, 3, 500.0f);	
		
		//ts.ToothPCAAnalysis(mesh, toothPCAList, 'U', labelType);
	}
	
	if (strcmp(modelExtensions.c_str(), "obj") == 0)
	{
		mlgcp.ResetParameters();
		mlgcp.lambda = lambda;
		ts.MultiLabelOptimizeBoundary(mesh, mlgcp);

		// left right energy display
		//for (size_t i = 0; i < mesh.fList.size(); i++)
		//{
		//	Face * f = mesh.fList[i];
		//	f->source_weight = f->sink_weight = 0.0f;
		//}

		if (classNum == 8 || classNum == 9 || classNum == 17)
		{
			if (classNum == 9 || classNum == 17)
			{
				mlgcp.ResetParameters();
				mlgcp.lambda = lambda2;
				mlgcp.optimzeGingiva = false;
				ts.MultiLabelOptimizeBoundary(mesh, mlgcp);
			}

			if (classNum == 8)
			{
				//ts.JudgeLeftRight(mesh);
				ts.JudgeLeftRight(mesh, lrsp);
			}
			else if (classNum == 9)
			{
				lrsp.load2ClassesProb = false;
				ts.JudgeLeftRight(mesh, lrsp);
			}

			ts.RemoveSmallComponent(mesh);

			
			if (ts.LoadPCAData(pcaDataFile))
			{
				// no prior
				ts.DetectJointedTeeth(mesh, toothPCAList, sjtp);

				// prior

			}

			//sjtp.labelType = 'g';
			//ts.ToothBoundaryPCAAnalysis(mesh, toothPCAList, sjtp);
		}
	}
		
	
	if (classNum == 2)
	{
		//ts.MultiLabelFuzzyOptimizeBoundary(mesh, ifcp);
		//ts.FuzzyOptimizeBoundary(mesh, ifcp);
		ts.RemoveSmallComponent(mesh);
	}
*/
	xtrans = ytrans = 0.0;
}

void LoadLabelsGTPr(Mesh & mesh, string & modelName)
{
	string modelPrefix = modelName.substr(0, modelName.length() - 4);
	string labelsGTPath = modelPrefix + ".txt";					// Ground Truth
	string labelsPrPath = modelPrefix + "_p4.txt";				// Predict

	vector<int> labels;
	vector<vector<float>> probList;

	LoadLabels(labelsGTPath.c_str(), labels, probList);

	if (labels.size() >= mesh.fList.size())
	{
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->SetLabel(labels[mesh.fList[i]->LabelPath()]);
			mesh.fList[i]->SetColor(colorList[labels[mesh.fList[i]->LabelPath()] % 20]);
		}
	}

	labels.clear();
	probList.clear();

	LoadLabels(labelsPrPath.c_str(), labels, probList);
	if (labels.size() >= mesh.fList.size() && probList.size() >= mesh.fList.size())
	{
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->p_label = labels[mesh.fList[i]->LabelPath()];
			mesh.fList[i]->r_label = mesh.fList[i]->p_label;
			mesh.fList[i]->prob.assign(probList[mesh.fList[i]->LabelPath()].begin(), probList[mesh.fList[i]->LabelPath()].end());
		}
	}

	if (justLoadLabel)
	{
		string modelExtensions = modelName.substr(modelName.length() - 3, modelName.length());
		// _r, _f
		string rlabelfile;
		if (modelExtensions == "stl")
		{
			rlabelfile = modelPrefix + "_r.txt";
		}
		else
		{
			rlabelfile = labelsPrPath;
			rlabelfile[rlabelfile.length() - 6] = 'r';
		}
		
		labels.clear();
		probList.clear();
		LoadLabels(rlabelfile.c_str(), labels, probList);
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->LabelPath() < labels.size())
				f->r_label = labels[f->LabelPath()];
		}

		string flabelfile;
		if (modelExtensions == "stl")
		{
			flabelfile = modelPrefix + "_f.txt";
		}
		else
		{
			flabelfile = labelsPrPath;
			flabelfile[flabelfile.length() - 6] = 'f';
		}
		labels.clear();
		probList.clear();
		LoadLabels(flabelfile.c_str(), labels, probList);
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->LabelPath() < labels.size())
				f->fuzzy_label = labels[f->LabelPath()];
		}
	}

	// load p0, r0, p1, r2, r3, p4, r4
	for (int k = 0; k < 5; k++)
	{
		stringstream ss;
		string s;
		ss << k;
		ss >> s;

		string p_label_file = modelPrefix + "_p" + s + ".txt";
		string r_label_file = modelPrefix + "_r" + s + ".txt";
		string f_label_file = modelPrefix + "_f" + s + ".txt";
		
		labels.clear();
		probList.clear();
		LoadLabels(p_label_file.c_str(), labels, probList);

		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->LabelPath() < (int)labels.size())
			{
				f->p_label_list.push_back(labels[f->LabelPath()]);
				if (k == 0)
				{
					// add two classification prob list
					f->twoClassesProb.assign(probList[f->LabelPath()].begin(), probList[f->LabelPath()].end());
				}
			}
			else
			{
				f->p_label_list.push_back(0);
			}
		}
		
		labels.clear();
		probList.clear();
		LoadLabels(r_label_file.c_str(), labels, probList);
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->LabelPath() < (int)labels.size())
				f->r_label_list.push_back(labels[f->LabelPath()]);
			else
				f->r_label_list.push_back(0);
		}

		labels.clear();
		probList.clear();
		LoadLabels(f_label_file.c_str(), labels, probList);
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->LabelPath() < (int)labels.size())
				f->f_label_list.push_back(labels[f->LabelPath()]);
			else
				f->f_label_list.push_back(0);
		}
	}
}

bool LoadLabels(const char * filename, vector<int> & labels, vector<vector<float>> & probList)
{
	if (filename == NULL || strlen(filename) == 0)
		return false;

	ifstream ifs(filename);
	if (ifs.fail())
		return false;

	labels.clear();

	char buf[1024];
	do
	{
		ifs.getline(buf, 1024);
		istrstream iss(buf);
		
		
		int x = -1, y;
		
		iss >> x >> y;
		if (x > 0)
		{
			labels.push_back(y);
			vector<float> prob;
			float z = -1.0f;
			iss >> z;
			while (z > -0.1)
			{
				prob.push_back(z);
				z = -1.0f;
				iss >> z;
			}
			probList.push_back(prob);
		}
	} while (!ifs.eof());

	ifs.close();
	return true;
}


void CreateColorList()
{
	int x = 0, y = 0, z = 0;

	colorList.push_back(Vector3d(0.5, 0.5, 0.5));
	

	for (int i = 0; i < 50; i++)
	{
		x = rand() % 256;
		y = rand() % 256;
		z = rand() % 256;
		colorList.push_back(Vector3d(x / 256.0, y / 256.0, z / 256.0));
	}

	colorList[13] = Vector3d(0.1, 0.8, 0.3);

	colorList[20] = Vector3d(0.9, 0.9, 0.9);
	colorList[21] = Vector3d(1.0, 0.0, 0.0);
	colorList[22] = Vector3d(0.0, 1.0, 0.0);
	colorList[23] = Vector3d(0.0, 0.0, 1.0);
}

// GLUT menu callback function
void MenuCallback(int value)
{
	vector<int> newFList;
	switch (value)
	{
	case 100:
		LoadNewModel();
		break;
	case 99:
		exit(0);
		break;
	default:
		displayMode = value;
		break;
	}
	glutPostRedisplay();
}

// GLUT reshape callback function
void ReshapeFunc(int width, int height)
{
	winWidth = width;
	winHeight = height;
	winAspect = (double)width / (double)height;
	glViewport(0, 0, width, height);
	glGetIntegerv(GL_VIEWPORT, viewport);
	glutPostRedisplay();
}


// GLUT display callback function
void DisplayFunc()
{
	float rotation[16];
	trackball.m_rotation.GetMatrix(rotation);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, winAspect, zNear, zFar);
	//glOrtho(-2.0, 2.0, -2.0, 2.0, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslated(xtrans, ytrans, -sdepth);
	glMultMatrixf(rotation);
	glTranslated(-g_center[0], -g_center[1], -g_center[2]);
	
	switch (displayMode)
	{
	case FLATSHADED:
		DrawFlatShaded(tooth);
		break;
	case COLORSMOOTHSHADED:
		DrawColorSmoothShaded(tooth);
		break;
	case SEGMENTATION:
		DrawSegmentationFlatShaded(tooth);
		break;
	case LABELMAPPING:
		DrawSegmentationFlatShaded(tooth, -1, 3);
		break;
	case SIMPLIFYRESULT:
		DrawSegmentationFlatShaded(tooth_simplified);
		break;
	case TRAINMODEL:
		DrawSegmentationFlatShaded(tooth_train);
		break;
	case TESTMODEL:
		//DrawSegmentationFlatShaded(tooth_test, -1, 2);
		DrawSegmentationFlatShaded(tooth_test);
		break;
	case TESTPREDICT:
		DrawSegmentationFlatShaded(tooth_test, -1, 1);
		break;
	case TESTREFINE:
		DrawSegmentationFlatShaded(tooth_test, -1, 2);
		break;
	case TESTFUZZY:
		DrawFuzzyResultFlatShaded(tooth_test);
		break;
	case LABELBOUNDARY:
		DrawLabelBoundaryFlatShaded(tooth);
		break;
	case PREDICT:
		DrawSegmentationFlatShaded(tooth, -1, 1);
		break;
	case REFINE:
		DrawSegmentationFlatShaded(tooth, -1, 2);
		break;
	case PREDICT0:
		DrawLayerSegmentationFlatShaded(tooth_test, 0, 1);	// layer
		break;
	case REFINE0:
		DrawLayerSegmentationFlatShaded(tooth_test, 0, 2);	// layer
		break;
	case PREDICT1:
		DrawLayerSegmentationFlatShaded(tooth_test, 1, 1);	// layer
		break;
	case REFINE1:
		DrawLayerSegmentationFlatShaded(tooth_test, 1, 2);	// layer
		break;
	case PREDICT2:
		DrawLayerSegmentationFlatShaded(tooth_test, 2, 1);	// layer
		break;
	case REFINE2:
		DrawLayerSegmentationFlatShaded(tooth_test, 2, 2);	// layer
		break;
	case REFINE3:
		DrawLayerSegmentationFlatShaded(tooth_test, 3, 2);	// layer
		break;
	case PREDICT4:
		DrawLayerSegmentationFlatShaded(tooth_test, 4, 1);	// layer
		break;
	case REFINE4:
		DrawLayerSegmentationFlatShaded(tooth_test, 4, 2);	// layer
		break;
	case REFINE5:
		DrawLayerSegmentationFlatShaded(tooth, 1, 3);	// layer
		break;
	case FINALRESULT:
		DrawLayerSegmentationFlatShaded(tooth, 1, 2);	// layer
		break;
	case REFINEBOUNDARY:
		DrawRefineBoundaryFlatShaded(U125_train, 0);
		break;
	case GEODESICDISTANCE:
		DrawGeodesicDistanceFlatShaded(tooth);
		break;
	case FUZZYRESULT:
		DrawFuzzyResultFlatShaded(tooth);
		break;
	case PREDIT_PROB_DISTRIBUTION:
		DrawProbFlatShaded(tooth, 1, 0);
		break;
	case BLEND_PROB_DISTRIBUTION:
		DrawProbFlatShaded(tooth, 1, 1);
		break;
	case BOUNdARY_LEVEL:
		DrawBoundaryLevelFlatShaded(tooth, 4, 15);
		break;
	case CANCAVE_REGION:
		DrawCancavePointsFlatShaded(tooth);
		break;
	case SEED_POINTS:
		DrawSeedPointsFlatShaded(tooth);
		break;
	case ROUGH_RESULT:
		DrawTmpLabelFlatShaded(tooth);
		break;
	case PCAANALYSIS:
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT);
		DrawPCAFlatShaded(toothPCAList, pcaToothIdx);
		glClearColor(0, 0, 0, 1);
		break;
	default:
		break;
	}

	glutSwapBuffers();
}


void DrawFlatShaded(Mesh & mesh, int groupID)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		if (groupID < 0 || groupID == fList[i]->GroupID())
		{
			Face *f = fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			normal /= normal.L2Norm();
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

// groupID 0: show all triangles; 
// flag 0: groundtruth; flag 1: predict label; flag 2: refine label
void DrawSegmentationFlatShaded(Mesh & mesh, int groupID, int flag)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		if (groupID < 0 || groupID == fList[i]->GroupID())
		{
			Face *f = fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			normal /= normal.L2Norm();

			Vector3d color(0.4, 0.4, 1.0);
			if (flag == 0)
			{
				color = f->Color();
			}
			else if (flag == 1)
			{
				color = colorList[labelList[f->p_label] % 20];
			}
			else if (flag == 2)
			{
				color = colorList[labelList[f->r_label] % 20];
			}
			else if (flag == 3)
			{
				color = colorList[f->simplfiyLabel % 20];
			}

			glColor3d(color[0], color[1], color[2]);
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

// layer: indicate the classification layer, such as 2 Classsification is no. 0 layer
// flag 0: groudtruth; flag 1: predict; flag 2: refine 
void DrawLayerSegmentationFlatShaded(Mesh & mesh, int layer, int flag)
{
	vector<int> labelMap;
	switch (layer)
	{
	case 0:
		labelMap = { 0, 11 };
		break;
	case 1:
		//labelMap = { 0, 11, 13, 14, 16 };
		labelMap = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		break;
	case 2:case 4:
		labelMap = { 0, 11, 12, 13, 14, 15, 16, 17, 18 };
		break;
	case 3:case 5:
		labelMap = { 0, 11, 12, 13, 14, 15, 16, 17, 18, 21, 22, 23, 24, 25, 26, 27, 28 };
		break;
	default:
		break;
	}

	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {

		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color(0.4, 0.4, 1.0);

		if (flag == 0)
		{
			color = f->Color();
		}
		else if (flag == 1)
		{
			if (f->p_label_list.size()>(size_t)layer)
				color = colorList[labelMap[f->p_label_list[layer]] % 20];
		}
		else if (flag == 2)
		{
			if (f->r_label_list.size()>(size_t)layer)
				color = colorList[labelMap[f->r_label_list[layer]] % 20];
		}
		else if (flag == 3)
		{
			if (f->r_label_list.size()>(size_t)layer)
				color = colorList[labelMap[f->f_label_list[layer]] % 20];
		}

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}


void DrawLabelBoundaryFlatShaded(Mesh & mesh)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color;
		color = f->Color();
		if (!f->labelBoundary)
		{
			color = colorList[20];
		}

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawRefineBoundaryFlatShaded(Mesh & mesh, int level)
{
	if (mesh.fList.size() == 0) return;
	if (mesh.fList[0]->r_label_list.size() <= (size_t)level) return;

	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color;
		color = color = colorList[f->r_label_list[level] % 20];
		if (f->labelBoundary)
		{
			color = colorList[f->r_label_list[level] + 20];
		}

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawFuzzyResultFlatShaded(Mesh & mesh)
{
	if (mesh.fList.size() == 0) return;

	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color = colorList[labelList[f->fuzzy_label] % 20];

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

// flag 0: predict prob distribution
// flag 1: blending prob distribution
void DrawProbFlatShaded(Mesh & mesh, int dim, int flag)
{
	if (mesh.fList.size() == 0) return;

	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color(0, 0, 0);
		if (flag == 0)
		{
			if (f->prob.size() > (size_t)dim)
			{
				color[0] = (double)f->prob[dim];
				//color[1] = 0.5;
				color[2] = (double)(1 - f->prob[dim]);
			}
			else color = Vector3d(0.4f, 0.4f, 1.0f);

		}
		else if (flag == 1)
		{
			if (f->blend_prob.size() > (size_t)dim)
			{
				color[0] = (double)f->blend_prob[dim];
				//color[1] = 0.5;
				color[2] = (double)(1 - f->blend_prob[dim]);
			}
			else color = Vector3d(0.4f, 0.4f, 1.0f);
		}

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawGeodesicDistanceFlatShaded(Mesh & mesh)
{
	if (mesh.fList.size() == 0) return;

	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color(0.4, 0.4, 1.0);
		if (f->labelBoundary)
		{
			float ratio = f->tmpGeodesicDistance;
			color[0] = ratio>1.0 ? 1.0 : ratio;
			color[2] = 1 - ratio < 0 ? 0 : 1 - ratio;
		}


		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawBoundaryLevelFlatShaded(Mesh & mesh, int start_level, int end_level)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {

			Face *f = fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			normal /= normal.L2Norm();

			Vector3d color(0.4, 0.4, 1.0);

			Vertex * v[3] = { f->HalfEdge()->Start(), f->HalfEdge()->End(), f->HalfEdge()->Next()->End() };

			bool flag = false;
			for (int j = 0; j < 3; j++)
			{
				if (v[j]->boundaryLevel >= start_level && v[j]->boundaryLevel < end_level)
				{
					flag = true;
					break;
				}
			}

			if (flag)
			{
				color = f->Color();
			}

			glColor3d(color[0], color[1], color[2]);
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawCancavePointsFlatShaded(Mesh & mesh)
{
	FaceList fList = mesh.Faces();
	VertexList vList = mesh.Vertices();
	glShadeModel(GL_FLAT);
	glBegin(GL_POINTS);
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->cancave_flag)
		{
			const Vector3d & pos = vList[i]->Position();
			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);

	glEnable(GL_LIGHTING);
	glPointSize(5.0f);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {

		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color(0.4, 0.4, 1.0);
		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());

	}
	glEnd();


	glDisable(GL_LIGHTING);
}



void DrawSeedPointsFlatShaded(Mesh & mesh)
{
	double thYCoord = 0.65*mesh.maxCoord[1] + 0.35*mesh.minCoord[1];

	FaceList fList = mesh.Faces();
	VertexList vList = mesh.Vertices();
	glShadeModel(GL_FLAT);

	// draw local heightest points
	glPointSize(8.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 0.0f, 0.0f);
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->local_heightest==0)
		{
			const Vector3d & pos = vList[i]->Position();

			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);

	// draw protected points
	glPointSize(12.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 0.0f);
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->protect_flag && vList[i]->seed_flag)
		//if (vList[i]->protect_flag)
		{
			const Vector3d & pos = vList[i]->Position();

			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);

	// draw cutting points
	glPointSize(12.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 0.0f, 1.0f);
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->cutting_point)
		{
			const Vector3d & pos = vList[i]->Position();

			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);

	// draw points
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	glColor3f(0.0f, 1.0f, 0.0f);
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->seed_flag)
		{
			const Vector3d & pos = vList[i]->Position();
			
			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);

	// draw lines
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.5f, 1.0f);
	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex * u = vList[i];

		if (!u->seed_flag)
			continue;

		OneRingVertex ring(u);
		Vertex * v = NULL;
		while ((v = ring.NextVertex()) != NULL)
		{
			if (!v->seed_flag)
				continue;

			const Vector3d & pos1 = u->Position();
			const Vector3d & pos2 = v->Position();
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
		}
	}
	glEnd();

	// draw cutting points
	//glPointSize(8.0f);
	//glBegin(GL_POINTS);
	//glColor3f(1.0f, 1.0f, 1.0f);
	//for (size_t i = 0; i < vList.size(); i++)
	//{
	//	if (vList[i]->maxCurv>-14.0f && vList[i]->maxCurv<-4.0f)
	//	{
	//		const Vector3d & pos = vList[i]->Position();

	//		glVertex3dv(pos.ToArray());
	//	}
	//}
	//glEnd();
	//glPointSize(1.0f);

	glEnable(GL_LIGHTING);
	
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {

		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();
		Vector3d color(0.4, 0.4, 1.0);
		if (f->center[1]>thYCoord)
			color = Vector3d(0.1, 0.5, 0.7);

		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());

	}
	glEnd();


	glDisable(GL_LIGHTING);
}

void DrawTmpLabelFlatShaded(Mesh & mesh)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);

	//Vector3d center = 0.5*(mesh.maxCoord + mesh.minCoord);
	//glPointSize(10.0f);
	//glBegin(GL_POINTS);
	//glColor3f(1.0f, 0.0f, 0.0f);
	//glVertex3dv(center.ToArray());
	//glColor3f(0.0f, 0.0f, 1.0f);
	//glVertex3d(0.0f, -10.0f, 0.0f);
	//glEnd();
	//glPointSize(1.0f);

	//
	//glBegin(GL_TRIANGLES);
	//glColor3f(0.0f, 1.0f, 0.0f);
	//glVertex3d(-10.0, 5.0, -10);
	//glVertex3d(0.0, -5.0, -10);
	//glVertex3d(10.0, 5.0, -10);
	//glEnd();

	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {

		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();

		Vector3d color(0.4, 0.4, 1.0);
		//if (f->tmpLabel == 1)
		//{
		//	color = Vector3d(1.0, 0.5, 0.1);
		//}
		//color = colorList[labelList[f->tmpLabel] % 20];
		//color = colorList[mesh.vList[f->v[0]]->simplify_level];
		color[0] = (float)(f->source_weight) / 1000;
		//color[0] = (float)mesh.vList[f->v[0]]->local_heightest / 50;
		//color[1] = 0.0f;
		//color[2] = 0.0f;


		glColor3d(color[0], color[1], color[2]);
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());

	}

	//for (size_t i = 0; i < gumfFlist.size(); i++) {

	//	Face *f = gumfFlist[i];
	//	const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
	//	const Vector3d & pos2 = f->HalfEdge()->End()->Position();
	//	const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
	//	Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
	//	normal /= normal.L2Norm();

	//	Vector3d color(1.0, 1.0, 0.2);

	//	glColor3d(color[0], color[1], color[2]);
	//	glNormal3dv(normal.ToArray());
	//	glVertex3dv(pos1.ToArray());
	//	glVertex3dv(pos2.ToArray());
	//	glVertex3dv(pos3.ToArray());
	//}
	glEnd();
	glDisable(GL_LIGHTING);

	glPointSize(8.0f);
	glBegin(GL_POINTS);
	glColor3f(0.0f, 0.0f, 1.0f);
	for (size_t i = 0; i < mesh.vList.size(); i++)
	{
		Vertex * v = mesh.vList[i];
		if (v->possibleBoundary)
		{
			const Vector3d & pos = v->Position();

			glVertex3dv(pos.ToArray());
		}
	}
	glEnd();
	glPointSize(1.0f);
}

void DrawPCAFlatShaded(ToothPCAInfoList & toothPCAList, int & pcaToothIdx)
{
	if (pcaToothIdx >= toothPCAList.size()) return;

	Vector3d & obbCenter = toothPCAList[pcaToothIdx].obbCenter;
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	for (size_t i = 0; i < toothPCAList[pcaToothIdx].axisList.size(); i++)
	{
		Vector3d & dir = toothPCAList[pcaToothIdx].axisList[i];
		double halfLength = 0.5*toothPCAList[pcaToothIdx].lengthList[i];

		if (i == 0)
			glColor3f(1.0f, 0.0f, 0.0f);
		else if (i == 1)
			glColor3f(0.0f, 1.0f, 0.0f);
		else
			glColor3f(0.0f, 0.0f, 1.0f);

		Vector3d start = obbCenter - halfLength*dir;
		Vector3d end = obbCenter + halfLength*dir;
		glVertex3dv(start.ToArray());
		glVertex3dv(end.ToArray());
	}
	glEnd();
	glLineWidth(1.0f);

	FaceList fList = toothPCAList[pcaToothIdx].fList;
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor4f(1.0f, 1.0f, 0.0f, 0.5f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i < fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
		normal /= normal.L2Norm();
		glNormal3dv(normal.ToArray());
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();

	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t k = 0; k < toothPCAList.size(); k++)
	{
		if (k == pcaToothIdx) continue;
		for (size_t i = 0; i < toothPCAList[k].fList.size(); i++)
		{
			Face *f = toothPCAList[k].fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			normal /= normal.L2Norm();
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	glEnd();
	glDisable(GL_LIGHTING);


}


void DrawSmoothShaded(Mesh & mesh)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++)
	{
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		glNormal3dv(v1->Normal().ToArray());
		glVertex3dv(v1->Position().ToArray());
		glNormal3dv(v2->Normal().ToArray());
		glVertex3dv(v2->Position().ToArray());
		glNormal3dv(v3->Normal().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}


void DrawColorSmoothShaded(Mesh & mesh)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++)
	{
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		glNormal3dv(v1->Normal().ToArray());
		glColor3dv(v1->Color().ToArray());
		glVertex3dv(v1->Position().ToArray());
		glNormal3dv(v2->Normal().ToArray());
		glColor3dv(v2->Color().ToArray());
		glVertex3dv(v2->Position().ToArray());
		glNormal3dv(v3->Normal().ToArray());
		glColor3dv(v3->Color().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void ResetBlendProb()
{
	for (size_t i = 0; i < tooth.fList.size(); i++)
	{
		Face * f = tooth.fList[i];
		f->blend_prob.clear();
		f->blend_prob.assign(f->prob.begin(), f->prob.end());
	}
}

void SaveErrorSimplfiedModel()
{
	string savefile = "D:/error_list.txt";
	ofstream fout(savefile, ios::app);
	if (fout.fail())
		return;

	fout << filePath << endl;
	fout.close();

	cout << "error simplfied model: " << filePath << " saved" << endl;

}

// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y)
{
	int tmp;
	switch (ch)
	{
	case 'o':case 'O':
		LoadNewModel();
		break;
	case 'a':case 'A':
		SaveErrorSimplfiedModel();
		break;
	case 'p':case 'P':
		cin >> tmp;
		if (tmp < 11) break;
		if (tmp > 18 && tmp < 21) break;
		if (tmp>28 && tmp < 31) break;
		if (tmp>38 && tmp < 41) break;
		if (tmp > 48) break;

		if (tmp >= 11 && tmp <= 18)
			pcaToothIdx = tmp - 10;
		else if (tmp >= 21 && tmp <= 28)
			pcaToothIdx = tmp - 12;
		else if (tmp >= 31 && tmp <= 38)
			pcaToothIdx = tmp - 30;
		else if (tmp >= 41 && tmp <= 48)
			pcaToothIdx = tmp - 32;
		break;
	case 27:
		exit(0);
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

void SpecialKeyFcn(GLint specialKey, GLint xMouse, GLint yMouse)
{
	int oldFileIdx = currentFileIdx;
	switch (specialKey)
	{
	case GLUT_KEY_LEFT:
		currentFileIdx--;
		currentFileIdx = currentFileIdx < 0 ? 0 : currentFileIdx;
		break;
	case GLUT_KEY_RIGHT:
		currentFileIdx++;
		currentFileIdx = currentFileIdx < modelPathList.size() ? currentFileIdx : modelPathList.size() - 1;
		break;
	case GLUT_KEY_UP:
		pcaToothIdx++;
		pcaToothIdx = min(pcaToothIdx, 16);
		break;
	case GLUT_KEY_DOWN:
		pcaToothIdx--;
		pcaToothIdx = max(pcaToothIdx, 1);
		break;
	}
	if (oldFileIdx != currentFileIdx)
	{
		filePath = modelPathList[currentFileIdx];
		LoadMesh(tooth, modelPathList[currentFileIdx] + modelName, simplifyOnoff);
		SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
		xtrans = ytrans = 0.0;
		//LoadMesh(U125_train, filePath + oldModelName);
	}
	glutPostRedisplay();
}


void MouseFunc(int button, int state, int x, int y)
{
	float nccX, nccY;
	//if (button == GLUT_RIGHT_BUTTON) exit(0);
	if (button == GLUT_LEFT_BUTTON)
	{
		switch (state)
		{
		case GLUT_DOWN:
			isLeftDown = true;
			ScreenToNCC(x, y, nccX, nccY);
			trackball.Push(nccX, nccY);
			break;
		case GLUT_UP:
			ScreenToNCC(x, y, nccX, nccY);
			isLeftDown = false;
			break;
		}
	}
	if (button == GLUT_MIDDLE_BUTTON)
	{
		switch (state)
		{
		case GLUT_DOWN:
			isMidDown = true;
			ScreenToNCC(x, y, nccX, nccY);
			lastNccX = nccX;
			lastNccY = nccY;
			break;
		case GLUT_UP:
			isMidDown = false;
			break;
		}
	}
}


// GLUT mouse motion callback function
void MotionFunc(int x, int y)
{
	if (isLeftDown)
	{
		float nccX, nccY;
		ScreenToNCC(x, y, nccX, nccY);
		trackball.Move(nccX, nccY);
		glutPostRedisplay();
	}
	else if (isMidDown)
	{	
		float nccX, nccY;
		ScreenToNCC(x, y, nccX, nccY);

		xtrans += 0.2*(nccX-lastNccX)*winAspect*sdepth / zNear;
		ytrans += 0.2*(nccY-lastNccY)*sdepth / zNear;
		lastNccX = nccX;
		lastNccY = nccY;
		glutPostRedisplay();
	}
}

void MouseWheelFunc(int button, int dir, int x, int y)
{
	//static double zDelta = 2;
	if (sdepth > zNear  && dir>0)
	{
		sdepth -= zDelta;
	}
	if (sdepth < zFar && dir < 0)
	{
		sdepth += zDelta;
	}
	glutPostRedisplay();
}


void display()
{
	float delta = 4;
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	glColor3d(0, 0.8, 1.0);
	double angle = 0;
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < 90; i++) {
		glVertex2d(cos(angle*PI / 180.0), sin(angle*PI / 180.0));
		angle += delta;
	}
	glEnd();

	glFlush();
	glutSwapBuffers();
}


