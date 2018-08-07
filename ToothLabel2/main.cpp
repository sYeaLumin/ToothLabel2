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
#include <direct.h>

#include <GL/freeglut.h>
#include "TrackBall.h"
#include "FileManager.h"
#include "ToothSegmentation/ToothSegmentation.h"
#include "ann_1.1.2\ANN\ANN.h"
#include "FeatureExtractor.h"

using namespace std;
using namespace MeshSegmentation;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

enum EnumDisplayMode
{
	FLATSHADED = 50,
};

// global variables
int displayMode = FLATSHADED;					// current display mode
int mainMenu, displayMenu;						// glut menu handlers
int winWidth = 1400, winHeight = 800;
int between = 35;
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

string rootFilePath = "E:/tooth_data/part1/5/"; // 2/1-4;
string pcaDataFile = "E:/tooth_data/parameters/pca.txt";
size_t currentFileIdx = 24; 

string filePath = "E:\\LAB\\Tooth\\Model\\";
string modelName = "LXB2L.obj";
string modelName2 = "LXB3L.obj";
string modelNameSimplify = "LXB2L150_test.obj";

Mesh tooth,tooth2,toothSimplify;
bool simplifyOnoff = false;
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
void LoadMesh(Mesh & mesh, string & modelName);
void LoadNewModel();
void CreateColorList();

// window related 
void MenuCallback(int value);
void ReshapeFunc(int width, int height);

// rendering functions
void DisplayFunc();
void DrawFlatShaded(Mesh & mesh, int groupID = -1);

// input related glut functions
void KeyboardFunc(unsigned char ch, int x, int y);
void SpecialKeyFcn(GLint specialKey, GLint xMouse, GLint yMouse);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void MouseWheelFunc(int button, int dir, int x, int y);

void CreateDir(string dir);
void ReadFiles(string rootpath, vector<string>& pathList);

int ToothNumber = 31;//31
int ModelNumber = 1584424; //1591804
string ModelRoot = "F:\\Tooth\\OriginalModel\\";
string LabelRoot = "F:\\Tooth\\STAGE2-180721\\Label\\";
string FeatureRoot = "F:\\Tooth\\STAGE2-180721\\Feature\\";
string SingleToothRoot = "F:\\Tooth\\STAGE2-180721\\SingleTooth\\";
bool searchFilesByWildcard(string Path, vector<string> & files);
void BuildLabelFromTXT(Mesh& mesh, string txtPath);

int main(int argc, char *argv[]) {

	if (argc == 3) {
		ToothNumber = atoi(argv[1]);
		ModelNumber = atoi(argv[2]);
	}

	stringstream ss;
	ss << ModelNumber;
	string ModelName = ss.str();
	string LorU = ToothNumber > 30 ? "L" : "U";
	cout << "ModelName :" << ModelName << endl;
	
	string searchPath = ModelRoot + ModelName + "\\2\\*" + LorU + "*.stl";
	cout << "searchPath:" << searchPath << endl;

	vector<string> modelFiles;
	searchFilesByWildcard(searchPath, modelFiles);

	stringstream sss;
	sss << ToothNumber;
	string featureTxt = FeatureRoot + sss.str() + "\\" + ss.str() + LorU;

	if (modelFiles.size() == 1) {
		Mesh tooth;
		LoadMesh(tooth, ModelRoot + ModelName + "\\2\\" +modelFiles[0]);
		BuildLabelFromTXT(tooth, LabelRoot + ss.str() + LorU + ".txt");
		//把要算的单个牙齿切出来
		CreateDir(SingleToothRoot + sss.str());
		vector<int> labelForTooth;
		tooth.SaveOBJWithLabel(ToothNumber, labelForTooth, SingleToothRoot+sss.str() + "\\" + ss.str() + LorU + ".obj");

		FeatureExtractor fExtractor;
		fExtractor.extractFeature(SingleToothRoot + sss.str() + "\\" + ss.str() + LorU + ".obj", labelForTooth);
		CreateDir(featureTxt);
		fExtractor.saveFeature(featureTxt);
	}

/*	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	string ori = "F:\\Tooth\\FinishOriModel\\STL5\\1625149\\2\\l2.stl";
	string tar = "F:\\Tooth\\FinishOriModel\\STL5\\1625149\\3\\l3.stl";
	string sim = "F:\\Tooth\\15SimM2Model\\STL5\\1625149\\l2.obj";
	string txt = "F:\\Tooth\\Model\\180515-15SimM2Feature\\004_1625149L.txt";
	
	LoadMesh(tooth, ori);
	LoadMesh(tooth2, tar);
	LoadMesh(toothSimplify, sim);
	SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
	BuildLabelForBubbleNoise(tooth, tooth2);
	mapLabelForBubbleNoise(tooth, toothSimplify);
	//remapLabelForBubbleNoise(toothSimplify, tooth);
	BuildLabelFromLearning(toothSimplify, txt);
	glutMainLoop();*/
	//system("pause");
	return 0;
}

bool searchFilesByWildcard(string Path, vector<string> & files) {
	_finddata_t fileInfo;
	long handle = _findfirst(Path.c_str(), &fileInfo);
	if (handle == -1L) {
		cout << "Failed to Find files." << endl;
		return false;
	}
	do {
		cout << fileInfo.name << endl;
		files.push_back(fileInfo.name);
	} while (_findnext(handle, &fileInfo) == 0);

	return true;
}

//通过txt记录的面片标签设置模型Label
void BuildLabelFromTXT(Mesh& mesh, string txtPath) {
	ifstream txt;
	txt.open(txtPath, ifstream::in);
	cout << txtPath << endl;
	int i = 0;
	while (!txt.eof()) {
		string s;
		getline(txt, s);
		istringstream is(s);
		string faceNum, label;
		is >> faceNum >> label;
		mesh.fList[atoi(faceNum.c_str())]->SetLabel(atoi(label.c_str()));
		i += 1;
		if (i % 1000 == 0)
			cout << ".";
	}
	cout << endl;
	txt.close();
}

void CreateDir(string dir)
{
	int m = 0, n;
	string str1, str2;

	str1 = dir;
	str2 = str1.substr(0, 2);
	str1 = str1.substr(3, str1.size());

	while (m >= 0)
	{
		m = str1.find('\\');

		str2 += '\\' + str1.substr(0, m);
		n = _access(str2.c_str(), 0); //判断该目录是否存在
		if (n == -1)
		{
			_mkdir(str2.c_str());     //创建目录
		}
		str1 = str1.substr(m + 1, str1.size());
	}
}

void ReadFiles(string rootpath, vector<string>& pathList)
{
	struct _finddata_t fa;
	long long fHandle;
	string ss = rootpath + "*";
	if ((fHandle = _findfirst(ss.c_str(), &fa)) == -1L)
	{
		printf("ReadFiles : There is no file\n");
	}
	else
	{
		do
		{
			string ss(fa.name);
			if (ss != "." && ss != "..")
			{
				pathList.push_back(ss);
				//cout << ss << endl;
			}
		} while (_findnext(fHandle, &fa) == 0);
		_findclose(fHandle);
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
	glPushMatrix();
	glTranslated(winWidth/ between, 0, 0);
	glTranslated(xtrans, ytrans, -sdepth);
	glMultMatrixf(rotation);
	glTranslated(-g_center[0], -g_center[1], -g_center[2]);
	//right
	switch (displayMode)
	{
	case FLATSHADED:
		DrawFlatShaded(tooth, tooth.maxGroupID);
		break;
	default:
		break;
	}
	glPopMatrix();

	glPushMatrix();
	glTranslated(-winWidth / between, 0, 0);
	glTranslated(xtrans, ytrans, -sdepth);
	glMultMatrixf(rotation);
	glTranslated(-g_center[0], -g_center[1], -g_center[2]);
	//left
	switch (displayMode)
	{
	case FLATSHADED:
		DrawFlatShaded(toothSimplify, toothSimplify.maxGroupID);
		break;
	default:
		break;
	}
	glPopMatrix();

	glutSwapBuffers();
}

// init right-click menu
void InitMenu()
{
	displayMenu = glutCreateMenu(MenuCallback);
	glutAddMenuEntry("Flat Shaded", FLATSHADED);


	mainMenu = glutCreateMenu(MenuCallback);
	glutAddSubMenu("Display", displayMenu);
	glutAddMenuEntry("Load New Model", 100);
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
		LoadMesh(tooth, filePath + modelName);
		SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
	}
}

// Load mesh file
void LoadMesh(Mesh & mesh, string & modelName)
{
	cout << "LoadMesh : " << modelName << endl;
	string modelExtensions = modelName.substr(modelName.length() - 3, modelName.length());

	if (!mesh.LoadModel(modelName.c_str()))
		return;

	mesh.DisplayMeshInfo();
	mesh.FindMaxConnectedComponet();

	xtrans = ytrans = 0.0;
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

void DrawFlatShaded(Mesh & mesh, int groupID)
{
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 0.4f);
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
			/*
			if (f->bubbleNoiseLabel == 1 && f->bubbleNoiseLabel2 == 1 && f->bubbleNoiseLabelResult == 0)
				glColor3d(1.0f, 1.0f, 0.1f);
			else if(f->bubbleNoiseLabel == 1  && f->bubbleNoiseLabel2 == 0 && f->bubbleNoiseLabelResult == 0)
				glColor3d(1.0f, 0.1f, 0.1f);
			else if (f->bubbleNoiseLabel == 0 && f->bubbleNoiseLabel2 == 1 && f->bubbleNoiseLabelResult == 0)
				glColor3d(0.1f, 1.0f, 0.1f);
			else if (f->bubbleNoiseLabel == 0 && f->bubbleNoiseLabel2 == 0 && f->bubbleNoiseLabelResult == 1)
				glColor3d(0.1f, 0.1f, 1.0f);
			else if (f->bubbleNoiseLabel == 1 && f->bubbleNoiseLabel2 == 0 && f->bubbleNoiseLabelResult == 1)
				glColor3d(1.0f, 0.1f, 1.0f);
			else 
				glColor3d(0.4f, 0.4f, 0.4f);*/
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
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



