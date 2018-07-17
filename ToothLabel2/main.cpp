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
void DisplayFunc2();
void DrawFlatShaded(Mesh & mesh, int groupID = -1);
void DrawFlatShaded2(Mesh & mesh, int groupID = -1);

// input related glut functions
void KeyboardFunc(unsigned char ch, int x, int y);
void SpecialKeyFcn(GLint specialKey, GLint xMouse, GLint yMouse);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void MouseWheelFunc(int button, int dir, int x, int y);

//标记气泡
void BuildLabelForBubbleNoise(Mesh & meshWith, Mesh & meshWithOut);
void mapLabelForBubbleNoise(Mesh & mesh, Mesh & meshSiplify);
void remapLabelForBubbleNoise(Mesh & meshSiplify, Mesh & mesh);
void BuildLabelFromLearning(Mesh& mesh, string txtPath);

//模型ANN映射参数
double threshold = 0.1; //0.15
int thresholdForMap = 7; //3
int KForMap = 10; //5

string rootPath = "F:\\Tooth\\";
string oriModelPath = "180515-15OriModel\\";//"OriginalModel\\";//
string simModelPath = "180515-15SimM2ModelNSFB\\";// "15SimM2Model\\";//
string featurePath = "180515-15SimM2FeatureNSFB\\";//  "15SimM2Feature\\";//
vector<string> modelPathList;
void CreateDir(string dir);
void ReadFiles(string rootpath, vector<string>& pathList);

//简化模型参数
double simplifyRatio = 0.15;
//MeshSimplify ms;
SimplifyParameters sp;

//特征提取
//FeatureExtractor fExtractor;

//面片数记录txt
string recordForFaceLabel = "F:\\Tooth\\recordFor15SimM2FL.txt";

//不同label面片颜色
float faceLabelColors[16][3] = {
	{ 0.50f, 0.50f, 0.50f },
	{ 0.95f, 0.93f, 0.91f },
	{ 0.86f, 0.09f, 0.03f },
	{ 0.93f, 0.33f, 0.04f },
	{ 0.95f, 0.61f, 0.10f },
	{ 0.95f, 0.91f, 0.32f },
	{ 0.69f, 0.82f, 0.27f },
	{ 0.31f, 0.61f, 0.20f },
	{ 0.04f, 0.38f, 0.05f },
	{ 0.47f, 0.70f, 0.85f },
	{ 0.23f, 0.55f, 0.81f },
	{ 0.18f, 0.36f, 0.75f },
	{ 0.39f, 0.09f, 0.73f },
	{ 0.76f, 0.42f, 0.91f },
	{ 0.88f, 0.37f, 0.52f },
	{ 1.0f, 0.77f, 0.82f },
};
Mesh toothMesh2, toothMeshSeparated;//有气泡的模型2、分割过的没气泡的模型
ANNkd_tree* buildANNTreeForMesh(Mesh& mesh);

int main(int argc, char *argv[]) {
	/*	sp.d_ratio = simplifyRatio;

	string stl = "STL5\\";
	string root = rootPath + oriModelPath + stl;
	ReadFiles(root, modelPathList);
	int breakNumver = 0;
	
	ofstream out_record_txt;
	out_record_txt.open(recordForFaceLabel.c_str(),ios::ate);

	for (size_t i = 23; i < modelPathList.size(); i++) {
		if (breakNumver > 1)
			break;
		else breakNumver++;

		string model = root + modelPathList[i];
		cout << model << endl;
		vector<string> nameList2,nameList3;
		ReadFiles(model + "\\2\\", nameList2);
		ReadFiles(model + "\\3\\", nameList3);

		//ofstream out_record_txt(recordForFaceLabel.c_str());
		out_record_txt << modelPathList[i] << "\t  ";

		for (size_t j = 0; j < nameList2.size(); j++) {
			string oriM2P = model + "\\2\\" + nameList2[j];
			string oriM3P = model + "\\3\\" + nameList3[j];
			string name = nameList2[j].substr(0, nameList2[j].find_last_of("."));
			string simM2P = rootPath + simModelPath + stl + modelPathList[i] + "\\";//+ name +".obj";
			string featureM2P = rootPath + featurePath + stl + modelPathList[i] + "\\" + name;// +".txt";
			CreateDir(simM2P);
			CreateDir(featureM2P);
			//简化
			Mesh OM2, OM3, SM2;
			LoadMesh(OM2, oriM2P);
			LoadMesh(OM3, oriM3P);
			BuildLabelForBubbleNoise(OM2, OM3);
			MeshSimplify ms;
			ms.Simplify(OM2, simM2P + name + ".obj", sp);
			LoadMesh(SM2, simM2P + name + ".obj");
			//映射label
			//BuildLabelForBubbleNoise(OM2, OM3);
			mapLabelForBubbleNoise(OM2, SM2);
			int* labelForTooth = new int[SM2.fList.size()];
			int numForLabel1 = 0;
			for (size_t t = 0; t < SM2.fList.size(); t++) {
				labelForTooth[t] = SM2.fList[t]->bubbleNoiseLabel;
				if (SM2.fList[t]->bubbleNoiseLabel == 1)
					numForLabel1++;
			}			
			out_record_txt << SM2.fList.size() << "\t  " << numForLabel1 << "\t  ";
			//特征提取		
			FeatureExtractor fExtractor;
			fExtractor.extractFeature(simM2P + name + ".obj");
			fExtractor.saveFeature(featureM2P, labelForTooth);
			delete[] labelForTooth;
		}
		out_record_txt << endl;
	}
		out_record_txt.close();*/

	/*
	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	string ori = "F:\\Tooth\\180515-15OriModel\\STL5\\1625037\\2\\l2.stl";
	string tar = "F:\\Tooth\\180515-15OriModel\\STL5\\1625037\\3\\l3.stl";
	string sim = "F:\\Tooth\\180515-15SimM2ModelNSFB\\STL5\\1625037\\l2.obj";
	string txt = "F:\\Tooth\\Model\\180515-15SimM2FeatureNSFB\\001_1625037L.txt";
	
	LoadMesh(tooth, ori);
	LoadMesh(tooth2, tar);
	LoadMesh(toothSimplify, sim);
	SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
	BuildLabelForBubbleNoise(tooth, tooth2);
	mapLabelForBubbleNoise(tooth, toothSimplify);
	//remapLabelForBubbleNoise(toothSimplify, tooth);
	BuildLabelFromLearning(toothSimplify, txt);
	glutMainLoop();
	system("pause");
	return 0;*/

	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	string mesh2 = "F:\\Tooth\\\OriginalModel\\STL5\\1635634\\2\\李雪玉_2018-03-08_C01001635634_U.stl";
	string meshSeparated = "F:\\Tooth\\SeparatedModel\\1635634\\U\\";
	vector<string> meshSepList;
	ReadFiles(meshSeparated, meshSepList);
	LoadMesh(toothMesh2, mesh2);
	SetBoundaryBox(toothMesh2.MinCoord(), toothMesh2.MaxCoord());
	ANNkd_tree* mesh2Tree = buildANNTreeForMesh(toothMesh2);//为模型2建搜索树
	for (int toothID = 0; toothID < meshSepList.size(); toothID++) {
		LoadMesh(toothMeshSeparated, meshSeparated+ meshSepList[toothID]);
		int  nearNum = 3;
		int dim = 3;
		double	eps = 0.1;		// error bound
		ANNpoint			queryPt;				// query point
		ANNidxArray			nnIdx;					// near neighbor indices
		ANNdistArray		dists;					// near neighbor distances
		queryPt = annAllocPt(dim);					// allocate query point
		nnIdx = new ANNidx[nearNum];						// allocate near neigh indices
		dists = new ANNdist[nearNum];						// allocate near neighbor dists

		for (size_t i = 0; i < toothMeshSeparated.fList.size(); i++)
		{
			Face * f = toothMeshSeparated.fList[i];
			if (f->GroupID() != toothMeshSeparated.maxGroupID) continue;
			for (int j = 0; j < dim; j++)
				queryPt[j] = f->center[j];

			mesh2Tree->annkSearch(					// search
				queryPt,						// query point
				nearNum,								// number of near neighbors
				nnIdx,							// nearest neighbors (returned)
				dists,							// distance (returned)
				eps);							// error bound

			
			toothMesh2.fList[nnIdx[0]]->faceLabel = toothID + 2;
			
			float tmp = 5.0;
			if(dists[1]/dists[0] < tmp)
				toothMesh2.fList[nnIdx[1]]->faceLabel = toothID + 2;
			/*if (dists[2] / dists[0] < tmp)
				toothMesh2.fList[nnIdx[2]]->faceLabel = toothID + 2;*/
		}
		delete[] nnIdx;
		delete[] dists;
	}
	delete mesh2Tree;//删除搜索树
	/**/
	for (int i = 0; i < toothMesh2.fList.size(); i++) {
		Face * f = toothMesh2.fList[i];
		Face *f1, *f2, *f3;
		f1 = f->HalfEdge()->Twin()->LeftFace();
		f2 = f->HalfEdge()->Prev()->Twin()->LeftFace();
		f3 = f->HalfEdge()->Next()->Twin()->LeftFace();
		if (f1->faceLabel == f2->faceLabel && f2->faceLabel == f3->faceLabel)
			f->faceLabel = f1->faceLabel;
	}
	glutMainLoop();
	return 0;
}

ANNkd_tree* buildANNTreeForMesh(Mesh& mesh) {
	int dim = 3;			// dimension
	double	eps = 0.1;		// error bound
	int maxPts = 1000000;  // maximum number of data points
	int					nPts = (int)mesh.fList.size();					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNkd_tree*			kdTree;					// search structure
	dataPts = annAllocPts(maxPts, dim);			// allocate data points

	for (size_t i = 0; i < mesh.fList.size(); i++)
	{
		for (int j = 0; j < dim; j++)
		{
			dataPts[i][j] = mesh.fList[i]->center[j];
		}
	}

	// set up kd-tree
	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	return kdTree;
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

void BuildLabelFromLearning(Mesh& mesh, string txtPath) {
	ifstream txt;
	txt.open(txtPath, ifstream::in);
	int i = 0;
	while (!txt.eof()) {
		string s;
		getline(txt, s);
		istringstream is(s);
		string faceNum, label;
		is >> faceNum >> label;
		mesh.fList[atoi(faceNum.c_str())]->bubbleNoiseLabelResult = atoi(label.c_str());
		i += 1;
		if (i % 1000 == 0)
			cout << ".";
	}
	cout << endl;
	txt.close();
}

void remapLabelForBubbleNoise(Mesh & meshSiplify, Mesh & mesh) {
	cout << "remapLabelForBubbleNoise..." << endl;

	int	k = 1;				// number of nearest neighbors
	int dim = 3;			// dimension
	double	eps = 0.1;		// error bound
	int maxPts = 1000000;  // maximum number of data points
	int					nPts = (int)meshSiplify.fList.size();					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists

												// set up data
	for (size_t i = 0; i < meshSiplify.fList.size(); i++)
	{
		for (int j = 0; j < dim; j++)
		{
			dataPts[i][j] = meshSiplify.fList[i]->center[j];
		}
	}

	// set up kd-tree
	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

									// query
									//meshWith.FindMaxConnectedComponet();
									//cout << src.maxGroupID << endl;
	for (size_t i = 0; i < mesh.fList.size(); i++)
	{
		Face * f = mesh.fList[i];
		if (f->GroupID() != mesh.maxGroupID) continue;
		for (int j = 0; j < dim; j++)
		{
			queryPt[j] = f->center[j];
		}

		kdTree->annkSearch(					// search
			queryPt,						// query point
			k,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		if (meshSiplify.fList[nnIdx[0]]->bubbleNoiseLabel == 1)
			f->bubbleNoiseLabel2 = 1;
	}
	cout << endl;
	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();
}

void mapLabelForBubbleNoise(Mesh & mesh, Mesh & meshSiplify) {
	cout << "mapLabelForBubbleNoise..." << endl;
	int dim = 3;			// dimension
	double	eps = 0.1;		// error bound
	int maxPts = 1000000;  // maximum number of data points
	int					nPts = (int)mesh.fList.size();					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[KForMap];						// allocate near neigh indices
	dists = new ANNdist[KForMap];						// allocate near neighbor dists

												// set up data
	for (size_t i = 0; i < mesh.fList.size(); i++)
	{
		for (int j = 0; j < dim; j++)
		{
			dataPts[i][j] = mesh.fList[i]->center[j];
		}
	}

	// set up kd-tree
	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

									// query
									//meshWith.FindMaxConnectedComponet();
									//cout << src.maxGroupID << endl;
	for (size_t i = 0; i < meshSiplify.fList.size(); i++)
	{
		Face * f = meshSiplify.fList[i];
		if (f->GroupID() != meshSiplify.maxGroupID) continue;
		for (int j = 0; j < dim; j++)
		{
			queryPt[j] = f->center[j];
		}

		kdTree->annkSearch(					// search
			queryPt,						// query point
			KForMap,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		double tmp = 0;
		for (int t = 0; t < KForMap; t++)
			tmp += mesh.fList[nnIdx[t]]->bubbleNoiseLabel;
		if (tmp > thresholdForMap) {
			meshSiplify.fList[i]->bubbleNoiseLabel = 1;
			//cout << "MapLabel:" << i << ":" << tmp  << endl;
		}
	}
	cout << endl;
	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();
}

//标记气泡
void BuildLabelForBubbleNoise(Mesh & meshWith, Mesh & meshWithOut) {
	cout << "BuildLabelForBubbleNoise..." << endl;

	int	k = 1;				// number of nearest neighbors
	int dim = 3;			// dimension
	double	eps = 0.1;		// error bound
	int maxPts = 1000000;  // maximum number of data points
	int					nPts = (int)meshWithOut.fList.size();					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);			// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neigh indices
	dists = new ANNdist[k];						// allocate near neighbor dists

	// set up data
	for (size_t i = 0; i < meshWithOut.fList.size(); i++)
	{
		for (int j = 0; j < dim; j++)
		{
			dataPts[i][j] = meshWithOut.fList[i]->center[j];
		}
	}

	// set up kd-tree
	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// number of points
		dim);						// dimension of space

	// query
	//meshWith.FindMaxConnectedComponet();
	//cout << src.maxGroupID << endl;
	for (size_t i = 0; i < meshWith.fList.size(); i++)
	{
		Face * f = meshWith.fList[i];
		if (f->GroupID() != meshWith.maxGroupID) continue;
		for (int j = 0; j < dim; j++)
		{
			queryPt[j] = f->center[j];
		}

		kdTree->annkSearch(					// search
			queryPt,						// query point
			k,								// number of near neighbors
			nnIdx,							// nearest neighbors (returned)
			dists,							// distance (returned)
			eps);							// error bound

		f->simplifyMapping = nnIdx[0];
		//double threshold = 0.1; //0.4  0.15
		double distance = f->center.Distance(meshWithOut.fList[nnIdx[0]]->center);
		//cout << distance << " ";
		if (distance > threshold) {
			meshWith.fList[i]->bubbleNoiseLabel = 1;
			Face *f = meshWith.fList[i];
			f->HalfEdge()->Start()->simplify_level = 1;
			f->HalfEdge()->End()->simplify_level = 1;
			f->HalfEdge()->Next()->End()->simplify_level = 1;
			//cout << "BuildLabel:" << i << ":" << distance << " " << nnIdx[0] << endl;
		}	
		//meshWithOut.fList[nnIdx[0]]->center
	}
	cout << endl;
	delete[] nnIdx;							// clean things up
	delete[] dists;
	delete kdTree;
	annClose();
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
	glutDisplayFunc(DisplayFunc2);
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

void DisplayFunc2()
{
	float rotation[16];
	trackball.m_rotation.GetMatrix(rotation);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, winAspect, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glPushMatrix();
	glTranslated(xtrans, ytrans, -sdepth);
	glMultMatrixf(rotation);
	glTranslated(-g_center[0], -g_center[1], -g_center[2]);
	switch (displayMode)
	{
	case FLATSHADED:
		DrawFlatShaded2(toothMesh2, toothMesh2.maxGroupID);
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
				glColor3d(0.4f, 0.4f, 0.4f);
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawFlatShaded2(Mesh & mesh, int groupID)
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
			float* faceColors = faceLabelColors[f->faceLabel];
			glColor3d(faceColors[0], faceColors[1], faceColors[2]);//根据label设置颜色
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
		LoadMesh(tooth, modelPathList[currentFileIdx] + modelName);
		SetBoundaryBox(tooth.MinCoord(), tooth.MaxCoord());
		xtrans = ytrans = 0.0;
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



