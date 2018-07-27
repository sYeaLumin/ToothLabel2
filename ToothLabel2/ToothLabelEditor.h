#pragma once
#include <map>
#include <vector>
#include <fstream>
#include <GL\freeglut.h>
#include "Mesh\mesh.h"

using namespace std;
using namespace BiMesh;

class ToothLabelEditor
{
public:
	ToothLabelEditor();
	~ToothLabelEditor();
	void pickLabel(Mesh & mesh, int pickedID);
	void setLabel(Mesh & mesh, int pickedID);
	void setLabels(Mesh & mesh, int pickedID);
	void paintLabels(Mesh & mesh, vector<int>& pos);
	void setBubbleLabel(Mesh & mesh, int pickedID);
	void recordLabel(Mesh & mesh, string labelTXT);
	int getLColor(int ID);
	void setCSV(string path);
	void cleanRecord();

private:
	void setAreaLabel(Face *f, int label1, int label2);

public:	
	int bubbleLabel = 100;//���ݵ�label
	int blankLabel = 0;//�հ�label

private:
	int pickedLabel = 0;//��ǰ�޸ĵ�label����
	int csvRecord[64];
	map<int, int> LColors;
	string csvRecordPath;
};

