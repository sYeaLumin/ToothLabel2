#pragma once
#include <map>
#include <vector>
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
	void setBubbleLabel(Mesh & mesh, int pickedID);
	int getLColor(int ID);

private:
	void setBubbleLabels(Face *f);

public:	
	int bubbleLabelNum = 1;//气泡的label
	int blankLabel = 0;//空白label

private:
	int pickedLabel = 0;//当前修改的label类型
	map<int, int> LColors;
	//vector<int> pickedIDList;//选中的面片列表
};

