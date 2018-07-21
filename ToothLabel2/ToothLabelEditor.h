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
	void setLabels(Mesh & mesh, int pickedID);
	void setBubbleLabel(Mesh & mesh, int pickedID);
	int getLColor(int ID);

private:
	void setAreaLabel(Face *f, int label1, int label2);

public:	
	int bubbleLabel = 100;//���ݵ�label
	int blankLabel = 0;//�հ�label

private:
	int pickedLabel = 0;//��ǰ�޸ĵ�label����
	map<int, int> LColors;
	//vector<int> pickedIDList;//ѡ�е���Ƭ�б�
};

