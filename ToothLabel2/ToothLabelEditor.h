#pragma once
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

public:
	int pickedLabel = 0;//��ǰ�޸ĵ�label����
	vector<int> pickedIDList;//ѡ�е���Ƭ�б�
};

