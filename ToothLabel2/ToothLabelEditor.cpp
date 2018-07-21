#include "ToothLabelEditor.h"



ToothLabelEditor::ToothLabelEditor()
{
	LColors[0] = 0; //牙龈
	//右上
	LColors[11] = 9; LColors[12] = 8; LColors[13] = 7; LColors[14] = 6;
	LColors[15] = 5; LColors[16] = 4; LColors[17] = 3; LColors[18] = 2;
	//左上
	LColors[21] = 10; LColors[22] = 11; LColors[23] = 12; LColors[24] = 13;
	LColors[25] = 14; LColors[26] = 15; LColors[27] = 16; LColors[28] = 17;
	//右下
	LColors[41] = 9; LColors[42] = 8; LColors[43] = 7; LColors[44] = 6;
	LColors[45] = 5; LColors[46] = 4; LColors[47] = 3; LColors[48] = 2;
	//左下
	LColors[31] = 10; LColors[32] = 11; LColors[33] = 12; LColors[34] = 13;
	LColors[35] = 14; LColors[36] = 15; LColors[37] = 16; LColors[38] = 17;
}


ToothLabelEditor::~ToothLabelEditor()
{
}

void ToothLabelEditor::pickLabel(Mesh & mesh, int pickedID)
{
	if (pickedID >= mesh.fList.size() || pickedID < 0)
		return;
	pickedLabel = mesh.fList[pickedID]->FaceLabel();
	cout << "PickedLabel:" << pickedLabel << endl;
}

void ToothLabelEditor::setLabel(Mesh & mesh, int pickedID)
{
	if (pickedID >= mesh.fList.size() || pickedID < 0)
		return;
	mesh.fList[pickedID]->SetFaceLabel(pickedLabel);
}

void ToothLabelEditor::setLabels(Mesh & mesh, int pickedID)
{
	if (pickedID >= mesh.fList.size() || pickedID < 0)
		return;
	if (pickedLabel == mesh.fList[pickedID]->FaceLabel())
		return;
	setAreaLabel(mesh.fList[pickedID], pickedLabel, mesh.fList[pickedID]->FaceLabel());
}

void ToothLabelEditor::setBubbleLabel(Mesh & mesh, int pickedID)
{
	if (pickedID >= mesh.fList.size() || pickedID < 0)
		return;
	setAreaLabel(mesh.fList[pickedID], bubbleLabel + pickedLabel, blankLabel);
}

int ToothLabelEditor::getLColor(int ID)
{
	if (ID < 100)
		return LColors[ID];
	else
		return 1;
}

void ToothLabelEditor::setAreaLabel(Face *f, int label1, int label2)
{
	if (f->faceLabel != label2)
		return;
	f->SetFaceLabel(label1);

	Face *f1, *f2, *f3;
	f1 = f->HalfEdge()->Twin()->LeftFace();
	f2 = f->HalfEdge()->Prev()->Twin()->LeftFace();
	f3 = f->HalfEdge()->Next()->Twin()->LeftFace();
	if (f1->faceLabel != label2 &&
		f2->faceLabel != label2 &&
		f3->faceLabel != label2)
		return;

	if (f1->faceLabel == label2)
		setAreaLabel(f1, label1, label2);
	if (f2->faceLabel == label2)
		setAreaLabel(f2, label1, label2);
	if (f3->faceLabel == label2)
		setAreaLabel(f3, label1, label2);
}
