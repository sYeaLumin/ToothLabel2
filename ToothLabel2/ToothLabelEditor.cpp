#include "ToothLabelEditor.h"

struct POS {
	int x, y;
};

ToothLabelEditor::ToothLabelEditor()
{
	LColors[0] = 0; //����
	//����
	LColors[11] = 9; LColors[12] = 8; LColors[13] = 7; LColors[14] = 6;
	LColors[15] = 5; LColors[16] = 4; LColors[17] = 3; LColors[18] = 2;
	//����
	LColors[21] = 10; LColors[22] = 11; LColors[23] = 12; LColors[24] = 13;
	LColors[25] = 14; LColors[26] = 15; LColors[27] = 16; LColors[28] = 17;
	//����
	LColors[41] = 9; LColors[42] = 8; LColors[43] = 7; LColors[44] = 6;
	LColors[45] = 5; LColors[46] = 4; LColors[47] = 3; LColors[48] = 2;
	//����
	LColors[31] = 10; LColors[32] = 11; LColors[33] = 12; LColors[34] = 13;
	LColors[35] = 14; LColors[36] = 15; LColors[37] = 16; LColors[38] = 17;

	for (int i = 0; i < 64; i++)
		csvRecord[i] = 0;
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

bool ifNeighboor(Face* f, Face* fnext) {
	Face *f1, *f2, *f3;
	f1 = f->HalfEdge()->Twin()->LeftFace();
	f2 = f->HalfEdge()->Prev()->Twin()->LeftFace();
	f3 = f->HalfEdge()->Next()->Twin()->LeftFace();
	if (f1 != fnext && f2 != fnext && f3 != fnext)
		return false;
	else return true;
}

void ToothLabelEditor::paintLabels(Mesh & mesh, vector<int>& pos)
{
	int viewport[4];
	unsigned char data[4];
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGetIntegerv(GL_VIEWPORT, viewport);

	vector<Face *> ring;
	for (int i = 0; i < pos.size() / 2; i++) {
		glReadPixels(pos[2 * i], viewport[3] - pos[2 * i + 1], 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, data);
		int pickedID = data[0] + data[1] * 256 + data[2] * 65536;
		if (pickedID == 0x00ffffff)
			pickedID = -1;
		if (pickedID >= mesh.fList.size() || pickedID < 0)
			return;
		if (ring.size()==0 || mesh.fList[pickedID] != ring[ring.size() - 1]) {
			ring.push_back(mesh.fList[pickedID]);
		}
	}

	for (int i = 0; i < ring.size(); i++) 
		ring[i]->SetFaceLabel(pickedLabel);
}

void ToothLabelEditor::setBubbleLabel(Mesh & mesh, int pickedID)
{
	if (pickedID >= mesh.fList.size() || pickedID < 0)
		return;
	setAreaLabel(mesh.fList[pickedID], bubbleLabel + pickedLabel, blankLabel);
}

void ToothLabelEditor::recordLabel(Mesh & mesh, string labelTXTPath)
{
	ofstream labelTXT(labelTXTPath);
	if (labelTXT.is_open()) {
		cout << "Recording..." << endl;
		for (int i = 0; i < mesh.fList.size(); i++) {
			int L = mesh.fList[i]->faceLabel;
			labelTXT << i << " " << L << endl;
			if (L == 0)
				continue;
			int tmp = L;
			if (L > 100)
				tmp = ((((L - 100) / 10) - 1) * 8 + ((L - 100) % 10)) * 2 - 1;
			else
				tmp = (((L / 10) - 1) * 8 + (L % 10)) * 2 - 2;
			csvRecord[tmp]++;
		}
	}
	labelTXT.close();

	ofstream labelCSV(csvRecordPath, ios::app);
	int p1 = labelTXTPath.find_last_of("\\");
	int p2 = labelTXTPath.find_last_of(".");
	labelCSV << labelTXTPath.substr(p1 + 1, p2- p1-1).c_str() << ",";
	for (int i = 0; i < 64; i++){
		if ((i + 1) % 16 == 0)
			labelCSV << csvRecord[i] << "," << " " << ",";
		else
			labelCSV << csvRecord[i] << ",";
	}
	labelCSV << endl;
	labelCSV.close();

	cout << "Finished !" << endl;
}

int ToothLabelEditor::getLColor(int ID)
{
	if (ID < 100)
		return LColors[ID];
	else if (ID == spacialLabel)
		return 0;
	else
		return 1;
}

void ToothLabelEditor::setCSV(string path)
{
	csvRecordPath = path;
}

void ToothLabelEditor::cleanRecord()
{
	for (int i = 0; i < 64; i++)
		csvRecord[i] = 0;
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

void ToothLabelEditor::setRingLabel(Face * f, int labelRing)
{
	if (f->faceLabel == labelRing)
		return;
	f->SetFaceLabel(labelRing);

	Face *f1, *f2, *f3;
	f1 = f->HalfEdge()->Twin()->LeftFace();
	f2 = f->HalfEdge()->Prev()->Twin()->LeftFace();
	f3 = f->HalfEdge()->Next()->Twin()->LeftFace();
	if (f1->faceLabel == labelRing &&
		f2->faceLabel == labelRing &&
		f3->faceLabel == labelRing)
		return;

	if (f1->faceLabel != labelRing)
		setRingLabel(f1, labelRing);
	if (f2->faceLabel != labelRing)
		setRingLabel(f2, labelRing);
	if (f3->faceLabel != labelRing)
		setRingLabel(f3, labelRing);
}
