#include "ToothLabelEditor.h"



ToothLabelEditor::ToothLabelEditor()
{
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
