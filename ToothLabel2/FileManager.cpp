#include "FileManager.h"

OpenModelFile::OpenModelFile()
{
	szBuffer[0] = 0;
	titleBuffer[0] = 0;
}


OpenModelFile::~OpenModelFile(){}


BOOL OpenModelFile::OpenNewModel(HWND hwnd, string & fullPathName, string & modelName)
{
	szBuffer[0] = 0;
	titleBuffer[0] = 0;
	ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);

	ofn.hwndOwner = hwnd;
	ofn.lpstrFilter = _T("Model files (*.stl;*.obj)\0*.stl;*.obj\0\0");//要选择的文件后缀   
	ofn.lpstrInitialDir = _T("D:/project/mesh_feature/tooth/Segmentation_Part1/5/");//默认的文件路径   
	ofn.lpstrFile = szBuffer;//存放文件的缓冲区  
	ofn.lpstrFileTitle = titleBuffer;
	ofn.nMaxFile = sizeof(szBuffer) / sizeof(*szBuffer);
	ofn.nMaxFileTitle = sizeof(titleBuffer) / sizeof(*titleBuffer);
	ofn.nFilterIndex = 0;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;//标志如果是多选要加上OFN_ALLOWMULTISELECT  
	ofn.Flags |= OFN_NOCHANGEDIR;//不改变根目录
	BOOL bSel = GetOpenFileName(&ofn);

	if (bSel)
	{/*
		USES_CONVERSION;
		string  strFullPath(W2A(ofn.lpstrFile));
		string  strTitle(W2A(ofn.lpstrFileTitle));
		fullPathName = strFullPath;
		modelName = strTitle;*/
	}
	return bSel;
}