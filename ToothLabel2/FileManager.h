// FileManager.h
#ifndef __FIlE_MANAGER_H__
#define __FIlE_MANAGER_H__

#include <iostream>
#include <windows.h>
#include "Tchar.h"
#include <atlconv.h>
#include <commdlg.h>

using namespace std;

class OpenModelFile
{
public:
	BOOL OpenNewModel(HWND hwnd, string & fullPathName, string & modelName);
	OpenModelFile();
	~OpenModelFile();
private:
	OPENFILENAME ofn;
	TCHAR szBuffer[MAX_PATH];
	TCHAR titleBuffer[MAX_PATH];
};



// END OF FILE MANAGER
#endif
