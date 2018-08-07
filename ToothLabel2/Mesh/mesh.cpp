#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "mesh.h"
#include "matrix.h"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <string>
#include <cmath>
#include <list>
#include <queue>
#include <float.h>
using namespace std;

namespace BiMesh
{
inline double Cot(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) 
{
	Vector3d v1 = p1 - p2;
	Vector3d v2 = p3 - p2;

	v1 /= v1.L2Norm();
	v2 /= v2.L2Norm();
	double tmp = v1.Dot(v2);
	return 1.0 / tan(acos(tmp));
}

inline double Area(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) 
{
	Vector3d v1 = p2 - p1;
	Vector3d v2 = p3 - p1;
	return v1.Cross(v2).L2Norm() / 2.0;
}

OneRingHEdge::OneRingHEdge(const Vertex * v) 
{
	if (v == NULL) start = next = NULL;
	else start = next = v->HalfEdge();
}

HEdge * OneRingHEdge::NextHEdge() 
{
	HEdge *ret = next;
	if (next && next->Prev()->Twin() != start)
		next = next->Prev()->Twin();
	else
		next = NULL;
	return ret;
}

void Mesh::AddFace(int v1, int v2, int v3) 
{
	int i;
	HEdge *he[3], *bhe[3];
	Vertex *v[3];
	Face *f;

	// obtain objects
	for (i=0; i<3; i++) he[i] = new HEdge();
	for (i=0; i<3; i++) bhe[i] = new HEdge(true);
	v[0] = vList[v1];
	v[1] = vList[v2];
	v[2] = vList[v3];
	f = new Face();
	f->v[0] = v1;
	f->v[1] = v2;
	f->v[2] = v3;

	// connect prev-next pointers
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	SetPrevNext(bhe[0], bhe[1]);
	SetPrevNext(bhe[1], bhe[2]);
	SetPrevNext(bhe[2], bhe[0]);

	// connect twin pointers
	SetTwin(he[0], bhe[0]);
	SetTwin(he[1], bhe[2]);
	SetTwin(he[2], bhe[1]);

	// connect start pointers for bhe
	bhe[0]->SetStart(v[1]);
	bhe[1]->SetStart(v[0]);
	bhe[2]->SetStart(v[2]);
	for (i=0; i<3; i++) he[i]->SetStart(v[i]);

	// connect start pointers
	// connect face-hedge pointers
	for (i=0; i<3; i++) {
		v[i]->SetHalfEdge(he[i]);
		v[i]->adjHEdges.push_back(he[i]);
		SetFace(f, he[i]);
	}
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// mearge boundary if in need
	for (i=0; i<3; i++) {
		Vertex *start = bhe[i]->Start();
		Vertex *end   = bhe[i]->End();
		for (size_t j=0; j<end->adjHEdges.size(); j++) {
			HEdge *curr = end->adjHEdges[j];
			if (curr->IsBoundary() && curr->End()==start) {
				SetPrevNext(bhe[i]->Prev(), curr->Next());
				SetPrevNext(curr->Prev(), bhe[i]->Next());
				SetTwin(bhe[i]->Twin(), curr->Twin());
				bhe[i]->SetStart(NULL);	// mark as unused
				curr->SetStart(NULL);	// mark as unused
				break;
			}
		}
	}

	// finally add hedges and faces to list
	for (i=0; i<3; i++) heList.push_back(he[i]);
	for (i=0; i<3; i++) bheList.push_back(bhe[i]);

	// calculate center, normal and area
	Vector3d center(0, 0, 0);
	for (size_t i = 0; i < 3; i++)
	{
		center += vList[f->v[i]]->Position();
	}
	center /= 3;

	Vector3d e1 = vList[f->v[1]]->Position() - vList[f->v[0]]->Position();
	Vector3d e2 = vList[f->v[2]]->Position() - vList[f->v[0]]->Position();
	Vector3d normal = e1.Cross(e2);
	double area = 0.5*normal.L2Norm();
	
	if (area>1e-10)
		normal.Normalize();
	f->center = center;
	f->normal = normal;
	f->area = area;

	fList.push_back(f);
	/**/
}


bool Mesh::LoadModel(const char * filename)
{
	string modelName(filename);
	string modelExtensions = modelName.substr(modelName.length() - 3, modelName.length());
	if (strcmp(modelExtensions.c_str(), "obj") == 0)
	{
		if (LoadObjFile(modelName.c_str()))
			cout << "Load " << modelName << endl;
		else return false;
	}
	else if (strcmp(modelExtensions.c_str(), "stl") == 0)
	{
		if (LoadSTLFile(modelName.c_str()))
			cout << "Load " << modelName << endl;
		else return false;
	}
	else
	{
		cout << "Unknow File Format" << endl;
		return false;
	}
	return true;
}

// load a .obj mesh definition file
bool Mesh::LoadObjFile(const char *filename) 
{
	if (filename==NULL || strlen(filename)==0) 
		return false;
	
	ifstream ifs(filename);
	if (ifs.fail()) 
		return false;

	// clear current mesh object
	Clear();

	char buf[1024], type[1024];
	do 
	{
		ifs.getline(buf, 1024);
		istrstream iss(buf);
		iss >> type;

		// vertex
		if (strcmp(type, "v") == 0) 
		{
			double x, y, z;
			iss >> x >> y >> z;
            AddVertex(new Vertex(x,y,z));
		}
		
		// face
		else if (strcmp(type, "f") == 0) 
		{
			int index[3];
			iss >> index[0] >> index[1] >> index[2];
			AddFace(index[0]-1, index[1]-1, index[2]-1);
		}
	} while (!ifs.eof());
	ifs.close();

	
	size_t i;
	/*
	// normalize
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());
	

	Vector3d tot;
	for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);
	*/

	for (i = 0; i < fList.size(); i++)
		fList[i]->SetLabelPath(i);

	HEdgeList list;
	for (i=0; i<bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i=0; i<vList.size(); i++) 
	{
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex(i);
	}

	cout << endl;

	minCoord = MinCoord();
	maxCoord = MaxCoord();
	CalculateSphericalCoords();
	return true;
}

bool Mesh::WriteObjFile(const char * filename)
{
	string outputfile = filename;

	ofstream fout(outputfile);
	if (fout.fail())
		return false;

	fout << "# vetices " << vList.size() << endl;
	fout << "# faces " << fList.size() << endl;
	fout << "#################################" << endl;
	fout.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
	fout.precision(8);  // 设置精度 6 
	for (size_t i = 0; i < vList.size(); i++)
	{
		Vector3d p = vList[i]->Position();
		fout << "v " << p[0] << " " << p[1] << " " << p[2] << endl;
	}

	for (size_t i = 0; i < fList.size(); i++)
	{
		fout << "f " << fList[i]->v[0]+1 << " " << fList[i]->v[1]+1 << " " << fList[i]->v[2]+1 << endl;
	}

	fout.close();
	return true;
}

bool Mesh::LoadSTLFile(const char *cfilename)
{
	if (cfilename == NULL)
		return false;

	ifstream in(cfilename, ios::in);
	if (!in)
		return false;
	Clear();

	string headStr;
	getline(in, headStr, ' ');
	
	in.close();

	if (headStr.empty())
		return false;

	if (headStr[0] == 's')
	{
		cout << "ASCII File." << endl;
		ReadSTLASCII(cfilename);
	}
	else
	{
		cout << "Binary File." << endl;
		ReadSTLBinary(cfilename);
	}

	size_t i;

	HEdgeList list;
	for (i = 0; i<bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i = 0; i<vList.size(); i++)
	{
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex(i);
	}

	minCoord = MinCoord();
	maxCoord = MaxCoord();
	CalculateSphericalCoords();
	return true;
}

bool Mesh::ReadSTLASCII(const char *cfilename)
{
	vector<STLVertex> xyzList;

	int i = 0, j = 0, cnt = 0, pCnt = 4;
	char a[100];
	char str[100];
	float x = 0, y = 0, z = 0;

	int count = 0;
	ifstream in;
	in.open(cfilename, ios::in);
	if (!in)
		return false;
	do
	{
		i = 0;
		cnt = 0;
		in.getline(a, 100, '\n');
		while (a[i] != '\0')
		{
			if (!islower((int)a[i]) && !isupper((int)a[i]) && a[i] != ' ')
				break;
			cnt++;
			i++;
		}

		while (a[cnt] != '\0')
		{
			str[j] = a[cnt];
			cnt++;
			j++;
		}
		str[j] = '\0';
		j = 0;

		if (sscanf(str, "%lf%lf%lf", &x, &y, &z) == 3)
		{
			xyzList.push_back(STLVertex(x, y, z, count));
			count++;
		}
		pCnt++;
	} while (!in.eof());

	//  cout << "******  ACSII FILES　******" << endl;  
	//  for (int i = 0; i < coorX.size();i++)  
	//  {  
	//      cout << coorX[i] << " : " << coorY[i] << " : " << coorZ[i] << endl;  
	//  }  
	STL2OBJ(xyzList);
	//cout << xyzList.size() / 3 << " triangles." << endl;
	return true;
}

bool Mesh::ReadSTLBinary(const char *cfilename)
{
	vector<STLVertex> xyzList;

	char str[80];
	ifstream in;

	in.open(cfilename, ios::in | ios::binary);

	if (!in)
		return false;

	in.read(str, 80);

	//number of triangles  
	int unTriangles;
	in.read((char*)&unTriangles, sizeof(int));

	if (unTriangles == 0)
		return false;

	int count = 0;
	for (int i = 0; i < unTriangles; i++)
	{
		float coorXYZ[12];
		in.read((char*)coorXYZ, 12 * sizeof(float));

		for (int j = 1; j < 4; j++)
		{
			xyzList.push_back(STLVertex(coorXYZ[j * 3], coorXYZ[j * 3 + 1], coorXYZ[j * 3 + 2], count));
			count++;
		}

		in.read((char*)coorXYZ, 2);
	}

	in.close();

	STL2OBJ(xyzList);
	//cout << xyzList.size() / 3 << " triangles." << endl;
	return true;
}

bool STLVertexCmpXYZ(STLVertex & a, STLVertex & b)
{
	if (a.x < b.x) return true;
	else if (a.x > b.x) return false;
	else if (a.y < b.y) return true;
	else if (a.y > b.y) return false;
	else if (a.z < b.z) return true;
	else if (a.z > b.z) return false;
	else return a.stlID < b.stlID;
}

bool STLVertexCmpSTLID(STLVertex & a, STLVertex & b)
{
	return a.stlID < b.stlID;
}

void Mesh::STL2OBJ(vector<STLVertex> & xyzList)
{
	if (xyzList.size() == 0) return;

	sort(xyzList.begin(), xyzList.end(), STLVertexCmpXYZ);

	xyzList[0].objID = 0;
	AddVertex(new Vertex(xyzList[0].x, xyzList[0].y, xyzList[0].z));

	int objID = 0;
	for (size_t i = 1; i < xyzList.size(); i++)
	{
		if (xyzList[i].x == xyzList[i - 1].x && xyzList[i].y == xyzList[i - 1].y && xyzList[i].z == xyzList[i - 1].z)
			xyzList[i].objID = objID;
		else
		{
			xyzList[i].objID = ++objID;
			AddVertex(new Vertex(xyzList[i].x, xyzList[i].y, xyzList[i].z));
		}
	}

	sort(xyzList.begin(), xyzList.end(), STLVertexCmpSTLID);

	int zeroCount = 0;
	for (size_t i = 0, j = 0; i < xyzList.size(); i += 3, j++)
	{
		if (xyzList[i].objID == xyzList[i + 1].objID || xyzList[i].objID == xyzList[i + 2].objID || xyzList[i + 1].objID == xyzList[i + 2].objID)
			zeroCount++;
		else
		{
			AddFace(xyzList[i].objID, xyzList[i + 1].objID, xyzList[i + 2].objID);
			//fList[fList.size() - 1]->SetLabelPath(j);
		}
	}

	cout << "removed zero area face count:  " << zeroCount << endl;
}

bool Mesh::LoadLabels(const char * filename)
{
	if (filename == NULL || strlen(filename) == 0)
		return false;

	ifstream ifs(filename);
	if (ifs.fail())
		return false;

	vector<int> labels;

	char buf[1024];
	do
	{
		ifs.getline(buf, 1024);
		istrstream iss(buf);


		int x = -1, y;

		iss >> x >> y;
		labels.push_back(y);
	} while (!ifs.eof());

	ifs.close();

	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		if (f->LabelPath() < labels.size())
			f->SetLabel(labels[f->LabelPath()]);
	}

	return true;
}

bool Mesh::LoadVectorData(vector<Vector3d> & points, vector<Vector3i> & faces)
{

	// clear current mesh object
	Clear();

	for (size_t i = 0; i < points.size(); i++)
	{
		AddVertex(new Vertex(points[i]));
	}

	for (size_t i = 0; i < faces.size(); i++)
	{
		AddFace(faces[i][0], faces[i][1], faces[i][2]);
	}
	size_t i;
	/*
	// normalize
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());


	Vector3d tot;
	for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);
	*/

	for (i = 0; i < fList.size(); i++)
		fList[i]->SetLabelPath(i);

	HEdgeList list;
	for (i = 0; i<bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i = 0; i<vList.size(); i++)
	{
		vList[i]->adjHEdges.clear();
		vList[i]->SetIndex(i);
	}

	cout << endl;

	minCoord = MinCoord();
	maxCoord = MaxCoord();
	CalculateSphericalCoords();
	return true;
}

bool Mesh::SetGroundTruthLables(vector<int> & labels)
{
	if (labels.size() >= fList.size())
	{
		for (size_t i = 0; i < fList.size(); i++)
		{
			fList[i]->SetLabel(labels[fList[i]->LabelPath()]);
		}
		return true;
	}
	else return false;
}

void Mesh::GetGroundTruthLabels(vector<int> & labels)
{
	labels.clear();
	for (size_t i = 0; i < fList.size(); i++)
	{
		labels.push_back(fList[i]->Label());
	}
}

bool Mesh::SetPredictLabelsLabelsProbabilities(vector<int> & labels, vector<vector<float>> probList)
{
	if (labels.size() >= fList.size() && probList.size() >= fList.size())
	{
		for (size_t i = 0; i < fList.size(); i++)
		{
			fList[i]->p_label = labels[fList[i]->LabelPath()];
			fList[i]->r_label = fList[i]->p_label;
			fList[i]->prob.assign(probList[fList[i]->LabelPath()].begin(), probList[fList[i]->LabelPath()].end());
		}
		return true;
	}
	else return  false;
}


void Mesh::GetPredictLabelsProbabilities(vector<int> & labels, vector<vector<float>> probList)
{
	labels.clear();
	probList.clear();

	for (size_t i = 0; i < fList.size(); i++)
	{
		labels.push_back(fList[i]->p_label);
	}

	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		vector<float> prob;
		prob.assign(f->prob.begin(), f->prob.end());
	}
}

bool Mesh::SetTwoClassProbabilities(vector<vector<float>> & probList)
{
	if (probList.size() >= fList.size())
	{
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			f->twoClassesProb.assign(probList[f->LabelPath()].begin(), probList[f->LabelPath()].end());
		}
		return true;
	}
}

void Mesh::GetR_labels(vector<int> & labels)
{
	labels.clear();
	for (size_t i = 0; i < fList.size(); i++)
	{
		labels.push_back(fList[i]->r_label);
	}
}

void Mesh::GetF_labels(vector<int> & labels)
{
	labels.clear();
	for (size_t i = 0; i < fList.size(); i++)
	{
		labels.push_back(fList[i]->fuzzy_label);
	}
}

bool Mesh::SaveOBJWithLabel(int label, vector<int> & labelForTooth, string fileName)
{
	cout << "Saving " << fileName << endl;
	vector<Vector3d> mvList;
	vector<Vector3i> mfList;
	// find vertice
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->Label() == label || fList[i]->Label() == (label + 100))
		{
			for (size_t j = 0; j < 3; j++)
			{
				vList[fList[i]->v[j]]->SetIfCutToSave(true);
			}
		}
	}

	// assign tmpIndx and export vertice
	int count = 0;
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->IfCutToSave())
		{
			vList[i]->SetTmpIndex(count++);
			mvList.push_back(vList[i]->Position());
		}
	}

	// export faces and labels
	int v[3];
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->Label() == label || fList[i]->Label() == (label + 100))
		{
			for (size_t j = 0; j < 3; j++)
			{
				v[j] = vList[fList[i]->v[j]]->TmpIndex();
			}
			mfList.push_back(Vector3i(v[0], v[1], v[2]));
			int labelOfFace;
			if (fList[i]->Label() == label)
				labelOfFace = 0;
			else if (fList[i]->Label() == (label + 100))
				labelOfFace = 1;
			labelForTooth.push_back(labelOfFace);
		}
	}

	ofstream fout(fileName.c_str());
	if (fout.is_open())
	{
		fout << "# vetices " << mvList.size() << endl;
		fout << "# faces " << mfList.size() << endl;
		fout << "#################################" << endl;
		for (size_t i = 0; i < mvList.size(); i++)
		{
			Vector3d p = mvList[i];
			fout << "v " << p[0] << " " << p[1] << " " << p[2] << endl;
		}

		for (size_t i = 0; i < mfList.size(); i++)
		{
			fout << "f " << mfList[i][0] + 1 << " " << mfList[i][1] + 1 << " " << mfList[i][2] + 1 << endl;
		}
		fout.close();
		return true;
	}
	return false;
}


void Mesh::CalculateSphericalCoords()
{
	Vector3d center = 0.5*(maxCoord + minCoord);
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->CalculateSphericalCoord(center);
	}
}

void Mesh::DisplayMeshInfo()
{
	int V = vList.size();
	int E = (heList.size() + bheList.size()) / 2;
	int F = fList.size();
	//int B = CountBoundaryLoops();
	//int C = CountConnectedComponents();
	//int M = FindMaxConnectedComponet();
	//int G = C - (V - E + F + B) / 2;
	cout << "# of vertices: " << V << endl;
	cout << "# of half-edges: " << E * 2 << endl;
	cout << "# of faces: " << F << endl;
	//cout << "# of boundary loops: " << B << endl;
	//cout << "# of connected components: " << C << endl;
	//cout << "# of max connected component(face): " << groupCount[M] << endl;
	//cout << "# of genus: " << G << endl;
}


// compute the normal of each vertex
void Mesh::ComputeVertexNormals() 
{
	for (size_t i=0; i<vList.size(); i++)
	{
		Vertex *u = vList[i];
		Vector3d t1, t2;
		int valence = u->Valence();
		double stepAngle = 2.0 * PI / (double)valence;

		if (u->IsBoundary())
		{
			HEdge *bhe = NULL;
			HEdge *he = NULL;
			OneRingHEdge ring(u);
			while ((he=ring.NextHEdge()) != NULL) {
				if (he->IsBoundary()) bhe = he;
			}
			Vector3d p_st = bhe->Prev()->Start()->Position(); // p_0
			Vector3d p_ed = bhe->End()->Position(); // p_k-1
			t1 = p_st - p_ed; // t_along
			if (valence == 2)
			{
				t2 = p_st - p_ed - u->Position() * 2.0;
			}
			else if (valence == 3)
			{
				Vector3d p1 = bhe->Twin()->Next()->End()->Position();
				t2 = p1 - u->Position();
			}
			else
			{
				double angle = PI / (double)(valence - 1);
				double tmp = 2.0 * cos(angle) - 2.0;
				t2 = sin(angle) * t1;
				HEdge *he2 = bhe->Prev()->Twin()->Prev()->Twin();
				int i = 1;
				do 
				{
					t2 += (tmp * sin(i*angle)) * he2->End()->Position();
					he2 = he2->Prev()->Twin();
					i++;
				} while(he2 != bhe);
			}
		}
		else
		{
			double angle = 0;
			Vertex *v = NULL;
			OneRingVertex ring(u);
			while ((v = ring.NextVertex()) != NULL)
			{
				Vector3d p = v->Position();
				t1 += cos(angle) * p;
				t2 += sin(angle) * p;
				angle += stepAngle;
			}
		}
		Vector3d normal = t1.Cross(t2);
		normal /= normal.L2Norm();
		u->SetNormal(normal);
	}
}

// compute the Euclidean length of a half edge 
inline double HEdge::getLength()
{
	return this->Start()->Position().Distance(this->End()->Position());
}

// smooth normal based on estimated geodesic distance
Vector3d Mesh::computeSmoothVertexNormal(Vertex* center, double radius)
{
	// keep the current progress
	list<Vertex*> supp_list;
	list<double> dist_list;
	supp_list.push_back( center );
	dist_list.push_back( 0 );

	// store the result
	list<Vertex*> supp_res_list;
	list<double> dist_res_list;

	// using the cubic func to assign weight
	// compute the normal using weighted sum
	Vector3d smooth_normal = center->Normal();
	while (true)
	{
		if ( dist_list.empty() )
			break;
		double dist_offset = dist_list.front(); 
		if ( dist_offset > radius )
			break; // we've got enough
		dist_list.pop_front();
		dist_res_list.push_back( dist_offset );

		Vertex* curr = supp_list.front();
		supp_list.pop_front();
		supp_res_list.push_back( curr );

		OneRingVertex curr_ring( curr );
		Vertex* next;
		while ( (next = curr_ring.NextVertex()) != NULL )
		{
			double next_dist = next->Position().Distance(curr->Position()) + dist_offset;
			
			// insert this vertex to the right place
			if ( next_dist < radius )			 
			{
				bool flag = true;
				int size = dist_list.size();
				list<Vertex*>::iterator supp_iter = supp_list.begin(); 
				for ( list<double>::iterator dist_iter = dist_list.begin(); dist_iter != dist_list.end(); ++dist_iter )
				{
					if ( *dist_iter > next_dist )
					{
						dist_list.insert( dist_iter, next_dist );
						supp_list.insert( supp_iter, next );
						flag = false; 
						break;
					}
					++supp_iter;
				}				
				if ( flag )
				{
					dist_list.push_back( next_dist );
					supp_list.push_back( next );
				}
			}
		}
	}
	supp_list.clear();
	dist_list.clear();

	// calculate the sum of weights
	double sum_weight = 0;
	for ( list<double>::iterator dist_iter = dist_res_list.begin(); 
		  dist_iter != dist_res_list.end(); ++ dist_iter )
		sum_weight += *dist_iter;

	// compute the vertex normal
	double r2 = radius * radius;
	double r3 = r2 * radius;
	list<Vertex*>::iterator supp_iter = supp_res_list.begin();
	for ( list<double>::iterator dist_iter = dist_res_list.begin(); 
		  dist_iter != dist_res_list.end(); ++dist_iter )
	{
		double t = *dist_iter;
		double t2 = t * t;
		double t3 = t2 * t;
		double weight = 2/r3 * t3 - 3/r2 * t2 + 1;
		smooth_normal += (*supp_iter++)->Normal() * weight / sum_weight;
	}
	supp_res_list.clear();
	dist_res_list.clear();
	smooth_normal.Normalize();

	center->SetNormal( smooth_normal );
	return smooth_normal;
}

// estimate smooth vertex normal for all vertices
void Mesh::computeSmoothVertexNormals(VertexList ver_list, double radius)
{
	VertexList novo_vlist( ver_list.begin(), ver_list.end() );
	for ( VertexList::iterator viter = novo_vlist.begin(); viter != novo_vlist.end(); ++viter )
	{
		Vertex* curr = *viter;
		curr->SetNormal( computeSmoothVertexNormal(curr, radius) );
	}
	ver_list.clear();
	ver_list.assign( novo_vlist.begin(), novo_vlist.end() );
}


//void Mesh::toLocalFrame()
//{
//	// clear the local frame reference list
//	frameList.clear();
//
//	// find all the neighbor vertices to get the best match
//	for ( int i=0; i<vList.size(); ++i )
//	{
//		if ( vList[i]->Flag() )
//			continue;
//
//		// choose the vertex reference
//		Vertex* center = vList[i];
//		OneRingVertex ring(center);
//		Vertex* curr;
//		double best_cos = 1.0;
//		Vertex* best_cand = NULL;
//		while ( (curr = ring.NextVertex()) != NULL )
//		{
//			double curr_cos = center->Normal().Dot( curr->Normal() ) 
//				/ ( center->Normal().L2Norm() * curr->Normal().L2Norm() );
//			if ( curr_cos < best_cos )
//			{
//				best_cos = curr_cos;
//				best_cand = curr;
//			}
//		}
//		frameList.push_back( best_cand );
//
//		// build the local frame
//		Vector3d basis_x = center->Normal();
//		Vector3d basis_y = best_cand->Normal() - 
//			( best_cand->Normal().L2Norm() * best_cos / center->Normal().L2Norm() ) 
//			* center->Normal();
//		Vector3d basis_z = basis_x.Cross( basis_y );
//		
//		// normalize the frame
//		basis_x.Normalize();
//		basis_y.Normalize();
//		basis_z.Normalize();
//
//		// get the local value
//		double novo_x = 
//			center->Position()[0] * basis_x[0] + 
//			center->Position()[1] * basis_y[0] + 
//			center->Position()[2] * basis_z[0];
//		double novo_y = 
//			center->Position()[0] * basis_x[1] + 
//			center->Position()[1] * basis_y[1] + 
//			center->Position()[2] * basis_z[1];
//		double novo_z = 
//			center->Position()[0] * basis_x[2] + 
//			center->Position()[1] * basis_y[2] + 
//			center->Position()[2] * basis_z[2];
//
//		// update the vertex position
//		Vector3d local_pos(novo_x, novo_y, novo_z);
//		curr->SetPosition( local_pos );
//	}
//}

//// rewrite the coordinate in global frame
//void Mesh::toGlobalFrame()
//{
//	if ( frameList.empty() )
//		return;
//
//	for ( int i=0; i<vList.size(); ++i )
//	{
//		if ( vList[i]->Flag() )
//			continue;
//
//		// find the reference local frame
//		Vertex* curr = vList[i];
//		Vertex* ref = frameList[i];
//		Vector3d basis_x = curr->Normal();
//		Vector3d basis_y = ref->Normal() - 
//			(curr->Normal().Dot(ref->Normal())/pow(curr->Normal().L2Norm(), 2))*curr->Normal();
//		Vector3d basis_z = basis_x.Cross( basis_y );
//		basis_x.Normalize();
//		basis_y.Normalize();
//		basis_z.Normalize();
//
//		// write the coord back to global frame
//		curr->SetPosition(
//			curr->Position()[0] * basis_x +  
//			curr->Position()[1] * basis_y +
//			curr->Position()[2] * basis_z );
//	}
//}

// umbrella smoothing
void Mesh::UmbrellaSmooth() 
{
	size_t i;
	vector<Vector3d> list;

	for (i=0; i<vList.size(); i++) {
		if (vList[i]->IsBoundary()) { 
			list.push_back(vList[i]->Position());
			continue;
		}
		Vertex *u = vList[i];
		OneRingHEdge ring(u);
		HEdge *he = NULL;
		Vector3d newPosition;
		double weight = 0.0;
		while ((he=ring.NextHEdge()) != NULL) {
			Vertex *v = he->End();
			weight += 1.0;
			newPosition += v->Position();
		}
		newPosition /= weight;
		if (_isnan(newPosition.L2Norm())) newPosition = u->Position();
		list.push_back(newPosition);
	}

	for (i=0; i<vList.size(); i++) 
		vList[i]->SetPosition(list[i]);
}


// implicit umbrella smoothing
void Mesh::ImplicitUmbrellaSmooth()
{
	int n = vList.size();
	Matrix* M = new Matrix(n, n);

	for (size_t i=0; i<vList.size(); i++) 
	{
		Vertex *u = vList[i];
		OneRingHEdge ring(u);
		HEdge *he = NULL;
		double val = u->Valence();
		while ((he=ring.NextHEdge()) != NULL)
			M->AddElement(i, he->End()->Index(), -1.0/val);
		M->AddElement(i, i, 2.0);
	}
	M->SortMatrix();

	double* b = new double[n];
	double* x[3];
	for (int i=0; i<3; i++)
	{
		x[i] = new double[n];
		for (int j=0; j<n; j++)
			x[i][j] = b[j] = vList[j]->Position()[i];
		M->BCG(b, x[i], 1000, 1e-12);
	}

	double ratio = 1.0;
	for (int i=0; i<n; i++)
	{
		Vector3d v = vList[i]->Position();
		double X = v.X() + ratio * (x[0][i] - v.X());
		double Y = v.Y() + ratio * (x[1][i] - v.Y());
		double Z = v.Z() + ratio * (x[2][i] - v.Z());
		vList[i]->SetPosition(Vector3d(X,Y,Z));
	}

	delete[] b;
	delete[] x[0];
	delete[] x[1];
	delete[] x[2];
	delete M;
}


// compute the vertex curvature of the graph
void Mesh::ComputeVertexCurvatures()
{
	size_t i, j;
	vector<double> curvatureList;
	double meanCurvature = 0.0;
	double maxCurvature = 0.0;
	double minCurvature = 1e10;

	for (i = 0, j = 0; i < vList.size(); i++)
	{
		if (vList[i]->IsBoundary())
		{
			curvatureList.push_back(0.0);
			vList[i]->SetColor(Vector3d(0, 0, 0));
			continue;
		}
		j++;
		Vertex *u = vList[i];
		OneRingHEdge ring(u);
		HEdge *he = NULL;
		Vector3d curvatureNormal;
		double area = 0.0;
		while ((he = ring.NextHEdge()) != NULL)
		{
			Vertex *v = he->End();
			Vertex *vNext = he->Next()->End();
			Vertex *vPrev = he->Twin()->Prev()->Start();
			double cotA = Cot(v->Position(), vNext->Position(), u->Position());
			double cotB = Cot(u->Position(), vPrev->Position(), v->Position());
			curvatureNormal += (cotA + cotB) * (v->Position() - u->Position());
			area += Area(u->Position(), v->Position(), vNext->Position());
		}
		curvatureNormal /= 4.0 * area;
		double len = curvatureNormal.L2Norm();
		maxCurvature = max(maxCurvature, len);
		minCurvature = min(minCurvature, len);
		meanCurvature += len;
		curvatureList.push_back(len);
	}
	meanCurvature /= (double)j;

	double diff = maxCurvature - minCurvature;
	for (i = 0; i < vList.size(); i++) {
		if (vList[i]->IsBoundary()) continue;
		Vertex *u = vList[i];
		double x = curvatureList[i] - meanCurvature;
		//double x = 2*(curvatureList[i] - minCurvature) / diff - 1.0;
		if (x >= 0) u->SetColor(Vector3d(x, 1, 0));
		else u->SetColor(Vector3d(0, 1, -x));
	}
}

int Mesh::CountBoundaryLoops()
{
	for (size_t i=0; i<bheList.size(); i++)
		bheList[i]->SetFlag(false);

	int count = 0;
	for (size_t i=0; i<bheList.size(); i++)
	{
		HEdge * bhe = bheList[i];
		if (bhe->Flag()) continue;
		
		count++;
		HEdge * curr = bhe;
		do 
		{
			curr->SetFlag(true);
			curr = curr->Next();
		} while(curr != bhe);
	}

	return count;
}

int Mesh::CountConnectedComponents()
{
	for (size_t i=0; i<heList.size(); i++)
		heList[i]->SetFlag(false);

	for (size_t i=0; i<bheList.size(); i++)
		bheList[i]->SetFlag(false);

	int count = 0;
	HEdgeList tmpList;

	for (size_t i=0; i<heList.size(); i++)
	{
		HEdge * he = heList[i];
		if (he->Flag()) continue;

		count++;
		he->SetFlag(true);
		tmpList.push_back(he);
		while (!tmpList.empty())
		{
			HEdge *curr = tmpList.back();
			tmpList.pop_back();
			if (curr->Twin()->Flag() == false)
			{
				curr->Twin()->SetFlag(true);
				tmpList.push_back(curr->Twin());
			}
			if (curr->Next()->Flag() == false)
			{
				curr->Next()->SetFlag(true);
				tmpList.push_back(curr->Next());
			}
			if (curr->Prev()->Flag() == false)
			{
				curr->Prev()->SetFlag(true);
				tmpList.push_back(curr->Prev());
			}
		}
	}
	return count;
}

int Mesh::FindMaxConnectedComponet()
{
	if (maxGroupID > 0) return maxGroupID;

	for (size_t i = 0; i<heList.size(); i++)
		heList[i]->SetFlag(false);

	for (size_t i = 0; i<bheList.size(); i++)
		bheList[i]->SetFlag(false);

	for (size_t i = 0; i < fList.size(); i++)
		fList[i]->SetGroupID(-1);

	groupCount.clear();
	maxGroupID = -1;
	HEdgeList tmpList;
	

	for (size_t i = 0; i<heList.size(); i++)
	{
		HEdge * he = heList[i];
		if (he->Flag()) continue;

		int count = 0;
		he->SetFlag(true);
		tmpList.push_back(he);
		while (!tmpList.empty())
		{
			HEdge *curr = tmpList.back();
			tmpList.pop_back();

			if (curr->LeftFace()!=NULL && curr->LeftFace()->GroupID() < 0)
			{
				curr->LeftFace()->SetGroupID(groupCount.size());
				count++;
			}

			if (curr->Twin() && curr->Twin()->Flag() == false)
			{
				curr->Twin()->SetFlag(true);
				tmpList.push_back(curr->Twin());
			}
			if (curr->Next() && curr->Next()->Flag() == false)
			{
				curr->Next()->SetFlag(true);
				tmpList.push_back(curr->Next());
			}
			if (curr->Prev() && curr->Prev()->Flag() == false)
			{
				curr->Prev()->SetFlag(true);
				tmpList.push_back(curr->Prev());
			}
		}

		groupCount.push_back(count);
		if (maxGroupID == -1 || groupCount[maxGroupID] < count)
			maxGroupID = groupCount.size() - 1;
	}

	return maxGroupID;
}

void Mesh::SaveMaxConnectedComponet(const char * filename)
{
	vector<Vector3d> mvList;
	vector<Vector3i> mfList;
	vector<int> mlList;
	vector<int> mvbList;
	ExportMaxConnectedComponent(mvList, mfList, mlList, mvbList);

	string outputfile = filename;

	ofstream fout(outputfile);
	if (fout.fail())
		return;

	fout << "# vetices " << mvList.size() << endl;
	fout << "# faces " << mfList.size() << endl;
	fout << "#################################" << endl;
	//fout.setf(ios::fixed, ios::floatfield);  // 设定为 fixed 模式，以小数点表示浮点数  
	//fout.precision(8);  // 设置精度 6 
	for (size_t i = 0; i < mvList.size(); i++)
	{
		Vector3d p = mvList[i];
		fout << "v " << p[0] << " " << p[1] << " " << p[2] << endl;
	}

	for (size_t i = 0; i < mfList.size(); i++)
	{
		fout << "f " << mfList[i][0] + 1 << " " << mfList[i][1] + 1 << " " << mfList[i][2] + 1 << endl;
	}

	fout.close();
	return;
}

void Mesh::GroupingVertexFlags()
{
	// set vertex flag to be 255 initially
	for (size_t i=0; i<vList.size(); i++)
		if (vList[i]->Flag() != 0) 
			vList[i]->SetFlag(255);

	int id = 0;
	VertexList tmpList;
	for (size_t i=0; i<vList.size(); i++)
		if (vList[i]->Flag() == 255)
		{
			id++;
			vList[i]->SetFlag(id);
			tmpList.push_back(vList[i]);
			while ( ! tmpList.empty())
			{
				Vertex * v = tmpList.back();
				tmpList.pop_back();
				OneRingVertex ring = OneRingVertex(v);
				while (Vertex * v2 = ring.NextVertex())
				{
					if (v2->Flag() == 255)
					{
						v2->SetFlag(id);
						tmpList.push_back(v2);
					}
				}
			}
		}
}

void Mesh::FindLabelBoundaryVertex()
{
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL && he->LeftFace()->Label() != he->Twin()->LeftFace()->Label())
		{
			he->Start()->SetLabelBoundary(1);
			he->End()->SetLabelBoundary(1);
		}
	}
}

void Mesh::BFSFromBoundaryVertex()
{
	for (size_t i = 0; i < vList.size(); i++)
	{
		vList[i]->boundaryLevel = -1;
	}

	// initialize boundary vertex
	int level = 0;
	VertexList bvList;
	for (size_t i = 0; i < fList.size(); i++)
	{
		Face *f = fList[i];
		HEdge *he = f->HalfEdge();

		do
		{
			if (he->Twin()->LeftFace() != NULL)
			{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (f->Label() != adjcentFace->Label())
				{
					if (he->Start()->boundaryLevel < 0)
					{
						bvList.push_back(he->Start());
						he->Start()->boundaryLevel = level;
					}
						
					if (he->End()->boundaryLevel < 0)
					{
						bvList.push_back(he->End());
						he->End()->boundaryLevel = level;
					}	
				}
			}
			he = he->Next();
		} while (he != f->HalfEdge());
	}
	//cout << bvList.size() << endl;

	VertexList tmpBvList;
	while (bvList.size()>0)
	{
		tmpBvList.clear();
		level++;
		while (bvList.size()>0)
		{
			Vertex * u = bvList.back();
			bvList.pop_back();
			
			OneRingHEdge ring(u);
			HEdge * he = NULL;
			int count = 0;
			while ((he = ring.NextHEdge()) != NULL)
			{
				if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
				{
					Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
					if (he->Start()->boundaryLevel < 0)
					{
						tmpBvList.push_back(he->Start());
						he->Start()->boundaryLevel = level;
					}

					if (he->End()->boundaryLevel < 0)
					{
						tmpBvList.push_back(he->End());
						he->End()->boundaryLevel = level;
					}
				}
				count++;
				if (count == 100000)
				{
					cout << "vertex valence error!" << endl;
					break;
				}
			}
		}
		//cout << bvList.size() << " " << tmpBvList.size() << endl;
		bvList.clear();
		bvList.assign(tmpBvList.begin(), tmpBvList.end());
		
	}

	
	//int halfLevel = level >> 2;
	//halfLevel = max(halfLevel, 1);
	//for (size_t i = 0; i < vList.size(); i++)
	//{
	//	Vertex * v = vList[i];
	//	if (v->boundaryLevel < 0)
	//	{
	//		v->SetColor(Vector3d(0, 0, 0));
	//	}
	//	else
	//	{
	//		if (v->boundaryLevel < halfLevel)
	//		{
	//			double x = 1 - (v->boundaryLevel / halfLevel);
	//			x = min(x, 1.0);
	//			v->SetColor(Vector3d(x, 0.5*(1 - x), 0.5*(1 - x)));
	//		}
	//		else
	//		{
	//			double x = v->boundaryLevel / halfLevel - 1;
	//			x = min(x, 1.0);
	//			v->SetColor(Vector3d(0.5*(1 - x), 0.5*(1 - x), x));
	//		}
	//	}

	//}


}

void Mesh::FindLabelBoundaryFace(int propagate)
{
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL && he->LeftFace()->Label() != he->Twin()->LeftFace()->Label())
		{
			he->LeftFace()->labelBoundary = true;
			he->Twin()->LeftFace()->labelBoundary = true;
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace()!=NULL)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << "Label Boundary Face Count: " << count << endl;
}

void Mesh::FindLabelBoundaryFace2(int propagate)
{
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL && he->LeftFace()->Label() != he->Twin()->LeftFace()->Label())
		{
			if (he->LeftFace()->Label() == 0 || he->Twin()->LeftFace()->Label() == 0)
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;

			}
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << "Label Boundary Face Count: " << count << endl;
}

void Mesh::FindLabelBoundaryFace4(int propagate)
{
	bool sameClass[10][10];
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			sameClass[i][j] = false;
		}
		sameClass[0][i] = true;
		sameClass[i][i] = true;
	}

	sameClass[1][2] = true;
	sameClass[4][5] = true;
	sameClass[6][7] = true;
	sameClass[6][8] = true;
	sameClass[7][8] = true;

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < i; j++)
		{
			sameClass[i][j] = sameClass[j][i];
		}
	}

	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];

		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			if (sameClass[he->LeftFace()->Label() % 10][he->Twin()->LeftFace()->Label() % 10] == false)
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;
			}
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL && he->Twin()->LeftFace()->Label() != 0)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << "Label Boundary Face Count: " << count << endl;
}

void Mesh::FindLabelBoundaryFace8(int propagate)
{
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			if (he->LeftFace()->Label()%10 != he->Twin()->LeftFace()->Label()%10 && he->LeftFace()->Label() != 0 && he->Twin()->LeftFace()->Label() != 0)
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;
			}
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL && he->Twin()->LeftFace()->Label() != 0)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << "Label Boundary Face Count: " << count << endl;
}

void Mesh::FindLabelBoundaryFace16(int propagate)
{
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			if (he->LeftFace()->Label() != he->Twin()->LeftFace()->Label() && he->LeftFace()->Label() != 0 && he->Twin()->LeftFace()->Label() != 0)
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;
			}
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL && he->Twin()->LeftFace()->Label() != 0)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << "Label Boundary Face Count: " << count << endl;
}

void Mesh::FindToothGumBoundary(int propagate)
{
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			if (he->LeftFace()->Label() != he->Twin()->LeftFace()->Label() && (he->LeftFace()->Label() == 0 || he->Twin()->LeftFace()->Label() == 0))
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;
			}
		}
	}

	// propagate
	for (int k = 0; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	int count[4] = { 0, 0, 0, 0 };

	for (size_t i = 0; i < fList.size(); i++)
	{
		Face *f = fList[i];
		if (f->labelBoundary == false)
		{
			if (f->Label()>0)
				count[1]++;
			else
				count[0]++;
		}
		else
		{
			if (f->Label() > 0)
				count[3]++;
			else
				count[2]++;
		}
	}

	cout << "Label Count: " << count[0] << " " << count[1] << " " << count[2] << " " << count[3] << endl;
}

void Mesh::FindGumFacesAroundTooth(FaceList & gumfList, vector<int> & toothIDList, int propagate)
{
	gumfList.clear();

	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->visited = false;
	}
	// initialize gum and toothID tooth boundary face;
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge * he = heList[i];
		if (he->LeftFace() == NULL || he->Twin()->LeftFace() == NULL)
			continue;

		Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
		if (f1->Label() != 0) continue;
		vector<int>::iterator iter = find(toothIDList.begin(), toothIDList.end(), f2->Label());
		if (iter == toothIDList.end()) continue;

		if (f1->visited) continue;
		f1->visited = true;
		gumfList.push_back(f1);
	}

	// propagate
	if (gumfList.empty()) return;

	size_t start = 0, end = gumfList.size();
	for (int k = 1; k < propagate; k++)
	{
		for (size_t i = start; i < end; i++)
		{
			Face * f = gumfList[i];
			HEdge * he = f->HalfEdge();
			do{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (adjcentFace && !adjcentFace->visited && adjcentFace->Label() == 0)
				{
					adjcentFace->visited = true;
					gumfList.push_back(adjcentFace);
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}

		start = end;
		end = gumfList.size();
	}

	cout << "gum faces around windowm: " << gumfList.size() << endl;
}

void Mesh::FindClassificationRefineBoundary(int level, int propagate)
{
	if (fList.size() == 0) return;
	if (fList[0]->r_label_list.size() <= (unsigned int)level) return;


	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			if (he->LeftFace()->r_label_list[level] != he->Twin()->LeftFace()->r_label_list[level])
			{
				he->LeftFace()->labelBoundary = true;
				he->Twin()->LeftFace()->labelBoundary = true;
			}
		}
	}

	// propagate
	for (int k = 1; k < propagate; k++)
	{
		FaceList bfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			if (fList[i]->labelBoundary)
				bfList.push_back(fList[i]);
		}
		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
						he->Twin()->LeftFace()->labelBoundary = true;
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
	}

	// output
	/*int count = 0;
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->labelBoundary)
			count++;
	}

	cout << level << " Level Classification Boundary Count( " << propagate << " propagate): " << count << endl;*/
}

void Mesh::FindClassificationRefineBoundaryCmd(int propagate, int optLabel)
{
	if (fList.size() == 0) return;


	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	FaceList bfList;
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			int r_label1 = he->LeftFace()->r_label;
			int r_label2 = he->Twin()->LeftFace()->r_label;
			if ((r_label1 == optLabel && r_label2 == 0) || (r_label1 == 0 && r_label2 == optLabel))
			{
				if (!he->LeftFace()->labelBoundary)
				{
					he->LeftFace()->labelBoundary = true;
					bfList.push_back(he->LeftFace());
				}
				if (!he->Twin()->LeftFace()->labelBoundary)
				{
					he->Twin()->LeftFace()->labelBoundary = true;
					bfList.push_back(he->Twin()->LeftFace());
				}
				
			}
		}
	}

	// propagate
	size_t start = 0, end;
	for (int k = 1; k < propagate; k++)
	{
		end = bfList.size();
		for (size_t i = start; i < end; i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
					{
						int tmpLabel = he->Twin()->LeftFace()->r_label;
						if (tmpLabel == optLabel || tmpLabel == 0)
						{
							if (!he->Twin()->LeftFace()->labelBoundary)
							{
								he->Twin()->LeftFace()->labelBoundary = true;
								bfList.push_back(he->Twin()->LeftFace());
							}
						}

					}

					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
		start = end;
	}

	// output
	//int count = 0;
	//for (size_t i = 0; i < fList.size(); i++)
	//{
	//	if (fList[i]->labelBoundary)
	//		count++;
	//}

	//cout << "2 Classification Boundary Count( " << propagate << " propagate): " << count << endl;
}

void Mesh::FindClassificationRefineBoundaryCmd(int propagate)
{
	if (fList.size() == 0) return;


	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	FaceList bfList;
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			int r_label1 = he->LeftFace()->r_label;
			int r_label2 = he->Twin()->LeftFace()->r_label;
			if (r_label1 != r_label2 == 0)
			{
				if (!he->LeftFace()->labelBoundary)
				{
					he->LeftFace()->labelBoundary = true;
					bfList.push_back(he->LeftFace());
				}
				if (!he->Twin()->LeftFace()->labelBoundary)
				{
					he->Twin()->LeftFace()->labelBoundary = true;
					bfList.push_back(he->Twin()->LeftFace());
				}

			}
		}
	}

	// propagate
	size_t start = 0, end;
	for (int k = 1; k < propagate; k++)
	{
		end = bfList.size();
		for (size_t i = start; i < end; i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
					{
						if (!he->Twin()->LeftFace()->labelBoundary)
						{
							he->Twin()->LeftFace()->labelBoundary = true;
							bfList.push_back(he->Twin()->LeftFace());
						}

					}

					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
		start = end;
	}

	// output
	//int count = 0;
	//for (size_t i = 0; i < fList.size(); i++)
	//{
	//	if (fList[i]->labelBoundary)
	//		count++;
	//}

	//cout << "2 Classification Boundary Count( " << propagate << " propagate): " << count << endl;
}

void Mesh::FindFuzzyBoundary(FaceList & bfList, int propagate, int optLabel)
{
	if (fList.size() == 0) return;


	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->labelBoundary = false;
	}

	// initial
	bfList.clear();
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			int fuzzy_label1 = he->LeftFace()->fuzzy_label;
			int fuzzy_label2 = he->Twin()->LeftFace()->fuzzy_label;
			if ((fuzzy_label1 == optLabel && fuzzy_label2 == 0) || (fuzzy_label1 == 0 && fuzzy_label2 == optLabel))
			{
				if (!he->LeftFace()->labelBoundary)
				{
					he->LeftFace()->labelBoundary = true;
					bfList.push_back(he->LeftFace());
				}
				if (!he->Twin()->LeftFace()->labelBoundary)
				{
					he->Twin()->LeftFace()->labelBoundary = true;
					bfList.push_back(he->Twin()->LeftFace());
				}

			}
		}
	}

	// propagate
	size_t start = 0, end;
	for (int k = 1; k < propagate; k++)
	{
		end = bfList.size();
		for (size_t i = start; i < end; i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
					{
						int tmpLabel = he->Twin()->LeftFace()->fuzzy_label;
						if (tmpLabel == optLabel || tmpLabel == 0)
						{
							if (!he->Twin()->LeftFace()->labelBoundary)
							{
								he->Twin()->LeftFace()->labelBoundary = true;
								bfList.push_back(he->Twin()->LeftFace());
							}
						}

					}

					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
		start = end;
	}
}

void Mesh::FindFuzzyToothBoundary()
{
	for (size_t i = 0; i < fList.size(); i++)
		fList[i]->fuzzyToothBoundary = false;

	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge * he = heList[i];
		Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
		if (f1==NULL || f2==NULL) continue;

		if (f1->fuzzy_label>0 && f2->fuzzy_label > 0 && f1->fuzzy_label != f2->fuzzy_label)
		{
			f1->fuzzyToothBoundary = true;
			f2->fuzzyToothBoundary = true;
		}
	}

}

void Mesh::FindFuzzyBoundaryAngDist(FaceList & bfList, int propagate, float angDistTh, float yita, int optLabel)
{
	if (fList.size() == 0) return;


	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->visited = false;
		fList[i]->labelBoundary = false;
		fList[i]->tmpAngDist = 1000.0f;
	}

	// initial
	bfList.clear();
	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge* he = heList[i];
		if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
		{
			Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
			int fuzzy_label1 = f1->fuzzy_label;
			int fuzzy_label2 = f2->fuzzy_label;
			if ((fuzzy_label1 == optLabel && fuzzy_label2 == 0) || (fuzzy_label1 == 0 && fuzzy_label2 == optLabel))
			{
				float angDist = he->convex ? yita*he->angDist : he->angDist;
				angDist = min(0.08f, angDist);
				if (!f1->labelBoundary && !f1->visited)
				{
					f1->labelBoundary = true;
					f1->visited = true;
					f1->tmpAngDist = angDist;
					bfList.push_back(f1);

					f1->tmpLabel = 1;
				}
				if (!f2->labelBoundary && !f2->visited)
				{
					f2->labelBoundary = true;
					f2->visited = true;
					f2->tmpAngDist = angDist;
					bfList.push_back(f2);

					f2->tmpLabel = 1;
				}

			}
		}
	}

	// propagate
	size_t start = 0, end;
	for (int k = 1; k < propagate; k++)
	{
		if (k == propagate - 1)
		{
			start = 0;
		}

		end = bfList.size();
		for (size_t i = start; i < end; i++)
		{
			Face * f = bfList[i];
			if (f->labelBoundary)
			{
				HEdge * he = f->HalfEdge();
				do{
					float angDist = he->convex ? yita*he->angDist : he->angDist;
					angDist = min(0.08f, angDist);
					if (he->Twin()->LeftFace() != NULL)
					{
						Face * adjcentFace = he->Twin()->LeftFace();
						int tmpLabel = adjcentFace->fuzzy_label;

						if (tmpLabel == optLabel || tmpLabel == 0)
						{
							if (!adjcentFace->labelBoundary && !adjcentFace->visited)
							{
								adjcentFace->labelBoundary = true;
								adjcentFace->visited = true;
								adjcentFace->tmpAngDist = min(adjcentFace->tmpAngDist, f->tmpAngDist + angDist);
								
								if (adjcentFace->tmpAngDist < angDistTh)
								{
									bfList.push_back(adjcentFace);
									adjcentFace->tmpLabel = 1;
								}
								if (k == propagate - 1)
								{
									adjcentFace->visited = true;
									bfList.push_back(adjcentFace);
								}
							}
						}


					}

					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}
		start = end;
	}

}

void Mesh::ExportToothCoordsRange(int id)
{
	Vector3d avgCoords(0, 0, 0), maxCoords(-100, -100, -100), minCoords(100, 100, 100);
	int count = 0;

	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		if (f->Label() == id)
		{
			count++;
			avgCoords += f->center;
			
			for (int j = 0; j < 3; j++)
			{
				maxCoords[j] = max(maxCoords[j], f->center[j]);
				minCoords[j] = min(minCoords[j], f->center[j]);
			}
		}
	}

	if (count>0)
		avgCoords /= count;
	cout << "Tooth " << id << " average coords: " << avgCoords << endl;
	cout << "Tooth " << id << " max coords: " << maxCoords << endl;
	cout << "Tooth " << id << " min coords: " << minCoords << endl;
}

void Mesh::FindCancavePoints()
{
	const float eps = 0.01f;
	need_normals();

	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex *u = vList[i];
		OneRingVertex ring(u);
		Vertex *v = NULL;
		while ((v = ring.NextVertex()) != NULL)
		{
			Vector3d vdiff = u->Position() - v->Position();
			vdiff.Normalize();

			Vector3d ndiff = v->Normal() - u->Normal();
			if (vdiff.Dot(ndiff)>eps)
			{
				u->cancave_flag = true;
				break;
			}
		}
	}
}

void Mesh::CalculateSimplifiyLevel(
char modelType,
float seedRange1, float seedRange2,
int isolatedRange, int iterationCount, int field,
int localHeightest1, int localHeightest2,
int protect1, int protect2,
int cuttingField, float cuttingTh)
{
	need_curvatures();
	
	// find seed
	//VertexList seedList;
	seedList.clear();
	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex * v = vList[i];
		if (v->maxCurv>seedRange1 && v->maxCurv < seedRange2)
		{
			seedList.push_back(v);
			v->seed_flag = true;
		}
	}

	// remove isolated points
	for (size_t i = 0; i < seedList.size(); i++)
	{
		seedList[i]->visited = false;
		seedList[i]->SetTmpIndex(-1);
	}

	vector<int> groupCount;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		Vertex * v = seedList[i];
		if (!v->visited)
		{
			VertexList tmpvList;
			tmpvList.push_back(v);
			v->visited = true;
			groupCount.push_back(0);

			while (!tmpvList.empty())
			{
				Vertex * u = tmpvList.back();
				u->SetTmpIndex(groupCount.size() - 1);
				tmpvList.pop_back();
				groupCount[groupCount.size() - 1]++;

				OneRingVertex ring(u);
				Vertex * vv = NULL;
				while ((vv = ring.NextVertex()) != NULL)
				{
					if (!vv->visited && vv->seed_flag)
					{
						tmpvList.push_back(vv);
						vv->visited = true;
					}
				}
			}
		}
	}

	for (size_t i = 0; i < seedList.size(); i++)
	{
		Vertex * v = seedList[i];
		if (groupCount[v->TmpIndex()] < isolatedRange)
		{
			v->seed_flag = false;
		}
	}

	UpdateSeedList(seedList);

	// open-close
	for (int k = 0; k < iterationCount; k++)
	{
		CloseOperation(seedList, field);
		//OpenOperation(seedList, field);
	}

	VertexList LHvList;
	FindLocalHeightestPoints(modelType, localHeightest1,localHeightest2,LHvList);
	FindProtectPoints(protect1, protect2, LHvList);

	VertexList cuttingList;
	FindCuttingPoints(cuttingField, cuttingList, cuttingTh);
}

void Mesh::UpdateSeedList(VertexList & seedList)
{
	int count = 0;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		if (seedList[i]->seed_flag)
			seedList[count++] = seedList[i];
	}
	seedList.resize(count);
}

// first Eroding, then Dilating
void Mesh::OpenOperation(VertexList & seedList, int field)
{
	// Eroding
	VertexList erodingList;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		if (ErodingOperation(seedList[i], field))
			erodingList.push_back(seedList[i]);
	}

	for (size_t i = 0; i < erodingList.size(); i++)
	{
		erodingList[i]->seed_flag = false;
	}
	UpdateSeedList(seedList);

	// Dilating
	VertexList dilatingList;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		DilatingOperation(seedList[i], field, dilatingList);
	}
	for (size_t i = 0; i < dilatingList.size(); i++)
	{
		seedList.push_back(dilatingList[i]);
	}
}

// first Dilating, then Eroding
void Mesh::CloseOperation(VertexList & seedList, int field)
{
	// Dilating
	VertexList dilatingList;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		DilatingOperation(seedList[i], field, dilatingList);
	}
	for (size_t i = 0; i < dilatingList.size(); i++)
	{
		seedList.push_back(dilatingList[i]);
	}

	// Eroding
	VertexList erodingList;
	for (size_t i = 0; i < seedList.size(); i++)
	{
		if (ErodingOperation(seedList[i], field))
			erodingList.push_back(seedList[i]);
	}

	for (size_t i = 0; i < erodingList.size(); i++)
	{
		erodingList[i]->seed_flag = false;
	}
	UpdateSeedList(seedList);
}

bool Mesh::ErodingOperation(Vertex * u, int field)
{
	OneRingVertex ring(u);
	VertexList oneFieldvList;
	Vertex * v = NULL;
	while ((v = ring.NextVertex()) != NULL)
	{
		oneFieldvList.push_back(v);
	}

	int sum_complex = 0;
	for (size_t i = 0; i < oneFieldvList.size(); i++)
	{
		Vertex * v1 = oneFieldvList[i];
		Vertex * v2 = oneFieldvList[(i + 1) % oneFieldvList.size()];
		if (v1->seed_flag != v2->seed_flag)
			sum_complex++;
	}
	if (sum_complex >= 4)
		return false;


	VertexList tmpvList;
	FindKFieldFromVertex(u, field, tmpvList);
	for (size_t i = 0; i < tmpvList.size(); i++)
	{
		if (!tmpvList[i]->seed_flag)
			return true;
	}

	return false;
}

void Mesh::DilatingOperation(Vertex * u, int field, VertexList & dilatingList)
{
	VertexList tmpvList;
	FindKFieldFromVertex(u, field, tmpvList);

	for (size_t i = 0; i < tmpvList.size(); i++)
	{
		if (!tmpvList[i]->seed_flag)
		{
			tmpvList[i]->seed_flag = true;
			dilatingList.push_back(tmpvList[i]);
		}
	}
}

void Mesh::FindKFieldFromVertex(Vertex * u, int field, VertexList & tmpvList)
{
	field = max(field, 0);
	tmpvList.clear();
	VertexList vSet;
	vSet.push_back(u);
	for (int k = 0; k < field; k++)
	{
		VertexList nextvSet;
		for (size_t i = 0; i < vSet.size(); i++)
		{
			Vertex * v = vSet[i];

			OneRingVertex ring(v);
			Vertex * vv = NULL;
			while ((vv = ring.NextVertex()) != NULL)
			{
				if (vv == u) continue;
				VertexList::iterator iter = find(tmpvList.begin(), tmpvList.end(), vv);
				if (iter == tmpvList.end())
				{
					nextvSet.push_back(vv);
					tmpvList.push_back(vv);
				}
			}
		}
		vSet.clear();
		vSet.assign(nextvSet.begin(), nextvSet.end());
	}
}


bool Mesh::IsLocalHeightest(char modelType, Vertex * u, Vertex * v)
{
	if (modelType == 'U')
	{
		if (u->Position()[2] > v->Position()[2]) return true;
		else return false;
	}
	else if (modelType == 'L')
	{
		if (u->Position()[2] < v->Position()[2]) return true;
		else return false;
	}
	else
	{
		cout << "Unknow Tooth Type!" << endl;
		return false;
	}
}

void Mesh::FindLocalHeightestPoints(char modelType, int frontfield, int backfield, VertexList & LHvList)
{
	double thYCoord;

	thYCoord = 0.4*maxCoord[1] + 0.6*minCoord[1];

	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex * u = vList[i];
		VertexList fieldvList;
		
		// quickly relu out some non-local heightest points
		bool heightestFlag = true;
		OneRingVertex ring(u);
		Vertex * v = NULL;
		while ((v = ring.NextVertex()) != NULL)
		{
			if (!IsLocalHeightest(modelType, u, v))
			{
				heightestFlag = false;
				break;
			}
		}
		if (!heightestFlag) continue;

		fieldvList.clear();
		if (u->Position()[1]< thYCoord)
			FindKFieldFromVertex(u, frontfield, fieldvList);
		else
			FindKFieldFromVertex(u, backfield, fieldvList);
		
		for (size_t j = 0; j < fieldvList.size(); j++)
		{
			Vertex * v = fieldvList[j];
			if (!IsLocalHeightest(modelType, u, v))
			{
				heightestFlag = false;
				break;
			}
		}

		if (heightestFlag)
		{
			u->local_heightest = 0;
			LHvList.push_back(u);
		}
	}

	// BFS
	for (size_t i = 0; i < vList.size(); i++)
		vList[i]->visited = false;

	VertexList vSet, nextvSet;
	vSet.assign(LHvList.begin(), LHvList.end());
	for (size_t i = 0; i < vSet.size(); i++)
		vSet[i]->visited = true;

	int count = 1;
	while (!vSet.empty())
	{
		Vertex * u = vSet.back();
		vSet.pop_back();
		OneRingVertex ring(u);
		Vertex * v = NULL;
		while ((v = ring.NextVertex()) != NULL)
		{
			if (!v->visited)
			{
				v->visited = true;
				v->local_heightest = count;
				nextvSet.push_back(v);
			}
		}

		if (vSet.empty() && !nextvSet.empty())
		{
			vSet.assign(nextvSet.begin(), nextvSet.end());
			nextvSet.clear();
			count++;
		}
	}

	localBFSLevel = count;

}

void Mesh::FindProtectPoints(int frontfield,int backfield, VertexList & LHvList)
{
	double thYCoord = 0.4*maxCoord[1] + 0.6*minCoord[1];

	for (size_t i = 0; i < vList.size(); i++)
	{
		vList[i]->visited = false;
	}

	VertexList frontvSet, backvSet;
	for (size_t i = 0; i < LHvList.size(); i++)
	{
		if (LHvList[i]->Position()[1]<thYCoord)
			frontvSet.push_back(LHvList[i]);
		else
			backvSet.push_back(LHvList[i]);

	}

	for (int k = 0; k < 2; k++)
	{
		VertexList vSet, nextvSet;
		int field;
		if (k == 0)
		{
			// front 
			vSet.clear();
			vSet.assign(frontvSet.begin(), frontvSet.end());
			field = frontfield;
		}
		else
		{
			// back
			vSet.clear();
			vSet.assign(backvSet.begin(), backvSet.end());
			field = backfield;
		}
		for (size_t i = 0; i < vSet.size(); i++)
			vSet[i]->visited = true;

		int count = 0;
		while (!vSet.empty())
		{
			Vertex * u = vSet.back();
			vSet.pop_back();
			OneRingVertex ring(u);
			Vertex * v = NULL;
			while ((v = ring.NextVertex()) != NULL)
			{
				if (!v->visited)
				{
					v->visited = true;
					v->protect_flag = true;
					nextvSet.push_back(v);
				}
			}

			if (vSet.empty() && !nextvSet.empty())
			{
				count++;
				if (count > field) break;
				vSet.clear();
				vSet.assign(nextvSet.begin(), nextvSet.end());
			}
		}
	}
}

void Mesh::FindCuttingPoints(int field, VertexList & cuttingList, float thDist)
{
	for (size_t t = 0; t < seedList.size(); t++)
	{
		seedList[t]->sample_point = true;
	}
	
	for (size_t t = 0; t < seedList.size(); t++)
	{
		Vertex * u = seedList[t];
		if (u->cutting_point) continue;
		if (!u->sample_point) continue;
	
		VertexList vSet;
		// first pass: set all vertices's visited false
		vSet.push_back(u);
		for (int k = 0; k < field+1; k++)
		{
			VertexList nextvSet;
			for (size_t i = 0; i < vSet.size(); i++)
			{
				Vertex * v = vSet[i];

				OneRingVertex ring(v);
				Vertex * vv = NULL;
				while ((vv = ring.NextVertex()) != NULL)
				{
					vv->visited = false;
					nextvSet.push_back(vv);
				}
			}
			vSet.clear();
			vSet.assign(nextvSet.begin(), nextvSet.end());
		}

		// second pass: find k-field boundary
		VertexList totalvList;
		vSet.clear();
		u->visited = true;
		vSet.push_back(u);
		totalvList.push_back(u);

		for (int k = 0; k < field; k++)
		{
			VertexList nextvSet;
			for (size_t i = 0; i < vSet.size(); i++)
			{
				Vertex * v = vSet[i];

				OneRingVertex ring(v);
				Vertex * vv = NULL;
				while ((vv = ring.NextVertex()) != NULL)
				{
					if (!vv->visited)
					{
						vv->visited = true;
						nextvSet.push_back(vv);
						totalvList.push_back(vv);
					}
				}
			}
			vSet.clear();
			vSet.assign(nextvSet.begin(), nextvSet.end());
		}

		// floyd algorithm
		// for each vertex v1 in vSet, find the v2 = max{the geodesic distance(v1, v2)}, v2 belong to vSet
		size_t n = totalvList.size();
		for (size_t i = 0; i < n; i++)
		{
			totalvList[i]->SetTmpIndex(i);
		}

		vector <vector<int>> distance;
		for (size_t i = 0; i < n; i++)
		{
			vector<int> vdist;
			vdist.resize(n, 10000);
			vdist[i] = 0;
			Vertex * v1 = totalvList[i];
			OneRingVertex ring(v1);
			Vertex * v2 = NULL;
			while ((v2 = ring.NextVertex()) != NULL)
			{
				if (v2->visited)
					vdist[v1->TmpIndex(), v2->TmpIndex()] = 1;
			}
			distance.push_back(vdist);
		}

		for (size_t k = 0; k < n; k++)
		{
			for (size_t i = 0; i < n; i++)
			{
				for (size_t j = 0; j < n; j++)
				{
					if (i == j) continue;
					int tmpdist = distance[i][k] + distance[k][j];
					if (distance[i][j] > tmpdist)
					{
						distance[i][j] = tmpdist;
					}
				}
			}
		}
		
		// calc v1 and v2 euclidean distance
		for (size_t i = 0; i < vSet.size(); i++)
		{
			Vertex * v1 = vSet[i];
			int v1Idx = v1->TmpIndex();
			int maxDistIdx = -1;
			for (size_t j = 0; j < n; j++)
			{
				if (v1Idx == j) continue;
				if (maxDistIdx == -1 || distance[v1Idx][j] > distance[v1Idx][maxDistIdx])
				{
					maxDistIdx = j;
				}
			}

			Vertex * v2 = totalvList[maxDistIdx];
			float dist = (float)(v1->Position() - v2->Position()).L2Norm();
			if (dist < thDist)
			{
				u->cutting_point = true;
				break;
			}
		}

		// BFS
		VertexList tmpvList;
		FindKFieldFromVertex(u, field, tmpvList);
		for (size_t i = 0; i < tmpvList.size(); i++)
		{
			tmpvList[i]->sample_point = false;
		}
		if (u->cutting_point)
		{
			for (size_t i = 0; i < tmpvList.size(); i++)
			{
				if (tmpvList[i]->seed_flag)
					tmpvList[i]->cutting_point = true;
			}
		}
	}
}

void Mesh::SetVertexSimplifyLevel(bool labelBased, int BFSfield)
{
	int field = BFSfield;
	double thYCoord = 0.65*maxCoord[1] + 0.35*minCoord[1];
	VertexList tmpboundaryvList;
	if (!labelBased)
	{
		for (size_t i = 0; i < vList.size(); i++)
			vList[i]->visited = false;
		
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			HEdge * he = f->HalfEdge();
			do
			{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (adjcentFace != NULL)
				{
					if (adjcentFace->tmpLabel != f->tmpLabel)
					{
						Vertex * v1 = he->Start(), *v2 = he->End();
						if (!v1->visited)
						{
							v1->visited = true;
							tmpboundaryvList.push_back(v1);
						}
						if (!v2->visited)
						{
							v2->visited = true;
							tmpboundaryvList.push_back(v2);
						}
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}

		for (size_t i = 0; i < seedList.size(); i++)
		{
			Vertex *v = seedList[i];
			if (v->Position()[1] > thYCoord && !v->visited)
			{
				v->visited = true;
				tmpboundaryvList.push_back(v);
			}
		}


	}
	else
	{
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face *f = fList[i];
			HEdge * he = f->HalfEdge();
			do
			{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (adjcentFace!=NULL && f->Label() != adjcentFace->Label())
				{
					if (f->Label() == 0 || adjcentFace->Label() == 0)
					{
						Vertex * v[2] = { he->Start(), he->End() };
						for (int j = 0; j < 2; j++)
						{
							if (v[j]->visited == false)
							{
								v[j]->visited = true;
								tmpboundaryvList.push_back(v[j]);
							}
						}
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}
	}


	// BFS
	VertexList vSet, nextvSet;
	vSet.assign(tmpboundaryvList.begin(), tmpboundaryvList.end());
	int count = 0;
	while (!vSet.empty())
	{
		Vertex * u = vSet.back();
		vSet.pop_back();
		OneRingVertex ring(u);
		Vertex * v = NULL;
		while ((v = ring.NextVertex()) != NULL)
		{
			if (!v->visited)
			{
				v->visited = true;
				nextvSet.push_back(v);
				tmpboundaryvList.push_back(v);
			}
		}

		if (vSet.empty() && !nextvSet.empty())
		{
			count++;
			if (count > field)
				break;

			vSet.clear();
			vSet.assign(nextvSet.begin(), nextvSet.end());
		}
	}

	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex * v = vList[i];
		v->simplify_level = 0;
	}

	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		if (!labelBased && f->tmpLabel == 0) continue;
		if (labelBased && f->Label() == 0) continue;

		for (int j = 0; j < 3; j++)
			vList[f->v[j]]->simplify_level = 1;
	}

	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex * v = vList[i];
		if (v->Position()[1] > thYCoord)
		{
			v->simplify_level = 1;
		}
	}

	for (size_t i = 0; i < tmpboundaryvList.size(); i++)
	{
		tmpboundaryvList[i]->simplify_level = 2;
	}

}

void Mesh::BuildAngDist()
{
	if (heList.empty()) return;
	if (heList[0]->angDist > -0.001f) return;

	for (size_t i = 0; i < heList.size(); i++)
	{
		heList[i]->visited = false;
	}

	for (size_t i = 0; i < heList.size(); i++)
	{
		HEdge * he = heList[i];
		if (he->visited) continue;

		he->visited = true;
		he->Twin()->visited = true;

		Face * f1 = he->LeftFace();
		Face * f2 = he->Twin()->LeftFace();
		if (!f1 || !f2) continue;

		double dotValue = f1->normal.Dot(f2->normal);
		dotValue = dotValue>1.0 ? 1.0 : dotValue;
		dotValue = dotValue < -1.0 ? -1.0 : dotValue;

		he->angDist = he->Twin()->angDist = (float)(1 - dotValue);

		Vector3d c2c1 = f2->center - f1->center;
		c2c1.Normalize();
		if (c2c1.Dot(f1->normal) > 0)
		{
			he->convex = he->Twin()->convex = false;
		}
		else
		{
			he->convex = he->Twin()->convex = true;
		}
	}
}

void Mesh::ExportMaxConnectedComponent(vector<Vector3d> & mvList, vector<Vector3i> & mfList, vector<int> & mlList, vector<int> & mvbList)
{
	FindMaxConnectedComponet();

	for (size_t i = 0; i < vList.size(); i++)
	{
		vList[i]->SetFlag(false);
	}

	// find vertice
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->GroupID() == maxGroupID)
		{
			for (size_t j = 0; j < 3; j++)
			{
				vList[fList[i]->v[j]]->SetFlag(true);
			}
		}
	}

	// assign tmpIndx and export vertice
	int count = 0;
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->Flag())
		{
			vList[i]->SetTmpIndex(count++);
			mvList.push_back(vList[i]->Position());
			mvbList.push_back(vList[i]->LabelBoundary());
		}
	}

	// export faces and labels
	int v[3];
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->GroupID() == maxGroupID)
		{
			for (size_t j = 0; j < 3; j++)
			{
				v[j] = vList[fList[i]->v[j]]->TmpIndex();
			}
			mfList.push_back(Vector3i(v[0], v[1], v[2]));
			mlList.push_back(fList[i]->Label());
		}
	}
}

void Mesh::ExportMaxConnectedComponent2(vector<Vector3d> & mvList, vector<Vector3i> & mfList, vector<int> & mlList, vector<int> & msLevelList)
{
	FindMaxConnectedComponet();
	
	for (size_t i = 0; i < vList.size(); i++)
	{
		vList[i]->SetFlag(false);
	}

	// find vertice
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->GroupID() == maxGroupID)
		{
			for (size_t j = 0; j < 3; j++)
			{
				vList[fList[i]->v[j]]->SetFlag(true);
			}
		}
	}

	// assign tmpIndx and export vertice
	int count = 0;
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->Flag())
		{
			vList[i]->SetTmpIndex(count++);
			mvList.push_back(vList[i]->Position()); 
			msLevelList.push_back(vList[i]->simplify_level);
		}
	}

	// export faces and labels
	int v[3];
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i]->GroupID() == maxGroupID)
		{
			for (size_t j = 0; j < 3; j++)
			{
				v[j] = vList[fList[i]->v[j]]->TmpIndex();
			}
			mfList.push_back(Vector3i(v[0], v[1], v[2]));
			mlList.push_back(fList[i]->Label());
		}
	}
}

void Mesh::FindSmallAreaWithRLabel(vector<FaceList> & rLabelComponents)
{
	rLabelComponents.clear();
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->visited = false;
	}

	FindMaxConnectedComponet();
	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		if (f->GroupID() != maxGroupID) continue;
		if (f->visited) continue;

		// BFS
		FaceList tmpfList;
		tmpfList.push_back(f);
		f->visited = true;
		size_t  count = 0;
		while (count < tmpfList.size())
		{
			Face * nowFace = tmpfList[count];
			nowFace->tmpIdx = rLabelComponents.size();
			HEdge *he = nowFace->HalfEdge();
			do
			{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					if (!adjcentFace->visited && adjcentFace->r_label == nowFace->r_label)
					{
						tmpfList.push_back(adjcentFace);
						adjcentFace->visited = true;
					}
				}
				he = he->Next();
			} while (he != nowFace->HalfEdge());
			count++;
		}
		rLabelComponents.push_back(tmpfList);
	}
}

void Mesh::FindSmallAreaWithFuzzyLabel(vector<FaceList> & rLabelComponents)
{
	rLabelComponents.clear();
	for (size_t i = 0; i < fList.size(); i++)
	{
		fList[i]->visited = false;
	}

	FindMaxConnectedComponet();
	for (size_t i = 0; i < fList.size(); i++)
	{
		Face * f = fList[i];
		if (f->GroupID() != maxGroupID) continue;
		if (f->visited) continue;

		// BFS
		FaceList tmpfList;
		tmpfList.push_back(f);
		f->visited = true;
		size_t  count = 0;
		while (count < tmpfList.size())
		{
			Face * nowFace = tmpfList[count];
			HEdge *he = nowFace->HalfEdge();
			do
			{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					if (!adjcentFace->visited && adjcentFace->fuzzy_label == nowFace->fuzzy_label)
					{
						tmpfList.push_back(adjcentFace);
						adjcentFace->visited = true;
					}
				}
				he = he->Next();
			} while (he != nowFace->HalfEdge());
			count++;
		}
		rLabelComponents.push_back(tmpfList);

		//cout << rLabelComponents.size() << endl;
	}
}

}