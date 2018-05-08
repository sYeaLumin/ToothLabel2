/**
* Mesh 
*
* using half-edge data structure
*/

#ifndef __MESH_H__
#define __MESH_H__

#include <cstdlib>
#include <vector>
#include "vector3.h"

#define infty 2e203
#ifndef PI
#define PI 3.1415926535898
#endif

namespace BiMesh
{
	// classes
	class HEdge;
	class Vertex;
	class Face;
	class Mesh;
	class OneRingHEdge;
	class OneRingVertex;
	class FaceEnumeration;
	class STLVeterx;

	// types
	typedef std::vector<HEdge*> HEdgeList;
	typedef std::vector<Vertex*> VertexList;
	typedef std::vector<Face*> FaceList;

	// other helper functions
	inline void SetPrevNext(HEdge *e1, HEdge *e2);
	inline void SetTwin(HEdge *e1, HEdge *e2);
	inline void SetFace(Face *f, HEdge *e);

	////////// class HEdge //////////
	class HEdge
	{
	private:
		HEdge *twin, *prev, *next;
		Vertex *start;
		Face *face;
		bool boundary;
		bool flag;	// for count boundary loop use ...

	public:
		float angDist = -100000.0f;
		bool convex = true;
		bool visited = false;


		HEdge(bool b = false)
		{
			boundary = b;
			twin = prev = next = NULL;
			start = NULL;
			face = NULL;
			flag = false;
		}

		HEdge*  Twin() const { return twin; }
		HEdge*  Prev() const { return prev; }
		HEdge*  Next() const { return next; }
		Vertex* Start() const { return start; }
		Vertex* End() const { return next->start; } // for convenience
		Face*   LeftFace() const { return face; }
		bool    Flag() const { return flag; }

		HEdge*  SetTwin(HEdge* e) { return twin = e; }
		HEdge*  SetPrev(HEdge* e) { return prev = e; }
		HEdge*  SetNext(HEdge* e) { return next = e; }
		Vertex* SetStart(Vertex* v) { return start = v; }
		Face*   SetFace(Face* f) { return face = f; }
		bool    SetFlag(bool b) { return flag = b; }
		bool    IsBoundary() const { return boundary; }

		// get the Euclidean length of this edge
		inline double getLength();
	};


	////////// class OneRingHEdge //////////
	class OneRingHEdge
	{
	private:
		HEdge *start, *next;
	public:
		OneRingHEdge(const Vertex * v);
		HEdge * NextHEdge();
	};


	////////// class OneRingVertex //////////
	class OneRingVertex
	{
	private:
		OneRingHEdge ring;
	public:
		OneRingVertex(const Vertex * v) : ring(v) { }
		Vertex * NextVertex() { HEdge *he = ring.NextHEdge(); return (he) ? he->End() : NULL; }
	};

	////////// class Vertex //////////
	class Vertex
	{
	private:
		Vector3d position;
		Vector3d normal;
		Vector3d color;
		HEdge *he;
		int index;
		int flag;

	public:
		int tmpIndex;

		vector<HEdge*> adjHEdges; // for reading object only ....
		float maxCurv = 0, minCurv = 0;
		Vector3f maxCurvDir, minCurvDir;
		
		int boundaryLevel = 0;
		int simplify_level = -1;
		bool cancave_flag = false;


		int source_weight, sink_weight;

		bool visited = false;
		bool seed_flag = false;
		bool protect_flag = false;
		bool cutting_point = false;
		int local_heightest = -1;	// 0 is the local heightest point
		bool sample_point = false;

		int labelBoundary, possibleBoundary;
		VertexList adjacentvList;
		vector<double> adjacentDist;

		Vertex() : he(NULL), flag(0), labelBoundary(0){ }
		Vertex(const Vector3d & v) : he(NULL), position(v), flag(0), labelBoundary(0) { }
		Vertex(double x, double y, double z) : he(NULL), position(x, y, z), flag(0), labelBoundary(0) { }

		const Vector3d & Position() const { return position; }
		const Vector3d & Normal() const { return normal; }
		const Vector3d & Color() const { return color; }
		HEdge * HalfEdge() const { return he; }
		int Index() const { return index; }
		int TmpIndex() const { return tmpIndex; }
		int LabelBoundary() const { return labelBoundary; }
		int Flag() const { return flag; }
		const Vector3d & SetPosition(const Vector3d & p) { return position = p; }
		const Vector3d & SetNormal(const Vector3d & n) { return normal = n; }
		const Vector3d & SetColor(const Vector3d & c) { return color = c; }
		HEdge * SetHalfEdge(HEdge * he) { return Vertex::he = he; }

		int SetIndex(int index) { return Vertex::index = index; }
		int SetTmpIndex(int index) { return Vertex::tmpIndex = index; }
		int SetLabelBoundary(int value) { return Vertex::labelBoundary = value; }
		int SetFlag(int value) { return flag = value; }
		bool IsBoundary() const
		{
			OneRingHEdge ring(this);
			HEdge *curr = NULL;
			while (curr = ring.NextHEdge())
				if (curr->IsBoundary())
					return true;
			return false;
		}

		int Valence() const
		{
			int count = 0;
			OneRingVertex ring(this);
			Vertex *curr = NULL;
			while (curr = ring.NextVertex()) count++;
			return count;
		}
	};

	////////// class Face //////////
	class Face
	{
	private:
		HEdge * he;
		int groupID;
		int labelPath, label;
		Vector3d color;

	public:
		int v[3];
		Vector3d center, normal, sphericalCoord;
		double area;

		int p_label = 0, r_label = 0, fuzzy_label;
		vector<float> prob;
		vector<float> blend_prob;
		vector<float> twoClassesProb;

		vector<int> p_label_list;
		vector<int> r_label_list;
		vector<int> f_label_list;

		float tmpDist = 0.0f;
		float tmpAngDist = 0.0f;
		float tmpGeoDist = 0.0f;

		bool protect = false;
		bool fuzzyToothBoundary = false;
		bool labelBoundary = false;		// label is different from the around faces
		bool visited = false;
		int flag;

		// source_weight = - log(prob); sink_weight = - log(1-prob);
		int tmpIdx, tmpLabel;
		int source_weight, sink_weight;
		FaceList adjcent_face;
		vector<int> adjcent_weight;
		vector<float> assist_weight;
		vector<int> dCost;
		float tmpGeodesicDistance = 1000;

		int simplfiyLabel = 0;
		int simplifyMapping = 0;
		int bubbleNoiseLabel = 0;//ÆøÅÝ±êÇ©
		int bubbleNoiseLabel2 = 0;

		Face() : he(NULL) { }

		int LabelPath() const { return labelPath; }
		int Label() const { return label; }
		int GroupID() const { return groupID; }
		HEdge * HalfEdge() const { return he; }
		const Vector3d & Color() const { return color; }

		int SetLabelPath(int l) { return labelPath = l; }
		int SetLabel(int l) { return label = l; }
		int SetGroupID(int l) { return groupID = l; }
		HEdge * SetHalfEdge(HEdge * he) { return Face::he = he; }
		const Vector3d & SetColor(const Vector3d & c) { return color = c; }


		// calculate sphere coords
		// this calculation is a bit different from tradictional mapping
		// spherical coords: (lou, theta, phi)
		// x = lou*sin(theta)*sin(phi)
		// y = -lou*sin(theta)*cos(phi)
		// z = lou*cos(phi)
		void CalculateSphericalCoord(const Vector3d & c)
		{
			Vector3d xyz = center - c;
			sphericalCoord[0] = xyz.L2Norm();
			if (sphericalCoord[0] > 0)
			{
				sphericalCoord[1] = acos(xyz[2] / sphericalCoord[0]);
				double denominater = sphericalCoord[0] * sin(sphericalCoord[1]);
				if (denominater > 0)
				{
					sphericalCoord[2] = asin(xyz[0] / denominater);
					if (xyz[1] > 0)
					{
						sphericalCoord[2] = xyz[0] > 0 ? PI - sphericalCoord[2] : -PI - sphericalCoord[2];
					}

				}
			}
		}

		bool IsBoundary() {
			HEdge *curr = he;
			do {
				if (curr->Twin()->IsBoundary()) return true;
				curr = curr->Next();
			} while (curr != he);
			return false;
		}
	};

	////////// class STLVertex //////////
	class STLVertex
	{
	public:
		float x, y, z;
		int stlID, objID;

		STLVertex(){}
		STLVertex(float & x, float & y, float & z, int & stlID) :x(x), y(y), z(z), stlID(stlID){}
		~STLVertex(){}

	private:

	};


	class Mesh
	{
	public:
		HEdgeList heList;
		HEdgeList bheList;
		VertexList vList;
		FaceList fList;

		// local frame
		VertexList frameList;
		vector<int> groupCount;
		int maxGroupID = -1;

		// vertex curvatures and directions
		vector<Vector3f> pdir1, pdir2;
		vector<float> curv1, curv2;
		vector<Vector3f> normals;
		vector<Vector3f> cornerareas;
		vector<float> pointareas;

		int localBFSLevel;
		Vector3d maxCoord, minCoord;	// x,y,z, max, min
		VertexList seedList;

		Mesh() { }
		~Mesh() {
			Clear();
		}

		const HEdgeList Edges() const { return heList; }
		const HEdgeList BoundaryEdges() const { return bheList; }
		const VertexList Vertices() const { return vList; }
		const FaceList Faces() const { return fList; }

		void AddVertex(Vertex *v) { vList.push_back(v); }
		void AddFace(int v1, int v2, int v3);
		void Clear() {
			size_t i;
			for (i = 0; i < heList.size(); i++) delete heList[i];
			for (i = 0; i < bheList.size(); i++) delete bheList[i];
			for (i = 0; i < vList.size(); i++) delete vList[i];
			for (i = 0; i < fList.size(); i++) delete fList[i];
			heList.clear();
			bheList.clear();
			vList.clear();
			fList.clear();

			curv1.clear();
			curv2.clear();
			pdir1.clear();
			pdir2.clear();
			cornerareas.clear();
			pointareas.clear();
			normals.clear();
			seedList.clear();
		}

		Vector3d MinCoord() const
		{
			Vector3d minCoord(1e20);
			for (size_t i = 0; i < vList.size(); i++)
				minCoord = minCoord.Min((vList[i])->Position());
			return minCoord;
		}

		Vector3d MaxCoord() const
		{
			Vector3d maxCoord(-1e20);
			for (size_t i = 0; i < vList.size(); i++)
				maxCoord = maxCoord.Max(vList[i]->Position());
			return maxCoord;
		}

		bool LoadModel(const char * filename);
		bool LoadObjFile(const char * filename);
		bool WriteObjFile(const char * filename);
		bool LoadSTLFile(const char * filename);
		bool LoadLabels(const char * filename);
		bool LoadVectorData(vector<Vector3d> & points, vector<Vector3i> & faces);
		

		// Set or Get label
		bool SetGroundTruthLables(vector<int> & labels);
		void GetGroundTruthLabels(vector<int> & labels);

		bool SetPredictLabelsLabelsProbabilities(vector<int> & labels, vector<vector<float>> probList);
		void GetPredictLabelsProbabilities(vector<int> & labels, vector<vector<float>> probList);

		bool SetTwoClassProbabilities(vector<vector<float>> & probList);
		void GetR_labels(vector<int> & labels);
		void GetF_labels(vector<int> & labels);


		//bool SaveObjFile(const char * filename);
		void DisplayMeshInfo();
		void ComputeVertexCurvatures();
		void UmbrellaSmooth();
		void ImplicitUmbrellaSmooth();

		///////////////////////////////////////////////////////
		// vertex normal estimation
		///////////////////////////////////////////////////////	
		void ComputeVertexNormals();

		// estimate smooth vertex normal for all vertices
		void computeSmoothVertexNormals(VertexList ver_list, double radius);

		// smooth normal based on estimated geodesic distance	
		// using Dijkstra's Algorithm
		Vector3d computeSmoothVertexNormal(Vertex* center, double radius);


		// additional helper functions
		int CountBoundaryLoops();
		int CountConnectedComponents();
		int FindMaxConnectedComponet();
		void SaveMaxConnectedComponet(const char * filename);
		void GroupingVertexFlags();
		void FindLabelBoundaryVertex();
		void BFSFromBoundaryVertex();
		void FindLabelBoundaryFace(int propagate = 0);
		void FindLabelBoundaryFace2(int propagate = 0);
		void FindLabelBoundaryFace4(int propagate = 0);
		void FindLabelBoundaryFace8(int propagate = 0);
		void FindLabelBoundaryFace16(int propagate = 0);
		void FindToothGumBoundary(int propagate = 0);
		void FindGumFacesAroundTooth(FaceList & gumfList, vector<int> & toothIDList, int propagate = 15);
		void FindClassificationRefineBoundary(int level = 0, int propagate = 0);
		void FindClassificationRefineBoundaryCmd(int propagate, int optLabel = 1);
		void FindClassificationRefineBoundaryCmd(int propagate);
		void FindFuzzyBoundary(FaceList & bfList, int propagate, int optLabel = 1);
		void FindFuzzyToothBoundary();
		void FindFuzzyBoundaryAngDist(FaceList & bfList, int propagate, float angDistTh, float yita, int optLabel = 1);
		void ExportToothCoordsRange(int id);
		void FindCancavePoints();
		void CalculateSimplifiyLevel(
			char modelType = 'U',
			float seedRange1 = -14.0f, float seedRange2=-0.4f,
			int isolatedRange=1000, int iterationCount=2, int field=2,
			int localHeightest1=12, int localHeightest2=6,
			int protect1=2, int protect2=10,
			int cuttingField=5, float cuttingTh=1.2f);
		void SetVertexSimplifyLevel(bool labelBased = true, int BFSfield = 4);
		
		void BuildAngDist();


		// Vertex List: mvList;		Face List: mfList;		Label List: mlList;
		void ExportMaxConnectedComponent(vector<Vector3d> & mvList, vector<Vector3i> & mfList, vector<int> & mlList, vector<int> & mvbList);
		void ExportMaxConnectedComponent2(vector<Vector3d> & mvList, vector<Vector3i> & mfList, vector<int> & mlList, vector<int> & msLevelList);

		void FindSmallAreaWithRLabel(vector<FaceList> & rLabelComponents);
		void FindSmallAreaWithFuzzyLabel(vector<FaceList> & rLabelComponents);
		// Compute principal curvatures and directions.
		void need_curvatures();


		void FindKFieldFromVertex(Vertex * u, int field, VertexList & tmpvList);

	private:
		// load stl file
		bool ReadSTLASCII(const char *cfilename);
		bool ReadSTLBinary(const char *cfilename);
		void STL2OBJ(vector<STLVertex> & xyzList);
		void CalculateSphericalCoords();

		void need_normals();
		void need_pointareas();
		void proj_curv(const Vector3f &old_u, const Vector3f &old_v,
			float old_ku, float old_kuv, float old_kv,
			const Vector3f &new_u, const Vector3f &new_v,
			float &new_ku, float &new_kuv, float &new_kv);
		void diagonalize_curv(const Vector3f &old_u, const Vector3f &old_v,
			float ku, float kuv, float kv,
			const Vector3f &new_norm,
			Vector3f &pdir1, Vector3f &pdir2, float &k1, float &k2);


		void OpenOperation(VertexList & seedList, int field);	// first Eroding, then Dilating
		void CloseOperation(VertexList & seedList, int field);	// first Dilating, then Eroding
		bool ErodingOperation(Vertex * u, int field);
		void DilatingOperation(Vertex * u, int field, VertexList & dilatingList);
		void UpdateSeedList(VertexList & seedList);
		
		bool IsLocalHeightest(char modelType, Vertex * u, Vertex * v);
		void FindLocalHeightestPoints(char modelType, int frontfield, int backfield, VertexList & LHvList);
		void FindProtectPoints(int frontfield, int backfield, VertexList & LHvList);
		void FindCuttingPoints(int field, VertexList & cuttingList, float thDist=1.2f);
	};

	// other helper functions
	inline void SetPrevNext(HEdge *e1, HEdge *e2) { e1->SetNext(e2); e2->SetPrev(e1); }
	inline void SetTwin(HEdge *e1, HEdge *e2) { e1->SetTwin(e2); e2->SetTwin(e1); }
	inline void SetFace(Face *f, HEdge *e) { f->SetHalfEdge(e); e->SetFace(f); }


}
#endif // __MESH_H__
