/*
Class:					Tooth Segmentation
Author:					Xiaojie Xu
E-mail:					xuxj@shanghaitech.edu.cn
Date:					20170317

Description:			This class is the package of mesh simplification, boundary optimization,
						mapping between simplified mesh and original mesh, excluding mesh feature
						extraction.

Mesh Structure:			Bilateral Data Structures, supporting model file(*.stl, *.obj)

Mesh Simplification:
*/


#ifndef __TOOTH_SEGMENTATION_H__
#define __TOOTH_SEGMENTATION_H__

#ifdef WIN32
#include <windows.h>
#endif
#include <iostream>
#include <strstream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <map>

#include "../Mesh/mesh.h"
#include "../Mesh/matrix.h"
#include "../MeshDecimation/mdMeshDecimator.h"
#include "../maxflow-v3.01/graph.h"
//#include "opencv2/opencv.hpp"


using namespace std;
using namespace BiMesh;
//using namespace cv;

namespace MeshSegmentation
{
	// Declaration
	class SegmentationBaseClass;

	class SimplifyParameters;
	class MultiLabelGraphCutParameters;
	class ImprovedFuzzyClusteringParameters;	
	//class LocalFeatureParameters;
	class LeftRightSeparationParameters;
	class SplitJointedTeethParameters;
	class BoundarySmoothingParameters;
	

	class MeshSimplify;
	class MultiLabelGraphCut;
	class ImprovedFuzzyClustering;
	//class LocalFeatureGenerator;
	class PCAThreshold;
	class ToothPCAInfo;
	class LeftRightSeparation;
	class SplitJointedTeeth;
	class BoundarySmoothing;

	class ToothSegmentation;

	typedef vector<ToothPCAInfo> ToothPCAInfoList;

	// Segmentation Base Class
	class SegmentationBaseClass
	{
	protected:
		int rate = 1000;
		double CalcGeodesicDistance(HEdge * he)
		{
			Face * f[2] = { he->LeftFace(), he->Twin()->LeftFace() };
			if (f[0] == NULL || f[1] == NULL) return 0.0;
			Vector3d e = he->End()->Position() - he->Start()->Position();
			double len = e.L2Norm();
			double distance = 0;
			for (int i = 0; i < 2; i++)
			{
				Vector3d v = f[i]->center - he->Start()->Position();
				distance += v.Cross(e).L2Norm() / len;
			}

			return distance;
		}
		double CalcGeodesicDistance(Face * f, HEdge * he)
		{
			Vector3d e = he->End()->Position() - he->Start()->Position();
			double len = e.L2Norm();
			Vector3d v = f->center - he->Start()->Position();
			return v.Cross(e).L2Norm() / len;
		}
		double CalcGeodesicDistance(Face * f1, Face * f2, Vertex * v)
		{
			double distance = (f1->center - v->Position()).L2Norm() + (f2->center - v->Position()).L2Norm();

			return distance;
		}

		void BuildLabelMap(map<int, int> & labelMap, char type = 'U')
		{
			labelMap.clear();

			labelMap.insert(pair<int, int>(0, 0));
			if (type == 'U')
			{
				for (int i = 11; i <= 18; i++)
				{
					labelMap.insert(pair<int, int>(i, i - 10));
				}

				for (int i = 21; i <= 28; i++)
				{
					labelMap.insert(pair<int, int>(i, i - 12));
				}
			}
			else if (type = 'L')
			{
				for (int i = 31; i <= 38; i++)
				{
					labelMap.insert(pair<int, int>(i, i - 30));
				}

				for (int i = 41; i <= 48; i++)
				{
					labelMap.insert(pair<int, int>(i, i - 32));
				}
			}

		}

		void BuildAdjcentWeight(FaceList & fList, float lambda);
		void BuildAdjcentWeight(Mesh & mesh, float lambda);

		Graph<int, int, int> * UseMaxflow(FaceList & fList);

		SegmentationBaseClass(){};
		~SegmentationBaseClass(){};

	private:

	};

	// Mesh Simplification
	class SimplifyParameters
	{
	public:
		char modelType;			// U, or L model
		float d_ratio;
		vector<float> simplifyWeights;
		vector<float> graphCutWeights;
		float lambda, yita;
		bool labelBased;

		string trainModelExtension;
		string trainLabelExtension;
		string testModelExtension;
		string testLabelExtension;

		SimplifyParameters(){ ResetParameters(); };
		~SimplifyParameters(){};

		void ResetParameters()
		{
			modelType = 'U';		// 'L'
			d_ratio = 0.200f;
			simplifyWeights.clear();
			simplifyWeights = { 1.0f, 20.0f, 500.0f };
			graphCutWeights = { 0.35f, 0.55f, 0.1f };	// 0.35f, 0.5f, 0.15f
			lambda = 100.0f;
			yita = 0.05f;
			labelBased = false;

			trainModelExtension = "_train.obj";
			trainLabelExtension = "_train.txt";
			testModelExtension  = "_test.obj";
			testLabelExtension  = "_test.txt";
		}
		void PrintParameters()
		{
			cout << "*******************************" << endl;
			cout << endl;
			cout << "Simplify Parameters	" << endl;
			cout << "Simplify Ratio:		" << d_ratio << endl;
			cout << "Simplify Weights:	   ";
			for (size_t i = 0; i < simplifyWeights.size(); i++)
				cout << " " << simplifyWeights[i];
			cout << endl;
			cout << "Graph Cut(lambda):		" << lambda << endl;
			cout << "Fuzzy Clustering(yita):" << yita << endl;
			cout << "Label Based:			" << labelBased << endl;
			cout << endl;
			cout << "*******************************" << endl;
		}

	private:
		
	};

	class MeshSimplify:virtual protected SegmentationBaseClass
	{
	public:
		void Simplify(string modelName, SimplifyParameters & para);
		void Simplify(Mesh & inputMesh, Mesh & outputMesh, SimplifyParameters & para);
		
		MeshSimplify(){};
		~MeshSimplify(){};
	private:

		vector< MeshDecimation::Vec3<float> > decimatedPoints;
		vector< MeshDecimation::Vec3<int> > decimatedtriangles;
		vector< int > decimatedLabels;

		void Simplify(Mesh & mesh, SimplifyParameters & para);
		void RoughlyClassifiyTooth(Mesh & mesh, SimplifyParameters & para);
		

		bool SaveOBJ(string & fileName);
		void SaveLabel(string & filename);
	};

	// Muti-label Optimization: Using Alpha-Beta Swap Strategy
	class MultiLabelGraphCutParameters
	{
	public:
		float lambda;
		int iteration;
		bool optimzeGingiva;

		void ResetParameters()
		{
			lambda = 50;
			iteration = 3;
			optimzeGingiva = true;
		}

		MultiLabelGraphCutParameters(){ ResetParameters(); }
		~MultiLabelGraphCutParameters(){}

	private:

	};

	class MultiLabelGraphCut:virtual protected SegmentationBaseClass
	{
	public:
		void MultiLabelOptimizeBoundary(Mesh & mesh, MultiLabelGraphCutParameters & para);

		MultiLabelGraphCut(){};
		~MultiLabelGraphCut(){};

	protected:
		//void BuildAdjcentWeight(Mesh & mesh, float lambda);

	private:
		bool AlphaBetaSwap(FaceList & component, int a, int b);
	};

	// Improved Fuzzy Clustering
	class ImprovedFuzzyClusteringParameters
	{
	public:
		int propagate;
		float yita;
		float yitaAngDist;
		float geodesicSigma;
		int iteration;
		int optLabel;
		bool useAngDist;
		float angDistThreshold;
		float edgeAngDistTh;

		float basicProtectAreaRatio;

		float geodesicPropagateTh; 
		int protectPropagate;
		int roughPropagate;
		int basePropagate;
		int nextPropagate;
		vector<float> curvTh;

		void ResetParameters()
		{
			propagate = 10;
			yita = 0.05f;
			yitaAngDist = 0.1f;
			geodesicSigma = 0.5f;
			iteration = 1;
			optLabel = 1;

			useAngDist = true;
			angDistThreshold = 0.2f;	// 8 classes: 0.2f
			edgeAngDistTh = 0.08f;

			basicProtectAreaRatio = 0.5f;	// there are at least 50% area which are protected
			
			geodesicPropagateTh = 3.5f;	// 5 mm
			protectPropagate = 4;		//
			roughPropagate = 16;		// 8 classes: 16
			basePropagate = 4;			// 8 classes: 4
			nextPropagate = 12;			// 8 classes: 12
			curvTh = { -12.0f, -0.4f };
		}

		ImprovedFuzzyClusteringParameters(){ ResetParameters(); };
		~ImprovedFuzzyClusteringParameters(){};

	private:

	};

	class ImprovedFuzzyClustering:virtual protected SegmentationBaseClass
	{
	public:
		void FuzzyOptimizeBoundary(Mesh & mesh, ImprovedFuzzyClusteringParameters & para);
		void MultiLabelFuzzyOptimizeBoundary(Mesh & mesh, ImprovedFuzzyClusteringParameters & para);

		ImprovedFuzzyClustering(){};
		~ImprovedFuzzyClustering(){};

	private:
		void FindProtectedRegion(Mesh & mesh, ImprovedFuzzyClusteringParameters & para);
		void FindFuzzyRegion(FaceList & fuzzyFaceList, Mesh & mesh, ImprovedFuzzyClusteringParameters & para);
		void BoundaryBFSPropagate(Mesh & mesh, FaceList & bfList, FaceList & roughfList, vector<int> & roughRegion, ImprovedFuzzyClusteringParameters & para, bool useAngDist = false);
		int BuildAdjcentWeight_AngDist(FaceList & fuzzyFaceList, ImprovedFuzzyClusteringParameters & para);
		void BuildAssitWeight(FaceList & fuzzyFaceList, ImprovedFuzzyClusteringParameters & para);
	};

	// Create Local Photo Feature 
	//class LocalFeatureParameters
	//{
	//public:
	//	int height;
	//	int width;
	//	char type;
	//	vector<float> ratio; // 1 pixel = "ratio" mm 

	//	void ResetParameters() {
	//		height = 30;
	//		width = 30;
	//		type = 'U';
	//		ratio = { 0.5f, 1.0f, 1.5f };
	//	}

	//	LocalFeatureParameters() {
	//		ResetParameters();
	//	}
	//	~LocalFeatureParameters() {
	//	}

	//private:

	//};

	//class LocalFeatureGenerator
	//{
	//public:
	//	vector<vector<int>> localPhotoCenterlist;
	//	vector<Mat> fullSizeGradientList;
	//	void CreateLocalPhotoFeature(Mesh & mesh, LocalFeatureParameters & para);
	//	void SaveLocalPhotoFeature(string filename);

	//	LocalFeatureGenerator() {}
	//	~LocalFeatureGenerator() {}

	//private:
	//	const float Thresh_area = 0.01f;
	//	
	//	void CreateFullSizeGradient(Mesh & mesh, LocalFeatureParameters & para);
	//	void FaceSampling(vector<Vector3d>& clist, vector<Vector3d>& t, float& area);
	//	void ComputeLocalPhotoCenter(Mesh & mesh, LocalFeatureParameters & para);
	//};

	// Left Right Separation
	class LeftRightSeparationParameters
	{
	public:
		char modelType;
		float lambda;
		float maxWidth;
		float minProb;

		vector<float> widthThresholdList;

		bool load2ClassesProb;

		void ResetParameters()
		{
			modelType = 'U';
			lambda = 50.0f;
			maxWidth = 12.0f;
			minProb = 1e-8;

			// the first is U model's width threshold and the second is L model's width threshold
			widthThresholdList = { 13.0f, 6.0f }; 

			load2ClassesProb = true;
		}

		LeftRightSeparationParameters(){ ResetParameters(); };
		~LeftRightSeparationParameters(){};

	private:

	};

	class LeftRightSeparation:virtual protected SegmentationBaseClass
	{
	public:
		void JudgeLeftRight(Mesh & mesh, LeftRightSeparationParameters & para);

		LeftRightSeparation(){};
		~LeftRightSeparation(){};

	private:
		bool TwoCentralIncisors(FaceList & fList, Vector3d & center, Vector3d & dir, LeftRightSeparationParameters & para);
		void Assign17ClassesProb(Mesh & mesh, bool load2ClassesProb = true);

		void CountComponentsForEachLabel(Mesh & mesh, vector<int> & componentsForEachLabel);
	};



	// Split Jointed Teeth
	class SplitJointedTeethParameters
	{
	public:
		char modelType;		// 'U' or 'L': indicate the tooth's type
		char labelType;
		float lambda;
		double minProb;

		int neighborLabelDiffTh;
		float neighborDistTh;

		int maxLabelCount;

		int affectToothCountTh;

		vector<bool> prior;

		void ResetParameters()
		{
			modelType = 'U';
			labelType = 'f';
			lambda = 5.0f;
			minProb = 1e-8;  

			neighborLabelDiffTh = 2;	// Abs(the neighbor's label - this->label)<=neighborLabelDiffTh
			neighborDistTh = 3.0f;		// 3mm

			maxLabelCount = 8;			// the max tooth count is 8 for each side
		
			affectToothCountTh = 2;
		}
		void AssignPriorKnowledge(vector<bool> & priorTooth)
		{
			prior.assign(priorTooth.begin(), priorTooth.end());
		}

		SplitJointedTeethParameters(){ ResetParameters(); }
		~SplitJointedTeethParameters(){}

	private:
		

	};

	class PCAThreshold
	{
	public:
		int label;
		//float maxAxisLength;
		float maxProjectToXYPlaneLength;
		float minArea;
		float maxBoundaryPCALength;

		PCAThreshold(){}
		PCAThreshold(int & label, float & maxProjectToXYPlaneLength, float & minArea, float & maxBoundaryPCALength) :label(label), maxProjectToXYPlaneLength(maxProjectToXYPlaneLength), minArea(minArea), maxBoundaryPCALength(maxBoundaryPCALength){}
		~PCAThreshold(){}
	private:
	};

	class ToothPCAInfo:virtual protected SegmentationBaseClass
	{
	public:
		FaceList fList;
		vector<double> varList;
		vector<Vector3d> axisList;
		vector<double> lengthList;
		double projectToXYPlaneLength;
		int projectTOXYPlaneAxisIdx;
		double maxBoundaryPCALengthWithOtherTooth;

		Vector3d center;
		Vector3d obbCenter;
		float area = 0.0f;

		int idx;		// range: 0~16; gum: 0; tooth: 1~16;
		int label;		// gum: 0; U model: 11~18, 21~28; L model: 31~38, 41~48;

		// save splited tooth
		vector<int> tmpLabelList;
		vector<FaceList> tmpfList;
		vector<int> bestLabelList;

		// adjcent tooth
		vector<ToothPCAInfo*> directAdjcentToothList;
		vector<ToothPCAInfo*> adjcentToothList;
		

		ToothPCAInfo(){}
		~ToothPCAInfo(){}

		void PCAAnalysis(map<int, int> & labelMap);
		void CalcLengthForEachAxis();
		void CalcArea();
		void CalcCenter()
		{
			center = Vector3d(0, 0, 0);
			if (fList.empty()) return;

			for (size_t i = 0; i < fList.size(); i++)
			{
				center += fList[i]->center;
			}
			center /= fList.size();
		}

		double BoundaryPCAAnalysisWithOtherTooth(Mesh & mesh, FaceList & fList1, FaceList & fList2);

		void FindAdjcentTooth(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		bool SplitJointedTeeth(Mesh & mesh, vector<PCAThreshold> & thresholdList, SplitJointedTeethParameters & para);
		void CorrectLabel(Mesh & mesh, ToothPCAInfoList & toothPCAList, vector<PCAThreshold> & thresholdList);

		int AffectToothAnalysis(ToothPCAInfoList & toothPCAList, map<int, int> & labelMap);
		
	private:
		

	};

	class SplitJointedTeeth:virtual protected SegmentationBaseClass
	{
	public:
		void BuildToothPCAList(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		void ToothPCAAnalysis(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		void DetectJointedTeeth(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		void DetectJointedTeethWithPrior(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		void ToothBoundaryPCAAnalysis(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);

		bool LoadPCAData(string filename);
		bool SavePCAInfo(string filename, ToothPCAInfoList & toothPCAList); 

		SplitJointedTeeth(){}
		~SplitJointedTeeth(){}

	private:
		map<int, int> labelMap;
		vector<PCAThreshold> thresholdList;
		int minCost;

		void CorrectTheSecondPremolares(Mesh & mesh, SplitJointedTeethParameters & para);
		
		void SearchBestLabelling(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		int CalcLabellingCost(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para);
		int CalcLabellingCostForEachTooth(ToothPCAInfo & toothPCA, SplitJointedTeethParameters & para);
		void AssignLabellingPlan(ToothPCAInfoList & toothPCAList);
		void SearchLabelling(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para, int toothID, int nowLabel, int cost);
		int FindPreviousFeasibleLabel(int nowLabel, int startLabel, SplitJointedTeethParameters & para);
		int FindNextFeasibleLabel(int startLabel, SplitJointedTeethParameters & para);
	};


	// Boundary Smoothing
	class BoundarySmoothingParameters
	{
	public:
		int BumpSmoothingIteration;
		int ShortestOptIteration;

		vector<double> possibleBoundaryTh;
		vector<int> possibleBoundaryField;
		int boundaryOptMinSize;
		int label;

		bool useSampling;
		int samplingInterval;
		int minSamplingInterval;

		double yita, lambda;
		double angDistTh;

		

		void ResetParameters()
		{
			BumpSmoothingIteration = 3;
			ShortestOptIteration = 1;
			
			boundaryOptMinSize = 20;

			useSampling = false;
			samplingInterval = 10;
			minSamplingInterval = 5;

			//yita = 1.2;
			//lambda = 0.05;
			//angDistTh = 0.05;

			yita = 4.0;
			lambda = 0.02;
			angDistTh = 0.08;

			possibleBoundaryTh = { -0.85, -0.3};
			possibleBoundaryField = { 1, 2 };
		}


		BoundarySmoothingParameters(){ ResetParameters(); }
		~BoundarySmoothingParameters(){}

	private:

	};

	class BoundarySmoothing
	{
	public:
		void SmoothBoundary(Mesh & mesh, BoundarySmoothingParameters & para);

		BoundarySmoothing(){}
		~BoundarySmoothing(){}

	private:
		void SmoothBump(Mesh & mesh);
		void ShortestRouteOptimzation(Mesh & mesh, BoundarySmoothingParameters & para);
		void FindBoundary(Mesh & mesh, vector<VertexList> & boundaries, int label, BoundarySmoothingParameters & para);
		void SingleBoundaryShortestRouteOptimization(Mesh & mesh, VertexList & boundary, BoundarySmoothingParameters & para);
		void SetPossibleBoundary(Mesh & mesh, Vertex * v, int field, VertexList & possibleBoundary);
		
		void BoundarySampling(VertexList & boundary, vector<VertexList> & subBoundaries, BoundarySmoothingParameters & para);

		void FindStartvList(VertexList & startvList, VertexList & totalvList, Vertex * u, BoundarySmoothingParameters & para);
		bool CreateEndvList(VertexList & startvList, VertexList & endvList, VertexList & boundary, VertexList & possibleBoundary);
		void CreateAdjacentMap(VertexList & startvList, VertexList & endvList, VertexList & totalvList, VertexList & boundary, BoundarySmoothingParameters & para);
		void FindAShortestPathToLabel(VertexList & shortestvList, VertexList & totalvList, Vertex * u, int label, bool labelOrNonLabel = true);
		void FindShortestBoundary(VertexList & startvList, VertexList & endvList, VertexList & possibleBoundary, VertexList & optBoundaryvList);
		void AssignNewLabel(Mesh & mesh, VertexList & optBoundaryvList, VertexList & possibleBoundary, BoundarySmoothingParameters & para);
		double CalcEdgeDistance(HEdge * he, BoundarySmoothingParameters & para);
		bool IsCycle(VertexList & boundary);
		double IsPossibleBoundary(VertexList & boundary, int idx, bool cycleFlag = false);
	};


	class ToothSegmentation :public MeshSimplify, public MultiLabelGraphCut, public ImprovedFuzzyClustering,public LeftRightSeparation, public SplitJointedTeeth, public BoundarySmoothing
	{
	public:	
		void RefineClass2(Mesh & simplifiedMesh);
		void RefineClass8(Mesh & simplifiedMesh, Mesh & originalMesh, char type = 'U');

		void MeshANNMapping(Mesh & src, Mesh & dst, float addProb = 2.0f);
		void FillGapBetweenTeeth(Mesh & mesh);
		void RemoveSmallComponent(Mesh & mesh, int iteration = 3, float protectedAreaTh = 50.0f, float RemovedAreaTh = 15.0f);
		

		ToothSegmentation(){};
		~ToothSegmentation(){};

	private:
		


	};



}

#endif