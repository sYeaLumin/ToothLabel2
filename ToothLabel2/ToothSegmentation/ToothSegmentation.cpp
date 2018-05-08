#include "ToothSegmentation.h"
#include "../ann_1.1.2/ANN/ANN.h"

namespace MeshSegmentation
{
	void SegmentationBaseClass::BuildAdjcentWeight(FaceList & fList, float lambda)
	{
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face *f = fList[i];
			f->adjcent_face.clear();
			f->adjcent_weight.clear();
		}

		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			HEdge * he = f->HalfEdge();
			do{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					f->adjcent_face.push_back(adjcentFace);
					double dotValue = f->normal.Dot(adjcentFace->normal);
					dotValue = dotValue>1.0 ? 1.0 : dotValue;
					dotValue = dotValue < -1.0 ? -1.0 : dotValue;

					double dihedral = acos(dotValue) + 1e-8;
					//double distance = (f->center - adjcentFace->center).L2Norm();
					double distance = CalcGeodesicDistance(he);

					double cost = rate*lambda*(-log(dihedral / PI)*distance);

					Vector3d c2c1 = adjcentFace->center - f->center;
					c2c1.Normalize();
					if (c2c1.Dot(f->normal) < 0)
						cost *= (1 + abs(dotValue));

					cost = cost > 1e9 ? 1e9 : cost;
					f->adjcent_weight.push_back((int)cost);
				}

				he = he->Next();
			} while (he != f->HalfEdge());
		}
	}

	void SegmentationBaseClass::BuildAdjcentWeight(Mesh & mesh, float lambda)
	{
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face *f = mesh.fList[i];
			f->adjcent_face.clear();
			f->adjcent_weight.clear();
		}

		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			HEdge * he = f->HalfEdge();
			do{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					f->adjcent_face.push_back(adjcentFace);
					double dotValue = f->normal.Dot(adjcentFace->normal);
					dotValue = dotValue>1.0 ? 1.0 : dotValue;
					dotValue = dotValue < -1.0 ? -1.0 : dotValue;

					double dihedral = acos(dotValue) + 1e-8;
					//double distance = (f->center - adjcentFace->center).L2Norm();
					double distance = CalcGeodesicDistance(he);

					double cost = rate*lambda*(-log(dihedral / PI)*distance);

					Vector3d c2c1 = adjcentFace->center - f->center;
					c2c1.Normalize();
					if (c2c1.Dot(f->normal) < 0)
						cost *= (1 + abs(dotValue));	// if the edge is convex

					cost = max(1e-10, cost);
					cost = min(1e10, cost);
					f->adjcent_weight.push_back((int)cost);
				}

				he = he->Next();
			} while (he != f->HalfEdge());
		}
	}

	Graph<int, int, int> * SegmentationBaseClass::UseMaxflow(FaceList & fList)
	{
		int edgeCount = 0;
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			for (size_t j = 0; j < f->adjcent_face.size(); j++)
			{
				if (f->adjcent_face[j]->tmpIdx >= 0 && f->adjcent_face[j]->tmpIdx > f->tmpIdx)
					edgeCount++;
			}
		}

		Graph<int, int, int> *maxflowPtr = new Graph<int, int, int>((int)fList.size(), edgeCount, NULL);

		// add_node
		maxflowPtr->add_node((int)fList.size());

		// add_edge
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f1 = fList[i];
			for (size_t j = 0; j < f1->adjcent_face.size(); j++)
			{
				Face * f2 = f1->adjcent_face[j];
				if (f2->tmpIdx >= 0 && f1->tmpIdx < f2->tmpIdx)
				{
					maxflowPtr->add_edge(f1->tmpIdx, f2->tmpIdx, f1->adjcent_weight[j], f1->adjcent_weight[j]);
				}
			}
		}

		// add_tweight
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];

			maxflowPtr->add_tweights(f->tmpIdx, f->source_weight, f->sink_weight);
		}

		// maxflow
		maxflowPtr->maxflow(false, NULL);

		return maxflowPtr;
	}


	void ToothSegmentation::RefineClass2(Mesh & simplifiedMesh)
	{
		MultiLabelGraphCutParameters mlgcp;
		mlgcp.ResetParameters();
		mlgcp.lambda = 20;	
		MultiLabelOptimizeBoundary(simplifiedMesh, mlgcp);
	}
	void ToothSegmentation::RefineClass8(Mesh & simplifiedMesh, Mesh & originalMesh, char type)
	{
		ToothPCAInfoList toothPCAList;
		MultiLabelGraphCutParameters mlgcp;
		ImprovedFuzzyClusteringParameters ifcp;
		LeftRightSeparationParameters lrsp;
		SplitJointedTeethParameters sjtp;
		BoundarySmoothingParameters bsp;

		// set type 'U' or 'L'
		lrsp.modelType = type;
		sjtp.modelType = type;

		// step 1: multi-label graph cut in the simplified mesh
		// the mlgcp.lambda can be changed
		mlgcp.ResetParameters();
		if (type == 'U')
		{
			mlgcp.lambda = 100; // U model
		}
		else
		{
			mlgcp.lambda = 50; // L model
		}
		MultiLabelOptimizeBoundary(simplifiedMesh, mlgcp);

		// step 2: judge left and right
		JudgeLeftRight(simplifiedMesh, lrsp);
		RemoveSmallComponent(simplifiedMesh);

		// step 3: detect jointed teeth
		DetectJointedTeeth(simplifiedMesh, toothPCAList, sjtp);

		// step 4: label mapping from simlified mesh to original mesh
		MeshANNMapping(originalMesh, simplifiedMesh);
		mlgcp.ResetParameters();
		mlgcp.lambda = 50;
		
		// step 5: multi-label graph cut in the original mesh
		MultiLabelOptimizeBoundary(originalMesh, mlgcp);

		// step 6: boundary smoothing
		ifcp.angDistThreshold = 0.3f;
		ifcp.yitaAngDist = 0.2f;
		ifcp.yita = 0.1f;

		MultiLabelFuzzyOptimizeBoundary(originalMesh, ifcp);
		FillGapBetweenTeeth(originalMesh);
		RemoveSmallComponent(originalMesh, 3, 500.0f);

		SmoothBoundary(originalMesh, bsp);
		RemoveSmallComponent(originalMesh, 3, 500.0f);
	}


	// Find the approximate nearest face in dst mesh for each face in src mesh
	// assign the refine label from dst mesh to src mesh
	// query: src
	// data : dst
	// addProb: in order to respect the refine result in simplified model, we add additional probility in the prob list.
	void ToothSegmentation::MeshANNMapping(Mesh & src, Mesh & dst, float addProb)
	{
		if (dst.fList.size() == 0) return;
		if (dst.fList[0]->prob.size() == 0) return;
		cout << "label mapping" << endl;
		dst.FindClassificationRefineBoundaryCmd(3, -1);
		int	k = 1;				// number of nearest neighbors
		int dim = 3;			// dimension
		double	eps = 0.1;		// error bound
		int maxPts = 1000000;  // maximum number of data points
		int					nPts = (int)dst.fList.size();					// actual number of data points
		ANNpointArray		dataPts;				// data points
		ANNpoint			queryPt;				// query point
		ANNidxArray			nnIdx;					// near neighbor indices
		ANNdistArray		dists;					// near neighbor distances
		ANNkd_tree*			kdTree;					// search structure

		queryPt = annAllocPt(dim);					// allocate query point
		dataPts = annAllocPts(maxPts, dim);			// allocate data points
		nnIdx = new ANNidx[k];						// allocate near neigh indices
		dists = new ANNdist[k];						// allocate near neighbor dists
		
		//cout << "src: " << src.fList.size() << endl;
		//cout << "dst: " << dst.fList.size() << endl;
		// set up data
		for (size_t i = 0; i < dst.fList.size(); i++)
		{
			for (int j = 0; j < dim; j++)
			{
				dataPts[i][j] = dst.fList[i]->center[j];
			}
		}

		// set up kd-tree
		kdTree = new ANNkd_tree(		// build search structure
			dataPts,					// the data points
			nPts,						// number of points
			dim);						// dimension of space

		// query
		src.FindMaxConnectedComponet();
		//cout << src.maxGroupID << endl;
		for (size_t i = 0; i < src.fList.size(); i++)
		{
			Face * f = src.fList[i];
			if (f->GroupID() != src.maxGroupID) continue;
			for (int j = 0; j < dim; j++)
			{
				queryPt[j] = f->center[j];
			}
			
			kdTree->annkSearch(					// search
				queryPt,						// query point
				k,								// number of near neighbors
				nnIdx,							// nearest neighbors (returned)
				dists,							// distance (returned)
				eps);							// error bound

			f->simplifyMapping = nnIdx[0];
			Face * dstFace = dst.fList[f->simplifyMapping];
			f->p_label = dstFace->fuzzy_label;
			f->r_label = f->p_label;

			f->prob.assign(dstFace->prob.begin(), dstFace->prob.end());
			if (!dstFace->labelBoundary)
				f->prob[dstFace->fuzzy_label] += addProb;
			
			float sumProb = 0.0f;
			for (size_t j = 0; j < f->prob.size(); j++)
			{
				sumProb += f->prob[j];
			}
			if (sumProb > 0.0f)
			{
				for (size_t j = 0; j < f->prob.size(); j++)
				{
					f->prob[j] /= sumProb;
				}
			}
		}

		
		delete[] nnIdx;							// clean things up
		delete[] dists;
		delete kdTree;
		annClose();
	}

	void ToothSegmentation::FillGapBetweenTeeth(Mesh & mesh)
	{
		FaceList tmpfList;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->r_label>0 && f->fuzzy_label == 0)
			{
				vector<int> aroundLabelList;
				vector<int>::iterator iter;
				// find two field adjcent face around f
				for (int j = 0; j < 3; j++)
				{
					Vertex * u = mesh.vList[f->v[j]];
					OneRingHEdge ring(u);
					HEdge * he = NULL;
					while ((he = ring.NextHEdge()) != NULL)
					{
						Face * adjcentFace = he->LeftFace();
						if (adjcentFace == NULL) continue;
						if (adjcentFace->fuzzy_label == 0) continue;

						iter = find(aroundLabelList.begin(), aroundLabelList.end(), adjcentFace->fuzzy_label);
						if (iter == aroundLabelList.end())
							aroundLabelList.push_back(adjcentFace->fuzzy_label);
					}
				}


				// one field
				//HEdge *he = f->HalfEdge();
				//do
				//{
				//	Face * adjcentFace = he->Twin()->LeftFace();
				//	if (adjcentFace != NULL)
				//	{
				//		if (adjcentFace->fuzzy_label > 0)
				//		{
				//			iter = find(aroundLabelList.begin(), aroundLabelList.end(), adjcentFace->fuzzy_label);
				//			if (iter == aroundLabelList.end())
				//				aroundLabelList.push_back(adjcentFace->fuzzy_label);
				//		}
				//	}

				//	he = he->Next();
				//} while (he != f->HalfEdge());

				iter = find(aroundLabelList.begin(), aroundLabelList.end(), f->r_label);
				if (iter != aroundLabelList.end() && aroundLabelList.size() > 1)
				{
					tmpfList.push_back(f);
			
				}
			}
		}

		for (size_t i = 0; i < tmpfList.size(); i++)
		{
			tmpfList[i]->fuzzy_label = tmpfList[i]->r_label;
		}

	}

	void ToothSegmentation::RemoveSmallComponent(Mesh & mesh, int iteration, float protectedAreaTh, float RemovedAreaTh)
	{
		for (int k = 0; k < iteration; k++)
		{
			vector<FaceList> labelComponents;
			vector<double> areaList;
			vector<int> labelList;
			vector<bool> protectedList;

			//mesh.FindSmallAreaWithRLabel(labelComponents);
			mesh.FindSmallAreaWithFuzzyLabel(labelComponents);

			for (size_t i = 0; i < labelComponents.size(); i++)
			{
				FaceList & component = labelComponents[i];
				labelList.push_back(component[0]->fuzzy_label);
				double area = 0.0;
				for (size_t j = 0; j < component.size(); j++)
					area += component[j]->area;
				areaList.push_back(area);
			}

			// for each label find the component with max area as protected component
			protectedList.resize(labelComponents.size(), false);
			for (int label = 0; label < 17; label++)
			{
				int idx = -1;
				for (size_t i = 0; i < labelList.size(); i++)
				{
					if (labelList[i] != label) continue;
					if (idx == -1 || areaList[idx] < areaList[i])
						idx = i;
				}
				if (idx < 0) continue;
				if (areaList[idx] < RemovedAreaTh) continue;

				protectedList[idx] = true;
			}

			// delete small region
			for (size_t i = 0; i < labelComponents.size(); i++)
			{
				FaceList & component = labelComponents[i];

				if (protectedList[i]) continue;
				if (areaList[i] > protectedAreaTh) continue;
				
				//cout << area << " " << component.size() << endl;
				// find a neighbor which has the maximun adjcent faces
				//for (size_t j = 0; j < component.size(); j++)
				//	component[j]->tmpIdx = 100;
				int neighbors[100];
				for (int j = 0; j < 100; j++) neighbors[j] = 0;
				for (size_t j = 0; j < component.size(); j++)
				{
					HEdge * he = component[j]->HalfEdge();
					do
					{
						if (he->Twin()->LeftFace() != NULL)
						{
							Face * adjcentFace = he->Twin()->LeftFace();
							if (adjcentFace->fuzzy_label != component[j]->fuzzy_label)
							{
								neighbors[adjcentFace->fuzzy_label]++;
							}
						}
						he = he->Next();
					} while (he != component[j]->HalfEdge());
				}

				int maxIdx = 0;
				for (int j = 1; j < 100; j++)
				{
					if (neighbors[j]>neighbors[maxIdx])
						maxIdx = j;
				}

				for (size_t j = 0; j < component.size(); j++)
				{
					component[j]->fuzzy_label = maxIdx;
				}
			}
		}
	}
}