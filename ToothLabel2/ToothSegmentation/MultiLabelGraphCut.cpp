#include "ToothSegmentation.h"

namespace MeshSegmentation
{
	// Multi-label boundary optimization
	void MultiLabelGraphCut::MultiLabelOptimizeBoundary(Mesh & mesh, MultiLabelGraphCutParameters & para)
	{
		if (mesh.fList.size() == 0) return;
		
		FaceList maxComponent;
		mesh.FindMaxConnectedComponet();
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->GroupID() == mesh.maxGroupID)
				maxComponent.push_back(f);
		}

		if (maxComponent.size() == 0) return;
		int labelCount = (int)maxComponent[0]->prob.size();
		if (labelCount < 2) return;

		BuildAdjcentWeight(mesh, para.lambda);

		int startOptLabel = para.optimzeGingiva ? 0 : 1;
		for (int k = 0; k < para.iteration; k++)
		{
			bool success = false;
			for (int a = startOptLabel; a < labelCount; a++)
			{
				for (int b = a + 1; b < labelCount; b++)
				{
					success |= AlphaBetaSwap(maxComponent, a, b);
				}
			}
			if (!success)
				break;
		}

		// initalize fuzzy label, assign r_label to fuzzy_label
		// this code is important
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->fuzzy_label = mesh.fList[i]->r_label;
		}
	}
	/*
	void MultiLabelGraphCut::BuildAdjcentWeight(Mesh & mesh, float lambda)
	{
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
						cost *= (1 + abs(dotValue));

					cost = cost > 1e9 ? 1e9 : cost;
					f->adjcent_weight.push_back((int)cost);
				}

				he = he->Next();
			} while (he != f->HalfEdge());
		}
	}
	*/

	bool MultiLabelGraphCut::AlphaBetaSwap(FaceList & component, int a, int b)
	{
		FaceList swapfList;
		int counta = 0;
		int countb = 0;
		for (size_t i = 0; i < component.size(); i++)
		{
			Face *f = component[i];
			f->tmpIdx = -1;
			if (f->r_label == a || f->r_label == b)
			{
				if (f->r_label == a) counta++;
				if (f->r_label == b) countb++;

				f->tmpIdx = (int)swapfList.size();
				swapfList.push_back(f);
			}
		}

		// there must be at least one node connect to source or sink
		if (counta == 0 || countb == 0)
			return false;

		// build up t-link
		for (size_t i = 0; i < swapfList.size(); i++)
		{
			Face * f = swapfList[i];
		
			double w1 = -rate*log10(f->prob[a] + 1e-10);
			double w2 = -rate*log10(f->prob[b] + 1e-10);

			w1 = w1>1e9 ? 1e9 : w1;
			w2 = w2>1e9 ? 1e9 : w2;
			f->source_weight = (int)(w2);
			f->sink_weight = (int)(w1);
		}


		// count edge count
		int edgeCount = 0;
		for (size_t i = 0; i < swapfList.size(); i++)
		{
			Face * f = swapfList[i];
			for (size_t j = 0; j < f->adjcent_face.size(); j++)
			{
				if (f->adjcent_face[j]->tmpIdx >= 0 && f->tmpIdx<f->adjcent_face[j]->tmpIdx)
					edgeCount++;
			}
		}

		Graph<int, int, int> *maxflowPtr = new Graph<int, int, int>((int)swapfList.size(), edgeCount, NULL);

		// add_node
		maxflowPtr->add_node((int)swapfList.size());

		// add_edge
		for (size_t i = 0; i < swapfList.size(); i++)
		{
			Face * f1 = swapfList[i];
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
		for (size_t i = 0; i < swapfList.size(); i++)
		{
			Face * f = swapfList[i];

			maxflowPtr->add_tweights(f->tmpIdx, f->source_weight, f->sink_weight);
		}

		// maxflow
		maxflowPtr->maxflow(false, NULL);

		// result
		bool success = false;
		int predictLabel;
		for (size_t i = 0; i < swapfList.size(); i++)
		{
			Face * f = swapfList[i];
			predictLabel = maxflowPtr->what_segment(f->tmpIdx) == 0 ? a : b;

			if (predictLabel != f->r_label)
				success = true;
			f->r_label = predictLabel;
		}

		delete maxflowPtr;
		maxflowPtr = NULL;

		return success;
	}
}