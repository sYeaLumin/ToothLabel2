#include "ToothSegmentation.h"

namespace MeshSegmentation
{
	// Improved Fuzzy Clustering boundary optimization
	void ImprovedFuzzyClustering::FuzzyOptimizeBoundary(Mesh & mesh, ImprovedFuzzyClusteringParameters & para)
	{
		if (mesh.fList.size() == 0) return;

		FaceList fuzzyFaceList;
//		if (para.useAngDist)
//			mesh.FindFuzzyBoundaryAngDist(fuzzyFaceList, para.propagate, para.angDistThreshold, para.yitaAngDist, para.optLabel);
		FindFuzzyRegion(fuzzyFaceList, mesh, para);


		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face *f = mesh.fList[i];
			f->tmpIdx = -1;
		}

		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];
			f->tmpIdx = i;
		}

		if (fuzzyFaceList.size() == 0) return;

		int edgeCount = BuildAdjcentWeight_AngDist(fuzzyFaceList, para);

		Graph<int, int, int> *maxflowPtr = new Graph<int, int, int>((int)fuzzyFaceList.size(), edgeCount, NULL);

		// add_node
		maxflowPtr->add_node((int)fuzzyFaceList.size());

		// add_edge
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f1 = fuzzyFaceList[i];
			for (size_t j = 0; j < f1->adjcent_face.size(); j++)
			{
				Face * f2 = f1->adjcent_face[j];
				if (f2->tmpIdx >= 0 && f1->tmpIdx < f2->tmpIdx)
				{
					float weight = f1->adjcent_weight[j];
					//if ((!f1->labelBoundary || !f2->labelBoundary) && (f1->fuzzy_label != f2->fuzzy_label))
					//	weight *= 20;
					maxflowPtr->add_edge(f1->tmpIdx, f2->tmpIdx, weight, weight);
				}
			}
		}

		// add_tweight
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];
			// 0 indicate the face f isn't the boundary of fuzzy region
			// 1 indicate the face f is the bounndary of fuzzy region and is near the gum
			// 2 indicate the face f is the bounndary of fuzzy region and is near the mesh
			int flag = 0;
			HEdge * he = f->HalfEdge();
			do{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (adjcentFace != NULL && adjcentFace->tmpIdx < 0)
				{
					if (adjcentFace->fuzzy_label == para.optLabel)
					{
						flag = 1;
						break;
					}
					else if (adjcentFace->fuzzy_label == 0)
					{
						flag = 2;
						break;
					}
					else
					{
						flag = 3;
						break;
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());

			if (flag == 1)
			{
				maxflowPtr->add_tweights(f->tmpIdx, 1000000, 0);
			}
			else if (flag == 2)
			{
				maxflowPtr->add_tweights(f->tmpIdx, 0, 1000000);
			}
			else if (flag == 3)
			{
				maxflowPtr->add_tweights(f->tmpIdx, 0, 1000);
			}
		}

		// maxflow
		maxflowPtr->maxflow(false, NULL);

		// result
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];
			if (!f->labelBoundary) continue;
			f->fuzzy_label = maxflowPtr->what_segment(f->tmpIdx) == 0 ? para.optLabel : 0;
		}

		delete maxflowPtr;
		maxflowPtr = NULL;
	}

	void ImprovedFuzzyClustering::MultiLabelFuzzyOptimizeBoundary(Mesh & mesh, ImprovedFuzzyClusteringParameters & para)
	{
		vector<int> labelList;
		vector<int>::iterator iter;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			iter = find(labelList.begin(), labelList.end(), f->fuzzy_label);
			if (iter == labelList.end())
			{
				labelList.push_back(f->fuzzy_label);
			}
		}
		if (labelList.size() < 2) return;

		if (para.useAngDist)
			mesh.BuildAngDist();

		// find protected region, which aren't fuzzy seed region
		FindProtectedRegion(mesh, para);

		// Multi-label Improved Fuzzy Clustring
		for (int k = 0; k < para.iteration; k++)
		{
			for (size_t i = 0; i < labelList.size(); i++)
			{
				if (labelList[i] == 0) continue;

				para.optLabel = labelList[i];
				FuzzyOptimizeBoundary(mesh, para);
			}
		}
	}

	void ImprovedFuzzyClustering::FindProtectedRegion(Mesh & mesh, ImprovedFuzzyClusteringParameters & para)
	{
		if (mesh.fList.empty()) return;
		for (size_t i = 0; i < mesh.fList.size(); i++)
			mesh.fList[i]->protect = false;
		mesh.need_curvatures();

		FaceList protectfList;
		for (size_t i = 0; i < mesh.heList.size(); i++)
		{
			HEdge * he = mesh.heList[i];
			Vertex * v1 = he->Start(), *v2 = he->End();
			if (v1->maxCurv<para.curvTh[0] || v1->maxCurv>para.curvTh[1]) continue;
			if (v2->maxCurv<para.curvTh[0] || v2->maxCurv>para.curvTh[1]) continue;

			Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
			if (f1 && f2 && f1->fuzzy_label > 0 && f2->fuzzy_label > 0 && f1->fuzzy_label != f2->fuzzy_label)
			{
				if (!f1->protect)
				{
					protectfList.push_back(f1);
					f1->protect = true;
				}
				if (!f2->protect)
				{
					protectfList.push_back(f2);
					f2->protect = true;
				}
			}
		}

		// propagate
		size_t start = 0, end;
		for (int k = 1; k < para.protectPropagate; k++)
		{
			end = protectfList.size();
			for (size_t i = start; i < end; i++)
			{
				Face * f = protectfList[i];
				HEdge * he = f->HalfEdge();
				do
				{
					Vertex * v1 = he->Start(), *v2 = he->End();
					if ((v1->maxCurv>para.curvTh[0] && v1->maxCurv<para.curvTh[1]) || (v2->maxCurv>para.curvTh[0] && v2->maxCurv < para.curvTh[1]))
					{
						if (he->Twin()->LeftFace() != NULL)
						{
							Face * adjcentFace = he->Twin()->LeftFace();
							if (!adjcentFace->protect && adjcentFace->r_label > 0)
							{
								protectfList.push_back(adjcentFace);
								adjcentFace->protect = true;
							}
						}
					}
					he = he->Next();
				} while (he != f->HalfEdge());
			}
		}

		cout << "protect: " << protectfList.size() << endl;
		//for (size_t i = 0; i < protectfList.size(); i++)
		//{
		//	protectfList[i]->tmpLabel = 17;
		//}
	}

	void ImprovedFuzzyClustering::FindFuzzyRegion(FaceList & fuzzyFaceList, Mesh & mesh, ImprovedFuzzyClusteringParameters & para)
	{
		FaceList roughfList;		// rough region
		FaceList cbfList;			// current boundary
		FaceList pbfList;			// possible boundary


		if (mesh.fList.size() == 0) return;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->visited = false;
			mesh.fList[i]->labelBoundary = false;
			//mesh.fList[i]->tmpAngDist = 1000.0f;
			mesh.fList[i]->tmpGeoDist = 1000.0f;
		}

		mesh.need_curvatures();
		fuzzyFaceList.clear();

		float totalArea = 0.0f, areaTh;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->fuzzy_label == para.optLabel)
				totalArea += f->area;
		}
		areaTh = totalArea*(1-para.basicProtectAreaRatio);

		float area = 0.0;
		// step 1: find current boundary
		cbfList.clear();
		for (size_t i = 0; i < mesh.heList.size(); i++)
		{
			HEdge* he = mesh.heList[i];
			if (he->LeftFace() != NULL && he->Twin()->LeftFace() != NULL)
			{
				Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
				int fuzzy_label1 = f1->fuzzy_label;
				int fuzzy_label2 = f2->fuzzy_label;
				if ((fuzzy_label1 == para.optLabel && fuzzy_label2 == 0) || (fuzzy_label1 == 0 && fuzzy_label2 == para.optLabel))
				{
					float angDist = he->convex ? para.yita*he->angDist : he->angDist;
					angDist = min(para.edgeAngDistTh, angDist);

					//float geoDist = CalcGeodesicDistance(he);
					float geoDist = 0.0f;
					if (!f1->visited)
					{
						f1->visited = true;
						f1->tmpDist = angDist;
						f1->tmpGeoDist = geoDist;
						cbfList.push_back(f1);

						if (f1->fuzzy_label == para.optLabel)
							area += f1->area;
					}
					if (!f2->visited)
					{
						f2->visited = true;
						f2->tmpDist = angDist;
						f2->tmpGeoDist = geoDist;
						cbfList.push_back(f2);

						if (f2->fuzzy_label == para.optLabel)
							area += f2->area;
					}
				}
			}
		}

		// step 2: find rough region
		roughfList.assign(cbfList.begin(), cbfList.end());
		size_t start = 0, end;
		for (int k = 1; k < para.roughPropagate; k++)
		{
			end = roughfList.size();
			for (size_t i = start; i < end; i++)
			{
				Face * f = roughfList[i];
				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() != NULL)
					{
						Face * adjcentFace = he->Twin()->LeftFace();
						int tmpLabel = adjcentFace->fuzzy_label;
						if (tmpLabel == para.optLabel || tmpLabel == 0)
						{
							if (!adjcentFace->visited)
							{
								float geoDist = CalcGeodesicDistance(he);
								adjcentFace->tmpGeoDist = min(adjcentFace->tmpGeoDist, f->tmpGeoDist + geoDist);
								if (adjcentFace->tmpGeoDist < para.geodesicPropagateTh)
								{
									adjcentFace->visited = true;
									roughfList.push_back(adjcentFace);

									if (adjcentFace->fuzzy_label == para.optLabel)
										area += adjcentFace->area;
								}
							}
						}
					}
					he = he->Next();
				} while (he != f->HalfEdge());
			}
			start = end;
			
			if (area>areaTh)
				break;
		}

		// step 3: find protected region



		// step 4: find possible boundary
		for (size_t i = 0; i < roughfList.size(); i++)
		{
			Face * f = roughfList[i];
			HEdge * he = f->HalfEdge();
			do
			{
				if (he->Twin()->LeftFace() != NULL)
				{
					float angDist = he->convex ? para.yita*he->angDist : he->angDist;
					angDist = min(para.edgeAngDistTh, angDist);
					Face * adjcentFace = he->Twin()->LeftFace();
					if (!adjcentFace->protect)
					{
						Vertex * v1 = he->Start(), * v2 = he->End();
						if ((v1->maxCurv>para.curvTh[0] && v1->maxCurv < para.curvTh[1]) || (v2->maxCurv>para.curvTh[0] && v2->maxCurv < para.curvTh[1]))
						{
							pbfList.push_back(f);
							f->tmpDist = angDist;
							//f->tmpLabel = 4;
							break;
						}
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}

		// step 5: find fuzzy region
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->tmpIdx = -1;
		}
		vector<int> roughRegion;
		roughRegion.resize(roughfList.size(), 0);
		for (size_t i = 0; i < roughfList.size(); i++)
		{
			Face *f = roughfList[i];
			f->tmpIdx = (int)i;
			
		}

		BoundaryBFSPropagate(mesh, cbfList, roughfList, roughRegion, para, true);
		BoundaryBFSPropagate(mesh, pbfList, roughfList, roughRegion, para);

		for (size_t i = 0; i < roughRegion.size(); i++)
		{
			if (roughRegion[i] >= 2)
			{
				fuzzyFaceList.push_back(roughfList[i]);
				roughfList[i]->labelBoundary = true;
				roughfList[i]->tmpLabel = 1;
			}
			else if (roughRegion[i] == 1)
			{
				roughfList[i]->tmpLabel = 2;
			}
			else
			{
				roughfList[i]->tmpLabel = 3;
			}
		}

	}

	void ImprovedFuzzyClustering::BoundaryBFSPropagate(Mesh & mesh, FaceList & bfList, FaceList & roughfList, vector<int> & roughRegion, ImprovedFuzzyClusteringParameters & para, bool useAngDist)
	{
		if (useAngDist)
		{
			for (size_t i = 0; i < mesh.fList.size(); i++)
				mesh.fList[i]->tmpAngDist = 1000.0f;
		}

		for (size_t i = 0; i < roughfList.size(); i++)
		{
			roughfList[i]->visited = false;
		}

		for (size_t i = 0; i < bfList.size(); i++)
		{
			Face * f = bfList[i];
			bfList[i]->visited = true;
			roughRegion[bfList[i]->tmpIdx] = 2;
			if (useAngDist)
			{
				bfList[i]->tmpAngDist = bfList[i]->tmpDist;
			}
		}



		size_t start = 0, end;
		for (size_t k = 0; k < para.basePropagate+para.nextPropagate; k++)
		{
			if (k == para.basePropagate + para.nextPropagate - 1)
			{
				start = 0;
			}

			end = bfList.size();
			for (size_t i = start; i < end; i++)
			{
				Face * f = bfList[i];
				HEdge * he = f->HalfEdge();
				do
				{
					if (he->Twin()->LeftFace() != NULL)
					{
						Face * adjcentFace = he->Twin()->LeftFace();
						if (!adjcentFace->visited && adjcentFace->tmpIdx >= 0)
						{
							adjcentFace->visited = true;
							if (useAngDist)
							{
								float angDist = he->convex ? para.yita*he->angDist : he->angDist;
			
								angDist = min(para.edgeAngDistTh, angDist);
								adjcentFace->tmpAngDist = min(adjcentFace->tmpAngDist, f->tmpAngDist + angDist);
								if (adjcentFace->tmpAngDist < para.angDistThreshold || k<para.basePropagate || k == para.basePropagate + para.nextPropagate - 1)
								{
									bfList.push_back(adjcentFace);
									if (k<para.basePropagate)
										roughRegion[adjcentFace->tmpIdx] = 2;
									else
										roughRegion[adjcentFace->tmpIdx] += 1;
								}
							}
							else
							{
								bfList.push_back(adjcentFace);
								if (k<para.basePropagate)
									roughRegion[adjcentFace->tmpIdx] = 2;
								else
									roughRegion[adjcentFace->tmpIdx] += 1;
							}
						}
					}
					he = he->Next();
				} while (he != f->HalfEdge());
			}
			start = end;
		}
	}

	int ImprovedFuzzyClustering::BuildAdjcentWeight_AngDist(FaceList & fuzzyFaceList, ImprovedFuzzyClusteringParameters & para)
	{
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			fuzzyFaceList[i]->adjcent_face.clear();
			fuzzyFaceList[i]->adjcent_weight.clear();
			fuzzyFaceList[i]->assist_weight.clear();
		}

		float dist_all = 0.0f; int count = 0;
		vector<vector<float>> ang_dist_list_all;
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];
			vector<float> ang_dist_list;
			HEdge * he = f->HalfEdge();
			do{
				if (he->Twin()->LeftFace() != NULL && he->Twin()->LeftFace()->tmpIdx >= 0)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					f->adjcent_face.push_back(adjcentFace);
					double dotValue = f->normal.Dot(adjcentFace->normal);
					dotValue = dotValue>1.0 ? 1.0 : dotValue;
					dotValue = dotValue < -1.0 ? -1.0 : dotValue;

					float ang_dist;

					Vector3d c2c1 = adjcentFace->center - f->center;
					c2c1.Normalize();
					if (c2c1.Dot(f->normal) > 0)
					{
						ang_dist = (float)(1 - dotValue);
					}
					else
					{
						ang_dist = (float)(para.yita*(1 - dotValue));
					}

					ang_dist_list.push_back(ang_dist);
					dist_all += ang_dist; count++;
				}

				he = he->Next();
			} while (he != f->HalfEdge());

			ang_dist_list_all.push_back(ang_dist_list);
		}

		BuildAssitWeight(fuzzyFaceList,para);

		float avg_ang_dist = dist_all / count;
		for (size_t i = 0; i < ang_dist_list_all.size(); i++)
		{
			for (size_t j = 0; j < ang_dist_list_all[i].size(); j++)
			{
				float cap = 1.0f / (1.0f + exp(ang_dist_list_all[i][j] / avg_ang_dist));

				cap *= fuzzyFaceList[i]->assist_weight[j];

				fuzzyFaceList[i]->adjcent_weight.push_back((int)(rate*cap));
			}
		}

		return count;
	}

	void ImprovedFuzzyClustering::BuildAssitWeight(FaceList & fuzzyFaceList, ImprovedFuzzyClusteringParameters & para)
	{
		const float maxFloat = 1e10;

		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];
			HEdge * he = f->HalfEdge();
			do{
				if (he->Twin()->LeftFace() != NULL && he->Twin()->LeftFace()->tmpIdx >= 0)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					double distance = CalcGeodesicDistance(he);
					f->assist_weight.push_back((float)distance);
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}


		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			fuzzyFaceList[i]->tmpGeodesicDistance = maxFloat;
			fuzzyFaceList[i]->visited = false;
		}

		// find source boundary
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face * f = fuzzyFaceList[i];

			for (size_t j = 0; j < f->adjcent_face.size(); j++)
			{
				Face * adjcentFace = f->adjcent_face[j];
				if (f->r_label != adjcentFace->r_label)
				{
					f->visited = true;

					HEdge * he = f->HalfEdge();
					do
					{
						if (he->Twin()->LeftFace() == adjcentFace)
							break;
						he = he->Next();
					} while (he != f->HalfEdge());

					f->tmpGeodesicDistance = (float)CalcGeodesicDistance(f, he);
					//f->tmpGeodesicDistance = 0;
					break;
				}
			}

			if (f->visited)
			{
				for (size_t j = 0; j < f->adjcent_face.size(); j++)
				{
					Face * adjcentFace = f->adjcent_face[j];
					if (!adjcentFace->visited)
					{
						float tmp = f->tmpGeodesicDistance + f->assist_weight[j];
						adjcentFace->tmpGeodesicDistance = adjcentFace->tmpGeodesicDistance < tmp ? adjcentFace->tmpGeodesicDistance : tmp;
					}
				}
			}
		}

		// dijkstra 
		int index = 0;
		while (index >= 0)
		{
			index = -1;
			for (size_t i = 0; i < fuzzyFaceList.size(); i++)
			{
				Face * f = fuzzyFaceList[i];
				if (!f->visited)
				{
					if (index == -1 || fuzzyFaceList[index]->tmpGeodesicDistance > f->tmpGeodesicDistance)
					{
						index = i;
					}
				}
			}

			if (index >= 0)
			{
				Face * f = fuzzyFaceList[index];
				f->visited = true;
				for (size_t j = 0; j < f->adjcent_face.size(); j++)
				{
					Face * adjcentFace = f->adjcent_face[j];
					if (!adjcentFace->visited)
					{
						float tmp = f->tmpGeodesicDistance + f->assist_weight[j];
						adjcentFace->tmpGeodesicDistance = adjcentFace->tmpGeodesicDistance < tmp ? adjcentFace->tmpGeodesicDistance : tmp;
					}
				}
			}
		}


		// build assist weight
		for (size_t i = 0; i < fuzzyFaceList.size(); i++)
		{
			Face*f = fuzzyFaceList[i];
			for (size_t j = 0; j < f->adjcent_face.size(); j++)
			{
				Face * adjcentFace = f->adjcent_face[j];
				float dist = f->tmpGeodesicDistance + adjcentFace->tmpGeodesicDistance;

				f->assist_weight[j] = 1.0f / (1.0f + exp(-dist*dist / para.geodesicSigma));
			}
		}

	}
}