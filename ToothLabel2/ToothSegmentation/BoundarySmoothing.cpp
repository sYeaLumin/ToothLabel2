#include "ToothSegmentation.h"

namespace MeshSegmentation
{
	void BoundarySmoothing::SmoothBoundary(Mesh & mesh, BoundarySmoothingParameters & para)
	{
		// Bump Smoothing
		for (int k = 0; k < para.BumpSmoothingIteration; k++)
		{
			SmoothBump(mesh);
		}

		// shortest route optimization
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->tmpLabel = mesh.fList[i]->fuzzy_label;
		}

		for (int k = 0; k < para.ShortestOptIteration; k++)
		{
			ShortestRouteOptimzation(mesh, para);
		}

		// Bump Smoothing
		for (int k = 0; k < para.BumpSmoothingIteration; k++)
		{
			SmoothBump(mesh);
		}

	}

	void BoundarySmoothing::SmoothBump(Mesh & mesh)
	{
		int bumpCount = 0;

		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			HEdge * he = f->HalfEdge();
			FaceList adjcentFaceList;
			
			do
			{
				Face * adjcentFace = he->Twin()->LeftFace();
				if (adjcentFace != NULL)
				{
					adjcentFaceList.push_back(adjcentFace);
				}
				he = he->Next();
			} while (he != f->HalfEdge());

			if (adjcentFaceList.size() < 3) continue;

			map<int, int> m;
			for (size_t j = 0; j < adjcentFaceList.size(); j++)
			{
				int label = adjcentFaceList[j]->fuzzy_label;
				if (m.find(label) == m.end())
				{
					m.insert(pair<int, int>(label, 1));
				}
				else
				{
					m[label] = m[label] + 1;
				}
			}
			
			if (m.size() != 2) continue;
			map<int, int>::iterator iter;
			for (iter = m.begin(); iter != m.end(); iter++)
			{
				if (iter->second != 2) continue;
				if (iter->first != f->fuzzy_label)
				{
					f->fuzzy_label = iter->first;
					bumpCount++;
					break;
				}
			}
		}
		cout << "bump: " << bumpCount << endl;

		
	}

	void BoundarySmoothing::ShortestRouteOptimzation(Mesh & mesh, BoundarySmoothingParameters & para)
	{
		for (int label = 1; label < 17; label++)	// ignore the label of gingiva
		{
			vector<VertexList> boundaries;
			FindBoundary(mesh, boundaries, label, para);
			cout << "label: " << label << " " << boundaries.size() << endl;

			para.label = label;
			for (size_t i = 0; i < boundaries.size(); i++)
			{
				// optimize boundary
				if (para.useSampling)
				{
					vector<VertexList> subBoundaries;
					BoundarySampling(boundaries[i], subBoundaries, para);
					for (size_t j = 0; j < subBoundaries.size(); j++)
					{
						SingleBoundaryShortestRouteOptimization(mesh, subBoundaries[j], para);
					}
				}
				else
				{
					SingleBoundaryShortestRouteOptimization(mesh, boundaries[i], para);
				}
			}
			
		}
	}

	void BoundarySmoothing::FindBoundary(Mesh & mesh, vector<VertexList> & boundaries, int label, BoundarySmoothingParameters & para)
	{
		for (size_t i = 0; i < mesh.vList.size(); i++)
		{
			Vertex * v = mesh.vList[i];
			v->labelBoundary = false;
			v->possibleBoundary = false;
			v->visited = false;
		}

		// step 1: find boundary for label
		VertexList allBoundaryvList;
		for (size_t i = 0; i < mesh.heList.size(); i++)
		{
			mesh.heList[i]->visited = false;
		}
		for (size_t i = 0; i < mesh.heList.size(); i++)
		{
			HEdge * he = mesh.heList[i];
			if (he->visited) continue;
			he->visited = true;
			he->Twin()->visited = true;

			Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
			if (f1 && f2 && (f1->fuzzy_label == label || f2->fuzzy_label == label) && f1->fuzzy_label != f2->fuzzy_label)
			{
				Vertex * v1 = he->Start(), *v2 = he->End();
				if (!v1->labelBoundary)
				{
					v1->labelBoundary = true;
					allBoundaryvList.push_back(v1);
				}
				if (!v2->labelBoundary)
				{
					v2->labelBoundary = true;
					allBoundaryvList.push_back(v2);
				}
			}
		}


		// step 2: find boundary loops
		boundaries.clear();
		for (size_t i = 0; i < allBoundaryvList.size(); i++)
		{
			Vertex * u = allBoundaryvList[i];
			if (u->visited) continue;

			VertexList boundary;
			boundary.push_back(u);
			u->visited = true;

			// dfs
			Vertex * vStart = u, *vNow = u;
			do
			{
				OneRingHEdge ring(vNow);
				HEdge * he = NULL;
				Vertex * lastv = NULL;

				while ((he = ring.NextHEdge()) != NULL)
				{
					Vertex * v = he->End();
					Face * f1 = he->LeftFace(), *f2 = he->Twin()->LeftFace();
					
					if (!f1 || !f2 || (f1->fuzzy_label != label) || (f1->fuzzy_label == f2->fuzzy_label)) continue;
					if (v == vStart)
					{
						lastv = vStart;
					}
					if (v->visited) continue;
					if (!v->labelBoundary) continue;
					lastv = v;
					break;
				}
				if (lastv != NULL)
				{
					vNow = lastv;
					if (vNow != vStart)
					{
						boundary.push_back(vNow);
						vNow->visited = true;
					}
				}
				else
				{
					cout << "can't find a closed cycle.";
					break;	// can't find a closed cycle
				}
			} while (vNow != vStart);

			boundaries.push_back(boundary);
			cout << "boundary: "<<boundary.size() << " ";

		}
	}

	void BoundarySmoothing::BoundarySampling(VertexList & boundary, vector<VertexList> & subBoundaries, BoundarySmoothingParameters & para)
	{
		subBoundaries.clear();
		int startIdx = rand() % boundary.size();

		vector<int> startIdxList;
		for (size_t i = 0; i < boundary.size(); i += para.samplingInterval)
		{
			int idx = (i + startIdx) % boundary.size();
			startIdxList.push_back(idx);
		}

		for (size_t i = 0; i < startIdxList.size(); i++)
		{
			VertexList tmpvList;
			int start = startIdxList[i];
			int end = startIdxList[(i + 1) % startIdxList.size()];
			for (int j = start; j < end; j++)
			{
				tmpvList.push_back(boundary[j]);
			}
			if (tmpvList.size()<para.minSamplingInterval) continue;
			subBoundaries.push_back(tmpvList);
		}
	}

	void BoundarySmoothing::SetPossibleBoundary(Mesh & mesh, Vertex * v, int field, VertexList & possibleBoundary)
	{
		VertexList tmpvList;
		mesh.FindKFieldFromVertex(v, field, tmpvList);
		tmpvList.push_back(v);
		for (size_t i = 0; i < tmpvList.size(); i++)
		{
			if (!tmpvList[i]->possibleBoundary)
			{
				possibleBoundary.push_back(tmpvList[i]);
				tmpvList[i]->possibleBoundary = true;
			}
		}
	}

	void BoundarySmoothing::SingleBoundaryShortestRouteOptimization(Mesh & mesh, VertexList & boundary, BoundarySmoothingParameters & para)
	{
		if (!para.useSampling && boundary.size() < para.boundaryOptMinSize) return;

		// step 1: find possible boundary region
		VertexList possibleBoundary;

		bool cycleFlag = IsCycle(boundary);
		for (size_t i = 0; i < boundary.size(); i++)
		{
			double tmp = IsPossibleBoundary(boundary, i, cycleFlag);
			if (tmp > para.possibleBoundaryTh[0])
			{
				int level = 0;
				for (size_t j = 0; j < para.possibleBoundaryTh.size(); j++)
				{
					if (tmp>para.possibleBoundaryField[j])
					{
						level = j;
					}
				}
				SetPossibleBoundary(mesh, boundary[i], para.possibleBoundaryField[level], possibleBoundary);
			}
			else
			{
				boundary[i]->possibleBoundary = true;
				possibleBoundary.push_back(boundary[i]);
			}

		}

		
		// debug
		for (size_t i = 0; i < possibleBoundary.size(); i++)
		{
			possibleBoundary[i]->seed_flag = true;
		}


		// step 2: find a cut
		VertexList startvList;
		VertexList endvList;

		bool createNewEndvList = false;

		if (para.useSampling)
		{
			startvList.push_back(boundary[0]);
			boundary[0]->cutting_point = true;

			//FindStartvList(startvList, possibleBoundary, boundary[0], para);
			endvList.push_back(boundary.back());
		}
		else
		{
			FindStartvList(startvList, possibleBoundary, boundary[0], para);
			createNewEndvList = CreateEndvList(startvList, endvList, boundary, possibleBoundary);
		}
	

		mesh.BuildAngDist();
		CreateAdjacentMap(startvList, endvList, possibleBoundary, boundary, para);

		//for (size_t i = 0; i < mesh.heList.size(); i++)
		//{
		//	HEdge * he = mesh.heList[i];
		//	if (he->angDist>para.angDistTh)
		//	{
		//		he->Start()->seed_flag = true;
		//		he->End()->seed_flag = true;
		//	}
		//}

		// step 3: dijkstra 
		VertexList optBoundaryvList;
		FindShortestBoundary(startvList, endvList, possibleBoundary, optBoundaryvList);

		//cout << "opt: " << boundary.size() << " " << optBoundaryvList.size() << endl;

		//for (size_t i = 0; i < optBoundaryvList.size(); i++)
		//{
		//	optBoundaryvList[i]->protect_flag = true;
		//}

		// step 4: assign new label for the possible boundary region
		AssignNewLabel(mesh, optBoundaryvList, possibleBoundary, para);

		// step 5: clear possibleBoundary, endvList
		for (size_t i = 0; i < possibleBoundary.size(); i++)
		{
			possibleBoundary[i]->possibleBoundary = false;
		}

		if (createNewEndvList)
		{
			for (size_t i = 0; i < endvList.size(); i++)
			{
				delete endvList[i];
			}
		}

	}

	void BoundarySmoothing::FindStartvList(VertexList & startvList, VertexList & totalvList, Vertex * u, BoundarySmoothingParameters & para)
	{
		
		// tmpvList is the a cut, and the cut should be directed 
		// from one vertex with label to one vertex with non-label
		VertexList tmpvList;
		FindAShortestPathToLabel(tmpvList, totalvList, u, para.label, true);
		for (size_t i = 0; i < tmpvList.size(); i++)
		{
			startvList.push_back(tmpvList[i]);
		}

		startvList.push_back(u);
		
		FindAShortestPathToLabel(tmpvList, totalvList, u, para.label, false);
		for (size_t i = 0; i < tmpvList.size(); i++)
		{
			startvList.push_back(tmpvList[i]);
		}

		//cout << "startvList: " << startvList.size() << endl;
		for (size_t i = 0; i < startvList.size(); i++)
		{
			startvList[i]->cutting_point = true;
		}
	}

	bool BoundarySmoothing::CreateEndvList(VertexList & startvList, VertexList & endvList, VertexList & boundary, VertexList & possibleBoundary)
	{
		bool findv = IsCycle(boundary);

		
		if (!findv)
		{
			startvList.clear();
			startvList.push_back(boundary[0]);
			endvList.push_back(boundary.back());

			return false;
		}

		for (size_t i = 0; i < startvList.size(); i++)
		{
			Vertex * v = new Vertex(startvList[i]->Position());
			v->possibleBoundary = true;
			endvList.push_back(v);
		}

		for (size_t i = 0; i < endvList.size(); i++)
		{
			possibleBoundary.push_back(endvList[i]);
		}

		return true;
	}

	void BoundarySmoothing::CreateAdjacentMap(VertexList & startvList, VertexList & endvList, VertexList & totalvList, VertexList & boundary, BoundarySmoothingParameters & para)
	{
		/*
					+====================-
					|					 |
					+====================-
					|					 |
					+====================-
				  start				    end
			  flag  0  1 ======2====== 3  4
		*/
		const int maxIntValue = 1000000;


		for (size_t i = 0; i < totalvList.size(); i++)
		{
			totalvList[i]->adjacentvList.clear();
			totalvList[i]->adjacentDist.clear();
			totalvList[i]->visited = false;
			totalvList[i]->SetFlag(2);
			totalvList[i]->tmpIndex = (int)i;
		}
		for (size_t i = 0; i < startvList.size(); i++)
			startvList[i]->SetFlag(0);
		for (size_t i = 0; i < endvList.size(); i++)
			endvList[i]->SetFlag(4);

		// find region with flag 1 and 3
		// how to set
		for (size_t i = 0; i < startvList.size(); i++)
		{
			Vertex * u = startvList[i];
			OneRingVertex ring(u);
			Vertex * v = NULL;
			while ((v = ring.NextVertex()) != NULL)
			{
				if (!v->possibleBoundary) continue;
				if (v->Flag() == 0) continue;	// v is a start vertex
				v->SetFlag(1);
			}
		}

		// I use threshold method to separate regions with flag 1 and 3
		// if the bfs distance is larger than th, these vertices is 3
		// this is a wrong method, but in general case it's true
		int th = (int)(boundary.size() >> 1);
		for (size_t i = 0; i < totalvList.size(); i++)
			totalvList[i]->visited = false;

		vector<int> distList;
		distList.resize(totalvList.size(), maxIntValue);

		VertexList vSet;
		Vertex * origin = boundary[1];
		origin->visited = true;
		distList[origin->tmpIndex] = 0;
		vSet.push_back(origin);

		size_t start = 0, end;
		int stepCount = 0;
		while (start < vSet.size())
		{
			stepCount++;
			end = vSet.size();
			for (size_t i = start; i < end; i++)
			{
				Vertex * u = vSet[i];
				OneRingVertex ring(u);
				Vertex * v = NULL;
				while ((v = ring.NextVertex()) != NULL)
				{
					if (!v->possibleBoundary) continue;
					if (v->visited) continue;
					if (v->Flag() == 0) continue;
			
					v->visited = true;
					distList[v->tmpIndex] = stepCount;
					vSet.push_back(v);
				}
			}
			start = end;
		
		}

		int count = 0;
		for (size_t i = 0; i < totalvList.size(); i++)
		{
			Vertex * v = totalvList[i];
			if (v->Flag() != 1) continue;
			if (distList[v->tmpIndex] < th) continue;

			v->SetFlag(3);
		}


		// build adjcent map
		// flag 0: 0, 1
		// flag 1: 0, 1, 2
		// flag 2: 1, 2, 3
		// flag 3: 2, 3, 4
		// flag 4: 3, 4

		bool cycleFlag = IsCycle(boundary);
		for (size_t i = 0; i < totalvList.size(); i++)
		{
			Vertex * u = totalvList[i];
			OneRingHEdge ring(u);
			HEdge * he = NULL;
			Vertex * v = NULL;
			while ((he = ring.NextHEdge()) != NULL)
			{
				v = he->End();
				if (!v->possibleBoundary) continue;
				if (v->tmpIndex < u->tmpIndex) continue;

				double edgeDist = CalcEdgeDistance(he, para);

				if (u->Flag() == 0)
				{
					if (v->Flag() == 0 || v->Flag() == 1)
					{
						u->adjacentvList.push_back(v);
						v->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						v->adjacentDist.push_back(edgeDist);
					}

					if (v->Flag() == 3 && cycleFlag)
					{
						size_t tmpIdx = 0;
						for (size_t i = 0; i < startvList.size(); i++)
						{
							if (startvList[i] == u)
							{
								tmpIdx = i; break;
							}
						}
						v->adjacentvList.push_back(endvList[tmpIdx]);
						endvList[tmpIdx]->adjacentvList.push_back(v);

						v->adjacentDist.push_back(edgeDist);
						endvList[tmpIdx]->adjacentDist.push_back(edgeDist);
					}
				}
				else if (u->Flag() == 1)
				{
					if (v->Flag() >=0 && v->Flag() <= 2)
					{
						u->adjacentvList.push_back(v);
						v->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						v->adjacentDist.push_back(edgeDist);
					}
				}
				else if (u->Flag() == 2)
				{
					if (v->Flag() >= 1 && v->Flag() <= 3)
					{
						u->adjacentvList.push_back(v);
						v->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						v->adjacentDist.push_back(edgeDist);
					}
				}
				else if (u->Flag() == 3)
				{
					if (v->Flag() >= 2 && v->Flag() <= 4)
					{
						u->adjacentvList.push_back(v);
						v->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						v->adjacentDist.push_back(edgeDist);
					}

					if (v->Flag() == 0 && cycleFlag)
					{
						size_t tmpIdx = 0;
						for (size_t i = 0; i < startvList.size(); i++)
						{
							if (startvList[i] == v)
							{
								tmpIdx = i; break;
							}
						}
						u->adjacentvList.push_back(endvList[tmpIdx]);
						endvList[tmpIdx]->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						endvList[tmpIdx]->adjacentDist.push_back(edgeDist);
					}
				}
				else if (u->Flag() == 4)
				{
					if (v->Flag() >= 1 && v->Flag() <= 4)
					{
						u->adjacentvList.push_back(v);
						v->adjacentvList.push_back(u);

						u->adjacentDist.push_back(edgeDist);
						v->adjacentDist.push_back(edgeDist);
					}
				}
			}
		}
	}

	void BoundarySmoothing::FindAShortestPathToLabel(VertexList & shortestvList, VertexList & totalvList, Vertex * u, int label, bool labelOrNonLabel)
	{
		const double maxDoubleValue = 1e10;
		shortestvList.clear();
		VertexList vSet;
		VertexList path;
		vector<double> dist;

		for (size_t i = 0; i < totalvList.size(); i++)
		{
			totalvList[i]->visited = false;
			totalvList[i]->tmpIndex = (int)i;
		}
		dist.resize(totalvList.size(), maxDoubleValue);
		path.resize(totalvList.size(), NULL);

		dist[u->tmpIndex] = 0.0;

		while (vSet.size()<totalvList.size())
		{
			int tmpIdx = -1;
			for (size_t i = 0; i < totalvList.size(); i++)
			{
				if (totalvList[i]->visited) continue;
				if ((dist[i] < maxDoubleValue) && (tmpIdx < 0 || dist[i] < dist[tmpIdx]))
					tmpIdx = i;
			}
			if (tmpIdx >= 0)
			{
				//cout << tmpIdx << " " << dist[tmpIdx] << endl;
				Vertex * v = totalvList[tmpIdx];
				vSet.push_back(v);
				v->visited = true;
		
				bool findAPath = false;
				OneRingVertex tmpRing(v);
				Vertex * tmpv = NULL;
				while ((tmpv = tmpRing.NextVertex()) != NULL)
				{
					// update distance
					if (tmpv->possibleBoundary && !tmpv->visited)
					{
						float tmpDist = dist[v->tmpIndex]+(tmpv->Position() - v->Position()).L2Norm();
						if (tmpDist < dist[tmpv->tmpIndex])
						{
							dist[tmpv->tmpIndex] = tmpDist;
							path[tmpv->tmpIndex] = v;
						}
					}
					
					// judge whether v is a boundary vertex
					if (tmpv->possibleBoundary) continue;
					Face * f = tmpv->HalfEdge()->LeftFace();
					if (f == NULL) continue;

					if (labelOrNonLabel)
					{
						if (f->fuzzy_label == label)
						{
							// find a path
							findAPath = true;
						}
					}
					else
					{
						if (f->fuzzy_label != label)
						{
							// find a path
							findAPath = true;
						}
					}
				}
				if (findAPath)
				{
					size_t idx = tmpIdx;
					while (path[idx] != NULL)
					{
						shortestvList.push_back(totalvList[idx]);
						idx = path[idx]->tmpIndex;
					}
					if (labelOrNonLabel == false)
						reverse(shortestvList.begin(), shortestvList.end());
					return;
				}
			}
			else
				break;
		}
	}
	
	void BoundarySmoothing::FindShortestBoundary(VertexList & startvList, VertexList & endvList, VertexList & possibleBoundary, VertexList & optBoundaryvList)
	{
		// algorithm: dijkstra

		const double maxDoubleValue = 1e10;

		float optDistance = maxDoubleValue;

		for (size_t k = 0; k < startvList.size(); k++)
		{
			Vertex * startv = startvList[k];
			Vertex * endv = endvList[k];

			VertexList vSet;
			VertexList path;
			vector<double> dist;

			VertexList tmpBoundaryvList;

			for (size_t i = 0; i < possibleBoundary.size(); i++)
			{
				possibleBoundary[i]->visited = false;
				possibleBoundary[i]->tmpIndex = (int)i;
			}

			path.resize(possibleBoundary.size(), NULL);
			dist.resize(possibleBoundary.size(), maxDoubleValue);

			dist[startv->tmpIndex] = 0.0;

			bool success = false;
			while (vSet.size() < possibleBoundary.size())
			{
				int tmpIdx = -1;
				for (size_t i = 0; i < possibleBoundary.size(); i++)
				{
					Vertex * v = possibleBoundary[i];
					if (v->visited) continue;

					if ((dist[v->tmpIndex] < maxDoubleValue) && (tmpIdx < 0 || dist[v->tmpIndex] < dist[tmpIdx]))
						tmpIdx = i;
				}

				if (tmpIdx >= 0)
				{
					Vertex * u = possibleBoundary[tmpIdx];
					u->visited = true;
					vSet.push_back(u);

					

					if (u == endv)
					{
						// find a shortest path
						success = true;
						int idx = endv->tmpIndex;
						while (path[idx] != NULL)
						{
							tmpBoundaryvList.push_back(path[idx]);
							idx = path[idx]->tmpIndex;
						}
						reverse(tmpBoundaryvList.begin(), tmpBoundaryvList.end());
						break;
					}
					else
					{
						// update
						for (size_t i = 0; i < u->adjacentvList.size(); i++)
						{
							Vertex * v = u->adjacentvList[i];
							if (v->visited) continue;

							double tmpDist = dist[u->tmpIndex] + u->adjacentDist[i];
							if (tmpDist < dist[v->tmpIndex])
							{
								dist[v->tmpIndex] = tmpDist;
								path[v->tmpIndex] = u;
							}
						}

					}
				}
				else
					break;
			}

			if (success)
			{
				if (dist[endv->tmpIndex] < optDistance)
				{
					optDistance = dist[endv->tmpIndex];
					optBoundaryvList.clear();
					optBoundaryvList.assign(tmpBoundaryvList.begin(), tmpBoundaryvList.end());
				}
			}
		}
	}

	void BoundarySmoothing::AssignNewLabel(Mesh & mesh, VertexList & optBoundaryvList, VertexList & possibleBoundary, BoundarySmoothingParameters & para)
	{
		const double maxDoubleValue = 1e10;
		if (optBoundaryvList.empty()) return;

		for (size_t i = 0; i < possibleBoundary.size(); i++)
		{
			possibleBoundary[i]->SetFlag(0);
		}

		for (size_t i = 0; i < optBoundaryvList.size(); i++)
		{
			optBoundaryvList[i]->SetFlag(1);
		}

		// find all possible face which fuzzy label maybe change
		FaceList possiblefList;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			f->flag = 0;
			f->visited = false;
			bool flag = true;
			for (int j = 0; j < 3; j++)
			{
				Vertex * v = mesh.vList[f->v[j]];
				if (!v->possibleBoundary)
				{
					flag = false;
					break;
				}
			}
			if (flag)
			{
				f->flag = 1;
				possiblefList.push_back(f);
			}
		}

		for (size_t i = 0; i < mesh.heList.size(); i++)
		{
			mesh.heList[i]->SetFlag(false);
		}
		for (size_t i = 0; i < optBoundaryvList.size(); i++)
		{
			Vertex * v1 = optBoundaryvList[i];
			Vertex * v2 = optBoundaryvList[(i + 1) % optBoundaryvList.size()];
			OneRingHEdge ring(v1);
			HEdge * he = NULL;
			while ((he = ring.NextHEdge()) != NULL)
			{
				if (he->End() == v2)
				{
					he->SetFlag(true);
					he->Twin()->SetFlag(true);
					break;
				}
			}
		}

		// assign new label for each face in possiblefList

		for (size_t i = 0; i < possiblefList.size(); i++)
		{
			Face * f = possiblefList[i];
			// bfs;
			FaceList tmpfList;
			Face * nearestface = NULL;
			double distance = maxDoubleValue;
			f->visited = true;
			tmpfList.push_back(f);
			size_t start = 0, end;
			while (start < tmpfList.size())
			{
				end = tmpfList.size();
				for (size_t j = start; j < end; j++)
				{
					Face * tmpf = tmpfList[j];
					if (tmpf->flag == 0) continue;
					double tmpDist = (tmpf->center - f->center).L2Norm();
					if (tmpDist >= distance) continue;

		
					HEdge * he = tmpf->HalfEdge();
					
					int count = 0;
					do
					{
						if (count >= 3) break;
						count++;
						// he can't be the optBoundary
						if (he->Flag() == false)
						{
							Face * adjacentFace = he->Twin()->LeftFace();

							if (adjacentFace != NULL && !adjacentFace->visited)
							{
								adjacentFace->visited = true;
								tmpfList.push_back(adjacentFace);

								// if adjacentFace isn't in possiblefList
								if (adjacentFace->flag == 0)
								{
									double tmpDistance = (adjacentFace->center - f->center).L2Norm();
									if (adjacentFace->fuzzy_label == para.label)
									{
										distance = tmpDistance;
										nearestface = adjacentFace;
									}
									else if (adjacentFace->fuzzy_label == 0 && (nearestface == NULL || nearestface->fuzzy_label != para.label))
									{
										distance = tmpDistance;
										nearestface = adjacentFace;
									}
									else if (tmpDistance < distance)
									{
										distance = tmpDistance;
										nearestface = adjacentFace;
									}
								}
							}
						}
						he = he->Next();
						
					} while (he != f->HalfEdge());

				}
				start = end;
			}

			// assign new label
			if (nearestface)
			{
				f->fuzzy_label = nearestface->fuzzy_label;
			}

			// clear tmpfList
			for (size_t j = 0; j < tmpfList.size(); j++)
			{
				tmpfList[j]->visited = false;
			}
		}
	}

	double BoundarySmoothing::CalcEdgeDistance(HEdge * he, BoundarySmoothingParameters & para)
	{
		double euclidDist = (he->Start()->Position() - he->End()->Position()).L2Norm();
		
		double tmp = (2 - he->angDist)*0.5;

		double angDist = he->angDist<para.angDistTh ? para.yita*tmp : tmp;

		double dist = euclidDist + para.lambda * angDist;
		
		return dist;
	}

	bool BoundarySmoothing::IsCycle(VertexList & boundary)
	{
		if (boundary.size()<3) return false;

		Vertex * u = boundary[0];
		Vertex * v = boundary.back();
		OneRingVertex ring(u);
		Vertex * vv = NULL;
		while ((vv = ring.NextVertex()) != NULL)
		{
			if (vv == v)
			{
				return true;
				break;
			}
		}

		return false;
	}

	double BoundarySmoothing::IsPossibleBoundary(VertexList & boundary, int idx, bool cycleFlag)
	{
		double ans = -1;
		Vertex * u = boundary[idx];
		if (cycleFlag)
		{
			for (int i = 1; i < 3; i++)
			{
				Vertex * v1 = boundary[(idx - i + boundary.size()) % boundary.size()];
				Vertex * v2 = boundary[(idx + i) % boundary.size()];

				Vector3d e1 = v1->Position() - u->Position();
				Vector3d e2 = v2->Position() - u->Position();
				e1.Normalize();
				e2.Normalize();
				double tmp = e1.Dot(e2);
				ans = max(ans, tmp);
			}
		}
		else
		{
			for (int i = 1; i < 3; i++)
			{
				int v1Idx = idx - i;
				int v2Idx = idx + i;
				if (v1Idx < 0) break;
				if (v2Idx >= boundary.size()) break;

				Vertex * v1 = boundary[v1Idx];
				Vertex * v2 = boundary[v2Idx];

				Vector3d e1 = v1->Position() - u->Position();
				Vector3d e2 = v2->Position() - u->Position();
				e1.Normalize();
				e2.Normalize();
				double tmp = e1.Dot(e2);
				ans = max(ans, tmp);
			}
		}
		return ans;
	}
}
