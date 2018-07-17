#include "ToothSegmentation.h"

namespace MeshSegmentation
{
	void CallBack(const char * msg)
	{
		cout << msg;
	}

	void MeshSimplify::Simplify(string modelName, string simplifyModelName, SimplifyParameters & para)
	{
		Mesh mesh;
		if (!mesh.LoadModel(modelName.c_str())) return;
		Simplify(mesh, para);
		SaveOBJ(simplifyModelName);

		decimatedPoints.clear();
		decimatedPoints.shrink_to_fit();
		decimatedtriangles.clear();
		decimatedtriangles.shrink_to_fit();
		decimatedLabels.clear();
		decimatedLabels.shrink_to_fit();
	}

	void MeshSimplify::Simplify(Mesh & inputMesh, string simplifyModelName, SimplifyParameters & para)
	{
		Simplify(inputMesh, para);
		SaveOBJ(simplifyModelName);

		decimatedPoints.clear();
		decimatedPoints.shrink_to_fit();
		decimatedtriangles.clear();
		decimatedtriangles.shrink_to_fit();
		decimatedLabels.clear();
		decimatedLabels.shrink_to_fit();
	}

	void MeshSimplify::Simplify(Mesh & inputMesh, Mesh & outputMesh, SimplifyParameters & para)
	{
		Simplify(inputMesh, para);
		vector<Vector3d> points;
		vector<Vector3i> faces;
		for (size_t i = 0; i < decimatedPoints.size(); i++)
		{
			points.push_back(Vector3d((double)decimatedPoints[i][0], (double)decimatedPoints[i][1], (double)decimatedPoints[i][2]));
		}

		for (size_t i = 0; i < decimatedtriangles.size(); i++)
		{
			faces.push_back(Vector3i(decimatedtriangles[i][0], decimatedtriangles[i][1], decimatedtriangles[i][2]));
		}

		outputMesh.LoadVectorData(points, faces);
		for (size_t i = 0; i < outputMesh.fList.size(); i++)
		{
			outputMesh.fList[i]->SetLabel(decimatedLabels[i]);
		}
	}

	void MeshSimplify::Simplify(Mesh & mesh, SimplifyParameters & para)
	{
		/*
		if (!para.labelBased)
		{
			mesh.CalculateSimplifiyLevel(para.modelType);
			RoughlyClassifiyTooth(mesh, para);
		}

		mesh.SetVertexSimplifyLevel(para.labelBased);*/

		for (size_t i = 0; i < mesh.vList.size(); i++)
		{
			if(mesh.vList[i]->simplify_level == -1)
				mesh.vList[i]->simplify_level = 0;
		}

		vector<Vector3d> mvList;	// vertex list
		vector<Vector3i> mfList;	// face list
		vector<int> mlList;			// label list
		vector<int> msLevelList;	// simplify level list, which is corresponding to simplify weights

		
		mesh.ExportMaxConnectedComponent2(mvList, mfList, mlList, msLevelList);

		vector< MeshDecimation::Vec3<float> > points;
		vector< MeshDecimation::Vec3<int> > triangles;
		vector< int > labels;
		for (size_t i = 0; i < mvList.size(); i++)
		{
			MeshDecimation::Vec3<float> x;
			for (size_t j = 0; j < 3; j++)
			{
				x[j] = (float)mvList[i][j];
			}
			points.push_back(x);
		}

		for (size_t i = 0; i < mfList.size(); i++)
		{
			MeshDecimation::Vec3<int> ip;
			for (size_t j = 0; j < 3; j++)
			{
				ip[j] = mfList[i][j];
			}
			triangles.push_back(ip);
		}

		int target_count = (int)round(mfList.size()*para.d_ratio);
		double maxDecimationError = 1.0;

		// time
		time_t   first, second;
		first = time(NULL);
		// decimate mesh
		MeshDecimation::MeshDecimator myMDecimator;
		myMDecimator.SetCallBack(&CallBack);
		myMDecimator.Initialize(points.size(), triangles.size(), para.simplifyWeights.size(), &points[0], &triangles[0], &mlList[0], &msLevelList[0], &para.simplifyWeights[0]);
		//myMDecimator.Initialize(points.size(), triangles.size(), &points[0], &triangles[0]);
		
		myMDecimator.Decimate(0,
			target_count,
			maxDecimationError);

		second = time(NULL);
		std::cout << "Mesh Simplification Time: " << difftime(second, first) << " seconds\n" << std::endl;

		//output
		decimatedPoints.resize(myMDecimator.GetNVertices());
		decimatedtriangles.resize(myMDecimator.GetNTriangles());
		decimatedLabels.resize(myMDecimator.GetNTriangles());

		// retreive decimated mesh
		myMDecimator.GetMeshData(&decimatedPoints[0], &decimatedtriangles[0], &decimatedLabels[0]);
	}

	void MeshSimplify::RoughlyClassifiyTooth(Mesh & mesh, SimplifyParameters & para)
	{
		// build smooth term
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->adjcent_face.clear();
			mesh.fList[i]->adjcent_weight.clear();
			mesh.fList[i]->tmpIdx = i;
		}

		float dist_all = 0.0f; int count = 0;
		vector<vector<float>> ang_dist_list_all;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			vector<float> ang_dist_list;
			HEdge * he = f->HalfEdge();
			do{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					f->adjcent_face.push_back(adjcentFace);
					double dotValue = f->normal.Dot(adjcentFace->normal);
					dotValue = dotValue>1.0 ? 1.0 : dotValue;
					dotValue = dotValue < -1.0 ? -1.0 : dotValue;

					float ang_dist;
					Vector3d c2c1 = adjcentFace->center - f->center;
					c2c1.Normalize();

					float tt = (float)(1 - dotValue);
					if (c2c1.Dot(f->normal) > 0)
					{
						ang_dist = tt;
					}
					else
					{
						ang_dist = para.yita *tt;
					}

					ang_dist_list.push_back(ang_dist);
					dist_all += ang_dist; count++;

				}

				he = he->Next();
			} while (he != f->HalfEdge());

			ang_dist_list_all.push_back(ang_dist_list);
		}

		float avg_ang_dist = dist_all / count;
		//cout << "avg " << avg_ang_dist << endl;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			for (size_t j = 0; j < f->adjcent_face.size(); j++)
			{
				Face * adjcentFace = f->adjcent_face[j];
				float cap = 1.0f / (1.0f + exp(ang_dist_list_all[i][j] / avg_ang_dist));

				HEdge * he = f->HalfEdge();
				do{
					if (he->Twin()->LeftFace() == adjcentFace)
						break;
					he = he->Next();
				} while (he != f->HalfEdge());

				Vertex * v1 = he->Start(), *v2 = he->End();
				if (v1->seed_flag && v1->seed_flag)
				{
					if (v1->cutting_point && v2->cutting_point)
						cap *= 5.0f;
					else if (v1->protect_flag && v2->protect_flag)
						cap *= 2.0f;
					else
						cap *= 0.1f;
				}


				mesh.fList[i]->adjcent_weight.push_back((int)(para.lambda*rate*cap));
			}
		}

		// build data term
		Vector3d maxCoord = mesh.maxCoord;
		Vector3d minCoord = mesh.minCoord;
		Vector3d centerCoord = 0.5*(maxCoord + minCoord);

		float xth = 0.2f;
		float yth = -0.7f;
		float xyth = sqrt(xth*xth + yth*yth);

		Vector3d diffCoord = maxCoord - minCoord;
		for (int i = 0; i < 3; i++)
		{
			diffCoord[i] = 1.0 / diffCoord[i];
		}
		float diffLocal = 1.0f / mesh.localBFSLevel;
		float maxProb = 0.90f, minProb = 0.1f;
		float source_weight, sink_weight;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			float w1 = 0.0f;
			if(para.modelType == 'U')
				w1 = (float)((f->center[2] - minCoord[2]) * diffCoord[2]);
			else if (para.modelType == 'L')
				w1 = (float)((maxCoord[2] - f->center[2]) * diffCoord[2]);
			float w2 = (float)(1.0f - mesh.vList[f->v[0]]->local_heightest *diffLocal);
			float w3 = 0.0f;

			float xdist = 2.0f*(float)(abs(f->center[0] - centerCoord[0]) * diffCoord[0]);
			float ydist = 2.0f*(float)((f->center[1] - centerCoord[1]) * diffCoord[1]);


			xdist = max(xdist, xth);
			ydist = max(ydist, 0.0f);

			if (ydist>yth && ydist<0.0f)
			{
				w3 = -abs(xyth - sqrt(ydist*ydist + xdist + xdist));
			}



			source_weight = para.graphCutWeights[0] * w1 + para.graphCutWeights[1] * w2 + para.graphCutWeights[2] * w3;
			sink_weight = 1 - source_weight;
			source_weight = min(source_weight, maxProb);
			source_weight = max(source_weight, minProb);

			sink_weight = min(sink_weight, maxProb);
			sink_weight = max(sink_weight, minProb);

			f->source_weight = (int)(rate*source_weight);
			f->sink_weight = (int)(rate*sink_weight);
		}

		// use maxflow
		Graph<int, int, int> * maxflowPtr = UseMaxflow(mesh.fList);

		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->tmpLabel = maxflowPtr->what_segment(i) == 0 ? 1 : 0;
		}
	}

	bool MeshSimplify::SaveOBJ(string & fileName)
	{
		cout << "Saving " << fileName << endl;
		ofstream fout(fileName.c_str());
		if (fout.is_open())
		{
			const size_t nV = decimatedPoints.size();
			const size_t nT = decimatedtriangles.size();
			for (size_t v = 0; v < nV; v++)
			{
				fout << "v " << decimatedPoints[v][0] << " "
					<< decimatedPoints[v][1] << " "
					<< decimatedPoints[v][2] << endl;
			}
			for (size_t f = 0; f < nT; f++)
			{
				fout << "f " << decimatedtriangles[f][0] + 1 << " "
					<< decimatedtriangles[f][1] + 1 << " "
					<< decimatedtriangles[f][2] + 1 << endl;
			}
			fout.close();
			return true;
		}
		return false;
	}

	void MeshSimplify::SaveLabel(string & filename)
	{
		filename = filename.substr(0, filename.length() - 4) + ".txt";
		std::ofstream fout(filename);
		if (fout.fail())
			return;

		for (size_t i = 0; i < decimatedLabels.size(); i++)
			fout << i + 1 << " " << decimatedLabels[i] << endl;
		fout.close();
	}
}