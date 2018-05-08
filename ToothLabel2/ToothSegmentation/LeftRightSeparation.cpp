#include "ToothSegmentation.h"
#include "../alglib\dataanalysis.h"

using namespace alglib;

namespace MeshSegmentation
{
	void LeftRightSeparation::JudgeLeftRight(Mesh & mesh, LeftRightSeparationParameters & para)
	{
		cout << "Judge Left and Right" << endl;
		double minProb = para.minProb;
		double maxProb = 1 - minProb;
		double midProb = 0.5*(maxProb + minProb);
		if (mesh.fList.empty())
			return;

		vector<int> componentsForEachLabel;
		CountComponentsForEachLabel(mesh, componentsForEachLabel);
		int minLabel = 1;
		for (size_t i = 1; i < componentsForEachLabel.size(); i++)
		{
			if (componentsForEachLabel[i] >= 1)
			{
				minLabel = (int)i; break;
			}
		}

		FaceList fList;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			int nowLabel = f->r_label;

			if (nowLabel == 0) continue;	// gum

			if (nowLabel == minLabel)				// center incisor
			{
				fList.push_back(f);
			}
			else
			{
				if (para.modelType == 'U')
				{
					// upper tooth
					if (f->center[0] < 0)
						f->r_label += 8;
				}
				else
				{
					// lower tooth
					if (f->center[0] > 0)
						f->r_label += 8;
				}
			}
		}

		if (fList.size() > 0)
		{
			// Two Centeral Incisor
			Vector3d center, dir;
			if (TwoCentralIncisors(fList, center, dir, para))
			{
				//cout << "fjkafjaf" << endl;
				// assign tmpidx
				for (size_t i = 0; i < mesh.fList.size(); i++)
				{
					mesh.fList[i]->tmpIdx = -1;
				}

				for (size_t i = 0; i < fList.size(); i++)
				{
					fList[i]->tmpIdx = i;
				}

				// we can modify the function, and just specify the adjcent
				// weight among centeral incisors.
				//BuildAdjcentWeight(mesh, lambda);

				BuildAdjcentWeight(fList, para.lambda);

				float diff = maxProb - minProb;
				for (size_t i = 0; i < fList.size(); i++)
				{
					Face * f = fList[i];
					double tmp = (f->center-center).Dot(dir) / para.maxWidth;
					double prob = (tmp)*diff + midProb;
					prob = min(maxProb, prob);
					prob = max(minProb, prob);

					double prob_left, prob_right;
					prob_left = prob;
					prob_right = 1 - prob;


					f->source_weight = (int)(-rate*log(prob_right + 1e-20));
					f->sink_weight = (int)(-rate*log(prob_left + 1e-20));
				}


				// count edge count
				Graph<int, int, int> *maxflowPtr = UseMaxflow(fList);
				// result
				int predictLabel;
				for (size_t i = 0; i < fList.size(); i++)
				{
					Face * f = fList[i];
					if (para.modelType == 'U')
					{
						predictLabel = maxflowPtr->what_segment(f->tmpIdx) == 0 ? minLabel : minLabel+8;
					}
					else if (para.modelType == 'L')
					{
						predictLabel = maxflowPtr->what_segment(f->tmpIdx) == 0 ? minLabel + 8 : minLabel;
					}
					f->r_label = predictLabel;
				}

				delete maxflowPtr;
				maxflowPtr = NULL;
			}
		}
		// assign new prob
		Assign17ClassesProb(mesh, para.load2ClassesProb);

		// initalize fuzzy label, assign r_label to fuzzy_label
		// this code is important
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->fuzzy_label = mesh.fList[i]->r_label;
		}
	}

	void LeftRightSeparation::Assign17ClassesProb(Mesh & mesh, bool load2ClassesProb)
	{
		if (mesh.fList[0]->prob.size() < 9) return;
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			vector<float> tmpProb;
			tmpProb.assign(f->prob.begin() + 1, f->prob.end());
			if (f->r_label < 9)
			{
				for (size_t j = 0; j < 8; j++)
					f->prob.push_back(0);
			}
			else
			{
				for (size_t j = 1; j <= 8; j++)
					f->prob[j] = 0;
				for (size_t j = 0; j < 8; j++)
					f->prob.push_back(tmpProb[j]);
			}

			if (!load2ClassesProb)  continue;

			if (f->p_label_list[0] > 0 && f->r_label > 0) continue;
			if (f->p_label_list[0] == 0 && f->r_label == 0) continue;
			if (f->twoClassesProb.empty()) continue;
			f->prob[0] = f->twoClassesProb[0];
			for (size_t j = 1; j < f->prob.size(); j++)
			{
				f->prob[j] *= f->twoClassesProb[1];
			}
		}
	}

	bool LeftRightSeparation::TwoCentralIncisors(FaceList & fList, Vector3d & center, Vector3d & dir, LeftRightSeparationParameters & para)
	{
		if (fList.empty()) return false;

		center = Vector3d(0, 0, 0);
		dir = Vector3d(1, 0, 0);

		for (size_t i = 0; i < fList.size(); i++)
		{
			center += fList[i]->center;
		}
		center /= fList.size();

		// PCA Analysis
		// input parameters
		ae_int_t nPoints = fList.size();
		ae_int_t nVars = 2;
		real_2d_array x;

		// output paramenters
		ae_int_t info;
		real_1d_array s;
		real_2d_array v;
		// build


		x.setlength(nPoints, nVars);
		for (int i = 0; i < nPoints; i++)
		{
			for (int j = 0; j < nVars; j++)
			{
				x[i][j] = fList[i]->center[j];
			}
		}
		pcabuildbasis(x, nPoints, nVars, info, s, v);

		// analysis resutl
		Vector3d xAxis(1, 0, 0);
		dir[0] = v[0][0];
		dir[1] = v[1][0];
		if (dir.Dot(xAxis) < 0)
			dir = -dir;
		dir.Normalize();

		cout << "Centeral Incisors Width: " << s[0] << "		" << dir << endl;
		if (para.modelType == 'U')
		{
			if (s[0] < para.widthThresholdList[0]) return false;
		}
		else if (para.modelType == 'L')
		{
			if (s[0] < para.widthThresholdList[1]) return false;
		}
		else return false;

		return true;
	}

	void LeftRightSeparation::CountComponentsForEachLabel(Mesh & mesh, vector<int> & componentsForEachLabel)
	{
		componentsForEachLabel.resize(9, 0);
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			f->visited = false;
		}

		// BFS
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];
			if (f->visited) continue;

			int nowLabel = f->r_label;
			queue<Face *> fqueue;
			f->visited = true;
			fqueue.push(f);
			while (!fqueue.empty())
			{
				Face * nowFace = fqueue.front();
				fqueue.pop();
				HEdge * he = nowFace->HalfEdge();
				do
				{
					Face * adjacentFace = he->Twin()->LeftFace();
					if (adjacentFace != NULL && !adjacentFace->visited && adjacentFace->r_label == nowLabel)
					{
						adjacentFace->visited = true;
						fqueue.push(adjacentFace);
					}
					he = he->Next();
				} while (he != nowFace->HalfEdge());
			}

			if (nowLabel<componentsForEachLabel.size())
				componentsForEachLabel[nowLabel]++;
			else cout << "Error: lable is over 9" << endl;
		}
	}
}