#include "ToothSegmentation.h"
#include "..\alglib\dataanalysis.h"

using namespace alglib;

namespace MeshSegmentation
{
	void SplitJointedTeeth::BuildToothPCAList(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		toothPCAList.clear();
		toothPCAList.resize(17);
		for (size_t i = 0; i < toothPCAList.size(); i++)
		{
			toothPCAList[i].idx = (int)i;
		}
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			Face * f = mesh.fList[i];

			int label = 0;
			if (para.labelType == 'g')
			{
				label = labelMap[f->Label()];
			}
			else if (para.labelType == 'r')
			{
				label = f->r_label;
			}
			else if (para.labelType == 'f')
			{
				label = f->fuzzy_label;
			}
			
			toothPCAList[label].fList.push_back(f);
		}

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			map<int, int>::iterator iter;
			for (iter = labelMap.begin(); iter != labelMap.end(); iter++)
			{
				if (iter->second == i)
					break;
			}
			if (iter == labelMap.end())
				continue;

			toothPCAList[i].label = iter->first;
		}

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			toothPCAList[i].CalcCenter();
		}
	}

	void SplitJointedTeeth::ToothPCAAnalysis(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		//cout << "type: " << para.modelType << endl;
		BuildLabelMap(labelMap, para.modelType);

		BuildToothPCAList(mesh, toothPCAList, para);

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			cout << "tooth ID: " << toothPCAList[i].label << endl;

			toothPCAList[i].PCAAnalysis(labelMap);
			toothPCAList[i].CalcLengthForEachAxis();
			toothPCAList[i].CalcArea();

			cout << endl;
		}
	}

	void SplitJointedTeeth::DetectJointedTeeth(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		if (para.modelType != 'U' && para.modelType != 'L') return;
		if (para.labelType != 'r' && para.labelType!='f') return;
		if (thresholdList.empty()) return;
		

		ToothPCAAnalysis(mesh, toothPCAList, para);

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			toothPCA.FindAdjcentTooth(toothPCAList, para);
			toothPCA.SplitJointedTeeth(mesh, thresholdList, para);
			toothPCA.CorrectLabel(mesh, toothPCAList, thresholdList);
			if (toothPCA.AffectToothAnalysis(toothPCAList, labelMap) > para.affectToothCountTh)
			{
				toothPCA.tmpLabelList.clear();
			}
			if (toothPCA.tmpLabelList.size() > 0)
			{
				cout << "tmp label: ";
				for (size_t j = 0; j < toothPCA.tmpLabelList.size(); j++)
				{
					cout << " " << toothPCA.tmpLabelList[j];
				}
				cout << endl;
			}

			// show
			//if (toothPCA.tmpfList.empty())
			//{
			//	for (size_t j = 0; j < toothPCA.fList.size(); j++)
			//	{
			//		Face * f = toothPCA.fList[j];
			//		f->tmpLabel = f->fuzzy_label;
			//	}
			//}
			//else
			//{
			//	for (size_t j = 0; j < toothPCA.tmpfList.size(); j++)
			//	{
			//		for (size_t k = 0; k < toothPCA.tmpfList[j].size(); k++)
			//		{
			//			Face * f = toothPCA.tmpfList[j][k];
			//			f->tmpLabel = f->fuzzy_label + j + 3;
			//		}
			//	}
			//}
		}

		for (int k = 0; k < 2; k++)
		{
			size_t start, end;
			if (k == 0)
				start = 1;
			else
				start = 9;
			end = start + 7;

			int totalHalfSideTooth = 0;
			for (size_t i = start; i <= end; i++)
			{
				ToothPCAInfo & toothPCA = toothPCAList[i];
				if (toothPCA.fList.empty()) continue;

				totalHalfSideTooth++;
				if (toothPCA.tmpLabelList.size() == 2)
					totalHalfSideTooth++;
			}
			//if (totalHalfSideTooth > 8) continue;

			int nowLabel = start;
			for (size_t i = start; i <= end; i++)
			{
				if (nowLabel-start+1 > para.maxLabelCount) break;

				ToothPCAInfo & toothPCA = toothPCAList[i];
				if (toothPCA.fList.empty()) continue;
				if (toothPCA.tmpLabelList.size() == 2 && toothPCA.tmpLabelList[0]!=toothPCA.tmpLabelList[1])
				{
					if (toothPCA.tmpLabelList[0] == -1)
					{
						nowLabel = max(toothPCA.idx, nowLabel + 1);
					}
					else
					{
						nowLabel = max(toothPCA.idx, nowLabel);
					}
					for (size_t t = 0; t < toothPCA.tmpLabelList.size(); t++)
					{
						for (size_t j = 0; j < toothPCA.tmpfList[t].size(); j++)
						{
							Face * f = toothPCA.tmpfList[t][j];
							f->fuzzy_label = nowLabel + toothPCA.tmpLabelList[t];
						}
					}

					nowLabel = toothPCA.tmpLabelList[0] == -1 ? nowLabel + 1 : nowLabel + 2;
				}
				else
				{
					if (nowLabel >= toothPCA.idx)
					{
						if (nowLabel > toothPCA.idx)
						{
							for (size_t j = 0; j < toothPCA.fList.size(); j++)
							{
								Face * f = toothPCA.fList[j];
								f->fuzzy_label = nowLabel;
							}
						}
						nowLabel++;
						continue;
					}
					else
					{
						int maxAdjPrevious = -1;
						for (size_t j = 0; j < toothPCA.adjcentToothList.size(); j++)
						{
							if (toothPCA.adjcentToothList[j]->idx < toothPCA.idx)
							{
								if (maxAdjPrevious == -1 || toothPCA.adjcentToothList[j]->idx > maxAdjPrevious)
									maxAdjPrevious = toothPCA.adjcentToothList[j]->idx;
							}
						}

						int maxDirectAdjPrevious = -1;
						for (size_t j = 0; j < toothPCA.directAdjcentToothList.size(); j++)
						{
							if (toothPCA.directAdjcentToothList[j]->idx < toothPCA.idx)
							{
								if (maxDirectAdjPrevious == -1 || toothPCA.directAdjcentToothList[j]->idx > maxDirectAdjPrevious)
									maxDirectAdjPrevious = toothPCA.directAdjcentToothList[j]->idx;
							}
						}

						int minAdjNext = -1;
						for (size_t j = 0; j < toothPCA.adjcentToothList.size(); j++)
						{
							if (toothPCA.adjcentToothList[j]->idx > toothPCA.idx)
							{
								if (minAdjNext == -1 || toothPCA.adjcentToothList[j]->idx < minAdjNext)
									minAdjNext = toothPCA.adjcentToothList[j]->idx;
							}
						}
						
						if (maxAdjPrevious>0 && maxAdjPrevious == maxDirectAdjPrevious && minAdjNext == -1)
						{
							
							nowLabel = maxAdjPrevious + 1;

							for (size_t j = 0; j < toothPCA.fList.size(); j++)
							{
								Face * f = toothPCA.fList[j];
								f->fuzzy_label = nowLabel;
							}
							nowLabel++;
						}
						else
						{
							nowLabel = toothPCA.idx + 1;
						}
					}
				}
			}
		}

		CorrectTheSecondPremolares(mesh, para);

	}

	void SplitJointedTeeth::DetectJointedTeethWithPrior(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		if (para.prior.empty())
		{
			cout << "There is no prior knowledge!" << endl;
			return;
		}

		if (para.modelType != 'U' && para.modelType != 'L') return;
		if (para.labelType != 'r' && para.labelType != 'f') return;
		if (thresholdList.empty()) return;


		ToothPCAAnalysis(mesh, toothPCAList, para);

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			toothPCA.FindAdjcentTooth(toothPCAList, para);
			toothPCA.SplitJointedTeeth(mesh, thresholdList, para);
		}

		SearchBestLabelling(toothPCAList, para);

	}

	void SplitJointedTeeth::SearchBestLabelling(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		// Assign initial labels
		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			if (toothPCA.tmpfList.size() == 0 && toothPCA.fList.size() > 0)
			{
				toothPCA.tmpLabelList.resize(1);
				toothPCA.tmpLabelList[0] = toothPCA.idx;
			}
			else if (toothPCA.tmpfList.size() == 2)
			{
				toothPCA.tmpLabelList.resize(2);
				toothPCA.tmpLabelList[0] = toothPCA.tmpLabelList[1] = toothPCA.idx;
			}
		}

		// Initial cost computation
		minCost = CalcLabellingCost(toothPCAList, para);
		AssignLabellingPlan(toothPCAList);

		// Search
		SearchLabelling(toothPCAList, para, 1, 1, 0);

		// Assign best labelling to all faces

	}

	int SplitJointedTeeth::CalcLabellingCost(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		int cost = 0;
		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			cost += CalcLabellingCostForEachTooth(toothPCA, para);
		}
		return cost;
	}

	int SplitJointedTeeth::CalcLabellingCostForEachTooth(ToothPCAInfo & toothPCA, SplitJointedTeethParameters & para)
	{
		if (toothPCA.tmpLabelList.empty()) return 0;

		if (toothPCA.tmpLabelList.size() == 1)
		{
			if (toothPCA.tmpLabelList[0] == toothPCA.idx)
			{
				if (para.prior[toothPCA.idx]) return 0;
				else return 2;
			}
			else return 1;
		}

		if (toothPCA.tmpLabelList.size() == 2)
		{
			if (toothPCA.tmpLabelList[0] == toothPCA.tmpLabelList[1])
			{
				if (toothPCA.tmpLabelList[0] == toothPCA.idx)
				{
					if (para.prior[toothPCA.idx]) return 2;
					else return 4;
				}
				else return 2;
			}
			else return 0;
		}

		return 0;
	}

	void SplitJointedTeeth::AssignLabellingPlan(ToothPCAInfoList & toothPCAList)
	{
		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			if (toothPCA.tmpLabelList.empty()) continue;
			toothPCA.bestLabelList.assign(toothPCA.tmpLabelList.begin(), toothPCA.tmpLabelList.end());
		}
	}

	void SplitJointedTeeth::SearchLabelling(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para, int toothID, int nowLabel, int cost)
	{
		// tootID: the current tooth index
		// nowLabel: the possible start label of the feasible labels
		
		if (cost > minCost) return;
		if (toothID == 17)
		{
			// finish searching
			AssignLabellingPlan(toothPCAList);
		}
		else
		{
			if (toothID == 9)
			{
				// This is a new central inscior, we must assign a new feasible nowLabel
				nowLabel = 9;
				for (size_t i = 1; i <= 8; i++)
				{
					for (size_t j = 0; j < toothPCAList[i].tmpLabelList.size(); j++)
					{
						nowLabel = max(nowLabel, toothPCAList[i].tmpLabelList[j] + 1);
					}
				}
			}

			ToothPCAInfo & toothPCA = toothPCAList[toothID];
			int tmpCost = 0;
			if (toothPCA.tmpLabelList.empty())
			{
				SearchLabelling(toothPCAList, para, toothID + 1, nowLabel, cost);
			}
			else if (toothPCA.tmpLabelList.size() == 1)
			{
				if (nowLabel <= toothPCA.idx && para.prior[toothPCA.idx])
				{
					SearchLabelling(toothPCAList, para, toothID + 1, nowLabel, cost);
				}
				else
				{
					int nextFeasibleLabel = FindNextFeasibleLabel(toothID, para);
					if (nextFeasibleLabel > 0)
					{
						toothPCA.tmpLabelList[0] = nextFeasibleLabel;
						tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
						SearchLabelling(toothPCAList, para, toothID + 1, nextFeasibleLabel + 1, cost + tmpCost);
					}
				}

				int previousFeasibleLabel = FindPreviousFeasibleLabel(nowLabel, toothID, para);
				if (previousFeasibleLabel > 0)
				{
					toothPCA.tmpLabelList[0] = previousFeasibleLabel;
					tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
					SearchLabelling(toothPCAList, para, toothID + 1, previousFeasibleLabel + 1, cost + tmpCost);
				}
			}
			else if (toothPCA.tmpLabelList.size() == 2)
			{
				if (nowLabel <= toothPCA.idx)
				{
					if (para.prior[toothPCA.idx])
					{
						// find one prevoius feasible label
						int previousFeasibleLabel = FindPreviousFeasibleLabel(nowLabel, toothID, para);
						if (previousFeasibleLabel > 0)
						{
							toothPCA.tmpLabelList[0] = previousFeasibleLabel;
							toothPCA.tmpLabelList[1] = toothID;
							tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
							SearchLabelling(toothPCAList, para, toothID + 1, toothID+ 1, cost + tmpCost);
						}
						// find one next feasible label
						int nextFeasibleLabel = FindNextFeasibleLabel(toothID, para);
						if (nextFeasibleLabel > 0)
						{
							toothPCA.tmpLabelList[0] = toothID;
							toothPCA.tmpLabelList[1] = nextFeasibleLabel;
							tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
							SearchLabelling(toothPCAList, para, toothID + 1, nextFeasibleLabel + 1, cost + tmpCost);
						}
					}
					else
					{
						// find two previous feasible labels
						int previousFeasibleLabel1 = FindPreviousFeasibleLabel(nowLabel, toothID, para);
						if (previousFeasibleLabel1 > 0)
						{
							int previousFeasibleLabel2 = FindPreviousFeasibleLabel(nowLabel, previousFeasibleLabel1, para);
							if (previousFeasibleLabel2 > 0)
							{
								toothPCA.tmpLabelList[0] = previousFeasibleLabel2;
								toothPCA.tmpLabelList[1] = previousFeasibleLabel1;
								tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
								SearchLabelling(toothPCAList, para, toothID + 1, previousFeasibleLabel1 + 1, cost + tmpCost);
							}
						}
					}
				}
				else
				{
					// find two next feasible labels
					int nextFeasibleLabel1 = FindNextFeasibleLabel(toothID, para);
					if (nextFeasibleLabel1 > 0)
					{
						int nextFeasibleLabel2 = FindNextFeasibleLabel(nextFeasibleLabel1, para);
						if (nextFeasibleLabel2 > 0)
						{
							toothPCA.tmpLabelList[0] = nextFeasibleLabel1;
							toothPCA.tmpLabelList[1] = nextFeasibleLabel2;
							tmpCost = CalcLabellingCostForEachTooth(toothPCA, para);
							SearchLabelling(toothPCAList, para, toothID + 1, nextFeasibleLabel2 + 1, cost + tmpCost);
						}
					}

				}
			}
		}
	}

	int SplitJointedTeeth::FindPreviousFeasibleLabel(int nowLabel, int startLabel, SplitJointedTeethParameters & para)
	{
		if (startLabel != 1 && startLabel != 9)
		{
			for (size_t i = startLabel - 1; i >= nowLabel; i--)
			{
				if (para.prior[i]) return (int)i;
			}
		}

		if (startLabel <= 8)
		{
			for (size_t i = 9; i <= 16; i++)
			{
				if (para.prior[i]) return (int)i;
			}
		}

		return -1;
	}

	int SplitJointedTeeth::FindNextFeasibleLabel(int startLabel, SplitJointedTeethParameters & para)
	{
		int endLabel = startLabel > 8 ? 16 : 8;
		for (size_t i = startLabel + 1; i <= endLabel; i++)
		{
			if (para.prior[i]) return (int)i;
		}

		// if can't find a next feaible label
		return -1;
	}

	void SplitJointedTeeth::CorrectTheSecondPremolares(Mesh & mesh, SplitJointedTeethParameters & para)
	{
		ToothPCAInfoList toothPCAList;

		BuildToothPCAList(mesh, toothPCAList, para);
		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			toothPCAList[i].FindAdjcentTooth(toothPCAList, para);
		}


		for (int k = 0; k < 2; k++)
		{
			int idx;
			if (k == 0)
				idx = 4;
			else
				idx = 12;

			ToothPCAInfo & toothPCA = toothPCAList[idx];

			int minNextIdx = 1000;
			for (size_t i = 0; i < toothPCA.directAdjcentToothList.size(); i++)
			{
				ToothPCAInfo & adjcentToothPCA = *toothPCA.directAdjcentToothList[i];
				if (adjcentToothPCA.idx>toothPCA.idx)
				{
					minNextIdx = min(minNextIdx, adjcentToothPCA.idx);
				}
			}

			if (minNextIdx == idx + 2)
			{
				cout << "Dicover wrong second premolares: " << toothPCA.label << endl;
				int nowLabel = minNextIdx - 1;
				for (size_t j = 0; j < toothPCA.fList.size(); j++)
				{
					Face * f = toothPCA.fList[j];
					f->fuzzy_label = nowLabel;
				}
			}
		}

	}

	void SplitJointedTeeth::ToothBoundaryPCAAnalysis(Mesh & mesh, ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		if (toothPCAList.empty()) return;

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			toothPCAList[i].FindAdjcentTooth(toothPCAList, para);
		}

		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			if (toothPCA.directAdjcentToothList.empty())
			{
				toothPCA.maxBoundaryPCALengthWithOtherTooth = 0;
				continue;
			}

			toothPCA.maxBoundaryPCALengthWithOtherTooth = 0.0;
			for (size_t j = 0; j < toothPCA.directAdjcentToothList.size(); j++)
			{
				double tmp = toothPCA.BoundaryPCAAnalysisWithOtherTooth(mesh, toothPCA.fList, toothPCA.directAdjcentToothList[j]->fList);
				toothPCA.maxBoundaryPCALengthWithOtherTooth = max(toothPCA.maxBoundaryPCALengthWithOtherTooth, tmp);
			}
			//cout << "boundary: " << i << " " << toothPCA.boundaryPCAMaxLengthWithOtherTooth << endl;
		}
	}

	bool SplitJointedTeeth::LoadPCAData(string filename)
	{
		ifstream ifs(filename);
		if (ifs.fail())
			return false;

		thresholdList.clear();

		char buf[1024];
		do
		{
			ifs.getline(buf, 1024);
			istrstream iss(buf);

			int x = -1;
			float y, z, w;
			iss >> x >> y >> z >> w;
			if (x > 0)
			{
				PCAThreshold th(x, y, z, w);
				thresholdList.push_back(th);
			}
		} while (!ifs.eof());

		ifs.close();
		return true;
	}

	bool SplitJointedTeeth::SavePCAInfo(string filename, ToothPCAInfoList & tooothPCAList)
	{
		filename = filename.substr(0, filename.length() - 4) + "_pca.txt";
		if (tooothPCAList.empty()) return false;

		ofstream fout(filename);
		if (fout.fail())
			return false;

		for (size_t i = 1; i < tooothPCAList.size(); i++)
		{
			ToothPCAInfo & ti = tooothPCAList[i];
			if (ti.fList.empty()) continue;
			fout << ti.label;
			for (size_t j = 0; j < ti.lengthList.size(); j++)
				fout << " " << ti.lengthList[j];
			fout << " " << ti.area << " " << ti.projectToXYPlaneLength << " " << ti.maxBoundaryPCALengthWithOtherTooth << endl;
		}
		fout.close();

		cout << filename << " saved!" << endl;

		return true;
	}
	
	void ToothPCAInfo::PCAAnalysis(map<int, int> & labelMap)
	{
		if (fList.empty()) return;
		// input parameters
		ae_int_t nPoints = fList.size();
		ae_int_t nVars = 3;
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

		// save ans
		for (int i = 0; i < 3; i++)
		{
			varList.push_back(s[i]);
			//cout << varList[i] << " ";
		}
		//cout << endl;
		Vector3d axis(0, 3, 0);
		if ((label >= 11 && label <= 18) || (label >= 41 && label <= 48))
			axis[0] = 1;	// x axis is positive
		else
			axis[0] = -1;	// x axis is negative
		
		axis.Normalize();
		
		for (int i = 0; i < 3; i++)
		{
			Vector3d tmp;
			for (int j = 0; j < 3; j++)
			{
				tmp[j] = v[j][i];
			}
			if (tmp.Dot(axis) < 0)
				tmp = -tmp;

			axisList.push_back(tmp);
			//cout << "axis " << i << ": " << tmp << " " << endl;
		}
	}

	void ToothPCAInfo::CalcLengthForEachAxis()
	{
		if (fList.empty()) return;
		

		obbCenter = center;

		Vector3d zAxis(0, 0, 1);
		projectToXYPlaneLength = 0.0;
		projectTOXYPlaneAxisIdx = 0;
		for (size_t k = 0; k < axisList.size(); k++)
		{
			double minValue = 1e10, maxValue = -1e10;
			Vector3d dir = axisList[k];
			dir.Normalize();

			for (size_t i = 0; i < fList.size(); i++)
			{
				double dotValue = (fList[i]->center - center).Dot(dir);

				minValue = min(minValue, dotValue);
				maxValue = max(maxValue, dotValue);
			}
			double len = maxValue - minValue;
			lengthList.push_back(len);

			obbCenter += 0.5*(maxValue + minValue)*dir;

			double theta = acos(dir.Dot(zAxis));
			float tmpLen = len*sin(theta);
			if (tmpLen > projectToXYPlaneLength)
			{
				projectToXYPlaneLength = tmpLen;
				projectTOXYPlaneAxisIdx = k;
			}
		}

		cout << "projectTOXYPlaneAxisIdx: " << projectTOXYPlaneAxisIdx << endl;
		//cout << "center: " << center << endl;
		
		cout << "obb center: " << obbCenter << endl;
		cout << "length: ";
		for (size_t i = 0; i < lengthList.size(); i++)
		{
			cout << lengthList[i] << " ";
		}
		cout << endl;
		cout << "volume: " << lengthList[0] * lengthList[1] * lengthList[2] << endl;
		cout << "project to xy plane: " << projectToXYPlaneLength << endl;
		
	}

	void ToothPCAInfo::CalcArea()
	{
		area = 0.0f;
		if (fList.empty()) return;
		for (size_t i = 0; i < fList.size(); i++)
		{
			area += fList[i]->area;
		}
		//cout << "area: " << area << endl;
	}

	double ToothPCAInfo::BoundaryPCAAnalysisWithOtherTooth(Mesh & mesh, FaceList & fList1, FaceList & fList2)
	{
		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->tmpIdx = -1;
		}
		for (size_t i = 0; i < mesh.vList.size(); i++)
		{
			mesh.vList[i]->visited = false;
		}

		for (size_t i = 0; i < fList1.size(); i++)
		{
			fList1[i]->tmpIdx = 1;
		}
		for (size_t i = 0; i < fList2.size(); i++)
		{
			fList2[i]->tmpIdx = 2;
		}


		VertexList boundaryvList;
		for (size_t i = 0; i < fList1.size(); i++)
		{
			Face * f = fList1[i];
			HEdge *he = f->HalfEdge();
			do
			{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					if (adjcentFace->tmpIdx == 2)
					{
						if (!he->Start()->visited)
						{
							boundaryvList.push_back(he->Start());
							he->Start()->visited = true;
						}
						if (!he->End()->visited)
						{
							boundaryvList.push_back(he->End());
							he->End()->visited = true;
						}
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}

		// if the boundary doesn't exist, return 0
		if (boundaryvList.size() < 2) return 0;

		// input parameters
		ae_int_t nPoints = boundaryvList.size();
		ae_int_t nVars = 3;
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
				x[i][j] = boundaryvList[i]->Position()[j];
			}
		}


		pcabuildbasis(x, nPoints, nVars, info, s, v);

		// the max axis direction
		Vector3d dir(v[0][0], v[1][0], v[2][0]);

		// project all vertices to the dir and calc the length
		double minValue = 1e10, maxValue = -1e10;
		for (size_t i = 0; i < boundaryvList.size(); i++)
		{
			Vertex * v = boundaryvList[i];
			double tmp = v->Position().Dot(dir);
			minValue = min(minValue, tmp);
			maxValue = max(maxValue, tmp);
		}

		return maxValue - minValue;
	}

	void ToothPCAInfo::FindAdjcentTooth(ToothPCAInfoList & toothPCAList, SplitJointedTeethParameters & para)
	{
		if (fList.empty()) return;
		adjcentToothList.clear();
		directAdjcentToothList.clear();

		map<int, int> labelMap;
		BuildLabelMap(labelMap, para.modelType);

		FaceList boundaryfList;
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			HEdge * he = f->HalfEdge();
			do
			{
				if (he->Twin()->LeftFace() != NULL)
				{
					Face * adjcentFace = he->Twin()->LeftFace();
					int adjcentLabel;
					if (para.labelType == 'g')
					{
						adjcentLabel = labelMap[adjcentFace->Label()];
					}
					else if (para.labelType == 'r')
					{
						adjcentLabel = adjcentFace->r_label;
					}
					else if (para.labelType == 'f')
					{
						adjcentLabel = adjcentFace->fuzzy_label;
					}
					
					if (adjcentLabel != idx && adjcentLabel>0)
					{
						vector<ToothPCAInfo*>::iterator iter = find(directAdjcentToothList.begin(), directAdjcentToothList.end(), &toothPCAList[adjcentLabel]);
						if (iter == directAdjcentToothList.end())
						{
							directAdjcentToothList.push_back(&toothPCAList[adjcentLabel]);
						}
					}
				}
				he = he->Next();
			} while (he != f->HalfEdge());
		}

		// add label diff == 1
		adjcentToothList.assign(directAdjcentToothList.begin(), directAdjcentToothList.end());
		for (size_t i = 1; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			if (toothPCA.fList.empty()) continue;
			if (toothPCA.label == label) continue;
			if (abs(toothPCA.label - label ) > 1 && !(label%10==1 && toothPCA.label%10==1)) continue;

			vector<ToothPCAInfo*>::iterator iter = find(adjcentToothList.begin(), adjcentToothList.end(), &toothPCA);
			if (iter == adjcentToothList.end())
			{
				adjcentToothList.push_back(&toothPCA);
			}
		}

		// sort directAdjcentToothList
		for (size_t i = 0; i < directAdjcentToothList.size(); i++)
		{
			size_t idx = i;
			for (size_t j = i + 1; j < directAdjcentToothList.size(); j++)
			{
				if ((directAdjcentToothList[j]->label % 10) < (directAdjcentToothList[idx]->label % 10))
				{
					idx = j;
				}
			}
			if (idx == i) continue;
			ToothPCAInfo * tmp = directAdjcentToothList[i];
			directAdjcentToothList[i] = directAdjcentToothList[idx];
			directAdjcentToothList[idx] = tmp;
		}

		// sort adjcentToothList
		for (size_t i = 0; i < adjcentToothList.size(); i++)
		{
			size_t idx = i;
			for (size_t j = i + 1; j < adjcentToothList.size(); j++)
			{
				//cout << adjcentToothList[j]->label << " " << adjcentToothList[idx]->label << endl;
				if ((adjcentToothList[j]->label % 10) < (adjcentToothList[idx]->label % 10))
				{
					idx = j;
				}
			}
			if (idx == i) continue;
			ToothPCAInfo * tmp = adjcentToothList[i];
			adjcentToothList[i] = adjcentToothList[idx];
			adjcentToothList[idx] = tmp;
		}
		
		cout << "neighbor: " << adjcentToothList.size();
		for (size_t i = 0; i < adjcentToothList.size(); i++)
		{
			cout << " " << adjcentToothList[i]->label;
		}
		cout << endl;
	}

	bool ToothPCAInfo::SplitJointedTeeth(Mesh & mesh, vector<PCAThreshold> & thresholdList, SplitJointedTeethParameters & para)
	{
		if (fList.empty()) return false;
		if (lengthList.size() < 3) return false;
		if (lengthList[1] < 0.000001f) return false;

		int thresholdIdx = -1;
		for (size_t j = 0; j < thresholdList.size(); j++)
		{
			if (thresholdList[j].label == label)
			{
				thresholdIdx = j;
				break;
			}
		}
		if (thresholdIdx == -1) return false;
		PCAThreshold & th = thresholdList[thresholdIdx];

		if (projectToXYPlaneLength < th.maxProjectToXYPlaneLength)
			return false;
		
		cout << "Discover Jointed Tooth: " << th.label << endl;

		double minProb = para.minProb;
		double maxProb = 1 - minProb;
		double midProb = 0.5*(maxProb + minProb);

		for (size_t i = 0; i < mesh.fList.size(); i++)
		{
			mesh.fList[i]->tmpIdx = -1;
		}

		for (size_t i = 0; i < fList.size(); i++)
		{
			fList[i]->tmpIdx = i;
		}

		BuildAdjcentWeight(fList, para.lambda);
		float diff = maxProb - minProb;
		Vector3d & dir = axisList[projectTOXYPlaneAxisIdx];
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			double tmp = (f->center - obbCenter).Dot(dir) / lengthList[projectTOXYPlaneAxisIdx];
			double prob = (tmp)*diff + midProb;
			prob = min(maxProb, prob);
			prob = max(minProb, prob);

			double prob_bigIdx, prob_smallIdx;
			prob_bigIdx = prob;
			prob_smallIdx = 1 - prob;


			f->source_weight = (int)(-rate*log(prob_bigIdx + 1e-20));
			f->sink_weight = (int)(-rate*log(prob_smallIdx + 1e-20));
		}

		Graph<int, int, int> *maxflowPtr = UseMaxflow(fList);
		// result
		tmpfList.clear();
		tmpfList.resize(2);
		tmpLabelList = { 0, 1 };
		int predictLabel;
		for (size_t i = 0; i < fList.size(); i++)
		{
			Face * f = fList[i];
			predictLabel = maxflowPtr->what_segment(f->tmpIdx) == 0 ? 0 : 1;
			tmpfList[predictLabel].push_back(f);
		}

		delete maxflowPtr;
		maxflowPtr = NULL;

		// if any part faces is zero, we should clear tmpfList and tmpLabelList
		if (tmpfList[0].empty() || tmpfList[1].empty())
		{
			tmpfList.clear();
			tmpLabelList.clear();
			return false;
		}
		return true;
	}

	void ToothPCAInfo::CorrectLabel(Mesh & mesh, ToothPCAInfoList & toothPCAList, vector<PCAThreshold> & thresholdList)
	{
		if (tmpLabelList.empty()) return;
		
		tmpLabelList = { 0, 1 };

		double boundaryPCALength = BoundaryPCAAnalysisWithOtherTooth(mesh, tmpfList[0], tmpfList[1]);
		for (size_t j = 0; j < thresholdList.size(); j++)
		{
			if (thresholdList[j].label == label && boundaryPCALength > thresholdList[j].maxBoundaryPCALength)
			{
				cout << "wrong detection(boundary pca length is larger than threshold)" << endl;
				cout << "boundary pca length: " << boundaryPCALength << endl;
				cout << "threshold: " << thresholdList[j].maxBoundaryPCALength << endl;
				tmpLabelList.clear();
				tmpfList.clear();
				return;
			}
		}

		int ID = label % 10;
		int range_start = (int)(label / 10) * 10 + 1;
		int range_end = range_start + 7;
		int range_previous_count = 0;
		int range_next_count = 0;
		
		// case 1: judge from tooth count
		for (size_t i = 0; i < toothPCAList.size(); i++)
		{
			ToothPCAInfo & toothPCA = toothPCAList[i];
			if (toothPCA.fList.empty()) continue;
			if (toothPCA.label<range_start || toothPCA.label>range_end) continue;
			if (toothPCA.label < label) range_previous_count++;
			if (toothPCA.label > label) range_next_count++;
		}

		

		if (tmpfList[0].empty() || tmpfList[1].empty())
		{
			tmpLabelList.clear();
			tmpfList.clear();
			return;
		}

		if (range_previous_count + range_next_count >= 7)
		{
			tmpLabelList.clear();
			tmpfList.clear();
			return;
		}
		else if (range_next_count + ID == 8)
		{
			tmpLabelList = { -1, 0 };
			return;
		}
		else if (range_previous_count == ID - 1)
		{
			tmpLabelList = { 0, 1 };
			return;
		}

		// case 2: judge from before tooth
		for (size_t i = 0; i < adjcentToothList.size(); i++)
		{
			int adjcentLabel = adjcentToothList[i]->label;
			if (adjcentLabel % 10 == 1 && adjcentLabel % 10 == label % 10) return; // central incisor
			if (adjcentLabel - label == -1) return;		// tmpLabelList must be {0, 1}
		}

		// case 3: unknow
		int now_label = label % 10;
		int previousLabel = now_label - 1;
		int nextLabel = now_label + 1;

		double ratio_previous, ratio_next;
		double area_previous, area_now, area_next;
		vector<double> tmpAreaList = { 0, 0 };

		area_previous = area_now = 0.0;
		for (size_t i = 0; i < tmpfList[0].size(); i++)
		{
			Face * f = tmpfList[0][i];
			if (f->p_label == previousLabel)
				area_previous += f->area;
			else if (f->p_label == now_label)
				area_now += f->area;

			tmpAreaList[0] += f->area;
		}
		ratio_previous = area_previous / area_now;
		
		area_next = area_now = 0.0;
		for (size_t i = 0; i < tmpfList[1].size(); i++)
		{
			Face * f = tmpfList[1][i];
			if (f->p_label == nextLabel)
				area_next += f->area;
			else if (f->p_label == now_label)
				area_now += f->area;

			tmpAreaList[1] += f->area;
		}
		ratio_next = area_next / area_now;

		if (ratio_previous > ratio_next)
			tmpLabelList = { -1, 0 };

	

		// delete wrong segmentation if any area is smaller than its threshold
		for (size_t i = 0; i < tmpLabelList.size(); i++)
		{
			int tmpLabel = label + tmpLabelList[i];

			for (size_t j = 0; j < thresholdList.size(); j++)
			{
				if (thresholdList[j].label == tmpLabel && tmpAreaList[i] < thresholdList[j].minArea)
				{
					cout << "wrong detection(one part area is smaller than threshold)" << endl;
					cout << "part " << i << " area: " << tmpAreaList[i] << endl;
					tmpLabelList.clear();
					tmpfList.clear();
					return;
				}
			}
		}
	}

	int ToothPCAInfo::AffectToothAnalysis(ToothPCAInfoList & toothPCAList, map<int, int> & labelMap)
	{
		if (tmpLabelList.size() == 2 && tmpLabelList[0] == 0 && tmpLabelList[1] == 1)
		{
			int affectedToothCount = 0;
			int start, end;


			start = labelMap[label] + 1;
			if (labelMap[label] <= 8) end = 8;
			else end = 16;

			for (int i = start; i <= end; i++)
			{
				if (toothPCAList[i].fList.empty()) break;
				else affectedToothCount++;
			}
			cout << "affect" << labelMap[label] << " " << affectedToothCount << endl;

			return affectedToothCount;
		}

		return 0;
	}
}
