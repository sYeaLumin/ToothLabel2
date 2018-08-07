#ifndef FEATUREEXTRACTOR_H
#define FEATUREEXTRACTOR_H

#include <iterator>
#include <string>
#include "Trimesh2\TriMesh.h"
#include "Trimesh2\TriMesh_algo.h"
#include "waPCA.h"
#include "DijkstraForGeodesics.h"
#include "MeshSegmentationFeatures.h"

using namespace std;

class FeatureExtractor
{
private:
	TriMesh* meshMain;
	FeatureSet* featuresALL;

public:
	bool CUROnOff = true;
	bool PCAOnOff = true;
	bool SCOnOff = true;
	bool SDFOnOff = true;
	bool SIOnOff = true;

public:
	FeatureExtractor() {}
	~FeatureExtractor() {}
	void extractFeature(string modelPath, vector<int> & labels);
	void saveFeature(string outPath);
	void reset();

private:
	TriMesh * processMesh(TriMesh *, bool writeDebugInfo = false);
	FeatureSet* exportCurvatureFeatures(TriMesh*, FeatureSet*, int, int, bool writeDebugInfo = false, bool returnNumFeaturesOnly = false, bool onoff = true);
	FeatureSet* exportPCAFeatures(TriMesh*, FeatureSet*, int, int, bool writeDebugInfo = false, bool returnNumFeaturesOnly = false, bool onoff = true);
	FeatureSet* exportSCFeatures(TriMesh* m, FeatureSet*, int, int, float, float, bool writeDebugInfo = false, bool returnNumFeaturesOnly = false, bool onoff = true);
	FeatureSet* exportSDFFeatures(TriMesh* m, FeatureSet*, int, int, bool writeDebugInfo = false, bool returnNumFeaturesOnly = false, bool onoff = true);
	FeatureSet* exportSpinImageFeatures(TriMesh* m, FeatureSet*, int, int, bool writeDebugInfo = false, bool returnNumFeaturesOnly = false, bool onoff = true);
	void writeDebugInfoToFile(TriMesh *m, const char *featureType, FeatureSet* features, int numFaces, int pos, int fid = 1);
};


#endif // !FEATUREEXTRACTOR_H
