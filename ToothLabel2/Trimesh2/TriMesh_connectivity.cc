/*
Szymon Rusinkiewicz
Princeton University

TriMesh_connectivity.cc
Manipulate data structures that describe connectivity between faces and verts.
*/


#include <stdio.h>
#include "TriMesh.h"
#include <algorithm>
using std::find;


// Find the direct neighbors of each vertex
void TriMesh::need_neighbors()
{
	if (!neighbors.empty())
		return;
	need_faces();

	dprintf("Finding vertex neighbors... ");
	int nv = vertices.size(), nf = faces.size();

	vector<int> numneighbors(nv);
	for (int i = 0; i < nf; i++) {
		numneighbors[faces[i][0]]++;
		numneighbors[faces[i][1]]++;
		numneighbors[faces[i][2]]++;
	}

	neighbors.resize(nv);
	for (int i = 0; i < nv; i++)
		neighbors[i].reserve(numneighbors[i]+2); // Slop for boundaries

	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++) {
			vector<int> &me = neighbors[faces[i][j]];
			int n1 = faces[i][(j+1)%3];
			int n2 = faces[i][(j+2)%3];
			if (find(me.begin(), me.end(), n1) == me.end())
				me.push_back(n1);
			if (find(me.begin(), me.end(), n2) == me.end())
				me.push_back(n2);
		}
	}

	dprintf("Done.\n");
}


// Find the faces touching each vertex
void TriMesh::need_adjacentfaces()
{
	if (!adjacentfaces.empty())
		return;
	need_faces();

	dprintf("Finding vertex to triangle maps... ");
	int nv = vertices.size(), nf = faces.size();

	vector<int> numadjacentfaces(nv);
	for (int i = 0; i < nf; i++) {
		numadjacentfaces[faces[i][0]]++;
		numadjacentfaces[faces[i][1]]++;
		numadjacentfaces[faces[i][2]]++;
	}

	adjacentfaces.resize(vertices.size());
	for (int i = 0; i < nv; i++)
		adjacentfaces[i].reserve(numadjacentfaces[i]);

	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++)
			adjacentfaces[faces[i][j]].push_back(i);
	}

	dprintf("Done.\n");
}


// Find the face across each edge from each other face (-1 on boundary)
// If topology is bad, not necessarily what one would expect...
void TriMesh::need_across_edge()
{
	if (!across_edge.empty())
		return;
	need_adjacentfaces();
	need_normals();

	dprintf("Finding across-edge maps... ");

	int nf = faces.size();
	across_edge.resize(nf, Face(-1,-1,-1));

	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++) {
			if (across_edge[i][j] != -1)
				continue;
			int v1 = faces[i][(j+1)%3];
			int v2 = faces[i][(j+2)%3];
			const vector<int> &a1 = adjacentfaces[v1];
			const vector<int> &a2 = adjacentfaces[v2];
			for (int k1 = 0; k1 < a1.size(); k1++) {
				int other = a1[k1];
				if (other == i)
					continue;
				vector<int>::const_iterator it =
					find(a2.begin(), a2.end(), other);
				if (it == a2.end())
					continue;
				int ind = (faces[other].indexof(v1)+1)%3;
				if (faces[other][(ind+1)%3] != v2)
					continue;
				across_edge[i][j] = other;
				across_edge[other][ind] = i;
				break;
			}
		}
	}

	dist_faces_across_edge.resize(nf);
	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++) {
			if ( across_edge[i][j] == -1 ) {
				dist_faces_across_edge[i].push_back( 0.0f );
			} else {
				int v1 = faces[i][(j+1)%3];
				int v2 = faces[i][(j+2)%3];		
				point midpoint = .5f * (vertices[v1] + vertices[v2]);
				dist_faces_across_edge[i].push_back( dist(faces[i].faceCenter, midpoint) + dist(faces[across_edge[i][j]].faceCenter, midpoint)  );
			}
		}
	}

	for (int i = 0; i < nf; i++) {
		for (int j = 0; j < 3; j++) {
			if (across_edge[i][j] == -1) {
				across_edge[i][j] = i;
			}
		}
	}


	dprintf("Done.\n");
}


// Find the max component
void TriMesh::find_max_component(TriMesh * component)
{
	vector<int> groups;

	need_across_edge();
	int nv = vertices.size();
	int nf = faces.size();

	flags.resize(nf, 0);
	for (int i = 0; i < nf; i++)
	{
		if (flags[i] == 0)
		{
			vector<int> myset;
			groups.push_back(0);
			unsigned id = (unsigned)groups.size();
			flags[i] = id;
			myset.push_back(i);
			while (!myset.empty())
			{
				int p = myset[myset.size() - 1];
				myset.pop_back();
				groups[groups.size() - 1]++;
				for (int j = 0; j < 3; j++)
				{
					int q = across_edge[p][j];
					if (flags[q] == 0)
					{
						flags[q] = id;
						myset.push_back(q);
					}
				}
			}
		}
	}

	int maxid = 1;
	for (size_t i = 0; i < groups.size(); i++)
	{
		if (groups[i]>groups[maxid - 1])
			maxid = i + 1;
	}
	
	// find max component vertices
	vector<int> vertices_flags;
	vector<point> max_v;

	vertices_flags.resize(nv, -1);	
	for (int i = 0; i < nf; i++)
	{
		if (flags[i] == maxid)
		{
			for (int j = 0; j < 3; j++)
			{
				
				vertices_flags[faces[i][j]] = 0;
			}
		}
	}

	int count = 0;
	for (int i = 0; i < nv; i++)
	{
		if (vertices_flags[i] == 0)
		{
			vertices_flags[i] = count++;
			component->vertices.push_back(vertices[i]);
		}
	}

	// find max component faces
	vector<int> thisface;
	for (int i = 0; i < nf; i++)
	{
		if (flags[i] == maxid)
		{
			thisface.clear();
			for (int j = 0; j < 3; j++)
			{
				int old_v = faces[i][j];
				int new_v = vertices_flags[old_v];
				thisface.push_back(new_v + 1);
			}
			component->faces.push_back(TriMesh::Face(thisface[0],
				thisface[1],
				thisface[2]));
		}
	}
}
