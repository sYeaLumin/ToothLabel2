#include "mesh.h"
#include "matrix.h"
#include <iostream>
#include <string>
#include "lineqn.h"
using namespace std;

// i+1 and i-1 modulo 3
// This way of computing it tends to be faster than using %
#define NEXT(i) ((i)<2 ? (i)+1 : (i)-2)
#define PREV(i) ((i)>0 ? (i)-1 : (i)+2)

namespace BiMesh
{
	// Rotate a coordinate system to be perpendicular to the given normal
	static void rot_coord_sys(const Vector3f &old_u, const Vector3f &old_v,
		const Vector3f &new_norm,
		Vector3f &new_u, Vector3f &new_v)
	{
		new_u = old_u;
		new_v = old_v;
		Vector3f old_norm = old_u.Cross(old_v);
		float ndot = old_norm.Dot(new_norm);
		if (unlikely(ndot <= -1.0f)) {
			new_u = -new_u;
			new_v = -new_v;
			return;
		}
		Vector3f perp_old = new_norm - ndot * old_norm;
		Vector3f dperp = 1.0f / (1 + ndot) * (old_norm + new_norm);
		new_u -= dperp * (new_u.Dot(perp_old));
		new_v -= dperp * (new_v.Dot(perp_old));
	}

	static Vector3f Vector3dToVector3f(Vector3d &v)
	{
		return Vector3f((float)v[0], (float)v[1], (float)v[2]);
	}

	static Vector3d Vector3fToVector3d(Vector3f &v)
	{
		return Vector3d((float)v[0], (float)v[1], (float)v[2]);
	}

	// Compute principal curvatures and directions.
	void Mesh::need_curvatures()
	{
		if (curv1.size() == vList.size())
			return;
		if (fList.size() == 0)
			return;

		need_normals();
		need_pointareas();

		printf("Computing curvatures... ");

		// Resize the arrays we'll be using
		int nv = vList.size(), nf = fList.size();
		curv1.clear(); curv1.resize(nv); curv2.clear(); curv2.resize(nv);
		pdir1.clear(); pdir1.resize(nv); pdir2.clear(); pdir2.resize(nv);
		vector<float> curv12(nv);

		// Set up an initial coordinate system per vertex
		for (int i = 0; i < nf; i++) {
			pdir1[fList[i]->v[0]] = Vector3dToVector3f(vList[fList[i]->v[1]]->Position() -
				vList[fList[i]->v[0]]->Position());
			pdir1[fList[i]->v[1]] = Vector3dToVector3f(vList[fList[i]->v[2]]->Position() -
				vList[fList[i]->v[1]]->Position());
			pdir1[fList[i]->v[2]] = Vector3dToVector3f(vList[fList[i]->v[0]]->Position() -
				vList[fList[i]->v[2]]->Position());
		}
#pragma omp parallel for
		for (int i = 0; i < nv; i++) {
			pdir1[i] = (pdir1[i].Cross(normals[i]));
			pdir1[i].Normalize();
			pdir2[i] = (normals[i].Cross(pdir1[i]));
		}

		// Compute curvature per-face
#pragma omp parallel for
		for (int i = 0; i < nf; i++) {
			// Edges
			Vector3f e[3] = {Vector3dToVector3f(vList[fList[i]->v[2]]->Position() - vList[fList[i]->v[1]]->Position()),
				Vector3dToVector3f(vList[fList[i]->v[0]]->Position() - vList[fList[i]->v[2]]->Position()),
				Vector3dToVector3f(vList[fList[i]->v[1]]->Position() - vList[fList[i]->v[0]]->Position()) };

			// N-T-B coordinate system per face
			Vector3f t = e[0];
			t.Normalize();
			Vector3f n = e[0].Cross(e[1]);
			Vector3f b = n.Cross(t);
			b.Normalize();

			// Estimate curvature based on variation of normals
			// along edges
			float m[3] = { 0, 0, 0 };
			float w[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
			for (int j = 0; j < 3; j++) {
				float u = e[j].Dot(t);
				float v = e[j].Dot(b);
				w[0][0] += u*u;
				w[0][1] += u*v;
				//w[1][1] += v*v + u*u; 
				//w[1][2] += u*v; 
				w[2][2] += v*v;
				Vector3f dn = normals[fList[i]->v[PREV(j)]] -
					normals[fList[i]->v[NEXT(j)]];
				float dnu = dn.Dot(t);
				float dnv = dn.Dot(b);
				m[0] += dnu*u;
				m[1] += dnu*v + dnv*u;
				m[2] += dnv*v;
			}
			w[1][1] = w[0][0] + w[2][2];
			w[1][2] = w[0][1];

			// Least squares solution
			float diag[3];
			if (!ldltdc<float, 3>(w, diag)) {
				//printf("ldltdc failed!\n");
				continue;
			}
			ldltsl<float, 3>(w, diag, m, m);

			// Push it back out to the vList
			for (int j = 0; j < 3; j++) {
				int vj = fList[i]->v[j];
				float c1, c12, c2;
				proj_curv(t, b, m[0], m[1], m[2],
					pdir1[vj], pdir2[vj], c1, c12, c2);
				float wt = cornerareas[i][j] / pointareas[vj];
#pragma omp atomic
				curv1[vj] += wt * c1;
#pragma omp atomic
				curv12[vj] += wt * c12;
#pragma omp atomic
				curv2[vj] += wt * c2;
			}
		}

		// Compute principal directions and curvatures at each vertex
#pragma omp parallel for
		for (int i = 0; i < nv; i++) {
			diagonalize_curv(pdir1[i], pdir2[i],
				curv1[i], curv12[i], curv2[i],
				normals[i], pdir1[i], pdir2[i],
				curv1[i], curv2[i]);
		}


		float maxValue = 0, minValue = 1e10;
		for (int i = 0; i < nv; i++)
		{
			maxValue = max(maxValue, curv1[i]);
			minValue = min(minValue, curv1[i]);
		}
		float diff = maxValue - minValue;

		
		// assign curvatures to each vertex
		for (size_t i = 0; i < vList.size(); i++)
		{
			Vertex * v = vList[i];
			v->maxCurv = curv1[i];
			v->minCurv = curv2[i];
			v->maxCurvDir = pdir1[i];
			v->minCurvDir = pdir2[i];	

			v->SetNormal(Vector3fToVector3d(normals[i]));
			
			if ( v->maxCurv > 0)
			{
				double x = sqrt(sqrt(v->maxCurv / maxValue));
				v->SetColor(Vector3d(x, 0, 0));
			}
			else
			{
				double x = sqrt(v->maxCurv / minValue);
				v->SetColor(Vector3d(0, 0, x));
			}
		}

		printf("Done.\n");
	}


	// Compute per-vertex normals
	void Mesh::need_normals()
	{
		// Nothing to do if we already have normals
		size_t nv = vList.size();
		if (normals.size() == nv)
			return;

		printf("Computing normals... ");
		normals.clear();
		normals.resize(nv);

		// Compute from fList
		int nf = fList.size();
#pragma omp parallel for
		for (int i = 0; i < nf; i++) {
			const Vector3d &p0 = vList[fList[i]->v[0]]->Position();
			const Vector3d &p1 = vList[fList[i]->v[1]]->Position();
			const Vector3d &p2 = vList[fList[i]->v[2]]->Position();
			Vector3d a = p0 - p1, b = p1 - p2, c = p2 - p0;
			float l2a = (float)a.Dot(a), l2b = (float)b.Dot(b), l2c = (float)c.Dot(c);
			if (!l2a || !l2b || !l2c)
				continue;
			Vector3d n = a.Cross(b);
			Vector3f facenormal((float)n[0], (float)n[1], (float)n[2]);
			normals[fList[i]->v[0]] += facenormal * (1.0f / (l2a * l2c));
			normals[fList[i]->v[1]] += facenormal * (1.0f / (l2b * l2a));
			normals[fList[i]->v[2]] += facenormal * (1.0f / (l2c * l2b));
		}


		// Make them all unit-length
#pragma omp parallel for
		for (size_t i = 0; i < nv; i++)
			normals[i].Normalize();

		for (size_t i = 0; i < nv; i++)
			vList[i]->SetNormal(Vector3fToVector3d(normals[i]));

		printf("Done.\n");
	}

	void Mesh::need_pointareas()
	{
		if (pointareas.size() == vList.size())
			return;
		if (fList.size() == 0) return;

		printf("Computing Vector3f areas... ");

		int nf = fList.size(), nv = vList.size();
		pointareas.clear();
		pointareas.resize(nv);
		cornerareas.clear();
		cornerareas.resize(nf);

#pragma omp parallel for
		for (int i = 0; i < nf; i++) {
			// Edges
			Vector3d e[3] = { vList[fList[i]->v[2]]->Position() - vList[fList[i]->v[1]]->Position(),
				vList[fList[i]->v[0]]->Position() - vList[fList[i]->v[2]]->Position(),
				vList[fList[i]->v[1]]->Position() - vList[fList[i]->v[0]]->Position() };

			// Compute corner weights
			float area = 0.5f * (float)(e[0].Cross(e[1])).L2Norm();
			float l2[3] = { (float)e[0].Dot(e[0]), (float)e[1].Dot(e[1]), (float)e[2].Dot(e[2]) };
			float ew[3] = { l2[0] * (l2[1] + l2[2] - l2[0]),
				l2[1] * (l2[2] + l2[0] - l2[1]),
				l2[2] * (l2[0] + l2[1] - l2[2]) };
			if (ew[0] <= 0.0f) {
				cornerareas[i][1] = -0.25f * l2[2] * area /
					(float)(e[0].Dot(e[2]));
				cornerareas[i][2] = -0.25f * l2[1] * area /
					(float)(e[0].Dot(e[1]));
				cornerareas[i][0] = area - cornerareas[i][1] -
					cornerareas[i][2];
			}
			else if (ew[1] <= 0.0f) {
				cornerareas[i][2] = -0.25f * l2[0] * area /
					(float)(e[1].Dot(e[0]));
				cornerareas[i][0] = -0.25f * l2[2] * area /
					(float)(e[1].Dot(e[2]));
				cornerareas[i][1] = area - cornerareas[i][2] -
					cornerareas[i][0];
			}
			else if (ew[2] <= 0.0f) {
				cornerareas[i][0] = -0.25f * l2[1] * area /
					(float)(e[2].Dot(e[1]));
				cornerareas[i][1] = -0.25f * l2[0] * area /
					(float)(e[2].Dot(e[0]));
				cornerareas[i][2] = area - cornerareas[i][0] -
					cornerareas[i][1];
			}
			else {
				float ewscale = 0.5f * area / (ew[0] + ew[1] + ew[2]);
				for (int j = 0; j < 3; j++)
					cornerareas[i][j] = ewscale * (ew[(j + 1) % 3] +
					ew[(j + 2) % 3]);
			}
#pragma omp atomic
			pointareas[fList[i]->v[0]] += cornerareas[i][0];
#pragma omp atomic
			pointareas[fList[i]->v[1]] += cornerareas[i][1];
#pragma omp atomic
			pointareas[fList[i]->v[2]] += cornerareas[i][2];
		}

		printf("Done.\n");
	}

	// Reproject a curvature tensor from the basis spanned by old_u and old_v
	// (which are assumed to be unit-length and perpendicular) to the
	// new_u, new_v basis.
	void Mesh::proj_curv(const Vector3f &old_u, const Vector3f &old_v,
		float old_ku, float old_kuv, float old_kv,
		const Vector3f &new_u, const Vector3f &new_v,
		float &new_ku, float &new_kuv, float &new_kv)
	{
		Vector3f r_new_u, r_new_v;
		rot_coord_sys(new_u, new_v, old_u.Cross(old_v), r_new_u, r_new_v);

		float u1 = r_new_u.Dot(old_u);
		float v1 = r_new_u.Dot(old_v);
		float u2 = r_new_v.Dot(old_u);
		float v2 = r_new_v.Dot(old_v);
		new_ku = old_ku * u1*u1 + old_kuv * (2.0f  * u1*v1) + old_kv * v1*v1;
		new_kuv = old_ku * u1*u2 + old_kuv * (u1*v2 + u2*v1) + old_kv * v1*v2;
		new_kv = old_ku * u2*u2 + old_kuv * (2.0f  * u2*v2) + old_kv * v2*v2;
	}

	// Given a curvature tensor, find principal directions and curvatures
	// Makes sure that pdir1 and pdir2 are perpendicular to normal
	void Mesh::diagonalize_curv(const Vector3f &old_u, const Vector3f &old_v,
		float ku, float kuv, float kv,
		const Vector3f &new_norm,
		Vector3f &pdir1, Vector3f &pdir2, float &k1, float &k2)
	{
		Vector3f r_old_u, r_old_v;
		rot_coord_sys(old_u, old_v, new_norm, r_old_u, r_old_v);

		float c = 1, s = 0, tt = 0;
		if (likely(kuv != 0.0f)) {
			// Jacobi rotation to diagonalize
			float h = 0.5f * (kv - ku) / kuv;
			tt = (h < 0.0f) ?
				1.0f / (h - sqrt(1.0f + h*h)) :
				1.0f / (h + sqrt(1.0f + h*h));
			c = 1.0f / sqrt(1.0f + tt*tt);
			s = tt * c;
		}

		k1 = ku - tt * kuv;
		k2 = kv + tt * kuv;

		if (fabs(k1) >= fabs(k2)) {
			pdir1 = c*r_old_u - s*r_old_v;
		}
		else {
			swap(k1, k2);
			pdir1 = s*r_old_u + c*r_old_v;
		}
		pdir2 = new_norm.Cross(pdir1);
	}

}