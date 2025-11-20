#include <set>
#include <algorithm>
#include <map>
#include <windows.h>
#include "SoftObject.h"
#include"common.h"
using namespace std;

void SoftObject::ComputeNormals(const std::vector<float>& springVertPos) {
	memset(&m_triNormsSurfCount[0], 0, m_triNormsSurfCount.size() * sizeof(float));
	memset(&m_triNormsSurf[0], 0, m_triNormsSurf.size() * sizeof(float));

	for (int i = 0; i < m_triNumSurf; i++) {

		int& v0i = m_triIdxSurf[i * 3];
		int& v1i = m_triIdxSurf[i * 3+1];
		int& v2i = m_triIdxSurf[i * 3+2];
		const float* v0 = &springVertPos[m_surfVertSolverMapping[v0i] * 3];
		const float* v1 = &springVertPos[m_surfVertSolverMapping[v1i] * 3];
		const float* v2 = &springVertPos[m_surfVertSolverMapping[v2i] * 3];

		float v10x = v1[0] - v0[0];
		float v10y = v1[1] - v0[1]; 
		float v10z = v1[2] - v0[2];

		float v20x = v2[0] - v0[0];
		float v20y = v2[1] - v0[1];
		float v20z = v2[2] - v0[2];

		float x = v10y * v20z - v10z*v20y;
		float y = v10z * v20x - v10x*v20z;
		float z = v10x * v20y - v10y*v20x;
		float l = sqrt(x * x + y * y + z * z);
		l = l < 0.0001 ? 1 : l;
		x /= l;
		y /= l;
		z /= l;
		float* vn0 = &m_triNormsSurf[v0i * 3];
		float* vn1 = &m_triNormsSurf[v1i * 3];
		float* vn2 = &m_triNormsSurf[v2i * 3];

		vn0[0] += x;
		vn1[0] += x;
		vn2[0] += x;

		vn0[1] += y;
		vn1[1] += y;
		vn2[1] += y;

		vn0[2] += z;
		vn1[2] += z;
		vn2[2] += z;

		m_triNormsSurfCount[v0i] += 1;
		m_triNormsSurfCount[v1i] += 1;
		m_triNormsSurfCount[v2i] += 1;
	}

	int vertNum = m_surfVertSolverMapping.size();
	for (int i = 0; i < vertNum; i++) {
		float* vn = &m_triNormsSurf[i * 3];
		float& l = m_triNormsSurfCount[i];
		l = l < 1 ? 1 : l;
		vn[0] /= l;
		vn[1] /= l;
		vn[2] /= l;
	}
}

void SoftObject::ReadFromFile()
{
	const uint32_t kMaxLineLength = 1024;
	char buffer[kMaxLineLength];
#pragma region
	fstream file(m_objFile.c_str());
	if (!file)		return;

	float x, y, z, u, v;
	while (file) {
		file >> buffer;

		if (strcmp(buffer, "vn") == 0) {// normals
			file >> x >> y >> z;
		}
		else if (strcmp(buffer, "vt") == 0) {
			// texture coords
			file >> u >> v;
			m_triUV.push_back(u);
			m_triUV.push_back(v);
		}
		else if (buffer[0] == 'v') {// positions
			file >> x >> y >> z;
			m_triVerts.push_back(x);
			m_triVerts.push_back(y);
			m_triVerts.push_back(z);
		}
		else if (buffer[0] == 'f') {
			// faces
			int pi[3];//三角形三个点
			int uvi[3];//三角形三个点的uv坐标
			for (int i = 0; i < 3; ++i) {
				int v = -1;
				int vt = 1;
				int vn = -1;

				file >> v;
				if (!file.eof()) {
					// failed to read another index continue on
					if (file.fail()) {
						file.clear();
						break;
					}

					if (file.peek() == '/') {
						file.ignore();

						if (file.peek() != '/') {
							file >> vt;
						}

						if (file.peek() == '/') {
							file.ignore();
							file >> vn;
						}
					}
				}

				pi[i] = v - 1;
				uvi[i] = vt - 1;
			}//for (int i = 0; i < 3; ++i)
			m_triIdx.push_back(pi[0]);
			m_triIdx.push_back(pi[1]);
			m_triIdx.push_back(pi[2]);
			m_triUVIdx.push_back(uvi[0]);
			m_triUVIdx.push_back(uvi[1]);
			m_triUVIdx.push_back(uvi[2]);
		}//else if (buffer[0] == 'f')
		else {
			char linebuf[1024];
			file.getline(linebuf, 1024);
		}
	}
	file.close();

	if (0 == m_triUV.size())
		m_triUV.resize(m_triVerts.size() / 3 * 2);
#pragma endregion 读取OBJ文件

#pragma region
	UDLog("开始读取四面体：" + m_tetFile);
	int eleNum;
	int number = 0;

	file.open(m_tetFile);
	if (!file) {
		UDError(std::string("打开四面体文件错误：") + m_tetFile);
		return;
	}
	while (file) {
		file >> buffer;
		if (strcmp(buffer, "$Nodes") == 0) {
			file >> number;
			unsigned int idx;
			float x, y, z;
			UDLog(string("四面体顶点数量：") + to_string(number));
			for (int i = 0; i < number; i++) {
				file >> idx >> x >> y >> z;
				m_tetVerts.push_back(x);
				m_tetVerts.push_back(y);
				m_tetVerts.push_back(z);
			}
		}
		else if (strcmp(buffer, "$Elements") == 0) {
			file >> eleNum;
			int idx, type, tag, phy, elem;
			unsigned int i0, i1, i2, i3;
			UDLog(string("四面体数量：") + to_string(eleNum));
			for (int i = 0; i < eleNum; i++) {
				file >> idx >> type >> tag >> phy >> elem;
				if (type == 2) { //surface indices
					file >> i0 >> i1 >> i2;
				}
				else if (type == 4) { //tet indices
					file >> i0 >> i1 >> i2 >> i3;
					m_tet.push_back(i0 - 1);
					m_tet.push_back(i1 - 1);
					m_tet.push_back(i2 - 1);
					m_tet.push_back(i3 - 1);
				}
			}
		}
	}
	file.close();
	UDLog("读取完毕：" + m_tetFile);
#pragma endregion 读取四面体文件

#pragma region
	int tetVertNum = m_tetVerts.size() / 3.0;
	int triVertNum = m_triVerts.size() / 3.0;
	std::set<int> phySkining;
	std::vector<int> tet2triVertMapping;//三角形顶点与四面体对应
	tet2triVertMapping.resize(tetVertNum, -1);
	for (int j = 0; j < triVertNum; j++) {
		float* p = &m_triVerts[j * 3];
		float minDist = FLT_MAX;
		int pIdx;
		for (int i = 0; i < tetVertNum; i++) {
			float dist = LengthSq(p, &m_tetVerts[i * 3]);
			if (dist > minDist) continue;
			minDist = dist;
			pIdx = i;
		}
		if (minDist > POINT_MERGE_EPS)
			UDError(m_name + std::string(" : 四面体与三角形顶点没有对齐：") + to_string(j));
		phySkining.insert(pIdx);
		tet2triVertMapping[pIdx] = j;
	}

	if (phySkining.size() != triVertNum)
		UDError(m_name + std::string(" : 两个顶点对应到一个物理粒子上了, ") + to_string(phySkining.size()) + ":" + to_string(triVertNum));


	/////////将对应三角面片的粒子全部排到前面
	int inSideCount = triVertNum;
	for (int j = 0; j < tetVertNum; j++) {
		if (phySkining.find(j) == phySkining.end())
			tet2triVertMapping[j] = inSideCount++;
	}

	std::vector<float> tetVerts = m_tetVerts;
	for (int j = 0; j < tetVertNum; j++) {
		int& idx = tet2triVertMapping[j];
		m_tetVerts[idx * 3] = tetVerts[j * 3];
		m_tetVerts[idx * 3 + 1] = tetVerts[j * 3 + 1];
		m_tetVerts[idx * 3 + 2] = tetVerts[j * 3 + 2];
	}
	int tetNum = m_tet.size() / 4;
	for (int i = 0; i < tetNum; i++) {
		int* t = &m_tet[i * 4];
		t[0] = tet2triVertMapping[t[0]];
		t[1] = tet2triVertMapping[t[1]];
		t[2] = tet2triVertMapping[t[2]];
		t[3] = tet2triVertMapping[t[3]];
	}
#pragma endregion 处理粒子，将对应三角面片的粒子全部排到前面

}

void SoftObject::ReBuildRenderMesh() {

	m_triIdxSurf.resize(m_triIdxSurf.size());///细分三角形
	m_triNormsSurf.resize(m_surfVertSolverMapping.size() * 3);
	m_surfVertSolverMapping.resize(m_surfVertSolverMapping.size());

	m_renderTriIdx.clear();
	m_renderTriUV.clear();
	m_renderVertSolverMapping.clear();
	m_renderNormalMapping.clear();
	std::map<MeshVertexKey, uint32_t> vertexLookup;
	m_triNumSurf = m_triIdxSurf.size() / 3.0f;
	m_renderTriVertsNum = 0;
	int idx = 0;
	for (int t = 0; t < m_triNumSurf; t++) {
		for (int i = 0; i < 3; i++) {
			MeshVertexKey key;
			key.v = m_triIdxSurf[idx];
			key.vt = m_triUVIdxSurf[idx];
			auto iter = vertexLookup.find(key);
			if (iter != vertexLookup.end()) {
				m_renderTriIdx.emplace_back(iter->second);
			}
			else {
				m_renderTriIdx.emplace_back(m_renderTriVertsNum);
				vertexLookup[key] = m_renderTriVertsNum;
				m_renderTriUV.emplace_back(m_triUVSurf[key.vt * 2]);
				m_renderTriUV.emplace_back(m_triUVSurf[key.vt * 2 + 1]);
				m_renderVertSolverMapping.emplace_back(m_surfVertSolverMapping[key.v]);
				m_renderNormalMapping.emplace_back(key.v);
				m_renderTriVertsNum++;
			}
			idx++;
		}
	}
	m_renderTriVertNormUV.resize(m_renderTriVertsNum * 9);
	///这里还需要初始化UV
	for (int i = 0; i < m_renderTriVertsNum; i++) {
		float* uv = &m_renderTriVertNormUV[i * 9 + 6];
		uv[0] = m_renderTriUV[i * 2];
		uv[1] = m_renderTriUV[i * 2 + 1];
	}

}

void SoftObject::SurfaceSubdivision()
{
	UDLog("对表面网格开始细分");
	////对现有的三角网格进行细分
	std::set<EdgeKey>	edgeSet;
	std::set<EdgeKey>	uvEdgeSet;
	int triNum = m_triIdx.size() / 3;
	//////找出所有的边
	for (unsigned int i = 0; i < triNum; i++) {
		int tri0 = m_triIdx[3 * i + 0];
		int tri1 = m_triIdx[3 * i + 1];
		int tri2 = m_triIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		edgeSet.insert(e01);
		edgeSet.insert(e02);
		edgeSet.insert(e12);
	}
	for (unsigned int i = 0; i < triNum; i++) {
		int tri0 = m_triUVIdx[3 * i + 0];
		int tri1 = m_triUVIdx[3 * i + 1];
		int tri2 = m_triUVIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		uvEdgeSet.insert(e01);
		uvEdgeSet.insert(e02);
		uvEdgeSet.insert(e12);
	}

	m_triUVSurf = m_triUV;//纹理坐标
	m_triVertsSurf = m_triVerts;//顶点

	int vertNum = m_triVerts.size() / 3;
	int uvNum = m_triUV.size() / 2;
	//////对边进行细分，一个边会变成2个
	std::map<EdgeKey, int>	 newVertMap;
	for (auto iter = edgeSet.begin(); iter != edgeSet.end(); iter++) {
		int tri0 = iter->k[0] * 3;
		int tri1 = iter->k[1] * 3;
		float px_new = (m_triVerts[tri0] + m_triVerts[tri1]) * 0.5f;
		float py_new = (m_triVerts[tri0 + 1] + m_triVerts[tri1 + 1]) * 0.5f;
		float pz_new = (m_triVerts[tri0 + 2] + m_triVerts[tri1 + 2]) * 0.5f;
		m_triVertsSurf.push_back(px_new);
		m_triVertsSurf.push_back(py_new);
		m_triVertsSurf.push_back(pz_new);
		newVertMap[*iter] = vertNum;
		vertNum++;
	}
	std::map<EdgeKey, int>	 newUVMap;
	for (auto iter = uvEdgeSet.begin(); iter != uvEdgeSet.end(); iter++) {
		int tri0 = iter->k[0] * 2;
		int tri1 = iter->k[1] * 2;
		float uvx_new = (m_triUV[tri0] + m_triUV[tri1]) * 0.5f;
		float uvy_new = (m_triUV[tri0 + 1] + m_triUV[tri1 + 1]) * 0.5f;
		m_triUVSurf.push_back(uvx_new);
		m_triUVSurf.push_back(uvy_new);
		newUVMap[*iter] = uvNum;
		uvNum++;
	}
	///重新组织三角形，将一个三角形变成四个
	for (int i = 0; i < triNum; i++) {
		int tri0 = m_triIdx[3 * i + 0];
		int tri1 = m_triIdx[3 * i + 1];
		int tri2 = m_triIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		auto tri3 = newVertMap.find(e01);
		auto tri4 = newVertMap.find(e02);
		auto tri5 = newVertMap.find(e12);

		unsigned int uv0 = m_triUVIdx[3 * i + 0];
		unsigned int uv1 = m_triUVIdx[3 * i + 1];
		unsigned int uv2 = m_triUVIdx[3 * i + 2];
		EdgeKey uve01(uv0, uv1);
		EdgeKey uve02(uv0, uv2);
		EdgeKey uve12(uv1, uv2);

		auto uv3 = newUVMap.find(uve01);
		auto uv4 = newUVMap.find(uve02);
		auto uv5 = newUVMap.find(uve12);
		if (tri3 == newVertMap.end() || tri4 == newVertMap.end() || tri5 == newVertMap.end()) {
			UDError("几何三角形索引错误");
			continue;
		}

		if (uv3 == newUVMap.end() || uv4 == newUVMap.end() || uv5 == newUVMap.end()) {
			UDError("UV三角形索引错误");
			continue;
		}
		m_triIdxSurf.push_back(tri0);
		m_triIdxSurf.push_back(tri3->second);
		m_triIdxSurf.push_back(tri4->second);
		m_triUVIdxSurf.push_back(uv0);
		m_triUVIdxSurf.push_back(uv3->second);
		m_triUVIdxSurf.push_back(uv4->second);

		m_triIdxSurf.push_back(tri1);
		m_triIdxSurf.push_back(tri5->second);
		m_triIdxSurf.push_back(tri3->second);
		m_triUVIdxSurf.push_back(uv1);
		m_triUVIdxSurf.push_back(uv5->second);
		m_triUVIdxSurf.push_back(uv3->second);

		m_triIdxSurf.push_back(tri2);
		m_triIdxSurf.push_back(tri4->second);
		m_triIdxSurf.push_back(tri5->second);
		m_triUVIdxSurf.push_back(uv2);
		m_triUVIdxSurf.push_back(uv4->second);
		m_triUVIdxSurf.push_back(uv5->second);

		m_triIdxSurf.push_back(tri3->second);
		m_triIdxSurf.push_back(tri5->second);
		m_triIdxSurf.push_back(tri4->second);
		m_triUVIdxSurf.push_back(uv3->second);
		m_triUVIdxSurf.push_back(uv5->second);
		m_triUVIdxSurf.push_back(uv4->second);
	}

	////先找到表面三角形顶点与四面体顶点的对应关系
	m_triNormsSurfCount.resize(m_triVertsSurf.size() / 3.0);
	m_triNormsSurf.resize(m_triVertsSurf.size());//三角形法向量

}

void SoftObject::BuildOpenGLMesh9(const std::vector<float>& springVertPos) {
	int vertNum = m_renderTriVertNormUV.size() / 9;
	for (int vi = 0; vi < vertNum; vi++) {
		const float* sv = &springVertPos[m_renderVertSolverMapping[vi] * 3];
		float* v = &m_renderTriVertNormUV[vi * 9];
		v[0] = sv[0];
		v[1] = sv[1];
		v[2] = sv[2];

		const float* sn = &m_triNormsSurf[m_renderNormalMapping[vi] * 3];
		float* n = &m_renderTriVertNormUV[vi * 9 + 3];
		n[0] = sn[0];
		n[1] = sn[1];
		n[2] = sn[2];
	}
}

void SoftObject::BuildTriVertSolverMapping(const std::vector<float>& triVertPos) {
	int vertNum = triVertPos.size() / 3;
	int triVertNum = m_triVertsSurf.size() / 3;
	m_surfVertSolverMapping.resize(triVertNum, -1);
	for (int i = 0; i < triVertNum; i++) {
		float trix = m_triVertsSurf[i * 3];
		float triy = m_triVertsSurf[i * 3 + 1];
		float triz = m_triVertsSurf[i * 3 + 2];
		float mindist = FLT_MAX;
		int tidx = -1;
		for (int j = 0; j < vertNum; j++) {
			float tetx = triVertPos[j * 3];
			float tety = triVertPos[j * 3 + 1];
			float tetz = triVertPos[j * 3 + 2];
			if (std::abs(tetx - trix) > POINT_MERGE_EPS || std::abs(tety - triy) > POINT_MERGE_EPS || std::abs(tetz - triz) > POINT_MERGE_EPS)
				continue;
			float dist = sqrt((trix - tetx) * (trix - tetx) + (triy - tety) * (triy - tety) + (triz - tetz) * (triz - tetz));
			if (dist > POINT_MERGE_EPS)
				continue;
			if (tidx >= 0)
				UDError(m_name + ": 两个物体三角形顶点对应到一个四面体顶点上：" + to_string(i));
			tidx = j;
		}
		if (tidx < 0)
			UDError("没有找到三角形顶点映射：" + to_string(i));
		m_surfVertSolverMapping[i] = tidx;
	}

	std::cout << "Build tri vert solver mapping completed..." << std::endl;
}


