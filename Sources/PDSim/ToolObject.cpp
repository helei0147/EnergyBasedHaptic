#include <algorithm>
#include <fstream>
#include "ToolObject.h"
#include "Solver.h"
#include "common.h"
using namespace std;

void ToolObject::InitFromFile() {
	m_gripperLower.ReadFromFile();
	m_gripperUpper.ReadFromFile();
	m_gripperPivot.ReadFromFile();
}

void ToolObject::SetTrans(const float* rt) {
	float r[3][3], t[3];
	///将力反馈设备传来的数据的RT分来
	ForcepsTransformer::TransfromGL(rt, r, t);
	///根据工具转轴位置，生成两个钳嘴和手柄的运动
	m_forcepsTranslater.GenToolTransYZ(r, rt[16], rt[17]);
	///赋值给对应的工具
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperPivot.m_RT[0]);

	if (m_forcepsTranslater.m_uperRotate)
		ForcepsTransformer::TranstoGL(m_forcepsTranslater.uper, t, &m_gripperUpper.m_RT[0]);
	else
		ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperUpper.m_RT[0]);

	if (m_forcepsTranslater.m_lowerRotate)
		ForcepsTransformer::TranstoGL(m_forcepsTranslater.lower, t, &m_gripperLower.m_RT[0]);
	else
		ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperLower.m_RT[0]);
}


void RigidObject::ReadFromFile() {
	int smoothgroup = 0;
#pragma region
	fstream file(m_objFile.c_str());
	if (!file)		return;
	const uint32_t kMaxLineLength = 1024;
	char buffer[kMaxLineLength];
	float x, y, z, u, v;
	while (file) {
		file >> buffer;

		if (strcmp(buffer, "vn") == 0) {// normals
			file >> x >> y >> z;
		}
		else if (strcmp(buffer, "vt") == 0) {
			// texture coords
			file >> u >> v;

		}
		else if (buffer[0] == 'v') {// positions
			file >> x >> y >> z;
		}
		else if (buffer[0] == 'f') {
			// faces
			unsigned int pi[3];
			unsigned int uvi[3];
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
				uvi[i] = vt-1;
			}//for (int i = 0; i < 3; ++i)

		}//else if (buffer[0] == 'f')
		else if (buffer[0] == 's') {
			file >> u;
			smoothgroup++;

		}//else if (buffer[0] == 'f')
		else {
			char linebuf[1024];
			file.getline(linebuf, 1024);
		}
	}
	file.close();

#pragma endregion 读取OBJ文件
	if (smoothgroup <= 1)
		ReadMeshwithoutSmoothGroup();
	else
		ReadMeshwithSmoothGroup();
}

void RigidObject::ReadMeshwithSmoothGroup() {
	//SG SMOOTH GROUP
	struct SG {
		std::vector<unsigned int> triIdx;
		std::vector<unsigned int> uvIdx;
	};
	std::vector<SG> triSG(1);
	std::map<int, int> triSGMap;
	int smoothGroupIdx = 0;

	std::vector<float> triVertsOrg;//渲染三角形顶点
#pragma region
	fstream file(m_objFile.c_str());
	if (!file)		return;
	const uint32_t kMaxLineLength = 1024;
	char buffer[kMaxLineLength];
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
			triVertsOrg.push_back(x);
			triVertsOrg.push_back(y);
			triVertsOrg.push_back(z);
		}
		else if (buffer[0] == 'f') {
			// faces
			unsigned int pi[3];
			unsigned int uvi[3];
			for (int i = 0; i < 3; ++i) {
				int v = -1;
				int vt = 0;
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
				if(vt == 0)
					uvi[i] = v - 1;
				else
					uvi[i] = vt - 1;
			}//for (int i = 0; i < 3; ++i)
			triSG[smoothGroupIdx].triIdx.push_back(pi[0]);
			triSG[smoothGroupIdx].triIdx.push_back(pi[1]);
			triSG[smoothGroupIdx].triIdx.push_back(pi[2]);
			triSG[smoothGroupIdx].uvIdx.push_back(uvi[0]);
			triSG[smoothGroupIdx].uvIdx.push_back(uvi[1]);
			triSG[smoothGroupIdx].uvIdx.push_back(uvi[2]);
		}//else if (buffer[0] == 'f')
		else if (buffer[0] == 's'){ 
			int id;
			file >> id;
			id = id < 0 ? 0 : id;
			auto iter = triSGMap.find(id);
			if (iter == triSGMap.end()) {
				triSG.push_back(SG());
				smoothGroupIdx = triSG.size() - 1;
				triSGMap[id] = smoothGroupIdx;
			}
			else
				smoothGroupIdx = iter->second;
		}
		else {
			char linebuf[1024];
			file.getline(linebuf, 1024);
		}
	}
	file.close();

#pragma endregion 读取OBJ文件


#pragma region
	m_vertNum = 0;
	std::vector<unsigned int> uvIdx;///三角形索引
	for (int i = 0; i < triSG.size(); i++) {
		std::map<int, int> lookuptable;
		SG& sg = triSG[i];
		for (int j = 0; j < sg.triIdx.size(); j++) {
			auto iter = lookuptable.find(sg.triIdx[j]);
			if (iter == lookuptable.end()) {
				m_triVertsOrg.push_back(triVertsOrg[sg.triIdx[j] * 3]);
				m_triVertsOrg.push_back(triVertsOrg[sg.triIdx[j] * 3+1]);
				m_triVertsOrg.push_back(triVertsOrg[sg.triIdx[j] * 3+2]);
				lookuptable[sg.triIdx[j]] = m_vertNum;
				m_triIdx.push_back(m_vertNum);
				m_vertNum++;
			}
			else {
				m_triIdx.push_back(iter->second);
			}
			uvIdx.push_back(sg.uvIdx[j]);
		}
	}
	#pragma endregion 重组顶点

	m_triNum = m_triIdx.size() / 3;
	m_triIdx.resize(m_triIdx.size());
	m_triVertsOrg.resize(m_triVertsOrg.size());
	m_triNormOrg.resize(m_triVertsOrg.size(), 0);
	m_triColor.resize(m_triVertsOrg.size(), 0.8);
	m_triVertNormColor.resize(m_vertNum * 9);

#pragma region

	std::vector<float> count(m_vertNum, 0);
	for (int i = 0; i < m_triNum; ++i)
	{
		int ii = i * 3;
		int a = m_triIdx[ii + 0] * 3;
		int b = m_triIdx[ii + 1] * 3;
		int c = m_triIdx[ii + 2] * 3;

		float bax = m_triVertsOrg[b] - m_triVertsOrg[a];
		float bay = m_triVertsOrg[b + 1] - m_triVertsOrg[a + 1];
		float baz = m_triVertsOrg[b + 2] - m_triVertsOrg[a + 2];

		float cax = m_triVertsOrg[c] - m_triVertsOrg[a];
		float cay = m_triVertsOrg[c + 1] - m_triVertsOrg[a + 1];
		float caz = m_triVertsOrg[c + 2] - m_triVertsOrg[a + 2];

		float nx = bay * caz - baz * cay;
		float ny = baz * cax - bax * caz;
		float nz = bax * cay - bay * cax;

		float l = sqrt(nx * nx + ny * ny + nz * nz);

		if (l > 0.00001) {
			count[m_triIdx[ii + 0]] += 1;
			count[m_triIdx[ii + 1]] += 1;
			count[m_triIdx[ii + 2]] += 1;
			nx /= l;
			ny /= l;
			nz /= l;

			m_triNormOrg[a] += nx;
			m_triNormOrg[a + 1] += ny;
			m_triNormOrg[a + 2] += nz;

			m_triNormOrg[b] += nx;
			m_triNormOrg[b + 1] += ny;
			m_triNormOrg[b + 2] += nz;

			m_triNormOrg[c] += nx;
			m_triNormOrg[c + 1] += ny;
			m_triNormOrg[c + 2] += nz;
		}
	}

	for (int i = 0; i < m_vertNum; ++i) {
		float c = count[i];
		int ii = i * 3;
		if (c < 1) {
			m_triNormOrg[ii] = sqrt(1 / 3);
			m_triNormOrg[ii + 1] = sqrt(1 / 3);
			m_triNormOrg[ii + 2] = sqrt(1 / 3);
			continue;
		}
		m_triNormOrg[ii] /= c;
		m_triNormOrg[ii + 1] /= c;
		m_triNormOrg[ii + 2] /= c;
	}

#pragma endregion 开始计算法向量

	if (m_triUV.size() == 0)
		m_triUV.resize(m_vertNum * 2, 0.5);


	m_vertNum = 0;
	int idx = 0;
	int triNumSurf = m_triIdx.size() / 3;
	triVertsOrg.clear();//渲染三角形顶点
	std::vector<unsigned int> triIdx;///三角形索引
	std::vector<float> triColor;//渲染三角形法向量
	std::vector<float> triUV;//渲染三角形法向量	
	std::vector<float> triNormOrg;//渲染三角形法向量
	std::map<MeshVertexKey, uint32_t> vertexLookup;


	for (int t = 0; t < triNumSurf; t++) {
		for (int i = 0; i < 3; i++) {
			MeshVertexKey key;
			key.v = m_triIdx[idx];
			key.vt = uvIdx[idx];
			auto iter = vertexLookup.find(key);
			if (iter != vertexLookup.end()) {
				triIdx.emplace_back(iter->second);
			}
			else {
				triIdx.emplace_back(m_vertNum);
				vertexLookup[key] = m_vertNum;
				triUV.emplace_back(m_triUV[key.vt * 2]);
				triUV.emplace_back(m_triUV[key.vt * 2 + 1]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3 + 1]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3 + 2]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3 + 1]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3 + 2]);
				triColor.emplace_back(m_triColor[key.v * 3]);
				triColor.emplace_back(m_triColor[key.v * 3 + 1]);
				triColor.emplace_back(m_triColor[key.v * 3 + 2]);
				m_vertNum++;
			}
			idx++;
		}//for (int i = 0; i < 3; i++)
	}
	m_triIdx = triIdx;///三角形索引
	m_triColor = triColor;//渲染三角形法向量
	m_triUV = triUV;//渲染三角形法向量
	m_triVertsOrg = triVertsOrg;//渲染三角形顶点
	m_triNormOrg = triNormOrg;//渲染三角形法向量
	m_triVertNormColor.resize(m_vertNum * 9);
	for (int i = 0; i < m_vertNum; i++) {
		m_triVertNormColor[i * 9] = m_triVertsOrg[i * 3];
		m_triVertNormColor[i * 9 + 1] = m_triVertsOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 2] = m_triVertsOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 3] = m_triNormOrg[i * 3];
		m_triVertNormColor[i * 9 + 4] = m_triNormOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 5] = m_triNormOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 6] = m_triUV[i * 2];
		m_triVertNormColor[i * 9 + 7] = m_triUV[i * 2 + 1];
		m_triVertNormColor[i * 9 + 8] = 0;
	}
}

void RigidObject::ReadMeshwithoutSmoothGroup() {
	m_vertNum = 0;///该物体实际的顶点
	m_triNum = 0;///该物体三角网格的顶点
	std::vector<unsigned int> uvIdx;
#pragma region
	fstream file(m_objFile.c_str());
	if (!file)		return;
	const uint32_t kMaxLineLength = 1024;
	char buffer[kMaxLineLength];
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
			m_triVertsOrg.push_back(x);
			m_triVertsOrg.push_back(y);
			m_triVertsOrg.push_back(z);
		}
		else if (buffer[0] == 'f') {
			// faces
			unsigned int pi[3];
			unsigned int uvi[3];
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
				if(vt>=1)
					uvi[i] = vt - 1;
				else
					uvi[i] = v - 1;
			}//for (int i = 0; i < 3; ++i)
			m_triIdx.push_back(pi[0]);
			m_triIdx.push_back(pi[1]);
			m_triIdx.push_back(pi[2]);
			uvIdx.push_back(uvi[0]);
			uvIdx.push_back(uvi[1]);
			uvIdx.push_back(uvi[2]);
		}//else if (buffer[0] == 'f')
		else {
			char linebuf[1024];
			file.getline(linebuf, 1024);
		}
	}
	file.close();

#pragma endregion 读取OBJ文件

	m_vertNum = m_triVertsOrg.size() / 3;
	m_triNum = m_triIdx.size() / 3;

	m_triIdx.resize(m_triIdx.size());
	m_triVertsOrg.resize(m_triVertsOrg.size());
	m_triNormOrg.resize(m_triVertsOrg.size(), 0);
	m_triColor.resize(m_triVertsOrg.size(), 0.8);
	m_triVertNormColor.resize(m_vertNum * 9);


#pragma region

	std::vector<float> count(m_vertNum, 0);
	for (int i = 0; i < m_triNum; ++i)
	{
		int ii = i * 3;
		int a = m_triIdx[ii + 0] * 3;
		int b = m_triIdx[ii + 1] * 3;
		int c = m_triIdx[ii + 2] * 3;

		float bax = m_triVertsOrg[b] - m_triVertsOrg[a];
		float bay = m_triVertsOrg[b + 1] - m_triVertsOrg[a + 1];
		float baz = m_triVertsOrg[b + 2] - m_triVertsOrg[a + 2];

		float cax = m_triVertsOrg[c] - m_triVertsOrg[a];
		float cay = m_triVertsOrg[c + 1] - m_triVertsOrg[a + 1];
		float caz = m_triVertsOrg[c + 2] - m_triVertsOrg[a + 2];

		float nx = bay * caz - baz * cay;
		float ny = baz * cax - bax * caz;
		float nz = bax * cay - bay * cax;

		float l = sqrt(nx * nx + ny * ny + nz * nz);

		if (l > 0.00001) {
			count[m_triIdx[ii + 0]] += 1;
			count[m_triIdx[ii + 1]] += 1;
			count[m_triIdx[ii + 2]] += 1;
			nx /= l;
			ny /= l;
			nz /= l;

			m_triNormOrg[a] += nx;
			m_triNormOrg[a + 1] += ny;
			m_triNormOrg[a + 2] += nz;

			m_triNormOrg[b] += nx;
			m_triNormOrg[b + 1] += ny;
			m_triNormOrg[b + 2] += nz;

			m_triNormOrg[c] += nx;
			m_triNormOrg[c + 1] += ny;
			m_triNormOrg[c + 2] += nz;
		}
	}

	for (int i = 0; i < m_vertNum; ++i) {
		float c = count[i];
		int ii = i * 3;
		if (c < 1) {
			m_triNormOrg[ii] = sqrt(1 / 3);
			m_triNormOrg[ii + 1] = sqrt(1 / 3);
			m_triNormOrg[ii + 2] = sqrt(1 / 3);
			continue;
		}
		m_triNormOrg[ii] /= c;
		m_triNormOrg[ii + 1] /= c;
		m_triNormOrg[ii + 2] /= c;
	}

#pragma endregion 开始计算法向量

	for (int i = 0; i < m_vertNum; i++) {
		m_triVertNormColor[i * 9] = m_triVertsOrg[i * 3];
		m_triVertNormColor[i * 9 + 1] = m_triVertsOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 2] = m_triVertsOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 3] = m_triNormOrg[i * 3];
		m_triVertNormColor[i * 9 + 4] = m_triNormOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 5] = m_triNormOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 6] = 1;
		m_triVertNormColor[i * 9 + 7] = 1;
		m_triVertNormColor[i * 9 + 8] = 1;
	}

	if (m_triUV.size() == 0)
		m_triUV.resize(m_vertNum * 2, 0.5);

	///如果有UV，进行UV处理
	std::map<MeshVertexKey, uint32_t> vertexLookup;

	m_vertNum = 0;
	int idx = 0;
	int triNumSurf = m_triIdx.size() / 3;

	std::vector<unsigned int> triIdx;///三角形索引
	std::vector<float> triColor;//渲染三角形法向量
	std::vector<float> triUV;//渲染三角形法向量
	std::vector<float> triVertsOrg;//渲染三角形顶点
	std::vector<float> triNormOrg;//渲染三角形法向量


	for (int t = 0; t < triNumSurf; t++) {
		for (int i = 0; i < 3; i++) {
			MeshVertexKey key;
			key.v = m_triIdx[idx];
			key.vt = uvIdx[idx];
			auto iter = vertexLookup.find(key);
			if (iter != vertexLookup.end()) {
				triIdx.emplace_back(iter->second);
			}
			else {
				triIdx.emplace_back(m_vertNum);
				vertexLookup[key] = m_vertNum;
				triUV.emplace_back(m_triUV[key.vt * 2]);
				triUV.emplace_back(m_triUV[key.vt * 2 + 1]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3 + 1]);
				triVertsOrg.emplace_back(m_triVertsOrg[key.v * 3 + 2]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3 + 1]);
				triNormOrg.emplace_back(m_triNormOrg[key.v * 3 + 2]);
				triColor.emplace_back(m_triColor[key.v * 3]);
				triColor.emplace_back(m_triColor[key.v * 3 + 1]);
				triColor.emplace_back(m_triColor[key.v * 3 + 2]);
				m_vertNum++;
			}
			idx++;
		}//for (int i = 0; i < 3; i++)
	}
	m_triIdx = triIdx;///三角形索引
	m_triColor = triColor;//渲染三角形法向量
	m_triUV = triUV;//渲染三角形法向量
	m_triVertsOrg = triVertsOrg;//渲染三角形顶点
	m_triNormOrg = triNormOrg;//渲染三角形法向量
	m_triVertNormColor.resize(m_vertNum * 9);
	for (int i = 0; i < m_vertNum; i++) {
		m_triVertNormColor[i * 9] = m_triVertsOrg[i * 3];
		m_triVertNormColor[i * 9 + 1] = m_triVertsOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 2] = m_triVertsOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 3] = m_triNormOrg[i * 3];
		m_triVertNormColor[i * 9 + 4] = m_triNormOrg[i * 3 + 1];
		m_triVertNormColor[i * 9 + 5] = m_triNormOrg[i * 3 + 2];

		m_triVertNormColor[i * 9 + 6] = m_triUV[i * 2];
		m_triVertNormColor[i * 9 + 7] = m_triUV[i * 2 + 1];
		m_triVertNormColor[i * 9 + 8] = 0;
	}
}


ForcepsTransformer::ForcepsTransformer() {
	closeAxis = 1;
	handleAxis = 2;
	m_uperRotate = true;
	m_lowerRotate = true;
}

void ForcepsTransformer::GenToolTransX(const float r[3][3], float toolAngle, float rotatorAngle) {

	float uperM[3][3], lowerM[3][3], rotator[3][3];
	//工具运动顺序
	//1、张开
	TransFromAxisX(uperM, m_uperRotate ? -toolAngle : 0);
	TransFromAxisX(lowerM, m_lowerRotate ? toolAngle : 0);
	memset(rotator, 0, 9 * sizeof(float));
	rotator[0][0] = rotator[1][1] = rotator[2][2] = 1;
	//2、按照力反馈设备转动， 移动
	M3vs3(r, rotator, handle);
	M3vs3(rotator, uperM, uper_local);
	M3vs3(r, uper_local, uper);
	M3vs3(rotator, lowerM, lower_local);
	M3vs3(r, lower_local, lower);
}

void ForcepsTransformer::GenToolTransYZ(const float r[3][3], float toolAngle, float rotatorAngle) {

	float uperM[3][3], lowerM[3][3], rotator[3][3];
	//工具运动顺序
	//1、张开
	TransFromAxisY(uperM, m_uperRotate ? -toolAngle : 0);
	TransFromAxisY(lowerM, m_lowerRotate ? toolAngle : 0);
	memset(rotator, 0, 9 * sizeof(float));
	rotator[0][0] = rotator[1][1] = rotator[2][2] = 1;
	//2、按照力反馈设备转动， 移动
	M3vs3(r, rotator, handle);
	M3vs3(rotator, uperM, uper_local);
	M3vs3(r, uper_local, uper);
	M3vs3(rotator, lowerM, lower_local);
	M3vs3(r, lower_local, lower);
}

void ForcepsTransformer::GenToolTransYZVirtual(const float r[3][3], float toolAngle, float rotatorAngle) {

	float uperM[3][3], lowerM[3][3], rotator[3][3];
	memset(rotator, 0, 9 * sizeof(float));
	rotator[0][0] = 1;
	rotator[1][1] = 1;
	rotator[2][2] = 1;
	//工具运动顺序
	//1、张开
	TransFromAxisY(uperM, m_uperRotate ? -toolAngle : 0);
	TransFromAxisY(lowerM, m_lowerRotate ? toolAngle : 0);
	//2、按照力反馈设备转动， 移动
	M3vs3(r, rotator, handle_g);
	M3vs3(rotator, uperM, uper_local_g);
	M3vs3(r, uper_local_g, uper_g);
	M3vs3(rotator, lowerM, lower_local_g);
	M3vs3(r, lower_local_g, lower_g);
}

void ForcepsTransformer::TransfromGL(const float* gltrans, float r[3][3], float t[3], float motionScale) {
	t[0] = gltrans[12] * motionScale;
	t[1] = gltrans[13] * motionScale;
	t[2] = gltrans[14] * motionScale;
	r[0][0] = gltrans[0];
	r[1][0] = gltrans[1];
	r[2][0] = gltrans[2];
	r[0][1] = gltrans[4];
	r[1][1] = gltrans[5];
	r[2][1] = gltrans[6];
	r[0][2] = gltrans[8];
	r[1][2] = gltrans[9];
	r[2][2] = gltrans[10];
}

void ForcepsTransformer::TransfromGL(const float* gltrans, double r[3][3], double t[3], float motionScale) {
	t[0] = gltrans[12] * motionScale;
	t[1] = gltrans[13] * motionScale;
	t[2] = gltrans[14] * motionScale;
	r[0][0] = gltrans[0];
	r[1][0] = gltrans[1];
	r[2][0] = gltrans[2];
	r[0][1] = gltrans[4];
	r[1][1] = gltrans[5];
	r[2][1] = gltrans[6];
	r[0][2] = gltrans[8];
	r[1][2] = gltrans[9];
	r[2][2] = gltrans[10];
}

//从存储矩阵转换为计算矩阵
void ForcepsTransformer::TransFromGL(float M[3][3], const float* M_GL) {
	M[0][0] = M_GL[0];
	M[1][0] = M_GL[1];
	M[2][0] = M_GL[2];
	M[0][1] = M_GL[4];
	M[1][1] = M_GL[5];
	M[2][1] = M_GL[6];
	M[0][2] = M_GL[8];
	M[1][2] = M_GL[9];
	M[2][2] = M_GL[10];
}

void ForcepsTransformer::TransFromAxisAngle(float trans[3][3], float angle, int axis) {
	memset(trans, 0, 9 * sizeof(float));
	trans[0][0] = 1;
	trans[1][1] = 1;
	trans[2][2] = 1;
	if (0 == axis) {
		float sin_x = sin(angle);
		float cos_x = cos(angle);
		trans[1][1] = cos_x;
		trans[1][2] = -sin_x;
		trans[2][1] = sin_x;
		trans[2][2] = cos_x;
	}
	if (1 == axis) {
		float sin_y = sin(angle);
		float cos_y = cos(angle);
		trans[0][0] = cos_y;
		trans[0][2] = sin_y;
		trans[2][0] = -sin_y;
		trans[2][2] = cos_y;
	}
	if (2 == axis) {
		float sin_z = sin(angle);
		float cos_z = cos(angle);
		trans[0][0] = cos_z;
		trans[0][1] = -sin_z;
		trans[1][0] = sin_z;
		trans[1][1] = cos_z;
	}
}

void ForcepsTransformer::TransFromAxisX(float trans[3][3], float angle) {
	memset(trans, 0, 9 * sizeof(float));
	trans[0][0] = 1;
	trans[1][1] = 1;
	trans[2][2] = 1;
	float sin_x = sin(angle);
	float cos_x = cos(angle);
	trans[1][1] = cos_x;
	trans[1][2] = -sin_x;
	trans[2][1] = sin_x;
	trans[2][2] = cos_x;
}

void ForcepsTransformer::TransFromAxisY(float trans[3][3], float angle) {
	memset(trans, 0, 9 * sizeof(float));
	trans[0][0] = 1;
	trans[1][1] = 1;
	trans[2][2] = 1;
	float sin_y = sin(angle);
	float cos_y = cos(angle);
	trans[0][0] = cos_y;
	trans[0][2] = sin_y;
	trans[2][0] = -sin_y;
	trans[2][2] = cos_y;
}

void ForcepsTransformer::TransFromAxisZ(float trans[3][3], float angle) {
	memset(trans, 0, 9 * sizeof(float));
	trans[0][0] = 1;
	trans[1][1] = 1;
	trans[2][2] = 1;
	float sin_z = sin(angle);
	float cos_z = cos(angle);
	trans[0][0] = cos_z;
	trans[0][1] = -sin_z;
	trans[1][0] = sin_z;
	trans[1][1] = cos_z;
}

void ForcepsTransformer::M3vs3(const float inleft[3][3], const float inright[3][3], float out[3][3]) {
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			out[i][j] = inleft[i][0] * inright[0][j] + inleft[i][1] * inright[1][j] + inleft[i][2] * inright[2][j];
}

//3*3矩阵乘3维列向量
void ForcepsTransformer::M3V3(const float inleft[3][3], const float inright[3], float out[3]) {
	int i;
	for (i = 0; i < 3; i++)
		out[i] = inleft[i][0] * inright[0] + inleft[i][1] * inright[1] + inleft[i][2] * inright[2];
}

void ForcepsTransformer::TranstoGL(const float r[3][3], const float t[3], float* gltrans) {
	gltrans[0] = r[0][0];
	gltrans[1] = r[1][0];
	gltrans[2] = r[2][0];
	gltrans[3] = 0.0f;
	gltrans[4] = r[0][1];
	gltrans[5] = r[1][1];
	gltrans[6] = r[2][1];
	gltrans[7] = 0.0f;
	gltrans[8] = r[0][2];
	gltrans[9] = r[1][2];
	gltrans[10] = r[2][2];
	gltrans[11] = 0.0f;
	gltrans[12] = t[0];
	gltrans[13] = t[1];
	gltrans[14] = t[2];
	gltrans[15] = 1.0f;
}

void ForcepsTransformer::NormalizeVec3(float* vec3) {
	float x = vec3[0];
	float y = vec3[1];
	float z = vec3[2];
	float length = sqrt(x * x + y * y + z * z);
	vec3[0] /= length;
	vec3[1] /= length;
	vec3[2] /= length;
}

float ForcepsTransformer::Norm(const float (&vec3)[3]) {
	float x = vec3[0];
	float y = vec3[1];
	float z = vec3[2];
	return sqrt(x * x + y * y + z * z);
}

void ToolObject::AddCylinderCollider(CylinderCollider cylinder) {
	cylinderColliders.push_back(cylinder);
	CylinderCollider::cylinder_num++;
}

void ToolObject::SetCylinderCollider(int id, const float(&p)[3], const float(&d)[3]) {
	cylinderColliders[id].SetCylinderCollider(p, d);
}

void ToolObject::SetCylinderActive(int id, bool a) {
	cylinderColliders[id].SetCylinderActive(a);
}

void ToolObject::AddSphereCollider(SphereCollider sphere) {
	sphereColliders.push_back(sphere);
	SphereCollider::sphere_num++;
}

void ToolObject::SetSphereCollider(int id, const float(&p)[3]) {
	sphereColliders[id].SetSphereCollider(p);
}

void ToolObject::SetSphereActive(int id, bool a) {
	sphereColliders[id].SetSphereActive(a);
}

void ToolObject::AddGripperCollider(GripperCollider gripper) {
	gripper.gripper_no = GripperCollider::gripper_num;
	gripperColliders.push_back(gripper);
	GripperCollider::gripper_num++;
}

void ToolObject::SetGripperCollider(int id, float a, const float(&p)[3],
	const float(&p_x)[3], const float(&p_y)[3], const float(&p_z)[3],
	const float(&u_x)[3], const float(&u_y)[3], const float(&u_z)[3],
	const float(&l_x)[3], const float(&l_y)[3], const float(&l_z)[3]) {
	gripperColliders[id].SetGripperCollider(a, p, p_x, p_y, p_z, u_x, u_y, u_z, l_x, l_y, l_z);
}

void ToolObject::SetGripperActive(int id, bool a) {
	gripperColliders[id].SetGripperActive(a);
}

void ToolObject::SendColliders2Gpu(int cylinderOffset, int sphereOffset) {
	memset(cylinder_active, 0, cylinder_num * sizeof(char));
	memset(sphere_active, 0, sphere_num * sizeof(char));
	memset(gripper_active, 0, gripper_num * sizeof(char));
}

std::vector<CylinderCollider>& ToolObject::GetCylinderColliders() {
	return cylinderColliders;
}

std::vector<SphereCollider>& ToolObject::GetSphereColliders() {
	return sphereColliders;
}

std::vector<GripperCollider>& ToolObject::GetGripperColliders() {
	return gripperColliders;
}