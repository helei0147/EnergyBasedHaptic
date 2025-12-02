#include <ctime>
#include <vector>
#include "SimManager.h"
#include "common.h"
#include "gpu/gpuvar.h"

using namespace std;

LARGE_INTEGER freq;
LARGE_INTEGER tbegin;
LARGE_INTEGER tend;


int g_ID_SimRender = -1;
int g_ID_SoftHaptic = -1;
int g_nv_deviceNum = 0;
bool g_asyncSimMode = false;// 只有在多GPU的情况下才会为真，用cpu存储作为中转进行多GPU之间的数据交换。

SimManager::SimManager() {

	m_logFid.open(m_moduleName + ".simlog", ios_base::out);
	if (m_logFid.is_open()) {
		time_t now = time(0);
		char* dt = ctime(&now);
		m_logFid << dt;
		m_logFid << "SoftHaptic 启动" << endl;
	}
	m_softHapticSolver.m_manager = (void*)(this);
}

SimManager::~SimManager() {
	for (int groupIdx = 0; groupIdx < m_toolIndexList.size(); groupIdx++)
	{
		for (int idxInGroup = 0; idxInGroup < m_toolList[groupIdx].size(); idxInGroup++)
		{
			delete m_toolList[groupIdx][idxInGroup];
		}
	}

	for (int i = 0; i < m_toolList.size(); i++) {
		auto tool = m_toolList[i];
		for (int j = 0; j < tool.size(); j++) {
			delete tool[j];
		}
	}

	for (int i = 0; i < m_softObjects.size(); i++)
		delete m_softObjects[i];
}

void SimManager::ComputeNormals() {
	for (int i = 0; i < m_softObjects.size(); i++)
		m_softObjects[i]->ComputeNormals(m_softHapticSolver.m_springVertPos);
}

void SimManager::PDUpdate()
{
	cudaSetDevice(g_ID_SimRender);
	m_softHapticSolver.Step();
	cudaSetDevice(g_ID_SoftHaptic);
	m_softHapticSolver.HapticSyn();
}

void SimManager::PDInit()
{
	InitSolverData();
	m_softHapticSolver.SolverInit();

	for (int i = 0; i < m_softObjects.size(); i++) {
		m_softObjects[i]->BuildTriVertSolverMapping(m_softHapticSolver.m_springVertPos);
	}
	for (int i = 0; i < m_softObjects.size(); i++) {
		m_softObjects[i]->ReBuildRenderMesh();
	}
	m_hapticDevice.InitHapticDevice();
}

// 初始化 cuda
void SimManager::InitCuda() {
	int i = 0;
	int nvDeviceNum = 0;
	cudaGetDeviceCount(&nvDeviceNum);
	if (nvDeviceNum == 0) {
		UDError("没有发现CUDA设备");
		return;
	}

	std::vector<cudaDeviceProp> props(nvDeviceNum);
	std::vector<bool> propertySuccess(nvDeviceNum);

	for (i = 0; i < nvDeviceNum; i++) {
		if (cudaSuccess == cudaGetDeviceProperties(&props[i], i))
			propertySuccess[i] = true;
		else
			propertySuccess[i] = false;
	}


	if (nvDeviceNum == 1&& propertySuccess[0] && props[0].major >= 1) {
		g_ID_SimRender = 0;
		g_ID_SoftHaptic = 0;
	}

	if (nvDeviceNum > 1) {
		g_asyncSimMode = true;
		size_t maxMemSize = 0;
		for (i = 0; i < nvDeviceNum; i++) {
			if (propertySuccess[i])
				if (props[i].major >= 1)
					if (props[i].totalGlobalMem > maxMemSize) {
						maxMemSize = props[i].totalGlobalMem;
						g_ID_SimRender = i;
					}
		}

		maxMemSize = 0;
		for (i = 0; i < nvDeviceNum; i++) {
			if (propertySuccess[i] && i != g_ID_SimRender)
				if (props[i].major >= 1)
					if (props[i].totalGlobalMem > maxMemSize) {
						maxMemSize = props[i].totalGlobalMem;
						g_ID_SoftHaptic = i;
					}
		}
	}

	if (g_ID_SimRender == -1) {
		UDError("没有设备支持CUDA");
		return;
	}
	m_cudaDeviceReady = true;
	UDLog("CUDA设备名称：" + std::string(props[g_ID_SimRender].name));
	g_nv_deviceNum = nvDeviceNum;
}

void SimManager::CloseManager() {

	m_hapticDevice.StopHapticDevice();
	m_logFid.close();
}

void SimManager::InitSolverData() {

	for (int i = 0; i < m_softObjects.size(); i++) {
		m_softObjects[i]->ReadFromFile();
		m_softObjects[i]->SurfaceSubdivision();
	}

	int tetVertOffset = 0;
	int triVertOffset = 0;

	set<pair<int, int>> triEdges;
	auto make_edge = [](int idx1, int idx2) {
		return make_pair(idx1 > idx2 ? idx2 : idx1, idx1 > idx2 ? idx1 : idx2);
		};

	vector<int> subedTriIdx;
	vector<float> subedTriPos;
	for (int i = 0; i < m_softObjects.size(); i++) {
		SoftObject* s = m_softObjects[i];
		///四面体顶点
		for (int vti = 0; vti < s->m_tetVerts.size(); vti++) {
			m_softHapticSolver.m_tetVertPos.push_back(s->m_tetVerts[vti]);
			
			m_softHapticSolver.m_tetVertfromSpringStiffness.push_back(s->m_tetVertfromSpringStiffness);
		}
		for (int idx = 0; idx < s->m_tetVerts.size() / 3; idx++)
		{
			m_softHapticSolver.m_tetVertfromSpringStiffness.push_back(s->m_tetVertfromSpringStiffness);
		}



		//四面体索引
		for (int vti = 0; vti < s->m_tet.size(); vti++)
			m_softHapticSolver.m_tetIndex.push_back(s->m_tet[vti] + tetVertOffset);
		for (int vti = 0; vti < s->m_tet.size() / 4.0; vti++)
			m_softHapticSolver.m_tetStiffness.push_back(s->m_tetStiffness);

		int triVNum = s->m_triVerts.size() / 3;
		for (int triVIdx = 0; triVIdx < triVNum; triVIdx++)
		{
			m_softHapticSolver.m_springVertPos.push_back(s->m_triVerts[triVIdx * 3 + 0]);
			m_softHapticSolver.m_springVertPos.push_back(s->m_triVerts[triVIdx * 3 + 1]);
			m_softHapticSolver.m_springVertPos.push_back(s->m_triVerts[triVIdx * 3 + 2]);
		}

		// 未细分三角形数据
		for (int triIdx = 0; triIdx < s->m_triIdx.size() / 3; triIdx++)
		{
			int a = s->m_triIdx[triIdx * 3 + 0] + triVertOffset;
			int b = s->m_triIdx[triIdx * 3 + 1] + triVertOffset;
			int c = s->m_triIdx[triIdx * 3 + 2] + triVertOffset;
			m_softHapticSolver.m_originTriIdx.push_back(a);
			m_softHapticSolver.m_originTriIdx.push_back(b);
			m_softHapticSolver.m_originTriIdx.push_back(c);
			auto e0 = make_edge(a, b);
			auto e1 = make_edge(b, c);
			auto e2 = make_edge(c, a);
			if (triEdges.find(e0) == triEdges.end())
				triEdges.insert(e0);
			if (triEdges.find(e1) == triEdges.end())
				triEdges.insert(e1);
			if (triEdges.find(e2) == triEdges.end())
				triEdges.insert(e2);
		}
		tetVertOffset += s->m_tetVerts.size() / 3;
		triVertOffset += s->m_triVerts.size() / 3;
	}

	if (m_softHapticSolver.m_tetActive.size() != m_softHapticSolver.m_tetIndex.size() / 4)
		m_softHapticSolver.m_tetActive.resize(m_softHapticSolver.m_tetIndex.size() / 4, 1);

	for (auto e = triEdges.begin(); e != triEdges.end(); e++)
	{
		m_softHapticSolver.m_springIndex.push_back(e->first);
		m_softHapticSolver.m_springIndex.push_back(e->second);
	}
}
