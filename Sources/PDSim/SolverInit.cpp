//////////所有和初始化相关的加到这个函数中
//////////包括文件读取，系统矩阵初始化
#include <io.h> 
#include <stdio.h>
#include <Windows.h>
#include <map>
#include "SimManager.h"
#include "Solver.h"
#include "gpu/gpuvar.h"
#include "gpu/gpufun.h"
#include "common.h"
using namespace std;


static void MyAssert(bool expression, string desc = "", string file = ".", string line = ".") {
	if (!expression) {
		cout << "MyAssert Failed!!! " << desc << " " << "in " << file << " in " << line << endl;
	}
}


//矩阵求逆
float Matrix_Inverse_3(float* A, float* R)				//R=inv(A)
{
	R[0] = A[4] * A[8] - A[7] * A[5];
	R[1] = A[7] * A[2] - A[1] * A[8];
	R[2] = A[1] * A[5] - A[4] * A[2];
	R[3] = A[5] * A[6] - A[3] * A[8];
	R[4] = A[0] * A[8] - A[2] * A[6];
	R[5] = A[2] * A[3] - A[0] * A[5];
	R[6] = A[3] * A[7] - A[4] * A[6];
	R[7] = A[1] * A[6] - A[0] * A[7];
	R[8] = A[0] * A[4] - A[1] * A[3];
	float det = A[0] * R[0] + A[3] * R[1] + A[6] * R[2];
	if (fabs(det) < 1e-7) {
		printf("det is %f\n", det);
		det = 1e-5;
	}
	float inv_det = 1 / det;
	for (int i = 0; i < 9; i++)	R[i] *= inv_det;
	return det;
}

float l2_dis(float* p0, float* p1, unsigned int dim = 3)
{
	float result = 0;
	for (int i = 0; i < dim; i++)
	{
		result += (p1[i] - p0[i]) * (p1[i] - p0[i]);
	}
	return result;
}

void Solver::SolverInit() {

	MergeVertex();
	CalculateTetTriMapping();
	FixSoft();//顶点合并后，从外部文件初始化不可灼烧点和固定点

	TriSubdivision(); // 计算细分过后的表面网格
	CalculateTetSubedtriMapping();
	InitSpringConstraint();
	InitVolumeConstraint();

	UDLog("解算器初始化结束");
	
	PreMalloc();
	CopyToGPU();

	// 记录力反馈循环中各部分的时间
	if (OUTPUT_TIME_TO_CSV)
	{
		timeInfo.open("deform_time.csv");
		timeInfo << "SimFrameTime" << endl;
		renderTimeInfo.open("render_time.csv");
		renderTimeInfo << "renderFrameTime" << endl;
	}
	timeInfo_6dof.open("6dof_time.csv");
	timeInfo_6dof << "solve, quaternion" << endl;

	forceInfoLeft.open("LeftToolInfo.csv");
	forceInfoLeft << "fx, fy, fz, tx, ty, tz, f_len, t_len" << endl;

	forceInfoDeformEnd.open("ForceInfoDeformEnd.csv");
	forceInfoDeformEnd << "fx, fy, fz, tx, ty, tz, f_len, t_len" << endl;

	qgqhLeft.open("qgqhLeft.csv");
	qgqhLeft << "cnt, ,qg0, qg1, qg2, qg3, qg4, qg5, ,qgCur0, qgCur1, qgCur2, qgCur3, qgCur4, qgCur5, ,qh0, qh1, qh2, qh3, qh4, qh5\n";

	colNumInfo.open("colNum.csv");
	colNumInfo << "colNum" << endl;
}


void Solver::InitSpringConstraint() {
	InitSpring();
	//初始化求解过程中使用的对角元素
	m_springDiag.resize(GetSpringVertNum());
	m_springOrgLength.resize(GetSpringNum());

	for (int i = 0; i < GetSpringNum(); i++) {
		//获取弹簧顶点索引
		unsigned int index0 = m_springIndex[2 * i + 0];
		unsigned int index1 = m_springIndex[2 * i + 1];


		m_springDiag[index0] += m_springStiffness[i];
		m_springDiag[index1] += m_springStiffness[i];

		auto x0 = m_springVertPos[index0 * SV_STRIDE + 0];
		auto x1 = m_springVertPos[index1 * SV_STRIDE + 0];
		auto dx = x0 - x1;

		auto y0 = m_springVertPos[index0 * SV_STRIDE + 1];
		auto y1 = m_springVertPos[index1 * SV_STRIDE + 1];
		auto dy = y0 - y1;

		auto z0 = m_springVertPos[index0 * SV_STRIDE + 2];
		auto z1 = m_springVertPos[index1 * SV_STRIDE + 2];
		auto dz = z0 - z1;

		m_springOrgLength[i] = sqrt(dx * dx + dy * dy + dz * dz);
	}
}

int Solver::GetTetVertNum(void)
{
	return m_tetVertPos.size() / 3;
}

int Solver::GetTetNum(void)
{
	return m_tetIndex.size() / 4;
}

int Solver::GetSpringVertNum(void)
{
	return m_springVertPos.size() / SV_STRIDE;
}

int Solver::GetSpringNum(void)
{
	return m_springIndex.size() / 2;
}

void Solver::InitVolumeConstraint()
{
	m_volumnSum = 0.0;
	m_tetVertfromSpringStiffness.resize(GetTetVertNum());
	m_tetVertMass.resize(GetTetVertNum());
	m_tetVolumeDiag.resize(GetTetVertNum());
	m_tetInvD3x3.resize(GetTetNum() * 9);
	m_tetInvD3x4.resize(GetTetNum() * 12);
	m_tetVertActive.resize(GetTetVertNum(), 1);
	printf("tetInvD3x4 size: %d\n", GetTetNum() * 12);
	for (int i = 0; i < GetTetNum(); i++) 
	{
		//计算每个四面体初始化的shape矩阵的逆
		int vIndex0 = m_tetIndex[i * 4 + 0];
		int vIndex1 = m_tetIndex[i * 4 + 1];
		int vIndex2 = m_tetIndex[i * 4 + 2];
		int vIndex3 = m_tetIndex[i * 4 + 3];
		//先计算shape矩阵
		float D[9];
		D[0] = m_tetVertPos[vIndex1 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
		D[1] = m_tetVertPos[vIndex2 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
		D[2] = m_tetVertPos[vIndex3 * 3 + 0] - m_tetVertPos[vIndex0 * 3 + 0];
		D[3] = m_tetVertPos[vIndex1 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
		D[4] = m_tetVertPos[vIndex2 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
		D[5] = m_tetVertPos[vIndex3 * 3 + 1] - m_tetVertPos[vIndex0 * 3 + 1];
		D[6] = m_tetVertPos[vIndex1 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];
		D[7] = m_tetVertPos[vIndex2 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];
		D[8] = m_tetVertPos[vIndex3 * 3 + 2] - m_tetVertPos[vIndex0 * 3 + 2];
		//计算D的逆,顺便记录体积
		m_tetVolume.push_back(fabs(Matrix_Inverse_3(D, &m_tetInvD3x3[i * 9])) / 6.0);
		m_volumnSum += m_tetVolume[i];

		//计算质量
		m_tetVertMass[vIndex0] += m_tetVolume[i] / 4;
		m_tetVertMass[vIndex1] += m_tetVolume[i] / 4;
		m_tetVertMass[vIndex2] += m_tetVolume[i] / 4;
		m_tetVertMass[vIndex3] += m_tetVolume[i] / 4;

		float* inv_D = &m_tetInvD3x3[i * 9];
		//论文中的AC矩阵
		m_tetInvD3x4[i * 12 + 0] = -inv_D[0] - inv_D[3] - inv_D[6];
		m_tetInvD3x4[i * 12 + 1] = inv_D[0];
		m_tetInvD3x4[i * 12 + 2] = inv_D[3];
		m_tetInvD3x4[i * 12 + 3] = inv_D[6];
		

		m_tetInvD3x4[i * 12 + 4] = -inv_D[1] - inv_D[4] - inv_D[7];
		m_tetInvD3x4[i * 12 + 5] = inv_D[1];
		m_tetInvD3x4[i * 12 + 6] = inv_D[4];
		m_tetInvD3x4[i * 12 + 7] = inv_D[7];

		m_tetInvD3x4[i * 12 + 8] = -inv_D[2] - inv_D[5] - inv_D[8];
		m_tetInvD3x4[i * 12 + 9] = inv_D[2];
		m_tetInvD3x4[i * 12 + 10] = inv_D[5];
		m_tetInvD3x4[i * 12 + 11] = inv_D[8];

		for (int j = 0; j < 12; j++) {
			if (m_tetInvD3x4[i * 12 + j] != m_tetInvD3x4[i * 12 + j]) {
				printf("m_tetInvD3x4[%d] is nan\n", i * 12 + j);
			}
		}

		//记录该点的对应的对角矩阵分量（用于雅各比迭代，因为只需要对角阵就可以实现）
		m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 0] * m_tetInvD3x4[i * 12 + 0] * m_tetVolume[i] * m_tetStiffness[i];//第i个四面体中的第一个点
		m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 4] * m_tetInvD3x4[i * 12 + 4] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex0] += m_tetInvD3x4[i * 12 + 8] * m_tetInvD3x4[i * 12 + 8] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 1] * m_tetInvD3x4[i * 12 + 1] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 5] * m_tetInvD3x4[i * 12 + 5] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex1] += m_tetInvD3x4[i * 12 + 9] * m_tetInvD3x4[i * 12 + 9] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 2] * m_tetInvD3x4[i * 12 + 2] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 6] * m_tetInvD3x4[i * 12 + 6] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex2] += m_tetInvD3x4[i * 12 + 10] * m_tetInvD3x4[i * 12 + 10] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 3] * m_tetInvD3x4[i * 12 + 3] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 7] * m_tetInvD3x4[i * 12 + 7] * m_tetVolume[i] * m_tetStiffness[i];
		m_tetVolumeDiag[vIndex3] += m_tetInvD3x4[i * 12 + 11] * m_tetInvD3x4[i * 12 + 11] * m_tetVolume[i] * m_tetStiffness[i];
	}
}

//将各个器官的四面体进行合并，并更新顶点索引
void Solver::MergeVertex() {
	int vertNum = m_tetVertPos.size() / 3;
	std::vector<float> tetVertPos;
	std::vector<float> tetVertColors;
	vector<int> vertMapping(vertNum,-1);
	/// 此处需要加刚度的映射
	for (int v0 = 0; v0 < vertNum; v0++) {
		if (vertMapping[v0] >= 0)
			continue;
		float x0 = m_tetVertPos[v0 * 3];
		float y0 = m_tetVertPos[v0 * 3+1];
		float z0 = m_tetVertPos[v0 * 3+2];
		int vertNumNew = tetVertPos.size() / 3;
		vertMapping[v0] = vertNumNew;
		///对于当前顶点，遍历
		for (int v1 = v0+1; v1 < vertNum; v1++) {
			float x1 = m_tetVertPos[v1 * 3];
			float y1 = m_tetVertPos[v1 * 3 + 1];
			float z1 = m_tetVertPos[v1 * 3 + 2];
			if (std::abs(x0 - x1) > POINT_MERGE_EPS || std::abs(y0 - y1) > POINT_MERGE_EPS || std::abs(z0 - z1) > POINT_MERGE_EPS)
				continue;
			float dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
			if (dist > POINT_MERGE_EPS)////obj默认是小数点后四位
				continue;
			vertMapping[v1] = vertNumNew;
		}
		tetVertPos.push_back(x0);
		tetVertPos.push_back(y0);
		tetVertPos.push_back(z0);
	}
	UDLog("合并的四面体顶点数为：" + to_string(int(vertNum - tetVertPos.size() / 3.0)));
	m_tetVertPos = tetVertPos;



	for (int i = 0; i < m_tetIndex.size(); i++)
		m_tetIndex[i] = vertMapping[m_tetIndex[i]];
	
}

void Solver::CalculateTetTriMapping()
{
	int triVertNum = GetSpringVertNum();
	int tetVertNum = GetTetVertNum();
	m_tet2triMapping.resize(tetVertNum, -1);
	m_tri2tetMapping.resize(triVertNum, -1);
	for (int i = 0; i < tetVertNum; i++)
	{
		float x0 = m_tetVertPos[i * 3];
		float y0 = m_tetVertPos[i * 3 + 1];
		float z0 = m_tetVertPos[i * 3 + 2];
		for (int j = 0; j < triVertNum; j++)
		{
			float x1 = m_springVertPos[j * 3 + 0];
			float y1 = m_springVertPos[j * 3 + 1];
			float z1 = m_springVertPos[j * 3 + 2];

			float dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
			if (dist < POINT_MERGE_EPS)
			{
				m_tet2triMapping[i] = j;
				m_tri2tetMapping[j] = i;
			}
		}
	}
}

void Solver::TriSubdivision()
{
	std::set<EdgeKey>	edgeSet;
	int triNum = m_originTriIdx.size() / 3;
		
	//////找出所有的边
	for (unsigned int i = 0; i < triNum; i++) {
		int tri0 = m_originTriIdx[3 * i + 0];
		int tri1 = m_originTriIdx[3 * i + 1];
		int tri2 = m_originTriIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		edgeSet.insert(e01);
		edgeSet.insert(e02);
		edgeSet.insert(e12);
	}


	m_subedTriPos = m_springVertPos;
	int vertNum = m_springVertPos.size() / 3;
	for (int i = 0; i < vertNum; i++)
	{
		m_mapSubedTriVert2OriginTriVert.push_back(i);
		m_mapSubedTriVert2OriginTriVert.push_back(i);
	}

	//////对边进行细分，一个边会变成2个
	std::map<EdgeKey, int>	 newVertMap;
	for (auto iter = edgeSet.begin(); iter != edgeSet.end(); iter++) {
		int tri0 = iter->k[0] * 3;
		int tri1 = iter->k[1] * 3;
		float px_new = (m_springVertPos[tri0] + m_springVertPos[tri1]) * 0.5f;
		float py_new = (m_springVertPos[tri0 + 1] + m_springVertPos[tri1 + 1]) * 0.5f;
		float pz_new = (m_springVertPos[tri0 + 2] + m_springVertPos[tri1 + 2]) * 0.5f;
		m_subedTriPos.push_back(px_new);
		m_subedTriPos.push_back(py_new);
		m_subedTriPos.push_back(pz_new);
		m_mapSubedTriVert2OriginTriVert.push_back(tri0/3);
		m_mapSubedTriVert2OriginTriVert.push_back(tri1/3);
		newVertMap[*iter] = vertNum;
		vertNum++;
	}
	///重新组织三角形，将一个三角形变成四个
	for (int i = 0; i < triNum; i++) {
		int tri0 = m_originTriIdx[3 * i + 0];
		int tri1 = m_originTriIdx[3 * i + 1];
		int tri2 = m_originTriIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		auto tri3 = newVertMap.find(e01);
		auto tri4 = newVertMap.find(e02);
		auto tri5 = newVertMap.find(e12);

		if (tri3 == newVertMap.end() || tri4 == newVertMap.end() || tri5 == newVertMap.end()) {
			UDError("几何三角形索引错误");
			continue;
		}

		m_subedTriIdx.push_back(tri0);
		m_subedTriIdx.push_back(tri3->second);
		m_subedTriIdx.push_back(tri4->second);

		m_subedTriIdx.push_back(tri1);
		m_subedTriIdx.push_back(tri5->second);
		m_subedTriIdx.push_back(tri3->second);

		m_subedTriIdx.push_back(tri2);
		m_subedTriIdx.push_back(tri4->second);
		m_subedTriIdx.push_back(tri5->second);

		m_subedTriIdx.push_back(tri3->second);
		m_subedTriIdx.push_back(tri5->second);
		m_subedTriIdx.push_back(tri4->second);
	}

	int subedTriVertNum = m_subedTriPos.size() / 3;
	m_springVertPos.clear();
	m_springVertPos.insert(m_springVertPos.begin(), m_subedTriPos.begin(), m_subedTriPos.end());
	m_springVertMass.resize(subedTriVertNum, 1);
	m_springVertFixed.resize(subedTriVertNum, 1);
	m_springVertActive.resize(subedTriVertNum, 1);
}

void Solver::InitSpring()
{
	std::set<EdgeKey>	edgeSet;
	int triNum = m_subedTriIdx.size() / 3;

	//////找出所有的边
	for (unsigned int i = 0; i < triNum; i++) {
		int tri0 = m_subedTriIdx[3 * i + 0];
		int tri1 = m_subedTriIdx[3 * i + 1];
		int tri2 = m_subedTriIdx[3 * i + 2];
		EdgeKey e01(tri0, tri1);
		EdgeKey e02(tri0, tri2);
		EdgeKey e12(tri1, tri2);
		edgeSet.insert(e01);
		edgeSet.insert(e02);
		edgeSet.insert(e12);
	}

	m_springIndex.clear();
	m_springStiffness.clear();
	for (auto iter = edgeSet.begin(); iter != edgeSet.end(); iter++)
	{
		m_springIndex.push_back(iter->k[0]);
		m_springIndex.push_back(iter->k[1]);
		m_springStiffness.push_back(500);
	}
	int springNum = m_springIndex.size() / 2;
	m_springActive.resize(springNum, 1);
}

void Solver::CalculateTetSubedtriMapping()
{
	 //原始表面三角顶点全部排列在m_subedTriIdx的最前面，因此m_tet2triMapping不会变。
	 //只需要更新细分之后的三角形顶点对应的四面体顶点下标
	int subedTriVertNum = m_subedTriPos.size() / 3;
	m_subedtri2tetMapping.resize(subedTriVertNum * 2);
	for (int i = 0; i < subedTriVertNum; i++)
	{
		int oriTriIdx0 = m_mapSubedTriVert2OriginTriVert[i * 2 + 0];
		int oriTriIdx1 = m_mapSubedTriVert2OriginTriVert[i * 2 + 1];
		m_subedtri2tetMapping[i * 2 + 0] = m_tri2tetMapping[oriTriIdx0];
		m_subedtri2tetMapping[i * 2 + 1] = m_tri2tetMapping[oriTriIdx1];
	}
}

void Solver::PreMalloc()
{
	SimManager* m = (SimManager*)m_manager;
	int tetVertNum = GetTetVertNum();
	int tetNum = GetTetNum();
	int springVertNum = GetSpringVertNum();
	int springNum = GetSpringNum();

	springVertisCollide_bridge.resize(springVertNum);
	springVertPos_bridge.resize(springVertNum * 3);
	springVertCollisionDepth_bridge.resize(springVertNum);
	springVertCollisionPos_bridge.resize(springVertNum*3);
	springVertCollisionToolFlag_bridge.resize(springVertNum);
	springVert2TetVertMapping_bridge.resize(springVertNum * 2);
	tetVertPos_bridge.resize(tetVertNum * 3);
	graspForce.resize(30);
	cudaSetDevice(g_ID_SimRender);

	cudaError e = cudaMalloc((void**)&tvCollisionNum_d, sizeof(int));
	if (e != cudaSuccess) {
		printf("cuda malloc tvCollisionNum_d failed: %s\n", cudaGetErrorString(e));
	}
	int a;
	int* aPtr = new int(-1);
	cudaMemcpy(aPtr, tvCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	// 四面体部分
	cudaMalloc((void**)&tetIndex_d, tetNum * 4 * sizeof(int));
	cudaMalloc((void**)&tetVertNum_d, sizeof(int));
	cudaMalloc((void**)&tetStiffness_d, tetNum * sizeof(float));
	cudaMalloc((void**)&tetVertPos_d, tetVertNum*3*sizeof(float));
	cudaMalloc((void**)&tetVertRestPos_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetActive_d, tetNum * sizeof(bool));
	cudaMemset(tetActive_d, true, tetNum * sizeof(bool));
	cudaMalloc((void**)&tetVertMass_d, tetVertNum*sizeof(float));

	cudaMalloc((void**)&tetInvD3x3_d, tetNum*9*sizeof(float));
	cudaMalloc((void**)&tetInvD3x4_d, tetNum*12*sizeof(float));
	cudaMalloc((void**)&tetVolume_d, tetNum*sizeof(float));
	cudaMalloc((void**)&tetVolumeDiag_d, tetVertNum * sizeof(float));
	cudaMalloc((void**)&tetVert2SpringVertMapping_d, tetVertNum * sizeof(int));
	

	cudaMalloc((void**)&tetVertVelocity_d, tetVertNum*3*sizeof(float));
	cudaMemset(tetVertVelocity_d, 0.0f, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertExternForce_d, tetVertNum * 3 * sizeof(float));
	cudaMemset(tetVertExternForce_d, 0.0f, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertForce_d, tetVertNum * 3 * sizeof(float));
	cudaMemset(tetVertForce_d, 0.0f, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertFixed_d, tetVertNum * sizeof(float));
	cudaMemset(tetVertFixed_d, 0.0f, tetVertNum * sizeof(float));
	cudaMalloc((void**)&tetVertPos_old_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertPos_prev_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertPos_last_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertPos_next_d, tetVertNum * 3 * sizeof(float));

	//四面体顶点是否处于有效状态
	cudaMalloc((void**)&tetVertActive_d, tetVertNum * sizeof(bool));
	cudaMemset(tetVertActive_d, 1, tetVertNum * sizeof(bool));
	cudaMalloc((void**)&tetVertNorm_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertNormAccu_d, tetVertNum * sizeof(float));
	// 四面体碰撞
	
	cudaMalloc((void**)&tetisCollide_d, tetNum * sizeof(unsigned char));
	cudaMalloc((void**)&tetVertisCollide_d, tetVertNum * sizeof(unsigned char));
    cudaMalloc((void**)&tetVertCollisionCnt_d, tetVertNum * sizeof(int));
	cudaMalloc((void**)&tetVertCollisionToolFlag_d, tetVertNum * sizeof(int));
	cudaMalloc((void**)&tetVertCollisionForce_d, tetVertNum * 3 * sizeof(float));
	cudaMemset(tetVertCollisionForce_d, 0.0f, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertCollisionDiag_d, tetVertNum * 3 * sizeof(float));
	cudaMemset(tetVertCollisionDiag_d, 0.0f, tetVertNum * 3 * sizeof(float));

	cudaMalloc((void**)&springVertisCollide_forTet, springVertNum * sizeof(unsigned char));
	cudaMalloc((void**)&springVertPos_forTet, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollisionDepth_forTet, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVertCollisionPos_forTet, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollisionToolFlag_forTet, springVertNum * sizeof(int));
	cudaMalloc((void**)&springVert2TetVertMapping_forTet,  springVertNum *2* sizeof(int));

	cudaMalloc((void**)&tetVertToolDistance_d, tetVertNum * sizeof(float));
	cudaMalloc((void**)&tetVertCollisionDepth_d, tetVertNum * sizeof(float));
	cudaMalloc((void**)&tetVertCollisionPos_d, tetVertNum * 3 * sizeof(float));

	cudaMalloc((void**)&tetVertQueueIndex_d, tetVertNum * sizeof(unsigned int));
	cudaMemset(tetVertQueueIndex_d, 0, tetVertNum * sizeof(unsigned int));
	cudaMalloc((void**)&tetVertAuxSumArray_d, tetVertNum * sizeof(unsigned int));
	cudaMemset(tetVertAuxSumArray_d, 0, tetVertNum * sizeof(unsigned int));
	//cudaSetDevice(DEVICE_ID_SIM);
	cudaMalloc((void**)&tetVertCollidedBuffer_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertCollidedNonPenetration_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertCollidedDepth_d, tetVertNum * sizeof(float));
	cudaMalloc((void**)&tetVertCollidedPos_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertCollidedToolFlag_d, tetVertNum * sizeof(int));
	cudaMalloc((void**)&tetVertColVelocityMax_d, tetVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&mapTetVertIdx2TriVertIdx_d, tetVertNum * sizeof(int));
	// 夹取参数
	gripper_num = GripperCollider::gripper_num;
	gripper_active = (char*)malloc(1 * sizeof(char) * gripper_num);
	memset(gripper_active, 0, 1 * sizeof(char) * gripper_num);
	gripper_angle = (float*)malloc(1 * sizeof(float) * gripper_num);
	gripper_unclose_tv = (bool*)malloc(1 * sizeof(bool) * gripper_num);
	memset(gripper_unclose_tv, true, 1 * sizeof(bool) * gripper_num);
	cudaMalloc((void**)&gripper_pos, 3 * sizeof(float) * gripper_num);
	gripper_scale = (float*)malloc(3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_pivot_x, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_pivot_y, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_pivot_z, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_upper_x, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_upper_y, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_upper_z, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_lower_x, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_lower_y, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_lower_z, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&tv_grab_flag, 1 * sizeof(unsigned int) * gripper_num * tetVertNum);
	cudaMalloc((void**)&tv_grab_half_flag, 1 * sizeof(unsigned int) * gripper_num * tetVertNum);
	cudaMalloc((void**)&tv_relative_pos, 3 * sizeof(float) * gripper_num * tetVertNum);
	cudaMalloc((void**)&sv_grab_flag, 1 * sizeof(unsigned int) * gripper_num * springVertNum);
	cudaMalloc((void**)&gripper_adsorb_force, 3 * sizeof(float) * gripper_num);
	cudaMalloc((void**)&gripper_grab_num, 1 * sizeof(float) * gripper_num);

	
	// 表面布料网格部分//////////////////////////////////////////////////////////////
	cudaSetDevice(g_ID_SoftHaptic);
	springNum_d = springNum;
	springVertNum_d = springVertNum;
	triNum_d = m_subedTriIdx.size() / 3;
	// 表面三角网格 三角形对应的顶点下标
	cudaMalloc((void**)&triIndex_d, m_subedTriIdx.size() * sizeof(unsigned int));
	// 弹簧顶点指导向量
	cudaMalloc((void**)&springVertNonPenetrationDir_d, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&tetVertPos_forTri, tetVertNum * 3 * sizeof(float));

	// 表面三角网格顶点信息
	cudaMalloc((void**)&springVertPos_d, springVertNum*SV_STRIDE*sizeof(float));
	cudaMalloc((void**)&springVertPos_old_d,     springVertNum*3*sizeof(float));
	cudaMalloc((void**)&springVertPos_prev_d,    springVertNum*3*sizeof(float));
	cudaMalloc((void**)&springVertPos_next_d,    springVertNum*3*sizeof(float));
	cudaMalloc((void**)&springVertVelocity_d,    springVertNum*3*sizeof(float));
	cudaMalloc((void**)&springVertExternForce_d, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertMass_d,     springVertNum*sizeof(float));
	cudaMalloc((void**)&springVertFixed_d,    springVertNum*sizeof(float));
	cudaMemset(springVertFixed_d, 0.0f, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVertForce_d, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertfromTetStiffness_d, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVert2TetVertMapping_d,  springVertNum *2* sizeof(int));

	cudaMalloc((void**)&springActive_d, springNum * sizeof(bool));
	cudaMalloc((void**)&springVertActive_d, springVertNum * sizeof(bool));
	cudaMalloc((void**)&springVertProjectedPos_d, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertInsertionDepth_d, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVertToolDistance_d, springVertNum * sizeof(float));


	// 表面三角网格弹簧数据
	cudaMalloc((void**)&springIndex_d, springNum * 2 * sizeof(unsigned int));
	cudaMalloc((void**)&springOrgLength_d, springNum * sizeof(float));
	cudaMalloc((void**)&springStiffness_d, springNum * sizeof(float));
	cudaMalloc((void**)&springDiag_d, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertNorms_d, 3 * sizeof(float) * springVertNum);
	// 表面三角网格碰撞信息
	cudaMalloc((void**)&springVertisCollide_d, springVertNum * sizeof(unsigned char));
	cudaMemset(springVertisCollide_d, 0, springVertNum * sizeof(unsigned char));
	cudaMalloc((void**)&springVertQueueIndex_d, springVertNum * sizeof(unsigned int));
	cudaMemset(springVertQueueIndex_d, 0, springVertNum * sizeof(unsigned int));
	cudaMalloc((void**)&springVertAuxSumArray_d, springVertNum * sizeof(unsigned int));
	cudaMemset(springVertAuxSumArray_d, 0, springVertNum * sizeof(unsigned int));
	cudaMalloc((void**)&springVertCollidedBuffer_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollidedBuffer_d, 0, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollidedNonPenetration_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollidedNonPenetration_d, 0, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollidedDepth_d, springVertNum * sizeof(float));
	cudaMemset(springVertCollidedDepth_d, 0, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVertCollidedPos_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollidedPos_d, 0, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollidedToolFlag_d, springVertNum * sizeof(int));
	cudaMemset(springVertCollidedToolFlag_d, 0, springVertNum * sizeof(int));

	cudaMalloc((void**)&springVertCollisionForce_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollisionForce_d, 0.0f, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollisionDiag_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollisionDiag_d, 0.0f, springVertNum * 3 * sizeof(float));

	cudaMalloc((void**)&springVertCollisionPos_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollisionPos_d, 0.0f, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollisionNormal_d, springVertNum * 3 * sizeof(float));
	cudaMemset(springVertCollisionNormal_d, 0.0f, springVertNum * 3 * sizeof(float));
	cudaMalloc((void**)&springVertCollisionDepth_d, springVertNum * sizeof(float));
	cudaMemset(springVertCollisionDepth_d, 0.0f, springVertNum * sizeof(float));
	cudaMalloc((void**)&springVertCollisionCnt_d, springVertNum * sizeof(unsigned int));
	cudaMemset(springVertCollisionCnt_d, 0, springVertNum * sizeof(unsigned int));
	cudaMalloc((void**)&springVertCollisionToolFlag_d, springVertNum * sizeof(int));
	cudaMemset(springVertCollisionToolFlag_d, 0, springVertNum * sizeof(int));

	// 碰撞体参数
	cylinder_num = CylinderCollider::cylinder_num;
	cylinder_active = (char*)malloc(1 * sizeof(char) * cylinder_num);
	memset(cylinder_active, 0, 1 * sizeof(char) * cylinder_num);
	cylinder_radius = (float*)malloc(1 * sizeof(float) * cylinder_num);
	cylinder_length = (float*)malloc(1 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_pos, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_dir, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_shift_tv, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_shift_sv, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_last_pos, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_collide_flag, 1 * sizeof(unsigned char) * cylinder_num);

	cudaMalloc((void**)&cylinder_pos_haptic, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_dir_haptic, 3 * sizeof(float) * cylinder_num);
	cudaMalloc((void**)&cylinder_last_pos_haptic, 3 * sizeof(float) * cylinder_num);

	sphere_num = SphereCollider::sphere_num;
	sphere_active = (char*)malloc(1 * sizeof(char) * sphere_num);
	memset(sphere_active, 0, 1 * sizeof(char) * sphere_num);
	sphere_radius = (float*)malloc(1 * sizeof(float) * sphere_num);
	cudaMalloc((void**)&sphere_pos, 3 * sizeof(float) * sphere_num);
	cudaMalloc((void**)&sphere_shift, 3 * sizeof(float) * sphere_num);
	cudaMalloc((void**)&sphere_last_pos, 3 * sizeof(float) * sphere_num);

	// 力反馈相关
	cudaMalloc((void**)&hapticCollisionNum_d, sizeof(int));

	if (g_nv_deviceNum == 1)// 单GPU，不需要在多个GPU之间进行数据传播，释放多余的空间，两个指针指向相同的存储区域
	{
		if (tetVertPos_forTri != tetVertPos_d)
		{
			cudaFree(tetVertPos_forTri);
			tetVertPos_forTri = tetVertPos_d;
		}
		if (springVertisCollide_forTet != springVertisCollide_d)
		{
			cudaFree(springVertisCollide_forTet);
			springVertisCollide_forTet = springVertisCollide_d;
		}
		if (springVertPos_forTet != springVertPos_d)
		{
			cudaFree(springVertPos_forTet);
			springVertPos_forTet = springVertPos_d;
		}
		if (springVertCollisionDepth_forTet != springVertCollisionDepth_d)
		{
			cudaFree(springVertCollisionDepth_forTet);
			springVertCollisionDepth_forTet = springVertCollisionDepth_d;
		}
		if (springVertCollisionPos_forTet != springVertCollisionPos_d)
		{
			cudaFree(springVertCollisionPos_forTet);
			springVertCollisionPos_forTet = springVertCollisionPos_d;
		}
		if (springVertCollisionToolFlag_forTet != springVertCollisionToolFlag_d)
		{
			cudaFree(springVertCollisionToolFlag_forTet);
			springVertCollisionToolFlag_forTet = springVertCollisionToolFlag_d;
		}
		if (springVert2TetVertMapping_forTet != springVert2TetVertMapping_d)
		{
			cudaFree(springVert2TetVertMapping_forTet);
			springVert2TetVertMapping_forTet = springVert2TetVertMapping_d;
		}
	}
	printCudaError("PreMalloc");
}

void Solver::FixSoft() {

	int tetVertNum = m_tetVertPos.size() / 3;
	m_tetVertFixed.resize(tetVertNum, 1);

	int fixNum = m_fixRadius.size();
	for (int i = 0; i < tetVertNum; i++) {
		float* vt = &m_tetVertPos[i * 3];
		for (int j = 0; j < fixNum; j++)
		{
			float squareDis = LengthSq(&m_fixCenter[j * 3], vt);
			if (squareDis < m_fixRadius[j] * m_fixRadius[j])
			{
				m_tetVertFixed[i] = 0;
			}
		}
	}
}