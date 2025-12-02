#include <Windows.h>
#include <iostream>
#include <fstream>
#include <thread>
#include "Solver.h"
#include "SimManager.h"
#include "gpu/gpuvar.h"
#include "gpu/gpufun.h"



Solver::~Solver() {

}

//从存储矩阵转换为计算矩阵
void translateFromGL(float M[3][3], const float* M_GL) {
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

//3*3矩阵乘3维列向量
void M3V3(const float inleft[3][3], const float inright[3], float out[3]) {
	int i;
	for (i = 0; i < 3; i++)
		out[i] = inleft[i][0] * inright[0] + inleft[i][1] * inright[1] + inleft[i][2] * inright[2];
}

void normalizeVec3(float* vec3) {
	float x = vec3[0];
	float y = vec3[1];
	float z = vec3[2];
	float length = sqrt(x * x + y * y + z * z);
	vec3[0] /= length;
	vec3[1] /= length;
	vec3[2] /= length;
}

void Solver::ApplyGravity() {
	gravityX_d = m_gravityX;
	gravityY_d = m_gravityY;
	gravityZ_d = m_gravityZ;
}

void Solver::CopyToGPU() {
	SimManager* m = (SimManager*)m_manager;
	// 四面体顶点信息
	tetVertNum_d = GetTetVertNum();
	tetNum_d = GetTetNum();
	cudaSetDevice(g_ID_SimRender);
	// 四面体顶点信息
	printf("tetVertNum:%d tetNum:%d\n", tetVertNum_d, tetNum_d);
	cudaMemcpy(tetVertPos_d, m_tetVertPos.data(), m_tetVertPos.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVertPos_d");
	cudaMemcpy(tetVertRestPos_d, m_tetVertPos.data(), m_tetVertPos.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVertRestPos_d");
	cudaMemcpy(tetVertMass_d, m_tetVertMass.data(), m_tetVertMass.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVertMass_d");
	cudaMemcpy(tetInvD3x3_d, m_tetInvD3x3.data(), m_tetInvD3x3.size()* sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetInvD3x3_d");
	cudaMemcpy(tetInvD3x4_d, m_tetInvD3x4.data(), m_tetInvD3x4.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetInvD3x4_d");
	cudaMemcpy(tetVolume_d, m_tetVolume.data(), m_tetVolume.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVolume_d");
	cudaMemcpy(tetVolumeDiag_d, m_tetVolumeDiag.data(), m_tetVolumeDiag.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVolumeDiag_d");
	cudaMemcpy(tetIndex_d, m_tetIndex.data(), m_tetIndex.size() * sizeof(int), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetIndex_d");
	cudaMemcpy(tetVertFixed_d, m_tetVertFixed.data(), m_tetVertFixed.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetVertFixed_d");
	cudaMemcpy(tetActive_d, m_tetActive.data(), m_tetActive.size() * sizeof(bool), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetStiffness_d");
	cudaMemcpy(tetStiffness_d, m_tetStiffness.data(), m_tetStiffness.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU tetStiffness_d");
	cudaMemcpy(mapTetVertIdx2TriVertIdx_d, m_tet2triMapping.data(), m_tet2triMapping.size() * sizeof(int), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU mapping tet");

	cudaSetDevice(g_ID_SoftHaptic);
	// 三角网格信息
	springVertNum_d = GetSpringVertNum();
	int springNum = GetSpringNum();
	springNum_d = springNum;
	
	printf("triVertNum:%d springNum:%d\n", springVertNum_d, springNum);
	cudaMemcpy(springVertPos_d, m_springVertPos.data(), m_springVertPos.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springVertPos_d");
	cudaMemcpy(springVertMass_d, m_springVertMass.data(), m_springVertMass.size()*sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springVertMass_d");
	cudaMemcpy(springVertFixed_d, m_springVertFixed.data(), m_springVertFixed.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springVertFixed_d");
	// 三角网格弹簧信息
	cudaMemcpy(springIndex_d, m_springIndex.data(), m_springIndex.size()*sizeof(int), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springIndex_d");
	cudaMemcpy(springOrgLength_d, m_springOrgLength.data(), m_springOrgLength.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springOrgLength_d");
	cudaMemcpy(springDiag_d, m_springDiag.data(), m_springDiag.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springDiag_d");
	cudaMemcpy(springStiffness_d, m_springStiffness.data(), m_springStiffness.size() * sizeof(float), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springStiffness_d");

	cudaMemcpy(springActive_d, m_springActive.data(), m_springActive.size() * sizeof(bool), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springActive_d");
	cudaMemcpy(springVertActive_d, m_springVertActive.data(), m_springVertActive.size() * sizeof(bool), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springVertActive_d");
	cudaMemcpy(springVert2TetVertMapping_d, m_subedtri2tetMapping.data(), m_subedtri2tetMapping.size() * sizeof(int), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU springVert2TetVertMapping_d");
	cudaMemcpy(triIndex_d, m_subedTriIdx.data(), m_subedTriIdx.size() * sizeof(int), cudaMemcpyHostToDevice);
	printCudaError("CopyToGPU triIndex_d");
	printCudaError("CopyToGPU");
}

