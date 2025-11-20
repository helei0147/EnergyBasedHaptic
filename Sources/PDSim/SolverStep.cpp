#include <Windows.h>
#include <iostream>
#include <fstream>
#include <thread>
#include "Solver.h"
#include "gpu/gpuvar.h"
#include "gpu/gpufun.h"
#include "SimManager.h"

LARGE_INTEGER nFreq;
LARGE_INTEGER t1;
LARGE_INTEGER t2;
LARGE_INTEGER t3;

void CollisionBuffer::ReadCollisionBuffer() {
	std::lock_guard<std::mutex> lock(mtx);

	cudaMemset(tetVertColVelocityMax_d, 0, tetVertNum_d * 3 * sizeof(float));
	// 改变速度
	for (int i = 0; i < m_collidedNumCpy; i++) {
		runApplyCollision(
			m_tetVertPosMerged[i * 3 + 0], m_tetVertPosMerged[i * 3 + 1], m_tetVertPosMerged[i * 3 + 2],
			m_tetVertCollidedPosMerged[i * 3 + 0], m_tetVertCollidedPosMerged[i * 3 + 1], m_tetVertCollidedPosMerged[i * 3 + 2],
			tv_colStiffness, tv_colSpreadRadius, tv_maxColVelocity);
	}
}

void Solver::HandleCollision() {
	CollisionBuffer::GetInstance().ReadCollisionBuffer();
	printCudaError("HandleCollision");
}

void Solver::Step() {
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&t1);
	m_frameTime = (t1.QuadPart - t3.QuadPart) / (double)nFreq.QuadPart;
	t3 = t1;

	ApplyGravity();

	ResetTetVertCollision();
	PropagateCollision();
	MergeCollisionInfoTet();
	CollisionBuffer::GetInstance().WriteCollisionBuffer();
	HandleCollision();
	
	/********************四面体仿真********************/
	LARGE_INTEGER st_t1, st_t2;
	QueryPerformanceCounter(&st_t1);
	// 计算初始状态
	runcalculateST(tv_damping, tv_dt);
	QueryPerformanceCounter(&st_t2);
	m_stTime = (st_t2.QuadPart - st_t1.QuadPart) / (double)nFreq.QuadPart;

	// 迭代求解
	float omega = 1.0;
	for (int i = 0; i < tv_iterateNum; i++) {
		// 清空内力、碰撞力
		cudaMemset(tetVertForce_d, 0.0f, tetVertNum_d * 3 * sizeof(float));
		cudaMemset(tetVertCollisionForce_d, 0.0f, tetVertNum_d * 3 * sizeof(float));
		cudaMemset(tetVertCollisionDiag_d, 0.0f, tetVertNum_d * 3 * sizeof(float));

		// 夹取
        cudaMemset(gripper_adsorb_force, 0.0f, gripper_num * 3 * sizeof(float));  // 重置拖拽力
		HapticCollideTetGripper();

		// 计算体积力
		runcalculateIF(tv_volumnStiffness);

		// 更新位置
		omega = 4 / (4 - rho * rho * omega);
		runcalculatePOS(omega, tv_dt);
	}
	// 更新速度
	runcalculateV(tv_dt);
	printCudaError("Step ---------- 1");
	QueryPerformanceCounter(&t2);
	m_opTime = (t2.QuadPart - t1.QuadPart) / (double)nFreq.QuadPart;
	if (OUTPUT_TIME_TO_CSV)
	{
		timeInfo << m_opTime << endl;
		renderTimeInfo << m_frameTime << endl;
	}
	renderStepNumPassed++;
}