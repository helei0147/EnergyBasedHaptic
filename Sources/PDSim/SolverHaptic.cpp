#include "SimManager.h"
#include "Solver.h"
#include "gpu/gpuvar.h"
#include "gpu/gpufun.h"

LARGE_INTEGER hapticNFreq;
LARGE_INTEGER hapticT1;
LARGE_INTEGER hapticT2;

int GetCollidedNum() {
	return CollisionBuffer::GetInstance().GetCollidedNumCpy();
}

float* GetTetvertPos() {
	return CollisionBuffer::GetInstance().GetTetVertPosMerged();
}

float* GetTetVertCollidedPos() {
	return CollisionBuffer::GetInstance().GetTetVertCollidedPosMerged();
}

inline void transpose4x4(float* target, const float* source)
{
	target[0] = source[0];
	target[1] = source[4];
	target[2] = source[8];
	target[3] = source[12];
	target[4] = source[1];
	target[5] = source[5];
	target[6] = source[9];
	target[7] = source[13];
	target[8] = source[2];
	target[9] = source[6];
	target[10] = source[10];
	target[11] = source[14];
	target[12] = source[3];
	target[13] = source[7];
	target[14] = source[11];
	target[15] = source[15];
}

// 纯函数，输入：力反馈 buffer 输出：解析后的位置参数
void Solver::SetQH(const float* toolTrans, 
	std::vector<float>& dir, float& theta, std::vector<float>& qh, 
	std::vector<float>& hapticToolTrans)
{
	memcpy(qh.data(), &toolTrans[12], 3 * sizeof(float));
	qh[0] = toolTrans[12];
	qh[1] = toolTrans[13];
	qh[2] = toolTrans[14];

	// Omega = \theta * K
	// \theta 为旋转角度（单位为弧度） K为旋转轴。
	Eigen::Quaterniond q;
	mat3 rot;
	rot << toolTrans[0], toolTrans[4], toolTrans[8],
		toolTrans[1], toolTrans[5], toolTrans[9],
		toolTrans[2], toolTrans[6], toolTrans[10];
	q = rot; // 可以给四元数直接赋值3x3的旋转矩阵，会自动计算对应的四元数。
	double halfTheta = acos(q.w());
	vec3 rotAxis(q.x(), q.y(), q.z());
	rotAxis.normalize();

	rotAxis *= (2 * halfTheta);
	qh[3] = rotAxis.x();
	qh[4] = rotAxis.y();
	qh[5] = rotAxis.z();

	auto dir_h = m_hapticSolver.calculateToolDir(rotAxis);

	dir[0] = dir_h.x();
	dir[1] = dir_h.y();
	dir[2] = dir_h.z();

	theta = toolTrans[16];

	memcpy(hapticToolTrans.data(), toolTrans, 16 * sizeof(float));

}

void Solver::CalculateContactWithCollisionInfo(
	std::vector<float> qg, float grasp_l, 
	int collidedNum, float* collidedVertDepth, float* collidedVertPos, float* collidedVertNonPenetrationDir,
	vec3& total_FC, vec3& total_TC,
	mat3& total_partial_FC_X, mat3& total_partial_FC_Omega,
	mat3& total_partial_TC_X, mat3& total_partial_TC_Omega)
{
	vec3 Xg, Omega_g;
	Xg << qg[0], qg[1], qg[2];
	Omega_g << qg[3], qg[4], qg[5];
	vec3 toolDir = m_hapticSolver.calculateToolDir(Omega_g);
	vec3 Xg_grasp = m_hapticSolver.calculateGraspPoint(Xg, Omega_g, grasp_l);
	
	// calculate contact force and torque
	vec3 FC, TC;
	mat3 partial_FC_X, partial_FC_Omega, partial_TC_X, partial_TC_Omega;
	for (int i = 0; i < collidedNum; i++)
	{
		double d = collidedVertDepth[i];
		vec3 p(collidedVertPos[3 * i + 0], collidedVertPos[3 * i + 1], collidedVertPos[3 * i + 2]);
		vec3 n_toh08(collidedVertNonPenetrationDir[3 * i + 0], collidedVertNonPenetrationDir[3 * i + 1], collidedVertNonPenetrationDir[3 * i + 2]);
		vec3 r = p - Xg_grasp;
		FC = m_hapticSolver.calculateFC(n_toh08, d);
		partial_FC_X = m_hapticSolver.calculatePartialFC_X(n_toh08);
		partial_FC_Omega = m_hapticSolver.calculatePartialFC_Omega(r, n_toh08, d);
		TC = m_hapticSolver.calculateTC(r, FC);
		partial_TC_X = m_hapticSolver.calculatePartialTC_X(FC, r, n_toh08);
		partial_TC_Omega = m_hapticSolver.calculatePartialTC_Omega(FC, r, n_toh08, d);

		total_FC += FC;
		total_TC += TC;
		total_partial_FC_X += partial_FC_X;
		total_partial_FC_Omega += partial_FC_Omega;
		total_partial_TC_X += partial_TC_X;
		total_partial_TC_Omega += partial_TC_Omega;
	}
}

// 提取出全局变量，左右工具复用
void Solver::UpdateQG_6DOF(
	std::vector<float> qh, float grasp_l,
	int collidedNum, float* collidedVertDepth, float* collidedVertPos, float* collidedVertNonPenetrationDir,
	std::vector<float>& qg, std::vector<float>& dir_g,
	std::vector<float>& virtualToolTrans
) {
	using namespace Eigen;

	mat3 partial_FC_X, partial_FC_Omega, partial_TC_X, partial_TC_Omega;
	vec3 Fc, Tc, T_vc;
	partial_FC_X.setZero();
	partial_FC_Omega.setZero();
	partial_TC_X.setZero();
	partial_TC_Omega.setZero();
	Fc.setZero();
	Tc.setZero();
	T_vc.setZero();

	CalculateContactWithCollisionInfo(
		qg, grasp_l, collidedNum,
		collidedVertDepth, collidedVertPos, collidedVertNonPenetrationDir,
		Fc, Tc,
		partial_FC_X, partial_FC_Omega,
		partial_TC_X, partial_TC_Omega);

	vec3 Xh, Omega_h;
	Xh << qh[0], qh[1], qh[2];
	Omega_h << qh[3], qh[4], qh[5];
	vec3 Xg, Omega_g;
	Xg << qg[0], qg[1], qg[2];
	Omega_g << qg[3], qg[4], qg[5];
	vec3 updatedX, updatedOmega;
	double times[5]; // 不用，单纯凑形参
	auto delta_6DOF = m_hapticSolver.solve_6DOF(Xh, Xg, Omega_h, Omega_g,
		grasp_l,
		Fc, Tc,
		partial_FC_X, partial_FC_Omega, partial_TC_X, partial_TC_Omega,
		updatedX, updatedOmega, T_vc,
		collidedNum, times);

	qg[0] = updatedX[0];
	qg[1] = updatedX[1];
	qg[2] = updatedX[2];
	qg[3] = updatedOmega.x();
	qg[4] = updatedOmega.y();
	qg[5] = updatedOmega.z();
	vec3 dirg = m_hapticSolver.calculateToolDir(updatedOmega);
	dir_g[0] = dirg.x();
	dir_g[1] = dirg.y();
	dir_g[2] = dirg.z();

	vec3 center = m_hapticSolver.calculateGraspPoint(updatedX, updatedOmega, grasp_l);
	vec3 axis = delta_6DOF.block(3, 0, 3, 1);
	axis.normalize();

	Quaterniond qua = m_hapticSolver.Omega2Quaternion(updatedOmega);
	auto rm = qua.toRotationMatrix();
	virtualToolTrans[0] = rm(0, 0);
	virtualToolTrans[1] = rm(1, 0);
	virtualToolTrans[2] = rm(2, 0);
	virtualToolTrans[4] = rm(0, 1);
	virtualToolTrans[5] = rm(1, 1);
	virtualToolTrans[6] = rm(2, 1);
	virtualToolTrans[8] = rm(0, 2);
	virtualToolTrans[9] = rm(1, 2);
	virtualToolTrans[10] = rm(2, 2);
	virtualToolTrans[12] = qg[0] ;
	virtualToolTrans[13] = qg[1];
	virtualToolTrans[14] = qg[2];
	virtualToolTrans[15] = 1;
}

void Solver::CalculateContactWithCollisionInfo(
	std::vector<float> qg, vec3 rotCenter,
	int collidedNum, float* collidedVertDepth, float* collidedVertPos, float* collidedVertNonPenetrationDir,
	vec3& total_FC, mat3& total_partial_FC_X)
{
	total_FC.setZero();
	total_partial_FC_X.setZero();
	vec3 Xg, Omega_g;
	Xg << qg[0], qg[1], qg[2];
	Omega_g << qg[3], qg[4], qg[5];
	vec3 toolDir = m_hapticSolver.calculateToolDir(Omega_g);

	// calculate contact force and torque
	vec3 FC;
	mat3 partial_FC_X;
	for (int i = 0; i < collidedNum; i++)
	{
		double d = collidedVertDepth[i];
		vec3 p(collidedVertPos[3 * i + 0], collidedVertPos[3 * i + 1], collidedVertPos[3 * i + 2]);
		vec3 n_toh08(collidedVertNonPenetrationDir[3 * i + 0], collidedVertNonPenetrationDir[3 * i + 1], collidedVertNonPenetrationDir[3 * i + 2]);
		vec3 r = p - rotCenter;
		float tip2RotationalCenterLength = (Xg - rotCenter).norm();
		float scale = r.norm() / tip2RotationalCenterLength; // 小于1，费力杠杆
		FC = m_hapticSolver.calculateFC(n_toh08, d) * scale;
		partial_FC_X = m_hapticSolver.calculatePartialFC_X(n_toh08);

		total_FC += FC;
		total_partial_FC_X += partial_FC_X;
	}
}

void Solver::MergeCollisionInfoTet() {
	// 1. 合并碰撞信息
	runMergeCollisionInfoTet();
	printCudaError("MergeCollisionInfoTet merge");

	int tetCollidedNum = 0;
	cudaError_t error = cudaMemcpy(&tetCollidedNum, tvCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	if (error != cudaSuccess)
	{
		printf("cudaerror: %d\n", error); // 700
	}
	printCudaError("MergeCollisionInfoTet tvCollisionNum_d");
	tv_collisionNum = tetCollidedNum;
	vector<float> collidedVertPos(tetCollidedNum * 3, 0);
	vector<float> collidedVertDDir(tetCollidedNum * 3, 0);
	vector<float> collidedVertDepth(tetCollidedNum, 0);
	vector<int> collidedVertToolFlag(tetCollidedNum, -1);
	//printf("tetCollidedNum in MergeCollisionInfoTet: %d\n", tetCollidedNum);
	
	cudaMemcpy(collidedVertPos.data(), tetVertCollidedBuffer_d, 3 * tetCollidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("MergeCollisionInfoTet copy after tetVertCollidedBuffer_d");
	cudaMemcpy(collidedVertDDir.data(), tetVertCollidedNonPenetration_d, 3 * tetCollidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("MergeCollisionInfoTet copy after tetVertCollidedNonPenetration_d");
	cudaMemcpy(collidedVertDepth.data(), tetVertCollidedDepth_d, tetCollidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("MergeCollisionInfoTet copy after tetVertCollidedDepth_d");
	cudaMemcpy(collidedVertToolFlag.data(), tetVertCollidedToolFlag_d, tetCollidedNum * sizeof(int), cudaMemcpyDeviceToHost);
	printCudaError("MergeCollisionInfoTet copy after tetVertCollidedDepth_d");
}

void Solver::MergeCollisionInfoTriVert() {
	// 1. 合并碰撞信息
	SimManager* m = (SimManager*)m_manager;
	
	runMergeCollisionInfoTriVert();

	// 2. 传送到CPU，需要分成多个部分
	int collidedNum = 0;
	cudaMemcpy(&collidedNum, hapticCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	//printf("collidedNum in MergeCollisionInfoTriVert: %d\n", collidedNum);
	sv_collisionNum = collidedNum;
	printCudaError("trivertmergeAfter memcpy num");
	vector<float> collidedVertPos(collidedNum * 3, 0);
	vector<float> collidedVertDDir(collidedNum * 3, 0);
	vector<float> collidedVertDepth(collidedNum, 0);
	vector<int> collidedVertToolFlag(collidedNum, -1);
	cudaMemcpy(collidedVertPos.data(), springVertCollidedBuffer_d, 3 * collidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("trivertmergeAfter memcpy pos");
	cudaMemcpy(collidedVertDDir.data(), springVertCollidedNonPenetration_d, 3 * collidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("trivertmergeAfter memcpy ddir");
	cudaMemcpy(collidedVertDepth.data(), springVertCollidedDepth_d, collidedNum * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("trivertmergeAfter memcpy depth");
	cudaMemcpy(collidedVertToolFlag.data(), springVertCollidedToolFlag_d, collidedNum * sizeof(int), cudaMemcpyDeviceToHost);
	printCudaError("trivertmergeAfter memcpy toolflag");

	for (int index = 0; index < m->m_toolList.size(); index++) {
		m_operatorTransList[index].Clear();
	}
	for (int i = 0; i < collidedNum; i++) {
		int operatorId = collidedVertToolFlag[i]; // 是哪个操作手的碰撞点
		m_operatorTransList[operatorId].AddCollidedVert(
			collidedVertPos[i * 3 + 0],
			collidedVertPos[i * 3 + 1],
			collidedVertPos[i * 3 + 2],
			collidedVertDDir[i * 3 + 0],
			collidedVertDDir[i * 3 + 1],
			collidedVertDDir[i * 3 + 2],
			collidedVertDepth[i]
		);
		m_operatorTransList[operatorId].collidedNum++;
	}

	printCudaError("MergeCollisionInfoTriVert");
}

void CollisionBuffer::WriteCollisionBuffer() {
	// 更新碰撞信息 tetVertCollisionPos_d, tetVertCollisionNormal_d
	std::lock_guard<std::mutex> lock(mtx);

	// 传到cpu用于碰撞扩散
	cudaMemcpy(&m_collidedNumCpy, tvCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	m_tetVertCollidedPosMerged.resize(m_collidedNumCpy * 3);
	cudaMemcpy(m_tetVertCollidedPosMerged.data(), tetVertCollidedPos_d, 3 * m_collidedNumCpy * sizeof(float), cudaMemcpyDeviceToHost);
	m_tetVertPosMerged.resize(m_collidedNumCpy * 3);
	cudaMemcpy(m_tetVertPosMerged.data(), tetVertCollidedBuffer_d, 3 * m_collidedNumCpy * sizeof(float), cudaMemcpyDeviceToHost);
	printCudaError("WriteCollisionBuffer");
}

void Solver::UpdateColliders() {
	// 根据虚拟位姿，调用当前的工具，更新对应的碰撞体
	SimManager* m = (SimManager*)m_manager;
	for (int index = 0; index < m->m_toolList.size(); index++) {
		int id = m->m_toolIndexList[index];
		auto& tool = m->m_toolList[index][id];
		if(id>0)
		tool->UpdateColliders(m_operatorTransList[index].qg, m_operatorTransList[index].dirg, m_operatorTransList[index].thetah);
	}
}

void Solver::DetectCollisionSV() {
	// 对当前工具拥有的碰撞体进行碰撞检测
	SimManager* m = (SimManager*)m_manager;
	for (int index = 0; index < m->m_toolList.size(); index++) {
		int id = m->m_toolIndexList[index];
		if (id > 0 && id < m->m_toolList[index].size()) {
			vector<CylinderCollider> cylinderColliders = m->m_toolList[index][id]->GetCylinderColliders();
			for (auto& cylinder : cylinderColliders) {
				if (!cylinder.active) continue;
				
				// 离散碰撞 顶点 - 胶囊体
				runHapticCDSV_cylinder(
					cylinder.positions[0], cylinder.positions[1], cylinder.positions[2],
					cylinder.lastPositions[0], cylinder.lastPositions[1], cylinder.lastPositions[2],
					cylinder.direction[0], cylinder.direction[1], cylinder.direction[2],
					cylinder.radius, cylinder.length, index);
			}
		}
	}
}

void Solver::ResetTetVertCollision()
{
	cudaMemset(tetVertCollisionPos_d, 0, tetVertNum_d * 3 * sizeof(float));
	cudaMemset(tetVertCollisionDepth_d, 0.0f, tetVertNum_d * sizeof(float));
	cudaMemset(tetisCollide_d, 0, tetNum_d * sizeof(unsigned char));
	cudaMemset(tetVertisCollide_d, 0, tetVertNum_d * sizeof(unsigned char));
	cudaMemset(tetVertCollisionCnt_d, 0, tetVertNum_d * sizeof(int));
	cudaMemset(tetVertCollisionToolFlag_d, 0, tetVertNum_d * sizeof(int));
	cudaMemset(tvCollisionNum_d, 0, sizeof(int));
	cudaMemset(tetVertCollidedBuffer_d, 0, tetVertNum_d * 3 * sizeof(float));
	cudaMemset(tetVertCollidedNonPenetration_d, 0, tetVertNum_d * 3 * sizeof(float));
	cudaMemset(tetVertCollidedDepth_d, 0, tetVertNum_d * sizeof(float));
	cudaMemcpy(tetVertCollidedPos_d, tetVertPos_d, tetVertNum_d * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemset(tetVertCollidedToolFlag_d, 0, tetVertNum_d * sizeof(int));
	printCudaError("ResetTetVertCollision");
}

void Solver::HapticCollideTetGripper() {
	// 1. 重置碰撞信息
	// =>ResetCollision();
	cudaMemcpy(tetVertCollisionPos_d, tetVertPos_d, tetVertNum_d * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemset(tetVertCollisionDepth_d, 0.0f, tetVertNum_d * sizeof(float));
	cudaMemset(tetVertisCollide_d, 0, tetVertNum_d * sizeof(unsigned char));
	cudaMemset(tetVertCollisionToolFlag_d, -1, tetVertNum_d * sizeof(int));
	cudaMemset(tvCollisionNum_d, 0, sizeof(int));
	printCudaError("HapticCollideTetGripper memset");
	runResetTetVertToolDistance();
	printCudaError("HapticCollideTetGripper runResetTetVertToolDistance");

	cudaMemset(tetVertCollidedBuffer_d, 0, tetVertNum_d * 3 * sizeof(float));
	cudaMemset(tetVertCollidedNonPenetration_d, 0, tetVertNum_d * 3 * sizeof(float));
	cudaMemset(tetVertCollidedDepth_d, 0, tetVertNum_d * sizeof(float));
	printCudaError("HapticCollideTetGripper memset collided");
	cudaMemcpy(tetVertCollidedPos_d, tetVertPos_d, tetVertNum_d * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemset(tetVertCollidedToolFlag_d, -1, tetVertNum_d * sizeof(int));
	printCudaError("HapticCollideTetGripper memset collided");

	// 2. 根据上一帧的虚拟位姿更新当前工具的碰撞体位姿
	UpdateColliders();

	// 3. 碰撞检测
	SimManager* m = (SimManager*)m_manager;
	for (int index = 0; index < m->m_toolList.size(); index++) {
		int id = m->m_toolIndexList[index];
		if (id > 0 && id < m->m_toolList[index].size()) {
			vector<GripperCollider>& gripperColliders = m->m_toolList[index][id]->GetGripperColliders();
			for (GripperCollider& gripper : gripperColliders) {
				if (!gripper.active) continue;
                runHapticCollisionDetect_gripper(
                    gripper.scale[0], gripper.scale[1], gripper.scale[2], gripper.position[0], gripper.position[1], gripper.position[2], gripper.upper_x[0],
                    gripper.upper_x[1], gripper.upper_x[2], gripper.upper_y[0], gripper.upper_y[1], gripper.upper_y[2], gripper.upper_z[0], gripper.upper_z[1],
                    gripper.upper_z[2], gripper.lower_y[0], gripper.lower_y[1], gripper.lower_y[2], gripper.lower_z[0], gripper.lower_z[1], gripper.lower_z[2],
                    gripper.pivot_x[0], gripper.pivot_x[1], gripper.pivot_x[2], gripper.pivot_y[0], gripper.pivot_y[1], gripper.pivot_y[2], gripper.pivot_z[0],
                    gripper.pivot_z[1], gripper.pivot_z[2], gripper.angle, &(gripper.closeFlag), gripper.gripper_no, adsorbStiffness);
			}
		}
	}
	printCudaError("HapticCollideTetGripper");
}

void Solver::HapticCollideTriVert()
{   
	SimManager* m = (SimManager*)m_manager;
	// 1. 重置碰撞信息
	cudaMemset(springVertCollisionPos_d, 0, springVertNum_d * 3 * sizeof(float));
	cudaMemcpy(springVertCollisionNormal_d, springVertNonPenetrationDir_d, springVertNum_d * 3 * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemset(springVertCollisionDepth_d, 0.0f, springVertNum_d * sizeof(float));	
	cudaMemset(springVertisCollide_d, 0, springVertNum_d * sizeof(unsigned char));
	
	cudaMemset(springVertCollisionCnt_d, 0, springVertNum_d * sizeof(int));
	cudaMemset(springVertCollisionToolFlag_d, 0, springVertNum_d * sizeof(int));
	cudaMemset(hapticCollisionNum_d, 0, sizeof(int));
	
	printCudaError("HapticCollideTriVert memset");
	runResetSpringVertToolDistance();
	printCudaError("HapticCollideTriVert runResetVertToolDistance");


	// 2. 根据上一帧的虚拟位姿更新当前工具的碰撞体位姿
	UpdateColliders();
	printCudaError("HapticCollideTriVert UpdateColliders");

	// 3. 碰撞检测
	DetectCollisionSV();

	printCudaError("HapticCollideTriVert DetectCollisionSV");
}

void Solver::UpdateQH(float* buffer, int operatorIndex) {
	auto& operatorTrans = m_operatorTransList[operatorIndex];
	SetQH(buffer,
		operatorTrans.dirh, operatorTrans.thetah, operatorTrans.qh,
		operatorTrans.buffer_haptic);
	if (!operatorTrans.initialized) {
		static int cnt = 0;
		cnt++;
		memcpy(operatorTrans.qg.data(), operatorTrans.qh.data(), 6 * sizeof(float));
		operatorTrans.initialized = cnt > 10;
		cout << "operator[" << operatorIndex <<  "].qg at first step : " 
			<< operatorTrans.qg[0] << " " << operatorTrans.qg[1] << " " << operatorTrans.qg[2] << endl;
	}
	m_hapticSolver.set_k(k_vc, k_c, k_vct);
}