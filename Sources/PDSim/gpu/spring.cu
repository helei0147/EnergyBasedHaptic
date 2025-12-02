#include "gpuvar.h"
#include "gpufun.h"
//#include "../SimManager.h"
//extern SimManager* g_simManager;
//计算顶点的初速度
extern "C" int runcalculateSTMU(float damping, float dt) {

	//每个block中的线程数
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;
	calculateSTMU << <blockNum, threadNum >> > (springVertPos_d, springVertPos_old_d, springVertPos_prev_d, springVertVelocity_d, springVertExternForce_d, springVertFixed_d,
		springVertActive_d, gravityX_d, gravityY_d, gravityZ_d,
		springVertNum_d, damping, dt);

	//cudaDeviceSynchronize();
	printCudaError("runcalculateSTMU");
	return 0;
}


__global__ void calculateSTMU(float* positions, float* old_positions, float* prev_positions, float* velocity, float* externForce, float* fixed,
	bool* vertIsActive, float gravityX, float gravityY, float gravityZ,
	int vertexNum, float damping, float dt) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;

	if (!vertIsActive[threadid]) return;
	//printf("svid=140358 pos=[%.4f,%.4f,%.4f]\n", positions[140358 * SV_STRIDE + 0], positions[140358 * SV_STRIDE + 1], positions[140358 * SV_STRIDE + 2]);
	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;
	int sv_index_x = threadid * SV_STRIDE + 0;
	int sv_index_y = threadid * SV_STRIDE + 1;
	int sv_index_z = threadid * SV_STRIDE + 2;

	float fixflag = fixed[threadid];

	//运动的阻尼
	velocity[indexX] *= damping* fixflag;
	velocity[indexY] *= damping * fixflag;
	velocity[indexZ] *= damping * fixflag;

	//施加其他外力
	velocity[indexX] += externForce[indexX] * dt * fixflag;
	velocity[indexY] += externForce[indexY] * dt * fixflag;
	velocity[indexZ] += externForce[indexZ] * dt * fixflag;

	positions[sv_index_x] += velocity[indexX] * dt * fixflag;
	positions[sv_index_y] += velocity[indexY] * dt * fixflag;
	positions[sv_index_z] += velocity[indexZ] * dt * fixflag;

	//st
	old_positions[indexX] = positions[sv_index_x];
	old_positions[indexY] = positions[sv_index_y];
	old_positions[indexZ] = positions[sv_index_z];
	prev_positions[indexX] = positions[sv_index_x];
	prev_positions[indexY] = positions[sv_index_y];
	prev_positions[indexZ] = positions[sv_index_z];

	//外力清零
	externForce[indexX] = 0;
	externForce[indexY] = 0;
	externForce[indexZ] = 0;
}



__global__ void resetNANSpringForce(float* force,
	int tetVertNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetVertNum) return;
	
	if (isnan(force[threadid * 3 + 0])) {
		//printf("collision force is nan : %d\n", threadid);
		force[threadid * 3 + 0] = 0;
	}
	if (isnan(force[threadid * 3 + 1])) {
		//printf("collision force is nan : %d\n", threadid);
		force[threadid * 3 + 1] = 0;
	}
	if (isnan(force[threadid * 3 + 2])) {
		//printf("collision force is nan : %d\n", threadid);
		force[threadid * 3 + 2] = 0;
	}
}

__global__ void CollisionResponse(float* vertPos, float* colForce, float* colDiag,
	float* projPos, float* colDepth, unsigned char* colFlag, float colStiffness, int vertNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertNum) return;
	if (colDepth[threadid]<1e-5) return;
	if (colFlag[threadid] == 0) return;
	int idx = threadid * 3 + 0;
	int idy = threadid * 3 + 1;
	int idz = threadid * 3 + 2;
	float dirx = (projPos[idx] - vertPos[idx]) / colDepth[threadid];
	float diry = (projPos[idy] - vertPos[idy]) / colDepth[threadid];
	float dirz = (projPos[idz] - vertPos[idz]) / colDepth[threadid];

	colForce[idx] = colStiffness * (projPos[idx] - vertPos[idx]);
	colForce[idy] = colStiffness * (projPos[idy] - vertPos[idy]);
	colForce[idz] = colStiffness * (projPos[idz] - vertPos[idz]);
	colDiag[idx] = colStiffness * dirx * dirx;
	colDiag[idy] = colStiffness * diry * diry;
	colDiag[idz] = colStiffness * dirz * dirz;
}

int runCollisionResponse(float collisionStiffness)
{
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;

	CollisionResponse << <blockNum, threadNum >> > (springVertPos_d,
		springVertCollisionForce_d, springVertCollisionDiag_d,
		springVertCollisionPos_d, springVertCollisionDepth_d, springVertisCollide_d, 
		collisionStiffness, springVertNum_d);
	printCudaError("runCollisionResponse");
	return 0;
}
//计算顶点的受力
extern "C" int runcalculateIFMU(float springStiffness) {
	int threadNum = 512;
	int blockNum = (springNum_d + threadNum - 1) / threadNum;


	calculateIFMU << <blockNum, threadNum >> > (springVertPos_d, springVertForce_d, springVertCollisionDiag_d,// vert number
		springStiffness, springOrgLength_d, springIndex_d, springActive_d,//spring number
		springNum_d);
	printCudaError("runcalculateIFMU");

	return 0;
}

// 将stiffness作为相同的值，每帧从用户获取
__global__ void calculateIFMU(float* positions, float* force, float* forceDiag, float springStiffness, float* springOrigin, unsigned int* springIndex, bool* springActive, int springNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	//printf("in calculateIFMU\n");
	if (threadid >= springNum) return;

	if (!springActive[threadid]) {
		return;
	}
	//printf("svid=140358 pos=[%.4f,%.4f,%.4f]\n", positions[140358 * SV_STRIDE + 0], positions[140358 * SV_STRIDE + 1], positions[140358 * SV_STRIDE + 2]);
	int vIndex0 = springIndex[threadid * 2 + 0];
	int vIndex1 = springIndex[threadid * 2 + 1];
	int sv0_index_x = vIndex0 * SV_STRIDE + 0;
	int sv0_index_y = vIndex0 * SV_STRIDE + 1;
	int sv0_index_z = vIndex0 * SV_STRIDE + 2;
	int sv1_index_x = vIndex1 * SV_STRIDE + 0;
	int sv1_index_y = vIndex1 * SV_STRIDE + 1;
	int sv1_index_z = vIndex1 * SV_STRIDE + 2;

	//printf("threadid:%d, spring index:%d %d\n", threadid, vIndex0, vIndex1);
	//获取顶点坐标计算local解
	float pos0x = positions[sv0_index_x];
	float pos0y = positions[sv0_index_y];
	float pos0z = positions[sv0_index_z];
	float pos1x = positions[sv1_index_x];
	float pos1y = positions[sv1_index_y];
	float pos1z = positions[sv1_index_z];

	//计算local解d
	float dx = pos0x - pos1x;
	float dy = pos0y - pos1y;
	float dz = pos0z - pos1z;

	float length = sqrt(dx * dx + dy * dy + dz * dz);

	if (length < springOrigin[threadid]) return;
	dx = dx * (springOrigin[threadid] / length);
	dy = dy * (springOrigin[threadid] / length);
	dz = dz * (springOrigin[threadid] / length);

	//对应的两个端点的内力
	//这里应该需要原子操作
	float tempx = dx - pos0x + pos1x;
	float tempy = dy - pos0y + pos1y;
	float tempz = dz - pos0z + pos1z;
	float tempLen = sqrt(tempx * tempx + tempy * tempy + tempz * tempz);
	float tempDirX = tempx / (tempLen + 1e-3);
	float tempDirY = tempy / (tempLen + 1e-3);
	float tempDirZ = tempz / (tempLen + 1e-3);
	//if (tempx != tempx || tempy != tempy || tempz != tempz) printf("temp is nan!!!\n");
	atomicAdd(force + vIndex0 * 3 + 0, tempx * springStiffness);
	atomicAdd(force + vIndex0 * 3 + 1, tempy * springStiffness);
	atomicAdd(force + vIndex0 * 3 + 2, tempz * springStiffness);
	atomicAdd(forceDiag + vIndex0 * 3 + 0, tempDirX * tempDirX * springStiffness);
	atomicAdd(forceDiag + vIndex0 * 3 + 1, tempDirY * tempDirY * springStiffness);
	atomicAdd(forceDiag + vIndex0 * 3 + 2, tempDirZ * tempDirZ * springStiffness);

	atomicAdd(force + vIndex1 * 3 + 0, -tempx * springStiffness);
	atomicAdd(force + vIndex1 * 3 + 1, -tempy * springStiffness);
	atomicAdd(force + vIndex1 * 3 + 2, -tempz * springStiffness);
	atomicAdd(forceDiag + vIndex1 * 3 + 0, tempDirX * tempDirX * springStiffness);
	atomicAdd(forceDiag + vIndex1 * 3 + 1, tempDirY * tempDirY * springStiffness);
	atomicAdd(forceDiag + vIndex1 * 3 + 2, tempDirZ * tempDirZ * springStiffness);
}

int runcalculateRestPosForceWithTetPos_vCAG(float minStiffnessTV2SV, float maxStiffnessTV2SV)
{
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;

	calculateRestStiffnessTV2SV << <blockNum, threadNum >> > (
		springVertisCollide_d, springVertToolDistance_d, springVertfromTetStiffness_d,
		minStiffnessTV2SV, maxStiffnessTV2SV,
		springVertActive_d, springVertNum_d);
	calculateRestPosWithTetPosMU << <blockNum, threadNum >> > (springVertPos_d, springVert2TetVertMapping_d,
		springVertCollisionForce_d, springVertCollisionDiag_d,
		tetVertPos_forTri, springVertfromTetStiffness_d,
		springVertNum_d, springVertActive_d);
	//cudaDeviceSynchronize();
	printCudaError("runcalculateRestPosForceWithTetPos_vCAG");
	return 0;
}

__global__ void calculateRestStiffnessTV2SV(unsigned char* springVertCollideFlag, float* springVertToolDistance, float* restStiffness,
	float minStiffnessTV2SV, float maxStiffnessTV2SV,
	bool* springVertIsActive, int springVertexNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= springVertexNum) return;
	if (!springVertIsActive[threadid]) return;
	//默认restStiffness=100
	switch (springVertCollideFlag[threadid]) {
	case 0: // 非碰撞点，SV需要随动；距离工具越近越自由，必要性存疑
		float r_dis = 1.0f;
		if (springVertToolDistance[threadid] < 0)
		{
			printf("Error: distance of non-collided vert should >0, dis: %.3f\n", springVertToolDistance[threadid]);
			break;
		}
		float saturatedDis = fmin(r_dis, springVertToolDistance[threadid]);
		restStiffness[threadid] = minStiffnessTV2SV + (maxStiffnessTV2SV - minStiffnessTV2SV) * saturatedDis / r_dis;
		//restStiffness[threadid] = minStiffnessTV2SV;

		break;
	case 1: // 普通碰撞点，SV更加自由，太自由容易穿透，因此也得大一些【需要调整】
		restStiffness[threadid] = minStiffnessTV2SV;
		//printf("collision: tri vert col.\n");
		break;
	case 2: // 夹取点，SV大，跟随抓钳移动，正确性存疑
		restStiffness[threadid] = maxStiffnessTV2SV;
		break;
	default:
		restStiffness[threadid] = maxStiffnessTV2SV;
		break;
	}
}

__global__ void calculateRestPosWithTetPosMU(float* positions, int* skeletonIndex, float* force, float* collisionDiag, 
	float* rest_positions, float* restStiffness, int vertexNum, bool* vertIsActive) {
	/*说明：skeletonIndex大小为表面三角顶点数量x2
		每一个表面三角网格顶点对应两个整数，表示与其绑定的四面体顶点下标
		如果两个整数相同，表明这个顶点直接对应着第一个整数标示的四面体顶点
		否则，这个表面三角网格顶点是在表面网格细分过程中增加的顶点，restpos对应两个四面体顶点的中点。
	*/
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;
	int tet_idx1 = skeletonIndex[2 * threadid + 0]*3; // 四面体顶点1下标
	int tet_idx2 = skeletonIndex[2 * threadid + 1]*3; // 四面体顶点2下标

	int sv_index_x = threadid * SV_STRIDE + 0;
	int sv_index_y = threadid * SV_STRIDE + 1;
	int sv_index_z = threadid * SV_STRIDE + 2;

	//获取顶点坐标
	float tri_pos0x = positions[sv_index_x]; // 表面顶点坐标
	float tri_pos0y = positions[sv_index_y];
	float tri_pos0z = positions[sv_index_z];
	//计算受力
	float tempx = (rest_positions[tet_idx1 + 0] + rest_positions[tet_idx2 + 0]) * 0.5f - tri_pos0x;
	float tempy = (rest_positions[tet_idx1 + 1] + rest_positions[tet_idx2 + 1]) * 0.5f - tri_pos0y;
	float tempz = (rest_positions[tet_idx1 + 2] + rest_positions[tet_idx2 + 2]) * 0.5f - tri_pos0z;

	atomicAdd(force + threadid * 3 + 0, tempx * restStiffness[threadid]);
	atomicAdd(force + threadid * 3 + 1, tempy * restStiffness[threadid]);
	atomicAdd(force + threadid * 3 + 2, tempz * restStiffness[threadid]);

	atomicAdd(collisionDiag + threadid * 3 + 0, restStiffness[threadid]);
	atomicAdd(collisionDiag + threadid * 3 + 1, restStiffness[threadid]);
	atomicAdd(collisionDiag + threadid * 3 + 2, restStiffness[threadid]);
}

//切比雪夫更新位置
extern "C" int runcalculatePosMU(float omega, float dt) {
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;

	//sv_on_surface;
	//并行计算
	calculatePOSMU << <blockNum, threadNum >> > (springVertPos_d,
		springVertForce_d, springVertFixed_d, springVertMass_d,
		springVertPos_next_d, springVertPos_prev_d, springVertPos_old_d,
		springDiag_d, springVertCollisionDiag_d, springVertCollisionForce_d, springVertActive_d,
		springVertNum_d, dt, omega);

	printCudaError("runcalculatePosMU");
	return 0;
}


__global__ void calculatePOSMU(float* positions, float* force, float* fixed, float* mass, float* next_positions, float* prev_positions, float* old_positions, float* springDiag, float* collisionDiag, float* collisionForce,
	bool* vertIsActive, int vertexNum, float dt, float omega) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;
	int sv_index_x = threadid * SV_STRIDE + 0;
	int sv_index_y = threadid * SV_STRIDE + 1;
	int sv_index_z = threadid * SV_STRIDE + 2;

	float diagConstant = (mass[threadid] + fixed[threadid]) / (dt * dt);
	float elementX = force[indexX] + collisionForce[indexX];
	float elementY = force[indexY] + collisionForce[indexY];
	float elementZ = force[indexZ] + collisionForce[indexZ];

	//相当于先按重力运动，每次再在受重力的效果上再修正
	next_positions[indexX] = (diagConstant * (old_positions[indexX] - positions[sv_index_x]) + elementX) / (springDiag[threadid] + collisionDiag[indexX] + diagConstant) + positions[sv_index_x];
	if (next_positions[indexX] != next_positions[indexX]) next_positions[indexX] = positions[sv_index_x];
	next_positions[indexY] = (diagConstant * (old_positions[indexY] - positions[sv_index_y]) + elementY) / (springDiag[threadid] + collisionDiag[indexY] + diagConstant) + positions[sv_index_y];
	if (next_positions[indexY] != next_positions[indexY]) next_positions[indexY] = positions[sv_index_y];
	next_positions[indexZ] = (diagConstant * (old_positions[indexZ] - positions[sv_index_z]) + elementZ) / (springDiag[threadid] + collisionDiag[indexZ] + diagConstant) + positions[sv_index_z];
	if (next_positions[indexZ] != next_positions[indexZ]) next_positions[indexZ] = positions[sv_index_z];

	//float deltax = next_positions[indexX] - positions[indexX];
	//float deltay = next_positions[indexY] - positions[indexY];
	//float deltaz = next_positions[indexZ] - positions[indexZ];
	//if (abs(deltax) > 1)
	//{
	//	printf("ID: %d force: %.3f %.3f %.3f colForce: %.3f %.3f %.3f colDiag: %.3f %.3f %.3f\nm: %f delta: %.3f %.3f %.3f\n", threadid,
	//		force[indexX], force[indexY], force[indexZ],
	//		collisionForce[indexX], collisionForce[indexY], collisionForce[indexZ], collisionDiag[indexX], collisionDiag[indexY], collisionDiag[indexZ],
	//		diagConstant, deltax, deltay, deltaz);
	//}
	//under-relaxation 和 切比雪夫迭代
	next_positions[indexX] = (next_positions[indexX] - positions[sv_index_x]) * 0.6 + positions[sv_index_x];
	next_positions[indexY] = (next_positions[indexY] - positions[sv_index_y]) * 0.6 + positions[sv_index_y];
	next_positions[indexZ] = (next_positions[indexZ] - positions[sv_index_z]) * 0.6 + positions[sv_index_z];

	next_positions[indexX] = omega * (next_positions[indexX] - prev_positions[indexX]) + prev_positions[indexX];
	next_positions[indexY] = omega * (next_positions[indexY] - prev_positions[indexY]) + prev_positions[indexY];
	next_positions[indexZ] = omega * (next_positions[indexZ] - prev_positions[indexZ]) + prev_positions[indexZ];

	prev_positions[indexX] = positions[sv_index_x];
	prev_positions[indexY] = positions[sv_index_y];
	prev_positions[indexZ] = positions[sv_index_z];

	positions[sv_index_x] = next_positions[indexX];
	positions[sv_index_y] = next_positions[indexY];
	positions[sv_index_z] = next_positions[indexZ];
}


//更新速度
extern "C" int runcalculateVMU(float dt) {
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;
	//并行计算
	calculateVMU << <blockNum, threadNum >> > (springVertPos_d, springVertVelocity_d, springVertPos_old_d, springVertActive_d, springVertNum_d, dt);
	printCudaError("runcalculateVMU");
	return 0;
}

__global__ void calculateVMU(float* positions, float* velocity, float* old_positions, bool* vertIsActive, int vertexNum, float dt) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;
	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;
	int sv_index_x = threadid * SV_STRIDE + 0;
	int sv_index_y = threadid * SV_STRIDE + 1;
	int sv_index_z = threadid * SV_STRIDE + 2;

	velocity[indexX] += (positions[sv_index_x] - old_positions[indexX]) / dt;
	velocity[indexY] += (positions[sv_index_y] - old_positions[indexY]) / dt;
	velocity[indexZ] += (positions[sv_index_z] - old_positions[indexZ]) / dt;

}

__global__ void updateSurfaceDDir(float* normalBuffer,
	float* directDir, int vertexNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;

	int idx0 = threadid * 3 + 0;
	int idx1 = threadid * 3 + 1;
	int idx2 = threadid * 3 + 2;

	float n[3] = { normalBuffer[idx0],normalBuffer[idx1] ,normalBuffer[idx2] };
	// 归一化
	float length = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	if (length > 1e-6)
	{
		normalBuffer[threadid * 3 + 0] = n[0] / length;
		normalBuffer[threadid * 3 + 1] = n[1] / length;
		normalBuffer[threadid * 3 + 2] = n[2] / length;
		directDir[threadid * 3 + 0] = -n[0] / length;
		directDir[threadid * 3 + 1] = -n[1] / length;
		directDir[threadid * 3 + 2] = -n[2] / length;
	}
	else
	{
		normalBuffer[threadid * 3 + 0] = 1;
		normalBuffer[threadid * 3 + 1] = 0;
		normalBuffer[threadid * 3 + 2] = 0;
		directDir[threadid * 3 + 0] = 1;
		directDir[threadid * 3 + 1] = 0;
		directDir[threadid * 3 + 2] = 0;
	}
}

__global__ void updateTriNorm(float* vertPos, int* triIdx, float* normBuffer, int triNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= triNum) return;
	
	int id0 = triIdx[threadid * 3 + 0];
	int id1 = triIdx[threadid * 3 + 1];
	int id2 = triIdx[threadid * 3 + 2];
	float d0[3] = {
		vertPos[id1 * 3 + 0] - vertPos[id0 * 3 + 0],
		vertPos[id1 * 3 + 1] - vertPos[id0 * 3 + 1],
		vertPos[id1 * 3 + 2] - vertPos[id0 * 3 + 2],
	};
	float d1[3] = {
		vertPos[id2 * 3 + 0] - vertPos[id1 * 3 + 0],
		vertPos[id2 * 3 + 1] - vertPos[id1 * 3 + 1],
		vertPos[id2 * 3 + 2] - vertPos[id1 * 3 + 2],
	};
	float n[3];
	tetCross_D(d0, d1, n);

	atomicAdd(normBuffer + id0 * 3 + 0, n[0]);
	atomicAdd(normBuffer + id0 * 3 + 1, n[1]);
	atomicAdd(normBuffer + id0 * 3 + 2, n[2]);
	atomicAdd(normBuffer + id1 * 3 + 0, n[0]);
	atomicAdd(normBuffer + id1 * 3 + 1, n[1]);
	atomicAdd(normBuffer + id1 * 3 + 2, n[2]);
	atomicAdd(normBuffer + id2 * 3 + 0, n[0]);
	atomicAdd(normBuffer + id2 * 3 + 1, n[1]);
	atomicAdd(normBuffer + id2 * 3 + 2, n[2]);
}
int runUpdateSurfaceDDir()
{
	int threadNum = 512;
	int blockNum = (triNum_d + threadNum - 1) / threadNum;
	updateTriNorm << <blockNum, threadNum >> > (springVertPos_d, triIndex_d,
		springVertNorms_d, triNum_d);


	threadNum = 512;
	blockNum = (springVertNum_d + threadNum - 1) / threadNum;
	updateSurfaceDDir << <blockNum, threadNum >> > (springVertNorms_d,
		springVertNonPenetrationDir_d, springVertNum_d);
	printCudaError("runUpdateSurfaceDDir");
	return 0;
}

__global__ void resetVertToolDistance(float* vertToolDistance, int vertNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertNum) return;
	vertToolDistance[threadid] = FLT_MAX;
}

int runResetSpringVertToolDistance() {

	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;
	resetVertToolDistance << <blockNum, threadNum >> > (
		springVertToolDistance_d, springVertNum_d);

	printCudaError("runResetSpringVertToolDistance");
	return 0;
}

int runResetTetVertToolDistance() {

	int threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;
	resetVertToolDistance << <blockNum, threadNum >> > (
		tetVertToolDistance_d, tetVertNum_d);
	printCudaError("runResetTetVertToolDistance");

	return 0;
}

__global__ void updateSpringVert(float* positions, int* skeletonIndex,
	float* rest_positions, int vertexNum, bool* vertIsActive, bool* vertIsBurned) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid] || vertIsBurned[threadid]) return;
	int tet_idx1 = skeletonIndex[2 * threadid + 0] * 3; // 四面体顶点1下标
	int tet_idx2 = skeletonIndex[2 * threadid + 1] * 3; // 四面体顶点2下标
	int sv_index_x = threadid * SV_STRIDE + 0;
	int sv_index_y = threadid * SV_STRIDE + 1;
	int sv_index_z = threadid * SV_STRIDE + 2;

	positions[sv_index_x] = (rest_positions[tet_idx1 + 0] + rest_positions[tet_idx2 + 0]) * 0.5f;
	positions[sv_index_y] = (rest_positions[tet_idx1 + 1] + rest_positions[tet_idx2 + 1]) * 0.5f;
	positions[sv_index_z] = (rest_positions[tet_idx1 + 2] + rest_positions[tet_idx2 + 2]) * 0.5f;
}

