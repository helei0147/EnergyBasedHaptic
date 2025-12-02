#include "gpuvar.h"
#include "gpufun.h"

void printCudaError(const char* funcName) {
	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
	{
		fprintf(stderr, "%s error: %s\n", funcName, cudaGetErrorString(cudaStatus));
	}
}

void checkPointerLocation(void* ptr, const char* name) {
	cudaPointerAttributes attributes;
	cudaError_t error = cudaPointerGetAttributes(&attributes, ptr);

	if (error != cudaSuccess) {
		printf("Error: %s\n", cudaGetErrorString(error));
		return;
	}

	if (attributes.type == cudaMemoryTypeDevice) {
		printf("%s Pointer is on GPU device: %d\n", name, attributes.device);
		printf("Device pointer: %p\n", attributes.devicePointer);
		printf("Host pointer: %p\n", attributes.hostPointer);
	}
	else if (attributes.type == cudaMemoryTypeHost) {
		printf("%s Pointer is on Host (CPU)\n", name);
	}
	else if (attributes.type == cudaMemoryTypeManaged) {
		printf("%s Pointer is Unified Memory (managed)\n", name);
		printf("Associated device: %d\n", attributes.device);
	}
}

//计算初始状态
int runcalculateST(float damping, float dt) {
	//每个block中的线程数
	int  threadNum = 512;
	//每个grid中的block数(为了保证)
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;

	//并行计算
	calculateST << <blockNum, threadNum >> > (tetVertPos_d, tetVertVelocity_d, 
		tetVertExternForce_d, 
		tetVertPos_old_d, tetVertPos_prev_d, tetVertPos_last_d, 
		tetVertFixed_d, tetVertActive_d, gravityX_d, gravityY_d, gravityZ_d,
		tetVertNum_d, damping, dt);

	printCudaError("runcalculateST");
	return 0;
}

__global__ void calculateST(float* positions, float* velocity, float* externForce,
	float* old_positions, float* prev_positions, float* last_Positions, float* fixed, bool* vertIsActive,  
	float gravityX, float gravityY, float gravityZ,
	int vertexNum, float damping, float dt)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	//printf("calculateST vertNum:%d\n", vertexNum);
	if (threadid >= vertexNum) return;

	if (!vertIsActive[threadid]) return;

	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;


	last_Positions[indexX] = positions[indexX];
	last_Positions[indexY] = positions[indexY];
	last_Positions[indexZ] = positions[indexZ];
	float fixflag = fixed[threadid];
	velocity[indexX] *= damping* fixflag;
	velocity[indexY] *= damping * fixflag;
	velocity[indexZ] *= damping * fixflag;
	//施加重力
	velocity[indexX] += gravityX * dt * fixflag;
	velocity[indexY] += gravityY * dt * fixflag;
	velocity[indexZ] += gravityZ * dt * fixflag;
	//施加其他外力
	velocity[indexX] += externForce[indexX] * dt * fixflag;
	velocity[indexY] += externForce[indexY] * dt * fixflag;
	velocity[indexZ] += externForce[indexZ] * dt * fixflag;

	positions[indexX] += velocity[indexX] * dt * fixflag;
	positions[indexY] += velocity[indexY] * dt * fixflag;
	positions[indexZ] += velocity[indexZ] * dt * fixflag;

	cudaError_t cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
	{
		printf("%s error: %s\n", "velocity and position", cudaGetErrorString(cudaStatus));
	}
	//st
	old_positions[indexX] = positions[indexX];
	old_positions[indexY] = positions[indexY];
	old_positions[indexZ] = positions[indexZ];
	prev_positions[indexX] = positions[indexX];
	prev_positions[indexY] = positions[indexY];
	prev_positions[indexZ] = positions[indexZ];
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
	{
		printf("%s error: %s\n", "prev and old position", cudaGetErrorString(cudaStatus));
	}
	//外力清零
	externForce[indexX] = 0.0;
	externForce[indexY] = 0.0;
	externForce[indexZ] = 0.0;
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess)
	{
		printf("%s error: %s\n", "externForce", cudaGetErrorString(cudaStatus));
	}
}

__global__ void resetNANIF(float* force,
	int tetVertNum, int id, bool* globalPause) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetVertNum) return;

	if (isnan(force[threadid * 3 + 0]) || isnan(force[threadid * 3 + 1]) || isnan(force[threadid * 3 + 2])) {
		//printf("force is nan : %d  in %d\n", threadid, id);
		force[threadid * 3 + 0] = 0;
		force[threadid * 3 + 1] = 0;
		force[threadid * 3 + 2] = 0;
		*globalPause = true;
	}
	
}

__global__ void resetNANIFByTet(int* tetIndex,
	float* force, bool* active,
	int tetNum, bool* gpause) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetNum) return;

	if (!active[threadid]) {
		return;
	}
	int vIndex[4];
	vIndex[0] = tetIndex[threadid * 4 + 0];
	vIndex[1] = tetIndex[threadid * 4 + 1];
	vIndex[2] = tetIndex[threadid * 4 + 2];
	vIndex[3] = tetIndex[threadid * 4 + 3];
	bool gp = false;
	for (int i = 0; i < 4; i++) {
		if (isnan(force[vIndex[i] * 3 + 0]) || isnan(force[vIndex[i] * 3 + 1]) || isnan(force[vIndex[i] * 3 + 2]) || gp) {
			//printf("force is nan : %d  in %d\n", threadid);
			force[vIndex[i] * 3 + 0] = 0;
			force[vIndex[i] * 3 + 1] = 0;
			force[vIndex[i] * 3 + 2] = 0;
			gp = true;
			*gpause = true;
		}
	}

}

int runcalculateIF(float volumnStiffness) {
	int  threadNum = 512;
	int blockNum = (tetNum_d + threadNum - 1) / threadNum;

	//并行计算
	calculateIF << <blockNum, threadNum >> > (tetVertPos_d, tetIndex_d,
		tetInvD3x3_d, tetInvD3x4_d,
		tetVertForce_d, tetVolume_d, tetActive_d,
		tetNum_d, volumnStiffness);

	printCudaError("runcalculateIF");

	return 0;
}

__global__ void calculateRestStiffnessSV2TV(unsigned char* tetVertCollideFlag, float* tetVertToolDistance, float* restStiffness,
	float minStiffnessSV2TV, float maxStiffnessSV2TV,
	bool* tetVertActive, bool* tetVertIsBurned, int tetVertNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetVertNum) return;
	if (!tetVertActive[threadid] || tetVertIsBurned[threadid]) return;

	// 默认stiffness 1000
	switch (tetVertCollideFlag[threadid]) {
	case COLLIDE_TYPE::NO_COL: // 非碰撞点，TV主动
		restStiffness[threadid] = 0;
		break;
	case COLLIDE_TYPE::NORMAL: // 普通碰撞点，SV更加自由，太自由容易穿透，因此也得大一些【需要调整】
		restStiffness[threadid] = minStiffnessSV2TV;
		break;
	case COLLIDE_TYPE::HALF_GRAB:
		restStiffness[threadid] = minStiffnessSV2TV;
		break;
	case COLLIDE_TYPE::FULL_GRAB: // 夹取点，适中调整，还是TV主动效果好
		restStiffness[threadid] = 0;
		break;
	default:
		break;
	}
}

__device__ void MatrixSubstract_3_D(float* A, float* B, float* R)						//R=A-B
{
	for (int i = 0; i < 9; i++)	R[i] = A[i] - B[i];
}
__device__ void MatrixProduct_3_D(const float* A, const float* B, float* R)				//R=A*B
{
	R[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
	R[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
	R[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];
	R[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
	R[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
	R[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];
	R[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
	R[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
	R[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];
}

__device__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz)	//R=A*B
{
	memset(R, 0, sizeof(float) * nx * nz);
	for (int i = 0; i < nx; i++)
		for (int j = 0; j < nz; j++)
			for (int k = 0; k < ny; k++)
				R[i * nz + j] += A[i * ny + k] * B[k * nz + j];
}

__global__ void calculateIF(float* positions, int* tetIndex,
	float* tetInvD3x3, float* tetInvD3x4,
	float* force, float* tetVolumn, bool* active,
	int tetNum, float volumnStiffness) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetNum) return;

	if (!active[threadid]) {
		return;
	}

	//计算每个四面体初始化的shape矩阵的逆
	int vIndex0 = tetIndex[threadid * 4 + 0];
	int vIndex1 = tetIndex[threadid * 4 + 1];
	int vIndex2 = tetIndex[threadid * 4 + 2];
	int vIndex3 = tetIndex[threadid * 4 + 3];

	//先计算shape矩阵
	float D[9];
	D[0] = positions[vIndex1 * 3 + 0] - positions[vIndex0 * 3 + 0];
	D[1] = positions[vIndex2 * 3 + 0] - positions[vIndex0 * 3 + 0];
	D[2] = positions[vIndex3 * 3 + 0] - positions[vIndex0 * 3 + 0];
	D[3] = positions[vIndex1 * 3 + 1] - positions[vIndex0 * 3 + 1];
	D[4] = positions[vIndex2 * 3 + 1] - positions[vIndex0 * 3 + 1];
	D[5] = positions[vIndex3 * 3 + 1] - positions[vIndex0 * 3 + 1];
	D[6] = positions[vIndex1 * 3 + 2] - positions[vIndex0 * 3 + 2];
	D[7] = positions[vIndex2 * 3 + 2] - positions[vIndex0 * 3 + 2];
	D[8] = positions[vIndex3 * 3 + 2] - positions[vIndex0 * 3 + 2];

	//计算形变梯度F
	float F[9];
	MatrixProduct_3_D(D, &tetInvD3x3[threadid * 9], F);

	//从F中分解出R
	float R[9];
	GetRotation_D((float(*)[3])F, (float(*)[3])R);//转化为数组指针，即对应二维数组的形参要求

	MatrixSubstract_3_D(R, F, R);

	float temp[12];
	memset(temp, 0, sizeof(float) * 12);
	MatrixProduct_D(R, &tetInvD3x4[threadid * 12], temp, 3, 3, 4);
	for (int i = 0; i < 12; i++) {
		if (isnan(temp[i]))
			temp[i] = 0;

		temp[i] = temp[i] > 10 ? 10 : temp[i];
		temp[i] = temp[i] < -10 ? -10 : temp[i];
	}

	atomicAdd(force + vIndex0 * 3 + 0, temp[0] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex0 * 3 + 1, temp[4] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex0 * 3 + 2, temp[8] * tetVolumn[threadid] * volumnStiffness);

	atomicAdd(force + vIndex1 * 3 + 0, temp[1] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex1 * 3 + 1, temp[5] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex1 * 3 + 2, temp[9] * tetVolumn[threadid] * volumnStiffness);

	atomicAdd(force + vIndex2 * 3 + 0, temp[2] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex2 * 3 + 1, temp[6] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex2 * 3 + 2, temp[10] * tetVolumn[threadid] * volumnStiffness);

	atomicAdd(force + vIndex3 * 3 + 0, temp[3] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex3 * 3 + 1, temp[7] * tetVolumn[threadid] * volumnStiffness);
	atomicAdd(force + vIndex3 * 3 + 2, temp[11] * tetVolumn[threadid] * volumnStiffness);

}

//计算更新位置
int runcalculatePOS(float omega, float dt) {

	int  threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;
	//并行计算
	calculatePOS << <blockNum, threadNum >> > (tetVertPos_d, tetVertForce_d,
		tetVertFixed_d, tetVertMass_d,
		tetVertPos_next_d, tetVertPos_prev_d, tetVertPos_old_d,
		tetVolumeDiag_d, tetVertCollisionDiag_d, tetVertCollisionForce_d, tetVertActive_d,
		tetVertNum_d, dt, omega);

	//cudaDeviceSynchronize();
	printCudaError("runcalculatePOS");
	return 0;
}

//计算position
__global__ void calculatePOS(float* positions, float* force, float* fixed, float* mass,
	float* next_positions, float* prev_positions, float* old_positions,
	float* volumnDiag, float* collisionDiag, float* collisionForce, bool* vertIsActive,
	int vertexNum, float dt, float omega)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum|| !vertIsActive[threadid]) 
		return;

	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;
	float fixflag = fixed[threadid];
	float diagConstant = (mass[threadid]) / (dt * dt);
	float forceLen = sqrt(force[indexX] * force[indexX] + force[indexY] * force[indexY] + force[indexZ] * force[indexZ]);

	//计算每个点的shape match产生的约束部分，因为之前是按照每个四面体计算的，现在要摊到每个顶点上
	float elementX = force[indexX] + collisionForce[indexX];
	float elementY = force[indexY] + collisionForce[indexY];
	float elementZ = force[indexZ] + collisionForce[indexZ];


	bool look = false;
	//相当于先按重力运动，每次再在收重力的效果上再修正
	next_positions[indexX] = (diagConstant * (old_positions[indexX] - positions[indexX]) + elementX) / (volumnDiag[threadid] + collisionDiag[indexX] + diagConstant)* fixflag + positions[indexX];
	if (next_positions[indexX] != next_positions[indexX]) look = true, next_positions[indexX] = positions[indexX];
	
	next_positions[indexY] = (diagConstant * (old_positions[indexY] - positions[indexY]) + elementY) / (volumnDiag[threadid] + collisionDiag[indexY] + diagConstant) * fixflag + positions[indexY];
	if (next_positions[indexY] != next_positions[indexY]) look = true, next_positions[indexY] = positions[indexY];
	
	next_positions[indexZ] = (diagConstant * (old_positions[indexZ] - positions[indexZ]) + elementZ) / (volumnDiag[threadid] + collisionDiag[indexZ] + diagConstant) * fixflag + positions[indexZ];
	if (next_positions[indexZ] != next_positions[indexZ]) look = true, next_positions[indexZ] = positions[indexZ];

	//under-relaxation 和 切比雪夫迭代
	next_positions[indexX] = (next_positions[indexX] - positions[indexX]) * 0.6 + positions[indexX];
	next_positions[indexY] = (next_positions[indexY] - positions[indexY]) * 0.6 + positions[indexY];
	next_positions[indexZ] = (next_positions[indexZ] - positions[indexZ]) * 0.6 + positions[indexZ];

	// omega定义：omega = 4 / (4 - rho*rho*omega);
	next_positions[indexX] = omega * (next_positions[indexX] - prev_positions[indexX]) + prev_positions[indexX];
	next_positions[indexY] = omega * (next_positions[indexY] - prev_positions[indexY]) + prev_positions[indexY];
	next_positions[indexZ] = omega * (next_positions[indexZ] - prev_positions[indexZ]) + prev_positions[indexZ];

	prev_positions[indexX] = positions[indexX];
	prev_positions[indexY] = positions[indexY];
	prev_positions[indexZ] = positions[indexZ];

	positions[indexX] = next_positions[indexX];
	positions[indexY] = next_positions[indexY];
	positions[indexZ] = next_positions[indexZ];

	float deltax = positions[indexX] - prev_positions[indexX];
	float deltay = positions[indexY] - prev_positions[indexY];
	float deltaz = positions[indexZ] - prev_positions[indexZ];

	if (forceLen > 2)
	{
		float movement = sqrt(deltax * deltax + deltay * deltay + deltaz * deltaz);
	}

}


int runcalculateV(float dt) {
	int  threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;
	//并行计算
	calculateV << <blockNum, threadNum >> > (tetVertPos_d, tetVertVelocity_d, tetVertPos_last_d, tetVertActive_d, tetVertNum_d, dt);

	printCudaError("runcalculateV");
	return 0;

}

//计算速度更新
__global__ void calculateV(float* positions, float* velocity, float* last_positions, bool* vertIsActive, int vertexNum, float dt) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;
	velocity[threadid * 3 + 0] = (positions[threadid * 3 + 0] - last_positions[threadid * 3 + 0]) / dt;
	velocity[threadid * 3 + 1] = (positions[threadid * 3 + 1] - last_positions[threadid * 3 + 1]) / dt;
	velocity[threadid * 3 + 2] = (positions[threadid * 3 + 2] - last_positions[threadid * 3 + 2]) / dt;
}

__device__ void GetRotation_D(float F[3][3], float R[3][3])
{
	float C[3][3];
	memset(&C[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				C[i][j] += F[k][i] * F[k][j];

	float C2[3][3];
	memset(&C2[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				C2[i][j] += C[i][k] * C[j][k];

	float det = F[0][0] * F[1][1] * F[2][2] +
		F[0][1] * F[1][2] * F[2][0] +
		F[1][0] * F[2][1] * F[0][2] -
		F[0][2] * F[1][1] * F[2][0] -
		F[0][1] * F[1][0] * F[2][2] -
		F[0][0] * F[1][2] * F[2][1];

	R[0][0] = 1;
	R[0][1] = 0;
	R[0][2] = 0;
	R[1][0] = 0;
	R[1][1] = 1;
	R[1][2] = 0;
	R[2][0] = 0;
	R[2][1] = 0;
	R[2][2] = 1;
	//检查，避免invert
	if (det <= 0)
		return;


	float I_c = C[0][0] + C[1][1] + C[2][2];
	float I_c2 = I_c * I_c;
	float II_c = 0.5 * (I_c2 - C2[0][0] - C2[1][1] - C2[2][2]);
	float III_c = det * det;
	float k = I_c2 - 3 * II_c;

	float inv_U[3][3];
	if (k < 1e-5f)
	{
		float inv_lambda = 1 / sqrt(I_c / 3 + 0.000001f);
		memset(inv_U, 0, sizeof(float) * 9);
		inv_U[0][0] = inv_lambda;
		inv_U[1][1] = inv_lambda;
		inv_U[2][2] = inv_lambda;
	}
	else
	{
		float l = I_c * (I_c * I_c - 4.5 * II_c) + 13.5 * III_c;
		float k_root = sqrt(k);
		float value = l / (k * k_root + 0.0001f);
		if (value < -1.0) value = -1.0;
		if (value > 1.0) value = 1.0;
		float phi = acos(value);
		float lambda2 = (I_c + 2 * k_root * cos(phi / 3)) / 3.0;
		float lambda = sqrt(lambda2);

		float III_u = sqrt(III_c);

		if (det < 0)   
			III_u = -III_u;

		if (isnan(III_u))
			III_u = 1.f;

		float I_u = lambda + sqrt(-lambda2 + I_c + 2 * III_u / (lambda+0.0001f));
		float II_u = (I_u * I_u - I_c) * 0.5;

		float U[3][3];
		float inv_rate, factor;

		inv_rate = 1 / (I_u * II_u - III_u);

		if (isnan(inv_rate)) 
			inv_rate = 1.f;

		factor = I_u * III_u * inv_rate;

		memset(U, 0, sizeof(float) * 9);
		U[0][0] = factor;
		U[1][1] = factor;
		U[2][2] = factor;

		factor = (I_u * I_u - II_u) * inv_rate;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				U[i][j] += factor * C[i][j] - inv_rate * C2[i][j];

		inv_rate = 1 / III_u;
		if (isnan(inv_rate)) 
			inv_rate = 1.f;
		factor = II_u * inv_rate;
		memset(inv_U, 0, sizeof(float) * 9);
		inv_U[0][0] = factor;
		inv_U[1][1] = factor;
		inv_U[2][2] = factor;





		factor = -I_u * inv_rate;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				inv_U[i][j] += factor * U[i][j] + inv_rate * C[i][j];
	}




	memset(&R[0][0], 0, sizeof(float) * 9);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				R[i][j] += F[i][k] * inv_U[k][j];


}


