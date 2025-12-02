#include "gpuvar.h"
#include "gpufun.h"



// 全局变量
float gripper_max_angle = 0.9f;
float gripper_min_angle = 0.1f;

// 圆柱碰撞体的局部变量
char cylinder_active_t;
float cylinder_radius_t;
float cylinder_length_t;
__device__ float* cylinder_pos_t;
__device__ float* cylinder_last_pos_t;
__device__ float* cylinder_dir_t;
__device__ float* cylinder_shift_t;
__device__ unsigned char* cylinder_collide_flag_t;

// 球碰撞体的局部变量
char sphere_active_t;
float sphere_radius_t;
__device__ float* sphere_pos_t;
__device__ float* sphere_last_pos_t;

// 夹钳碰撞体的局部变量
char gripper_active_t;
float gripper_angle_t;
bool* gripper_unclose_t;
__device__ float* gripper_pos_t;
__device__ float* gripper_scale_t;
__device__ float* gripper_pivot_x_t;
__device__ float* gripper_pivot_y_t;
__device__ float* gripper_pivot_z_t;
__device__ float* gripper_upper_x_t;
__device__ float* gripper_upper_y_t;
__device__ float* gripper_upper_z_t;
__device__ float* gripper_lower_x_t;
__device__ float* gripper_lower_y_t;
__device__ float* gripper_lower_z_t;
__device__ unsigned int* tv_grab_flag_t;
__device__ unsigned int* tv_grab_half_flag_t;
__device__ float* tv_relative_pos_t;
__device__ unsigned int* sv_grab_flag_t;
__device__ unsigned int* sv_grab_half_flag_t;
__device__ float* sv_relative_pos_t;
__device__ float* gripper_adsorb_force_t;
__device__ int* gripper_grab_num_t;



__global__ void propagateCollision(unsigned char* svisCollide, float* svPos, float* svColDepth, float* svProjPos, int* svColToolFlag,
	int* s2tMapping,
	unsigned char* tvisCollide, float* tvPos, float* tvColDepth, float* tvProjPos, int* tvColToolFlag,
	int* colNum, int springVertNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= springVertNum) return;
	int tvIdx0 = s2tMapping[threadid * 2 + 0];
	int tvIdx1 = s2tMapping[threadid * 2 + 1];
	if (tvIdx0 != tvIdx1) return;//如果是细分之后的顶点，不传播碰撞

	if (svisCollide[threadid])
	{
		tvisCollide[tvIdx0] = 1;
		tvColDepth[tvIdx0] = svColDepth[threadid];
		tvProjPos[tvIdx0 * 3 + 0] = svProjPos[threadid * 3 + 0];
		tvProjPos[tvIdx0 * 3 + 1] = svProjPos[threadid * 3 + 1];
		tvProjPos[tvIdx0 * 3 + 2] = svProjPos[threadid * 3 + 2];
		tvColToolFlag[tvIdx0] = svColToolFlag[threadid];
		atomicAdd(colNum, 1);
	}
}

void PropagateCollision(void)
{
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;

	propagateCollision << <blockNum, threadNum >> > (
		springVertisCollide_forTet, springVertPos_forTet, springVertCollisionDepth_forTet, springVertCollisionPos_forTet, springVertCollisionToolFlag_forTet,
		springVert2TetVertMapping_forTet,
		tetVertisCollide_d, tetVertPos_d, tetVertCollisionDepth_d, tetVertCollisionPos_d, tetVertCollisionToolFlag_d, 
		tvCollisionNum_d, springVertNum_d);

	printCudaError("PropagateCollision");
}

__global__ void hapticCalculatePrefixSum(
	unsigned char* isCollide, unsigned int* queueIndex,
	unsigned int* auxArray, int vertexNum) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;

	//编译器这个数组的大小未知
	extern __shared__ unsigned int temp[];


	//给块内共享内存分配数据
	temp[threadIdx.x] = isCollide[threadid];

	for (unsigned int stride = 1; stride <= blockDim.x; stride *= 2) {
		__syncthreads();
		int index = (threadIdx.x + 1) * stride * 2 - 1;
		if (index < blockDim.x)
			temp[index] += temp[index - stride];//index is alway bigger than stride
		__syncthreads();
	}
	for (unsigned int stride = blockDim.x / 2; stride > 0; stride /= 2) {
		__syncthreads();
		int index = (threadIdx.x + 1) * stride * 2 - 1;
		if (index + stride < blockDim.x)
			temp[index + stride] += temp[index];
	}
	__syncthreads();

	//更新每个block内的前缀和
	queueIndex[threadid] = temp[threadIdx.x];
	__syncthreads();

	//计算整个block的和
	if (threadid % (blockDim.x) == (blockDim.x - 1) && threadid != 0) {
		auxArray[blockIdx.x] = queueIndex[threadid];
	}
}
__global__ void hapticAddCollisionToQueueTri(
	unsigned char* isCollide, float* tetPositions, float* triVertNonPenetrationDir, float* triVertDepth, float* tetVertCollisionPos, int* tetVertCollisionToolFlag,
	float* constraintPoints, float* constriantNonPenetrationDir, float* constraintDepth, float* tetVertCollidedPos, int* tetVertCollidedToolFlag,
	unsigned int* queueIndex, unsigned int* auxArray, int vertexNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;

	if (isCollide[threadid]) {
		int index = -1;
		//计算index
		for (int block = 0; block < blockIdx.x; block++) {
			index += auxArray[block];
		}

		index += queueIndex[threadid];
		constraintPoints[index * 3 + 0] = tetPositions[threadid * 3 + 0];
		constraintPoints[index * 3 + 1] = tetPositions[threadid * 3 + 1];
		constraintPoints[index * 3 + 2] = tetPositions[threadid * 3 + 2];
		constriantNonPenetrationDir[index * 3 + 0] = triVertNonPenetrationDir[threadid * 3 + 0];
		constriantNonPenetrationDir[index * 3 + 1] = triVertNonPenetrationDir[threadid * 3 + 1];
		constriantNonPenetrationDir[index * 3 + 2] = triVertNonPenetrationDir[threadid * 3 + 2];
		tetVertCollidedPos[index * 3 + 0] = tetVertCollisionPos[threadid * 3 + 0];
		tetVertCollidedPos[index * 3 + 1] = tetVertCollisionPos[threadid * 3 + 1];
		tetVertCollidedPos[index * 3 + 2] = tetVertCollisionPos[threadid * 3 + 2];
		tetVertCollidedToolFlag[index] = tetVertCollisionToolFlag[threadid];
		constraintDepth[index] = -triVertDepth[threadid];
	}
}

__global__ void hapticAddCollisionToQueue(
	unsigned char* isCollide, float* tetPositions, float* triVertDepth, float* tetVertCollisionPos, int* tetVertCollisionToolFlag,
	float* constraintPoints, float* constraintDepth, float* tetVertCollidedPos, int* tetVertCollidedToolFlag,
	unsigned int* queueIndex, unsigned int* auxArray, int vertexNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;

	if (isCollide[threadid]) {
		int index = -1;
		//计算index
		for (int block = 0; block < blockIdx.x; block++) {
			index += auxArray[block];
		}

		index += queueIndex[threadid];
		constraintPoints[index * 3 + 0] = tetPositions[threadid * 3 + 0];
		constraintPoints[index * 3 + 1] = tetPositions[threadid * 3 + 1];
		constraintPoints[index * 3 + 2] = tetPositions[threadid * 3 + 2];
		tetVertCollidedPos[index * 3 + 0] = tetVertCollisionPos[threadid * 3 + 0];
		tetVertCollidedPos[index * 3 + 1] = tetVertCollisionPos[threadid * 3 + 1];
		tetVertCollidedPos[index * 3 + 2] = tetVertCollisionPos[threadid * 3 + 2];
		tetVertCollidedToolFlag[index] = tetVertCollisionToolFlag[threadid];
		constraintDepth[index] = -triVertDepth[threadid];
	}
}

// 行优先排列矩阵result
__device__ void DeviceVec3MulVec3T(float* v0, float* v1, float* result)
{
	result[0] = v0[0] * v1[0];
	result[1] = v0[0] * v1[1];
	result[2] = v0[0] * v1[2];
	result[3] = v0[1] * v1[0];
	result[4] = v0[1] * v1[1];
	result[5] = v0[1] * v1[2];
	result[6] = v0[2] * v1[0];
	result[7] = v0[2] * v1[1];
	result[8] = v0[2] * v1[2];
}

// 行优先排列矩阵
__device__ void DeviceVec3toSkewSymmetricMatrix(
	float* v, float* m)
{
	m[0] = 0; m[1] = -v[2]; m[2] = v[1];
	m[3] = v[2]; m[4] = 0; m[5] = -v[0];
	m[6] = -v[1]; m[7] = v[0]; m[8] = 0;
}

// 行优先排列矩阵
__device__ void DeviceMat3MulMat3(
	float* m0, float* m1, float* result)
{
	result[0] = m0[0] * m1[0] + m0[1] * m1[3] + m0[2] * m1[6];
	result[1] = m0[0] * m1[1] + m0[1] * m1[4] + m0[2] * m1[7];
	result[2] = m0[0] * m1[2] + m0[1] * m1[5] + m0[2] * m1[8];
	result[3] = m0[3] * m1[0] + m0[4] * m1[3] + m0[5] * m1[6];
	result[4] = m0[3] * m1[1] + m0[4] * m1[4] + m0[5] * m1[7];
	result[5] = m0[3] * m1[2] + m0[4] * m1[5] + m0[5] * m1[8];
	result[6] = m0[6] * m1[0] + m0[7] * m1[3] + m0[8] * m1[6];
	result[7] = m0[6] * m1[1] + m0[7] * m1[4] + m0[8] * m1[7];
	result[8] = m0[6] * m1[2] + m0[7] * m1[5] + m0[8] * m1[8];
}

__device__ void DeviceScaleMulMat3(float s, float* m, float* result)
{
	result[0] = m[0] * s;
	result[1] = m[1] * s;
	result[2] = m[2] * s;
	result[3] = m[3] * s;
	result[4] = m[4] * s;
	result[5] = m[5] * s;
	result[6] = m[6] * s;
	result[7] = m[7] * s;
	result[8] = m[8] * s;
}

__device__ void DeviceMat3AddMat3(float* m0, float* m1, float* result)
{
	result[0] = m0[0] + m1[0];
	result[1] = m0[1] + m1[1];
	result[2] = m0[2] + m1[2];
	result[3] = m0[3] + m1[3];
	result[4] = m0[4] + m1[4];
	result[5] = m0[5] + m1[5];
	result[6] = m0[6] + m1[6];
	result[7] = m0[7] + m1[7];
	result[8] = m0[8] + m1[8];
}

__device__ void DeviceMat3AtomicAddMat3(float* m, float* result)
{
	atomicAdd(result + 0, m[0]);
	atomicAdd(result + 1, m[1]);
	atomicAdd(result + 2, m[2]);
	atomicAdd(result + 3, m[3]);
	atomicAdd(result + 4, m[4]);
	atomicAdd(result + 5, m[5]);
	atomicAdd(result + 6, m[6]);
	atomicAdd(result + 7, m[7]);
	atomicAdd(result + 8, m[8]);
}

// 行优先排列矩阵
__device__ void DeviceMatrixDotVec(
	float* m, float* v, float* result
)
{
	result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
	result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
	result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

// -------------------------------- //
__device__ bool collisionDetect_cylinder(
	float* vertPos, float* vertDDir,
	float* cylinderPos, float* cylinderDir, float cylinderRadius, float cylinderLength,
	float* collisionPos, float* collisionNormal, float* depth, float* distance
) {
	float cylinder0x = cylinderPos[0];
	float cylinder0y = cylinderPos[1];
	float cylinder0z = cylinderPos[2];
	float cylinder1x = cylinderPos[0] + cylinderDir[0] * cylinderLength;
	float cylinder1y = cylinderPos[1] + cylinderDir[1] * cylinderLength;
	float cylinder1z = cylinderPos[2] + cylinderDir[2] * cylinderLength;
	float cylinderdx = cylinder1x - cylinder0x;
	float cylinderdy = cylinder1y - cylinder0y;
	float cylinderdz = cylinder1z - cylinder0z;
	float dx = vertPos[0] - cylinder0x;
	float dy = vertPos[1] - cylinder0y;
	float dz = vertPos[2] - cylinder0z;

	float t = cylinderDir[0] * dx + cylinderDir[1] * dy + cylinderDir[2] * dz;
	t /= cylinderLength;
	if (t < 0) t = 0;
	if (t > 1) t = 1;

	dx = vertPos[0] - (cylinder0x + t * cylinderdx);
	dy = vertPos[1] - (cylinder0y + t * cylinderdy);
	dz = vertPos[2] - (cylinder0z + t * cylinderdz);
	*distance = sqrt(dx * dx + dy * dy + dz * dz);


	collisionPos[0] = vertPos[0];
	collisionPos[1] = vertPos[1];
	collisionPos[2] = vertPos[2];

	float colDirX = dx / (*distance + 1e-4);
	float colDirY = dy / (*distance + 1e-4);
	float colDirZ = dz / (*distance + 1e-4);
	float dot = vertDDir[0] * colDirX + vertDDir[1] * colDirY + vertDDir[2] * colDirZ;
	if (dot < 0)// 碰撞方向和顶点指导向量夹角大于90°（参考情况：排出方向与指导向量方向一致是最理想的碰撞情况，大于90°按照工具排出方向排出）
	{
		collisionNormal[0] = colDirX;
		collisionNormal[1] = colDirY;
		collisionNormal[2] = colDirZ;
	}
	else
	{
		collisionNormal[0] = vertDDir[0];
		collisionNormal[1] = vertDDir[1];
		collisionNormal[2] = vertDDir[2];
	}

	if (*distance > cylinderRadius) {
		*depth = 0;
		return false;
	}
	else if (*distance < 1e-5) {
		*depth = cylinderRadius - *distance;
		return true;
	}
	else {
		// 计算射线与圆柱的交点
        //tetCross_D
        float cylinderLen = sqrt(cylinderdx * cylinderdx + cylinderdy * cylinderdy + cylinderdz * cylinderdz);
        if (fabs(cylinderLen) < 1e-5f) printf("ERROR!!!!!!!!!!!!!!!!!! cylinderLen = %f\n", cylinderLen);
        float ABNorm[3] = {cylinderdx / cylinderLen, cylinderdy / cylinderLen, cylinderdz / cylinderLen};
        float AP[3] = {vertPos[0] - cylinder0x, vertPos[1] - cylinder0y, vertPos[2] - cylinder0z};
        float APCrossAB[3];
        tetCross_D(AP, ABNorm, APCrossAB);
        float DirCrossAB[3];
        tetCross_D(vertDDir, ABNorm, DirCrossAB);
        float a = tetDot_D(DirCrossAB, DirCrossAB);
        if (fabs(a) < 1e-5f) printf("ERROR!!!!!!!!!!!!!!!!!! a = %f\n", a);
		float b = 2 * tetDot_D(DirCrossAB, APCrossAB);
        float c = tetDot_D(APCrossAB, APCrossAB) - cylinderRadius * cylinderRadius;
        float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
		float intersection[3] = {vertPos[0] + t * vertDDir[0], vertPos[1] + t * vertDDir[1], vertPos[2] + t * vertDDir[2]};

		// 判断圆柱交点是否合法
        float AX[3] = {intersection[0] - cylinder0x, intersection[1] - cylinder0y, intersection[2] - cylinder0z};
        float dot = tetDot_D(AX, ABNorm) / cylinderLength;
        if (dot >= 0 && dot <= 1) {
            collisionPos[0] = intersection[0];
            collisionPos[1] = intersection[1];
            collisionPos[2] = intersection[2];
            float delta[3] = {intersection[0] - vertPos[0], intersection[1] - vertPos[1], intersection[2] - vertPos[2]};
            *depth = tetNormal_D(delta);

            collisionNormal[0] = delta[0];
            collisionNormal[1] = delta[1];
            collisionNormal[2] = delta[2];
            return true;
		} else{
            float spherePos[3];
            if (dot < 0) {
                spherePos[0] = cylinder0x;
				spherePos[1] = cylinder0y;
				spherePos[2] = cylinder0z;
            } else {
				spherePos[0] = cylinder1x;
				spherePos[1] = cylinder1y;
				spherePos[2] = cylinder1z;
			}
            float SP[3] = {vertPos[0] - spherePos[0], vertPos[1] - spherePos[1], vertPos[2] - spherePos[2]};
            float a = 1;
			float b = 2 * tetDot_D(SP, vertDDir);
			float c = tetDot_D(SP, SP) - cylinderRadius * cylinderRadius;
			if (b * b - 4 * a * c < 0) {
                float SP0[3] = {vertPos[0] - cylinder0x, vertPos[1] - cylinder0y, vertPos[2] - cylinder0z};
				float SP1[3] = {vertPos[0] - cylinder1x, vertPos[1] - cylinder1y, vertPos[2] - cylinder1z};
                float SP0CrossDir[3], SP1CrossDir[3];
                tetCross_D(SP0, vertDDir, SP0CrossDir);
				tetCross_D(SP1, vertDDir, SP1CrossDir);
                float dis0 = fabs(tetDot_D(SP0CrossDir, SP0CrossDir));
                float dis1 = fabs(tetDot_D(SP1CrossDir, SP1CrossDir));
				printf("ERROR!!!!!!!!!!!!!!!!!! b * b - 4 * a * c = %f, dot = %f, shperePos = [%f, %f, %f], vertPos = [%f, %f, %f], vertDDir = [%f, %f, %f], cylinder0 = [%f, %f, %f], cylinder1 = [%f, %f, %f], dis0 = %f, dis1 = %f\n", 
					b * b - 4 * a * c, dot, spherePos[0], spherePos[1], spherePos[2], vertPos[0], vertPos[1], vertPos[2], vertDDir[0], vertDDir[1], vertDDir[2],
                    cylinder0x, cylinder0y, cylinder0z, cylinder1x, cylinder1y, cylinder1z, dis0, dis1);
				
			}
			float t = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
            float intersection[3] = {vertPos[0] + t * vertDDir[0], vertPos[1] + t * vertDDir[1], vertPos[2] + t * vertDDir[2]};

			collisionPos[0] = intersection[0];
            collisionPos[1] = intersection[1];
            collisionPos[2] = intersection[2];
            float delta[3] = {intersection[0] - vertPos[0], intersection[1] - vertPos[1], intersection[2] - vertPos[2]};
            *depth = tetNormal_D(delta);
            if (*depth < 1e-5f) {
                printf("intersection is close to point, dot = %f\n", dot);
			}
            collisionNormal[0] = delta[0];
            collisionNormal[1] = delta[1];
            collisionNormal[2] = delta[2];
            return true;
		} 
	}
}

__global__ void hapticCollisionDetect_cylinder(
	int tetVertNum, bool* tetVertIsActive,
	float* tetVertPos, float* tetVertDDir,
	float cylinderPosX, float cylinderPosY, float cylinderPosZ,
	float cylinderLastPosX, float cylinderLastPosY, float cylinderLastPosZ,
	float cylinderDirX, float cylinderDirY, float cylinderDirZ,
	float cylinderRadius, float cylinderLength,
	int toolFlag, int* tetVertCollideToolFlag,
	float* tetVertCollisionPos, float* tetVertCollisionNormal,
	float* tetVertCollisionDepth, float* tetVertToolDistance, 
	unsigned char* tetVertIsCollide, int* collisionNumPtr
) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetVertNum) return;
	if (!tetVertIsActive[threadid]) return;

	if (tetVertIsCollide[threadid]) return;

	int tvidx = threadid * 3 + 0;
	int tvidy = threadid * 3 + 1;
	int tvidz = threadid * 3 + 2;
	
	float vertPos[3] = { 
		tetVertPos[tvidx], 
		tetVertPos[tvidy], 
		tetVertPos[tvidz] };
	float directDir[3] = { 
		tetVertDDir[tvidx], 
		tetVertDDir[tvidy], 
		tetVertDDir[tvidz] };
	float cylinderPos[3] = { 
		cylinderPosX, 
		cylinderPosY, 
		cylinderPosZ };
	float cylinderDir[3] = { 
		cylinderDirX, 
		cylinderDirY, 
		cylinderDirZ };
	float cylinderMoveDir[3] = {	
		cylinderLastPosX - cylinderPosX,
		cylinderLastPosY - cylinderPosY,
		cylinderLastPosZ - cylinderPosZ };
	float cylinderMoveDis = tetNormal_D(cylinderMoveDir);

	// 离散碰撞检测
	float collisionPos[3], collisionNormal[3];
	float depth = 0, distance = FLT_MAX;
	bool collision = collisionDetect_cylinder(
		vertPos, directDir,
		cylinderPos, cylinderDir, cylinderRadius, cylinderLength,
		collisionPos, collisionNormal, &depth, &distance);

	// 更新投影位置
	tetVertCollisionPos[tvidx] = collisionPos[0];
	tetVertCollisionPos[tvidy] = collisionPos[1];
	tetVertCollisionPos[tvidz] = collisionPos[2];

	// 更新碰撞方向
	tetVertCollisionNormal[tvidx] = collisionNormal[0];
	tetVertCollisionNormal[tvidy] = collisionNormal[1];
	tetVertCollisionNormal[tvidz] = collisionNormal[2];

	// 更新嵌入深度
	tetVertCollisionDepth[threadid] = depth;

	// 更新顶点到工具的距离
	distance -= cylinderRadius;
	tetVertToolDistance[threadid] = (tetVertToolDistance[threadid] < distance) ? tetVertToolDistance[threadid] : distance;
	
	// 更新碰撞标记位以及碰撞数
	if (collision) {
		tetVertIsCollide[threadid] = 1;
		tetVertCollideToolFlag[threadid] = toolFlag;
		atomicAdd(collisionNumPtr, 1);

	}
}

void runHapticCDSV_cylinder(
	float cylinderPosX, float cylinderPosY, float cylinderPosZ,
	float cylinderLastPosX, float cylinderLastPosY, float cylinderLastPosZ,
	float cylinderDirX, float cylinderDirY, float cylinderDirZ,
	float cylinderRadius, float cylinderLength,
	int toolFlag) 
{
	int threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;


	hapticCollisionDetect_cylinder << < blockNum, threadNum >> > (
		springVertNum_d, springVertActive_d, 
		springVertPos_d, springVertNonPenetrationDir_d,
		cylinderPosX, cylinderPosY, cylinderPosZ,
		cylinderLastPosX, cylinderLastPosY, cylinderLastPosZ,
		cylinderDirX, cylinderDirY, cylinderDirZ,
		cylinderRadius, cylinderLength,
		toolFlag, springVertCollisionToolFlag_d,
		springVertCollisionPos_d, springVertCollisionNormal_d,
		springVertCollisionDepth_d, springVertToolDistance_d,
		springVertisCollide_d, hapticCollisionNum_d
		);
}
__global__ void spreadCollisionByV(
	int tetVertNum, bool* tetVertIsActive, float* tetVertPos,
	float tetVertPosX, float tetVertPosY, float tetVertPosZ,
	float tetVertCollidedPosX, float tetVertCollidedPosY, float tetVertCollidedPosZ,
	float collisionStiffness, float spreadRadius, float maxVelocity,
	float* tetVertColVelocityMax,
	float* velocity) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= tetVertNum) return;
	if (!tetVertIsActive[threadid]) return;

	int tvidx = threadid * 3 + 0;
	int tvidy = threadid * 3 + 1;
	int tvidz = threadid * 3 + 2;

	float dx = tetVertPosX - tetVertPos[tvidx];
	float dy = tetVertPosY - tetVertPos[tvidy];
	float dz = tetVertPosZ - tetVertPos[tvidz];

	float distance = sqrt(dx * dx + dy * dy + dz * dz);
	if (distance > spreadRadius) return;

	float deltaPos[3];
	deltaPos[0] = tetVertCollidedPosX - tetVertPosX;
	deltaPos[1] = tetVertCollidedPosY - tetVertPosY;
	deltaPos[2] = tetVertCollidedPosZ - tetVertPosZ;

	float collisionNormal[3];
	float length = sqrt(deltaPos[0] * deltaPos[0] + deltaPos[1] * deltaPos[1] + deltaPos[2] * deltaPos[2]);
	if (length < 1e-5) return;
	collisionNormal[0] = deltaPos[0] / length;
	collisionNormal[1] = deltaPos[1] / length;
	collisionNormal[2] = deltaPos[2] / length;

	float temp[3];
	temp[0] = collisionStiffness / 10 * deltaPos[0];
	temp[1] = collisionStiffness / 10 * deltaPos[1];
	temp[2] = collisionStiffness / 10 * deltaPos[2];

	
	float tempLength = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);

	// 更新最大碰撞速度
	float preV[3] = { tetVertColVelocityMax[tvidx], tetVertColVelocityMax[tvidy], tetVertColVelocityMax[tvidz] };
	float preVLength = sqrt(preV[0] * preV[0] + preV[1] * preV[1] + preV[2] * preV[2]);

	velocity[tvidx] -= preV[0];
	velocity[tvidy] -= preV[1];
	velocity[tvidz] -= preV[2];

	if (tempLength > preVLength) {
		tetVertColVelocityMax[tvidx] = (tempLength < maxVelocity) ? temp[0] : (temp[0] / tempLength * maxVelocity);
		tetVertColVelocityMax[tvidy] = (tempLength < maxVelocity) ? temp[1] : (temp[1] / tempLength * maxVelocity);
		tetVertColVelocityMax[tvidz] = (tempLength < maxVelocity) ? temp[2] : (temp[2] / tempLength * maxVelocity);
	}

	velocity[tvidx] += tetVertColVelocityMax[tvidx];
	velocity[tvidy] += tetVertColVelocityMax[tvidy];
	velocity[tvidz] += tetVertColVelocityMax[tvidz];
}

void runApplyCollision(float tetVertPosX, float tetVertPosY, float tetVertPosZ,
	float tetVertCollidedPosX, float tetVertCollidedPosY, float tetVertCollidedPosZ,
	float tv_colStiffness, float tv_colSpreadRadius, float tv_maxColVelocity) {
	int  threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;
	spreadCollisionByV << < blockNum, threadNum >> > (
		tetVertNum_d, tetVertActive_d, tetVertPos_d,
		tetVertPosX, tetVertPosY, tetVertPosZ,
		tetVertCollidedPosX, tetVertCollidedPosY, tetVertCollidedPosZ,
		tv_colStiffness, tv_colSpreadRadius, tv_maxColVelocity,
		tetVertColVelocityMax_d,
		tetVertVelocity_d
		);
}

void runMergeCollisionInfoTet()
{
		
	int  threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;

	//同样使用前缀和将碰撞结果放置到碰撞队列中
	//得到碰撞点之后，计算前缀和得到在队列中的索引(第三个参数是共享内存大小)
	hapticCalculatePrefixSum << <blockNum, threadNum, threadNum * sizeof(unsigned int) >> > (
		tetVertisCollide_d, tetVertQueueIndex_d,
		tetVertAuxSumArray_d, tetVertNum_d);
	printCudaError("runMergeCollisionInfoTet prefix");
	////再根据索引，填写碰撞点到队列
	hapticAddCollisionToQueue << <blockNum, threadNum >> > (
		tetVertisCollide_d, tetVertPos_d, tetVertCollisionDepth_d, tetVertCollisionPos_d, tetVertCollisionToolFlag_d,
		tetVertCollidedBuffer_d, tetVertCollidedDepth_d, tetVertCollidedPos_d, tetVertCollidedToolFlag_d,
		tetVertQueueIndex_d, tetVertAuxSumArray_d,
		tetVertNum_d);
	printCudaError("runMergeCollisionInfoTet toqueue");
}

void runMergeCollisionInfoTriVert()
{
	int  threadNum = 512;
	int blockNum = (springVertNum_d + threadNum - 1) / threadNum;

	//同样使用前缀和将碰撞结果放置到碰撞队列中
	//得到碰撞点之后，计算前缀和得到在队列中的索引(第三个参数是共享内存大小)
	hapticCalculatePrefixSum << <blockNum, threadNum, threadNum * sizeof(unsigned int) >> > (
		springVertisCollide_d, springVertQueueIndex_d,
		springVertAuxSumArray_d, springVertNum_d);

	printCudaError("runMergeCollisionInfoTet prefix");
	//再根据索引，填写碰撞点到队列
	hapticAddCollisionToQueueTri << <blockNum, threadNum >> > (
		springVertisCollide_d, springVertPos_d, springVertNonPenetrationDir_d, springVertCollisionDepth_d, springVertCollisionPos_d, springVertCollisionToolFlag_d,
		springVertCollidedBuffer_d, springVertCollidedNonPenetration_d, springVertCollidedDepth_d, springVertCollidedPos_d, springVertCollidedToolFlag_d,
		springVertQueueIndex_d, springVertAuxSumArray_d,
		springVertNum_d);

	printCudaError("runMergeCollisionInfoTet toqueue");
}

__global__ void hapticCollisionDetect_gripper_half(
	float scaleX, float scaleY, float scaleZ,
	float px, float py, float pz,
	float x0, float x1, float x2,
	float y0, float y1, float y2,
	float z0, float z1, float z2,
	unsigned int* grab_half_flag, unsigned int grabFlag,
	float* positions, int vertexNum, float adsorbStiffness,
	bool* vertIsActive,
	float* tetVertCollisionPos,
	unsigned char* tetVertIsCollide, int* collisionNumPtr,
	float* force, float* collisionDiag
) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;

	if (grab_half_flag[threadid] == 0) {
		bool isCollided = obbCollision(
			px, py, pz,
			x0, x1, x2,
			y0, y1, y2,
			z0, z1, z2,
			positions[indexX], positions[indexY], positions[indexZ],
			scaleX, scaleY, scaleZ);
		if (!isCollided) return;
		grab_half_flag[threadid] = grabFlag;
	}

	if (grab_half_flag[threadid] != grabFlag) return;
	

	// 计算投影点 - 投影到平面
	float vertexPosShift = (positions[indexX] - px) * y0
		+ (positions[indexY] - py) * y1
		+ (positions[indexZ] - pz) * y2;
	if (vertexPosShift < 0) vertexPosShift = 0;
	float relativePos[3];

	//记录碰撞点和工具的相对位置
	relativePos[0] = positions[indexX] - y0 * (vertexPosShift - 0.05) - px;
	relativePos[1] = positions[indexY] - y1 * (vertexPosShift - 0.05) - py;
	relativePos[2] = positions[indexZ] - y2 * (vertexPosShift - 0.05) - pz;

	//计算局部坐标
	float x = relativePos[0] * x0 + relativePos[1] * x1 + relativePos[2] * x2;
	float y = relativePos[0] * y0 + relativePos[1] * y1 + relativePos[2] * y2;
	float z = relativePos[0] * z0 + relativePos[1] * z1 + relativePos[2] * z2;

	//计算偏移向量
	float deltaPos[3];
	float deltax = x * x0 + y * y0 + z * z0;
	float deltay = x * x1 + y * y1 + z * z1;
	float deltaz = x * x2 + y * y2 + z * z2;

	// 力反馈端碰撞响应-改变力-碰撞检测有问题
	force[indexX] += adsorbStiffness * (deltax + px - positions[indexX]);
	force[indexY] += adsorbStiffness * (deltay + py - positions[indexY]);
	force[indexZ] += adsorbStiffness * (deltaz + pz - positions[indexZ]);
	collisionDiag[indexX] += adsorbStiffness;
	collisionDiag[indexY] += adsorbStiffness;
	collisionDiag[indexZ] += adsorbStiffness;

	return;
}

__global__ void hapticCollisionDetect_gripper(
	float scaleX, float scaleY, float scaleZ,
	float p0, float p1, float p2,
	float x0, float x1, float x2,
	float y0, float y1, float y2,
	float z0, float z1, float z2,
	float* positions, int vertexNum, bool* vertIsActive,
	unsigned int* isCollide, float* relativePosition,
	int* tetVert2SpringVertMapping, unsigned int* sv_isCollide
) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	unsigned int sv_threadid = tetVert2SpringVertMapping[threadid];
	isCollide[threadid] = sv_isCollide[sv_threadid] = 0; // 清空抓取标志位
	float collisionNormal[3];
	float collisionPos[3];
	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;

	//判断是否与upperGrapper碰撞
	bool collisionUp = obbCollision(
		p0, p1, p2,
		y0, y1, y2,
		-x0, -x1, -x2,
		-z0, -z1, -z2,
		positions[indexX], positions[indexY], positions[indexZ],
		scaleX, scaleY, scaleZ);
	if (collisionUp) {
		isCollide[threadid] = sv_isCollide[sv_threadid] = 1;
		float vertexPosShift =	(positions[indexX] - p0) * -x0 +
								(positions[indexY] - p1) * -x1 +
								(positions[indexZ] - p2) * -x2;
		vertexPosShift = abs(vertexPosShift);
		//记录碰撞点和工具的相对位置
		relativePosition[indexX] = positions[indexX] - p0 + x0 * (vertexPosShift - 0.05);
		relativePosition[indexY] = positions[indexY] - p1 + x1 * (vertexPosShift - 0.05);
		relativePosition[indexZ] = positions[indexZ] - p2 + x2 * (vertexPosShift - 0.05);
	}

	//判断是否与lowerGrapper碰撞
	bool collisionDown = obbCollision(
		p0, p1, p2, 
		y0, y1, y2,
		x0, x1, x2,
		-z0, -z1, -z2,
		positions[indexX], positions[indexY], positions[indexZ],
		scaleX, scaleY, scaleZ);
	if (collisionDown) {
		isCollide[threadid] = sv_isCollide[sv_threadid] = 1;
		float vertexPosShift = (positions[indexX] - p0) * -x0 +
			(positions[indexY] - p1) * -x1 +
			(positions[indexZ] - p2) * -x2;
		vertexPosShift = abs(vertexPosShift);
		relativePosition[indexX] = positions[indexX] - p0 - x0 * (vertexPosShift - 0.05);
		relativePosition[indexY] = positions[indexY] - p1 - x1 * (vertexPosShift - 0.05);
		relativePosition[indexZ] = positions[indexZ] - p2 - x2 * (vertexPosShift - 0.05);
	}

	//未碰撞直接退出
	if (isCollide[threadid] != 1) return;

	//计算局部坐标
	float x = relativePosition[indexX] * y0 + relativePosition[indexY] * y1 + relativePosition[indexZ] * y2;
	float y = relativePosition[indexX] * -x0 + relativePosition[indexY] * -x1 + relativePosition[indexZ] * -x2;
	float z = relativePosition[indexX] * z0 + relativePosition[indexY] * z1 + relativePosition[indexZ] * z2;
	//记录局部坐标
	relativePosition[indexX] = x;
	relativePosition[indexY] = y;
	relativePosition[indexZ] = z;
}

__global__ void hapticCollisionHandle_gripper(
	float scaleX, float scaleY, float scaleZ,
	float p0, float p1, float p2,
	float x0, float x1, float x2,
	float y0, float y1, float y2,
	float z0, float z1, float z2,
	float* positions, int vertexNum, float adsorbStiffness, bool* vertIsActive,
	unsigned int* isCollide, float* relativePosition,
	float* force, float* collisionDiag, float* adsorbForce
) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;

	//如果不是碰撞点就直接跳过
	if (isCollide[threadid] == 0) return;

	//是碰撞点就计算需要更新的位置，再加上attach约束
	float posx = positions[indexX];
	float posy = positions[indexY];
	float posz = positions[indexZ];
	float deltaPos[3];

	//计算偏移向量
	float deltax = relativePosition[indexX] * y0 + relativePosition[indexY] * -x0 + relativePosition[indexZ] * z0;
	float deltay = relativePosition[indexX] * y1 + relativePosition[indexY] * -x1 + relativePosition[indexZ] * z1;
	float deltaz = relativePosition[indexX] * y2 + relativePosition[indexY] * -x2 + relativePosition[indexZ] * z2;

	float targetPosx = deltax + p0;
	float targetPosy = deltay + p1;
	float targetPosz = deltaz + p2;


	deltaPos[0] = targetPosx - posx;
	deltaPos[1] = targetPosy - posy;
	deltaPos[2] = targetPosz - posz;

	//每次都会清零，累加可以
	force[indexX] += adsorbStiffness * deltaPos[0];
	force[indexY] += adsorbStiffness * deltaPos[1];
	force[indexZ] += adsorbStiffness * deltaPos[2];

	//会被清零，可以累加
	collisionDiag[indexX] += adsorbStiffness;
	collisionDiag[indexY] += adsorbStiffness;
	collisionDiag[indexZ] += adsorbStiffness;


	atomicAdd(adsorbForce + 0, deltaPos[0]);
    atomicAdd(adsorbForce + 1, deltaPos[1]);
    atomicAdd(adsorbForce + 2, deltaPos[2]);
}

void runHapticCollisionDetect_gripper(
	float scaleX, float scaleY, float scaleZ,
	float px, float py, float pz,
	float ux0, float ux1, float ux2,
	float uy0, float uy1, float uy2,
	float uz0, float uz1, float uz2,
	float ly0, float ly1, float ly2,
	float lz0, float lz1, float lz2,
	float px0, float px1, float px2,
	float py0, float py1, float py2,
	float pz0, float pz1, float pz2,
	float angle, bool* closeFlag, int gripper_no,
	float adsorbStiffness
) {
	// 用OBB包围盒进行碰撞检测
	int threadNum = 512;
	int blockNum = (tetVertNum_d + threadNum - 1) / threadNum;

	tv_grab_half_flag_t = &tv_grab_half_flag[gripper_no * tetVertNum_d]; // 夹取中间状态时用
	cudaMemset(tv_grab_half_flag_t, 0, sizeof(unsigned int) * tetVertNum_d);
	tv_grab_flag_t = &tv_grab_flag[gripper_no * tetVertNum_d]; // 夹取闭合状态时用，标记被夹取的顶点
	tv_relative_pos_t = &tv_relative_pos[gripper_no * tetVertNum_d * 3];
	sv_grab_flag_t = &sv_grab_flag[gripper_no * springVertNum_d];

	if (angle < gripper_min_angle) {
		// 1. 如果是第一帧，则重新计算夹取区域，置标志位tv_grab_flag_t以及记录局部坐标relativePosition
		if (!(*closeFlag)) { // 原来是张开的，现在第一次闭合
			*closeFlag = !*closeFlag;
			hapticCollisionDetect_gripper << <blockNum, threadNum >> > (
				scaleX, scaleY, scaleZ, px, py, pz, px0, px1, px2, py0, py1, py2, pz0, pz1, pz2, 
				tetVertPos_d, tetVertNum_d, tetVertActive_d,
				tv_grab_flag_t, tv_relative_pos_t,
				tetVert2SpringVertMapping_d, sv_grab_flag_t);
		}

		// 2. 根据夹取区域计算力
		hapticCollisionHandle_gripper << <blockNum, threadNum >> > (
			scaleX, scaleY, scaleZ, px, py, pz, px0, px1, px2, py0, py1, py2, pz0, pz1, pz2, 
			tetVertPos_d, tetVertNum_d, adsorbStiffness / 20, tetVertActive_d,
			tv_grab_flag_t, tv_relative_pos_t, 
			tetVertCollisionForce_d, tetVertCollisionDiag_d, &gripper_adsorb_force[gripper_no * 3]);
	}
	else if(angle < gripper_max_angle){
        cudaMemset(tv_grab_flag, 0, 1 * sizeof(unsigned int) * gripper_num * tetVertNum_d);
        cudaMemset(sv_grab_flag, 0, 1 * sizeof(unsigned int) * gripper_num * springVertNum_d);
		*closeFlag = false;
		hapticCollisionDetect_gripper_half << <blockNum, threadNum >> > (
			scaleX, scaleY, scaleZ, px, py, pz, ux0, ux1, ux2, uy0, uy1, uy2, uz0, uz1, uz2,
			tv_grab_half_flag_t, 1,
			tetVertPos_d, tetVertNum_d, adsorbStiffness / 50,
			tetVertActive_d,
			tetVertCollisionPos_d,
			tetVertisCollide_d, hapticCollisionNum_d,
            tetVertCollisionForce_d, tetVertCollisionDiag_d);
		hapticCollisionDetect_gripper_half << <blockNum, threadNum >> > (
			scaleX, scaleY, scaleZ, px, py, pz, ux0, ux1, ux2, ly0, ly1, ly2, lz0, lz1, lz2,
			tv_grab_half_flag_t, 2,
			tetVertPos_d, tetVertNum_d, adsorbStiffness / 50,
			tetVertActive_d,
			tetVertCollisionPos_d,
			tetVertisCollide_d, hapticCollisionNum_d,
            tetVertCollisionForce_d, tetVertCollisionDiag_d);
	}
	else {
        cudaMemset(tv_grab_flag, 0, 1 * sizeof(unsigned int) * gripper_num * tetVertNum_d);
        cudaMemset(sv_grab_flag, 0, 1 * sizeof(unsigned int) * gripper_num * springVertNum_d);
		*closeFlag = false;
	}
}

__global__ void getColNum(unsigned char* isCollide, int* colNum, int vertNum)
{
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertNum) return;

	if (isCollide[threadid] > 0)
		atomicAdd(colNum, 1);
}
