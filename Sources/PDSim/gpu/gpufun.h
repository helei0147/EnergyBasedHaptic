#pragma once
#include "cuda.h"
#include "cuda_runtime.h"  
#include "gpuvar.h"


#pragma region
//计算初始状态
int runcalculateST(float damping, float dt);
__global__ void resetNANIF(float* force,
	int tetVertNum, int id, bool* gp);

int runcalculateIF(float tetStiffness);
//计算更新位置
int runcalculatePOS(float omega, float dt);

//计算速度
int runcalculateV(float dt);
int runUpdateSurfaceDDir();
void printCudaError(const char* funcName);
void checkPointerLocation(void* ptr, const char* name="");

#pragma endregion

#pragma region 碰撞
void PropagateCollision();
#pragma endregion

#pragma region 力反馈

// 数学计算
__device__ void DeviceVec3MulVec3T(float* v0, float* v1, float* result);
__device__ void DeviceVec3toSkewSymmetricMatrix(float* v, float* m);
__device__ void DeviceMat3MulMat3(float* m0, float* m1, float* result);
__device__ void DeviceScaleMulMat3(float s, float* m, float* result);
__device__ void DeviceMat3AddMat3(float* m0, float* m1, float* result);
__device__ void DeviceMat3AtomicAddMat3(float* m, float* result);
__device__ void DeviceMatrixDotVec(float* m, float* v, float* result);
#pragma endregion

#pragma region

extern "C" int runcalculateSTMU(float damping, float dt);

extern "C" int runcalculateRestPosForceWithTetPos_vCAG(float minStiffnessTV2SV, float maxStiffnessTV2SV);
extern "C" int runcalculateVMU(float dt);
#pragma endregion



#pragma region

__global__ void calculateRestStiffnessSV2TV(unsigned char* tetVertCollideFlag, float* tetVertToolDistance, float* restStiffness,
	float minStiffnessSV2TV, float maxStiffnessSV2TV,
	bool* tetVertActive, bool* tetVertIsBurned, int tetVertNum);

//计算st
__global__ void calculateST(float* positions, float* velocity, float* externForce,
	float* old_positions, float* prev_positions, float* last_Positions, float* fixed, bool* vertIsActive, float gravityX, float gravityY, float gravityZ,
	int vertexNum, float damping, float dt);

//计算F,R
__global__ void calculateIF(float* positions, int* tetIndex,
	float* tetInvD3x3, float* tetInvD3x4,
	float* force, float* tetVolumn, bool* active,
	int tetNum, float volumnStiffness);

//计算抓取力
__global__ void calculateAdsorbForce(float gripper_scale_x, float gripper_scale_y, float gripper_scale_z, float* cylinderPos, float* cylinderDirX, float* cylinderDirY, float* cylinderDirZ, 
	float* positions, unsigned int* isCollide, float* force, float* collisionDiag, float* relativePosition, unsigned char* tetVertisCollide,
	int vertexNum, float adsorbStiffness, bool* vertIsActive);

//计算需要被夹取的区域的粒子
__global__ void calculateGrabRegion(float* cylinderPos, float* cylinderDirZ, float* cylinderDirY, float* cylinderDirX,
	float grappleX, float grappleY, float grappleZ, float* positions, unsigned int* isCollide, int* grabNum,
	int vertexNum, float* relativePosition, bool* vertIsActive);

//计算夹取力2.0
__global__ void calculateGrabForce(float gripper_scale_x, float gripper_scale_y, float gripper_scale_z, float* grapperPos, float* grapperDirZ, float* grapperDirY, float* grapperDirX,
	float* positions, unsigned int* isCollide, int vertexNum, unsigned char* tetVertisCollide, 
	float adsorbStiffness, float* force, float* collisionDiag, unsigned int grabFlag, bool* vertIsActive);

//计算顶点到胶囊体的距离
__device__ float calculateCylinderDis(float posx, float posy, float posz, float dirx, float diry, float dirz,
	float vertx, float verty, float vertz, float length);

//与自定义的obb包围盒进行碰撞检测（模拟抓钳抓取的范围）
__device__ bool obbCollision(float posx, float posy, float posz, float dirXx, float dirXy, float dirXz, float dirYx, float dirYy, float dirYz, float dirZx, float dirZy, float dirZz,
	float vertx, float verty, float vertz, float width, float length, float height);
//计算position


__global__ void calculatePOS(float* positions, float* force, float* fixed, float* mass,
	float* next_positions, float* prev_positions, float* old_positions,
	float* volumnDiag, float* collisionDiag, float* collisionForce, bool* vertActive,
	int vertexNum, float dt, float omega);

//计算速度更新

__global__ void calculateV(float* positions, float* velocity, float* last_positions, bool* vertActive, int vertexNum, float dt);

__device__ void MatrixProduct_3_D(const float* A, const float* B, float* R);

__device__ void MatrixProduct_D(float* A, float* B, float* R, int nx, int ny, int nz);

__device__ void MatrixSubstract_3_D(float* A, float* B, float* R);

__device__ void GetRotation_D(float F[3][3], float R[3][3]);

__device__ float tetDot_D(float* a, float* b);

__device__ void tetCross_D(float* a, float* b, float* c);

__device__ float tetNormal_D(float* vec0);
#pragma endregion


#pragma region  
//计算顶点的初速度

__global__ void calculateSTMU(float* positions, float* old_positions, float* prev_positions, float* velocity, float* externForce, float* fixed,
	bool* vertIsActive, float gravityX, float gravityY, float gravityZ,
	int vertexNum, float damping, float dt);

extern "C" int runcalculateIFMU(float springStiffness);

int runCollisionResponse(float collisionStiffness);

__global__ void calculateIFMU(float* positions, float* force, float* forceDiag, float springStiffness, float* springOrigin, unsigned int* springIndex, bool* springActive, int springNum);

//计算Rest-pos力
__global__ void calculateRestPosWithTetPosMU(float* positions, int* skeletonIndex, float* force, float* collisionDiag, float* rest_positions, float* restStiffness, int vertexNum, bool* vertIsActive);
__global__ void calculateRestStiffnessTV2SV(unsigned char* springVertCollideFlag, float* springVertToolDistance, float* restStiffness,
	float minStiffnessTV2SV, float maxStiffnessTV2SV,
	bool* springVertIsActive, int springVertexNum);
//切比雪夫更新位置
extern "C" int runcalculatePosMU(float omega, float dt);
__global__ void calculatePOSMU(float* positions, float* force, float* fixed, float* mass, float* next_positions, float* prev_positions,
	float* old_positions, float* springDiag, float* collisionDiag, float* collisionForce, bool* springVertActive,
	int vertexNum, float dt, float omega);

//更新速度
__global__ void calculateVMU(float* positions, float* velocity, float* old_positions,bool* vertActive, int vertexNum, float dt);
/**************************************************辅助函数*************************************************/
__device__ void tetCrossMU_D(float* a, float* b, float* c);

__device__ float tetDotMU_D(float* a, float* b);
#pragma endregion

__global__ void resetVertToolDistance(float* springVertToolDistance, int springVertNum);
int runResetSpringVertToolDistance();
int runResetTetVertToolDistance();

//计算前缀和
__global__ void hapticCalculatePrefixSum(
	unsigned char* isCollide, unsigned int* queueIndex,
	unsigned int* auxArray, int vertexNum);
__global__ void hapticAddCollisionToQueueTri(
	unsigned char* isCollide, float* triPositions, float* triVertDepth, float* triVertCollisionPos, int* triVertCollisionToolFlag,
	float* constraintPoints, float* constraintDepth, float* tetVertCollidedPos, int* triVertCollidedToolFlag,
	unsigned int* queueIndex, unsigned int* auxArray, int vertexNum);
__global__ void hapticAddCollisionToQueue(
	unsigned char* isCollide, float* tetPositions, float* triVertNonPenetrationDir, float* triVertDepth, float* tetVertCollisionPos, int* tetVertCollisionToolFlag,
	float* constraintPoints, float* constriantNonPenetrationDir, float* constraintDepth, float* tetVertCollidedPos, int* tetVertCollidedToolFlag,
	unsigned int* queueIndex, unsigned int* auxArray, int vertexNum);

 void runHapticCDSV_cylinder(float cylinderPosX, float cylinderPosY, float cylinderPosZ, float cylinderLastPosX, float cylinderLastPosY,
	 float cylinderLastPosZ, float cylinderDirX, float cylinderDirY, float cylinderDirZ, float cylinderRadius, float cylinderLength,
	 int toolFlag);

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
);

void runApplyCollision(float tetVertPosX, float tetVertPosY, float tetVertPosZ,
	float tetVertCollidedPosX, float tetVertCollidedPosY, float tetVertCollidedPosZ,
	float tv_colStiffness, float tv_colSpreadRadius, float tv_maxColVelocity);

void runMergeCollisionInfoTet();
void runMergeCollisionInfoTriVert();
