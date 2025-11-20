#include "gpuvar.h"
#include "gpufun.h"

//中转
__device__ float* cylinderShiftMU;
__device__ float* cylinderLastPosMU;
__device__ float* cylinderPosMU;
__device__ float* cylinderGraphicalPosMU;
__device__ float* cylinderDirZMU;
__device__ float* cylinderDirYMU;
__device__ float* cylinderDirXMU;
__device__ float* cylinderVMU;
__device__ float* relativePositionMU;
__device__ unsigned int* isGrapMU;
__device__ unsigned int* isGrapHalfMU;
__device__ float* adsorbForceMU;
__device__ float* grapperUpDirXMU;
__device__ float* grapperUpDirYMU;
__device__ float* grapperUpDirZMU;
__device__ float* grapperDownDirXMU;
__device__ float* grapperDownDirYMU;
__device__ float* grapperDownDirZMU;
__device__ unsigned int* collideFlag;

__device__ void tetCrossMU_D(float* a, float* b, float* c) {
	//叉乘计算三角形法线
	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = a[2] * b[0] - b[2] * a[0];
	c[2] = a[0] * b[1] - b[0] * a[1];
}

__device__ float tetDotMU_D(float* a, float* b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

__global__ void calculateAdsorbForce(float gripper_scale_x, float gripper_scale_y, float gripper_scale_z, float* cylinderPos, float* cylinderDirX, float* cylinderDirY, float* cylinderDirZ, float* positions, unsigned int* isCollide, float* force, float* collisionDiag, float* relativePosition, unsigned char* tetVertisCollide, int vertexNum, float adsorbStiffness,
	bool* vertIsActive) {
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
	float deltax = relativePosition[indexX] * cylinderDirY[0] + relativePosition[indexY] * -cylinderDirX[0] + relativePosition[indexZ] * cylinderDirZ[0];
	float deltay = relativePosition[indexX] * cylinderDirY[1] + relativePosition[indexY] * -cylinderDirX[1] + relativePosition[indexZ] * cylinderDirZ[1];
	float deltaz = relativePosition[indexX] * cylinderDirY[2] + relativePosition[indexY] * -cylinderDirX[2] + relativePosition[indexZ] * cylinderDirZ[2];

	float targetPosx = deltax + cylinderPos[0];
	float targetPosy = deltay + cylinderPos[1];
	float targetPosz = deltaz + cylinderPos[2];

	float distance = calculateCylinderDis(cylinderPos[0], cylinderPos[1], cylinderPos[2], -cylinderDirZ[0], -cylinderDirZ[1], -cylinderDirZ[2], targetPosx, targetPosy, targetPosz, 1.5 * gripper_scale_z);
	distance -= gripper_scale_y * 2; // 比例系数待定
	float k;
	k = 1 / (1 + exp(12 * distance - 5));
	adsorbStiffness = k * adsorbStiffness;

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

	tetVertisCollide[threadid] = COLLIDE_TYPE::FULL_GRAB;
}

//计算顶点到胶囊体的距离
__device__ float calculateCylinderDis(float posx, float posy, float posz, float dirx, float diry, float dirz, float vertx, float verty, float vertz, float length) {
	float pos1x = posx + dirx * length;
	float pos1y = posy + diry * length;
	float pos1z = posz + dirz * length;
	float posdx = pos1x - posx;
	float posdy = pos1y - posy;
	float posdz = pos1z - posz;

	float dx = vertx - posx;
	float dy = verty - posy;
	float dz = vertz - posz;

	float t = dirx * dx + diry * dy + dirz * dz;
	t /= length;
	if (t < 0) {
		t = 0;
	}
	else if (t > 1) {
		t = 1;
	}

	dx = vertx - posx - t * posdx;
	dy = verty - posy - t * posdy;
	dz = vertz - posz - t * posdz;
	float distance = sqrt(dx * dx + dy * dy + dz * dz);
	return distance;
}

//修改旋转轴，绕y轴闭合
__global__ void calculateGrabRegion(float* cylinderPos, float* cylinderDirZ, float* cylinderDirY, float* cylinderDirX,
	float grappleX, float grappleY, float grappleZ, float* positions, unsigned int* isCollide, int* grabNum,
	int vertexNum, float* relativePosition, bool* vertIsActive) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	isCollide[threadid] = 0;//清空碰撞标志位，即isGrap
	float collisionNormal[3];
	float collisionPos[3];
	float t = 0.0;
	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;
	int sv_index_x = threadid * 3 + 0;
	int sv_index_y = threadid * 3 + 1;
	int sv_index_z = threadid * 3 + 2;

	//判断是否与upperGrapper碰撞
	bool collisionUp = obbCollision(cylinderPos[0], cylinderPos[1], cylinderPos[2], 
		cylinderDirY[0], cylinderDirY[1], cylinderDirY[2], 
		-cylinderDirX[0], -cylinderDirX[1], -cylinderDirX[2], 
		-cylinderDirZ[0], -cylinderDirZ[1], -cylinderDirZ[2], 
		positions[sv_index_x], positions[sv_index_y], positions[sv_index_z],
		1.5 * grappleX, grappleY, grappleZ);
	if (collisionUp) {
		//设置标志位
		isCollide[threadid] = 1;
		//计算顶点的偏移值
		float vertexPosShift =	(positions[sv_index_x] - cylinderPos[0]) * -cylinderDirX[0] +
								(positions[sv_index_y] - cylinderPos[1]) * -cylinderDirX[1] +
								(positions[sv_index_z] - cylinderPos[2]) * -cylinderDirX[2];
		vertexPosShift = abs(vertexPosShift);
		//记录碰撞点和工具的相对位置
		relativePosition[indexX] = positions[sv_index_x] - -cylinderDirX[0] * (vertexPosShift - 0.05) - cylinderPos[0];
		relativePosition[indexY] = positions[sv_index_y] - -cylinderDirX[1] * (vertexPosShift - 0.05) - cylinderPos[1];
		relativePosition[indexZ] = positions[sv_index_z] - -cylinderDirX[2] * (vertexPosShift - 0.05) - cylinderPos[2];
	}

	//判断是否与lowerGrapper碰撞
	bool collisionDown = obbCollision(cylinderPos[0], cylinderPos[1], cylinderPos[2], 
		cylinderDirY[0], cylinderDirY[1], cylinderDirY[2], 
		cylinderDirX[0], cylinderDirX[1], cylinderDirX[2], 
		-cylinderDirZ[0], -cylinderDirZ[1], -cylinderDirZ[2], 
		positions[sv_index_x], positions[sv_index_y], positions[sv_index_z],
		1.5 * grappleX, grappleY, grappleZ);
	if (collisionDown) {
		isCollide[threadid] = 1;
		float vertexPosShift =	(positions[sv_index_x] - cylinderPos[0]) * -cylinderDirX[0] +
								(positions[sv_index_y] - cylinderPos[1]) * -cylinderDirX[1] +
								(positions[sv_index_z] - cylinderPos[2]) * -cylinderDirX[2];
		vertexPosShift = abs(vertexPosShift);
		relativePosition[indexX] = positions[sv_index_x] + -cylinderDirX[0] * (vertexPosShift - 0.05) - cylinderPos[0];
		relativePosition[indexY] = positions[sv_index_y] + -cylinderDirX[1] * (vertexPosShift - 0.05) - cylinderPos[1];
		relativePosition[indexZ] = positions[sv_index_z] + -cylinderDirX[2] * (vertexPosShift - 0.05) - cylinderPos[2];
	}

	//未碰撞直接退出
	if (isCollide[threadid] != 1) return;

	//计算局部坐标
	float x = relativePosition[indexX] * cylinderDirY[0] + relativePosition[indexY] * cylinderDirY[1] + relativePosition[indexZ] * cylinderDirY[2];
	float y = relativePosition[indexX] * -cylinderDirX[0] + relativePosition[indexY] * -cylinderDirX[1] + relativePosition[indexZ] * -cylinderDirX[2];
	float z = relativePosition[indexX] * cylinderDirZ[0] + relativePosition[indexY] * cylinderDirZ[1] + relativePosition[indexZ] * cylinderDirZ[2];
	//记录局部坐标
	relativePosition[indexX] = x;
	relativePosition[indexY] = y;
	relativePosition[indexZ] = z;
	//atomicAdd(grabNum, 1);
	grabNum[0]++; // 不用非常精确
}

__global__ void calculateGrabForce(float gripper_scale_x, float gripper_scale_y, float gripper_scale_z, float* grapperPos, float* grapperDirZ, float* grapperDirY, float* grapperDirX, float* positions, unsigned int* isCollide, int vertexNum, unsigned char* tetVertisCollide, float adsorbStiffness, float* force, float* collisionDiag, unsigned int grabFlag, bool* vertIsActive) {
	unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadid >= vertexNum) return;
	if (!vertIsActive[threadid]) return;

	float relativePos[3];
	int indexX = threadid * 3 + 0;
	int indexY = threadid * 3 + 1;
	int indexZ = threadid * 3 + 2;

	if (isCollide[threadid] == 0) {
		bool collisionFlag = obbCollision(grapperPos[0], grapperPos[1], grapperPos[2], grapperDirX[0], grapperDirX[1], grapperDirX[2], grapperDirY[0], grapperDirY[1], grapperDirY[2], grapperDirZ[0], grapperDirZ[1], grapperDirZ[2], positions[indexX], positions[indexY], positions[indexZ], gripper_scale_x * 1.5, gripper_scale_y * 0.5, gripper_scale_z);
		if (!collisionFlag) return;
		//设置标志位--和哪个抓钳碰撞
		isCollide[threadid] = grabFlag;
	}

	if (isCollide[threadid] != grabFlag) return;
	//计算顶点的偏移值
	float vertexPosShift = (positions[indexX] - grapperPos[0]) * grapperDirY[0] + (positions[indexY] - grapperPos[1]) * grapperDirY[1] + (positions[indexZ] - grapperPos[2]) * grapperDirY[2];
	//vertexPosShift = abs(vertexPosShift);
	if (vertexPosShift < 0) vertexPosShift = 0;
	//记录碰撞点和工具的相对位置
	relativePos[0] = positions[indexX] - grapperDirY[0] * (vertexPosShift - 0.05) - grapperPos[0];
	relativePos[1] = positions[indexY] - grapperDirY[1] * (vertexPosShift - 0.05) - grapperPos[1];
	relativePos[2] = positions[indexZ] - grapperDirY[2] * (vertexPosShift - 0.05) - grapperPos[2];

	//计算局部坐标
	float x = relativePos[0] * grapperDirX[0] + relativePos[1] * grapperDirX[1] + relativePos[2] * grapperDirX[2];
	float y = relativePos[0] * grapperDirY[0] + relativePos[1] * grapperDirY[1] + relativePos[2] * grapperDirY[2];
	float z = relativePos[0] * grapperDirZ[0] + relativePos[1] * grapperDirZ[1] + relativePos[2] * grapperDirZ[2];

	float deltaPos[3];

	//计算偏移向量
	float deltax = x * grapperDirX[0] + y * grapperDirY[0] + z * grapperDirZ[0];
	float deltay = x * grapperDirX[1] + y * grapperDirY[1] + z * grapperDirZ[1];
	float deltaz = x * grapperDirX[2] + y * grapperDirY[2] + z * grapperDirZ[2];

	float targetPosx = deltax + grapperPos[0];
	float targetPosy = deltay + grapperPos[1];
	float targetPosz = deltaz + grapperPos[2];

	float distance = calculateCylinderDis(grapperPos[0], grapperPos[1], grapperPos[2], grapperDirZ[0], grapperDirZ[1], grapperDirZ[2], targetPosx, targetPosy, targetPosz, 1.5 * gripper_scale_z);
	float k;
	distance -= gripper_scale_y * 2;
	k = 1 / (1 + exp(12 * distance - 5));
	adsorbStiffness = k * adsorbStiffness;

	deltaPos[0] = targetPosx - positions[indexX];
	deltaPos[1] = targetPosy - positions[indexY];
	deltaPos[2] = targetPosz - positions[indexZ];

	//每次都会清零，可以累加
	force[indexX] += adsorbStiffness * deltaPos[0];
	force[indexY] += adsorbStiffness * deltaPos[1];
	force[indexZ] += adsorbStiffness * deltaPos[2];

	collisionDiag[indexX] += adsorbStiffness;
	collisionDiag[indexY] += adsorbStiffness;
	collisionDiag[indexZ] += adsorbStiffness;

	tetVertisCollide[threadid] = COLLIDE_TYPE::HALF_GRAB;
}


//与自定义的obb包围盒进行碰撞检测（模拟抓钳抓取的范围）
__device__ bool obbCollision(float posx, float posy, float posz, float dirXx, float dirXy, float dirXz, float dirYx, float dirYy, float dirYz, float dirZx, float dirZy, float dirZz, float vertx, float verty, float vertz, float width, float length, float height) {
	float x = (vertx - posx) * dirXx + (verty - posy) * dirXy + (vertz - posz) * dirXz;
	float y = (vertx - posx) * dirYx + (verty - posy) * dirYy + (vertz - posz) * dirYz;
	float z = (vertx - posx) * dirZx + (verty - posy) * dirZy + (vertz - posz) * dirZz;

	if (z<0 || z>height) return false;
	if (y<0 || y>length) return false;
	if (x<-width || x>width) return false;

	return true;
}

__device__ void tetCross_D(float* a, float* b, float* c) {
	//叉乘计算三角形法线
	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = a[2] * b[0] - b[2] * a[0];
	c[2] = a[0] * b[1] - b[0] * a[1];
}

__device__ float tetDot_D(float* a, float* b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

__device__ float tetNormal_D(float* vec0) {
	float length = vec0[0] * vec0[0] + vec0[1] * vec0[1] + vec0[2] * vec0[2];
	length = sqrt(length);
    if (length < 1e-5f) {
        //printf("vec0 == 0!!!!!");
        return 0.0;
	}
	vec0[0] /= length;
	vec0[1] /= length;
	vec0[2] /= length;
	return length;
}

