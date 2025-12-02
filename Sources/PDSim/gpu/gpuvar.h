#pragma once
#include <chrono> // 用于高精度计算函数运行时间
#include "cuda.h"
#include "cuda_runtime.h"  

#define SV_STRIDE 3
#define HASH_BUCKET_NUM 10

extern int g_ID_SimRender;
extern int g_ID_SoftHaptic;
extern bool g_asyncSimMode;
extern int g_nv_deviceNum;

enum COLLIDE_TYPE {
	NO_COL = 0,
	NORMAL = 1,
	HALF_GRAB = 2,
	FULL_GRAB = 3
};

#pragma region
extern	float             gravityX_d;//重力
extern	float				gravityY_d;//重力
extern	float				gravityZ_d;//重力
//顶点数和四面体数
extern	int		tetVertNum_d;
extern	int		tetNum_d;
extern  float* tetVertPos_d;
extern  float* tetVertRestPos_d;
extern  float* tetVertPos_last_d;
extern  float* tetVertPos_old_d;
extern  float* tetVertPos_prev_d;
extern  float* tetVertPos_next_d;
extern  float* tetVertVelocity_d;
extern  float* tetVertExternForce_d;
extern  float* tetVertMass_d;
extern  int* tetIndex_d;
extern int* mapTetVertIdx2TriVertIdx_d;

extern  float* tetVertCollisionDiag_d;
extern  float* tetVertFixed_d;
extern  bool* tetActive_d;
extern  float* tetInvD3x3_d;//预处理
extern  float* tetInvD3x4_d;//预处理
extern  float* tetVolume_d;//预处理
extern float* tetVolumeDiag_d;
extern  float* tetVertForce_d;
extern	float* tetStiffness_d;
extern int* tetVert2SpringVertMapping_d;
extern float* tetVertPos_forTri;

extern bool* tetVertActive_d;
extern float* tetVertNorm_d; // 外表面三角网格顶点的法向量，对应数组中的部分元素，tetVertNum_d*3
extern float* tetVertNormAccu_d; // tetVertNum_d
//碰撞约束力
extern float* tetVertCollisionForce_d;
extern float* tetVertToolDistance_d;

extern unsigned char* springVertisCollide_forTet;
extern float* springVertPos_forTet;
extern float* springVertCollisionDepth_forTet;
extern float* springVertCollisionPos_forTet;
extern int* springVertCollisionToolFlag_forTet;
extern int* springVert2TetVertMapping_forTet;

#pragma region 

extern	int		springVertNum_d;
extern	float* springVertPos_d;
extern	float* springVertPos_old_d;
extern	float* springVertPos_prev_d;
extern	float* springVertPos_next_d;
extern	float* springVertVelocity_d;
extern	float* springVertExternForce_d;
extern	float* springVertMass_d;
extern	float* springVertFixed_d;
extern	float* springVertForce_d;
extern float* springVertInsertionDepth_d;
extern float* springVertProjectedPos_d;
extern float* springVertToolDistance_d;
extern float* springVertNonPenetrationDir_d;

//spring
extern	int		springNum_d;
extern	unsigned int* springIndex_d;
extern	float* springOrgLength_d;
extern	float* springDiag_d;
extern	float* springVertCollisionDiag_d;
extern  int* springVert2TetVertMapping_d;
extern  float* springVertfromTetStiffness_d;
extern	float* springStiffness_d;
extern float* springVertNorms_d;

extern bool* springActive_d;
extern bool* springVertActive_d;
extern float* springVertCollisionPos_d; // 表面三角形顶点位置
extern float* springVertCollisionNormal_d;// 表面三角形顶点被碰撞后的投影位置
extern float* springVertCollisionDepth_d;//// 表面三角形顶点被碰撞之后的
extern unsigned int* springVertCollisionCnt_d;//// 表面三角形顶点碰撞数量？
extern int* springVertCollisionToolFlag_d;// 表面三角形被碰撞的工具编号。

extern int triNum_d;
extern int* triIndex_d;


extern	unsigned char* springVertisCollide_d;  //是否发生碰撞
extern unsigned int* springVertQueueIndex_d;
extern unsigned int* springVertAuxSumArray_d;
extern float* springVertCollidedBuffer_d;
extern float* springVertCollidedNonPenetration_d;
extern float* springVertCollidedDepth_d;
extern float* springVertCollidedPos_d; // 表面顶点碰撞后的投影位置
extern int* springVertCollidedToolFlag_d;// 表面三角形被碰撞的工具编号。

//碰撞约束力
extern	float* springVertCollisionForce_d;

#pragma endregion


#pragma region 碰撞相关
extern unsigned char* tetisCollide_d;
extern unsigned char* tetVertisCollide_d;
extern int* tetVertCollisionCnt_d;
extern int* tetVertCollisionToolFlag_d; // 顶点是和左工具还是右工具碰撞的
extern float* tetVertCollisionPos_d;
extern float* tetVertCollisionDepth_d;
extern float* tetVertCollidedBuffer_d;
extern float* tetVertCollidedNonPenetration_d;
extern float* tetVertCollidedDepth_d;
extern float* tetVertCollidedPos_d;
extern int* tetVertCollidedToolFlag_d;
extern unsigned int* tetVertQueueIndex_d;
extern unsigned int* tetVertAuxSumArray_d;
extern float* tetVertColVelocityMax_d;
#pragma endregion 

#pragma region 碰撞体参数
extern int cylinder_num;
extern char* cylinder_active;
extern float* cylinder_pos;
extern float* cylinder_dir;
extern float* cylinder_radius;
extern float* cylinder_length;
extern float* cylinder_shift_tv;
extern float* cylinder_shift_sv;
extern float* cylinder_last_pos;
extern unsigned char* cylinder_collide_flag;
extern int sphere_num;
extern char* sphere_active;
extern float* sphere_pos;
extern float* sphere_radius;
extern float* sphere_shift;
extern float* sphere_last_pos;
extern float* cylinder_pos_haptic;
extern float* cylinder_dir_haptic;
extern float* cylinder_last_pos_haptic;
#pragma endregion

#pragma region 夹取参数
extern int gripper_num;
extern char* gripper_active;
extern float* gripper_pos;
extern float* gripper_scale;
extern float* gripper_pivot_x;
extern float* gripper_pivot_y;
extern float* gripper_pivot_z;
extern float* gripper_upper_x;
extern float* gripper_upper_y;
extern float* gripper_upper_z;
extern float* gripper_lower_x;
extern float* gripper_lower_y;
extern float* gripper_lower_z;
extern float* gripper_angle;
extern bool* gripper_unclose_tv;
extern unsigned int* tv_grab_flag;
extern unsigned int* tv_grab_half_flag;
extern float* tv_relative_pos;
extern unsigned int* sv_grab_flag;
extern float* gripper_adsorb_force;
extern int* gripper_grab_num;
#pragma endregion

#pragma region 力反馈参数
extern int* hapticCollisionNum_d;
extern int* tvCollisionNum_d;
#pragma endregion
