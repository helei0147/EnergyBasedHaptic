#pragma once
#include <string>
#include <vector>
#include "gpuvar.h"

extern float gripper_max_angle;
extern float gripper_min_angle;

#define POINT_MERGE_EPS (0.008)

int GetOperatorCnt();
int GetOperatorToolId(int index);
int GetOperatorToolCnt(int index);
void SetOperatorTool(int index, int type);
void UDLog(std::string mess);
void UDError(std::string mess);
void UDWarning(std::string mess);


void GetRegion(float* region);
float GetSimTime();
float GetFrameTime();
float GetSTTime();
float GetHapticTime();
int GetTetVertNum();
int GetTetNum();
int GetSpringVertNum();
float GetHapticFrameTime();

// 从collisionbuffer中获取
int GetCollidedNum();

void SetGravityX(float x);
void SetGravityY(float y);
void SetGravityZ(float z);

float GetGravityX();
float GetGravityY();
float GetGravityZ();

float Getsv_damping();
void Setsv_damping(float v);
float Getsv_dt();
void Setsv_dt(float v);
float Getsv_springStiffness();
void Setsv_springStiffness(float v);
float Getsv_collisionStiffness();
void Setsv_collisionStiffness(float v);
float Getsv_adsorbStiffness();
void Setsv_adsorbStiffness(float v);
float Getsv_maxStiffnessTV2SV();
void Setsv_maxStiffnessTV2SV(float v);
float Getsv_minStiffnessTV2SV();
void Setsv_minStiffnessTV2SV(float v);
float Gettv_damping();
void Settv_damping(float v);
float Gettv_dt();
void Settv_dt(float v);
float Gettv_iterateNum();
void Settv_iterateNum(float v);
float Gettv_volumnStiffness();
void Settv_volumnStiffness(float v);
float Gettv_collisionStiffness();
void Settv_collisionStiffness(float v);
float Gettv_colSpreadRadius();
void Settv_colSpreadRadius(float v);
float Gettv_maxColVelocity();
void Settv_maxColVelocity(float v);
float Getk_vc();
void Setk_vc(float v);
float Getk_c();
void Setk_c(float v);
float Getk_vct();
void Setk_vct(float v);
bool Getsv_transmit_grab();
void Setsv_transmit_grab(bool v);
float Getsv_gripper_grab_smoothing();
void Setsv_gripper_grab_smoothing(float v);
bool Getsv_transmit_adsorb();
void Setsv_transmit_adsorb(bool v);
float Getsv_gripper_adsorb_smoothing();
void Setsv_gripper_adsorb_smoothing(float v);
float Getsv_conColThreshold();
void Setsv_conColThreshold(float v);
float Gettv_maxStiffnessSV2TV();
void Settv_maxStiffnessSV2TV(float v);
float Gettv_minStiffnessSV2TV();
void Settv_minStiffnessSV2TV(float v);



inline float LengthSq(const float* p1, const const float* p2) {
	float x = p1[0] - p2[0];
	float y = p1[1] - p2[1];
	float z = p1[2] - p2[2];
	return x * x + y * y + z * z;
}

// 传入对应索引的操作手的位姿数据，更新 solver 中对应的位姿
void UpdateQH(int operatorIndex, float* buffer);

void ComputeOperatorForce(int operatorIndex, float* f);

int GetCollidedNum(int index);
