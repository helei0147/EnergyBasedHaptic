#pragma once
#include <utility>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <deque>
#include <set>
#include <algorithm>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/src/Geometry/Quaternion.h>
#include <mutex>
#include "common.h"
#include "CylinderCollider.h"
#include "SphereCollider.h"
#include "GripperCollider.h"
#include "calculate.h"


class Solver
{
public:
	float sv_damping = 0.5f;
	float sv_dt = 0.333f;
	float sv_springStiffness = 500.0f;
	float sv_collisionStiffness = 2000.0f;
	float sv_adsorbStiffness = 10000.0f;
	float sv_maxStiffnessTV2SV = 500.0f;
	float sv_minStiffnessTV2SV = 0.0f;
	bool sv_transmit_grab = true;
	bool sv_transmit_adsorb = true;
	float sv_conColThreshold = 0.5f;
	float sv_gripper_grab_smoothing = 256.0f;
	float sv_gripper_adsorb_smoothing = 512.0f;
	int sv_collisionNum = 0;
	int tv_collisionNum = 0;

	float tv_damping = 0.5f;
	float tv_dt = 10.0f / 30.0f;
	float tv_iterateNum = 16;
	float tv_volumnStiffness = 300.0f;
	float tv_collisionStiffness = 30.0f;
	float tv_minStiffnessSV2TV = 150.0f;
	float tv_maxStiffnessSV2TV = 500.0f;


	//迭代次数,等参数
	int m_iterateNum=12;

	float volumnStiffness=2000.0f;
	float adsorbStiffness=10000.0f;

	float rho=0.9992f;
	float m_damping=0.0f;
	float m_dampingForTriVert = 0.1f;
	float m_gravityX = 0;
	float m_gravityY = 0;
	float m_gravityZ = 0;
	float m_volumnSum = 0;

	double m_opTime;
	double m_frameTime;
	double m_hapFrameTime;
	double m_stTime;

	// 软体中的固定球心和半径
	std::vector<float> m_fixCenter;
	std::vector<float> m_fixRadius;


#pragma region 外界提供给求解器的数据
	///四面体
	std::vector<int> m_tetIndex;//四面体索引	
	std::vector<float> m_tetStiffness;//四面体的软硬
	std::vector<float> m_tetVertPos;//顶点位置	
	std::vector<float> m_tetVertFixed;//是否固定	0表示固定，大于0表示不固定	
	std::vector<float> m_tetVertfromSpringStiffness;//三角形顶点对四面体的约束刚度
	std::vector<int> m_onSurfaceSpringVertIndices; // 在表面的弹簧顶点下标
	std::vector<int> m_mapSurfaceSpringVert2OutsideFaceTri; // 在表面的弹簧顶点映射到外表面三角形顶点
#pragma endregion

	
#pragma region 四面体约束
	std::vector<float> m_tetVertMass;//顶点质量
	//四面体是否处于有效状态
	std::vector<char> m_tetActive;
	//四面体矩阵的逆//用于计算变形梯度
	std::vector<float> m_tetInvD3x3;
	//用于计算对角阵
	std::vector<float> m_tetInvD3x4;
	//四面体体积，长度：tetNum
	std::vector<float> m_tetVolume;
	//线性系统的对角矩阵的ACT*AC部分, 长度：tetNum
	std::vector<float> m_tetVolumeDiag;

	//四面体顶点是否处于有效状态，长度 : tetVertNum
	std::vector<char> m_tetVertActive;
	std::vector<int> m_tet2triMapping;
	std::vector<int> m_tri2tetMapping;
	std::vector<int> m_subedtri2tetMapping;
	
#pragma region 
	std::vector<int> m_subedTriIdx;
	std::vector<float> m_subedTriPos;
	std::vector<int> m_originTriIdx;
	std::vector<int> m_mapSubedTriVert2OriginTriVert;
#pragma endregion

#pragma region 表面弹簧约束
	std::vector<unsigned int> m_springIndex;// 弹簧索引
	std::vector<float> m_springStiffness;// 弹簧刚度
	std::vector<float> m_springVertPos;//弹簧顶点
	std::vector<float> m_springVertFixed;
	std::vector<float> m_springVertMass;
	std::vector<float> m_springOrgLength; // 弹簧原长
	std::vector<float> m_springDiag;

	std::vector<char> m_springActive;
	std::vector<char> m_springVertActive;

	// 用于两个GPU之间传递数据的cpu存储
	std::vector<unsigned char> springVertisCollide_bridge;
	std::vector<float> springVertPos_bridge;
	std::vector<float> springVertCollisionDepth_bridge;
	std::vector<float> springVertCollisionPos_bridge;
	std::vector<int> springVertCollisionToolFlag_bridge;
	std::vector<int> springVert2TetVertMapping_bridge;
	std::vector <float> tetVertPos_bridge;
	std::vector<float> graspForce;
#pragma endregion
	 ~Solver();
	void CopyToGPU();
	void SolverInit();
	void PreMalloc();
	void Step();
	void HandleCollision();
	void ApplyGravity();

	int GetTetVertNum(void);
	int GetTetNum(void);
	int GetSpringVertNum(void);
	int GetSpringNum(void);

	void InitSpring();
	void InitSpringConstraint();
	void InitVolumeConstraint();
	void MergeVertex();
	void CalculateTetTriMapping();
	void TriSubdivision();
	void CalculateTetSubedtriMapping();
	void HapticSyn();

	void* m_manager = nullptr;
	
	void FixSoft();

	ofstream m_frameCntFile;


#pragma region 图形位姿求解
	calculator m_hapticSolver;
	struct OperatorTrans
	{
		OperatorTrans() : buffer_haptic(16, 0.0f), buffer_graph(16, 0.0f), qg(6, 0.0f), qh(6, 0.0f), dirg(3, 0.0f), dirh(3, 0.0f) {}
		std::vector<float> buffer_haptic; // 原始物理位姿矩阵
		std::vector<float> buffer_graph; // 虚拟位姿矩阵
		std::vector<float> qg; // [x, y, z, OmegaX, OmegaY, OmegaZ]
		std::vector<float> qh;
		std::vector<float> dirg; // 3-dim
		std::vector<float> dirh; // 没用到，凑形参
		float graspL = 0; 
		float thetah = 0; // 工具的夹角
		bool initialized = false;

		// 用于更新 qg
		int collidedNum = 0;
		std::vector<float> collidedVertPos;
		std::vector<float> collidedVertNonPenetrationDir;
		std::vector<float> collidedVertDepth;
		void Clear() {
			collidedNum = 0;
			collidedVertPos.clear();
			collidedVertNonPenetrationDir.clear();
			collidedVertDepth.clear();
		}
		void AddCollidedVert(float cdPosX, float cdPosY, float cdPosZ, float cdDDirX, float cdDDirY, float cdDDirZ, float cdDepth) {
			collidedVertPos.push_back(cdPosX);
			collidedVertPos.push_back(cdPosY);
			collidedVertPos.push_back(cdPosZ);
			collidedVertNonPenetrationDir.push_back(cdDDirX);
			collidedVertNonPenetrationDir.push_back(cdDDirY);
			collidedVertNonPenetrationDir.push_back(cdDDirZ);
			collidedVertDepth.push_back(cdDepth);
		}
	};
	std::vector<OperatorTrans> m_operatorTransList;
	std::vector<vec6> m_operatorOutput6DForceList;

	float k_vc = 2;
	float k_c = 4;
	float k_vct = 400;
	int hapticIterNum = 3;

	void SetQH(const float* toolTrans, // 去掉无用参数
		std::vector<float>& m_dir, float& m_theta, std::vector<float>& m_qh,
		std::vector<float>& m_hapticToolTrans);
	void HapticCollideTriVert();
	void UpdateQG_6DOF(
		std::vector<float> m_qh, float m_graspL,
		int m_collidedNum, float* m_collidedVertDepth, float* m_collidedVertPos, float* m_collidedVertNonPenetrationDir,
		std::vector<float>& m_qg, std::vector<float>& m_dir_g,
		std::vector<float>& m_virtualToolTrans);
	void UpdateColliders();
	void DetectCollisionSV();
	void ResetTetVertCollision();
	void HapticCollideTetGripper();
	void MergeCollisionInfoTet();
	void MergeCollisionInfoTriVert();
	void CalculateContactWithCollisionInfo(
		std::vector<float> m_qg, float m_graspL,
		int m_collidedNum, float* m_collidedVertDepth, float* m_collidedVertPos, float* m_collidedVertNonPenetrationDir,
		vec3& total_FC, vec3& total_TC,
		mat3& total_partial_FC_X, mat3& total_partial_FC_Omega,
		mat3& total_partial_TC_X, mat3& total_partial_TC_Omega);
	void CalculateContactWithCollisionInfo(
		std::vector<float> m_qg, vec3 m_rotCenter,
		int m_collidedNum, float* m_collidedVertDepth, float* m_collidedVertPos, float* m_collidedVertNonPenetrationDir,
		vec3& total_FC, mat3& total_partial_FC_X);
	void UpdateQH(float* buffer, int operatorIndex);


	double m_hapticTriOpTime;
	double m_hapticQGOpTime;
	double m_haptic6dofTime;
	double m_hapticStepOpTime;
	bool OUTPUT_TIME_TO_CSV = false;
	bool OUTPUT_FORCE_TO_CSV = false;
	ofstream timeInfo;
	ofstream renderTimeInfo;
	ofstream timeInfo_6dof;
	ofstream forceInfoLeft;
	ofstream forceInfoRight;
	ofstream qgqhLeft;
	ofstream forceInfoDeformEnd;
	ofstream colNumInfo;
	unsigned int renderStepNumPassed = 0;
	unsigned int hapticStepNum = 0;
#pragma endregion
};

class CollisionBuffer {
private:
	//static CollisionBuffer* cb;
	std::mutex mtx;
	int testCnt;
	int m_collidedNumCpy;
	std::vector<float> m_tetVertPosMerged;
	std::vector<float> m_tetVertCollidedPosMerged;
	CollisionBuffer() { testCnt = 0; }
	CollisionBuffer(const CollisionBuffer& cb) = delete;
	const CollisionBuffer& operator=(const CollisionBuffer cb) = delete;
public:
	float tv_colStiffness = 500;
	float tv_colSpreadRadius = 0.8f;
	float tv_maxColVelocity = 2.5f;

	~CollisionBuffer() {}
	static CollisionBuffer& GetInstance() {
		static CollisionBuffer cb;
		return cb;
	}
	void WriteCollisionBuffer();
	void ReadCollisionBuffer();
	int GetCollidedNumCpy() {
		return m_collidedNumCpy;
	}
	float* GetTetVertPosMerged() {
		return m_tetVertPosMerged.data();
	}
	float* GetTetVertCollidedPosMerged() {
		return m_tetVertCollidedPosMerged.data();
	}
};

struct EdgeKey {
	EdgeKey(int a, int b) {
		k[0] = a;
		k[1] = b;
		f[0] = a;
		f[1] = b;
		Sort();
	}
	int k[2];
	int f[2];
	bool operator == (const EdgeKey& rhs) const {
		return k[0] == rhs.k[0] && k[1] == rhs.k[1];
	}

	bool operator < (const EdgeKey& rhs) const {
		if (rhs.k[0] != k[0]) 	return rhs.k[0] < k[0];
		else return rhs.k[1] < k[1];

	}
	void Sort() {
		if (k[0] > k[1]) {
			int temp = k[0];
			k[0] = k[1];
			k[1] = temp;
		}
	}
};

struct MeshVertexKey {
	MeshVertexKey() : v(0), vt(0) {}

	uint32_t v, vt;

	bool operator == (const MeshVertexKey& rhs) const {
		return v == rhs.v && vt == rhs.vt;
	}

	bool operator < (const MeshVertexKey& rhs) const {
		if (v != rhs.v) 	return v < rhs.v;
		else   				return vt < rhs.vt;
	}

};
