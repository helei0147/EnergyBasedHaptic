#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include "gpu/gpuvar.h"
#include "gpu/gpufun.h"
#include "CylinderCollider.h"
#include "SphereCollider.h"
#include "GripperCollider.h"
#include "common.h"

struct ForcepsTransformer {
	///针对抓钳生成平移旋转矩阵
	int closeAxis, handleAxis;
	bool m_uperRotate;
	bool m_lowerRotate;
	float uper[3][3];
	float lower[3][3];
	float handle[3][3];
	float uper_local[3][3];
	float lower_local[3][3];
	float uper_g[3][3];
	float lower_g[3][3];
	float handle_g[3][3];
	float uper_local_g[3][3];
	float lower_local_g[3][3];
	float t[3];
	ForcepsTransformer();
	void GenToolTransX(const float r[3][3], float toolAngle = 0.0f, float rotatorAngle = 0.0f);
	void GenToolTransYZ(const float r[3][3], float toolAngle = 0.0f, float rotatorAngle = 0.0f);
	void GenToolTransYZVirtual(const float r[3][3], float toolAngle = 0.0f, float rotatorAngle = 0.0f);
	void static TransfromGL(const float* gltrans, float r[3][3], float t[3], float motionScale = 1.0f);
	void static TransfromGL(const float* gltrans, double r[3][3], double t[3], float motionScale = 1.0f);
	void static TransFromGL(float M[3][3], const float* M_GL);
	void static TranstoGL(const float r[3][3], const float t[3], float* gltrans);
	void static TransFromAxisAngle(float trans[3][3], float angle, int axis);
	void static TransFromAxisX(float trans[3][3], float angle);
	void static TransFromAxisY(float trans[3][3], float angle);
	void static TransFromAxisZ(float trans[3][3], float angle);
	void static M3vs3(const float inleft[3][3], const float inright[3][3], float out[3][3]);
	void static M3V3(const float inleft[3][3], const float inright[3], float out[3]);
	void static NormalizeVec3(float* vec3);
	float static Norm(const float(&vec3)[3]);
};

struct RigidObject {
	///存储物体信息
	int m_glId = -1;//在opengl场景中的id
    int m_glId_haptic = -1;  // 在opengl场景中的id
	std::string m_Name;//创建物体用的名称
	std::string m_objFile;
	float m_radius;
	float m_length;
	float m_RT[16];
	float m_RT_g[16];
	RigidObject() {
		memset(m_RT, 0, 16 * sizeof(float));
		m_RT[0] = 1;
		m_RT[5] = 1;
		m_RT[10] = 1;
		m_RT[15] = 1;
	}


	int m_vertNum = 0;///该物体实际的顶点
	int m_triNum = 0;///该物体三角网格的顶点
	std::vector<unsigned int> m_triIdx;///三角形索引
	std::vector<float> m_triColor;//渲染三角形法向量
	std::vector<float> m_triUV;//渲染三角形法向量
	std::vector<float> m_triVertsOrg;//渲染三角形顶点
	std::vector<float> m_triNormOrg;//渲染三角形法向量
	std::vector<float> m_triVertNormColor;//渲染三角形顶点


	void ReadFromFile();
	void ReadMeshwithSmoothGroup();
	void ReadMeshwithoutSmoothGroup();
};

class ToolObject {
public:
	ToolObject() {};
	~ToolObject() {};
	bool m_meshUpdate = true;
	RigidObject m_gripperLower;
	RigidObject m_gripperUpper;
	RigidObject m_gripperPivot;
	float m_gripperAngle;
	std::vector<CylinderCollider> cylinderColliders;
	std::vector<SphereCollider> sphereColliders;
	std::vector<GripperCollider> gripperColliders;
	int cylinderOffset;
	int sphereOffset;
	int gripperOffset;
	ForcepsTransformer m_forcepsTranslater;
	virtual void SetTrans(const float* rt);
	virtual void InitFromFile();
	virtual void AddCylinderCollider(CylinderCollider cylinder);
	virtual void SetCylinderCollider(int id, const float(&p)[3], const float(&d)[3]);
	virtual void SetCylinderActive(int id, bool a);
	virtual void AddSphereCollider(SphereCollider sphere);
	virtual void SetSphereCollider(int id, const float(&p)[3]);
	virtual void SetSphereActive(int id, bool a);
	virtual void AddGripperCollider(GripperCollider gripper);
	virtual void SetGripperCollider(int id, float a, const float(&p)[3],
		const float(&p_x)[3], const float(&p_y)[3], const float(&p_z)[3],
		const float(&u_x)[3], const float(&u_y)[3], const float(&u_z)[3],
		const float(&l_x)[3], const float(&l_y)[3], const float(&l_z)[3]);
	virtual void SetGripperActive(int id, bool a);
	virtual void SendColliders2Gpu(int cylinderOffset, int sphereOffset);
	virtual void UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta) = 0;
	std::vector<CylinderCollider>& GetCylinderColliders();
	std::vector<SphereCollider>& GetSphereColliders();
	std::vector<GripperCollider>& GetGripperColliders();
};