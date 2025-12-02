#include "EHookObject.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

void EHookObject::InitFromFile() {
	m_gripperPivot.ReadFromFile();
}

void EHookObject::SetTrans(const float* rt) {
	float r[3][3], t[3];
	///将力反馈设备传来的数据的RT分来
	ForcepsTransformer::TransfromGL(rt, r, t);

	///根据工具转轴位置，生成两个钳嘴和手柄的运动
	m_forcepsTranslater.GenToolTransYZ(r, rt[16], rt[17]);
	///赋值给对应的工具
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperPivot.m_RT[0]);
}

void EHookObject::UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta) {
	// 1. 从图形位姿中获取旋转矩阵和位置
	std::vector<float> omega(qg.begin() + 3, qg.end());
	double angle = sqrt(omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2]);
	if (angle < 1e-6) {
		omega = { 1,0,0 };
	}
	else {
		omega[0] /= angle;
		omega[1] /= angle;
		omega[2] /= angle;
	}
	Eigen::Vector3d axis(omega[0], omega[1], omega[2]);

	Eigen::AngleAxisd rotation(angle, axis);

	Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();

	float r[3][3];
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			r[i][j] = rotation_matrix(i, j);
		}
	}
	float t[3] = { qg[0], qg[1], qg[2] };

	// 2. 解析碰撞体的位姿

	// 设置旋转矩阵
	m_forcepsTranslater.GenToolTransYZVirtual(r, theta, 0);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle_g, t, &m_gripperPivot.m_RT_g[0]);

	// 主圆柱体
	float pos[3] = { m_gripperPivot.m_RT_g[12], m_gripperPivot.m_RT_g[13], m_gripperPivot.m_RT_g[14] };
	float pivotDirX[3] = { m_gripperPivot.m_RT_g[0], m_gripperPivot.m_RT_g[1], m_gripperPivot.m_RT_g[2] };
	float pivotDirY[3] = { m_gripperPivot.m_RT_g[4], m_gripperPivot.m_RT_g[5], m_gripperPivot.m_RT_g[6] };
	float pivotDirZ[3] = { m_gripperPivot.m_RT_g[8], m_gripperPivot.m_RT_g[9], m_gripperPivot.m_RT_g[10] };

	// 3. 更新碰撞体的位姿
	SetCylinderCollider(0, pos, pivotDirZ);
	SetCylinderActive(0, true);

	SetSphereCollider(0, pos);
	SetSphereActive(0, false);
}