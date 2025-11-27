#include "GrasperObject.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

void GrasperObject::InitFromFile() {
	m_gripperLower.ReadFromFile();
	m_gripperUpper.ReadFromFile();
	m_gripperPivot.ReadFromFile();
}

void GrasperObject::SetTrans(const float* rt) {
	float r[3][3], t[3];
	///将力反馈设备传来的数据的RT分来
	ForcepsTransformer::TransfromGL(rt, r, t);
	m_forcepsTranslater.GenToolTransYZ(r, rt[16], -3.14159f / 2);
	m_gripperAngle = rt[16];
	///赋值给对应的工具
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperPivot.m_RT[0]);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.uper, t, &m_gripperUpper.m_RT[0]);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.lower, t, &m_gripperLower.m_RT[0]);
}

void GrasperObject::SendColliders2Gpu(int cylinderOffset, int sphereOffset) {
	// 组织数据到一个vec中
	int cnum = cylinderColliders.size();
	std::vector<float> posV;
	std::vector<float> dirV;
	std::vector<float> radiusV;
	std::vector<float> lengthV;
	for (int i = 0; i < cnum; i++) {
		CylinderCollider cylinder = cylinderColliders[i];
		posV.insert(posV.end(), std::begin(cylinder.positions), std::end(cylinder.positions));
		dirV.insert(dirV.end(), std::begin(cylinder.direction), std::end(cylinder.direction));
		radiusV.push_back(cylinder.radius);
		lengthV.push_back(cylinder.length);
	}

	// 数据传送
	memset(cylinder_active, 0, cylinder_num * sizeof(char));
	memset(cylinder_active + cylinderOffset, 1, 1 * cnum * sizeof(char));
	cudaMemcpy(cylinder_last_pos + cylinderOffset, cylinder_pos + cylinderOffset, 3 * cnum * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemcpy(cylinder_pos + cylinderOffset, posV.data(), 3 * cnum * sizeof(float), cudaMemcpyHostToDevice);
	cudaMemcpy(cylinder_dir + cylinderOffset, dirV.data(), 3 * cnum * sizeof(float), cudaMemcpyHostToDevice);
	memcpy(cylinder_radius + cylinderOffset, radiusV.data(), 1 * cnum * sizeof(float));
	memcpy(cylinder_length + cylinderOffset, lengthV.data(), 1 * cnum * sizeof(float));

	int snum = sphereColliders.size();
	posV.clear();
	radiusV.clear();
	for (int i = 0; i < snum; i++) {
		SphereCollider sphere = sphereColliders[i];
		posV.insert(posV.end(), std::begin(sphere.positions), std::end(sphere.positions));
		radiusV.push_back(sphere.radius);
	}

	memset(sphere_active, 0, sphere_num * sizeof(char));
	memset(sphere_active + sphereOffset, 1, 1 * snum * sizeof(char));
	cudaMemcpy(sphere_last_pos + sphereOffset, sphere_pos + sphereOffset, 3 * snum * sizeof(float), cudaMemcpyDeviceToDevice);
	cudaMemcpy(sphere_pos + sphereOffset, posV.data(), 3 * snum * sizeof(float), cudaMemcpyHostToDevice);
	memcpy(sphere_radius + sphereOffset, radiusV.data(), 1 * snum * sizeof(float));

}

void GrasperObject::UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta) {
	
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
	m_forcepsTranslater.GenToolTransYZVirtual(r, theta, -3.14159f / 2);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle_g, t, &m_gripperPivot.m_RT_g[0]);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.uper_g, t, &m_gripperUpper.m_RT_g[0]);
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.lower_g, t, &m_gripperLower.m_RT_g[0]);

	// 主圆柱体
	float pos[3] = { m_gripperPivot.m_RT_g[12], m_gripperPivot.m_RT_g[13], m_gripperPivot.m_RT_g[14] };
	float pivotDirX[3] = { m_gripperPivot.m_RT_g[0], m_gripperPivot.m_RT_g[1], m_gripperPivot.m_RT_g[2] };
	float pivotDirY[3] = { m_gripperPivot.m_RT_g[4], m_gripperPivot.m_RT_g[5], m_gripperPivot.m_RT_g[6] };
	float pivotDirZ[3] = { m_gripperPivot.m_RT_g[8], m_gripperPivot.m_RT_g[9], m_gripperPivot.m_RT_g[10] };
	
	// 上下夹子
	float dirX[3] = { 0.0, 1.0, 0.0 };
	float dirYUpper[3] = { 1.0, 0.0, 0.0 };
	float dirYLower[3] = { -1.0, 0.0, 0.0 };
	float dirZ[3] = { 0.0, 0.0, -1.0 };
	float upperDirX[3], upperDirY[3], upperDirZ[3], lowerDirX[3], lowerDirY[3], lowerDirZ[3];
	float upperM[3][3], lowerM[3][3];
	ForcepsTransformer::TransFromGL(upperM, m_gripperUpper.m_RT_g);
	ForcepsTransformer::TransFromGL(lowerM, m_gripperLower.m_RT_g);
	ForcepsTransformer::M3V3(upperM, dirX, upperDirX);
	ForcepsTransformer::M3V3(upperM, dirYUpper, upperDirY);
	ForcepsTransformer::M3V3(upperM, dirZ, upperDirZ);
	ForcepsTransformer::M3V3(lowerM, dirX, lowerDirX);
	ForcepsTransformer::M3V3(lowerM, dirYLower, lowerDirY);
	ForcepsTransformer::M3V3(lowerM, dirZ, lowerDirZ);
	ForcepsTransformer::NormalizeVec3(upperDirX);
	ForcepsTransformer::NormalizeVec3(upperDirY);
	ForcepsTransformer::NormalizeVec3(upperDirZ);
	ForcepsTransformer::NormalizeVec3(lowerDirX);
	ForcepsTransformer::NormalizeVec3(lowerDirY);
	ForcepsTransformer::NormalizeVec3(lowerDirZ);


	//// debug 传给全局变量用于渲染
	//SetVirtualTool(pos, 
	//	pivotDirX, pivotDirY, pivotDirZ,
	//	upperDirX, upperDirY, upperDirZ,
	//	lowerDirX, lowerDirY, lowerDirZ);

	// 3. 更新碰撞体的位姿
	SetCylinderCollider(0, pos, pivotDirZ);
	SetCylinderCollider(1, pos, upperDirZ);
	SetCylinderCollider(2, pos, lowerDirZ);
	SetGripperCollider(0, theta, pos, pivotDirX, pivotDirY, pivotDirZ,
		upperDirX, upperDirY, upperDirZ, lowerDirX, lowerDirY, lowerDirZ);

	// 4. 控制小夹钳的碰撞体类型，按照角度切换
	bool useGripperCollider = theta < gripper_max_angle;
	SetCylinderActive(0, !useGripperCollider);
	SetCylinderActive(1, !useGripperCollider);
	SetCylinderActive(2, !useGripperCollider);
	SetGripperActive(0, useGripperCollider);
}
