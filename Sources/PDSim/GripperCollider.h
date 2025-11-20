#pragma once
class GripperCollider {
public:
	static int gripper_num;
	bool active;
	float position[3];
	float scale[3];
	float pivot_x[3]; // 主柄的x轴方向
	float pivot_y[3];
	float pivot_z[3];
	float upper_x[3];
	float upper_y[3];
	float upper_z[3];
	float lower_x[3];
	float lower_y[3];
	float lower_z[3];
	float angle;

	bool closeFlag; // 中间变量
	int gripper_no; // 序列号，用于区别 cuda 数组


	GripperCollider(float x, float y, float z);
	void SetGripperCollider(float a, const float(&p)[3], 
		const float(&p_x)[3], const float(&p_y)[3], const float(&p_z)[3],
		const float(&u_x)[3], const float(&u_y)[3], const float(&u_z)[3],
		const float(&l_x)[3], const float(&l_y)[3], const float(&l_z)[3]);
	void SetGripperActive(bool a);
};