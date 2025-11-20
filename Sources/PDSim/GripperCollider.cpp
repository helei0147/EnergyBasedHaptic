#include "GripperCollider.h"
int GripperCollider::gripper_num = 0;

GripperCollider::GripperCollider(float x, float y, float z) {
	scale[0] = x;
	scale[1] = y;
	scale[2] = z;
	closeFlag = false;
}

void GripperCollider::SetGripperCollider(float a, const float(&p)[3],
	const float(&p_x)[3], const float(&p_y)[3], const float(&p_z)[3],
	const float(&u_x)[3], const float(&u_y)[3], const float(&u_z)[3],
	const float(&l_x)[3], const float(&l_y)[3], const float(&l_z)[3]) {
	angle = a;
	for (int i = 0; i < 3; i++) {
		position[i] = p[i];
		pivot_x[i] = p_x[i];
		pivot_y[i] = p_y[i];
		pivot_z[i] = p_z[i];
		upper_x[i] = u_x[i];
		upper_y[i] = u_y[i];
		upper_z[i] = u_z[i];
		lower_x[i] = l_x[i];
		lower_y[i] = l_y[i];
		lower_z[i] = l_z[i];
	}
}

void GripperCollider::SetGripperActive(bool a) {
	active = a;
}