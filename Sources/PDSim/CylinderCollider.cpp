#include "CylinderCollider.h"
int CylinderCollider::cylinder_num = 0;

CylinderCollider::CylinderCollider(float r, float l) {
	radius = r;
	length = l;
}

CylinderCollider::CylinderCollider(float r, float l, const float(&p)[3], const float(&d)[3]) {
	radius = r;
	length = l;
	for (int i = 0; i < 3; ++i) {
		positions[i] = p[i];
		direction[i] = d[i];
	}
}

void CylinderCollider::SetCylinderCollider(const float(&p)[3], const float(&d)[3]) {
	for (int i = 0; i < 3; ++i) {
		lastPositions[i] = positions[i];
		positions[i] = p[i];
		direction[i] = d[i];
	}
}

void CylinderCollider::SetCylinderActive(bool a) {
	active = a;
}