#include "SphereCollider.h"
int SphereCollider::sphere_num = 0;

SphereCollider::SphereCollider(float r) {
	radius = r;
}

void SphereCollider::SetSphereCollider(const float(&p)[3]) {
	for (int i = 0; i < 3; ++i) {
		lastPositions[i] = positions[i];
		positions[i] = p[i];
	}
}

void SphereCollider::SetSphereActive(bool a) {
	active = a;
}