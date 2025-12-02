#pragma once
class SphereCollider{
public:
	static int sphere_num; // 统计总数，用于开辟显存
	bool active;
	float radius;
	float positions[3];
	float lastPositions[3];
	SphereCollider(float r);
	void SetSphereCollider(const float(&p)[3]);
	void SetSphereActive(bool a);
};
