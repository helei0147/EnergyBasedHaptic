#pragma once
class CylinderCollider{
public:
	static int cylinder_num; // 统计总数，用于开辟显存
	bool active;
	float radius;
	float length;
	float positions[3];
	float lastPositions[3];
	float direction[3];
	CylinderCollider(float r, float l);
	CylinderCollider(float r, float l, const float(&p)[3], const float(&d)[3]);
	void SetCylinderCollider(const float(&p)[3], const float(&d)[3]);
	void SetCylinderActive(bool a);
};