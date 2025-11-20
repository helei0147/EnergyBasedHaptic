#pragma once
#include "ToolObject.h"
class GrasperObject : public ToolObject {
public:
	void SetTrans(const float* rt);
	void InitFromFile();
	virtual void SendColliders2Gpu(int cylinderOffset, int sphereOffset);
	virtual void UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta);
};