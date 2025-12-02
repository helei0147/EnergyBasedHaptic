#pragma once
#include "ToolObject.h"
class EmptyObject : public ToolObject {
public:
	void SetTrans(const float* rt);
	void InitFromFile();
	virtual void UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta);
};