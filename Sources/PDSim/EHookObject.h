#pragma once
#include "ToolObject.h"
class EHookObject : public ToolObject {
public:
	virtual void SetTrans(const float* rt);
	virtual void InitFromFile();
	virtual void UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta);
};