#include "EmptyObject.h"

void EmptyObject::InitFromFile() {
	m_gripperPivot.ReadFromFile();
}


void EmptyObject::SetTrans(const float* rt) {
	float r[3][3], t[3];
	///将力反馈设备传来的数据的RT分来
	ForcepsTransformer::TransfromGL(rt, r, t);
	///根据工具转轴位置，生成两个钳嘴和手柄的运动
	m_forcepsTranslater.GenToolTransYZ(r, rt[16], rt[17]);
	///赋值给对应的工具
	ForcepsTransformer::TranstoGL(m_forcepsTranslater.handle, t, &m_gripperPivot.m_RT[0]);
}

void EmptyObject::UpdateColliders(const std::vector<float>& qg, const std::vector<float>& dir, float theta){

}