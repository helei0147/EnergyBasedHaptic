#define ENABLE_OPENGL
#include <iostream>
#include <string>
#include <chrono>
#include "../GLRender/opengl.hpp"
#include "../GLRender/application.hpp"
#include "../GLRender/ImguiHelper.hpp"
#include "../PDSim/SimManager.h"
#include "../PDSim/common.h"

char g_sceneName[256] = "cholecystectomy";// "cholecystectomy", "gynaecology", "playground", "tube", "gap"

Application* g_renderManager = nullptr;
SimManager* g_simManager = nullptr;

void Renderer::doUI() {
	ImGui::GetStyle().Alpha = 0.5f;
	
	ImGui::Begin(u8"Paras");
	ImGui::Text(u8"SimRate: %.2f", 1.0 / GetSimTime());
	ImGui::Text(u8"FrameRate: %.2f", 1.0 / GetFrameTime());
	ImGui::Text(u8"HapticRate: %.2f", 1.0 / GetHapticFrameTime());
	ImGui::Text(u8"Tet Num: %d", GetTetNum());
	ImGui::Text(u8"Surface Vert Num: %d", GetSpringVertNum());
	ImGui::Checkbox("sky", &m_sceneobject->use_sky);
	ImGui::Checkbox("SSAO", &m_sceneobject->enable_ao);

	// 创建一个浮点型滑动条

	ImGui::SliderFloat(u8"GravityX", &g_simManager->m_softHapticSolver.m_gravityX, -5.0, 5.0f);
	ImGui::SliderFloat(u8"GravityY", &g_simManager->m_softHapticSolver.m_gravityY, -5.0, 5.0f);
	ImGui::SliderFloat(u8"GravityZ", &g_simManager->m_softHapticSolver.m_gravityZ, -5.0, 5.0f);

	int OperatorCnt = GetOperatorCnt();
	for (int i = 0; i < OperatorCnt; i++) {
		int id = GetOperatorToolId(i);
		string str = "Tools: "+ to_string(i);
		if (ImGui::SliderInt(str.c_str(), &id, 0, GetOperatorToolCnt(i)))
			SetOperatorTool(i, id);
	}

	float tv_volumnStiffness = Gettv_volumnStiffness();
	if (ImGui::SliderFloat("tv_volumnStiffness", &tv_volumnStiffness, 0, 1250))
		Settv_volumnStiffness(tv_volumnStiffness);

	float tv_collisionStiffness = Gettv_collisionStiffness();
	if (ImGui::SliderFloat("tv_collisionStiffness", &tv_collisionStiffness, 0, 1000))
		Settv_collisionStiffness(tv_collisionStiffness);

	float tv_spreadRadius = Gettv_colSpreadRadius();
	if (ImGui::SliderFloat("tv_spreadRadius", &tv_spreadRadius, 0.0001f, 2.5f))
		Settv_colSpreadRadius(tv_spreadRadius);
    float tv_maxColVelocity = Gettv_maxColVelocity();
    if (ImGui::SliderFloat("tv_maxColVelocity", &tv_maxColVelocity, 0.0001f, 4.0f)) 
		Settv_maxColVelocity(tv_maxColVelocity);


	float tv_maxStiffnessSV2TV = Gettv_maxStiffnessSV2TV();
	if (ImGui::SliderFloat("tv_maxStiffnessSV2TV", &tv_maxStiffnessSV2TV, 20, 2000)) Settv_maxStiffnessSV2TV(tv_maxStiffnessSV2TV);
	float tv_minStiffnessSV2TV = Gettv_minStiffnessSV2TV();
	if (ImGui::SliderFloat("tv_minStiffnessSV2TV", &tv_minStiffnessSV2TV, 20, 500)) Settv_minStiffnessSV2TV(tv_minStiffnessSV2TV);
	ImGui::End();
	//抓取力的曲线图
	ImGui::Begin("Force visualization");
	static float forcex[90] = {};
	static float forcey[90] = {};
	static float forcez[90] = {};
	static float torquex[90] = {};
	static float torquey[90] = {};
	static float torquez[90] = {};
	static float volume[90] = {};
	static int values_offset = 0;

	static double refresh_time = 0.0;
	if (refresh_time == 0.0)
		refresh_time = ImGui::GetTime();
	while (refresh_time < ImGui::GetTime()) // Create data at fixed 60 Hz rate for the demo
	{
		vec6 force6D = g_simManager->m_softHapticSolver.m_operatorOutput6DForceList[0];
		forcex[values_offset] = force6D[0];
		forcey[values_offset] = force6D[1];
		forcez[values_offset] = force6D[2];
		torquex[values_offset] = force6D[3];
		torquey[values_offset] = force6D[4];
		torquez[values_offset] = force6D[5];

		values_offset = (values_offset + 1) % IM_ARRAYSIZE(forcex);
		refresh_time += 1.0f / 60.0f;
	}

	ImGui::PlotLines("Force x", forcex, IM_ARRAYSIZE(forcex), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::PlotLines("Force y", forcey, IM_ARRAYSIZE(forcey), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::PlotLines("Force z", forcez, IM_ARRAYSIZE(forcez), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::PlotLines("Torque x", torquex, IM_ARRAYSIZE(torquex), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::PlotLines("Torque y", torquey, IM_ARRAYSIZE(torquey), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::PlotLines("Torque z", torquez, IM_ARRAYSIZE(torquez), values_offset, nullptr, -3.0f, 3.0f, ImVec2(0, 80.0f));
	ImGui::End();
}


void loadSofts(std::string dataFolder = "../../data/") {
	float ssoaparas[8];
	int ret;
	std::string name;

	name = "cholecystectomy";
	ret = g_renderManager->CreatePBRObj(name,
		dataFolder + "cholecystectomy_Col.png",
		dataFolder + "cholecystectomy_Nor.png",
		dataFolder + "cholecystectomy_Met.png",
		dataFolder + "cholecystectomy_Ro.png");
	ssoaparas[0] = 2;
	ssoaparas[1] = 1.5;
	ssoaparas[2] = 0.9;
	ssoaparas[3] = 0.9;
	ssoaparas[4] = 0.009;
	ssoaparas[5] = 2.8;
	g_renderManager->SetSSAOParas(ssoaparas);
	SoftObject* cholecystectomyobj = new SoftObject();
	cholecystectomyobj->m_name = name;
	cholecystectomyobj->m_objFile = dataFolder + "cholecystectomy.obj"; 
	cholecystectomyobj->m_tetFile = dataFolder + "cholecystectomy.msh"; 
	cholecystectomyobj->m_glId = ret;
	g_simManager->m_softObjects.push_back(cholecystectomyobj);
	g_simManager->m_softHapticSolver.m_fixCenter.push_back(-4);//x
	g_simManager->m_softHapticSolver.m_fixCenter.push_back(-2);//y
	g_simManager->m_softHapticSolver.m_fixCenter.push_back(-12);//z
	g_simManager->m_softHapticSolver.m_fixRadius.push_back(2);//r
	g_simManager->m_softHapticSolver.k_vc = 0.9;
	g_simManager->m_softHapticSolver.k_c = 3;
	g_simManager->m_softHapticSolver.k_vct = 400;
	g_simManager->m_softHapticSolver.tv_volumnStiffness = 750.0f;
	g_simManager->m_softHapticSolver.tv_collisionStiffness = 1000.0f;



	//name = "playground";
	//ret = g_app->CreatePBRObj(name, 0.6, 0.5, 0.4, 0.2, 0.3);
	//ssoaparas[0] = 2;
	//ssoaparas[1] = 1.5;
	//ssoaparas[2] = 0.9;
	//ssoaparas[3] = 0.098;
	//ssoaparas[4] = 0.001;
	//ssoaparas[5] = 0.7;
	//g_app->SetSSAOParas(ssoaparas);	
	//SoftObject* playgroundobj = new SoftObject();
	//playgroundobj->m_name = name;
	//playgroundobj->m_objFile = dataFolder + "basic_scene.obj";
	//playgroundobj->m_tetFile = dataFolder + "basic_scene.msh";
	//playgroundobj->m_glId = ret;
	//m_simManager->m_softObjects.push_back(playgroundobj);
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-4);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1);//r

	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(2.5);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1.5);//r

	//m_simManager->m_softHapticSolver.k_vc = 2;
	//m_simManager->m_softHapticSolver.k_c = 4;
	//m_simManager->m_softHapticSolver.k_vct = 80;
	//m_simManager->m_softHapticSolver.tv_volumnStiffness = 300.0f;
	//m_simManager->m_softHapticSolver.tv_collisionStiffness = 30.0f;



	//name = "tube";
	//ret = g_app->CreatePBRObj(name, 0.6, 0.5, 0.4, 0.2, 0.3);
	//ssoaparas[0] = 2;
	//ssoaparas[1] = 1.5;
	//ssoaparas[2] = 0.9;
	//ssoaparas[3] = 0.098;
	//ssoaparas[4] = 0.001;
	//ssoaparas[5] = 0.7;
	//g_app->SetSSAOParas(ssoaparas);
	//SoftObject* tubeobj = new SoftObject();
	//tubeobj->m_name = name;
	//tubeobj->m_objFile = dataFolder + "tube.obj";
	//tubeobj->m_tetFile = dataFolder + "tube.msh";
	//tubeobj->m_glId = ret;
	//m_simManager->m_softObjects.push_back(tubeobj);
	//m_simManager->m_softHapticSolver.k_vc = 2;
	//m_simManager->m_softHapticSolver.k_c = 4;
	//m_simManager->m_softHapticSolver.k_vct = 80;
	//Settv_colSpreadRadius(0.3);
	//m_simManager->m_softHapticSolver.tv_volumnStiffness = 300.0f;
	//m_simManager->m_softHapticSolver.tv_collisionStiffness = 30.0f;


	//name = "gap";
	//ret = g_app->CreatePBRObj(name, 0.6, 0.5, 0.4, 0.2, 0.3);
	//ssoaparas[0] = 2;
	//ssoaparas[1] = 1.5;
	//ssoaparas[2] = 0.9;
	//ssoaparas[3] = 0.098;
	//ssoaparas[4] = 0.001;
	//ssoaparas[5] = 0.7;
	//g_app->SetSSAOParas(ssoaparas);
	//SoftObject* gapobj = new SoftObject();
	//gapobj->m_name = name;
	//gapobj->m_objFile = dataFolder + "gap.obj";
	//gapobj->m_tetFile = dataFolder + "gap.msh";
	//m_simManager->m_softObjects.push_back(gapobj);

	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(0);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(2.5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1.5);//r

	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(0);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-2.5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1.5);//r

	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(3.1);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(2.5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1.5);//r

	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(3.1);//x
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-2.5);//y
	//m_simManager->m_softHapticSolver.m_fixCenter.push_back(-11.5);//z
	//m_simManager->m_softHapticSolver.m_fixRadius.push_back(1.5);//r

	//m_simManager->m_softHapticSolver.k_vc = 2;
	//m_simManager->m_softHapticSolver.k_c = 6;
	//m_simManager->m_softHapticSolver.k_vct = 100;
	//Settv_colSpreadRadius(0.4);
	//m_simManager->m_softHapticSolver.tv_volumnStiffness = 490.0f;
	//m_simManager->m_softHapticSolver.tv_collisionStiffness = 45.0f;

	
	g_simManager->m_softHapticSolver.m_hapticSolver.set_k();

}

void loadTools(std::string dataFolder = "../../data/", int n = 0) {
	UDLog("初始化工具");
	float ssoaparas[8];
	int ret;
	std::string name;
	////增加钳子	
	int ret2 = g_renderManager->CreatePBRObj("GripperUpperLeft",
		dataFolder + "Gripper_Col.png",
		dataFolder + "Gripper_Nor.png",
		dataFolder + "Gripper_Met.png",
		dataFolder + "Gripper_Ro.png");
	int ret3 = g_renderManager->CreatePBRObj("GripperLowerLeft",
		dataFolder + "Gripper_Col.png",
		dataFolder + "Gripper_Nor.png",
		dataFolder + "Gripper_Met.png",
		dataFolder + "Gripper_Ro.png");
	int ret4 = g_renderManager->CreatePBRObj("GripperPivotLeft",
		dataFolder + "Gripper_Col.png",
		dataFolder + "Gripper_Nor.png",
		dataFolder + "Gripper_Met.png",
		dataFolder + "Gripper_Ro.png");
	int ret22 = g_renderManager->CreatePBRObj("GripperUpperLeft", 0.2, 0.8, 0.2, 0.5, 0.9);
	int ret33 = g_renderManager->CreatePBRObj("GripperLowerLeft", 0.2, 0.8, 0.2, 0.5, 0.9);
	int ret44 = g_renderManager->CreatePBRObj("GripperPivotLeft", 0.1, 0.4, 0.1, 0.5, 0.9);


	int ret5 = g_renderManager->CreatePBRObj("ElecHookLeft",
		dataFolder + "ElecHook_Col.png",
		dataFolder + "ElecHook_Nor.png",
		dataFolder + "Gripper_Met.png",
		dataFolder + "Gripper_Ro.png");
	int ret55 = g_renderManager->CreatePBRObj("ElecHookLeft", 0.2, 0.8, 0.2, 0.5, 0.9);

	

	// 往 m_toolList 中添加数据，m_toolList 与物理工具不一定数量相等，但是每个物理工具包含多个相同的虚拟工具
	for (int i = 0; i < n; i++) {
		vector<ToolObject*> _operator; // 单个操作手拥有的各种工具类型

		EmptyObject* emptyTool = new EmptyObject();
		_operator.push_back(emptyTool);

		GrasperObject* forceps = new GrasperObject();
		forceps->m_gripperUpper.m_objFile = dataFolder + "Gripper_Upper.obj";
		forceps->m_gripperLower.m_objFile = dataFolder + "Gripper_Lower.obj";
		forceps->m_gripperPivot.m_objFile = dataFolder + "Gripper_Pivot.obj";
		forceps->m_gripperUpper.m_Name = "GripperUpper_"+to_string(i);
		forceps->m_gripperLower.m_Name = "GripperLower_" + to_string(i);
		forceps->m_gripperPivot.m_Name = "GripperPivot_" + to_string(i);
		forceps->AddCylinderCollider(CylinderCollider(0.3, 31)); // 主柄
		forceps->AddCylinderCollider(CylinderCollider(0.3, 2)); // 上夹子
		forceps->AddCylinderCollider(CylinderCollider(0.3, 2)); // 下夹子
		forceps->AddGripperCollider(GripperCollider(0.3, 1.0, 2.0));
		forceps->m_gripperPivot.m_glId = ret4;
		forceps->m_gripperLower.m_glId = ret3;
		forceps->m_gripperUpper.m_glId = ret2;
		forceps->m_gripperPivot.m_glId_haptic = ret44;
		forceps->m_gripperLower.m_glId_haptic = ret33;
		forceps->m_gripperUpper.m_glId_haptic = ret22;
		forceps->InitFromFile();
		_operator.push_back(forceps);

		g_renderManager->UpdateMesh(ret4,
			forceps->m_gripperPivot.m_triIdx.size(),
			forceps->m_gripperPivot.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperPivot.m_triIdx[0],
			forceps->m_gripperPivot.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperPivot.m_triVertNormColor[0]);

		g_renderManager->UpdateMesh(ret3,
			forceps->m_gripperLower.m_triIdx.size(),
			forceps->m_gripperLower.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperLower.m_triIdx[0],
			forceps->m_gripperLower.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperLower.m_triVertNormColor[0]);

		g_renderManager->UpdateMesh(ret2,
			forceps->m_gripperUpper.m_triIdx.size(),
			forceps->m_gripperUpper.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperUpper.m_triIdx[0],
			forceps->m_gripperUpper.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperUpper.m_triVertNormColor[0]);

		g_renderManager->UpdateMesh(ret44, 
			forceps->m_gripperPivot.m_triIdx.size(),
			forceps->m_gripperPivot.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperPivot.m_triIdx[0],
			forceps->m_gripperPivot.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperPivot.m_triVertNormColor[0]);

		g_renderManager->UpdateMesh(ret33, 
			forceps->m_gripperLower.m_triIdx.size(),
			forceps->m_gripperLower.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperLower.m_triIdx[0],
			forceps->m_gripperLower.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperLower.m_triVertNormColor[0]);

		g_renderManager->UpdateMesh(ret22, 
			forceps->m_gripperUpper.m_triIdx.size(),
			forceps->m_gripperUpper.m_triIdx.size() * sizeof(unsigned int),
			&forceps->m_gripperUpper.m_triIdx[0],
			forceps->m_gripperUpper.m_triVertNormColor.size() * sizeof(float),
			&forceps->m_gripperUpper.m_triVertNormColor[0]);



		EHookObject* ehook = new EHookObject();
		ehook->m_gripperPivot.m_objFile = dataFolder + "ElecHook.obj";
		ehook->m_gripperPivot.m_Name = "ElecHook_" + to_string(i);
		ehook->AddCylinderCollider(CylinderCollider(0.25, 31)); // 主柄
		ehook->AddSphereCollider(SphereCollider(0.25)); // 尖端的碰撞球
		ehook->m_gripperPivot.m_glId = ret5;
		ehook->m_gripperPivot.m_glId_haptic = ret55;
		ehook->InitFromFile();
		_operator.push_back(ehook);

		g_renderManager->UpdateMesh(ret5,
			ehook->m_gripperPivot.m_triIdx.size(),
			ehook->m_gripperPivot.m_triIdx.size() * sizeof(unsigned int),
			&ehook->m_gripperPivot.m_triIdx[0],
			ehook->m_gripperPivot.m_triVertNormColor.size() * sizeof(float),
			&ehook->m_gripperPivot.m_triVertNormColor[0]);

		ehook->m_gripperPivot.m_glId_haptic = ret55;
		g_renderManager->UpdateMesh(ret55, 
			ehook->m_gripperPivot.m_triIdx.size(),
			ehook->m_gripperPivot.m_triIdx.size() * sizeof(unsigned int),
			&ehook->m_gripperPivot.m_triIdx[0],
			ehook->m_gripperPivot.m_triVertNormColor.size() * sizeof(float),
			&ehook->m_gripperPivot.m_triVertNormColor[0]);



		g_simManager->m_toolList.push_back(_operator);
	}
	g_simManager->m_softHapticSolver.m_operatorTransList.resize(n);

	// init graspL here for each operator
	g_simManager->m_softHapticSolver.m_operatorTransList[0].graspL = 3;

	// init output 6-DOF force for each operator
	g_simManager->m_softHapticSolver.m_operatorOutput6DForceList.resize(n);

	g_simManager->m_toolIndexList.resize(n, 0);
}


void UpateTriangle() {
	for (int i = 0; i < g_simManager->m_softObjects.size(); i++) {
		g_simManager->m_softObjects[i]->BuildOpenGLMesh9(
			g_simManager->m_softHapticSolver.m_springVertPos);
		g_renderManager->UpdateMesh(g_simManager->m_softObjects[i]->m_glId,
			g_simManager->m_softObjects[i]->m_renderTriIdx.size(),
			g_simManager->m_softObjects[i]->m_renderTriIdx.size() * sizeof(unsigned int),
			&g_simManager->m_softObjects[i]->m_renderTriIdx[0],
			sizeof(float) * g_simManager->m_softObjects[i]->m_renderTriVertNormUV.size(),
			&g_simManager->m_softObjects[i]->m_renderTriVertNormUV[0]);
	}
}


void UpdateMesh() {

	for (int i = 0; i < g_simManager->m_softObjects.size(); i++) {
		g_simManager->m_softObjects[i]->BuildOpenGLMesh9(
			g_simManager->m_softHapticSolver.m_springVertPos);
			g_renderManager->UpdateMeshConst(g_simManager->m_softObjects[i]->m_glId,
				0, 0,
				sizeof(float) * g_simManager->m_softObjects[i]->m_renderTriVertNormUV.size(),
				&g_simManager->m_softObjects[i]->m_renderTriVertNormUV[0]);
	}

	for (int toolGroupIdx = 0; toolGroupIdx < g_simManager->m_toolIndexList.size(); toolGroupIdx++)
	{
		auto& op = g_simManager->m_toolList[toolGroupIdx]; // 操作手
		auto& idxInGroup = g_simManager->m_toolIndexList[toolGroupIdx]; // 所选操作手的工具 id
        auto& operatorTrans = g_simManager->m_softHapticSolver.m_operatorTransList[toolGroupIdx];
		if(idxInGroup>0 && idxInGroup<op.size())
		{
            auto& tool = *op[idxInGroup];
            
			// 更新虚拟工具位姿，m_RT_g 已经在 updateColliders 中更新
			int& id0 = tool.m_gripperPivot.m_glId;
            int& id1 = tool.m_gripperLower.m_glId;
            int& id2 = tool.m_gripperUpper.m_glId;
			if (id0 >= 0)
			{
                g_renderManager->UpdateTransform(id0, tool.m_gripperPivot.m_RT_g);
			}
			if (id1 >= 0) g_renderManager->UpdateTransform(id1, tool.m_gripperLower.m_RT_g);
			if (id2 >= 0) g_renderManager->UpdateTransform(id2, tool.m_gripperUpper.m_RT_g);

			// 更新物理工具位姿
            float hapticBuffer[17];
            memcpy(hapticBuffer, operatorTrans.buffer_haptic.data(), 16 * sizeof(float));
            hapticBuffer[16] = operatorTrans.thetah;
            tool.SetTrans(hapticBuffer);
            int& id00 = tool.m_gripperPivot.m_glId_haptic;
            int& id11 = tool.m_gripperLower.m_glId_haptic;
            int& id22 = tool.m_gripperUpper.m_glId_haptic;
            if (id00 >= 0) g_renderManager->UpdateTransform(id00, tool.m_gripperPivot.m_RT);
            if (id11 >= 0) g_renderManager->UpdateTransform(id11, tool.m_gripperLower.m_RT);
            if (id22 >= 0) g_renderManager->UpdateTransform(id22, tool.m_gripperUpper.m_RT);
		}
	}

}

int main()
{
	std::string shaderFolder = "../../shader/";

	char buffer[256];
	sprintf(buffer, "../../data/");
	std::string dataFolder(buffer);

	std::string hdrfile = "env.hdr";
	g_simManager = new SimManager();
	g_simManager->m_dataFolder = "../../data/";
	g_simManager->InitCuda();

	//配置力反馈设备
	g_simManager->m_hapticDevice.m_deviceName.push_back("Default Device");
	g_simManager->m_hapticDevice.m_deviceCfgFile.push_back("../../data/TransConfLeft.cfg");
	g_simManager->m_hapticDevice.m_deviceMotionFile.push_back("../../data/hapticRecord/0.record");


	g_renderManager = Application::Instance();
	g_renderManager->InitRender(hdrfile, shaderFolder);


	loadSofts(dataFolder);
	loadTools(dataFolder, g_simManager->m_hapticDevice.GetOperatorCnt());
	g_simManager->PDInit();
	UpateTriangle();
	int ret = 0;	
	while (!ret) {
		if (g_simManager->m_cudaDeviceReady) {
			g_simManager->PDUpdate();
			g_simManager->ComputeNormals();
		}
		UpdateMesh();
		ret = g_renderManager->Render();
	}
	g_simManager->CloseManager();
	g_renderManager->ShutDown();
	return 0;
}


void UDLog(std::string mess)
{
	time_t now = time(0);
	char* dt = ctime(&now);
	if (g_simManager->m_logFid.is_open()) {
		g_simManager->m_logFid << dt;
		g_simManager->m_logFid << mess << endl << endl;
	}
}

void UDError(std::string mess)
{
	time_t now = time(0);
	char* dt = ctime(&now);
	if (g_simManager->m_logFid.is_open()) {
		g_simManager->m_logFid << dt;
		g_simManager->m_logFid << "Error：" << mess << endl << endl;
	}
}

void UDWarning(std::string mess)
{
	time_t now = time(0);
	char* dt = ctime(&now);
	if (g_simManager->m_logFid.is_open()) {
		g_simManager->m_logFid << dt;
		g_simManager->m_logFid << "Warning：" << mess << endl << endl;
	}
}

int GetOperatorCnt() {
	return g_simManager->m_toolList.size();
}

int GetOperatorToolId(int index) {
	return g_simManager->m_toolIndexList[index];
}

int GetOperatorToolCnt(int index) {
	return g_simManager->m_toolList[index].size();
}

void SetOperatorTool(int index, int type) {
	if (type >= 0 && type < g_simManager->m_toolList[index].size())
		g_simManager->m_toolIndexList[index] = type;
}

float GetSimTime() {
	return g_simManager->m_softHapticSolver.m_opTime;
}

float GetFrameTime() {
	return g_simManager->m_softHapticSolver.m_frameTime;
}

float GetHapticFrameTime()
{
	return g_simManager->m_softHapticSolver.m_hapFrameTime;
}

float GetSTTime() {
	return g_simManager->m_softHapticSolver.m_stTime;
}


float GetHapticTime() {
	return g_simManager->m_softHapticSolver.m_hapticStepOpTime;
}


int GetTetVertNum() {
	return g_simManager->m_softHapticSolver.GetTetVertNum();
}

int GetTetNum() {
	return g_simManager->m_softHapticSolver.GetTetNum();
}

int GetSpringVertNum() {
	return g_simManager->m_softHapticSolver.GetSpringVertNum();
}


void SetGravityX(float x) {
	g_simManager->m_softHapticSolver.m_gravityX = x;
}

void SetGravityY(float y) {
	g_simManager->m_softHapticSolver.m_gravityY = y;
}

void SetGravityZ(float z) {
	g_simManager->m_softHapticSolver.m_gravityZ = z;
}

float GetGravityX() {
	return g_simManager->m_softHapticSolver.m_gravityX;
}

float GetGravityY() {
	return g_simManager->m_softHapticSolver.m_gravityY;
}

float GetGravityZ() {
	return g_simManager->m_softHapticSolver.m_gravityZ;
}

float Getsv_damping() {
	return g_simManager->m_softHapticSolver.sv_damping;
}

void Setsv_damping(float v) {
	g_simManager->m_softHapticSolver.sv_damping = v;
}

float Getsv_dt() {
	return g_simManager->m_softHapticSolver.sv_dt;
}

void Setsv_dt(float v) {
	g_simManager->m_softHapticSolver.sv_dt = v;
}

float Getsv_springStiffness() {
	return g_simManager->m_softHapticSolver.sv_springStiffness;
}

void Setsv_springStiffness(float v) {
	g_simManager->m_softHapticSolver.sv_springStiffness = v;
}

float Getsv_collisionStiffness() {
	return g_simManager->m_softHapticSolver.sv_collisionStiffness;
}

void Setsv_collisionStiffness(float v) {
	g_simManager->m_softHapticSolver.sv_collisionStiffness = v;
}

float Getsv_adsorbStiffness() {
	return g_simManager->m_softHapticSolver.sv_adsorbStiffness;
}

void Setsv_adsorbStiffness(float v) {
	g_simManager->m_softHapticSolver.sv_adsorbStiffness = v;
}

float Getsv_maxStiffnessTV2SV() {
	return g_simManager->m_softHapticSolver.sv_maxStiffnessTV2SV;
}

void Setsv_maxStiffnessTV2SV(float v) {
	g_simManager->m_softHapticSolver.sv_maxStiffnessTV2SV = v;
}

float Getsv_minStiffnessTV2SV() {
	return g_simManager->m_softHapticSolver.sv_minStiffnessTV2SV;
}

void Setsv_minStiffnessTV2SV(float v) {
	g_simManager->m_softHapticSolver.sv_minStiffnessTV2SV = v;
}

float Gettv_damping() {
	return g_simManager->m_softHapticSolver.tv_damping;
}

void Settv_damping(float v) {
	g_simManager->m_softHapticSolver.tv_damping = v;
}

float Gettv_dt() {
	return g_simManager->m_softHapticSolver.tv_dt;
}

void Settv_dt(float v) {
	g_simManager->m_softHapticSolver.tv_dt = v;
}

float Gettv_iterateNum() {
	return g_simManager->m_softHapticSolver.tv_iterateNum;
}

void Settv_iterateNum(float v) {
	g_simManager->m_softHapticSolver.tv_iterateNum = v;
}

float Gettv_volumnStiffness() {
	return g_simManager->m_softHapticSolver.tv_volumnStiffness;
}

void Settv_volumnStiffness(float v) {
	g_simManager->m_softHapticSolver.tv_volumnStiffness = v;
}

float Gettv_collisionStiffness() {
	return CollisionBuffer::GetInstance().tv_colStiffness;
}

void Settv_collisionStiffness(float v) {
	CollisionBuffer::GetInstance().tv_colStiffness = v;
}

float Gettv_colSpreadRadius() {
	return CollisionBuffer::GetInstance().tv_colSpreadRadius;
}

void Settv_colSpreadRadius(float v) {
	CollisionBuffer::GetInstance().tv_colSpreadRadius = v;
}

float Gettv_maxColVelocity() {
	return CollisionBuffer::GetInstance().tv_maxColVelocity;
}

void Settv_maxColVelocity(float v) {
	CollisionBuffer::GetInstance().tv_maxColVelocity = v;
}

float Getk_vc() {
	return g_simManager->m_softHapticSolver.k_vc;
}

void Setk_vc(float v) {
	g_simManager->m_softHapticSolver.k_vc = v;
}

float Getk_c() {
	return g_simManager->m_softHapticSolver.k_c;
}

void Setk_c(float v) {
	g_simManager->m_softHapticSolver.k_c = v;
}

float Getk_vct() {
	return g_simManager->m_softHapticSolver.k_vct;
}

void Setk_vct(float v) {
	g_simManager->m_softHapticSolver.k_vct = v;
}

bool Getsv_transmit_grab() {
	return g_simManager->m_softHapticSolver.sv_transmit_grab;
}

void Setsv_transmit_grab(bool v) {
	g_simManager->m_softHapticSolver.sv_transmit_grab = v;
}

float Getsv_gripper_grab_smoothing() {
	return g_simManager->m_softHapticSolver.sv_gripper_grab_smoothing;
}

void Setsv_gripper_grab_smoothing(float v) {
	g_simManager->m_softHapticSolver.sv_gripper_grab_smoothing = v;
}

bool Getsv_transmit_adsorb() {
	return g_simManager->m_softHapticSolver.sv_transmit_adsorb;
}

void Setsv_transmit_adsorb(bool v) {
	g_simManager->m_softHapticSolver.sv_transmit_adsorb = v;
}

float Getsv_conColThreshold() {
	return g_simManager->m_softHapticSolver.sv_conColThreshold;
}

void Setsv_conColThreshold(float v) {
	g_simManager->m_softHapticSolver.sv_conColThreshold = v;
}

float Getsv_gripper_adsorb_smoothing() {
	return g_simManager->m_softHapticSolver.sv_gripper_adsorb_smoothing;
}

void Setsv_gripper_adsorb_smoothing(float v) {
	g_simManager->m_softHapticSolver.sv_gripper_adsorb_smoothing = v;
}

float Gettv_maxStiffnessSV2TV() {
	return g_simManager->m_softHapticSolver.tv_maxStiffnessSV2TV;
}

void Settv_maxStiffnessSV2TV(float v) {
	g_simManager->m_softHapticSolver.tv_maxStiffnessSV2TV = v;
}

float Gettv_minStiffnessSV2TV() {
	return g_simManager->m_softHapticSolver.tv_minStiffnessSV2TV;
}

void Settv_minStiffnessSV2TV(float v) {
	g_simManager->m_softHapticSolver.tv_minStiffnessSV2TV = v;
}


void GetRegion(float* region) {
	int tvnum = g_simManager->m_softHapticSolver.m_tetVertPos.size() / 3;
	region[0] = FLT_MAX;
	region[1] = FLT_MAX;
	region[2] = FLT_MAX;
	region[3] = -FLT_MAX;
	region[4] = -FLT_MAX;
	region[5] = -FLT_MAX;
	for (int i = 0; i < tvnum; i++) {
		int j = i * 3;
		float x = g_simManager->m_softHapticSolver.m_tetVertPos[j];
		float y = g_simManager->m_softHapticSolver.m_tetVertPos[j + 1];
		float z = g_simManager->m_softHapticSolver.m_tetVertPos[j + 2];
		region[0] = std::min(x, region[0]);
		region[1] = std::min(y, region[1]);
		region[2] = std::min(z, region[2]);

		region[3] = std::max(x, region[3]);
		region[4] = std::max(y, region[4]);
		region[5] = std::max(z, region[5]);
	}
}


void UpdateQH(int operatorIndex, float* buffer) {
	Solver& solver = g_simManager->m_softHapticSolver;
	solver.UpdateQH(buffer, operatorIndex);
}

void ComputeOperatorForce(int operatorIndex, float* f) {
	Solver& solver = g_simManager->m_softHapticSolver;

	std::vector<float> qgLerp, qgCur, qh;
	qgCur = solver.m_operatorTransList[operatorIndex].qg;
	qh = solver.m_operatorTransList[operatorIndex].qh;
	float graspL = solver.m_operatorTransList[operatorIndex].graspL;

	// 计算力
	vec3 Fvc, Tvc;
	solver.m_hapticSolver.calculateForce6DOF(qh, qgCur, graspL, Fvc, Tvc);
	f[0] = -Fvc[0];
	f[1] = -Fvc[1];
	f[2] = -Fvc[2];
	f[3] = -Tvc[0];
	f[4] = -Tvc[1];
	f[5] = -Tvc[2];
	float len_FVC = Fvc.norm();
	float len_TVC = Tvc.norm();

	solver.forceInfoLeft << f[0] << "," << f[1] << "," << f[2] << "," << f[3] << "," << f[4] << "," << f[5] << len_FVC << "," << len_TVC << endl;
}


int GetCollidedNum(int index) {
	Solver& solver = g_simManager->m_softHapticSolver;
	return solver.m_operatorTransList[index].collidedNum;
}


