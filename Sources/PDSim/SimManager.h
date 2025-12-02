#pragma once
///此函数创建dll插件入口
#include <string>
#include <map>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>
#include <set>
#include "Solver.h"
#include "HapticDevice.h"
#include "SoftObject.h"
#include "ToolObject.h"
#include "EHookObject.h"
#include "GrasperObject.h"
#include "EmptyObject.h"


class SimManager {
public:
	std::string m_moduleName = "Hepatectomy";
	std::fstream m_logFid;
	std::string m_dataFolder;
	std::vector<SoftObject*> m_softObjects;
	std::vector<std::vector<ToolObject*>> m_toolList;
	std::vector<int> m_toolIndexList; // 每个操作手对应的工具 id
	Solver m_softHapticSolver;
	HapticDevice m_hapticDevice;
	bool m_cudaDeviceReady = false;
	SimManager();
	~SimManager();
	void CloseManager();
	void PDInit();
	void InitCuda();
	void InitSolverData();
	void PDUpdate();
	void ComputeNormals();
};