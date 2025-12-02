#include "common.h"
#include "HapticDevice.h"
#include "../3rd/eigen/Eigen/Core"
#include <iostream>
#include <map>
#include <locale>
#include <codecvt>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Cholesky"
#include "Eigen/src/Geometry/Quaternion.h"

#include "gpufun.h"
#include "../PDSim/SimManager.h"
extern SimManager* g_simManager;

HANDLE g_hapticThread;
bool g_hapticPlayBackMode = true;
LARGE_INTEGER hapFreq;
LARGE_INTEGER hap_t1;
LARGE_INTEGER hap_t2;

int _stdcall SetHapticState(void* pParam);

DWORD RunHapticState(LPVOID lpvThreadParam);

void Mul44(const Matrix44& left, const Matrix44& right, Matrix44& res) {
	res[0] = left[0] * right[0] + left[4] * right[1] + left[8] * right[2] + left[12] * right[3];
	res[1] = left[1] * right[0] + left[5] * right[1] + left[9] * right[2] + left[13] * right[3];
	res[2] = left[2] * right[0] + left[6] * right[1] + left[10] * right[2] + left[14] * right[3];
	res[3] = left[3] * right[0] + left[7] * right[1] + left[11] * right[2] + left[15] * right[3];

	res[4] = left[0] * right[4] + left[4] * right[5] + left[8] * right[6] + left[12] * right[7];
	res[5] = left[1] * right[4] + left[5] * right[5] + left[9] * right[6] + left[13] * right[7];
	res[6] = left[2] * right[4] + left[6] * right[5] + left[10] * right[6] + left[14] * right[7];
	res[7] = left[3] * right[4] + left[7] * right[5] + left[11] * right[6] + left[15] * right[7];

	res[8] = left[0] * right[8] + left[4] * right[9] + left[8] * right[10] + left[12] * right[11];
	res[9] = left[1] * right[8] + left[5] * right[9] + left[9] * right[10] + left[13] * right[11];
	res[10] = left[2] * right[8] + left[6] * right[9] + left[10] * right[10] + left[14] * right[11];
	res[11] = left[3] * right[8] + left[7] * right[9] + left[11] * right[10] + left[15] * right[11];

	res[12] = left[0] * right[12] + left[4] * right[13] + left[8] * right[14] + left[12] * right[15];
	res[13] = left[1] * right[12] + left[5] * right[13] + left[9] * right[14] + left[13] * right[15];
	res[14] = left[2] * right[12] + left[6] * right[13] + left[10] * right[14] + left[14] * right[15];
	res[15] = left[3] * right[12] + left[7] * right[13] + left[11] * right[14] + left[15] * right[15];
}

void Mul44(const float* left, const Matrix44& right, float* res) {
	res[0] = left[0] * right[0] + left[4] * right[1] + left[8] * right[2] + left[12] * right[3];
	res[1] = left[1] * right[0] + left[5] * right[1] + left[9] * right[2] + left[13] * right[3];
	res[2] = left[2] * right[0] + left[6] * right[1] + left[10] * right[2] + left[14] * right[3];
	res[3] = left[3] * right[0] + left[7] * right[1] + left[11] * right[2] + left[15] * right[3];

	res[4] = left[0] * right[4] + left[4] * right[5] + left[8] * right[6] + left[12] * right[7];
	res[5] = left[1] * right[4] + left[5] * right[5] + left[9] * right[6] + left[13] * right[7];
	res[6] = left[2] * right[4] + left[6] * right[5] + left[10] * right[6] + left[14] * right[7];
	res[7] = left[3] * right[4] + left[7] * right[5] + left[11] * right[6] + left[15] * right[7];

	res[8] = left[0] * right[8] + left[4] * right[9] + left[8] * right[10] + left[12] * right[11];
	res[9] = left[1] * right[8] + left[5] * right[9] + left[9] * right[10] + left[13] * right[11];
	res[10] = left[2] * right[8] + left[6] * right[9] + left[10] * right[10] + left[14] * right[11];
	res[11] = left[3] * right[8] + left[7] * right[9] + left[11] * right[10] + left[15] * right[11];

	res[12] = left[0] * right[12] + left[4] * right[13] + left[8] * right[14] + left[12] * right[15];
	res[13] = left[1] * right[12] + left[5] * right[13] + left[9] * right[14] + left[13] * right[15];
	res[14] = left[2] * right[12] + left[6] * right[13] + left[10] * right[14] + left[14] * right[15];
	res[15] = left[3] * right[12] + left[7] * right[13] + left[11] * right[14] + left[15] * right[15];
}

void runServoLoop(LPVOID pp) {
	HapticDevice* hapticDevice = (HapticDevice*)pp;
	for (int index = 0; index < g_simManager->m_softHapticSolver.m_operatorTransList.size(); index++) {
		string& filename = hapticDevice->m_deviceMotionFile[index];
		FILE* fptr = fopen(filename.c_str(), "r");
		while (true& fptr != nullptr) {
			HapticDevice::OperatorInfo tool;
			int readSuccessNum = fscanf(fptr, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
				&tool.buffer[0], &tool.buffer[1], &tool.buffer[2], &tool.buffer[3],
				&tool.buffer[4], &tool.buffer[5], &tool.buffer[6], &tool.buffer[7],
				&tool.buffer[8], &tool.buffer[9], &tool.buffer[10], &tool.buffer[11],
				&tool.buffer[12], &tool.buffer[13], &tool.buffer[14], &tool.buffer[15],
				&tool.buffer[16], &tool.buffer[17], &tool.buffer[18]);
			if (readSuccessNum == 19) {
				memcpy(tool.M, tool.buffer, 16 * sizeof(float));
				hapticDevice->m_operatorList[index].toolMotion.push_back(tool);
			}
			else
				break;
		}
	}

	DWORD  threadId;
	g_hapticThread = CreateThread(NULL, 0, RunHapticState, pp, 0, &threadId);
}

void shutDownServoLoop() {
	TerminateThread(g_hapticThread, 0);
	CloseHandle(g_hapticThread);
}

void HapticTranslator::LoadTransFormFile() {
	///////////////从 3ds Max 导出的数据
	FILE* fid = fopen(configFile.c_str(), "r");
	if (!fid) {
		UDError(configFile+"打开失败");
		return;
	}
	UDLog(configFile + "打开成功");
	float toolRx, toolRy, toolRz;
	float toolTx, toolTy, toolTz;
	float organRx, organRy, organRz;
	float organTx, organTy, organTz;

	fscanf(fid, "%f", &toolRx);
	fscanf(fid, "%f", &toolRy);
	fscanf(fid, "%f", &toolRz);

	fscanf(fid, "%f", &toolTx);
	fscanf(fid, "%f", &toolTy);
	fscanf(fid, "%f", &toolTz);

	fscanf(fid, "%f", &organRx);
	fscanf(fid, "%f", &organRy);
	fscanf(fid, "%f", &organRz);

	fscanf(fid, "%f", &organTx);
	fscanf(fid, "%f", &organTy);
	fscanf(fid, "%f", &organTz);
	fclose(fid);
	//////从角度转到弧度
	organRx *= 0.017453292519943f;
	organRy *= 0.017453292519943f;
	organRz *= 0.017453292519943f;
	toolRx *= 0.017453292519943f;
	toolRy *= 0.017453292519943f;
	toolRz *= 0.017453292519943f;

	/////////工具自身移动的矩阵
	deviceTrans = GenMatrixform3dxMax(toolRx, toolRy, toolRz, toolTx, toolTy, toolTz);
	Matrix44 organrt_inv = GenMatrixform3dxMax(organRx, organRy, organRz, organTx, organTy, organTz);
	Matrix44 organrt = AffineInverse(organrt_inv);
	Mul44(organrt, deviceTrans, trans2Device);
	trans2World = AffineInverse(trans2Device);
}

void HapticTranslator::Trans2World(float* res, float scale) {

	res[12] *= scale;
	res[13] *= scale;
	res[14] *= scale;

	float right[32];
	memcpy(right, res, 16 * sizeof(float));
	Matrix44& left = trans2Device;
	res[0] = left[0] * right[0] + left[4] * right[1] + left[8] * right[2] + left[12] * right[3];
	res[1] = left[1] * right[0] + left[5] * right[1] + left[9] * right[2] + left[13] * right[3];
	res[2] = left[2] * right[0] + left[6] * right[1] + left[10] * right[2] + left[14] * right[3];
	res[3] = left[3] * right[0] + left[7] * right[1] + left[11] * right[2] + left[15] * right[3];

	res[4] = left[0] * right[4] + left[4] * right[5] + left[8] * right[6] + left[12] * right[7];
	res[5] = left[1] * right[4] + left[5] * right[5] + left[9] * right[6] + left[13] * right[7];
	res[6] = left[2] * right[4] + left[6] * right[5] + left[10] * right[6] + left[14] * right[7];
	res[7] = left[3] * right[4] + left[7] * right[5] + left[11] * right[6] + left[15] * right[7];

	res[8] = left[0] * right[8] + left[4] * right[9] + left[8] * right[10] + left[12] * right[11];
	res[9] = left[1] * right[8] + left[5] * right[9] + left[9] * right[10] + left[13] * right[11];
	res[10] = left[2] * right[8] + left[6] * right[9] + left[10] * right[10] + left[14] * right[11];
	res[11] = left[3] * right[8] + left[7] * right[9] + left[11] * right[10] + left[15] * right[11];

	res[12] = left[0] * right[12] + left[4] * right[13] + left[8] * right[14] + left[12] * right[15];
	res[13] = left[1] * right[12] + left[5] * right[13] + left[9] * right[14] + left[13] * right[15];
	res[14] = left[2] * right[12] + left[6] * right[13] + left[10] * right[14] + left[14] * right[15];
	res[15] = left[3] * right[12] + left[7] * right[13] + left[11] * right[14] + left[15] * right[15];
}

void HapticTranslator::TransForce2Device(float* f) {
	float xin = f[0]; float yin = f[1]; float zin = f[2];
	f[0] = xin * trans2World[0] + yin * trans2World[4] + zin * trans2World[8];
	f[1] = xin * trans2World[1] + yin * trans2World[5] + zin * trans2World[9];
	f[2] = xin * trans2World[2] + yin * trans2World[6] + zin * trans2World[10];
}

void HapticTranslator::TransForce2Device(double* f) {
	float xin = f[0]; float yin = f[1]; float zin = f[2];
	f[0] = xin * trans2World[0] + yin * trans2World[4] + zin * trans2World[8];
	f[1] = xin * trans2World[1] + yin * trans2World[5] + zin * trans2World[9];
	f[2] = xin * trans2World[2] + yin * trans2World[6] + zin * trans2World[10];
}

Matrix44 HapticTranslator::GenMatrixform3dxMax(float rx, float ry, float rz, float tx, float ty, float tz) {
	//先绕x逆时针，再绕y逆时针，再绕z逆时针
	float rotate[3][3];
	double sin_x = sin(rx);
	double cos_x = cos(rx);
	double sin_y = sin(ry);
	double cos_y = cos(ry);
	double sin_z = sin(rz);
	double cos_z = cos(rz);

	rotate[0][0] = cos_z * cos_y;
	rotate[0][1] = -sin_z * cos_x + cos_z * sin_y * sin_x;
	rotate[0][2] = sin_z * sin_x + cos_z * sin_y * cos_x;

	rotate[1][0] = sin_z * cos_y;
	rotate[1][1] = cos_z * cos_x + sin_z * sin_y * sin_x;
	rotate[1][2] = -sin_x * cos_z + cos_x * sin_y * sin_z;

	rotate[2][0] = -sin_y;
	rotate[2][1] = sin_x * cos_y;
	rotate[2][2] = cos_x * cos_y;

	Matrix44 gltrans;

	gltrans[0] = rotate[0][0];
	gltrans[1] = rotate[1][0];
	gltrans[2] = rotate[2][0];
	gltrans[3] = 0.0f;
	gltrans[4] = rotate[0][1];
	gltrans[5] = rotate[1][1];
	gltrans[6] = rotate[2][1];
	gltrans[7] = 0.0f;
	gltrans[8] = rotate[0][2];
	gltrans[9] = rotate[1][2];
	gltrans[10] = rotate[2][2];
	gltrans[11] = 0.0f;
	gltrans[12] = tx;
	gltrans[13] = ty;
	gltrans[14] = tz;
	gltrans[15] = 1.0f;

	return gltrans;
}

HapticTranslator::HapticTranslator() {
	trans2Device.Identity();
	trans2World.Identity();
}

int HapticDevice::GetOperatorCnt() {
	return m_deviceName.size();
}

void HapticDevice::InitHapticDevice() {
	forceInfoFirst.open("forceFirst.csv");
	forceInfoFirst << "fx, fy, fz, collidedNum, qhx, qhy,qhz" << std::endl;

	// 设置不同设备的操作手数量
	m_operatorList = std::vector<OperatorInfo>(m_deviceName.size()); // 等价于 resize

	for (int i = 0; i < m_operatorList.size(); i++) {
		m_operatorList[i].translator.configFile = m_deviceCfgFile[i];
		m_operatorList[i].translator.LoadTransFormFile();
	}

	// 打开 dll
	m_deviceDriverDLL = LoadLibrary(L"PhantomIoLib42.dll");
	if (m_deviceDriverDLL == NULL) {
		UDError(std::string("Load PhantomIoLib42.dll"));
		return;
	}

	// 通用接口
	//创建服务循环
	createServoLoop = (int(*)()) GetProcAddress(m_deviceDriverDLL, "createServoLoop");
	//停止服务循环
	stopServoLoop = (int(*)())GetProcAddress(m_deviceDriverDLL, "stopServoLoop");
	//销毁服务循环
	destroyServoLoop = (int(*)())GetProcAddress(m_deviceDriverDLL, "destroyServoLoop");
	//初始化设备
	init_phantom = (int(*)(const char* configname))GetProcAddress(m_deviceDriverDLL, "init_phantom");
	//移除设备
	disable_phantom = (int(*)(unsigned int index))GetProcAddress(m_deviceDriverDLL, "disable_phantom");
	//开始服务循环
	startServoLoop = (int(*)(int(_stdcall * fntServo)(void*), void* lpParam))GetProcAddress(m_deviceDriverDLL, "startServoLoop");
	//更新数据
	update_phantom = (int(*)(unsigned int index))GetProcAddress(m_deviceDriverDLL, "update_phantom");
	//回零
	update_calibration = (int(*)(unsigned int index))GetProcAddress(m_deviceDriverDLL, "update_calibration");
	//使能力
	enable_phantom_forces = (int(*)(unsigned int index))GetProcAddress(m_deviceDriverDLL, "enable_phantom_forces");
	//撤销力
	disable_phantom_forces = (int(*)(unsigned int index))GetProcAddress(m_deviceDriverDLL, "disable_phantom_forces");
	//施加力
	send_phantom_force = (int(*)(unsigned int index, const float forces[3]))GetProcAddress(m_deviceDriverDLL, "send_phantom_force");
	//变化矩阵
	get_stylus_matrix = (int(*)(unsigned int index, float(*matrix)[16]))GetProcAddress(m_deviceDriverDLL, "get_stylus_matrix");

	// 检查
	if (createServoLoop == nullptr) UDError("无法获取createServoLoop入口");
	if (stopServoLoop == nullptr) UDError("无法获取stopServoLoop入口");
	if (destroyServoLoop == nullptr) UDError("无法获取destroyServoLoop入口");
	if (init_phantom == nullptr) UDError("无法获取init_phantom入口");
	if (disable_phantom == nullptr) UDError("无法获取disable_phantom入口");
	if (startServoLoop == nullptr) UDError("无法获取startServoLoop入口");
	if (update_phantom == nullptr) UDError("无法获取update_phantom入口");
	if (enable_phantom_forces == nullptr) UDError("无法获取enable_phantom_forces入口");
	if (disable_phantom_forces == nullptr) UDError("无法获取disable_phantom_forces入口");
	if (send_phantom_force == nullptr) UDError("无法获取send_phantom_force入口");
	if (get_stylus_matrix == nullptr) UDError("无法获取get_stylus_matrix入口");
	get_stylus_switch = (int(*)(unsigned int index, int no))GetProcAddress(m_deviceDriverDLL, "get_stylus_switch");
	if (get_stylus_switch == nullptr) UDError("无法获取get_stylus_switch入口");

	// 初始化

	for (int i = 0; i < m_operatorList.size(); i++) {
		int id = m_operatorList[i].deviceHandle = init_phantom(m_deviceName[i].c_str());
		if (id < 0) {
			UDError( "无法打开 " + m_deviceName[i]);
		} else {
			enable_phantom_forces(id);
			g_hapticPlayBackMode = false;
		}
	}
	if (!g_hapticPlayBackMode) {
		int ret = createServoLoop();
		UDLog("createServoLoop ret = " + std::to_string(ret));
		ret = startServoLoop(SetHapticState, this);
		UDLog("SetHapticState ret = " + std::to_string(ret));
	}
	else {
		runServoLoop( (LPVOID) this );
	}
	UDLog("InitHapticDevice 成功");
}

void HapticDevice::StopHapticDevice() {
	if (g_hapticPlayBackMode) {
		m_stopFlag = true;
		Sleep(50);
		shutDownServoLoop();
		return;
	}
	if (m_deviceDriverDLL != NULL && !m_stopFlag) {
		m_stopFlag = true;
		Sleep(50);
		if (stopServoLoop && m_hapticThreadRunFlag) {
			stopServoLoop();
			UDLog("stopServoLoop调用");
		}
		if (destroyServoLoop) {
			destroyServoLoop();
			UDLog("destroyServoLoop调用");
		}

		for (int i = 0; i < m_operatorList.size(); i++) {
			int id = m_operatorList[i].deviceHandle;
			if (disable_phantom_forces) {
				disable_phantom_forces(id);
				UDLog("disable_phantom_forces调用");
			}
			if (disable_phantom) { 
				disable_phantom(id);
				UDLog("disable_phantom调用");
			}
		}
		UDLog("力反馈设备停止");
	}
}

// 力反馈循环
int _stdcall SetHapticState(void* pParam) {
	QueryPerformanceFrequency(&hapFreq);
	QueryPerformanceCounter(&hap_t1);

	HapticDevice* hapticDevice = (HapticDevice*)pParam;

	if (!hapticDevice->m_hapticThreadRunFlag)
		UDLog("力反馈线程开始运行");
	hapticDevice->m_hapticThreadRunFlag = true;
	if (hapticDevice->m_stopFlag) {
		UDLog("力反馈线程准备退出");
		Sleep(10);
		return -1;
	}
	
	
	Solver& solver = g_simManager->m_softHapticSolver;
	float adsorbForceLen = 0;
	int collidedNum = 0;
	cudaSetDevice(g_ID_SoftHaptic);
	// 更新表面弹簧顶点的指导向量
	runUpdateSurfaceDDir();	
	///////////////////////////////////////////////////////////////////////////////
	solver.HapticCollideTriVert();	
	solver.MergeCollisionInfoTriVert();

	cudaMemcpy(&collidedNum, hapticCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	float adsorbForce[30];
	memcpy(adsorbForce, solver.graspForce.data(), solver.graspForce.size() * sizeof(float));

	for (int index = 0; index < solver.m_operatorTransList.size(); index++) {
		auto& operatorTrans = solver.m_operatorTransList[index];

		// 根据上述的碰撞信息更新虚拟位姿
		solver.UpdateQG_6DOF(
			operatorTrans.qh,
			operatorTrans.graspL,
			operatorTrans.collidedNum,
			operatorTrans.collidedVertDepth.data(),
			operatorTrans.collidedVertPos.data(),
			operatorTrans.collidedVertNonPenetrationDir.data(),
			operatorTrans.qg,
			operatorTrans.dirg,
			operatorTrans.buffer_graph);
	}

	////////////////////////////////////////////////////////////////////
	cudaMemcpy(&collidedNum, hapticCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
	solver.colNumInfo << collidedNum << endl;

	/********************表面网格仿真********************/
	// 计算初始状态
	runcalculateSTMU(solver.m_dampingForTriVert, solver.sv_dt);

	//迭代求解
	int omega = 1.0;
	float rho = 0.9992f;
	for (int i = 0; i < 3; i++) {
		// 清空弹簧力、碰撞力
		
		cudaMemset(springVertForce_d, 0.0f, springVertNum_d * 3 * sizeof(float));
		cudaMemset(springDiag_d, 0.0f, springVertNum_d * 3 * sizeof(float));
		cudaMemset(springVertCollisionDiag_d, 0.0f, springVertNum_d * 3 * sizeof(float));
		cudaMemset(springVertCollisionForce_d, 0.0f, springVertNum_d * 3 * sizeof(float));
		printCudaError("runClearCollisionMU");

		runCollisionResponse(200.0f);
		// 计算弹簧力-表面网格
		runcalculateIFMU(300.0f);

		cudaMemcpy(tetVertPos_forTri, solver.tetVertPos_bridge.data(),
			solver.tetVertPos_bridge.size() * sizeof(float), cudaMemcpyHostToDevice);
		printCudaError("cudaMemcpy tetVertPos_forTri");
		//表面网格顶点与四面体顶点之间的Rest-Pos约束
		runcalculateRestPosForceWithTetPos_vCAG(solver.tv_minStiffnessSV2TV, solver.tv_maxStiffnessSV2TV);

		// 更新位置
		omega = 4 / (4 - rho * rho * omega);
		runcalculatePosMU(omega, solver.sv_dt);
	}

	//更新速度-表面网格
	runcalculateVMU(solver.sv_dt);

	for (int i = 0; i < hapticDevice->m_operatorList.size(); i++) {
		HapticDevice::OperatorInfo& tool = hapticDevice->m_operatorList[i];
		int id = tool.deviceHandle;
		// 读取设备数据
		hapticDevice->update_phantom(id);
		hapticDevice->get_stylus_matrix(id, &tool.M);
		memcpy(tool.buffer, tool.M, 16 * sizeof(float));
		// mm单位改为cm单位
		tool.translator.Trans2World(tool.buffer, 0.1);

		// 移动工具的初始位置，用于Phantom工具的肝脏场景
		tool.buffer[14] -= 12;

		tool.bt0 = hapticDevice->get_stylus_switch(id, 0);
		tool.bt1 = hapticDevice->get_stylus_switch(id, 1);
		if (tool.bt0) {
			tool.anglei += AngleDelta;
			tool.anglei = min(tool.anglei, MaxAngleI);
		}
		else {
			tool.anglei -= AngleDelta;
			tool.anglei = max(tool.anglei, 0);
		}
		tool.buffer[16] = 1 - float(tool.anglei) / MaxAngleI;
		tool.buffer[17] = 0;
		tool.buffer[18] = tool.bt1;
		// 设置 solver 中的 qh
		UpdateQH(i, tool.buffer);

		// 计算反馈力
		float force[6] = { 0 };
		ComputeOperatorForce(i, force);
		tool.translator.TransForce2Device(force);
		float grabForce[3] = { adsorbForce[i * 3 + 0], adsorbForce[i * 3 + 1], adsorbForce[i * 3 + 2] };
		float grabForceLen = grabForce[0] * grabForce[0] + grabForce[1] * grabForce[1] + grabForce[2] * grabForce[2];
		if (grabForceLen > 1e-5)
		{
			force[0] = grabForce[0];
			force[1] = grabForce[1];
			force[2] = grabForce[2];
		}
		else
		{
			for (int k = 0; k < 6; k++)
				solver.m_operatorOutput6DForceList[i][k] = force[k];
		}
		if (id >= 0)
			hapticDevice->send_phantom_force(id, force);
	}
	QueryPerformanceCounter(&hap_t2);
	solver.m_hapFrameTime = (hap_t2.QuadPart - hap_t1.QuadPart) / (double)hapFreq.QuadPart;
	return 0;
}

DWORD RunHapticState(LPVOID lpvThreadParam) {
	HapticDevice* hapticDevice = (HapticDevice*)lpvThreadParam;

	if (!hapticDevice->m_hapticThreadRunFlag)
		UDLog("力反馈线程开始运行");
	hapticDevice->m_hapticThreadRunFlag = true;
	cudaSetDevice(g_ID_SoftHaptic);
	while (!hapticDevice->m_stopFlag) {
		QueryPerformanceFrequency(&hapFreq);
		QueryPerformanceCounter(&hap_t1);

		Solver& solver = g_simManager->m_softHapticSolver;
		float adsorbForceLen = 0;
		int collidedNum = 0;
		// 更新表面弹簧顶点的指导向量
		runUpdateSurfaceDDir();
		///////////////////////////////////////////////////////////////////////////////
		solver.HapticCollideTriVert();
		solver.MergeCollisionInfoTriVert();
		cudaMemcpy(&collidedNum, hapticCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
		float adsorbForce[30];
		memcpy(adsorbForce, solver.graspForce.data(), solver.graspForce.size() * sizeof(float));
		for (int index = 0; index < solver.m_operatorTransList.size(); index++) {
			auto& operatorTrans = solver.m_operatorTransList[index];

			// 根据上述的碰撞信息更新虚拟位姿
			solver.UpdateQG_6DOF(
				operatorTrans.qh,
				operatorTrans.graspL,
				operatorTrans.collidedNum,
				operatorTrans.collidedVertDepth.data(),
				operatorTrans.collidedVertPos.data(),
				operatorTrans.collidedVertNonPenetrationDir.data(),
				operatorTrans.qg,
				operatorTrans.dirg,
				operatorTrans.buffer_graph);
		}

		////////////////////////////////////////////////////////////////////
		cudaMemcpy(&collidedNum, hapticCollisionNum_d, sizeof(int), cudaMemcpyDeviceToHost);
		solver.colNumInfo << collidedNum << endl;
		/********************表面网格仿真********************/
		// 计算初始状态
		runcalculateSTMU(solver.m_dampingForTriVert, solver.sv_dt);

		//迭代求解
		int omega = 1.0;
		float rho = 0.9992f;
		for (int i = 0; i < 3; i++) {
			// 清空弹簧力、碰撞力
			cudaMemset(springVertForce_d, 0.0f, springVertNum_d * 3 * sizeof(float));
			cudaMemset(springDiag_d, 0.0f, springVertNum_d * 3 * sizeof(float));
			cudaMemset(springVertCollisionDiag_d, 0.0f, springVertNum_d * 3 * sizeof(float));
			cudaMemset(springVertCollisionForce_d, 0.0f, springVertNum_d * 3 * sizeof(float));
			printCudaError("runClearCollisionMU");
			runCollisionResponse(200.0f);
			// 计算弹簧力-表面网格
			runcalculateIFMU(300.0f);
			//表面网格顶点与四面体顶点之间的Rest-Pos约束
			runcalculateRestPosForceWithTetPos_vCAG(solver.tv_minStiffnessSV2TV, solver.tv_maxStiffnessSV2TV);

			// 更新位置
			omega = 4 / (4 - rho * rho * omega);
			runcalculatePosMU(omega, solver.sv_dt);
		}
		//更新速度-表面网格
		runcalculateVMU(solver.sv_dt);

		for (int i = 0; i < hapticDevice->m_operatorList.size(); i++) {
			HapticDevice::OperatorInfo& tool = hapticDevice->m_operatorList[i];
			HapticDevice::OperatorInfo& toolMotion = tool.toolMotion[tool.motionCount];
			// 设置 solver 中的 qh
			UpdateQH(i, tool.toolMotion[tool.motionCount].buffer);
			tool.motionCount++;
			tool.motionCount = tool.motionCount % tool.toolMotion.size();
			// 计算反馈力
			float force[6] = { 0 };
			ComputeOperatorForce(i, force);
			tool.translator.TransForce2Device(force);
			float grabForce[3] = { adsorbForce[i*3+0], adsorbForce[i*3+1], adsorbForce[i*3+2] };
			float grabForceLen = grabForce[0] * grabForce[0] + grabForce[1] * grabForce[1] + grabForce[2] * grabForce[2];
			if (grabForceLen > 1e-5)
			{
				force[0] = grabForce[0];
				force[1] = grabForce[1];
				force[2] = grabForce[2];
			}
			else
			{
				for (int k = 0; k < 6; k++)
					solver.m_operatorOutput6DForceList[i][k] = force[k];
			}

		}

		QueryPerformanceCounter(&hap_t2);
		solver.m_hapFrameTime = (hap_t2.QuadPart - hap_t1.QuadPart) / (double)hapFreq.QuadPart;

	}
	UDLog("力反馈线程准备退出");
	Sleep(10);
	return -1;
}