#pragma once
#include <Windows.h>
#include <map>
#include "mat44.h"
#include <fstream>
#include <iostream>
#define TransSize 64
static const float AngleDelta = 1;
static const int MaxAngleI = 90;

struct HapticTranslator {
	////实际情况下，力反馈设备是斜着摆放
	////对力反馈设备进行移动，从而能够更加贴近器官
	std::string configFile;
	Matrix44 trans2Device;
	Matrix44 trans2World;
	Matrix44 deviceTrans;
	void LoadTransFormFile();
	void Trans2World(float* r, float scale  = 1.0f);
	void TransForce2Device(float* f);
	void TransForce2Device(double* f);
	Matrix44 GenMatrixform3dxMax(float rx, float ry, float rz, float tx, float ty, float tz);
	HapticTranslator();
};


class HapticDevice {
public: // 通用
	// 存储设备状态信息的通用结构体
	struct OperatorInfo { 
		std::string deviceName = "";
		float buffer[TransSize];
		float rotate_center_world[3];
		unsigned short maxValue = 0;
		unsigned short minValue = INT16_MAX;
		HapticTranslator translator;
		float M[16];
		int deviceHandle = -1;
		int bt0, bt1;
		int anglei = 0;
		std::vector<OperatorInfo> toolMotion;
		int motionCount = 0;
	};
	std::vector<OperatorInfo> m_operatorList; // 多个设备
	std::ofstream forceInfoFirst;
	int GetOperatorCnt();
	// 初始化设备，以及开启力反馈循环
	void InitHapticDevice();
	// 停止力反馈循环
	void StopHapticDevice();

public: 
	std::vector<std::string> m_deviceName;
	std::vector<std::string> m_deviceCfgFile;
	std::vector<std::string> m_deviceMotionFile;

	bool m_stopFlag = false;
	bool m_hapticThreadRunFlag = false;

	HMODULE  m_deviceDriverDLL; // dll 设备通用接口

	int(*createServoLoop)();
	int(*stopServoLoop)();
	int(*destroyServoLoop)();
	int(*init_phantom)(const char* configname);
	int(*disable_phantom)(unsigned int index);
	int(*startServoLoop)(int(_stdcall* fntServo)(void*), void* lpParam);
	int(*get_stylus_matrix)(unsigned int index, float(*matrix)[16]);
	int(*update_phantom)(unsigned int index);
	int(*enable_phantom_forces)(unsigned int index);
	int(*disable_phantom_forces)(unsigned int index);
	int(*send_phantom_force)(unsigned int index, const float forces[3]);
	int(*is_phantom_forces_enabled)(unsigned int index);
	int(*get_phantom_pos)(unsigned int index, float pos[3]);
	int(*update_calibration)(unsigned int index);
	int(*get_phantom_joint_angles)(unsigned int index, float angles[6]);
	int(*phantom_status)(unsigned int index);
	int(*command_motor_dac_values)(unsigned int index, long MotorDACValues[6]);
	int(*get_phantom_info)(unsigned int index, void* pPhantomInfo, unsigned int size);
	int(*get_encoder_values)(unsigned int index, long* a2);
	int(*get_stylus_switch)(unsigned int index, int no);
};