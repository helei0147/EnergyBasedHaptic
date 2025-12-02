#pragma once

#ifdef FLLAPARO_EXPORTS
#define FLLAPARO_API __declspec(dllexport)
#else
#define FLLAPARO_API __declspec(dllimport)
#endif
namespace fllaparo {

	extern "C"
	{
		//打开设备为设备序列号
		//输入 sn：设备序列号
		//返回 设备ID
		FLLAPARO_API int openDevice(DWORD sn);
		//暂停设备
		FLLAPARO_API void PauseDevice(int id = 0);
		//恢复设备
		FLLAPARO_API void ResumeDevice(int id = 0);
		//关闭设备
		FLLAPARO_API void closeDevice();
		//开始服务循环
		FLLAPARO_API int startServoLoop(int(_stdcall* func)(void*), void* lpParam);
		//停止服务循环
		FLLAPARO_API int stopServoLoop();
		//判断服务循环是否在执行
		FLLAPARO_API bool isServoLoopRunning();
		//取服务循环刷新率
		FLLAPARO_API double getServoLoopRate(int id = 0);
		//设备力使能函数 true 使能，false 不使能
		FLLAPARO_API void enableForces(bool en = true, int id = 0);
		//判断设备是否已力使能
		FLLAPARO_API bool isForcesEnabled(int id = 0);
		//获得设备序列号
		FLLAPARO_API int getSerialNumber(int id = 0);
		//设备状态（尚未实现）
		FLLAPARO_API int deviceStatus(int id = 0);
		//发送零力矩和力
		FLLAPARO_API void sendZeroForce(int id = 0);
		//发送手术钳尖端受力
		FLLAPARO_API void sendForce(double force[], int id = 0);
		//重置各编码器值为32768
		FLLAPARO_API void zeroEncoders(int id = 0);
		//取各轴编码器值
		FLLAPARO_API void getEncoders(long encs[], int id = 0);
		//取各轴编码器速度
		FLLAPARO_API void getEncVel(double evels[], int id = 0);
		//取各开关状态  
		//bit7:button1  bit5:button2
		FLLAPARO_API void getSwitch(byte& swts, int id = 0);
		//取关节值
		//joints: 关节数组
		FLLAPARO_API void getJoints(double joints[], int id = 0);
		//取手术钳尖端坐标系  
		FLLAPARO_API void getEndFrame(double EndFrame[], int id = 0);
		//取手术钳尖端位置、旋转角度、钳口角度
		FLLAPARO_API void getPositionAndAngle(double pos[], double& rotAngle, double& Angle, int id = 0);
		//取手术钳尖端姿态
		FLLAPARO_API void getPose(double rot[], int id = 0);

	}
}