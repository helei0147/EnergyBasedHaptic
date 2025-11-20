//////////所有和初始化相关的加到这个函数中
//////////包括文件读取，系统矩阵初始化
#include <Windows.h>
#include "Solver.h"
#include"common.h"
using namespace std;


void FileStream(FILE* fid, int* v, std::string msg, FILEMODE m_fileMode) {
	if (FILEMODE::R == m_fileMode) {
		if (!ReadStream(fid, (char*)v, sizeof(int), msg))
			return;
		UDLog("读取变量成功：" + msg);
	}
	else {
		if (!WriteStream(fid, (char*)v, sizeof(int), msg))
			return;
		UDLog("写入变量成功：" + msg);
	}
}

void FileStream(FILE* fid, char* v, std::string msg, FILEMODE m_fileMode) {
	if (FILEMODE::R == m_fileMode) {
		if (!ReadStream(fid, v, sizeof(char), msg))
			return;
		UDLog("读取变量成功：" + msg);
	}
	else {
		if (!WriteStream(fid, v, sizeof(char), msg))
			return;
		UDLog("写入变量成功：" + msg);
	}
}

void FileStream(FILE* fid, float* v, std::string msg, FILEMODE m_fileMode) {
	if (FILEMODE::R == m_fileMode) {
		if (!ReadStream(fid, (char*)v, sizeof(float), msg))
			return;
		UDLog("读取变量成功：" + msg);
	}
	else {
		if (!WriteStream(fid, (char*)v, sizeof(float), msg))
			return;
		UDLog("写入变量成功：" + msg);
	}
}

void FileStream(FILE* fid, unsigned int* v, std::string msg, FILEMODE m_fileMode) {
	if (FILEMODE::R == m_fileMode) {
		if (!ReadStream(fid, (char*)v, sizeof(unsigned int), msg))
			return;
		UDLog("读取变量成功：" + msg);
	}
	else {
		if (!WriteStream(fid, (char*)v, sizeof(unsigned int), msg))
			return;
		UDLog("写入变量成功：" + msg);
	}
}

void FileStream(FILE* fid, std::vector<int>& vec, std::string msg, FILEMODE m_fileMode) {
	int  num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num <= 0)
			return;
		vec.resize(num);
		if (!ReadStream(fid, (char*)&vec[0], num * sizeof(int), msg))
			return;
		UDLog("读取数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		if (num == 0) return;
		if (!WriteStream(fid, (char*)&vec[0], num * sizeof(int), msg))
			return;
		UDLog("写入数组成功：" + msg);
	}
}

void FileStream(FILE* fid, std::vector<char>& vec, std::string msg, FILEMODE m_fileMode) {
	int num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		if (num == 0) return;
		if (!ReadStream(fid, (char*)&vec[0], num * sizeof(char), msg))
			return;
		UDLog("读取数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		if (num == 0) return;
		if (!WriteStream(fid, (char*)&vec[0], num * sizeof(char), msg))
			return;
		UDLog("写入数组成功：" + msg);
	}
}

void FileStream(FILE* fid, std::vector<float>& vec, std::string msg, FILEMODE m_fileMode) {
	uint32_t num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		if (num == 0) return;
		if (!ReadStream(fid, (char*)&vec[0], num * sizeof(float), msg))
			return;
		UDLog("读取数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		if (num == 0) return;
		if (!WriteStream(fid, (char*)&vec[0], num * sizeof(float), msg))
			return;
		UDLog("写入数组成功：" + msg);
	}
}

void FileStream(FILE* fid, std::vector<unsigned int>& vec, std::string msg, FILEMODE m_fileMode) {
	int  num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		if (num == 0) return;
		if (!ReadStream(fid, (char*)&vec[0], num * sizeof(unsigned int), msg))
			return;
		UDLog("读取数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		if (num == 0) return;
		if (!WriteStream(fid, (char*)&vec[0], num * sizeof(unsigned int), msg))
			return;
		UDLog("写入数组成功：" + msg);
	}
}

void FileStreamLength(FILE* fid, std::vector<int>& vec, std::string msg, FILEMODE m_fileMode) {
	int  num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		if (num == 0) return;
		vec.resize(num);
		UDLog("初始化数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		UDLog("写入数组长度成功：" + msg);
	}
}

void FileStreamLength(FILE* fid, std::vector<char>& vec, std::string msg, FILEMODE m_fileMode) {
	int num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		if (num == 0) return;
		UDLog("初始化数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		if (num == 0) return;
		UDLog("写入数组长度成功：" + msg);
	}
}

void FileStreamLength(FILE* fid, std::vector<float>& vec, std::string msg, FILEMODE m_fileMode) {
	uint32_t num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		UDLog("初始化数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		UDLog("写入数组长度成功：" + msg);
	}
}

void FileStreamLength(FILE* fid, std::vector<unsigned int>& vec, std::string msg, FILEMODE m_fileMode) {
	int  num;
	if (FILEMODE::R == m_fileMode) {
		num = ReadNum(fid, msg);
		if (num < 0)
			return;
		vec.resize(num);
		UDLog("初始化数组成功：" + msg);
	}
	else {
		num = vec.size();
		if (!WriteNum(fid, num, msg))
			return;
		UDLog("写入数组长度成功：" + msg);
	}
}

bool WriteStream(FILE* fid, const char* buffer, uint32_t length, string msg) {

	int ret = fwrite(buffer, sizeof(char), length, fid);
	if (ret != length) {
		UDError("写入缓存错误: " + to_string(ret- length) + " " + msg);
		return false;
	}
	return true;
}

bool ReadStream(FILE* fid, char* buffer, uint32_t length, string msg) {

	int ret = fread(buffer, sizeof(char), length, fid);
	if (ret != length) {
		UDError("读取缓存错误:  " + to_string(ret- length) + " " + msg);
		return false;
	}
	return true;

}

bool WriteNum(FILE* fid, uint32_t num, string msg) {
	int ret = fwrite(&num, sizeof(uint32_t), 1, fid);
	if (num == 0)
		UDError("Write num zero: " + msg);

	if (ret != 1) {
		UDError("Write num error: " + to_string(ret) + " " + msg);
		return false;
	}
	return true;
}

uint32_t ReadNum(FILE* fid, string msg) {
	int num = -1;
	int ret = fread(&num, sizeof(int), 1, fid);
	if (ret != 1 || num < 0) {
		UDError("read num error: " + to_string(num) + " " + msg);
		return -100;
	}
	return num;
}

void ReadNum(FILE* fid, int& num, string msg) {
	int temp;
	int ret = fread(&temp, sizeof(int), 1, fid);
	num = temp;
	if (ret != 1 || temp <= 0) {
		UDError("read num error: " + to_string(temp) + " " + msg);
		num = -1;
	}
}

void Solver::BinStream(FILE* fid) {
	int ret;


#pragma region
	int bufferi[1024];

	if (FILEMODE::R == m_fileMode) {
		ret = fread(&bufferi[0], sizeof(int), 1024, fid);
		if (ret != 1024) {
			fclose(fid);
			UDError("Read  Num error!" + to_string(ret));
			return;
		}
		memcpy(&m_volumnSum, &bufferi[0], sizeof(int));

	}
	else {
		memcpy(&bufferi[0], &m_volumnSum, sizeof(int));
		ret = fwrite(&bufferi[0], sizeof(int), 1024, fid);
		if (ret != 1024) {
			fclose(fid);
			UDError("Write Num error!" + to_string(ret));
			return;
		}

	}
#pragma endregion  各种变量


	//std::vector<int> m_tetIndex;//四面体索引	
	FileStream(fid, m_tetIndex, "m_tetIndex", m_fileMode);
	//std::vector<float> m_tetStiffness;//四面体的软硬
	FileStream(fid, m_tetStiffness, "m_tetStiffness", m_fileMode);
	//std::vector<float> m_tetVertPos;//顶点位置	
	FileStream(fid, m_tetVertPos, "m_tetVertPos", m_fileMode);
	//std::vector<float> m_tetVertFixed;//是否固定	
	FileStream(fid, m_tetVertFixed, "m_tetVertFixed", m_fileMode);
	//std::vector<float> m_tetVertIsAbleToBurn;
	FileStream(fid, m_tetVertIsAbleToBurn, "m_tetVertIsAbleToBurn", m_fileMode);
	//std::vector<float> m_tetVertIsBurned;//是否固定	
	FileStream(fid, m_tetVertIsBurned, "m_tetVertIsBurned", m_fileMode);
	//
	FileStream(fid, m_tetVertIsSmooth, "m_tetVertIsSmooth", m_fileMode);
	//
	FileStream(fid, m_tetVertActive, "m_tetVertActive", m_fileMode);
	//
	FileStream(fid, m_tetVertDensity, "m_tetVertDensity", m_fileMode);
	//std::vector<float> m_tetVertMass;//顶点质量
	FileStream(fid, m_tetVertMass, "m_tetVertMass", m_fileMode);
	//std::vector<float> m_tetVertRestStiffness;//每个点restpos的约束
	FileStream(fid, m_tetVertRestStiffness, "m_tetVertRestStiffness", m_fileMode);
	//std::vector<float> m_tetVertfromSpringStiffness;////三角形顶点对四面体的约束刚度
	FileStream(fid, m_tetVertfromSpringStiffness, "m_tetVertfromSpringStiffness", m_fileMode);



	//std::vector<char> m_tetActive;//是否处于有效状态
	FileStream(fid, m_tetActive, "m_tetActive", m_fileMode);
	//
	FileStream(fid, m_tetIsSmooth, "m_tetIsSmooth", m_fileMode);
	//
	FileStream(fid, m_tetIsBurnning, "m_tetIsBurnning", m_fileMode);
	//
	FileStream(fid, m_tetBurnState, "m_tetBurnState", m_fileMode);
	//std::vector<float> m_tetInvD3x3;//四面体矩阵的逆//用于计算变形梯度
	FileStream(fid, m_tetInvD3x3, "m_tetInvD3x3", m_fileMode);
	//std::vector<float> m_tetInvD3x4;//用于计算对角阵
	FileStream(fid, m_tetInvD3x4, "m_tetInvD3x4", m_fileMode);
	//std::vector<float> m_tetVolume;//四面体体积，长度：tetNum
	FileStream(fid, m_tetVolume, "m_tetVolume", m_fileMode);
	//std::vector<float> m_tetVolumeDiag;//线性系统的对角矩阵的ACT*AC部分, 长度：tetNum
	FileStream(fid, m_tetVolumeDiag, "m_tetVolumeDiag", m_fileMode);
	//std::vector<int> m_mapTetVertIndexToSpringVertIndex;//四面体对应表面顶点，一对一
	FileStream(fid, m_mapTetVertIndexToSpringVertIndex, "m_mapTetVertIndexToSpringVertIndex", m_fileMode);
	//std::vector<int> m_mapSpringVertIndexToTetVertSetIndex;//表面顶点对四面体顶点，一对二
	FileStream(fid, m_mapSpringVertIndexToTetVertSetIndex, "m_triIndex", m_fileMode);
	//std::vector<int> m_mapTetIndexToSpringIndex; //一对四，一个四面体内部最多有四个弹簧是激活的
	FileStream(fid, m_mapTetIndexToSpringIndex, "m_mapTetIndexToSpringIndex", m_fileMode);
	//std::vector<int> m_mapTetIndexToSubtetIndex;// 粗四面体的6个子四面体的索引映射，长度 : 6 * tetNum
	FileStream(fid, m_mapTetIndexToSubtetIndex, "m_mapTetIndexToSubtetIndex", m_fileMode);
	//std::vector<int> m_mapTetIndexToSubtetVertIndex;// 粗四面体内部的6个中点和1个重心的索引映射，长度 : 7 * tetNum
	FileStream(fid, m_mapTetIndexToSubtetVertIndex, "m_mapTetIndexToSubtetVertIndex", m_fileMode);
	//std::vector<int> m_mapTetIndexToTetEdgeIndex;大四面体到大四面体四条边的索引映射，一对四
	FileStream(fid, m_mapTetIndexToTetEdgeIndex, "m_mapTetIndexToTetEdgeIndex", m_fileMode);
	//std::vector<int> m_mapTetInsideFaceTriIndexToMidVertIndex; // 一对三，原来的三角形的 边e(0,1)的中点 -> 第0个位置， e(0,2) -> 1, e(1,2) -> 2
	FileStream(fid, m_mapTetInsideFaceTriIndexToMidVertIndex, "m_mapTetInsideFaceTriIndexToMidVertIndex", m_fileMode);
	//std::vector<int> m_mapTetInsideFaceTriIndexToSpringIndex; //一对一，一个内表面最多有一个弹簧是激活的
	FileStream(fid, m_mapTetInsideFaceTriIndexToSpringIndex, "m_mapTetInsideFaceTriIndexToSpringIndex", m_fileMode);
	//std::vector<int> m_mapTetInsideFaceTriIndexToTetEdgeIndex;
	FileStream(fid, m_mapTetInsideFaceTriIndexToTetEdgeIndex, "m_mapTetInsideFaceTriIndexToTetEdgeIndex", m_fileMode);
	//std::vector<int> m_tetOutsideEdgeIndex;	//四面体外表面的边, 每个索引有三个单元： 0是这条边对应的中点索引， 1是这条边的一个顶点索引，2是这条边的另一个顶点索引
	FileStream(fid, m_tetOutsideEdgeIndex, "m_tetOutsideEdgeIndex", m_fileMode);
	FileStream(fid, m_tetOutsideEdgeActive, "m_tetOutsideEdgeActive", m_fileMode);
	FileStream(fid, m_tetOutsideEdgeBurnState, "m_tetOutsideEdgeBurnState", m_fileMode);
	//std::vector<int> m_mapTetOutsideEdgeIndexToSpringIndex; //一对二，四面体外表面一条边对应两个弹簧
	FileStream(fid, m_mapTetOutsideEdgeIndexToSpringIndex, "m_mapTetOutsideEdgeIndexToSpringIndex", m_fileMode);
	//std::vector<int> m_mapTetOutsideEdgeIndexToTetEdgeIndex;
	FileStream(fid, m_mapTetOutsideEdgeIndexToTetEdgeIndex, "m_mapTetOutsideEdgeIndexToTetEdgeIndex", m_fileMode);
	//std::vector<int> m_mapTetOutsideFaceTriIndexToMidVertIndex;
	FileStream(fid, m_mapTetOutsideFaceTriIndexToMidVertIndex, "m_mapTetOutsideFaceTriIndexToMidVertIndex", m_fileMode);
	//std::vector<int> m_mapTetOutsideFaceTriIndexToSpringIndex; //一对三，一个外表面内部有3个弹簧
	FileStream(fid, m_mapTetOutsideFaceTriIndexToSpringIndex, "m_mapTetOutsideFaceTriIndexToSpringIndex", m_fileMode);
	//std::vector<int> m_mapTetOutsideFaceTriIndexToTetEdgeIndex;
	FileStream(fid, m_mapTetOutsideFaceTriIndexToTetEdgeIndex, "m_mapTetOutsideFaceTriIndexToTetEdgeIndex", m_fileMode);
	//std::vector<int> m_onSurfaceSpringVertIndices; // 在表面的弹簧顶点下标
	//FileStream(fid, m_onSurfaceSpringVertIndices, "m_onSurfaceSpringVertIndices", m_fileMode);
	//std::vector<int> m_onSurfaceTetVertIndices; // 在表面的四面体顶点下标
	//FileStream(fid, m_onSurfaceTetVertIndices, "m_onSurfaceTetVertIndices", m_fileMode);


	//std::vector<unsigned int> m_edgeIndex;// 弹簧索引
	FileStream(fid, m_springIndex, "m_springIndex", m_fileMode);
	FileStream(fid, m_springActive, "m_springActive", m_fileMode);
	//std::vector<float> m_edgeStiffness;// 弹簧刚度
	FileStream(fid, m_springStiffness, "m_springStiffness", m_fileMode);
	//std::vector<float> m_edgeVertPos;//弹簧顶点
	FileStream(fid, m_springVertPos, "m_springVertPos", m_fileMode);
	//std::vector<float> m_edgeVertFixed;
	FileStream(fid, m_springVertFixed, "m_springVertFixed", m_fileMode);
	//std::vector<float> m_edgeVertMass;
	FileStream(fid, m_springVertMass, "m_springVertMass", m_fileMode);
	//std::vector<float> m_edgeOrgLength; // 弹簧原长
	FileStream(fid, m_springOrgLength, "m_springOrgLength", m_fileMode);
	//std::vector<float> m_springDiag;
	FileStream(fid, m_springDiag, "m_springDiag", m_fileMode);
	//std::vector<float> m_springVertfromTetStiffness;
	FileStream(fid, m_springVertfromTetStiffness, " m_springVertfromTetStiffness", m_fileMode);
	FileStream(fid, m_springVertActive, "m_springVertActive", m_fileMode);
	FileStream(fid, m_springVertIsBurned, "m_springVertIsBurned", m_fileMode);
	FileStream(fid, m_springVertInterpolationW, "m_springVertInterpolationW", m_fileMode);

	FileStream(fid, m_tetEdgeIndex, "m_tetEdgeIndex", m_fileMode);
	FileStream(fid, m_tetEdgeIsBurned, "m_tetEdgeIsBurned", m_fileMode);
	FileStream(fid, m_tetOutsideFaceTriIndex, "m_tetOutsideFaceTriIndex", m_fileMode);
	FileStream(fid, m_tetOutsideFaceActive, "m_tetOutsideFaceActive", m_fileMode);
	FileStream(fid, m_tetOutsideFaceBurnState, "m_tetOutsideFaceBurnState", m_fileMode);
	FileStream(fid, m_tetInsideFaceTriIndex, "m_tetInsideFaceTriIndex", m_fileMode);
	FileStream(fid, m_tetInsideFaceActive, "m_tetInsideFaceActive", m_fileMode);
	FileStream(fid, m_tetInsideFaceBurnState, "m_tetInsideFaceBurnState", m_fileMode);

	FileStream(fid, m_renderTetVertPos, "m_renderTetVertPos", m_fileMode);
	FileStream(fid, m_renderSpringVertPos, "m_renderSpringVertPos", m_fileMode);
	FileStream(fid, m_tetVertPosForCPUCut, "m_tetVertPosForCPUCut", m_fileMode);
	FileStream(fid, m_tetCenterPointNeedUpdate, "m_tetCenterPointNeedUpdate", m_fileMode);
	FileStream(fid, m_tetEdgeMidPointNeedUpdate, "m_tetEdgeMidPointNeedUpdate", m_fileMode);
	FileStream(fid, m_tetEdgeVertInterpolationWSrcDst, "m_tetEdgeVertInterpolationWSrcDst", m_fileMode);
	FileStream(fid, m_tetCenterVertIndexEdgeVertIndex4, "m_tetCenterVertIndexEdgeVertIndex4", m_fileMode);

	FileStream(fid, m_renderTetVertColors, "m_renderTetVertColors", m_fileMode);
}

void Solver::ReadFromBin(FILE* fid) {
	m_fileMode = R;
	BinStream(fid);
	BuildTetSolverMapping();
	BuildTriSolverMapping();
	BuildOCTree();
	BuildTriOCTree();
	PreMalloc();
	CopyToGPU();
#ifdef GLDebug
	RebuildTetActiveVertIdx();
	RebuildTriActiveVertIdx();
#endif
}

void Solver::WriteToBin(FILE* fid) {
	m_fileMode = W;
	BinStream(fid);
#ifdef GLDebug
	RebuildTetActiveVertIdx();
	RebuildTriActiveVertIdx();
#endif
}