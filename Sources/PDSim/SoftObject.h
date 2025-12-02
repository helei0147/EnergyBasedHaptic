#pragma once
#include <chrono>
#include <string>
#include <vector>
#include "Solver.h"

struct SoftObject {
	int m_glId = -1;//表面在opengl场景中的id
	float m_tetStiffness = 1000.0f;//四面体的软硬
	float m_tetVertfromSpringStiffness = 1000.0f;//三角形顶点对四面体的约束刚度

	std::string m_name;//创建物体用的名称
	std::string m_objFile;
	std::string m_tetFile;	
	float m_color[4];
	std::vector<int> m_tet;//四面体索引
	std::vector<float>m_tetVerts;

	std::vector<int> m_triIdx;///原始三角形
	std::vector<int> m_triUVIdx;///原始UV三角形	
	std::vector<float> m_triUV;//纹理坐标
	std::vector<float> m_triVerts;//顶点


	int m_triNumSurf = 0;
	std::vector<int> m_triIdxSurf;///细分三角形
	std::vector<int> m_triUVIdxSurf;//细分UV三角形	
	std::vector<float> m_triUVSurf;//细分纹理坐标
	std::vector<float> m_triVertsSurf;//细分顶点
	std::vector<float> m_triNormsSurf;//三角形法向量
	std::vector<float> m_triNormsSurfCount;//计算法向量用来统计的
	std::vector<int> m_surfVertSolverMapping;//表面顶点与求解器顶点的映射关系
	std::vector<int> m_renderVertSolverMapping;//每个渲染顶点与GPU中顶点的对应关系
	std::vector<int> m_renderNormalMapping;//每个渲染顶点与实际顶点的关系
	std::vector<unsigned int> m_renderTriIdx;///渲染三角形
	std::vector<float> m_renderTriUV;//渲染三角形纹理

	int m_renderTriVertsNum;
	std::vector<float> m_renderTriVertNormUV;
	void ComputeNormals(const std::vector<float>& tetVertPos);
	void ReadFromFile();
	void BuildTriVertSolverMapping(const std::vector<float>& tetVertPos);
	void ReBuildRenderMesh();
	void SurfaceSubdivision();
	void BuildOpenGLMesh9(const std::vector<float>& tetVertPos);
};


