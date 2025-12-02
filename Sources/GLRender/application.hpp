/*
 * Physically Based Rendering
 * Copyright (c) 2017-2018 Michał Siejak
 */

#pragma once

#include <memory>
#include "opengl.hpp"
#include "ArcballHelper.h"

class Application
{
public:
	Application();
	~Application();


	void InitRender(std::string& hdrfile, std::string& shaderfolder);

	int Render();

	void ShutDown();

	int CreatePBRObj(std::string name, const std::string& abeldo, const std::string& normal,
		const std::string& metallic, const std::string& roughness);

	int CreatePBRObj(std::string name, float r, float g, float b, float metallic, float roughness);

	void UpdateMesh(int idx, int eleNum, int triNum, unsigned int* triIdx, int vertNum, float* vert);

	void UpdateMeshConst(int idx, int triNum, unsigned int* triIdx, int vertNum, float* vert);

	void UpdateTransform(int idx, float* t);

	void SetHDRFile(std::string name);

	void SetShaderFolder(std::string name);

	void SetSSAOParas(float* p);

	static void SaveCame();
	static void LoadCame(char* name);

	Utility::ArcballCamera* getCamera()
	{
		return m_camera;
	};

	Renderer* getRenderer()
	{
		return m_renderer;
	}

	static Application* Instance();

private:
	static void framebufferSizeCallback(GLFWwindow* window, int w, int h);
	static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos);
	static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void mouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
	static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	

	GLFWwindow* m_window;
	double m_prevCursorX;
	double m_prevCursorY;

	ViewSettings m_viewSettings;
	SceneSettings m_sceneSettings;

	enum class InputMode
	{
		None,
		RotatingView,
		RotatingScene,
	};
	InputMode m_mode;

	Utility::ArcballCamera* m_camera = nullptr;
	Renderer* m_renderer = nullptr;

	
};
