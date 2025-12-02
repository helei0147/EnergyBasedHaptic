#ifndef _IMGUI_HELPER_H_
#define _IMGUI_HELPER_H_

#include <imgui_impl_opengl3.h>
#include <imgui_impl_glfw.h>

#include <imgui.h>
#include <imgui_internal.h>

#include <glm/glm.hpp>

class Mesh;

class ImguiHelper
{
	static ImguiHelper* instance;
	bool enable = false;
public:

	void init(GLFWwindow* window);
	void newFrame();
	bool isHover();
	void display(const glm::mat4& view_matrix, const glm::mat4& projection_matrix);
	static ImguiHelper* Instance();
	static void Free();
};

#endif