#include "opengl.hpp"
#include "ImguiHelper.hpp"
#include <iostream>

ImguiHelper* ImguiHelper::instance = nullptr;

void ImguiHelper::init(GLFWwindow* window)
{

	auto g_ctx = ImGui::CreateContext();
	ImGui::SetCurrentContext(g_ctx);


	auto& io = ImGui::GetIO();
	io.Fonts->AddFontFromFileTTF("c:/windows/fonts/simhei.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesChineseSimplifiedCommon());

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
	io.ConfigWindowsMoveFromTitleBarOnly = true;

	if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
	{
		printf("ImGui_ImplGlfw_InitForOpenGL failed\n");
	};
	if (!ImGui_ImplOpenGL3_Init("#version 330"))
	{
		printf("ImGui_ImplOpenGL3_Init failed\n");
	};

	enable = true;
}


void ImguiHelper::newFrame()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	ImGui::DockSpaceOverViewport(nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

}

bool ImguiHelper::isHover()
{
	return enable && ImGui::GetIO().WantCaptureMouse;
}

void ImguiHelper::display(const glm::mat4& view_matrix, const glm::mat4& projection_matrix)
{
	glm::mat4 model_matrix = glm::mat4(1.0f);
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}


ImguiHelper* ImguiHelper::Instance()
{
    if (!instance)
        instance = new ImguiHelper();

    return instance;
}

void ImguiHelper::Free()
{
    if (instance)
    {
        delete instance;
        instance = nullptr;
    }
}
