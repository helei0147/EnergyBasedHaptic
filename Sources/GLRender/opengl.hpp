#pragma once

#include <glad/glad.h>
#include <string>
#include <map>
#include <memory>
#include "ArcballHelper.h"
#include <glm/mat4x4.hpp>


struct GLFWwindow;

struct ViewSettings
{

	Utility::ArcballCamera* camera = nullptr;
};

struct SceneSettings
{
	float pitch = 0.0f;
	float yaw = 0.0f;

	static const int NumLights = 3;
	struct Light {
		glm::vec3 direction;
		glm::vec3 radiance;
		bool enabled = false;
	} lights[NumLights];
};


struct Texture
{
	Texture() : id(0) {}
	GLuint id;
	int width, height;
	int levels;
};


struct Material
{
	std::string m_dataFolder;
	std::string m_vertShaderFile;
	std::string m_fragShaderFile;
	std::string m_shaderFile;
	std::map<std::string, Texture> m_textures;
	std::map<std::string,uint32_t> m_programs;

	glm::vec4 m_diffColor=glm::vec4(1,1,1,1);
	bool m_wireframe = false;	
	int m_useTexture = 0;
	int m_useSSS = 0;
	glm::vec4 m_material = glm::vec4(1, 1, 0.2, 1);

	virtual void apply(Utility::ArcballCamera* camera)=0;
	virtual uint32_t getDefaultProgram() = 0;
};

struct MaterialPhong : public Material
{
	MaterialPhong();

	glm::vec3 light_position = glm::vec3(3, 3, 3);

	void apply(Utility::ArcballCamera* camera) override;
	uint32_t getDefaultProgram() override;
};

struct MaterialPBR : public Material
{
	MaterialPBR(std::string dataFolder);

	void apply(Utility::ArcballCamera* camera) override;
	uint32_t getDefaultProgram() override;

	void setTextures(const std::string& abeldo,
		const std::string& normal,
		const std::string& metallic,
		const std::string& roughness);

	void setEnvLut(uint32_t env, uint32_t irmap, uint32_t lut_map)
	{
		env_texture_id = env;
		irmap_texture_id = irmap;
		sp_BRDF_LUT_id = lut_map;
	};

	//textures
	uint32_t env_texture_id = 0;
	uint32_t irmap_texture_id = 0;
	uint32_t sp_BRDF_LUT_id = 0;
};

struct MeshBuffer
{
	MeshBuffer() : vbo(0), ibo(0), vao(0) {}
	GLuint vbo, ibo, vao;
	GLuint numElements = 0;
	int vertNum = 10240*9*4;
	int triNum = 65535*3*4;
};

struct FrameBuffer
{
	FrameBuffer() : id(0), depthStencilTarget(0) {
		colorTarget[0] = 0;
		colorTarget[1] = 0;
		colorTarget[2] = 0;
	}
	GLuint id;
	GLuint colorTarget[3];
	GLuint depthStencilTarget;
	int width, height;
	int samples;
};

class Mesh
{
public:
	MeshBuffer m_buffer;
	std::shared_ptr<Material> m_material;

	void updateMesh(int eleNum, int triNum, unsigned int* triIdx, int vertNum, float* vert);
	void updateMeshConst( int triNum, unsigned int* triIdx, int vertNum, float* vert);
	void draw(Utility::ArcballCamera* cam,int new_program=0);
	void updateTransform(float* t);

	glm::mat4 m_transform = glm::mat4(1.0f);
	std::string m_name;
};

struct TransformUB
{
	glm::mat4 mvp;
	glm::mat4 sky_mvp;
	glm::vec4 eyepos;
};

struct ShadingUB
{
	struct {
		glm::vec4 direction;
		glm::vec4 radiance;
	} lights[SceneSettings::NumLights];
	glm::vec4 eyePosition;
};


class Scene
{
public:
	std::string m_shaderFolder = "../../shader/";
	std::string m_hdrfile = "clear_sky_dome_4k.hdr";
	MeshBuffer m_skybox;

	GLuint m_emptyVAO;

	GLuint m_tonemapProgram;
	GLuint m_skyboxProgram;
	Texture m_envTexture;
	Texture m_irmapTexture;
	Texture m_spBRDF_LUT;


	GLuint m_transformUB;
	GLuint m_shadingUB;

	GLuint m_gbufferProgram;
	GLuint m_gtaoProgram;
	GLuint m_aoblurProgram;


	std::vector<std::shared_ptr<Mesh>> m_meshes;

	std::vector<glm::vec3> noise;

	std::vector<glm::vec3> ssaosamples;

	glm::vec3 m_blurpara = glm::vec3(1,1.5,0.9);

	bool use_sky = false;

	glm::vec3 sky_color = glm::vec3(1, 1, 1);

	bool enable_ao = true;
	bool enable_sss = true;

	float m_exposure = 3.0f;
	float m_gamma = 2.2f;
	float m_pureWhite = 1.0f;
	float m_tonemapPara[4];//exposure  gamma pureWhite
	float m_ssaoPara[4];//raidus bias

public:

	~Scene();

	int getObjectId(std::string name);

	void add(std::shared_ptr<Mesh> mesh);

	void initialize();

	void setEnvForPBR();

	void draw(Utility::ArcballCamera* cam,int new_prgram=0);

	int createPBRObj(std::string name, const std::string& abeldo, const std::string& normal, const std::string& metallic, const std::string& roughness);

	int createPBRObj(std::string name, float r, float g, float b, float metallic, float roughness);
};


class Renderer
{
public:
	
	GLFWwindow* initialize(int width, int height, int maxSamples);
	void shutdown();
	void render(GLFWwindow* window, const ViewSettings& view, const SceneSettings& scene);
	void resize(int w, int h);
	void doUI();
public:
	static GLuint compileShader(const std::string& filename, GLenum type);
	static GLuint linkProgram(std::initializer_list<GLuint> shaders);

	static Texture createTexture(GLenum target, int width, int height, GLenum internalformat, int levels=0);
	static Texture createTexture(const std::shared_ptr<class Image>& image, GLenum format, GLenum internalformat, int levels=0);
	static Texture createTexture(GLenum target, unsigned char* data, int width, int height, GLenum format, GLenum internalformat, int levels = 0);
	static void deleteTexture(Texture& texture);

	static FrameBuffer createFrameBuffer(int width, int height, int samples, std::vector<GLenum> colorFormats, GLenum depthstencilFormat);
	static void resolveFramebuffer(const FrameBuffer& srcfb, const FrameBuffer& dstfb);
	static void deleteFrameBuffer(FrameBuffer& fb);

	static MeshBuffer createMeshBuffer(const std::shared_ptr<class MeshData>& mesh, uint32_t buf_flag = GL_DYNAMIC_DRAW);
	static MeshBuffer createMeshBuffer();
	static void deleteMeshBuffer(MeshBuffer& buffer);

	static GLuint createUniformBuffer(const void* data, size_t size);
	template<typename T> static GLuint createUniformBuffer(const T* data=nullptr)
	{
		return createUniformBuffer(data, sizeof(T));
	}

#if _DEBUG
	static void logMessage(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam);
#endif

	struct {
		float maxAnisotropy = 1.0f;
	} m_capabilities;

	int max_sample_ = 1;

	FrameBuffer m_gbuffer;
	FrameBuffer m_postbuffer;
	FrameBuffer m_aoblurbuffer;

	FrameBuffer m_framebuffer;
	FrameBuffer m_resolveFramebuffer;
	

	std::shared_ptr< Scene >  m_sceneobject;
	bool m_doUI = true;
};
uint32_t loadProgram(std::string glslpath, bool forcenew = false);
uint32_t loadProgramFile(std::string vertpath, std::string fgpath);
uint32_t loadProgram(std::string vertstr, std::string fragstr, std::string geostr);