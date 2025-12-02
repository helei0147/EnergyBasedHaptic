/*
 * physically based rendering
 * copyright (c) 2017-2018 michał siejak
 *
 * OpenGL 4.5 renderer.
 */

#define NOMINMAX
#include "opengl.hpp"
#include <stdexcept>
#include <memory>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>


#include <GLFW/glfw3.h>

#include "mesh.hpp"
#include "image.hpp"
#include "glutils.hpp"

#include "ImguiHelper.hpp"


GLFWwindow* Renderer::initialize(int width, int height, int maxSamples)
{
	max_sample_ = maxSamples;
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
#if _DEBUG
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

	glfwWindowHint(GLFW_DEPTH_BITS, 0);
	glfwWindowHint(GLFW_STENCIL_BITS, 0);
	glfwWindowHint(GLFW_SAMPLES, 0);

	GLFWwindow* window = glfwCreateWindow(width, height, "Renderer", nullptr, nullptr);
	if(!window) {
		throw std::runtime_error("Failed to create OpenGL context");
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(-1);

	if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		throw std::runtime_error("Failed to initialize OpenGL extensions loader");
	}
	
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &m_capabilities.maxAnisotropy);

#if _DEBUG
	glDebugMessageCallback(Renderer::logMessage, nullptr);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
#endif

	GLint maxSupportedSamples;
	glGetIntegerv(GL_MAX_SAMPLES, &maxSupportedSamples);

	const int samples = glm::min(maxSamples, maxSupportedSamples);
	m_framebuffer = createFrameBuffer(width, height, samples, { GL_RGBA16F }, GL_DEPTH_COMPONENT32);
	if(samples > 0) {
		m_resolveFramebuffer = createFrameBuffer(width, height, 0, { GL_RGBA16F }, GL_NONE);
	}
	else {
		m_resolveFramebuffer = m_framebuffer;
	}

	m_gbuffer = createFrameBuffer(width, height, 0, { GL_RGBA8,
		GL_RGBA16F,
		GL_RGBA32F 
		}, GL_DEPTH_COMPONENT32);


	m_postbuffer = createFrameBuffer(width, height, 0, { GL_RGBA8 }, GL_NONE);

	m_aoblurbuffer = createFrameBuffer(width, height, 0, { GL_RGBA8 }, GL_NONE);

	std::printf("OpenGL 4.5 Renderer [%s]\n", glGetString(GL_RENDERER));


	m_sceneobject = std::make_shared<Scene>();
	

	return window;
}

void Renderer::shutdown()
{
	if(m_framebuffer.id != m_resolveFramebuffer.id) {
		deleteFrameBuffer(m_resolveFramebuffer);
	}
	deleteFrameBuffer(m_framebuffer);

}

	
GLuint Renderer::compileShader(const std::string& filename, GLenum type)
{
	const std::string src = File::readText(filename);
	if(src.empty()) {
		throw std::runtime_error("Cannot read shader source file: " + filename);
	}
	const GLchar* srcBufferPtr = src.c_str();

	std::printf("Compiling GLSL shader: %s\n", filename.c_str());

	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &srcBufferPtr, nullptr);
	glCompileShader(shader);

	GLint status;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
	if(status != GL_TRUE) {
		GLsizei infoLogSize;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogSize);
		std::unique_ptr<GLchar[]> infoLog(new GLchar[infoLogSize]);
		glGetShaderInfoLog(shader, infoLogSize, nullptr, infoLog.get());
		throw std::runtime_error(std::string("Shader compilation failed: ") + filename + "\n" + infoLog.get());
	}
	return shader;
}
	
GLuint Renderer::linkProgram(std::initializer_list<GLuint> shaders)
{
	GLuint program = glCreateProgram();

	for(GLuint shader : shaders) {
		glAttachShader(program, shader);
	}
	glLinkProgram(program);
	for(GLuint shader : shaders) {
		glDetachShader(program, shader);
		glDeleteShader(shader);
	}

	GLint status;
	glGetProgramiv(program, GL_LINK_STATUS, &status);
	if(status == GL_TRUE) {
		glValidateProgram(program);
		glGetProgramiv(program, GL_VALIDATE_STATUS, &status);
	}
	if(status != GL_TRUE) {
		GLsizei infoLogSize;
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLogSize);
		std::unique_ptr<GLchar[]> infoLog(new GLchar[infoLogSize]);
		glGetProgramInfoLog(program, infoLogSize, nullptr, infoLog.get());
		throw std::runtime_error(std::string("Program link failed\n") + infoLog.get());
	}
	return program;
}
	
Texture Renderer::createTexture(GLenum target, int width, int height, GLenum internalformat, int levels)
{
	Texture texture;
	texture.width  = width;
	texture.height = height;
	texture.levels = (levels > 0) ? levels : UtilityFunction::numMipmapLevels(width, height);
	
	glCreateTextures(target, 1, &texture.id);
	glTextureStorage2D(texture.id, texture.levels, internalformat, width, height);
	glTextureParameteri(texture.id, GL_TEXTURE_MIN_FILTER, texture.levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
	glTextureParameteri(texture.id, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//glTextureParameterf(texture.id, GL_TEXTURE_MAX_ANISOTROPY_EXT, m_capabilities.maxAnisotropy);
	return texture;
}
	
Texture Renderer::createTexture(const std::shared_ptr<class ::Image>& image, GLenum format, GLenum internalformat, int levels)
{
	Texture texture = createTexture(GL_TEXTURE_2D, image->width(), image->height(), internalformat, levels);
	if(image->isHDR()) {
		glTextureSubImage2D(texture.id, 0, 0, 0, texture.width, texture.height, format, GL_FLOAT, image->pixels<float>());
	}
	else {
		glTextureSubImage2D(texture.id, 0, 0, 0, texture.width, texture.height, format, GL_UNSIGNED_BYTE, image->pixels<unsigned char>());
	}

	if(texture.levels > 1) {
		glGenerateTextureMipmap(texture.id);
	}
	return texture;
}

Texture Renderer::createTexture(GLenum target, unsigned char* data, int width, int height, GLenum format, GLenum internalformat, int levels)
{
	Texture texture;
	texture.width = width;
	texture.height = height;
	texture.levels = (levels > 0) ? levels : UtilityFunction::numMipmapLevels(width, height);

	glCreateTextures(target, 1, &texture.id);
	glTextureStorage2D(texture.id, texture.levels, internalformat, width, height);
	glTextureParameteri(texture.id, GL_TEXTURE_MIN_FILTER, texture.levels > 1 ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
	glTextureParameteri(texture.id, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//glTextureParameterf(texture.id, GL_TEXTURE_MAX_ANISOTROPY_EXT, m_capabilities.maxAnisotropy);
	glTexImage2D(
		GL_TEXTURE_2D, 0, internalformat, width, height, 0,
		format,GL_UNSIGNED_BYTE, data);

	return texture;
}
	
void Renderer::deleteTexture(Texture& texture)
{
	glDeleteTextures(1, &texture.id);
	std::memset(&texture, 0, sizeof(Texture));
}

FrameBuffer Renderer::createFrameBuffer(int width, int height, int samples, std::vector<GLenum> colorFormats, GLenum depthstencilFormat)
{
	FrameBuffer fb;
	fb.width   = width;
	fb.height  = height;
	fb.samples = samples;

	glCreateFramebuffers(1, &fb.id);

	std::vector<GLenum> drawbuffers;
	int idx = 0;
	for (auto colorFormat : colorFormats)
	{
		if (colorFormat != GL_NONE) {
			if (samples > 0) {
				glCreateRenderbuffers(1, &fb.colorTarget[idx]);
				glNamedRenderbufferStorageMultisample(fb.colorTarget[idx], samples, colorFormat, width, height);
				glNamedFramebufferRenderbuffer(fb.id, GL_COLOR_ATTACHMENT0 + idx, GL_RENDERBUFFER, fb.colorTarget[idx]);
			}
			else
			{
				glCreateTextures(GL_TEXTURE_2D, 1, &fb.colorTarget[idx]);
				glTextureStorage2D(fb.colorTarget[idx], 1, colorFormat, width, height);
				glNamedFramebufferTexture(fb.id, GL_COLOR_ATTACHMENT0 + idx, fb.colorTarget[idx], 0);

				//auto format = GL_RGBA;
				//auto datatype = GL_UNSIGNED_BYTE;
				//if (colorFormat == GL_R32F)
				//{
				//	format = GL_RED;
				//	datatype = GL_FLOAT;
				//}
				//if (colorFormat == GL_RGBA16F)
				//{
				//	datatype = GL_FLOAT;
				//}

				//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, format, datatype, NULL);
				//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
				//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
				//glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0+idx, GL_TEXTURE_2D, fb.colorTarget[idx], 0);

			}

			drawbuffers.emplace_back(GL_COLOR_ATTACHMENT0 + idx);
		}
		idx++;
	}
	if(depthstencilFormat != GL_NONE) {
		glCreateRenderbuffers(1, &fb.depthStencilTarget);
		if(samples > 0) {
			glNamedRenderbufferStorageMultisample(fb.depthStencilTarget, samples, depthstencilFormat, width, height);
		}
		else {
			glNamedRenderbufferStorage(fb.depthStencilTarget, depthstencilFormat, width, height);
		}
		glNamedFramebufferRenderbuffer(fb.id, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, fb.depthStencilTarget);

		//drawbuffers.emplace_back(GL_DEPTH_ATTACHMENT);
	}

	glNamedFramebufferDrawBuffers(fb.id, drawbuffers.size(), &drawbuffers[0]);

	GLenum status = glCheckNamedFramebufferStatus(fb.id, GL_DRAW_FRAMEBUFFER);
	if(status != GL_FRAMEBUFFER_COMPLETE) {
		throw std::runtime_error("Framebuffer completeness check failed: " + std::to_string(status));
	}

	return fb;
}

void Renderer::resolveFramebuffer(const FrameBuffer& srcfb, const FrameBuffer& dstfb)
{
	if(srcfb.id == dstfb.id) {
		return;
	}

	std::vector<GLenum> attachments;
	if(srcfb.colorTarget) {
		attachments.push_back(GL_COLOR_ATTACHMENT0);
	}
	if(srcfb.depthStencilTarget) {
		attachments.push_back(GL_DEPTH_STENCIL_ATTACHMENT);
	}
	assert(attachments.size() > 0);

	glBlitNamedFramebuffer(srcfb.id, dstfb.id, 0, 0, srcfb.width, srcfb.height, 0, 0, dstfb.width, dstfb.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
	glInvalidateNamedFramebufferData(srcfb.id, (GLsizei)attachments.size(), &attachments[0]);
}
	
void Renderer::deleteFrameBuffer(FrameBuffer& fb)
{
	if(fb.id) {
		glDeleteFramebuffers(1, &fb.id);
	}
	for(int i=0;i<3;i++)
		if(fb.colorTarget[i]) {
			if(fb.samples == 0) {
				glDeleteTextures(1, &fb.colorTarget[i]);
			}
			else {
				glDeleteRenderbuffers(1, &fb.colorTarget[i]);
			}
		}

	if(fb.depthStencilTarget) {
		glDeleteRenderbuffers(1, &fb.depthStencilTarget);
	}
	std::memset(&fb, 0, sizeof(FrameBuffer));
}

MeshBuffer Renderer::createMeshBuffer(const std::shared_ptr<class ::MeshData>& mesh, uint32_t buf_flag)
{
	MeshBuffer buffer;
	buffer.numElements = static_cast<GLuint>(mesh->faces().size()) * 3;

	const size_t vertexDataSize = mesh->vertices().size() * sizeof(::MeshData::Vertex);
	const size_t indexDataSize = mesh->faces().size() * sizeof(::MeshData::Face);
	buffer.vertNum = vertexDataSize;
	buffer.triNum = indexDataSize;

	glCreateBuffers(1, &buffer.vbo);
	glNamedBufferData(buffer.vbo, vertexDataSize, reinterpret_cast<const void*>(&mesh->vertices()[0]), buf_flag);
	glCreateBuffers(1, &buffer.ibo);
	glNamedBufferData(buffer.ibo, indexDataSize, reinterpret_cast<const void*>(&mesh->faces()[0]), buf_flag);

	glCreateVertexArrays(1, &buffer.vao);
	glVertexArrayElementBuffer(buffer.vao, buffer.ibo);
	for (int i = 0; i < ::MeshData::NumAttributes; ++i) {
		glVertexArrayVertexBuffer(buffer.vao, i, buffer.vbo, i * sizeof(glm::vec3), sizeof(::MeshData::Vertex));
		glEnableVertexArrayAttrib(buffer.vao, i);
		glVertexArrayAttribFormat(buffer.vao, i, 3, GL_FLOAT, GL_FALSE, 0);
		glVertexArrayAttribBinding(buffer.vao, i, i);
	}
	return buffer;
}

MeshBuffer Renderer::createMeshBuffer()
{
	MeshBuffer buffer;
	buffer.numElements = buffer.triNum/3;
	std::vector<char> verts(buffer.vertNum, 0);
	std::vector<char> tri(buffer.triNum, 0);

	glCreateBuffers(1, &buffer.vbo);
	glNamedBufferData(buffer.vbo, buffer.vertNum, reinterpret_cast<const void*>(&verts[0]), GL_DYNAMIC_DRAW);
	glCreateBuffers(1, &buffer.ibo);
	glNamedBufferData(buffer.ibo, buffer.triNum, reinterpret_cast<const void*>(&tri[0]), GL_DYNAMIC_DRAW);

	glCreateVertexArrays(1, &buffer.vao);
	glVertexArrayElementBuffer(buffer.vao, buffer.ibo);
	for (int i = 0; i < ::MeshData::NumAttributes; ++i) {
		glVertexArrayVertexBuffer(buffer.vao, i, buffer.vbo, i * sizeof(glm::vec3), sizeof(::MeshData::Vertex));
		glEnableVertexArrayAttrib(buffer.vao, i);
		glVertexArrayAttribFormat(buffer.vao, i, 3, GL_FLOAT, GL_FALSE, 0);
		glVertexArrayAttribBinding(buffer.vao, i, i);
	}
	return buffer;
}

void Renderer::deleteMeshBuffer(MeshBuffer& buffer)
{
	if(buffer.vao) {
		glDeleteVertexArrays(1, &buffer.vao);
	}
	if(buffer.vbo) {
		glDeleteBuffers(1, &buffer.vbo);
	}
	if(buffer.ibo) {
		glDeleteBuffers(1, &buffer.ibo);
	}
	std::memset(&buffer, 0, sizeof(MeshBuffer));
}
	
GLuint Renderer::createUniformBuffer(const void* data, size_t size)
{
	GLuint ubo;
	glCreateBuffers(1, &ubo);
	glNamedBufferStorage(ubo, size, data, GL_DYNAMIC_STORAGE_BIT);
	return ubo;
}

#if _DEBUG
void Renderer::logMessage(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar* message, const void* userParam)
{
	if(severity != GL_DEBUG_SEVERITY_NOTIFICATION) {
		std::fprintf(stderr, "GL: %s\n", message);
	}
}
#endif

#include "pystring.h"
#include "FileHelper.hpp"

uint32_t loadProgram(std::string glsl_filepath, bool forcenew)
{
	static std::map<std::string, uint32_t> shader_map;

	if (shader_map.count(glsl_filepath) && !forcenew)
		return shader_map[glsl_filepath];

	printf("loading %s\n", glsl_filepath.c_str());
	//vertex:
	//fragment:
	//geometry:
	auto lines = Utility::FileHelper::readLines(glsl_filepath);
	std::string vertex_str, fragment_str, geometry_str;
	enum LineMode
	{
		Vertex,
		Fragment,
		Geometry,
		None
	};

	LineMode mode = LineMode::None;
	for (int i = 0; i < lines.size(); i++)
	{
		//check
		auto line = pystring::strip(lines[i]);
		if (pystring::startswith(line, "vertex:"))
		{
			mode = LineMode::Vertex;
			continue;
		}
		else if (pystring::startswith(line, "fragment:"))
		{
			mode = LineMode::Fragment;
			continue;
		}
		else if (pystring::startswith(line, "geometry:"))
		{
			mode = LineMode::Geometry;
			continue;
		}

		if (line.size() == 0) continue;

		line.push_back('\n');
		//append
		switch (mode)
		{

		case LineMode::Vertex:
			vertex_str += line;
			break;
		case LineMode::Fragment:
			fragment_str += line;
			break;
		case LineMode::Geometry:
			geometry_str += line;
			break;
		default:
		case LineMode::None:
			break;
		}
	}

	auto program_id = loadProgram(vertex_str, fragment_str, geometry_str);

	if (program_id > 0)
		shader_map[glsl_filepath] = program_id;

	return program_id;
}

uint32_t loadProgramFile(std::string vertpath, std::string fgpath)
{
	static std::map<std::string, uint32_t> shader_map;
	auto glsl_filepath = vertpath + "_" + fgpath;
	if (shader_map.count(glsl_filepath))
		return shader_map[glsl_filepath];

	auto vs_str = File::readText(vertpath);
	auto fg_str = File::readText(fgpath);
	std::string geo_str = "";

	auto program_id = loadProgram(vs_str, fg_str, geo_str);

	if (program_id > 0)
		shader_map[glsl_filepath] = program_id;

	return program_id;
}

uint32_t loadProgram(std::string vs_str, std::string fg_str, std::string gs_str)
{
#pragma region createShader
	auto createShader = [](GLenum shaderType, const std::string code)->unsigned int
	{
		GLuint shader = glCreateShader(shaderType);
		if (shader == 0)
		{
			std::cout << "Failed to create shader.\n";
			return 0;
		}
		const char* codes[] = { code.c_str() };
		GLint codesLen[] = { (GLint)code.size() };
		glShaderSource(shader, 1, codes, codesLen);
		glCompileShader(shader);

		GLint infoLength;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLength);
		if (infoLength != 0)
		{
			std::vector<char> info;
			info.reserve(infoLength + 1);
			info.resize(infoLength);

			GLsizei len;
			glGetShaderInfoLog(shader, infoLength, &len, &info[0]);
			if (info[infoLength - 1] != '\0')
			{
				info.push_back('\0');
			}

			std::cout << &info[0] << "\n";
		}

		GLint compileStatus;
		glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);
		if (compileStatus != GL_TRUE)
		{
			glDeleteShader(shader);
			std::cout << "Failed to compile shader.\n";
			return 0;
		}

		return shader;
	};
#pragma endregion

	printf("vertex shader...\n");
	auto vs = createShader(GL_VERTEX_SHADER, vs_str);

	printf("fragment shader...\n");
	auto fs = createShader(GL_FRAGMENT_SHADER, fg_str);
	unsigned int gs = 0;
	if (gs_str.size())
	{

		printf("geometry shader...\n");
		gs = createShader(GL_GEOMETRY_SHADER, gs_str);
	}
	if (vs == 0 || fs == 0)
	{
		if (vs != 0) { glDeleteShader(vs); }
		else printf("error in vs part");
		if (fs != 0) { glDeleteShader(fs); }
		else printf("error in fs part");

		return 0;
	}

	GLuint prog = glCreateProgram();
	if (prog == 0)
	{
		glDeleteShader(vs);
		glDeleteShader(fs);
		std::cout << "[SHADER]Failed to create program.\n";
		system("pause");
		return 0;
	}
	glAttachShader(prog, vs);
	glAttachShader(prog, fs);
	if (gs)
		glAttachShader(prog, gs);
	glLinkProgram(prog);

	GLint infoLength;
	glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &infoLength);
	if (infoLength != 0)
	{
		std::vector<char> info;
		info.reserve(infoLength + 1);
		info.resize(infoLength);

		GLsizei len;
		glGetProgramInfoLog(prog, infoLength, &len, &info[0]);
		if (info[infoLength - 1] != '\0')
		{
			info.push_back('\0');
		}

		std::cout << &info[0] << "\n";
	}

	GLint linkStatus;
	glGetProgramiv(prog, GL_LINK_STATUS, &linkStatus);
	if (linkStatus != GL_TRUE)
	{
		glDeleteShader(vs);
		glDeleteShader(fs);
		std::cout << "Failed to link shader.\n";
		system("pause");
		return 0;
	}

	glDeleteShader(vs);
	glDeleteShader(fs);

	return prog;

}