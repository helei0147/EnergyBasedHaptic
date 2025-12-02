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

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <GLFW/glfw3.h>
#include "mesh.hpp"
#include "image.hpp"
#include "glutils.hpp"
#include "ImguiHelper.hpp"


std::shared_ptr<MeshData> createSkyBox();


void Mesh::updateTransform(float* t) {
	memcpy(&m_transform[0], t, 16 * sizeof(float));
}

void Mesh::updateMesh(int eleNum, int triNum, unsigned int* triIdx, int vertNum, float* verts) {
	if (triNum > 0) {
		m_buffer.numElements = eleNum;
		m_buffer.triNum = triNum;
		glNamedBufferData(m_buffer.ibo,  triNum, reinterpret_cast<const void*>(triIdx), GL_DYNAMIC_DRAW);
	}
	if (vertNum > 0) {
		m_buffer.vertNum = vertNum;
		glNamedBufferData(m_buffer.vbo, vertNum, reinterpret_cast<const void*>(verts), GL_DYNAMIC_DRAW);
	}
}

void Mesh::updateMeshConst( int triNum, unsigned int* triIdx, int vertNum, float* vert) {
	if (triNum > 0) {
		assert(triNum == m_buffer.triNum);
		glNamedBufferSubData(m_buffer.ibo, 0, triNum, reinterpret_cast<const void*>(triIdx));
	}
	if (vertNum > 0) {
		assert(vertNum == m_buffer.vertNum);
		glNamedBufferSubData(m_buffer.vbo, 0, vertNum, reinterpret_cast<const void*>(vert));
	}
}


void Mesh::draw(Utility::ArcballCamera* cam, int new_program)
{
	if (!new_program)
	{
		m_material->apply(cam);
		GLint model_location = glGetUniformLocation(m_material->getDefaultProgram(), "model");
		if (model_location != -1) {
			glm::mat4 identity = m_transform;
			glUniformMatrix4fv(model_location, 1, false, &identity[0][0]);
		}
	}
	else
	{
		GLint model_location = glGetUniformLocation(new_program, "model");
		if (model_location != -1) {
			glUniformMatrix4fv(model_location, 1, false, &m_transform[0][0]);
		}
	}
	
	glBindVertexArray(m_buffer.vao);

	glDrawElements(GL_TRIANGLES, m_buffer.numElements, GL_UNSIGNED_INT, 0);

	if (!new_program && m_material->m_wireframe)
	{
		auto program = m_material->getDefaultProgram();
		auto color_location = glGetUniformLocation(program, "color");
		auto default_color = m_material->m_diffColor;
		glm::vec3 color(1, 0, 0);
		if (color_location != -1)
			glUniform3fv(color_location, 1, &color.x);
		glPolygonOffset(1, 1);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glDrawElements(GL_TRIANGLES, m_buffer.numElements, GL_UNSIGNED_INT, 0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		if (color_location != -1)
		{
			glUniform3fv(color_location, 1, &default_color.x);
		}
	}

}

void Scene::add(std::shared_ptr<Mesh> mesh)
{
	m_meshes.emplace_back(mesh);
}

int Scene::getObjectId(std::string name) {
	for (int i = 0; i < m_meshes.size(); i++) {
		if (m_meshes[i]->m_name == name)
			return i;
	}
	return -1;
}


Scene::~Scene()
{
	glDeleteVertexArrays(1, &m_emptyVAO);

	glDeleteBuffers(1, &m_transformUB);
	glDeleteBuffers(1, &m_shadingUB);

	Renderer::deleteMeshBuffer(m_skybox);

	glDeleteProgram(m_tonemapProgram);
	glDeleteProgram(m_aoblurProgram);
	glDeleteProgram(m_skyboxProgram);
	Renderer::deleteTexture(m_envTexture);
	Renderer::deleteTexture(m_irmapTexture);
	Renderer::deleteTexture(m_spBRDF_LUT);
}


void Scene::initialize()
{
	m_blurpara = glm::vec3(2, 1.5, 0.9);
	m_ssaoPara[0] = 0.9f;
	m_ssaoPara[1] = 0.009f;
	m_ssaoPara[2] = 3.0f;
	m_tonemapPara[0] = 3.0f;//exposure
	m_tonemapPara[1] = 2.0f;//gamma
	m_tonemapPara[2] = 1.0f;//pureWhite
	// Parameters
	static constexpr int kEnvMapSize = 1024;
	static constexpr int kIrradianceMapSize = 32;
	static constexpr int kBRDF_LUT_Size = 256;

	// Set global OpenGL state.
	glEnable(GL_CULL_FACE);
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
	glFrontFace(GL_CCW);

	// Create empty VAO for rendering full screen triangle.
	glCreateVertexArrays(1, &m_emptyVAO);

	// Create uniform buffers.
	m_transformUB = Renderer::createUniformBuffer<TransformUB>();
	m_shadingUB = Renderer::createUniformBuffer<ShadingUB>();

	printf("compiling...tonemap");
	// Load assets & compile/link rendering programs.
	m_tonemapProgram = Renderer::linkProgram({
		Renderer::compileShader(m_shaderFolder + "tonemap_vs.glsl", GL_VERTEX_SHADER),
		Renderer::compileShader(m_shaderFolder + "tonemap_fs.glsl", GL_FRAGMENT_SHADER)
		});

	m_aoblurProgram = Renderer::linkProgram({
		Renderer::compileShader(m_shaderFolder + "tonemap_vs.glsl", GL_VERTEX_SHADER),
		Renderer::compileShader(m_shaderFolder + "ssao_blur.glsl", GL_FRAGMENT_SHADER)
		});

	m_skybox = Renderer::createMeshBuffer(createSkyBox());

	printf("compiling...skybox");
	m_skyboxProgram = Renderer::linkProgram({
		Renderer::compileShader(m_shaderFolder + "skybox_vs.glsl", GL_VERTEX_SHADER),
		Renderer::compileShader(m_shaderFolder + "skybox_fs.glsl", GL_FRAGMENT_SHADER)
		});

	
	// Unfiltered environment cube map (temporary).
	Texture envTextureUnfiltered = Renderer::createTexture(GL_TEXTURE_CUBE_MAP, kEnvMapSize, kEnvMapSize, GL_RGBA16F);

	// Load & convert equirectangular environment map to a cubemap texture.
	{
		GLuint equirectToCubeProgram = Renderer::linkProgram({
			Renderer::compileShader(m_shaderFolder +"equirect2cube_cs.glsl", GL_COMPUTE_SHADER)
			});

		Texture envTextureEquirect = Renderer::createTexture(Image::fromFile(m_shaderFolder+m_hdrfile, 3), GL_RGB, GL_RGB16F, 1);

		glUseProgram(equirectToCubeProgram);
		glBindTextureUnit(0, envTextureEquirect.id);
		glBindImageTexture(0, envTextureUnfiltered.id, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glDispatchCompute(envTextureUnfiltered.width / 32, envTextureUnfiltered.height / 32, 6);

		glDeleteTextures(1, &envTextureEquirect.id);
		glDeleteProgram(equirectToCubeProgram);
	}

	glGenerateTextureMipmap(envTextureUnfiltered.id);

	// Compute pre-filtered specular environment map.
	{
		GLuint spmapProgram = Renderer::linkProgram({
			Renderer::compileShader(m_shaderFolder + "spmap_cs.glsl", GL_COMPUTE_SHADER)
			});

		m_envTexture = Renderer::createTexture(GL_TEXTURE_CUBE_MAP, kEnvMapSize, kEnvMapSize, GL_RGBA16F);

		// Copy 0th mipmap level into destination environment map.
		glCopyImageSubData(envTextureUnfiltered.id, GL_TEXTURE_CUBE_MAP, 0, 0, 0, 0,
			m_envTexture.id, GL_TEXTURE_CUBE_MAP, 0, 0, 0, 0,
			m_envTexture.width, m_envTexture.height, 6);

		glUseProgram(spmapProgram);
		glBindTextureUnit(0, envTextureUnfiltered.id);

		// Pre-filter rest of the mip chain.
		const float deltaRoughness = 1.0f / glm::max(float(m_envTexture.levels - 1), 1.0f);
		for (int level = 1, size = kEnvMapSize / 2; level <= m_envTexture.levels; ++level, size /= 2) {
			const GLuint numGroups = glm::max(1, size / 32);
			glBindImageTexture(0, m_envTexture.id, level, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
			glProgramUniform1f(spmapProgram, 0, level * deltaRoughness);
			glDispatchCompute(numGroups, numGroups, 6);
		}
		glDeleteProgram(spmapProgram);
	}

	glDeleteTextures(1, &envTextureUnfiltered.id);

	// Compute diffuse irradiance cubemap.
	{
		GLuint irmapProgram = Renderer::linkProgram({
			Renderer::compileShader(m_shaderFolder + "irmap_cs.glsl", GL_COMPUTE_SHADER)
			});

		m_irmapTexture = Renderer::createTexture(GL_TEXTURE_CUBE_MAP, kIrradianceMapSize, kIrradianceMapSize, GL_RGBA16F, 1);

		glUseProgram(irmapProgram);
		glBindTextureUnit(0, m_envTexture.id);
		glBindImageTexture(0, m_irmapTexture.id, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glDispatchCompute(m_irmapTexture.width / 32, m_irmapTexture.height / 32, 6);
		glDeleteProgram(irmapProgram);
	}

	// Compute Cook-Torrance BRDF 2D LUT for split-sum approximation.
	{
		GLuint spBRDFProgram = Renderer::linkProgram({
			Renderer::compileShader(m_shaderFolder + "spbrdf_cs.glsl", GL_COMPUTE_SHADER)
			});

		m_spBRDF_LUT = Renderer::createTexture(GL_TEXTURE_2D, kBRDF_LUT_Size, kBRDF_LUT_Size, GL_RG16F, 1);
		glTextureParameteri(m_spBRDF_LUT.id, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTextureParameteri(m_spBRDF_LUT.id, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glUseProgram(spBRDFProgram);
		glBindImageTexture(0, m_spBRDF_LUT.id, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RG16F);
		glDispatchCompute(m_spBRDF_LUT.width / 32, m_spBRDF_LUT.height / 32, 1);
		glDeleteProgram(spBRDFProgram);
	}

	glFinish();

	m_gbufferProgram = loadProgram(m_shaderFolder + "gbuffer.glsl");


	m_gtaoProgram = loadProgram(m_shaderFolder + "ssao.glsl");


	auto rand_float = []() {
		return rand() / (float)RAND_MAX;
	};
	const int numberOfNoise = 16;
	for (int i = 0; i < numberOfNoise; ++i) {
		auto noise_vec = glm::vec3(rand_float() * 2.f - 1.f, rand_float() * 2.f - 1.f, 0);
		noise.push_back(noise_vec);
	}

	const int numberOfSamples = 64;
	for (int i = 0; i < numberOfSamples; ++i) {
		glm::vec3 sample = glm::normalize (glm::vec3(rand_float() * 2.0 - 1.0
				, rand_float() * 2.0 - 1.0
				, rand_float()
			));
		

		float rand = rand_float();
		sample[0] *= rand;
		sample[1] *= rand;
		sample[2] *= rand;

		float scale = (float)i / (float)numberOfSamples;
		scale = glm::lerp<float>(0.1f, 1.0f, scale * scale);
		sample[0] *= scale;
		sample[1] *= scale;
		sample[2] *= scale;

		ssaosamples.push_back(sample);
	}

}

int Scene::createPBRObj(std::string name, const std::string& abeldo, const std::string& normal,
	const std::string& metallic, const std::string& roughness) {
	std::shared_ptr<Mesh> pbr_mesh(new Mesh());
	pbr_mesh->m_buffer = Renderer::createMeshBuffer();
	auto mat = std::make_shared<MaterialPBR>(m_shaderFolder);
	mat->setTextures(abeldo, 	normal,  metallic, roughness);
	mat->setEnvLut(m_envTexture.id, m_irmapTexture.id, m_spBRDF_LUT.id);
	pbr_mesh->m_material = mat;
	pbr_mesh->m_name = name;
	pbr_mesh->m_transform = glm::mat4(1.0f);
	int id = m_meshes.size();
	m_meshes.emplace_back(pbr_mesh);
	return id;
}

int Scene::createPBRObj(std::string name, float r, float g, float b, float metallic, float roughness) {
	std::shared_ptr<Mesh> pbr_mesh(new Mesh());
	pbr_mesh->m_buffer = Renderer::createMeshBuffer();
	auto mat = std::make_shared<MaterialPBR>(m_shaderFolder);
	mat->m_diffColor.r = r;
	mat->m_diffColor.g = g;
	mat->m_diffColor.b = b;
	mat->m_diffColor.a = 1;
	mat->m_material.r = metallic;
	mat->m_material.g = roughness;
	mat->setEnvLut(m_envTexture.id, m_irmapTexture.id, m_spBRDF_LUT.id);
	pbr_mesh->m_material = mat;
	pbr_mesh->m_name = name;
	pbr_mesh->m_transform = glm::mat4(1.0f);
	int id = m_meshes.size();
	m_meshes.emplace_back(pbr_mesh);
	return id;
}

void Scene::setEnvForPBR()
{
	for (auto& obj : m_meshes)
	{
		if (typeid(obj->m_material) == typeid(MaterialPBR))
		{
			(dynamic_cast<MaterialPBR*>(obj->m_material.get())->setEnvLut(m_envTexture.id, m_irmapTexture.id, m_spBRDF_LUT.id));
		}
	}
}

void Scene::draw(Utility::ArcballCamera* cam, int new_program)
{
	// Draw skybox.
	if (!new_program)
	{
		if (use_sky)
		{
			glDisable(GL_DEPTH_TEST);
			glUseProgram(this->m_skyboxProgram);
			glBindTextureUnit(0, this->m_envTexture.id);
			glBindVertexArray(this->m_skybox.vao);
			glDrawElements(GL_TRIANGLES, this->m_skybox.numElements, GL_UNSIGNED_INT, 0);

			glEnable(GL_DEPTH_TEST);
		}
		else
		{
			glClearColor(sky_color.x, sky_color.y, sky_color.z, 1);
			glClear(GL_COLOR_BUFFER_BIT);
		}
	}

	// Draw PBR model.

	for (auto obj : m_meshes)
	{
		obj->draw(cam, new_program);		
	}

}


std::shared_ptr<MeshData> createSkyBox(){
	std::shared_ptr<MeshData> mesh = std::make_shared<MeshData>();
	MeshData::Vertex vert;
	MeshData::Face face;
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = -9.999999;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999996;
	vert.position[1] = -10.000000;
	vert.position[2] = -10.000004;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000001;
	vert.position[1] = -10.000000;
	vert.position[2] = 9.999998;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = 10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000005;
	vert.position[1] = 10.000000;
	vert.position[2] = -9.999994;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 9.999993;
	vert.position[1] = 10.000000;
	vert.position[2] = 10.000006;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000004;
	vert.position[1] = 10.000000;
	vert.position[2] = 9.999996;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999999;
	vert.position[1] = 10.000000;
	vert.position[2] = -10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = -9.999999;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = 10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 9.999993;
	vert.position[1] = 10.000000;
	vert.position[2] = 10.000006;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000005;
	vert.position[1] = 10.000000;
	vert.position[2] = -9.999994;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = 10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000001;
	vert.position[1] = -10.000000;
	vert.position[2] = 9.999998;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000004;
	vert.position[1] = 10.000000;
	vert.position[2] = 9.999996;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 9.999993;
	vert.position[1] = 10.000000;
	vert.position[2] = 10.000006;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000001;
	vert.position[1] = -10.000000;
	vert.position[2] = 9.999998;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999996;
	vert.position[1] = -10.000000;
	vert.position[2] = -10.000004;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999999;
	vert.position[1] = 10.000000;
	vert.position[2] = -10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -10.000004;
	vert.position[1] = 10.000000;
	vert.position[2] = 9.999996;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000005;
	vert.position[1] = 10.000000;
	vert.position[2] = -9.999994;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999999;
	vert.position[1] = 10.000000;
	vert.position[2] = -10.000000;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = -9.999996;
	vert.position[1] = -10.000000;
	vert.position[2] = -10.000004;
	mesh->m_vertices.push_back(vert);
	vert.position[0] = 10.000000;
	vert.position[1] = -10.000000;
	vert.position[2] = -9.999999;
	mesh->m_vertices.push_back(vert);
	face.v1 = 0;
	face.v2 = 1;
	face.v3 = 2;
	mesh->m_faces.push_back(face);
	face.v1 = 0;
	face.v2 = 2;
	face.v3 = 3;
	mesh->m_faces.push_back(face);
	face.v1 = 4;
	face.v2 = 5;
	face.v3 = 6;
	mesh->m_faces.push_back(face);
	face.v1 = 4;
	face.v2 = 6;
	face.v3 = 7;
	mesh->m_faces.push_back(face);
	face.v1 = 8;
	face.v2 = 9;
	face.v3 = 10;
	mesh->m_faces.push_back(face);
	face.v1 = 8;
	face.v2 = 10;
	face.v3 = 11;
	mesh->m_faces.push_back(face);
	face.v1 = 12;
	face.v2 = 13;
	face.v3 = 14;
	mesh->m_faces.push_back(face);
	face.v1 = 12;
	face.v2 = 14;
	face.v3 = 15;
	mesh->m_faces.push_back(face);
	face.v1 = 16;
	face.v2 = 17;
	face.v3 = 18;
	mesh->m_faces.push_back(face);
	face.v1 = 16;
	face.v2 = 18;
	face.v3 = 19;
	mesh->m_faces.push_back(face);
	face.v1 = 20;
	face.v2 = 21;
	face.v3 = 22;
	mesh->m_faces.push_back(face);
	face.v1 = 20;
	face.v2 = 22;
	face.v3 = 23;
	mesh->m_faces.push_back(face);
	return mesh;
}


