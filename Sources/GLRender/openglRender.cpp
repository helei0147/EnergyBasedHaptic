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


void Renderer::render(GLFWwindow* window, const ViewSettings& view, const SceneSettings& scene)
{
	glm::vec3 clipinfo;
	glm::vec4 projinfo;

	//setup common used uniform matrix
	const glm::mat4 projectionMatrix = view.camera->getProj();
	const glm::mat4 viewRotationMatrix = glm::mat3(view.camera->getView());
	const glm::mat4 sceneRotationMatrix = glm::mat4(1.0f);
	const glm::mat4 viewMatrix = view.camera->getView();
	const glm::vec3 eyePosition = view.camera->getPosition();

	// Update transform uniform buffer.
	{
		TransformUB transformUniforms;
		transformUniforms.mvp = projectionMatrix * viewMatrix;
		transformUniforms.sky_mvp = projectionMatrix * viewRotationMatrix;
		transformUniforms.eyepos = glm::vec4(eyePosition,0.0);
		glNamedBufferSubData(m_sceneobject->m_transformUB, 0, sizeof(TransformUB), &transformUniforms);
	}

	// Update shading uniform buffer.
	{
		ShadingUB shadingUniforms;
		shadingUniforms.eyePosition = glm::vec4(eyePosition, 0.0f);
		for(int i=0; i<SceneSettings::NumLights; ++i) {
			const SceneSettings::Light& light = scene.lights[i];
			shadingUniforms.lights[i].direction = glm::vec4{light.direction, 0.0f};
			if(light.enabled) {
				shadingUniforms.lights[i].radiance = glm::vec4{light.radiance, 0.0f};
			}
			else {
				shadingUniforms.lights[i].radiance = glm::vec4{};
			}
		}
		glNamedBufferSubData(m_sceneobject->m_shadingUB, 0, sizeof(ShadingUB), &shadingUniforms);
	}
	//generate gbuffer

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	//glCullFace(GL_BACK);
	glDisable(GL_CULL_FACE);
	glBindFramebuffer(GL_FRAMEBUFFER, m_gbuffer.id);

	glDepthMask(true);
	glClearDepth(1.0);

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glUseProgram(m_sceneobject->m_gbufferProgram);
	
	clipinfo.x = view.camera->near;
	clipinfo.y = view.camera->far;
	clipinfo.z = 0.5f * (m_gbuffer.width/ (2.0f * tanf(3.14159265f/4 * 0.5f)));

	projinfo = { 2.0f / (m_gbuffer.width * projectionMatrix[1][1]), 
		2.0f / (m_gbuffer.height * projectionMatrix[2][2]),
		-1.0f / projectionMatrix[1][1], 
		-1.0f / projectionMatrix[2][2]
	};


	auto uniform_view = glGetUniformLocation(m_sceneobject->m_gbufferProgram, "view");
	if (uniform_view != -1)
	{
		glUniformMatrix4fv(uniform_view, 1,false, &view.camera->getView()[0][0]);
	}


	auto uniform_clipPlanes = glGetUniformLocation(m_sceneobject->m_gbufferProgram, "clipPlanes");
	if (uniform_clipPlanes != -1)
	{
		glm::vec2 clipPlane(view.camera->near, view.camera->far);
		glUniform2fv(uniform_clipPlanes, 1, &clipPlane.x);
	}
	m_sceneobject->draw(view.camera, m_sceneobject->m_gbufferProgram);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Prepare framebuffer for rendering.
	glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer.id);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); // No need to clear color, since we'll overwrite the screen with our skybox.
	
	// Bind uniform buffers.
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_sceneobject->m_transformUB);
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, m_sceneobject->m_shadingUB);

	glDisable(GL_CULL_FACE);
	m_sceneobject->draw(view.camera);
	glDepthMask(GL_FALSE);

	
	// Resolve multisample framebuffer.
	resolveFramebuffer(m_framebuffer, m_resolveFramebuffer);

	// Draw a full screen triangle for postprocessing/tone mapping.
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	if (!m_sceneobject->enable_ao)
	{
		glUseProgram(m_sceneobject->m_tonemapProgram);
		glBindTextureUnit(0, m_resolveFramebuffer.colorTarget[0]);// m_gbuffer.colorTarget[2]);//

		glUniform1i(glGetUniformLocation(m_sceneobject->m_tonemapProgram, "enable_ao"), 0);
	}
	else
	{

		glBindFramebuffer(GL_FRAMEBUFFER, m_postbuffer.id);
		glUseProgram(m_sceneobject->m_gtaoProgram);
		glBindTextureUnit(0, m_gbuffer.colorTarget[1]);//normal buffer
		glBindTextureUnit(1, m_gbuffer.colorTarget[2]);//position buffer
		//glBindTextureUnit(2, sceneobject->noise.id);

		auto loc_noise = glGetUniformLocation(m_sceneobject->m_gtaoProgram, "noise");
		glUniform3fv(loc_noise, m_sceneobject->noise.size(), &m_sceneobject->noise[0].x);

		auto loc_samples = glGetUniformLocation(m_sceneobject->m_gtaoProgram, "samples");
		glUniform3fv(loc_samples, m_sceneobject->ssaosamples.size(), &m_sceneobject->ssaosamples[0].x);

		auto loc_campos = glGetUniformLocation(m_sceneobject->m_gtaoProgram, "cam_pos");
		glUniform3fv(loc_campos, 1, &view.camera->eye.x);
		
		auto loc_projection = glGetUniformLocation(m_sceneobject->m_gtaoProgram, "projection");
		glUniformMatrix4fv(loc_projection, 1, false,&projectionMatrix[0][0]);

		auto loc_paras = glGetUniformLocation(m_sceneobject->m_gtaoProgram, "paras");
		glUniform4fv(loc_paras, 1, &m_sceneobject->m_ssaoPara[0]);

		glUniform4fv(glGetUniformLocation(m_sceneobject->m_tonemapProgram, "para"), 1, &m_sceneobject->m_tonemapPara[0]);

		glBindVertexArray(m_sceneobject->m_emptyVAO);
		glDrawArrays(GL_TRIANGLES, 0, 3);

		//------
		glBindFramebuffer(GL_FRAMEBUFFER, 0);


		////blur

		glBindFramebuffer(GL_FRAMEBUFFER, m_aoblurbuffer.id);
		glUseProgram(m_sceneobject->m_aoblurProgram);
		auto blur_para = glGetUniformLocation(m_sceneobject->m_aoblurProgram, "blurpara");
		glUniform3fv(blur_para, 1, &m_sceneobject->m_blurpara.x);

		glBindTextureUnit(0, m_postbuffer.colorTarget[0]);//normal buffer

		glBindVertexArray(m_sceneobject->m_emptyVAO);
		glDrawArrays(GL_TRIANGLES, 0, 3);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		////end blur


		glUseProgram(m_sceneobject->m_tonemapProgram);

		glUniform1i(glGetUniformLocation(m_sceneobject->m_tonemapProgram, "enable_ao"), 1);
		glUniform4fv(glGetUniformLocation(m_sceneobject->m_tonemapProgram, "para"), 1, &m_sceneobject->m_tonemapPara[0]);


		glBindTextureUnit(0, m_resolveFramebuffer.colorTarget[0]);
		glBindTextureUnit(1, m_aoblurbuffer.colorTarget[0]);
	}
	glBindVertexArray(m_sceneobject->m_emptyVAO);
	glDrawArrays(GL_TRIANGLES, 0, 3);

	if(m_doUI)
		doUI();
}

void Renderer::resize(int w, int h)
{
	if (w < 10 || h < 10)
		return;
	glViewport(0, 0, w, h);

	deleteFrameBuffer(m_framebuffer);
	deleteFrameBuffer(m_resolveFramebuffer);
	deleteFrameBuffer(m_gbuffer);
	deleteFrameBuffer(m_postbuffer);
	deleteFrameBuffer(m_aoblurbuffer);

	GLint maxSupportedSamples;
	glGetIntegerv(GL_MAX_SAMPLES, &maxSupportedSamples);

	const int samples = glm::min(max_sample_, maxSupportedSamples);
	m_framebuffer = createFrameBuffer(w, h, samples, { GL_RGBA16F }, GL_DEPTH_COMPONENT32);
	if (samples > 0) {
		m_resolveFramebuffer = createFrameBuffer(w, h, 0, { GL_RGBA16F }, GL_NONE);
	}
	else {
		m_resolveFramebuffer = m_framebuffer;
	}
	m_gbuffer = createFrameBuffer(w, h, 0, { GL_RGBA8,
		GL_RGBA16F,
		GL_RGBA32F
		}, GL_DEPTH_COMPONENT32);

	m_postbuffer = createFrameBuffer(w, h, 0, { GL_RGBA8 }, GL_NONE);
	m_aoblurbuffer = createFrameBuffer(w, h, 0, { GL_RGBA8 }, GL_NONE);
}
	
