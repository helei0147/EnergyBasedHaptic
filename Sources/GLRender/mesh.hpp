/*
 * Physically Based Rendering
 * Copyright (c) 2017-2018 Michał Siejak
 */

#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
class MeshData
{
public:
	struct Vertex
	{
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec3 texcoord;
	};

	//static_assert(sizeof(Vertex) == 9 * sizeof(float));
	static const int NumAttributes = 3;

	struct Face
	{
		uint32_t v1, v2, v3;
	};

	//static_assert(sizeof(Face) == 3 * sizeof(uint32_t));

	static std::shared_ptr<MeshData> fromFile(const std::string& filename);
	static std::shared_ptr<MeshData> fromString(const std::string& data);

	const std::vector<Vertex>& vertices() const { return m_vertices; }
	const std::vector<Face>& faces() const { return m_faces; }

	static std::shared_ptr<MeshData> createGrid(int with, int height,float side_length = 1.0f);

	void setVertices(const std::vector<Vertex>& v);
	void setFaces(const std::vector<uint32_t>& i);
	//! create from assimp
	MeshData(const struct aiMesh* mesh);

	MeshData();

	std::vector<Vertex> m_vertices;
	std::vector<Face> m_faces;

};

