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

	static const int NumAttributes = 3;

	struct Face
	{
		uint32_t v1, v2, v3;
	};

	const std::vector<Vertex>& vertices() const { return m_vertices; }
	const std::vector<Face>& faces() const { return m_faces; }

	void setVertices(const std::vector<Vertex>& v)
	{
		m_vertices = v;
	}

	void setFaces(const std::vector<uint32_t>& indices)
	{
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			Face f{
				indices[i],
				indices[i + 1],
				indices[i + 2]
			};
			m_faces.emplace_back(f);
		}
	}


	MeshData()
	{
		m_vertices.clear();
		m_faces.clear();
	}


	std::vector<Vertex> m_vertices;
	std::vector<Face> m_faces;

};

