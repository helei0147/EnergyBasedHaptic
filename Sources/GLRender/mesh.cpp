/*
 * Physically Based Rendering
 * Copyright (c) 2017-2018 Michał Siejak
 */

#include <cstdio>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

#include <stdexcept> 
#include "mesh.hpp"

namespace {
	const unsigned int ImportFlags = 
		aiProcess_CalcTangentSpace |
		aiProcess_Triangulate |
		aiProcess_SortByPType |
		aiProcess_PreTransformVertices |
		aiProcess_GenSmoothNormals |
		aiProcess_GenUVCoords |
		aiProcess_OptimizeMeshes |
		aiProcess_Debone |
		aiProcess_ValidateDataStructure;
}

struct LogStream : public Assimp::LogStream
{
	static void initialize()
	{
		if(Assimp::DefaultLogger::isNullLogger()) {
			Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);
			Assimp::DefaultLogger::get()->attachStream(new LogStream, Assimp::Logger::Err | Assimp::Logger::Warn);
		}
	}
	
	void write(const char* message) override
	{
		std::fprintf(stderr, "Assimp: %s", message);
	}
};

std::shared_ptr<MeshData> MeshData::createGrid(int width, int height, float side_length)
{

	std::shared_ptr<MeshData> grid(new MeshData);

	//vertices for grid
	std::vector<Vertex> vertices;
	glm::vec3 offset(width * side_length,0, height * side_length);
	offset = offset * -0.5f;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			Vertex v;
			v.position = glm::vec3(x,0,y) * side_length + offset;
			v.normal = glm::vec3(0, 1, 0);
			//////v.bitangent = glm::vec3(1, 0, 0);
			//////v.tangent = glm::vec3(0, 0, 1);
			glm::vec2 t = glm::vec2(x, y) / glm::vec2(width - 1, height - 1);
			v.texcoord.x = t.x;
			v.texcoord.y = t.y;
			vertices.emplace_back(v);
		}
	}

	//faces for grid
	std::vector<unsigned int> faces;
	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{
			faces.emplace_back(y * width + x);
			faces.emplace_back((y + 1) * width + x + 1);
			faces.emplace_back(y * width + x + 1);

			faces.emplace_back(y * width + x);
			faces.emplace_back((y + 1) * width + x);
			faces.emplace_back((y + 1) * width + x + 1);

		}
	}

	grid->setVertices(vertices);
	grid->setFaces(faces);

	return grid;
}

void MeshData::setVertices(const std::vector<Vertex>& v)
{
	m_vertices = v;
}

void MeshData::setFaces(const std::vector<uint32_t>& indices)
{
	for (size_t i = 0; i < indices.size(); i+=3)
	{
		Face f{
			indices[i],
			indices[i + 1],
			indices[i + 2]
		};
		m_faces.emplace_back(f);
	}
}

MeshData::MeshData(const aiMesh* mesh)
{
	assert(mesh->HasPositions());
	assert(mesh->HasNormals());

	m_vertices.reserve(mesh->mNumVertices);
	for(size_t i=0; i<m_vertices.capacity(); ++i) {
		Vertex vertex;
		vertex.position = {mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z};
		vertex.normal = {mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z};
		//////if(mesh->HasTangentsAndBitangents()) {
		//////	vertex.tangent = {mesh->mTangents[i].x, mesh->mTangents[i].y, mesh->mTangents[i].z};
		//////	vertex.bitangent = {mesh->mBitangents[i].x, mesh->mBitangents[i].y, mesh->mBitangents[i].z};
		//////}
		if(mesh->HasTextureCoords(0)) {
			vertex.texcoord.x = mesh->mTextureCoords[0][i].x;
			vertex.texcoord.y = mesh->mTextureCoords[0][i].y;
		}
		m_vertices.push_back(vertex);
	}
	
	m_faces.reserve(mesh->mNumFaces);
	for(size_t i=0; i<m_faces.capacity(); ++i) {
		assert(mesh->mFaces[i].mNumIndices == 3);
		m_faces.push_back({mesh->mFaces[i].mIndices[0], mesh->mFaces[i].mIndices[1], mesh->mFaces[i].mIndices[2]});
	}

	//--------------------------------------------------
	// Create a vector to store vertex normals
	//std::vector<glm::vec3> normals(m_vertices.size(), glm::vec3(0.0f));
	//for (int i = 0; i < m_faces.size(); i++)
	//{
	//	auto& face = m_faces[i];
	//	auto edge_1 = m_vertices[face.v2].position - m_vertices[face.v1].position;
	//	auto edge_2 = m_vertices[face.v3].position - m_vertices[face.v1].position;

	//	auto normal = glm::normalize(glm::cross(edge_2, edge_1));

	//	normals[face.v1] += normal;
	//	normals[face.v2] += normal;
	//	normals[face.v3] += normal;
	//}

	//for (auto i = 0; i < normals.size(); i++)
	//{
	//	normals[i] = glm::normalize(normals[i]);

	//	m_vertices[i].normal = normals[i];
	//}

}

MeshData::MeshData()
{
	m_vertices.clear();
	m_faces.clear();
}

std::shared_ptr<MeshData> MeshData::fromFile(const std::string& filename)
{
	LogStream::initialize();

	std::printf("Loading mesh: %s\n", filename.c_str());
	
	std::shared_ptr<MeshData> mesh;
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(filename, ImportFlags);
	if(scene && scene->HasMeshes()) {
		mesh = std::shared_ptr<MeshData>(new MeshData{scene->mMeshes[0]});
	}
	else {
		throw std::runtime_error("Failed to load mesh file: " + filename);
	}
	return mesh;
}

std::shared_ptr<MeshData> MeshData::fromString(const std::string& data)
{
	LogStream::initialize();

	std::shared_ptr<MeshData> mesh;
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFileFromMemory(data.c_str(), data.length(), ImportFlags, "nff");
	if(scene && scene->HasMeshes()) {
		mesh = std::shared_ptr<MeshData>(new MeshData{scene->mMeshes[0]});
	}
	else {
		throw std::runtime_error("Failed to create mesh from string: " + data);
	}
	return mesh;
}
