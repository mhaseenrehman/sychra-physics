#include "stb_image.h"
#include <iostream>
#include "vk_engine.h"
#include "vk_initializers.h"
#include "vk_types.h"
#include <glm/gtx/quaternion.hpp>

#include <fastgltf/glm_element_traits.hpp>
#include <fastgltf/parser.hpp>
#include <fastgltf/tools.hpp>

#include <vk_loader.h>

std::optional<std::vector<std::shared_ptr<MeshAsset>>> loadGltfMeshes(VulkanEngine* engine, std::filesystem::path filePath) {
	std::cout << "Loading GLTF: " << filePath << std::endl;

	fastgltf::GltfDataBuffer data;
	data.loadFromFile(filePath);

	constexpr auto gltfOptions = fastgltf::Options::LoadGLBBuffers | fastgltf::Options::LoadExternalBuffers;

	fastgltf::Asset gltf;
	fastgltf::Parser parser{};

	auto load = parser.loadBinaryGLTF(&data, filePath.parent_path(), gltfOptions);
	
	if (load) {
		gltf = std::move(load.get());
	}
	else {
		fmt::print("Failed to load GLTF : {} \n", fastgltf::to_underlying(load.error()));
		return {};
	}

	// Temporary Mesh Asset Arrays
	std::vector<std::shared_ptr<MeshAsset>> meshes;

	// Indices and Vertex Arrays
	std::vector<uint32_t> indices;
	std::vector<Vertex> vertices;

	for (fastgltf::Mesh& mesh : gltf.meshes) {
		MeshAsset newMesh;

		newMesh.name = mesh.name;

		indices.clear();
		vertices.clear();

		for (auto&& p : mesh.primitives) {
			GeoSurface newSurface;
			newSurface.startIndex = (uint32_t)indices.size();
			newSurface.count = (uint32_t)gltf.accessors[p.indicesAccessor.value()].count;

			size_t initial_vtx = vertices.size();

			// Load Indexes
			{
				fastgltf::Accessor& indexAccessor = gltf.accessors[p.indicesAccessor.value()];
				indices.reserve(indices.size() + indexAccessor.count);

				fastgltf::iterateAccessor<std::uint32_t>(gltf, indexAccessor, 
					[&](std::uint32_t idx) {
						indices.push_back(idx + initial_vtx);
					});
			}

			// Load Vertex Positions
			{
				fastgltf::Accessor& posAccessor = gltf.accessors[p.findAttribute("POSITION")->second];
				vertices.resize(vertices.size() + posAccessor.count);

				fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, posAccessor,
					[&](glm::vec3 v,size_t index) {
						Vertex newVertex;
						newVertex.position = v;
						newVertex.normal = { 1, 0, 0 };
						newVertex.colour = glm::vec4{ 1.f };
						newVertex.uv_x = 0;
						newVertex.uv_y = 0;
						vertices[initial_vtx + index] = newVertex;
					});
			}

			// Load Vertex Normals
			auto normals = p.findAttribute("NORMAL");
			if (normals != p.attributes.end()) {
				fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, gltf.accessors[(*normals).second],
					[&](glm::vec3 v, size_t index) {
						vertices[initial_vtx + index].normal = v;
					});
			}

			// Load UVs
			auto uv = p.findAttribute("TEXCOORD_0");
			if (uv != p.attributes.end()) {
				fastgltf::iterateAccessorWithIndex<glm::vec2>(gltf, gltf.accessors[(*uv).second],
					[&](glm::vec2 v, size_t index) {
						vertices[initial_vtx + index].uv_x = v.x;
						vertices[initial_vtx + index].uv_y = v.y;
					});
			}

			// Load Vertex Colours
			auto colours = p.findAttribute("COLOR_0");
			if (colours != p.attributes.end()) {
				fastgltf::iterateAccessorWithIndex<glm::vec4>(gltf, gltf.accessors[(*colours).second],
					[&](glm::vec4 v, size_t index) {
						vertices[initial_vtx + index].colour = v;
					});
			}

			// Add new surface to new mesh
			newMesh.surfaces.push_back(newSurface);
		}

		constexpr bool OverrideColours = true;
		if (OverrideColours) {
			for (Vertex& vtx : vertices) {
				vtx.colour = glm::vec4(vtx.normal, 1.f);
			}
		}

		newMesh.meshBuffers = engine->upload_mesh(indices, vertices);

		meshes.emplace_back(std::make_shared<MeshAsset>(std::move(newMesh)));
	}

	return meshes;
}