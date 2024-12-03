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


std::optional<std::shared_ptr<LoadedGLTF>> loadGltf(VulkanEngine* engine, std::string_view filePath) {

	fmt::print("Loading GLTF: {}", filePath);

	std::shared_ptr<LoadedGLTF> scene = std::make_shared<LoadedGLTF>();
	scene->creator = engine;
	LoadedGLTF& file = *scene.get();

	fastgltf::Parser parser{};

	constexpr auto gltfOptions = 
		  fastgltf::Options::DontRequireValidAssetMember 
		| fastgltf::Options::AllowDouble 
		| fastgltf::Options::LoadGLBBuffers 
		| fastgltf::Options::LoadExternalBuffers;

	fastgltf::GltfDataBuffer data;
	data.loadFromFile(filePath);

	fastgltf::Asset gltf;

	std::filesystem::path path = filePath;

	auto type = fastgltf::determineGltfFileType(&data);
	if (type == fastgltf::GltfType::glTF) {
		auto load = parser.loadGLTF(&data, path.parent_path(), gltfOptions);
		if (load) {
			gltf = std::move(load.get());
		}
		else {
			std::cerr << "Failed to load GLTF: " << fastgltf::to_underlying(load.error()) << "\n";
			return {};
		}
	}
	else if (type == fastgltf::GltfType::GLB) {
		auto load = parser.loadBinaryGLTF(&data, path.parent_path(), gltfOptions);
		if (load) {
			gltf = std::move(load.get());
		}
		else {
			std::cerr << "Failed to load GLTF: " << fastgltf::to_underlying(load.error()) << "\n";
			return {};
		}
	}
	else {
		std::cerr << "Failed to determine GLTF container\n";
		return {};
	}

	// can estimate descriptors we need accurately
	std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = {
		{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3},
		{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1}
	};

	file.descriptorPool.init(engine->_device, gltf.materials.size(), sizes);

	// load samplers
	for (fastgltf::Sampler& sampler : gltf.samplers) {

		VkSamplerCreateInfo sampl = { .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
		sampl.maxLod = VK_LOD_CLAMP_NONE;
		sampl.minLod = 0;

		sampl.magFilter = extract_filter(sampler.magFilter.value_or(fastgltf::Filter::Nearest));
		sampl.minFilter = extract_filter(sampler.minFilter.value_or(fastgltf::Filter::Nearest));

		sampl.mipmapMode = extract_mipmap_mode(sampler.minFilter.value_or(fastgltf::Filter::Nearest));

		VkSampler newSampler;
		vkCreateSampler(engine->_device, &sampl, nullptr, &newSampler);

		file.samplers.push_back(newSampler);
	}

	// temporal arrays for all the objects to use while creating the GLTF data
	std::vector<std::shared_ptr<MeshAsset>> meshes;
	std::vector<std::shared_ptr<Node>> nodes;
	std::vector<AllocatedImage> images;
	std::vector<std::shared_ptr<GLTFMaterial>> materials;

	// load textures
	for (fastgltf::Image& image : gltf.images) {
		images.push_back(engine->_errorCheckerboardImage);
	}

	// hold material data
	file.materialDataBuffer = engine->create_buffer(sizeof(GLTFMetallic_Roughness::MaterialConstants) * gltf.materials.size(),
		VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
	int data_index = 0;
	GLTFMetallic_Roughness::MaterialConstants* sceneMaterialConstants = (GLTFMetallic_Roughness::MaterialConstants*)file.materialDataBuffer.info.pMappedData;

	// load materials
	for (fastgltf::Material& mat : gltf.materials) {
		std::shared_ptr<GLTFMaterial> newMat = std::make_shared<GLTFMaterial>();
		materials.push_back(newMat);
		file.materials[mat.name.c_str()] = newMat;

		GLTFMetallic_Roughness::MaterialConstants constants;
		constants.colourFactors.x = mat.pbrData.baseColorFactor[0];
		constants.colourFactors.y = mat.pbrData.baseColorFactor[1];
		constants.colourFactors.z = mat.pbrData.baseColorFactor[2];
		constants.colourFactors.w = mat.pbrData.baseColorFactor[3];

		constants.metalRoughFactors.x = mat.pbrData.metallicFactor;
		constants.metalRoughFactors.y = mat.pbrData.roughnessFactor;

		// write material parameters to buffer
		sceneMaterialConstants[data_index] = constants;

		MaterialPass passType = MaterialPass::MainColour;
		if (mat.alphaMode == fastgltf::AlphaMode::Blend) {
			passType = MaterialPass::Transparent;
		}

		GLTFMetallic_Roughness::MaterialResources materialResources;
		
		// default material textures
		materialResources.colourImage = engine->_whiteImage;
		materialResources.colourSampler = engine->_defaultSamplerLinear;
		materialResources.metalRoughImage = engine->_whiteImage;
		materialResources.metalRoughSampler = engine->_defaultSamplerLinear;

		// set uniform buffer for material data
		materialResources.dataBuffer = file.materialDataBuffer.buffer;
		materialResources.dataBufferOffset = data_index * sizeof(GLTFMetallic_Roughness::MaterialConstants);

		// grab textures from GLTF File
		if (mat.pbrData.baseColorTexture.has_value()) {
			size_t img = gltf.textures[mat.pbrData.baseColorTexture.value().textureIndex].imageIndex.value();
			size_t sampler = gltf.textures[mat.pbrData.baseColorTexture.value().textureIndex].samplerIndex.value();
		
			materialResources.colourImage = images[img];
			materialResources.colourSampler = file.samplers[sampler];
		}

		// build material
		newMat->data = engine->metalRoughMaterial
	}
}

VkFilter extract_filter(fastgltf::Filter filter) {
	switch (filter) {
	// nearest samplers
	case fastgltf::Filter::Nearest:
	case fastgltf::Filter::NearestMipMapNearest:
	case fastgltf::Filter::NearestMipMapLinear:
		return VK_FILTER_NEAREST;

	// linear samplers
	case fastgltf::Filter::Linear:
	case fastgltf::Filter::LinearMipMapNearest:
	case fastgltf::Filter::LinearMipMapLinear:
	default:
		return VK_FILTER_LINEAR;
	}
}

VkSamplerMipmapMode extract_mipmap_mode(fastgltf::Filter filter) {
	switch (filter) {
	case fastgltf::Filter::NearestMipMapNearest:
	case fastgltf::Filter::LinearMipMapNearest:
		return VK_SAMPLER_MIPMAP_MODE_NEAREST;

	case fastgltf::Filter::NearestMipMapLinear:
	case fastgltf::Filter::LinearMipMapLinear:
	default:
		return VK_SAMPLER_MIPMAP_MODE_LINEAR;
	}
}


//std::optional<std::vector<std::shared_ptr<MeshAsset>>> loadGltfMeshes(VulkanEngine* engine, std::filesystem::path filePath) {
//	std::cout << "Loading GLTF: " << filePath << std::endl;
//
//	fastgltf::GltfDataBuffer data;
//	data.loadFromFile(filePath);
//
//	constexpr auto gltfOptions = fastgltf::Options::LoadGLBBuffers | fastgltf::Options::LoadExternalBuffers;
//
//	fastgltf::Asset gltf;
//	fastgltf::Parser parser{};
//
//	auto load = parser.loadBinaryGLTF(&data, filePath.parent_path(), gltfOptions);
//	
//	if (load) {
//		gltf = std::move(load.get());
//	}
//	else {
//		fmt::print("Failed to load GLTF : {} \n", fastgltf::to_underlying(load.error()));
//		return {};
//	}
//
//	// Temporary Mesh Asset Arrays
//	std::vector<std::shared_ptr<MeshAsset>> meshes;
//
//	// Indices and Vertex Arrays
//	std::vector<uint32_t> indices;
//	std::vector<Vertex> vertices;
//
//	for (fastgltf::Mesh& mesh : gltf.meshes) {
//		MeshAsset newMesh;
//
//		newMesh.name = mesh.name;
//
//		indices.clear();
//		vertices.clear();
//
//		for (auto&& p : mesh.primitives) {
//			GeoSurface newSurface;
//			newSurface.startIndex = (uint32_t)indices.size();
//			newSurface.count = (uint32_t)gltf.accessors[p.indicesAccessor.value()].count;
//
//			size_t initial_vtx = vertices.size();
//
//			// Load Indexes
//			{
//				fastgltf::Accessor& indexAccessor = gltf.accessors[p.indicesAccessor.value()];
//				indices.reserve(indices.size() + indexAccessor.count);
//
//				fastgltf::iterateAccessor<std::uint32_t>(gltf, indexAccessor, 
//					[&](std::uint32_t idx) {
//						indices.push_back(idx + initial_vtx);
//					});
//			}
//
//			// Load Vertex Positions
//			{
//				fastgltf::Accessor& posAccessor = gltf.accessors[p.findAttribute("POSITION")->second];
//				vertices.resize(vertices.size() + posAccessor.count);
//
//				fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, posAccessor,
//					[&](glm::vec3 v,size_t index) {
//						Vertex newVertex;
//						newVertex.position = v;
//						newVertex.normal = { 1, 0, 0 };
//						newVertex.colour = glm::vec4{ 1.f };
//						newVertex.uv_x = 0;
//						newVertex.uv_y = 0;
//						vertices[initial_vtx + index] = newVertex;
//					});
//			}
//
//			// Load Vertex Normals
//			auto normals = p.findAttribute("NORMAL");
//			if (normals != p.attributes.end()) {
//				fastgltf::iterateAccessorWithIndex<glm::vec3>(gltf, gltf.accessors[(*normals).second],
//					[&](glm::vec3 v, size_t index) {
//						vertices[initial_vtx + index].normal = v;
//					});
//			}
//
//			// Load UVs
//			auto uv = p.findAttribute("TEXCOORD_0");
//			if (uv != p.attributes.end()) {
//				fastgltf::iterateAccessorWithIndex<glm::vec2>(gltf, gltf.accessors[(*uv).second],
//					[&](glm::vec2 v, size_t index) {
//						vertices[initial_vtx + index].uv_x = v.x;
//						vertices[initial_vtx + index].uv_y = v.y;
//					});
//			}
//
//			// Load Vertex Colours
//			auto colours = p.findAttribute("COLOR_0");
//			if (colours != p.attributes.end()) {
//				fastgltf::iterateAccessorWithIndex<glm::vec4>(gltf, gltf.accessors[(*colours).second],
//					[&](glm::vec4 v, size_t index) {
//						vertices[initial_vtx + index].colour = v;
//					});
//			}
//
//			// Add new surface to new mesh
//			newMesh.surfaces.push_back(newSurface);
//		}
//
//
//		constexpr bool OverrideColours = false;
//		if (OverrideColours) {
//			for (Vertex& vtx : vertices) {
//				vtx.colour = glm::vec4(vtx.normal, 1.f);
//			}
//		}
//
//		newMesh.meshBuffers = engine->upload_mesh(indices, vertices);
//
//		meshes.emplace_back(std::make_shared<MeshAsset>(std::move(newMesh)));
//	}
//
//	return meshes;
//}