#pragma once

// std::optional<std::vector<std::shared_ptr<MeshAsset>>> loadGltfMeshes(VulkanEngine* engine, std::filesystem::path filePath);

struct LoadedGLTF : public IRenderable {

public:

	// storage for all data on a given GLTF file
	std::unordered_map<std::string, std::shared_ptr<MeshAsset>> meshes;
	std::unordered_map<std::string, std::shared_ptr<Node>> nodes;
	std::unordered_map<std::string, AllocatedImage> images;
	std::unordered_map<std::string, std::shared_ptr<GLTFMaterial>> materials;

	// nodes that don't have a parent, for iterating through the file in tree order
	std::vector<std::shared_ptr<Node>> topNodes;

	std::vector<VkSampler> samplers;

	DescriptorAllocatorGrowable descriptorPool;

	AllocatedBuffer materialDataBuffer;

	VulkanEngine* creator;

	~LoadedGLTF() { clearAll(); }

	// std::optional<std::shared_ptr<LoadedGLTF>> loadGltf(VulkanEngine* engine, std::string_view filePath);

	virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx);

private:

	void clearAll();

};

// Loader Code
std::optional<std::shared_ptr<LoadedGLTF>> loadGltf(VulkanEngine* engine, std::string_view filePath);

// Conversion Code
VkFilter extract_filter(fastgltf::Filter filter);
VkSamplerMipmapMode extract_mipmap_mode(fastgltf::Filter filter);