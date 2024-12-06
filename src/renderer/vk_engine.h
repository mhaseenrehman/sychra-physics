// vulkan_guide.h : Include file for standard system include files,
// or project specific include files.
#pragma once
#include <vk_types.h>
#include <vk_descriptors.h>
#include "vk_loader.h"
#include <camera.h>
#include <unordered_map>
#include <filesystem>

// Globals -------------------------------------------------------------------------------

constexpr unsigned int FRAME_OVERLAP = 2;

// Structs -------------------------------------------------------------------------------

// Main Cleanup Struct
struct DeletionQueue {
	std::deque<std::function<void()>> deletors;

	void push_function(std::function<void()>&& function) {
		deletors.push_back(function);
	}

	void flush() {
		for (auto it = deletors.rbegin(); it != deletors.rend(); it++) {
			(*it)();
		}

		deletors.clear();
	}
};

// Swapchain Frame Struct
struct FrameData {
	VkCommandPool _commandPool;
	VkCommandBuffer _mainCommandBuffer;

	VkSemaphore _swapchainSemaphore, _renderSemaphore;
	VkFence _renderFence;

	DescriptorAllocatorGrowable _frameDescriptors;
	DeletionQueue _deletionQueue;
};

// Compute Pipeline Structs
struct ComputePushConstants {
	glm::vec4 data1;
	glm::vec4 data2;
	glm::vec4 data3;
	glm::vec4 data4;

};

struct ComputeEffect {
	const char* name;

	VkPipeline pipeline;
	VkPipelineLayout layout;

	ComputePushConstants data;
};

// Scene Data
struct GPUSceneData {
	glm::mat4 view;
	glm::mat4 proj;
	glm::mat4 viewproj;
	glm::vec4 ambientColour;
	glm::vec4 sunlightDirection; // w for sun power
	glm::vec4 sunlightColour;
};

// Material Setup - GLTF Loading Structs
struct GLTFMetallic_Roughness {
	MaterialPipeline opaquePipeline;
	MaterialPipeline transparentPipeline;

	VkDescriptorSetLayout materialLayout;

	struct MaterialConstants {
		glm::vec4 colourFactors;
		glm::vec4 metalRoughFactors;
		// Padding for Uniform Buffers
		glm::vec4 extra[14];
	};

	struct MaterialResources {
		AllocatedImage colourImage;
		VkSampler colourSampler;

		AllocatedImage metalRoughImage;
		VkSampler metalRoughSampler;

		VkBuffer dataBuffer;
		uint32_t dataBufferOffset;
	};

	DescriptorWriter writer;

	void build_pipelines(VulkanEngine* engine);
	void clear_resources(VkDevice device);

	MaterialInstance write_material(VkDevice device, MaterialPass pass, const MaterialResources& resources, DescriptorAllocatorGrowable& descriptorAllocator);
};


// Timing Info Struct
struct EngineStats {
	float frametime;
	int triangle_count;
	int drawcall_count;
	float scene_update_time;
	float mesh_draw_time;
};

// Classes -------------------------------------------------------------------------------

class VulkanEngine {
public:

	// Base Structures
	bool _isInitialized{ false };
	bool bUseValidationLayers{ false };
	int _frameNumber {0};
	bool stop_rendering{ false };
	VkExtent2D _windowExtent{ 1700 , 900 };
	struct SDL_Window* _window{ nullptr };
	static VulkanEngine& Get();
	bool resize_requested{ false };
	EngineStats stats;
	
	// World Camera
	Camera mainCamera;

	// Draw Context, Mesh Nodes & Scene
	DrawContext mainDrawContext;
	std::unordered_map<std::string, std::shared_ptr<Node>> loadedNodes;
	std::unordered_map<std::string, std::shared_ptr<LoadedGLTF>> loadedScenes;

	// Initial Structures
	VkInstance _instance;
	VkDebugUtilsMessengerEXT _debug_messenger;
	VkPhysicalDevice _chosenGPU;
	VkDevice _device;
	VkSurfaceKHR _surface;

	// Swapchain Structures
	VkSwapchainKHR _swapchain;
	VkFormat _swapchainImageFormat;
	std::vector<VkImage> _swapchainImages;
	std::vector<VkImageView> _swapchainImageViews;
	VkExtent2D _swapchainExtent;

	// Frame Data Structures
	FrameData _frames[FRAME_OVERLAP];
	FrameData& get_current_frame() { return _frames[_frameNumber % FRAME_OVERLAP]; }
	VkQueue _graphicsQueue;
	uint32_t _graphicsQueueFamily;

	// Memory Allocator
	VmaAllocator _allocator;

	// Deletion Queue
	DeletionQueue _mainDeletionQueue;

	// Draw Resource Structures
	AllocatedImage _drawImage;
	VkExtent2D _drawExtent;
	float renderScale = 1.f;

	// Descriptor Set Structures
	DescriptorAllocatorGrowable globalDescriptorAllocator;
	VkDescriptorSet _drawImageDescriptors;
	VkDescriptorSetLayout _drawImageDescriptorLayout;

	// Scene Data
	GPUSceneData sceneData;
	VkDescriptorSetLayout _gpuSceneDataDescriptorLayout;

	// Pipeline Structures
	VkPipeline _gradientPipeline;
	VkPipelineLayout _gradientPipelineLayout;

	VkPipeline _meshPipeline;
	VkPipelineLayout _meshPipelineLayout;
	//GPUMeshBuffers rectangle;

	// Immediate Submit Structures
	VkFence _immFence;
	VkCommandBuffer _immCommandBuffer;
	VkCommandPool _immCommandPool;

	// Push Constant Effects
	std::vector<ComputeEffect> backgroundEffects;
	int currentBackgroundEffect{ 0 };

	// Depth Testing Setup
	AllocatedImage _depthImage;

	// Default Textures
	AllocatedImage _whiteImage;
	AllocatedImage _blackImage;
	AllocatedImage _greyImage;
	AllocatedImage _errorCheckerboardImage;

	// Default Samplers
	VkSampler _defaultSamplerLinear;
	VkSampler _defaultSamplerNearest;

	// Image Descriptor Set
	VkDescriptorSetLayout _singleImageDescriptorLayout;

	// Test Meshes
	std::vector<std::shared_ptr<MeshAsset>> testMeshes;

	// Test Material
	MaterialInstance defaultData;
	GLTFMetallic_Roughness metalRoughMaterial;
	
	// Initialises Engine
	void init();

	// Drawing loop
	void draw();
	
	// Main loop
	void run();

	// Update Scene -> Physics & Draw functions + Camera
	void update_scene();
	
	// Immediate Submit Functions Setup
	void immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function);

	// Cleans Up Engine after shutdown
	void cleanup();

	// Mesh Buffer Setup
	AllocatedBuffer create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage);
	GPUMeshBuffers upload_mesh(std::span<uint32_t> indices, std::span<Vertex> vertices);
	void destroy_buffer(const AllocatedBuffer& buffer);

	// Image Functions Setup
	AllocatedImage create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage, bool mipmapped = false);
	AllocatedImage create_image(void* data, VkExtent3D size, VkFormat format, VkImageUsageFlags usage, bool mipmapped = false);
	void destroy_image(const AllocatedImage& img);
	

private:

	// Initial Functions Setup
	void init_vulkan();
	void init_swapchain();
	void init_commands();
	void init_sync_structures();
	void init_descriptors();
	void init_pipelines();
	void init_background_pipelines();
	void init_imgui();

	// Swapchain Functions Setup
	void create_swapchain(uint32_t width, uint32_t height);
	void destroy_swapchain();

	// Drawing Functions Setup
	void draw_background(VkCommandBuffer cmd);
	void draw_geometry(VkCommandBuffer cmd);
	void draw_particles(VkCommandBuffer cmd);
	void draw_imgui(VkCommandBuffer cmd, VkImageView targetImageView);


	// Triangle Shader & Mesh Shader
	//void init_triangle_pipeline();
	void init_mesh_pipeline();
	void init_default_mesh_data();
	void init_default_data();

	// Resizing Function
	void resize_swapchain();
};

