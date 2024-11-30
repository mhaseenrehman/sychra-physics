// vulkan_guide.h : Include file for standard system include files,
// or project specific include files.
#pragma once
#include <vk_types.h>
#include <vk_descriptors.h>
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

// GLTF Mesh Structs
struct GeoSurface {
	uint32_t startIndex;
	uint32_t count;
};

struct MeshAsset {
	std::string name;
	std::vector<GeoSurface> surfaces;
	GPUMeshBuffers meshBuffers;
};

// Classes -------------------------------------------------------------------------------

class VulkanEngine {
public:

	// Base Structures
	bool _isInitialized{ false };
	bool bUseValidationLayers{ true };
	int _frameNumber {0};
	bool stop_rendering{ false };
	VkExtent2D _windowExtent{ 1700 , 900 };
	struct SDL_Window* _window{ nullptr };
	static VulkanEngine& Get();
	bool resize_requested{ false };

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
	DescriptorAllocator globalDescriptorAllocator;
	VkDescriptorSet _drawImageDescriptors;
	VkDescriptorSetLayout _drawImageDescriptorLayout;

	// Pipeline Structures
	VkPipeline _gradientPipeline;
	VkPipelineLayout _gradientPipelineLayout;

	VkPipeline _meshPipeline;
	VkPipelineLayout _meshPipelineLayout;
	GPUMeshBuffers rectangle;

	// Immediate Submit Structures
	VkFence _immFence;
	VkCommandBuffer _immCommandBuffer;
	VkCommandPool _immCommandPool;

	// Push Constant Effects
	std::vector<ComputeEffect> backgroundEffects;
	int currentBackgroundEffect{ 0 };

	// Test Meshes
	std::vector<std::shared_ptr<MeshAsset>> testMeshes;

	// Depth Testing Setup
	AllocatedImage _depthImage;
	
	// Initialises Engine
	void init();

	// Drawing loop
	void draw();
	
	// Main loop
	void run();
	
	// Immediate Submit Functions Setup
	void immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function);

	// Cleans Up Engine after shutdown
	void cleanup();

	// Mesh Buffer Setup
	AllocatedBuffer create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage);
	GPUMeshBuffers upload_mesh(std::span<uint32_t> indices, std::span<Vertex> vertices);
	void destroy_buffer(const AllocatedBuffer& buffer);

	

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
	void draw_imgui(VkCommandBuffer cmd, VkImageView targetImageView);

	// Triangle Shader & Mesh Shader
	void init_triangle_pipeline();
	void init_mesh_pipeline();
	void init_default_mesh_data();

	// Resizing Function
	void resize_swapchain();
};
