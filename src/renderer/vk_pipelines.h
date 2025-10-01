#pragma once 
#include <vk_types.h>

namespace vkutil {
	bool load_shader_module(const char* filePath, VkDevice, VkShaderModule* outShaderModule);
};

class PipelineBuilder {
public:
	std::vector<VkPipelineShaderStageCreateInfo> _shaderStages;
	VkPipelineInputAssemblyStateCreateInfo _inputAssembly;
	VkPipelineRasterizationStateCreateInfo _rasterizer;
	VkPipelineVertexInputStateCreateInfo _verticesInputState;
	VkPipelineColorBlendAttachmentState _colorBlendAttachment;
	VkPipelineMultisampleStateCreateInfo _multisampling;
	VkPipelineLayout _pipelineLayout;
	VkPipelineDepthStencilStateCreateInfo _depthStencil;
	VkPipelineRenderingCreateInfo _renderInfo;
	VkFormat _colorAttachmentFormat;

	bool setVertexInputState = false;

	PipelineBuilder() {
		clear();
	}

	// Inital Graphics Pipeline Options
	void set_shaders(VkShaderModule vertexShader, VkShaderModule fragmentShader);
	void set_input_topology(VkPrimitiveTopology topology);

	// Rasterizer Options
	void set_polygon_mode(VkPolygonMode mode);
	void set_cull_mode(VkCullModeFlags cullMode, VkFrontFace frontFace);

	// Multisampling Options
	void set_multisampling_none();

	// Color Blend Options
	void enable_blending_additive();
	void enable_blending_alphablend();
	void disable_blending();

	// Color / Depth Attachment
	void set_color_attachment_format(VkFormat format);
	void set_depth_format(VkFormat format);

	// Vertex Input Attributes
	VkVertexInputBindingDescription set_vertex_input_binding(uint32_t binding, uint32_t stride, VkVertexInputRate inputRate);
	VkVertexInputAttributeDescription set_vertex_input_attribute_description(uint32_t binding, uint32_t location, VkFormat format, uint32_t offset);
	void set_vertex_input_state(uint32_t bindingsSize, VkVertexInputBindingDescription* pBindings, uint32_t attributesSize, VkVertexInputAttributeDescription* pAttributes);

	// Depth Testing Options
	void enable_depthtest(bool depthWriteEnable, VkCompareOp op);
	void disable_depthtest();

	// Pipeline Builder Function
	VkPipeline build_pipeline(VkDevice device);

	// Reset Pipeline
	void clear();

private:
};