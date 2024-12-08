// vulkan_guide.h : Include file for standard system include files,
// or project specific include files.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <span>
#include <array>
#include <functional>
#include <deque>

#include <vulkan/vulkan.h>
#include <vulkan/vk_enum_string_helper.h>
#include <vk_mem_alloc.h>

#include <phy_particle.h>

#include <fmt/core.h>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>


#define VK_CHECK(x)                                                     \
    do {                                                                \
        VkResult err = x;                                               \
        if (err) {                                                      \
            fmt::println("Detected Vulkan error: {}", string_VkResult(err)); \
            abort();                                                    \
        }                                                               \
    } while (0)

class VulkanEngine;

struct AllocatedImage {
    VkImage image;
    VkImageView imageView;
    VmaAllocation allocation;
    VkExtent3D imageExtent;
    VkFormat imageFormat;
};

struct AllocatedBuffer {
    VkBuffer buffer;
    VmaAllocation allocation;
    VmaAllocationInfo info;
};

// Standard Vertex Object
struct Vertex {
    glm::vec3 position;
    float uv_x;
    glm::vec3 normal;
    float uv_y;
    glm::vec4 colour;
};

// GPU Mesh Data
struct GPUMeshBuffers {
    AllocatedBuffer indexBuffer;
    AllocatedBuffer vertexBuffer;
    VkDeviceAddress vertexBufferAddress;
};

// Containts Push Constants for Mesh Data
struct GPUDrawPushConstants {
    glm::mat4 worldMatrix;
    VkDeviceAddress vertexBuffer;
};

// Dynamic mesh Rendering Structures --------------------------------------------------------------

// Frustum Culling Struct
struct Bounds {
    glm::vec3 origin;
    float sphereRadius;
    glm::vec3 extents;
};

enum class MaterialPass :uint8_t {
    MainColour,
    Transparent,
    Other
};

struct MaterialPipeline {
    VkPipeline pipeline;
    VkPipelineLayout layout;
};

struct MaterialInstance {
    MaterialPipeline* pipeline;
    VkDescriptorSet materialSet;
    MaterialPass passType;
};

// Render Object - Main Object for meshes
struct RenderObject {
    uint32_t indexCount;
    uint32_t firstIndex;
    VkBuffer indexBuffer;

    MaterialInstance* material;
    Bounds bounds;
    glm::mat4 transform;
    VkDeviceAddress vertexBufferAddress;
};

// Full Draw Context - List of objects
struct DrawContext {
    std::vector<RenderObject> OpaqueSurfaces;
    std::vector<RenderObject> TransparentSurfaces;
};

// Interfaces
class IRenderable {
    virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx) = 0;
};

// Base node Class
struct Node : public IRenderable {

    // Parent pointer must be a weak pointer to avoid circular dependencies
    std::weak_ptr<Node> parent;

    // List of children
    std::vector<std::shared_ptr<Node>> children;

    // Matrix transformations
    glm::mat4 localTransform;
    glm::mat4 worldTransform;

    void refreshTransform(const glm::mat4& parentMatrix) {
        worldTransform = parentMatrix * localTransform;
        for (auto c : children) {
            c->refreshTransform(worldTransform);
        }
    }

    virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx) {
        // Draw Children
        for (auto& c : children) {
            c->Draw(topMatrix, ctx);
        }
    }
};

// GLTF Material Struct
struct GLTFMaterial {
    MaterialInstance data;
};

// GLTF Mesh Structs
struct GeoSurface {
    uint32_t startIndex;
    uint32_t count;

    Bounds bounds;
    std::shared_ptr<GLTFMaterial> material;
};

// Mesh Asset Struct
struct MeshAsset {
    std::string name;
    std::vector<GeoSurface> surfaces;
    GPUMeshBuffers meshBuffers;
};

// Mesh Node Object
struct MeshNode : public Node {
    std::shared_ptr<MeshAsset> mesh;

    virtual void Draw(const glm::mat4& topMatrix, DrawContext& ctx) override;
};



// Dynamic Particle System Structs ----------------------------------------------------------------

struct ParticleUniformBuffers {
    VkBuffer particles;
};

struct ParticlePipeline {
    VkPipelineLayout pipelineLayout;
    VkPipeline pipeline;
};

struct ParticleResources {
    VkBuffer buffer{ VK_NULL_HANDLE };
    VkDeviceMemory memory{ VK_NULL_HANDLE };

    void* mappedMemory;
    size_t size{ 0 };
};

// Particle Structures
struct ParticleSystem {
public:
    // Pointer to Vulkan Engine
    VulkanEngine* engine;

    // Particles Resources
    ParticleResources particleResources;

    // Shader Descriptor Sets
    VkDescriptorSetLayout particleDescriptorSetLayout;
    VkDescriptorSet particleDescriptorSet;

    // Particle Pipelines
    ParticlePipeline particlesPipeline;

    // Set of all Particles
    std::vector<Particle> particleList;

    // Mapped Memory of particle list
    AllocatedBuffer particleMemory;

    // Particle Texture
    AllocatedImage particleTexture;
    VkSampler particleTextureSampler;

    void init_particles();
    void set_particle(Particle* particle, glm::vec3 emitterPosition);
    void set_descriptor_sets();
    void create_pipelines();
    void draw_particles(VkCommandBuffer cmd);
    void clear_particles();
};