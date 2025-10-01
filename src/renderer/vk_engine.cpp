//> defines
#define VMA_IMPLEMENTATION
#define PARTICLE_COUNT 512


//> includes
#include <SDL.h>
#include <SDL_vulkan.h>

#include "vk_engine.h"
#include <vk_initializers.h>
#include <vk_types.h>
#include <vk_images.h>
#include <vk_mem_alloc.h>
#include <vk_pipelines.h>
#include "VkBootstrap.h"
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_vulkan.h"



#include "stb_image.h"

#include <string>
#include <chrono>
#include <thread>
#include <random>

// ========================================================================================================
//
//                                          VULKAN RIGID BODY APPLICATION
// 
// ========================================================================================================

VulkanPhysicsApplication::VulkanPhysicsApplication() : resolver(maxContacts*8)
{
    cData.contactArray = contacts;
}

RBDemo::RBDemo() : VulkanPhysicsApplication() {
    createObjects();
}

//void RBDemo::createObjects() {
//    // Create boxes
//    float x = 0.5f;
//    int y = 3;
//    float z = 0.5f;
//    for (Box* box = boxData; box < boxData+boxes; box++) {
//        box->setState(glm::vec3(x, y+10, z),
//            glm::quat(0.4, 0.5, 0.6, 0.2),
//            glm::vec3(1, 1, 1),
//            glm::vec3(0, 0, 0));
//
//        box->calculateInternals();
//
//        glm::mat4 transform = box->getTransform();
//
//        x += 0.5;
//        y += 5; 
//        z += 0.5;
//    }
//}
void RBDemo::createObjects() {
    // Create boxes
    float x = 0.5f;
    int y = 3;
    float z = 0.5f;
    for (Box* box = boxData; box < boxData + boxes; box++) {
        box->setState(
            Vector3(x, y+10, z),
            Quaternion(0.1, 0.15, 0.16, 0.12),
            //Quaternion(0, 0, 0, 0),
            Vector3(1, 1, 1),
            Vector3(0, 0, 0)
        );

        box->calculateInternals();

        x += 0.5;
        y += 5;
        z += 0.5;
    }
}

void RBDemo::update(float durationOfFrame) {
    // Find duration of last frame in seconds
    if (durationOfFrame <= 0.0f or !_isInitialized) {
        return;
    }
    else if (durationOfFrame > 0.05f) {
        //durationOfFrame = 0.001f; 
        durationOfFrame = 0.02f;
    }

    //printf("================== UPDATE ==================\n");

    // Update objects
    updateObjects(durationOfFrame);

    // Perform contact generation
    generateContacts();

    // Resolve detected contacts - EDGE-EDGE CONTACTS DO NOT WORK, BOX-BOX DOES NOT WORK
    resolver.resolveContacts(cData.contactArray, cData.contactCount, durationOfFrame);
}
void RBDemo::updateObjects(float durationOfFrame) {
    // Update physics of each box in turn
    for (Box* box = boxData; box < boxData + boxes; box++) {
        // Run physics - Integrate RigidBody = Calculate transform for box
        box->body->integrate(durationOfFrame);

        // Updates CollisionBox primitive not RigidBody
        box->calculateInternals();
        box->isOverlapping = false;

        

        // Debug
        //printf("POSITION UPDATE AT END OF UPDATE:\n");
        //printf("\r%s", glm::to_string(box->body->getTransform()[3]).c_str());
        //printf("\r%s", box->body->getTransform().data[11]);
        

    }

    // Update physics of each ball in turn
    // TODO
}
//void RBDemo::generateContacts() {
//    // Create ground plane
//    // GLM -> Y axis points up
//    Physics::CollisionPlane ground;
//    
//    //ground.direction = glm::vec3(0, 1, 0);
//    // plane.direction = glm::vec3(0, 0, 1);
//    ground.direction = Physics::Vector3(0, 1, 0);
//    ground.offset = 0;
//
//    // Makes use of early outs to avoid fine processing
//    // Set up collision data structure
//    cData.reset(maxContacts);
//    cData.friction = (real)0.9;
//    cData.restitution = (real)0.6;
//    cData.tolerance = (real)0.1;
//
//    // Perform exhaustive collision detection
//    glm::mat4 transform, otherTransform;
//    glm::vec3 position, otherPosition;
//
//    // Box collision Detection
//    for (Box* box = boxData; box < boxData + boxes; box++) {
//        // Check collisions with ground plane
//        if (!cData.hasMoreContacts()) return;
//        Physics::CollisionDetector::boxAndHalfSpace(*box, ground, &cData);
//
//        // Check collisions with every other box
//        /*for (Box* other = box + 1; other < boxData + boxes; other++) {
//            if (!cData.hasMoreContacts()) return;
//            Physics::CollisionDetector::boxAndBox(*box, *other, &cData);
//
//            if (Physics::IntersectionTests::boxAndBox(*box, *other)) {
//                box->isOverlapping = other->isOverlapping = true;
//            }
//        }*/
//
//        // Check collisions with every other ball
//        // TODO
//    }
//
//    // Ball Collision Detection
//}
void RBDemo::generateContacts() {
    // Note that this method makes a lot of use of early returns to avoid
    // processing lots of potential contacts that it hasn't got room to
    // store.

    // Create the ground plane data
    CollisionPlane plane;
    Vector3 d = Vector3(0, 1, 0);
    
    plane.direction = Vector3(0,1,0);
    plane.offset = 0;

    // Set up the collision data structure
    cData.reset(maxContacts);
    cData.friction = (real)0.9;
    cData.restitution = (real)0.6;
    cData.tolerance = (real)0.1;

    // Perform exhaustive collision detection
    Matrix4 transform, otherTransform;
    Vector3 position, otherPosition;
    for (Box* box = boxData; box < boxData + boxes; box++)
    {
        // Check for collisions with the ground plane
        if (!cData.hasMoreContacts()) return;
        CollisionDetector::boxAndHalfSpace(*box, plane, &cData);

        // Check for collisions with each other box
        /*for (Box* other = box + 1; other < boxData + boxes; other++)
        {
            if (!cData.hasMoreContacts()) return;
            cyclone::CollisionDetector::boxAndBox(*box, *other, &cData);

            if (cyclone::IntersectionTests::boxAndBox(*box, *other))
            {
                box->isOverlapping = other->isOverlapping = true;
            }
        }*/

        // Check for collisions with each ball
        /*for (Ball* other = ballData; other < ballData + balls; other++)
        {
            if (!cData.hasMoreContacts()) return;
            cyclone::CollisionDetector::boxAndSphere(*box, *other, &cData);
        }*/
    }

    //for (Ball* ball = ballData; ball < ballData + balls; ball++)
    //{
    //    // Check for collisions with the ground plane
    //    if (!cData.hasMoreContacts()) return;
    //    cyclone::CollisionDetector::sphereAndHalfSpace(*ball, plane, &cData);

    //    for (Ball* other = ball + 1; other < ballData + balls; other++)
    //    {
    //        // Check for collisions with the ground plane
    //        if (!cData.hasMoreContacts()) return;
    //        cyclone::CollisionDetector::sphereAndSphere(*ball, *other, &cData);
    //    }
    //}
}
// ========================================================================================================
//
//                                              PHYSICS ARCHITECTURE
// 
// ========================================================================================================

//Box::Box() : isOverlapping(false) {
//    body = new Physics::RigidBody();
//}
//
//Box::~Box() {
//    delete body;
//}
//void Box::setState(const glm::vec3& position, const glm::quat& orientation, const glm::vec3& extents, const glm::vec3& velocity) {
//    
//    offset = glm::mat4(1);
//
//    body->setPosition(position);
//    body->setRotation(glm::vec3(0, 0, 0));
//    body->setOrientation(orientation);
//    body->setVelocity(velocity);
//    
//    halfSize = extents;
//
//    float mass = halfSize.x * halfSize.y * halfSize.z * 8.0f;
//    body->setMass(mass);
//
//    glm::mat3 tensor = glm::mat3(0.f);
//    glm::vec3 squares = glm::vec3(
//        halfSize.x * halfSize.x,
//        halfSize.y * halfSize.y,
//        halfSize.z * halfSize.z);
//    tensor[0][0] = 0.3f * mass * (squares.y + squares.z);
//    tensor[1][1] = 0.3f * mass * (squares.x + squares.z);
//    tensor[2][2] = 0.3f * mass * (squares.x + squares.y);
//
//    body->setInertiaTensor(tensor);
//
//    body->setLinearDamping(0.95f);
//    body->setAngularDamping(0.8f);
//    body->clearAccumulators();
//
//    // Artificial Gravity
//    body->setAcceleration(0, -10.0f, 0);
//
//    body->setAwake();
//
//    body->calculateDerivedData();
//}

// ========================================================================================================
//
//                                                 VULKAN ENGINE
// 
// ========================================================================================================

VulkanEngine* loadedEngine = nullptr;

VulkanEngine& VulkanEngine::Get() { return *loadedEngine; }

// ========================================================================================================
//
//                                              GLOBAL FUNCTIONALITY
// 
// ========================================================================================================

bool is_visible(const RenderObject& obj, const glm::mat4& viewproj) {
    std::array<glm::vec3, 8> corners{
        glm::vec3 { 1, 1, 1 },
        glm::vec3 { 1, 1, -1 },
        glm::vec3 { 1, -1, 1 },
        glm::vec3 { 1, -1, -1 },
        glm::vec3 { -1, 1, 1 },
        glm::vec3 { -1, 1, -1 },
        glm::vec3 { -1, -1, 1 },
        glm::vec3 { -1, -1, -1 },
    };

    glm::mat4 matrix = viewproj * obj.transform;

    glm::vec3 min = { 1.5, 1.5, 1.5 };
    glm::vec3 max = { -1.5, -1.5, -1.5 };

    for (int c = 0; c < 8; c++) {
        // Project each corner into clip space
        glm::vec4 v = matrix * glm::vec4(obj.bounds.origin + (corners[c] * obj.bounds.extents), 1.f);

        // Perspective Correction
        v.x = v.x / v.w;
        v.y = v.y / v.w;
        v.z = v.z / v.w;

        min = glm::min(glm::vec3{ v.x, v.y, v.z }, min);
        max = glm::max(glm::vec3{ v.x, v.y, v.z }, max);
    }

    // Check the clip space box is within the view
    if (min.z > 1.f || max.z < 0.f || min.x > 1.f || max.x < -1.f || min.y > 1.f || max.y < -1.f) {
        return false;
    }
    else {
        return true;
    }
}

// Initialisation Functionality ---------------------------------------------------------------------------

void VulkanEngine::init()
{
    // only one engine initialization is allowed with the application.
    assert(loadedEngine == nullptr);
    loadedEngine = this;

    // We initialize SDL and create a window with it.
    SDL_Init(SDL_INIT_VIDEO);

    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);

    _window = SDL_CreateWindow(
        "Vulkan Engine",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        _windowExtent.width,
        _windowExtent.height,
        window_flags);

    init_vulkan();
    init_swapchain();
    init_commands();
    //init_compute_commands();
    init_sync_structures();
    init_descriptors();
    init_pipelines();
    init_imgui();
    init_default_mesh_data();
    init_default_data();
    //init_particle_system();
    init_physics();

    // Initialise camera coordinates
    mainCamera.velocity = glm::vec3(0.f);
    mainCamera.position = glm::vec3(0, 2, 7);
    //mainCamera.position = glm::vec3(30.f, -00.f, -085.f);
    mainCamera.pitch = 0;
    mainCamera.yaw = 0;

    // Scene Data
    std::string structurePath = { "..\\assets\\initial.glb" };
    auto structureFile = loadGltf(this, structurePath);
    assert(structureFile.has_value());
    loadedScenes["initial"] = *structureFile;

    // everything went fine
    _isInitialized = true;
    RigidBodyPhysicsEngine->_isInitialized = true;
}

void VulkanEngine::init_vulkan() {
    vkb::InstanceBuilder builder;

    auto inst_ret = builder.set_app_name("Example Vulkan Application")
        .request_validation_layers(bUseValidationLayers)
        .use_default_debug_messenger()
        .require_api_version(1, 3, 0)
        .build();

    vkb::Instance vkb_inst = inst_ret.value();

    _instance = vkb_inst.instance;
    _debug_messenger = vkb_inst.debug_messenger;

    SDL_Vulkan_CreateSurface(_window, _instance, &_surface);

    VkPhysicalDeviceVulkan13Features features{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    features.dynamicRendering = true;
    features.synchronization2 = true;

    VkPhysicalDeviceVulkan12Features features12{ .sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES };
    features12.bufferDeviceAddress = true;
    features12.descriptorIndexing = true;

    vkb::PhysicalDeviceSelector selector{vkb_inst};
    vkb::PhysicalDevice physicalDevice = selector
        .set_minimum_version(1, 3)
        .set_required_features_13(features)
        .set_required_features_12(features12)
        .set_surface(_surface)
        .select()
        .value();

    vkb::DeviceBuilder deviceBuilder(physicalDevice);
    vkb::Device vkbDevice = deviceBuilder.build().value();

    _device = vkbDevice.device;
    _chosenGPU = physicalDevice.physical_device;

    auto queue_families = physicalDevice.get_queue_families();
    for (uint32_t i = 0; i < static_cast<uint32_t>(queue_families.size()); i++) {
        if ((queue_families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && (queue_families[i].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
            // Find the first queue family with graphics operations supported and compute operations supported
            _graphicsQueueFamily = i;
            _computeQueueFamily = i;
        }
    }

    _graphicsQueue = vkbDevice.get_queue(vkb::QueueType::graphics).value();
    //_graphicsQueueFamily = vkbDevice.get_queue_index(vkb::QueueType::graphics).value();

    // Obtain Compute queue
    vkGetDeviceQueue(_device, _computeQueueFamily, 0, &_computeQueue);

    VmaAllocatorCreateInfo allocatorInfo = {};
    allocatorInfo.physicalDevice = _chosenGPU;
    allocatorInfo.device = _device;
    allocatorInfo.instance = _instance;
    allocatorInfo.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
    vmaCreateAllocator(&allocatorInfo, &_allocator);

    _mainDeletionQueue.push_function([&]() {
        vmaDestroyAllocator(_allocator);
    });


}

void VulkanEngine::init_swapchain() {
    create_swapchain(_windowExtent.width, _windowExtent.height);

    VkExtent3D drawImageExtent = {
        _windowExtent.width,
        _windowExtent.height,
        1
    };


    // Draw Image Allocation
    _drawImage.imageFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
    _drawImage.imageExtent = drawImageExtent;

    VkImageUsageFlags drawImageUsages{};
    drawImageUsages |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_STORAGE_BIT;
    drawImageUsages |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    VkImageCreateInfo rimg_info = vkinit::image_create_info(_drawImage.imageFormat, drawImageUsages, drawImageExtent);

    VmaAllocationCreateInfo rimg_allocinfo = {};
    rimg_allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    rimg_allocinfo.requiredFlags = VkMemoryPropertyFlags(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    
    // allocator draw image
    vmaCreateImage(_allocator, &rimg_info, &rimg_allocinfo, &_drawImage.image, &_drawImage.allocation, nullptr);

    VkImageViewCreateInfo rview_info = vkinit::imageview_create_info(_drawImage.imageFormat, _drawImage.image, VK_IMAGE_ASPECT_COLOR_BIT);
    
    VK_CHECK(vkCreateImageView(_device, &rview_info, nullptr, &_drawImage.imageView));

    // Depth Image Allocation
    _depthImage.imageFormat = VK_FORMAT_D32_SFLOAT;
    _depthImage.imageExtent = drawImageExtent;

    VkImageUsageFlags depthImageUsages{};
    depthImageUsages |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;

    VkImageCreateInfo dimg_info = vkinit::image_create_info(_depthImage.imageFormat, depthImageUsages, drawImageExtent);

    // allocate depth image
    vmaCreateImage(_allocator, &dimg_info, &rimg_allocinfo, &_depthImage.image, &_depthImage.allocation, nullptr);

    VkImageViewCreateInfo dview_info = vkinit::imageview_create_info(_depthImage.imageFormat, _depthImage.image, VK_IMAGE_ASPECT_DEPTH_BIT);

    VK_CHECK(vkCreateImageView(_device, &dview_info, nullptr, &_depthImage.imageView));

    _mainDeletionQueue.push_function([=]() {
        vkDestroyImageView(_device, _drawImage.imageView, nullptr);
        vmaDestroyImage(_allocator, _drawImage.image, _drawImage.allocation);

        vkDestroyImageView(_device, _depthImage.imageView, nullptr);
        vmaDestroyImage(_allocator, _depthImage.image, _depthImage.allocation);
    });
}

void VulkanEngine::init_commands() {
    VkCommandPoolCreateInfo commandPoolInfo = {};
    commandPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    commandPoolInfo.pNext = nullptr;
    commandPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    commandPoolInfo.queueFamilyIndex = _graphicsQueueFamily;

    for (int i = 0; i < FRAME_OVERLAP; i++) {
        VK_CHECK(vkCreateCommandPool(_device, &commandPoolInfo, nullptr, &_frames[i]._commandPool));

        VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_frames[i]._commandPool, 1);

        VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAllocInfo, &_frames[i]._mainCommandBuffer));
    }

    VK_CHECK(vkCreateCommandPool(_device, &commandPoolInfo, nullptr, &_immCommandPool));

    VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_immCommandPool, 1);

    VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAllocInfo, &_immCommandBuffer));

    _mainDeletionQueue.push_function([=]() {
        vkDestroyCommandPool(_device, _immCommandPool, nullptr);
    });
}

void VulkanEngine::init_compute_commands() {
    /*VkCommandPoolCreateInfo commandPoolInfo = {};
    commandPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    commandPoolInfo.pNext = nullptr;
    commandPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    commandPoolInfo.queueFamilyIndex = _computeQueueFamily;

    for (int i = 0; i < FRAME_OVERLAP; i++) {
        VK_CHECK(vkCreateCommandPool(_device, &commandPoolInfo, nullptr, &_frames[i]._computePool));

        VkCommandBufferAllocateInfo cmdAllocInfo = vkinit::command_buffer_allocate_info(_frames[i]._computePool, 1);

        VK_CHECK(vkAllocateCommandBuffers(_device, &cmdAllocInfo, &_frames[i]._mainComputeBuffer));
    }*/
}

void VulkanEngine::init_sync_structures() {
    VkFenceCreateInfo fenceCreateInfo = vkinit::fence_create_info(VK_FENCE_CREATE_SIGNALED_BIT);
    VkSemaphoreCreateInfo semaphoreCreateInfo = vkinit::semaphore_create_info();

    for (int i = 0; i < FRAME_OVERLAP; i++) {
        VK_CHECK(vkCreateFence(_device, &fenceCreateInfo, nullptr, &_frames[i]._renderFence));
        VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_frames[i]._swapchainSemaphore));
        VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_frames[i]._renderSemaphore));
        
        // Create Particle Compute Sync Objects per frame
        /*VK_CHECK(vkCreateFence(_device, &fenceCreateInfo, nullptr, &_frames[i]._computeInFlightFence));
        VK_CHECK(vkCreateSemaphore(_device, &semaphoreCreateInfo, nullptr, &_frames[i]._computeFinishedSemaphore));
        */
    }

    VK_CHECK(vkCreateFence(_device, &fenceCreateInfo, nullptr, &_immFence));
    _mainDeletionQueue.push_function([=]() {
        vkDestroyFence(_device, _immFence, nullptr);
    });
}

void VulkanEngine::init_descriptors()
{
    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> sizes = { 
        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1 } 
    };
 
    globalDescriptorAllocator.init(_device, 10, sizes);

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE);
        _drawImageDescriptorLayout = builder.build(_device, VK_SHADER_STAGE_COMPUTE_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
        _gpuSceneDataDescriptorLayout = builder.build(_device, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);
    }

    {
        DescriptorLayoutBuilder builder;
        builder.add_binding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
        _singleImageDescriptorLayout = builder.build(_device, VK_SHADER_STAGE_FRAGMENT_BIT);
    }

    _drawImageDescriptors = globalDescriptorAllocator.allocate(_device, _drawImageDescriptorLayout);

    DescriptorWriter writer;
    writer.write_image(0, _drawImage.imageView, VK_NULL_HANDLE, VK_IMAGE_LAYOUT_GENERAL, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE);
    writer.update_set(_device, _drawImageDescriptors);

    _mainDeletionQueue.push_function([&]() {
        globalDescriptorAllocator.destroy_pools(_device);
        
        vkDestroyDescriptorSetLayout(_device, _singleImageDescriptorLayout, nullptr);
        vkDestroyDescriptorSetLayout(_device, _gpuSceneDataDescriptorLayout, nullptr);
        vkDestroyDescriptorSetLayout(_device, _drawImageDescriptorLayout, nullptr);

    });

    // Initialise global descriptor set
    for (int i = 0; i < FRAME_OVERLAP; i++) {
        std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> frame_sizes = {
            {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 3},
            {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3},
            {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 3},
            {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4}
        };

        _frames[i]._frameDescriptors = DescriptorAllocatorGrowable{};
        _frames[i]._frameDescriptors.init(_device, 1000, frame_sizes);

        _mainDeletionQueue.push_function([&, i]() {
            _frames[i]._frameDescriptors.destroy_pools(_device);
        });
    }

}

void VulkanEngine::init_pipelines() {
    // Compute Pipelines
    init_background_pipelines();

    // Graphics Pipelines
    //init_mesh_pipeline();
    metalRoughMaterial.build_pipelines(this);
}

void VulkanEngine::init_background_pipelines() {
    VkPipelineLayoutCreateInfo computeLayout{};
    computeLayout.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    computeLayout.pNext = nullptr;
    computeLayout.pSetLayouts = &_drawImageDescriptorLayout;
    computeLayout.setLayoutCount = 1;

    VkPushConstantRange pushConstant{};
    pushConstant.offset = 0;
    pushConstant.size = sizeof(ComputePushConstants);
    pushConstant.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    computeLayout.pPushConstantRanges = &pushConstant;
    computeLayout.pushConstantRangeCount = 1;

    VK_CHECK(vkCreatePipelineLayout(_device, &computeLayout, nullptr, &_gradientPipelineLayout ));

    VkShaderModule gradientShader;
    if (!vkutil::load_shader_module("../shaders/gradient_color.comp.spv", _device, &gradientShader))
    {
        fmt::print("Error when building the compute shader \n");
    }

    VkShaderModule skyShader;
    if (!vkutil::load_shader_module("../shaders/sky.comp.spv", _device, &skyShader)) {
        fmt::print("Error when building the sky shader \n");
    }

    VkPipelineShaderStageCreateInfo stageInfo{};
    stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stageInfo.pNext = nullptr;
    stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    stageInfo.module = gradientShader;
    stageInfo.pName = "main";

    VkComputePipelineCreateInfo computePipelineCreateInfo{};
    computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    computePipelineCreateInfo.pNext = nullptr;
    computePipelineCreateInfo.layout = _gradientPipelineLayout;
    computePipelineCreateInfo.stage = stageInfo;

    ComputeEffect gradient;
    gradient.layout = _gradientPipelineLayout;
    gradient.name = "gradient";
    gradient.data = {};
    gradient.data.data1 = glm::vec4(1, 0, 0, 1);
    gradient.data.data2 = glm::vec4(0, 0, 1, 1);

    VK_CHECK(vkCreateComputePipelines(_device, VK_NULL_HANDLE, 1, &computePipelineCreateInfo, nullptr, &gradient.pipeline));

    computePipelineCreateInfo.stage.module = skyShader;

    ComputeEffect sky;
    gradient.layout = _gradientPipelineLayout;
    gradient.name = "sky";
    gradient.data = {};
    gradient.data.data1 = glm::vec4(0.1, 0.2, 0.4, 0.97);

    VK_CHECK(vkCreateComputePipelines(_device, VK_NULL_HANDLE, 1, &computePipelineCreateInfo, nullptr, &sky.pipeline));

    backgroundEffects.push_back(gradient);
    backgroundEffects.push_back(sky);

    vkDestroyShaderModule(_device, gradientShader, nullptr);
    vkDestroyShaderModule(_device, skyShader, nullptr);
    _mainDeletionQueue.push_function([=]() {
        vkDestroyPipelineLayout(_device, _gradientPipelineLayout, nullptr);
        vkDestroyPipeline(_device, sky.pipeline, nullptr);
        vkDestroyPipeline(_device, gradient.pipeline, nullptr);
    });
}

void VulkanEngine::init_mesh_pipeline() {
    
    VkShaderModule triangleVertShader;
    if (!vkutil::load_shader_module("../shaders/colored_triangle_mesh_vert.spv", _device, &triangleVertShader)) {
        fmt::print("Error when building the triangle vertex shader module");
    }

    VkShaderModule triangleFragShader;
    if (!vkutil::load_shader_module("../shaders/tex_image.spv", _device, &triangleFragShader)) {
        fmt::print("Error when building the triangle fragment shader module");
    }

    VkPushConstantRange bufferRange{};
    bufferRange.offset = 0;
    bufferRange.size = sizeof(GPUDrawPushConstants);
    bufferRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    VkPipelineLayoutCreateInfo pipeline_layout_info = vkinit::pipeline_layout_create_info();
    pipeline_layout_info.pPushConstantRanges = &bufferRange;
    pipeline_layout_info.pushConstantRangeCount = 1;
    pipeline_layout_info.pSetLayouts = &_singleImageDescriptorLayout;
    pipeline_layout_info.setLayoutCount = 1;

    VK_CHECK(vkCreatePipelineLayout(_device, &pipeline_layout_info, nullptr, &_meshPipelineLayout));

    PipelineBuilder pipelineBuilder;

    //use the triangle layout we created
    pipelineBuilder._pipelineLayout = _meshPipelineLayout;
    //connecting the vertex and pixel shaders to the pipeline
    pipelineBuilder.set_shaders(triangleVertShader, triangleFragShader);
    //it will draw triangles
    pipelineBuilder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
    //filled triangles
    pipelineBuilder.set_polygon_mode(VK_POLYGON_MODE_FILL);
    //no backface culling
    pipelineBuilder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
    //no multisampling
    pipelineBuilder.set_multisampling_none();
    //no blending
    pipelineBuilder.disable_blending();
    //pipelineBuilder.enable_blending_additive();

    pipelineBuilder.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);

    //connect the image format we will draw into, from draw image
    pipelineBuilder.set_color_attachment_format(_drawImage.imageFormat);
    pipelineBuilder.set_depth_format(_depthImage.imageFormat);

    //finally build the pipeline
    _meshPipeline = pipelineBuilder.build_pipeline(_device);

    //clean structures
    vkDestroyShaderModule(_device, triangleFragShader, nullptr);
    vkDestroyShaderModule(_device, triangleVertShader, nullptr);

    _mainDeletionQueue.push_function([&]() {
        vkDestroyPipelineLayout(_device, _meshPipelineLayout, nullptr);
        vkDestroyPipeline(_device, _meshPipeline, nullptr);
    });
}

void VulkanEngine::init_default_mesh_data() {
    // testMeshes = loadGltfMeshes(this, "..\\assets\\basicmesh.glb").value();
}

void VulkanEngine::init_default_data() {
    // Default Textures
    uint32_t white = glm::packUnorm4x8(glm::vec4(1, 1, 1, 1));
    _whiteImage = create_image((void*)&white, VkExtent3D{1, 1, 1}, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);

    uint32_t grey = glm::packUnorm4x8(glm::vec4(0.66f, 0.66f, 0.66f, 1));
    _greyImage = create_image((void*)&grey, VkExtent3D{ 1, 1, 1 }, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);

    uint32_t black = glm::packUnorm4x8(glm::vec4(0, 0, 0, 0));
    _blackImage = create_image((void*)&black, VkExtent3D{ 1, 1, 1 }, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);

    uint32_t magenta = glm::packUnorm4x8(glm::vec4(1, 0, 1, 1));
    std::array<uint32_t, 16 *16> pixels;
    for (int x = 0; x < 16; x++) {
        for (int y = 0; y < 16; y++) {
            pixels[y*16 + x] = ((x%2) ^ (y%2)) ? magenta : black;
        }
    }
    _errorCheckerboardImage = create_image(pixels.data(), VkExtent3D{ 16, 16, 1 }, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_USAGE_SAMPLED_BIT);

    VkSamplerCreateInfo sampl = {.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};

    sampl.magFilter = VK_FILTER_NEAREST;
    sampl.minFilter = VK_FILTER_NEAREST;

    vkCreateSampler(_device, &sampl, nullptr, &_defaultSamplerNearest);

    sampl.magFilter = VK_FILTER_LINEAR;
    sampl.minFilter = VK_FILTER_LINEAR;

    vkCreateSampler(_device, &sampl, nullptr, &_defaultSamplerLinear);

    _mainDeletionQueue.push_function([&]() {
        vkDestroySampler(_device, _defaultSamplerNearest, nullptr);
        vkDestroySampler(_device, _defaultSamplerLinear, nullptr);

        destroy_image(_whiteImage);
        destroy_image(_greyImage);
        destroy_image(_blackImage);
        destroy_image(_errorCheckerboardImage);
    });

    // Default Materials
    GLTFMetallic_Roughness::MaterialResources materialResources;
    materialResources.colourImage = _whiteImage;
    materialResources.colourSampler = _defaultSamplerLinear;
    materialResources.metalRoughImage = _whiteImage;
    materialResources.metalRoughSampler = _defaultSamplerLinear;

    // set uniform buffer for material data
    AllocatedBuffer materialConstants = create_buffer(sizeof(GLTFMetallic_Roughness::MaterialConstants), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);

    // write the buffer
    GLTFMetallic_Roughness::MaterialConstants* sceneUniformData = (GLTFMetallic_Roughness::MaterialConstants*)materialConstants.allocation->GetMappedData();
    sceneUniformData->colourFactors = glm::vec4{1,1,1,1};
    sceneUniformData->metalRoughFactors = glm::vec4{ 1,0.5,0,0 };
    
    // Delete buffer
    _mainDeletionQueue.push_function([=, this]() {
        destroy_buffer(materialConstants);
    });

    materialResources.dataBuffer = materialConstants.buffer;
    materialResources.dataBufferOffset = 0;

    defaultData = metalRoughMaterial.write_material(_device, MaterialPass::MainColour, materialResources, globalDescriptorAllocator);

    // Add default material to meshes
    /*for (auto& m : testMeshes) {
        std::shared_ptr<MeshNode> newNode = std::make_shared<MeshNode>();
        newNode->mesh = m;

        newNode->localTransform = glm::mat4{ 1.f };
        newNode->worldTransform = glm::mat4{ 1.f };

        for (auto& s : newNode->mesh->surfaces) {
            s.material = std::make_shared<GLTFMaterial>(defaultData);
        }

        loadedNodes[m->name] = std::move(newNode);
    }*/
}

void VulkanEngine::init_imgui() {
    // Descriptor Pool for IMGUI - Pool size is oversized but copied from docs
    VkDescriptorPoolSize pool_sizes[] = {
        { VK_DESCRIPTOR_TYPE_SAMPLER, 1000 },
        { VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 },
        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 },
        { VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 }
    };

    VkDescriptorPoolCreateInfo pool_info = {};
    pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    pool_info.maxSets = 1000;
    pool_info.poolSizeCount = (uint32_t)std::size(pool_sizes);
    pool_info.pPoolSizes = pool_sizes;

    VkDescriptorPool imguiPool;
    VK_CHECK(vkCreateDescriptorPool(_device, &pool_info, nullptr, &imguiPool));

    // ImGUI library Initialisation
    ImGui::CreateContext();

    // Initialise ImGui for SDL
    ImGui_ImplSDL2_InitForVulkan(_window);

    // Initialise ImGui for Vulkan
    ImGui_ImplVulkan_InitInfo init_info = {};
    init_info.Instance = _instance;
    init_info.PhysicalDevice = _chosenGPU;
    init_info.Device = _device;
    init_info.Queue = _graphicsQueue;
    init_info.DescriptorPool = imguiPool;
    init_info.MinImageCount = 3;
    init_info.ImageCount = 3;
    init_info.UseDynamicRendering = true;

    // Dynamic Rendering Parameters for ImGUI to use
    init_info.PipelineRenderingCreateInfo = { .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO };
    init_info.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
    init_info.PipelineRenderingCreateInfo.pColorAttachmentFormats = &_swapchainImageFormat;
    
    // Anti-Aliasing
    init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;

    ImGui_ImplVulkan_Init(&init_info);
    ImGui_ImplVulkan_CreateFontsTexture();

    _mainDeletionQueue.push_function([=]() {
        ImGui_ImplVulkan_Shutdown();
        vkDestroyDescriptorPool(_device, imguiPool, nullptr);
    });
}

void VulkanEngine::init_physics() {
    RigidBodyPhysicsEngine = new RBDemo();
}

// Swapchain Functionality --------------------------------------------------------------------------------

void VulkanEngine::create_swapchain(uint32_t width, uint32_t height) {
    vkb::SwapchainBuilder swapchainBuilder{ _chosenGPU, _device, _surface };

    _swapchainImageFormat = VK_FORMAT_B8G8R8A8_UNORM;

    vkb::Swapchain vkbSwapchain = swapchainBuilder
        .set_desired_format(VkSurfaceFormatKHR{ .format = _swapchainImageFormat, .colorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR })
        .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
        .set_desired_extent(width, height)
        .add_image_usage_flags(VK_IMAGE_USAGE_TRANSFER_DST_BIT)
        .build()
        .value();

    _swapchainExtent = vkbSwapchain.extent;
    _swapchain = vkbSwapchain.swapchain;
    _swapchainImages = vkbSwapchain.get_images().value();
    _swapchainImageViews = vkbSwapchain.get_image_views().value();
}

void VulkanEngine::destroy_swapchain() {
    vkDestroySwapchainKHR(_device, _swapchain, nullptr);

    for (int i = 0; i < _swapchainImageViews.size(); i++) {
        vkDestroyImageView(_device, _swapchainImageViews[i], nullptr);
    }
}

void VulkanEngine::resize_swapchain() {
    vkDeviceWaitIdle(_device);

    destroy_swapchain();

    int w, h;
    SDL_GetWindowSize(_window, &w, &h);
    _windowExtent.width = w;
    _windowExtent.height = h;

    create_swapchain(_windowExtent.width, _windowExtent.height);

    resize_requested = false;
}

// Draw Functionality -------------------------------------------------------------------------------------
void VulkanEngine::draw()
{
    // Begin Clock
    auto start = std::chrono::system_clock::now();

    update_scene();

    // Compute Submission ---------------------------------------------------------------------------
    // NOT USED FOR NOW
    /*VkCommandBufferBeginInfo cmdBeginInfo_compute = vkinit::command_buffer_begin_info(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
    VkCommandBuffer comp = get_current_frame()._mainComputeBuffer;

    VK_CHECK(vkWaitForFences(_device, 1, &get_current_frame()._computeInFlightFence, VK_TRUE, UINT64_MAX));
    update_scene();
    VK_CHECK(vkResetFences(_device, 1, &get_current_frame()._computeInFlightFence));
    VK_CHECK(vkResetCommandBuffer(comp, 0));
    VK_CHECK(vkBeginCommandBuffer(comp, &cmdBeginInfo_compute));
    draw_compute_particles(comp);
    VK_CHECK(vkEndCommandBuffer(comp));

    VkCommandBufferSubmitInfo cmdInfo_compute = vkinit::command_buffer_submit_info(comp);
    VkSemaphoreSubmitInfo signalInfo_compute = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT_KHR, get_current_frame()._computeFinishedSemaphore);
    VkSubmitInfo2 submit_compute = vkinit::submit_info(&cmdInfo_compute, &signalInfo_compute, nullptr);
    VK_CHECK(vkQueueSubmit2(_computeQueue, 1, &submit_compute, get_current_frame()._computeInFlightFence));*/

    // Graphics Submission ---------------------------------------------------------------------------
    VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
    VkCommandBuffer cmd = get_current_frame()._mainCommandBuffer;

    VK_CHECK(vkWaitForFences(_device, 1, &get_current_frame()._renderFence, true, 1000000000));
    get_current_frame()._deletionQueue.flush();
    get_current_frame()._frameDescriptors.clear_pools(_device);
    VK_CHECK(vkResetFences(_device, 1, &get_current_frame()._renderFence));

    uint32_t swapchainImageIndex;
    VkResult e = vkAcquireNextImageKHR(_device, _swapchain, 1000000000, get_current_frame()._swapchainSemaphore, nullptr, &swapchainImageIndex);
    if (e == VK_ERROR_OUT_OF_DATE_KHR) {
        resize_requested = true;
        return;
    }

    //VkCommandBuffer cmd = get_current_frame()._mainCommandBuffer;
    VK_CHECK(vkResetCommandBuffer(cmd, 0));

    // Set Up new Image Extent
    _drawExtent.width = std::min(_swapchainExtent.width, _drawImage.imageExtent.width) * renderScale;
    _drawExtent.height = std::min(_swapchainExtent.height, _drawImage.imageExtent.height) * renderScale;

    // BEGIN Command Buffer Recording
    VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));
    
    //draw_compute_particles(cmd);
    // Transition principle draw image to General, Draw onto it, then transition to SRC and then Transition image to swapchain layout
    vkutil::transition_image(cmd, _drawImage.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);
    draw_background(cmd);
    vkutil::transition_image(cmd, _drawImage.image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    vkutil::transition_image(cmd, _depthImage.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
    draw_geometry(cmd);
    // draw_graphics_particles(cmd);
    vkutil::transition_image(cmd, _drawImage.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
    vkutil::transition_image(cmd, _swapchainImages[swapchainImageIndex], VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

    // Copy draw image to Swapchain image & transition to Presentable Layout
    vkutil::copy_image_to_image(cmd, _drawImage.image, _swapchainImages[swapchainImageIndex], _drawExtent, _swapchainExtent);
    vkutil::transition_image(cmd, _swapchainImages[swapchainImageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);

    // Draw ImGUI into Swapchain Image
    draw_imgui(cmd, _swapchainImageViews[swapchainImageIndex]);

    // Set Swapchain Image Layout to Present So we can draw it
    vkutil::transition_image(cmd, _swapchainImages[swapchainImageIndex], VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);

    // END Command Buffer Recording
    VK_CHECK(vkEndCommandBuffer(cmd));

    // Submit Command Buffer
    VkCommandBufferSubmitInfo cmdInfo = vkinit::command_buffer_submit_info(cmd);
    VkSemaphoreSubmitInfo waitInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, get_current_frame()._swapchainSemaphore);
    VkSemaphoreSubmitInfo signalInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT, get_current_frame()._renderSemaphore);
    VkSubmitInfo2 submit = vkinit::submit_info(&cmdInfo, &signalInfo, &waitInfo);
    VK_CHECK(vkQueueSubmit2(_graphicsQueue, 1, &submit, get_current_frame()._renderFence));

    /*VkCommandBufferSubmitInfo cmdInfo = vkinit::command_buffer_submit_info(cmd);
    VkSemaphoreSubmitInfo waitInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT_KHR, get_current_frame()._swapchainSemaphore);
    VkSemaphoreSubmitInfo waitInfoCompute = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT, get_current_frame()._computeFinishedSemaphore);
    VkSemaphoreSubmitInfo waitInfos[] = { waitInfoCompute, waitInfo };
    VkSemaphoreSubmitInfo signalInfo = vkinit::semaphore_submit_info(VK_PIPELINE_STAGE_2_ALL_GRAPHICS_BIT, get_current_frame()._renderSemaphore);
    VkSubmitInfo2 submit = vkinit::submit_info_multiple(&cmdInfo, &signalInfo, waitInfos, 2);
    VK_CHECK(vkQueueSubmit2(_graphicsQueue, 1, &submit, get_current_frame()._renderFence));*/

    // Present Image to Screen
    VkPresentInfoKHR presentInfo = {};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.pNext = nullptr;
    presentInfo.pSwapchains = &_swapchain;
    presentInfo.swapchainCount = 1;
    presentInfo.pWaitSemaphores = &get_current_frame()._renderSemaphore;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pImageIndices = &swapchainImageIndex;

    VkResult presentResult = vkQueuePresentKHR(_graphicsQueue, &presentInfo);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR) {
        resize_requested = true;
    }

    auto end = std::chrono::system_clock::now();

    // Convert to microseconds (int) and then come back to milliseconds - Update delta time
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Increase Frame Number
    _frameNumber++;
}

void VulkanEngine::draw_background(VkCommandBuffer cmd) {
    ComputeEffect& effect = backgroundEffects[currentBackgroundEffect];

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, effect.pipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, _gradientPipelineLayout, 0, 1, &_drawImageDescriptors, 0, nullptr);
    vkCmdPushConstants(cmd, _gradientPipelineLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(ComputePushConstants), &effect.data);
    vkCmdDispatch(cmd, std::ceil(_drawExtent.width / 16.0), std::ceil(_drawExtent.height / 16.0), 1);
}

void VulkanEngine::draw_geometry(VkCommandBuffer cmd) {
    std::vector<uint32_t> opaque_draws;
    opaque_draws.reserve(mainDrawContext.OpaqueSurfaces.size());

    for (uint32_t i = 0; i < mainDrawContext.OpaqueSurfaces.size(); i++) {
        if (is_visible(mainDrawContext.OpaqueSurfaces[i], sceneData.viewproj)) {
            opaque_draws.push_back(i);
        }
    }

    // Sort opaque surfaces by material and mesh
    std::sort(opaque_draws.begin(), opaque_draws.end(), [&](const auto& iA, const auto& iB) {
        const RenderObject& A = mainDrawContext.OpaqueSurfaces[iA];
        const RenderObject& B = mainDrawContext.OpaqueSurfaces[iB];

        if (A.material == B.material) {
            return A.indexBuffer < B.indexBuffer;
        }
        else {
            return A.material < B.material;
        }
    });

    // Reset counters
    stats.drawcall_count = 0;
    stats.triangle_count = 0;

    // Begin Clock
    auto start = std::chrono::system_clock::now();

    VkRenderingAttachmentInfo colorAttachment = vkinit::attachment_info(_drawImage.imageView, nullptr, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    VkRenderingAttachmentInfo depthAttachment = vkinit::depth_attachment_info(_depthImage.imageView, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
    VkRenderingInfo renderInfo = vkinit::rendering_info(_windowExtent, &colorAttachment, &depthAttachment);
    vkCmdBeginRendering(cmd, &renderInfo);

    // Allocate new uniform buffer for scene data
    AllocatedBuffer gpuSceneDataBuffer = create_buffer(sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);

    // Add it to end of deletion queue of this frame
    get_current_frame()._deletionQueue.push_function([=, this]() {
        destroy_buffer(gpuSceneDataBuffer);
    });

    // Write buffer
    GPUSceneData* sceneUniformData = (GPUSceneData*)gpuSceneDataBuffer.allocation->GetMappedData();
    *sceneUniformData = sceneData;

    // Create a descriptor set that binds that buffer and updates it
    VkDescriptorSet globalDescriptor = get_current_frame()._frameDescriptors.allocate(_device, _gpuSceneDataDescriptorLayout);

    // Write GPU scene data
    DescriptorWriter writer;
    writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.update_set(_device, globalDescriptor);

    // Define outside draw function: stateful pipeline
    MaterialPipeline* lastPipeline = nullptr;
    MaterialInstance* lastMaterial = nullptr;
    VkBuffer lastIndexBuffer = VK_NULL_HANDLE;

    // Draw Entire Object List
    /*for (const RenderObject& draw : mainDrawContext.OpaqueSurfaces) {}*/
    auto draw = [&](const RenderObject& r) {
        if (r.material != lastMaterial) {
            lastMaterial = r.material;

            // check if pipeline has changed - rebind pipelines and descriptors if material changed
            if (r.material->pipeline != lastPipeline) {

                lastPipeline = r.material->pipeline;
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->pipeline);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 0, 1, &globalDescriptor, 0, nullptr);
                
                // Allocate Viewport and Scissor
                VkViewport viewport = {};
                viewport.x = 0;
                viewport.y = 0;
                viewport.width = _drawExtent.width;
                viewport.height = _drawExtent.height;
                viewport.minDepth = 0.f;
                viewport.maxDepth = 1.f;
                vkCmdSetViewport(cmd, 0, 1, &viewport);

                VkRect2D scissor = {};
                scissor.offset.x = 0;
                scissor.offset.y = 0;
                scissor.extent.width = _drawExtent.width;
                scissor.extent.height = _drawExtent.height;
                vkCmdSetScissor(cmd, 0, 1, &scissor);
            }

            // bind descriptor sets for material parameters and textures if material isntance changed
            vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, r.material->pipeline->layout, 1, 1, &r.material->materialSet, 0, nullptr);
        }

        // rebind index buffer if changed
        if (r.indexBuffer != lastIndexBuffer) {
            lastIndexBuffer = r.indexBuffer;
            vkCmdBindIndexBuffer(cmd, r.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        }
        
        // calculate final mesh matrix
        GPUDrawPushConstants push_constants;
        push_constants.worldMatrix = r.transform;
        push_constants.vertexBuffer = r.vertexBufferAddress;

        vkCmdPushConstants(cmd, r.material->pipeline->layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(GPUDrawPushConstants), &push_constants);
        
        vkCmdDrawIndexed(cmd, r.indexCount, 1, r.firstIndex, 0, 0);

        // Add counters for triangles and draws
        stats.drawcall_count++;
        stats.triangle_count += r.indexCount / 3;
    };

    for (auto& o : opaque_draws) {
        draw(mainDrawContext.OpaqueSurfaces[o]);
    }

    for (auto& t : mainDrawContext.TransparentSurfaces) {
        draw(t);
    }
    
    vkCmdEndRendering(cmd);

    // delete draw commands now that we processed them
    mainDrawContext.OpaqueSurfaces.clear();
    mainDrawContext.TransparentSurfaces.clear();

    auto end = std::chrono::system_clock::now();

    // Convert to microseconds (int) and then come back to milliseconds
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.mesh_draw_time = elapsed.count() / 1000.f;
}

void VulkanEngine::draw_imgui(VkCommandBuffer cmd, VkImageView targetImageView) {
    VkRenderingAttachmentInfo colorAttachment = vkinit::attachment_info(targetImageView, nullptr, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    VkRenderingInfo renderInfo = vkinit::rendering_info(_swapchainExtent, &colorAttachment, nullptr);

    vkCmdBeginRendering(cmd, &renderInfo);

    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);

    vkCmdEndRendering(cmd);
}

// Mesh Functionality -------------------------------------------------------------------------------------

GPUMeshBuffers VulkanEngine::upload_mesh(std::span<uint32_t> indices, std::span<Vertex> vertices) {
    const size_t vertexBufferSize = vertices.size() * sizeof(Vertex);
    const size_t indexBufferSize = indices.size() * sizeof(uint32_t);

    GPUMeshBuffers newSurface;

    // Create Vertex Buffer
    newSurface.vertexBuffer = create_buffer(vertexBufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Find address of Vertex Buffer
    VkBufferDeviceAddressInfo deviceAddressInfo{
        .sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO,
        .buffer = newSurface.vertexBuffer.buffer
    };
    newSurface.vertexBufferAddress = vkGetBufferDeviceAddress(_device, &deviceAddressInfo);

    // Create Index Buffer
    newSurface.indexBuffer = create_buffer(indexBufferSize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, VMA_MEMORY_USAGE_GPU_ONLY);

    // Staging Buffer Copy to GPU from CPU
    AllocatedBuffer staging = create_buffer(vertexBufferSize + indexBufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);

    void* data = staging.allocation->GetMappedData();
    memcpy(data, vertices.data(), vertexBufferSize);
    memcpy((char*)data + vertexBufferSize, indices.data(), indexBufferSize);

    // Immediate Submit outside of draw loop
    immediate_submit([&](VkCommandBuffer cmd) {
        VkBufferCopy vertexCopy{ 0 };
        vertexCopy.dstOffset = 0;
        vertexCopy.srcOffset = 0;
        vertexCopy.size = vertexBufferSize;

        vkCmdCopyBuffer(cmd, staging.buffer, newSurface.vertexBuffer.buffer, 1, &vertexCopy);

        VkBufferCopy indexCopy{ 0 };
        indexCopy.dstOffset = 0;
        indexCopy.srcOffset = vertexBufferSize;
        indexCopy.size = indexBufferSize;

        vkCmdCopyBuffer(cmd, staging.buffer, newSurface.indexBuffer.buffer, 1, &indexCopy);
        });

    destroy_buffer(staging);

    return newSurface;
}

// Memory & Image Functionality ----------------------------------------------------------------------------

AllocatedBuffer VulkanEngine::create_buffer(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage) {
    VkBufferCreateInfo bufferInfo = { .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.pNext = nullptr;
    bufferInfo.size = allocSize;
    bufferInfo.usage = usage;

    VmaAllocationCreateInfo vmaAllocInfo = {};
    vmaAllocInfo.usage = memoryUsage;
    vmaAllocInfo.flags = VMA_ALLOCATION_CREATE_MAPPED_BIT;

    AllocatedBuffer newBuffer;

    VK_CHECK(vmaCreateBuffer(_allocator, &bufferInfo, &vmaAllocInfo, &newBuffer.buffer, &newBuffer.allocation, &newBuffer.info));

    return newBuffer;
}

AllocatedBuffer VulkanEngine::create_buffer_detailed(size_t allocSize, VkBufferUsageFlags usage, VmaMemoryUsage memoryUsage, VkFlags flags) {
    VkBufferCreateInfo bufferInfo = { .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.pNext = nullptr;
    bufferInfo.size = allocSize;
    bufferInfo.usage = usage;

    VmaAllocationCreateInfo vmaAllocInfo = {};
    vmaAllocInfo.usage = memoryUsage; 
    vmaAllocInfo.requiredFlags = VkMemoryPropertyFlags(flags);

    AllocatedBuffer newBuffer;
    
    VK_CHECK(vmaCreateBuffer(_allocator, &bufferInfo, &vmaAllocInfo, &newBuffer.buffer, &newBuffer.allocation, &newBuffer.info));

    return newBuffer;
}

void VulkanEngine::destroy_buffer(const AllocatedBuffer& buffer) {
    vmaDestroyBuffer(_allocator, buffer.buffer, buffer.allocation);
}

AllocatedImage VulkanEngine::create_image(VkExtent3D size, VkFormat format, VkImageUsageFlags usage, bool mipmapped)
{
    AllocatedImage newImage;
    newImage.imageFormat = format;
    newImage.imageExtent = size;

    VkImageCreateInfo img_info = vkinit::image_create_info(format, usage, size);
    if (mipmapped) {
        img_info.mipLevels = static_cast<uint32_t>(std::floor(std::log2(std::max(size.width, size.height)))) + 1;
    }

    // always allocate images on dedicated GPU memory
    VmaAllocationCreateInfo allocinfo = {};
    allocinfo.usage = VMA_MEMORY_USAGE_GPU_ONLY;
    allocinfo.requiredFlags = VkMemoryPropertyFlags(VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    // allocate and create the image
    VK_CHECK(vmaCreateImage(_allocator, &img_info, &allocinfo, &newImage.image, &newImage.allocation, nullptr));

    // if the format is a depth format, we will need to have it use the correct
    // aspect flag
    VkImageAspectFlags aspectFlag = VK_IMAGE_ASPECT_COLOR_BIT;
    if (format == VK_FORMAT_D32_SFLOAT) {
        aspectFlag = VK_IMAGE_ASPECT_DEPTH_BIT;
    }

    // build a image-view for the image
    VkImageViewCreateInfo view_info = vkinit::imageview_create_info(format, newImage.image, aspectFlag);
    view_info.subresourceRange.levelCount = img_info.mipLevels;

    VK_CHECK(vkCreateImageView(_device, &view_info, nullptr, &newImage.imageView));

    return newImage;
}

AllocatedImage VulkanEngine::create_image(void* data, VkExtent3D size, VkFormat format, VkImageUsageFlags usage, bool mipmapped)
{
    size_t data_size = size.depth * size.width * size.height * 4;
    AllocatedBuffer uploadbuffer = create_buffer(data_size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);

    memcpy(uploadbuffer.info.pMappedData, data, data_size);

    AllocatedImage new_image = create_image(size, format, usage | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT, mipmapped);

    immediate_submit([&](VkCommandBuffer cmd) {
        vkutil::transition_image(cmd, new_image.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

        VkBufferImageCopy copyRegion = {};
        copyRegion.bufferOffset = 0;
        copyRegion.bufferRowLength = 0;
        copyRegion.bufferImageHeight = 0;

        copyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copyRegion.imageSubresource.mipLevel = 0;
        copyRegion.imageSubresource.baseArrayLayer = 0;
        copyRegion.imageSubresource.layerCount = 1;
        copyRegion.imageExtent = size;

        // Copy the buffer into the image
        vkCmdCopyBufferToImage(cmd, uploadbuffer.buffer, new_image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
            &copyRegion);

        if (mipmapped) {
            vkutil::generate_mipmaps(cmd, new_image.image, VkExtent2D{new_image.imageExtent.width, new_image.imageExtent.height});
            fmt::println("Generated Mip Maps!\n");
        }
        else {
            vkutil::transition_image(cmd, new_image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
    });

    destroy_buffer(uploadbuffer);
    return new_image;
}

AllocatedImage VulkanEngine::createTextureImage(const char* filePath) {
    int texWidth;
    int texHeight;
    int texChannels;

    stbi_uc* pixels = stbi_load(filePath, &texWidth, &texHeight, &texChannels, STBI_rgb_alpha);

    VkDeviceSize imageSize = texWidth * texHeight * 4;

    if (!pixels) {
        throw std::runtime_error("failed to load texture image!");
    }
    else {
        fmt::println("GOT THE TEXTURES");
    }

    AllocatedBuffer stagingBuffer = create_buffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);

    void* data = stagingBuffer.allocation->GetMappedData();
    memcpy(data, pixels, static_cast<size_t>(imageSize));
    stbi_image_free(pixels);

    fmt::println("Got the staging buffer!");
    
    VkExtent3D size;
    size.width = texWidth;
    size.height = texHeight;
    size.depth = 1;

    AllocatedImage particleTexture = create_image(
        data,
        size,
        VK_FORMAT_R8G8B8A8_UNORM,
        VK_IMAGE_USAGE_SAMPLED_BIT);

    fmt::println("MADE THE IMAGE");

    destroy_buffer(stagingBuffer);

    return particleTexture;
}

void VulkanEngine::destroy_image(const AllocatedImage& img) {
    vkDestroyImageView(_device, img.imageView, nullptr);
    vmaDestroyImage(_allocator, img.image, img.allocation);
}

// Object Creation Functionality --------------------------------------------------------------------------



// Miscellaneous Functionality ----------------------------------------------------------------------------

void VulkanEngine::immediate_submit(std::function<void(VkCommandBuffer cmd)>&& function) {
    VK_CHECK(vkResetFences(_device,1, &_immFence));
    VK_CHECK(vkResetCommandBuffer(_immCommandBuffer, 0));

    VkCommandBuffer cmd = _immCommandBuffer;
    VkCommandBufferBeginInfo cmdBeginInfo = vkinit::command_buffer_begin_info(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

    VK_CHECK(vkBeginCommandBuffer(cmd, &cmdBeginInfo));
    function(cmd);
    VK_CHECK(vkEndCommandBuffer(cmd));

    VkCommandBufferSubmitInfo submitInfo = vkinit::command_buffer_submit_info(cmd);
    VkSubmitInfo2 submit = vkinit::submit_info(&submitInfo, nullptr, nullptr);

    VK_CHECK(vkQueueSubmit2(_graphicsQueue, 1, &submit, _immFence));
    VK_CHECK(vkWaitForFences(_device, 1, &_immFence, true, 9999999999));
}

// Main Functionality -------------------------------------------------------------------------------------

void VulkanEngine::run()
{
    SDL_Event e;
    bool bQuit = false;
    bool pause = false;

    // main loop
    while (!bQuit) {
        // begin clock
        auto start = std::chrono::system_clock::now();
        
        // Handle events on queue
        while (SDL_PollEvent(&e) != 0) {
            // close the window when user alt-f4s or clicks the X button
            if (e.type == SDL_QUIT)
                bQuit = true;

            if (e.type == SDL_WINDOWEVENT) {
                if (e.window.event == SDL_WINDOWEVENT_MINIMIZED) {
                    stop_rendering = true;
                }
                if (e.window.event == SDL_WINDOWEVENT_RESTORED) {
                    stop_rendering = false;
                }
            }

            // Keyboard events
            /*if (e.type == SDL_KEYDOWN) {
                pause = !pause;
            }*/

            if (updateCamera) {
                mainCamera.processSDLEvent(e);
            }
            
            ImGui_ImplSDL2_ProcessEvent(&e);
        }

        if (resize_requested) {
            resize_swapchain();
        }

        // do not draw if we are minimized
        if (stop_rendering) {
            // throttle the speed to avoid the endless spinning
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // ImGui New Frame
        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // ImGUI UI
        if (ImGui::Begin("background")) {
            ImGui::SliderFloat("Render Scale", &renderScale, 0.3f, 1.f);

            ComputeEffect& selected = backgroundEffects[currentBackgroundEffect];

            ImGui::Text("Selected Effect: ", selected.name);
            ImGui::SliderInt("Effect Index", &currentBackgroundEffect, 0, backgroundEffects.size() - 1);
            ImGui::InputFloat4("data1", (float*)&selected.data.data1);
            ImGui::InputFloat4("data2", (float*)&selected.data.data2);
            ImGui::InputFloat4("data3", (float*)&selected.data.data3);
            ImGui::InputFloat4("data4", (float*)&selected.data.data4);
        }
        
        ImGui::End();

        /*ImGui::Begin("Transform matrix Slider & camera updater");
        ImGui::SliderFloat("x", &r_x, -10.f, 10.f);
        ImGui::SliderFloat("y", &r_y, -10.f, 10.f);
        ImGui::SliderFloat("z", &r_z, -10.f, 10.f);
        ImGui::Checkbox("camera update", &updateCamera);
        ImGui::End();*/

        ImGui::Begin("Stats");
        ImGui::Text("frametime %f ms", stats.frametime);
        ImGui::Text("draw time %f ms", stats.mesh_draw_time);
        ImGui::Text("update time %f ms", stats.scene_update_time);
        ImGui::Text("triangles %i", stats.triangle_count);
        ImGui::Text("draws %i", stats.drawcall_count);
        ImGui::End();

        // ImGUI Calculates Internal Draw Structures
        ImGui::Render();

        draw();

        // get clock again
        auto end = std::chrono::system_clock::now();

        // convert to microseconds (int) and then come back to milliseconds
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        float durationOfFrame = elapsed.count() / 1000.f;
        stats.frametime = durationOfFrame;

        // Physics Update
        if (!pause) RigidBodyPhysicsEngine->update(durationOfFrame);

    }
}

void VulkanEngine::update_scene()
{
    // Start Clock
    auto start = std::chrono::system_clock::now();

    // Camera Update
    if (updateCamera) {
        mainCamera.update();
    }
    
    mainDrawContext.OpaqueSurfaces.clear();

    // Camera view and projection
    glm::mat4 view = mainCamera.getViewMatrix();
    glm::mat4 projection = glm::perspective(glm::radians(70.f), (float)_windowExtent.width / (float)_windowExtent.height, 10000.f, 0.1f);
    
    // Invert projection to translate from OpenGL coordinates to Vulkan coordinates
    projection[1][1] *= -1;

    // Copy view, projection into sceneData
    sceneData.view = view;
    sceneData.proj = projection;
    sceneData.viewproj = projection * view;

    // Default lighting parameters - Will replace with either Ray tracing or PBR
    sceneData.ambientColour = glm::vec4(.1f);
    sceneData.sunlightColour = glm::vec4(1.f);
    sceneData.sunlightDirection = glm::vec4(0, 1, 0.5, 1.f);

    // Draw Baseplate
    glm::mat4 scale = glm::scale(glm::vec3{ 1 });
    glm::mat4 translation = glm::translate(glm::vec3{0, 0, 0});
    loadedScenes["initial"]->nodes["Ground"]->Draw(translation * scale, mainDrawContext);

    // Update scene based on RigidBodyPhysics simulation - Draw all boxes currently kept in application
    for (Box* box = RigidBodyPhysicsEngine->boxData; box < RigidBodyPhysicsEngine->boxData+RigidBodyPhysicsEngine->boxes; box++) {
        //glm::mat4 transform = box->getTransform();
        Matrix4 transform = box->getTransform();

        glm::mat4 bbb = glm::mat4();
        bbb[0][0] = transform.data[0];
        bbb[1][0] = transform.data[1];
        bbb[2][0] = transform.data[2];
        bbb[3][0] = transform.data[3];

        bbb[0][1] = transform.data[4];
        bbb[1][1] = transform.data[5];
        bbb[2][1] = transform.data[6];
        bbb[3][1] = transform.data[7];

        bbb[0][2] = transform.data[8];
        bbb[1][2] = transform.data[9];
        bbb[2][2] = transform.data[10];
        bbb[3][2] = transform.data[11];

        bbb[0][3] = 0;
        bbb[1][3] = 0;
        bbb[2][3] = 0;
        bbb[3][3] = 1;

        //glm::transpose(bbb)
        loadedScenes["initial"]->nodes["Cube"]->Draw(bbb, mainDrawContext);
        //loadedScenes["initial"]->nodes["Cube"]->Draw(translation*scale, mainDrawContext);
        
        printf("%s\n", glm::to_string(glm::transpose(bbb)).c_str());

        
    }
    
    /*// Draw Baseplate
    //loadedScenes["initial"]->nodes["Ground"]->Draw(glm::mat4{1.f}, mainDrawContext);
    //loadedScenes["initial"]->Draw(glm::mat4{ 1.f }, mainDrawContext);
    // Test Mesh cubes draw
    //for (int x = -3; x < 3; x++) {
    //    glm::mat4 scale = glm::scale(glm::vec3{0.2});
    //    glm::mat4 translation = glm::translate(glm::vec3{x, 1, 0});
    //    //loadedScenes["initial"]->nodes["Ground"]->Draw(translation * scale, mainDrawContext);
    //    
    //}
    glm::mat4 transform = glm::translate(glm::vec3{ 1, 1, 0 });
    transform = glm::rotate(transform, static_cast<float>(45 * (PI / 180)), glm::vec3(1, 0, 0));
    // testRigidBody1.transformMatrix = glm::translate(glm::vec3{ r_x, r_y, r_z });
    glm::mat4 transform1 = glm::translate(glm::vec3{ r_x, r_y, r_z });
    glm::mat4 transform2 = glm::translate(glm::vec3{ 1, 1, 0 });
    loadedScenes["initial"]->nodes["Cube"]->Draw(transform1, mainDrawContext);
    loadedScenes["initial"]->nodes["Cube"]->Draw(transform2, mainDrawContext);*/

    // End Clock
    auto end = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    stats.scene_update_time = elapsed.count() / 1000.f;
    sceneData.deltaTime = elapsed.count() / 1000.f;
}

void VulkanEngine::cleanup()
{
    if (_isInitialized) {
        vkDeviceWaitIdle(_device);

        loadedScenes.clear();


        for (int i = 0; i < FRAME_OVERLAP; i++) {
            vkDestroyCommandPool(_device, _frames[i]._commandPool, nullptr);

            vkDestroyFence(_device, _frames[i]._renderFence, nullptr);
            vkDestroySemaphore(_device, _frames[i]._renderSemaphore, nullptr);
            vkDestroySemaphore(_device, _frames[i]._swapchainSemaphore, nullptr);

            _frames[i]._deletionQueue.flush();
        }

        for (auto& mesh : testMeshes) {
            destroy_buffer(mesh->meshBuffers.indexBuffer);
            destroy_buffer(mesh->meshBuffers.vertexBuffer);
        }

        _mainDeletionQueue.flush();

        destroy_swapchain();

        vkDestroySurfaceKHR(_instance, _surface, nullptr);
        vkDestroyDevice(_device, nullptr);

        vkb::destroy_debug_utils_messenger(_instance, _debug_messenger);
        vkDestroyInstance(_instance, nullptr);

        SDL_DestroyWindow(_window);
    }

    // clear engine pointer
    loadedEngine = nullptr;
}


// ========================================================================================================
//
//                                               GLTF ARCHITECTURE
// 
// ========================================================================================================

void GLTFMetallic_Roughness::build_pipelines(VulkanEngine* engine) {
    VkShaderModule meshVertShader;
    if (!vkutil::load_shader_module("../shaders/mesh_vert.spv", engine->_device, &meshVertShader)) {
        fmt::println("Error when building the triangle vertex shader module!");
    }

    VkShaderModule meshFragShader;
    if (!vkutil::load_shader_module("../shaders/mesh_frag.spv", engine->_device, &meshFragShader)) {
        fmt::println("Error when building the triangle fragment shader module!");
    }

    VkPushConstantRange matrixRange{};
    matrixRange.offset = 0;
    matrixRange.size = sizeof(GPUDrawPushConstants);
    matrixRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    DescriptorLayoutBuilder layoutBuilder;
    layoutBuilder.add_binding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    layoutBuilder.add_binding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    layoutBuilder.add_binding(2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    materialLayout = layoutBuilder.build(engine->_device, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT);

    VkDescriptorSetLayout layouts[] = { engine->_gpuSceneDataDescriptorLayout, materialLayout };

    VkPipelineLayoutCreateInfo mesh_layout_info = vkinit::pipeline_layout_create_info();
    mesh_layout_info.setLayoutCount = 2;
    mesh_layout_info.pSetLayouts = layouts;
    mesh_layout_info.pPushConstantRanges = &matrixRange;
    mesh_layout_info.pushConstantRangeCount = 1;

    VkPipelineLayout newLayout;
    VK_CHECK(vkCreatePipelineLayout(engine->_device, &mesh_layout_info, nullptr, &newLayout));

    opaquePipeline.layout = newLayout;
    transparentPipeline.layout = newLayout;

    // build stage create info for both vertex and fragment stages
    // Lets pipeline know shader modules per stage
    PipelineBuilder pipelineBuilder;
    pipelineBuilder.set_shaders(meshVertShader, meshFragShader);
    pipelineBuilder.set_input_topology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
    pipelineBuilder.set_polygon_mode(VK_POLYGON_MODE_FILL);
    pipelineBuilder.set_cull_mode(VK_CULL_MODE_NONE, VK_FRONT_FACE_CLOCKWISE);
    pipelineBuilder.set_multisampling_none();
    pipelineBuilder.disable_blending();
    pipelineBuilder.enable_depthtest(true, VK_COMPARE_OP_GREATER_OR_EQUAL);

    // render format
    pipelineBuilder.set_color_attachment_format(engine->_drawImage.imageFormat);
    pipelineBuilder.set_depth_format(engine->_depthImage.imageFormat);

    // use triangle layout we created
    pipelineBuilder._pipelineLayout = newLayout;

    // finally build opaque pipeline
    opaquePipeline.pipeline = pipelineBuilder.build_pipeline(engine->_device);

    // create transparent variant
    pipelineBuilder.enable_blending_additive();
    pipelineBuilder.enable_depthtest(false, VK_COMPARE_OP_GREATER_OR_EQUAL);

    // finally build transparent pipeline
    transparentPipeline.pipeline = pipelineBuilder.build_pipeline(engine->_device);

    // destroy shader modules
    vkDestroyShaderModule(engine->_device, meshFragShader, nullptr);
    vkDestroyShaderModule(engine->_device, meshVertShader, nullptr);

    // add objects to main deletion queue
    engine->_mainDeletionQueue.push_function([=]() {
        vkDestroyDescriptorSetLayout(engine->_device, materialLayout, nullptr);
        vkDestroyPipelineLayout(engine->_device, newLayout, nullptr);
        vkDestroyPipeline(engine->_device, opaquePipeline.pipeline, nullptr);
        vkDestroyPipeline(engine->_device, transparentPipeline.pipeline, nullptr);
    });
}

MaterialInstance GLTFMetallic_Roughness::write_material(VkDevice device, MaterialPass pass, const MaterialResources& resources, DescriptorAllocatorGrowable& descriptorAllocator) {
    MaterialInstance matData;
    matData.passType = pass;
    if (pass == MaterialPass::Transparent) {
        matData.pipeline = &transparentPipeline;
    }
    else {
        matData.pipeline = &opaquePipeline;
    }
    matData.materialSet = descriptorAllocator.allocate(device, materialLayout);

    writer.clear();
    writer.write_buffer(0, resources.dataBuffer, sizeof(MaterialConstants), resources.dataBufferOffset, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    writer.write_image(1, resources.colourImage.imageView, resources.colourSampler, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    writer.write_image(2, resources.metalRoughImage.imageView, resources.metalRoughSampler, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);

    writer.update_set(device, matData.materialSet);

    return matData;
}

void MeshNode::Draw(const glm::mat4& topMatrix, DrawContext& ctx) {
    glm::mat4 nodeMatrix = topMatrix * worldTransform;

    for (auto& s : mesh->surfaces) {
        RenderObject def;
        def.indexCount = s.count;
        def.firstIndex = s.startIndex;
        def.indexBuffer = mesh->meshBuffers.indexBuffer.buffer;
        def.material = &s.material->data;
        def.bounds = s.bounds;
        def.transform = nodeMatrix;
        def.vertexBufferAddress = mesh->meshBuffers.vertexBufferAddress;

        if (s.material->data.passType == MaterialPass::Transparent) {
            ctx.TransparentSurfaces.push_back(def);
        }
        else {
            ctx.OpaqueSurfaces.push_back(def);
        }
    }

    // Recurse down Draw Tree
    Node::Draw(topMatrix, ctx);
}

void GLTFMetallic_Roughness::clear_resources(VkDevice device) {
    // Add cleanup code here instead of in build_pipelines
}



// Particle Functionality ---------------------------------------------------------------------------------
//void VulkanEngine::init_particle_system() {
//    fmt::println("particle system init started");
//    init_particle_buffers();
//    create_particle_pools();
//    create_particle_descriptor_layout();
//    create_particle_descriptor_sets();
//    create_particle_compute_pipeline();
//    create_particle_graphics_pipeline();
//    fmt::println("particle system init ended");
//}
//
//void VulkanEngine::init_particle_buffers() {
//    // Initialize particles
//    std::default_random_engine rndEngine((unsigned)time(nullptr));
//    std::uniform_real_distribution<float> rndDist(0.0f, 1.0f);
//    std::uniform_real_distribution<float> rndDist2(0.1f, 1.0f);
//    // Initial particle positions on a circle
//    std::vector<Particle> particles(PARTICLE_COUNT);
//    for (auto& particle : particles) {
//        float randPosition_x = rndDist(rndEngine) * 3.0f;
//        float randPosition_y = rndDist(rndEngine) * 3.0f;
//        float randPosition_z = rndDist(rndEngine) * 3.0f;
//        particle.position = glm::vec3(randPosition_x, randPosition_y, randPosition_z);
//
//        float randVel_1 = rndDist(rndEngine);
//        float randVel_2 = rndDist(rndEngine);
//        float randVel_3 = rndDist(rndEngine);
//        particle.velocity = glm::normalize(glm::vec3(randVel_1, randVel_2, randVel_3));
//        //particle.velocity = glm::vec3(randVel_1, randVel_2, randVel_3);
//
//        particle.color = glm::vec4(rndDist(rndEngine), rndDist(rndEngine), rndDist(rndEngine), 1.0f);
//    }
//
//    VkDeviceSize bufferSize = sizeof(Particle) * PARTICLE_COUNT;
//
//    // Staging Buffer Copy to GPU from CPU
//    AllocatedBuffer staging = create_buffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_CPU_ONLY);
//
//    void* data = staging.allocation->GetMappedData();
//    memcpy(data, particles.data(), bufferSize);
//
//    // Resize Shader buffers to number of frames rendering
//    particleShaderStorageBuffers.resize(FRAME_OVERLAP);
//
//    // Create and Copy buffer - Immediate Submit outside of draw loop
//    for (size_t i = 0; i < FRAME_OVERLAP; i++) {
//        particleShaderStorageBuffers[i] = create_buffer(bufferSize,
//            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
//            VMA_MEMORY_USAGE_GPU_ONLY);
//
//        // Copy to GPU buffers
//        immediate_submit([&](VkCommandBuffer cmd) {
//            VkBufferCopy particleVertexCopy{ 0 };
//            particleVertexCopy.dstOffset = 0;
//            particleVertexCopy.srcOffset = 0;
//            particleVertexCopy.size = bufferSize;
//
//            vkCmdCopyBuffer(cmd, staging.buffer, particleShaderStorageBuffers[i].buffer, 1, &particleVertexCopy);
//        });
//    }
//
//    // Create Uniform Buffers
//    particleUniformBuffer = create_buffer(sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
//    
//    destroy_buffer(staging);
//}
//
//void VulkanEngine::create_particle_pools() {
//    std::vector<DescriptorAllocatorGrowable::PoolSizeRatio> poolSizes = {
//        {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2},
//        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4},
//    };
//
//    particleDescriptorAllocator.init(_device, 10, poolSizes);
//}
//
//void VulkanEngine::create_particle_descriptor_layout() {
//    std::array<VkDescriptorSetLayoutBinding, 3> layoutBindings{};
//    layoutBindings[0].binding = 0;
//    layoutBindings[0].descriptorCount = 1;
//    layoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
//    layoutBindings[0].pImmutableSamplers = nullptr;
//    layoutBindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
//
//    layoutBindings[1].binding = 1;
//    layoutBindings[1].descriptorCount = 1;
//    layoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
//    layoutBindings[1].pImmutableSamplers = nullptr;
//    layoutBindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
//
//    layoutBindings[2].binding = 2;
//    layoutBindings[2].descriptorCount = 1;
//    layoutBindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
//    layoutBindings[2].pImmutableSamplers = nullptr;
//    layoutBindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
//
//    VkDescriptorSetLayoutCreateInfo layoutInfo{};
//    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
//    layoutInfo.bindingCount = 3;
//    layoutInfo.pBindings = layoutBindings.data();
//
//    if (vkCreateDescriptorSetLayout(_device, &layoutInfo, nullptr, &particleComputeDescriptorSetLayout) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create compute descriptor set layout!");
//    }
//
//    std::array<VkDescriptorSetLayoutBinding, 1> layoutBindings_vert{};
//    layoutBindings_vert[0].binding = 0;
//    layoutBindings_vert[0].descriptorCount = 1;
//    layoutBindings_vert[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
//    layoutBindings_vert[0].pImmutableSamplers = nullptr;
//    layoutBindings_vert[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
//
//    VkDescriptorSetLayoutCreateInfo layoutInfo2{};
//    layoutInfo2.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
//    layoutInfo2.bindingCount = 1;
//    layoutInfo2.pBindings = layoutBindings_vert.data();
//
//    if (vkCreateDescriptorSetLayout(_device, &layoutInfo2, nullptr, &particleVertexComputeDescriptorSetLayout) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create compute descriptor set layout!");
//    }
//}
//
//void VulkanEngine::create_particle_descriptor_sets() {
//    std::vector<VkDescriptorSetLayout> layouts(FRAME_OVERLAP, particleComputeDescriptorSetLayout);
//    particleComputeDescriptorSets.resize(FRAME_OVERLAP);
//
//    for (size_t i = 0; i < FRAME_OVERLAP; i++) {
//        particleComputeDescriptorSets[i] = particleDescriptorAllocator.allocate(_device, layouts[i]);
//    }
//
//    for (size_t i = 0; i < FRAME_OVERLAP; i++) {
//        VkDescriptorBufferInfo uniformBufferInfo{};
//        uniformBufferInfo.buffer = particleUniformBuffer.buffer;
//        uniformBufferInfo.offset = 0;
//        uniformBufferInfo.range = sizeof(GPUSceneData);
//
//        std::array<VkWriteDescriptorSet, 3> descriptorWrites{};
//        descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
//        descriptorWrites[0].dstSet = particleComputeDescriptorSets[i];
//        descriptorWrites[0].dstBinding = 0;
//        descriptorWrites[0].dstArrayElement = 0;
//        descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
//        descriptorWrites[0].descriptorCount = 1;
//        descriptorWrites[0].pBufferInfo = &uniformBufferInfo;
//
//        VkDescriptorBufferInfo storageBufferInfoLastFrame{};
//        storageBufferInfoLastFrame.buffer = particleShaderStorageBuffers[(i - 1) % FRAME_OVERLAP].buffer;
//        storageBufferInfoLastFrame.offset = 0;
//        storageBufferInfoLastFrame.range = sizeof(Particle) * PARTICLE_COUNT;
//
//        descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
//        descriptorWrites[1].dstSet = particleComputeDescriptorSets[i];
//        descriptorWrites[1].dstBinding = 1;
//        descriptorWrites[1].dstArrayElement = 0;
//        descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
//        descriptorWrites[1].descriptorCount = 1;
//        descriptorWrites[1].pBufferInfo = &storageBufferInfoLastFrame;
//
//        VkDescriptorBufferInfo storageBufferInfoCurrentFrame{};
//        storageBufferInfoCurrentFrame.buffer = particleShaderStorageBuffers[i].buffer;
//        storageBufferInfoCurrentFrame.offset = 0;
//        storageBufferInfoCurrentFrame.range = sizeof(Particle) * PARTICLE_COUNT;
//
//        descriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
//        descriptorWrites[2].dstSet = particleComputeDescriptorSets[i];
//        descriptorWrites[2].dstBinding = 2;
//        descriptorWrites[2].dstArrayElement = 0;
//        descriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
//        descriptorWrites[2].descriptorCount = 1;
//        descriptorWrites[2].pBufferInfo = &storageBufferInfoCurrentFrame;
//
//        vkUpdateDescriptorSets(_device, 3, descriptorWrites.data(), 0, nullptr);
//    }
//}
//
//void VulkanEngine::create_particle_compute_pipeline() {
//    VkShaderModule computeShaderModule;
//    if (!vkutil::load_shader_module("../shaders/particle/p_comp.spv", _device, &computeShaderModule)) {
//        fmt::println("Error when building the particle compute shader module!");
//    }
//
//    VkPipelineShaderStageCreateInfo computeShaderStageInfo{};
//    computeShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
//    computeShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
//    computeShaderStageInfo.module = computeShaderModule;
//    computeShaderStageInfo.pName = "main";
//
//    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
//    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
//    pipelineLayoutInfo.setLayoutCount = 1;
//    pipelineLayoutInfo.pSetLayouts = &particleComputeDescriptorSetLayout;
//
//    if (vkCreatePipelineLayout(_device, &pipelineLayoutInfo, nullptr, &particleComputePipelineLayout) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create compute pipeline layout!");
//    }
//
//    
//    VkComputePipelineCreateInfo pipelineInfo{};
//    pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
//    pipelineInfo.layout = particleComputePipelineLayout;
//    pipelineInfo.stage = computeShaderStageInfo;
//
//    if (vkCreateComputePipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &particleComputePipeline) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create compute pipeline!");
//    }
//
//    vkDestroyShaderModule(_device, computeShaderModule, nullptr);
//}
//
//void VulkanEngine::create_particle_graphics_pipeline() {
//    VkShaderModule vertShaderModule;
//    if (!vkutil::load_shader_module("../shaders/particle/p_vert.spv", _device, &vertShaderModule)) {
//        fmt::println("Error when building the particle compute shader module!");
//    }
//    VkShaderModule fragShaderModule;
//    if (!vkutil::load_shader_module("../shaders/particle/p_frag.spv", _device, &fragShaderModule)) {
//        fmt::println("Error when building the particle compute shader module!");
//    }
//
//    VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
//    vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
//    vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
//    vertShaderStageInfo.module = vertShaderModule;
//    vertShaderStageInfo.pName = "main";
//
//    VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
//    fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
//    fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
//    fragShaderStageInfo.module = fragShaderModule;
//    fragShaderStageInfo.pName = "main";
//
//    VkPipelineShaderStageCreateInfo shaderStages[] = {
//        vertShaderStageInfo,
//        fragShaderStageInfo
//    };
//
//    VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
//    vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
//
//    VkVertexInputBindingDescription bindingDescription{};
//    bindingDescription.binding = 0;
//    bindingDescription.stride = sizeof(Particle);
//    bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
//
//    std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions{};
//    attributeDescriptions[0].binding = 0;
//    attributeDescriptions[0].location = 0;
//    attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
//    attributeDescriptions[0].offset = offsetof(Particle, position);
//
//    attributeDescriptions[1].binding = 0;
//    attributeDescriptions[1].location = 1;
//    attributeDescriptions[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
//    attributeDescriptions[1].offset = offsetof(Particle, color);
//
//    vertexInputInfo.vertexBindingDescriptionCount = 1;
//    vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
//    vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
//    vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();
//
//    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
//    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
//    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
//
//    VkPipelineRasterizationStateCreateInfo rasterization = {};
//    rasterization.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
//    rasterization.polygonMode = VK_POLYGON_MODE_FILL;
//    rasterization.cullMode = VK_CULL_MODE_NONE;
//    rasterization.frontFace = VK_FRONT_FACE_CLOCKWISE;
//    rasterization.lineWidth = 1.0f;
//
//    VkPipelineMultisampleStateCreateInfo multisampling{};
//    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
//    multisampling.sampleShadingEnable = VK_FALSE;
//    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
//    multisampling.pSampleMask = nullptr;
//
//    VkPipelineColorBlendAttachmentState blendAttachmentState[1] = {};
//    blendAttachmentState[0].colorWriteMask = 0xf;
//    blendAttachmentState[0].blendEnable = VK_FALSE;
//    
//    VkPipelineColorBlendStateCreateInfo colorBlending{};
//    colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
//    colorBlending.attachmentCount = 1;
//    colorBlending.pAttachments = blendAttachmentState;
//
//    VkPipelineDepthStencilStateCreateInfo depthStencilState = {};
//    depthStencilState.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
//    depthStencilState.depthTestEnable = VK_TRUE;
//    depthStencilState.depthWriteEnable = VK_TRUE;
//    depthStencilState.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
//    depthStencilState.depthBoundsTestEnable = VK_FALSE;
//    depthStencilState.back.failOp = VK_STENCIL_OP_KEEP;
//    depthStencilState.back.passOp = VK_STENCIL_OP_KEEP;
//    depthStencilState.back.compareOp = VK_COMPARE_OP_ALWAYS;
//    depthStencilState.stencilTestEnable = VK_FALSE;
//    depthStencilState.front = depthStencilState.back;
//
//    VkPipelineRenderingCreateInfo _renderInfo{};
//    _renderInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
//    _renderInfo.depthAttachmentFormat = _depthImage.imageFormat;
//    VkFormat colorAttachmentFormat = _drawImage.imageFormat;
//    _renderInfo.colorAttachmentCount = 1;
//    _renderInfo.pColorAttachmentFormats = &colorAttachmentFormat;
//
//    std::vector<VkDynamicState> dynamicStates = {
//        VK_DYNAMIC_STATE_VIEWPORT,
//        VK_DYNAMIC_STATE_SCISSOR
//    };
//
//    VkPipelineDynamicStateCreateInfo dynamicState{};
//    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
//    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
//    dynamicState.pDynamicStates = dynamicStates.data();
//
//    VkPipelineViewportStateCreateInfo viewportState{};
//    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
//    viewportState.viewportCount = 1;
//    viewportState.scissorCount = 1;
//
//    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
//    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
//    pipelineLayoutInfo.setLayoutCount = 1;
//    pipelineLayoutInfo.pSetLayouts = &particleVertexComputeDescriptorSetLayout;
//
//    if (vkCreatePipelineLayout(_device, &pipelineLayoutInfo, nullptr, &particleGraphicsPipelineLayout) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create pipeline layout!");
//    }
//
//    VkGraphicsPipelineCreateInfo pipelineInfo{ .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
//    pipelineInfo.pNext = &_renderInfo;
//    pipelineInfo.stageCount = 2;
//    pipelineInfo.pStages = shaderStages;
//    pipelineInfo.pVertexInputState = &vertexInputInfo;
//    pipelineInfo.pInputAssemblyState = &inputAssembly;
//    pipelineInfo.pRasterizationState = &rasterization;
//    pipelineInfo.pMultisampleState = &multisampling;
//    pipelineInfo.pViewportState = &viewportState;
//    pipelineInfo.pColorBlendState = &colorBlending;
//    pipelineInfo.pDepthStencilState = &depthStencilState;
//    pipelineInfo.pDynamicState = &dynamicState;
//    pipelineInfo.layout = particleGraphicsPipelineLayout;
//
//    if (vkCreateGraphicsPipelines(_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &particleGraphicsPipeline) != VK_SUCCESS) {
//        throw std::runtime_error("failed to create graphics pipeline!");
//    }
//
//    vkDestroyShaderModule(_device, fragShaderModule, nullptr);
//    vkDestroyShaderModule(_device, vertShaderModule, nullptr);
//}
//
//void VulkanEngine::draw_compute_particles(VkCommandBuffer cmd) {
//    // Get current frame
//    size_t currentFrame = _frameNumber % FRAME_OVERLAP;
//
//    size_t sceneSize = sizeof(sceneData);
//    memcpy(particleUniformBuffer.allocation->GetMappedData(), &sceneData, sceneSize);
//
//    // Binding
//    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, particleComputePipeline);
//    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, particleComputePipelineLayout, 0, 1, &particleComputeDescriptorSets[currentFrame], 0, nullptr);
//    vkCmdDispatch(cmd, PARTICLE_COUNT, 1, 1);
//}
//
//void VulkanEngine::draw_graphics_particles(VkCommandBuffer cmd) {
//    size_t currentFrame = _frameNumber % FRAME_OVERLAP;
//
//    VkRenderingAttachmentInfo colorAttachment = vkinit::attachment_info(_drawImage.imageView, nullptr, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
//    VkRenderingAttachmentInfo depthAttachment = vkinit::depth_attachment_info(_depthImage.imageView, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL);
//    VkRenderingInfo renderInfo = vkinit::rendering_info(_windowExtent, &colorAttachment, &depthAttachment);
//    vkCmdBeginRendering(cmd, &renderInfo);
//
//    // Allocate new uniform buffer for scene data
//    AllocatedBuffer gpuSceneDataBuffer = create_buffer(sizeof(GPUSceneData), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_CPU_TO_GPU);
//
//    // Add it to end of deletion queue of this frame
//    get_current_frame()._deletionQueue.push_function([=, this]() {
//        destroy_buffer(gpuSceneDataBuffer);
//    });
//
//    // Write buffer
//    GPUSceneData* sceneUniformData = (GPUSceneData*)gpuSceneDataBuffer.allocation->GetMappedData();
//    *sceneUniformData = sceneData;
//
//    // Create a descriptor set that binds that buffer and updates it
//    VkDescriptorSet globalDescriptor = get_current_frame()._frameDescriptors.allocate(_device, particleVertexComputeDescriptorSetLayout);
//
//    // Write GPU scene data
//    DescriptorWriter writer;
//    writer.write_buffer(0, gpuSceneDataBuffer.buffer, sizeof(GPUSceneData), 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
//    writer.update_set(_device, globalDescriptor);
//    
//    VkViewport viewport = {};
//    viewport.x = 0;
//    viewport.y = 0;
//    viewport.width = _drawExtent.width;
//    viewport.height = _drawExtent.height;
//    viewport.minDepth = 0.f;
//    viewport.maxDepth = 1.f;
//    vkCmdSetViewport(cmd, 0, 1, &viewport);
//
//    VkRect2D scissor = {};
//    scissor.offset.x = 0;
//    scissor.offset.y = 0;
//    scissor.extent.width = _drawExtent.width;
//    scissor.extent.height = _drawExtent.height;
//    vkCmdSetScissor(cmd, 0, 1, &scissor);
//
//    VkDeviceSize offsets[1] = { 0 };
//    vkCmdBindVertexBuffers(cmd, 0, 1, &particleShaderStorageBuffers[currentFrame].buffer, offsets);
//    
//    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, particleGraphicsPipeline);
//   
//    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, particleGraphicsPipelineLayout, 0, 1, &globalDescriptor, 0, nullptr);
//    
//    vkCmdDraw(cmd, PARTICLE_COUNT, 1, 0, 0);
//
//    vkCmdEndRendering(cmd);
//}