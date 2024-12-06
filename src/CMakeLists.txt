# Physics source root
set(SYCHRA_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/physics)

# Physics source files
set(SYCHRA_SOURCE_FILES 
		${SYCHRA_SOURCE_ROOT}/phy_engine.cpp 
		${SYCHRA_SOURCE_ROOT}/phy_engine.h)

# Create library
add_library(physics ${SYCHRA_SOURCE_FILES})

# Add dependencies to library
target_link_libraries(physics PUBLIC glm fmt::fmt)

# Renderer Source Root
set(RENDERER_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/renderer)

set(RENDERER_SOURCE_FILES 
		${RENDERER_SOURCE_ROOT}/main.cpp
  		${RENDERER_SOURCE_ROOT}/vk_types.h
  		${RENDERER_SOURCE_ROOT}/vk_initializers.cpp
  		${RENDERER_SOURCE_ROOT}/vk_initializers.h
  		${RENDERER_SOURCE_ROOT}/vk_images.h
  		${RENDERER_SOURCE_ROOT}/vk_images.cpp 
  		${RENDERER_SOURCE_ROOT}/vk_descriptors.h
  		${RENDERER_SOURCE_ROOT}/vk_descriptors.cpp
  		${RENDERER_SOURCE_ROOT}/vk_pipelines.h
  		${RENDERER_SOURCE_ROOT}/vk_pipelines.cpp
  		${RENDERER_SOURCE_ROOT}/vk_engine.h
  		${RENDERER_SOURCE_ROOT}/vk_engine.cpp
  		${RENDERER_SOURCE_ROOT}/vk_loader.h
  		${RENDERER_SOURCE_ROOT}/vk_loader.cpp
  		${RENDERER_SOURCE_ROOT}/camera.cpp
  		${RENDERER_SOURCE_ROOT}/camera.h)

# Add source to this project's executable.
add_executable (engine ${RENDERER_SOURCE_FILES})

set_property(TARGET engine PROPERTY CXX_STANDARD 20)
target_compile_definitions(engine PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)
target_include_directories(engine PUBLIC 
        "${CMAKE_CURRENT_SOURCE_DIR}/renderer"
        "${SYCHRA_SOURCE_ROOT}")

target_link_libraries(engine PUBLIC vma glm Vulkan::Vulkan fmt::fmt physics stb_image SDL2::SDL2 vkbootstrap imgui fastgltf::fastgltf )

target_precompile_headers(engine PUBLIC <optional> <vector> <memory> <string> <vector> <unordered_map> <glm/mat4x4.hpp>  <glm/vec4.hpp> <vulkan/vulkan.h>)

add_custom_command(TARGET engine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:engine> $<TARGET_FILE_DIR:engine>
  COMMAND_EXPAND_LISTS
  )