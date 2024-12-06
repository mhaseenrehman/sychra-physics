#pragma once
#include <glm/gtx/transform.hpp>

struct Particle {
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;
};
