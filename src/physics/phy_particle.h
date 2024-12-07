#pragma once
#include <glm/gtx/transform.hpp>

struct Particle {
	// Standard Point Mass Properties
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 acceleration;

	// Mass of Particle
	float mass;

	// Damping / Drag
	float damping;
};
