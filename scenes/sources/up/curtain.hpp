#pragma once
#include "scenes/base/base.hpp"

#ifdef UP

class Curtain {
private:
	float m = 0.1f;    // Global mass (to be divided by the number of particles)
	float K = 300.f;    // Global stiffness (to be divided by the number of particles)
	float mu = 0.3f;   // Damping

public:
	int N_cloth = 10;
	float L0;	// spring rest length
	vcl::vec3 wind;
	vcl::vec3 origin;
	vcl::vec3 u1;
	vcl::vec3 u2;

	vcl::buffer2D<vcl::vec3> position;
	vcl::buffer2D<vcl::vec3> speed;
	vcl::buffer2D<vcl::vec3> force;

	vcl::mesh_drawable cloth;				// Visual model for the cloth    
	GLuint texture_cloth;					// Texture
	vcl::buffer<vcl::vec3> normals;			// Normal of the cloth used for rendering and wind force computation
	vcl::buffer<vcl::uint3> connectivity;	// Connectivity of the triangular model

	// Store index and position of vertices constrained to have a fixed 3D position
	std::map<int, vcl::vec3> positional_constraints;

	// Parameters used to control if the simulation runs when a numerical divergence is detected
	bool simulation_diverged; // Active when divergence is detected
	bool force_simulation;    // Force to run simulation even if divergence is detected

	Curtain(vcl::vec3 _origin, vcl::vec3 _u1, vcl::vec3 _u2, std::map<int, vcl::vec3> _positional_constraints);
	Curtain();

	void initialize();
	void compute_forces();
	void numerical_integration(float h);
	void hard_constraints();
	void update_curtain();
	void detect_simulation_divergence();

	vcl::timer_event timer;
};

#endif