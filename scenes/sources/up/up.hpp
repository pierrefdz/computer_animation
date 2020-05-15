#pragma once

#include "scenes/base/base.hpp"
#include "scenes/sources/up/boid.hpp"
#include "scenes/sources/up/curtain.hpp"
#include "scenes/sources/up/balloon.hpp"

#ifdef UP

struct scene_model : scene_base {

	void setup_data(std::map<std::string, GLuint>& shaders, scene_structure& scene, gui_structure& gui);
	void frame_draw(std::map<std::string, GLuint>& shaders, scene_structure& scene, gui_structure& gui);
	void set_gui();

	// Gravity
	vcl::vec3 g = vcl::vec3(0.f, -0.4f, 0.f);

	float dt;
	float time;

	// Ground
	vcl::mesh_drawable ground_draw;
	GLuint texture_ground;

	// House
	vcl::mesh_drawable house;
	GLuint texture_house;
	vcl::vec3 offset_chimney = vcl::vec3(-0.4f, 2.9f, 0.8f);
	vcl::vec3 house_force;
	vcl::vec3 boost_force;
	void update_house();

	// Curtain
	vcl::mesh_drawable rod;
	Curtain curtain1 = Curtain();
	Curtain curtain2 = Curtain();
	GLuint texture_curtain;

	// Skybox
	vcl::mesh_drawable skybox;
	GLuint texture_skybox;
	vcl::mesh create_skybox();
	void display_skybox(std::map<std::string, GLuint>& shaders, scene_structure& scene);

	// Animated Birds
	std::vector<vcl::mesh> birds;
	vcl::mesh_drawable visual_bird;
	vcl::buffer<vcl::vec3> bird_shape;
	vcl::buffer<vcl::vec3> bird_shape_normals;
	void update_bird_visual();

	// Boids
	std::vector<Boid> boids;
	Boid house_boid = Boid();

	// Balloons
	vcl::mesh_drawable balloon;
	std::vector<particle_structure> particles;
	std::vector<particle_structure> sub_particles;
	vcl::vec3 sub_vec = vcl::vec3(0, 0.22f, 0);
	std::vector<string_structure> strings;
	vcl::segments_drawable string_part;
	GLuint shader_string;
	float alpha_coll = 2.0f; // Collision parameters
	float beta_coll = 2.0f;

	void update_balloons();
	void update_remove();

	// Pipes
	const float h = 1.5f;
	const float radius = 0.4f;
	const float N_sample_circumferential = 80;
	const float N_sample_length = int(h / (2 * 3.14f * radius) * (N_sample_circumferential - 1) + 1 + 0.5f);
	vcl::mesh cylinder_shape;
	vcl::mesh_drawable cylinder_visual;
	float T_smoke = 1.f; // Period of the smoke

	float modulo(float a, float b) { // modulo for floats
		float r = a;
		while (r - b > 0) {
			r = r - b;
		}
		return r;
	}

	void update_cylinder();

	// Smoke
	std::vector<particle_structure> smokes;
	vcl::mesh_drawable sprite;
	void update_sprites();

	// Motor on or off (boost or not)
	bool motor;
	void keyboard_input(scene_structure& scene, GLFWwindow* window, int key, int scancode, int action, int mods);

	// Timer used for the animation
	vcl::timer_basic timer;

	bool show_house = true;
	bool show_boids = true;
	bool show_balloons = true;


};

#endif