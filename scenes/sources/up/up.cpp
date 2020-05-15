#include "up.hpp"

#ifdef UP

using namespace vcl;

void scene_model::update_bird_visual() {
	float flap_speed = 2;
	float extrap = 0.5;
	float alpha = flap_speed * timer.t;
	alpha = alpha - (int)alpha; // get decimal fraction
	alpha *= alpha * alpha; // speed map
	alpha = -extrap + (1 + 2. * extrap) * abs(2 * (alpha - 0.5)); // get importance coefficient
	bird_shape = alpha * birds[0].position + (1 - alpha) * birds[1].position;
	normal(bird_shape, birds[0].connectivity, bird_shape_normals);
	visual_bird.update_position(bird_shape);
	visual_bird.update_normal(bird_shape_normals);
}

mesh scene_model::create_skybox()
{
	const vec3 p000 = { -1,-1,-1 };
	const vec3 p001 = { -1,-1, 1 };
	const vec3 p010 = { -1, 1,-1 };
	const vec3 p011 = { -1, 1, 1 };
	const vec3 p100 = { 1,-1,-1 };
	const vec3 p101 = { 1,-1, 1 };
	const vec3 p110 = { 1, 1,-1 };
	const vec3 p111 = { 1, 1, 1 };
	mesh skybox;
	skybox.position = {
		p000, p100, p110, p010,
		p010, p110, p111, p011,
		p100, p110, p111, p101,
		p000, p001, p010, p011,
		p001, p101, p111, p011,
		p000, p100, p101, p001
	};
	skybox.connectivity = {
		{0,1,2}, {0,2,3}, {4,5,6}, {4,6,7},
		{8,11,10}, {8,10,9}, {17,16,19}, {17,19,18},
		{23,22,21}, {23,21,20}, {13,12,14}, {13,14,15}
	};
	const float e = 1e-3f;
	const float u0 = 0.0f;
	const float u1 = 0.25f + e;
	const float u2 = 0.5f - e;
	const float u3 = 0.75f - e;
	const float u4 = 1.0f;
	const float v0 = 0.0f;
	const float v1 = 1.0f / 3.0f + e;
	const float v2 = 2.0f / 3.0f - e;
	const float v3 = 1.0f;
	skybox.texture_uv = {
		{u1,v1}, {u2,v1}, {u2,v2}, {u1,v2},
		{u1,v2}, {u2,v2}, {u2,v3}, {u1,v3},
		{u2,v1}, {u2,v2}, {u3,v2}, {u3,v1},
		{u1,v1}, {u0,v1}, {u1,v2}, {u0,v2},
		{u4,v1}, {u3,v1}, {u3,v2}, {u4,v2},
		{u1,v1}, {u2,v1}, {u2,v0}, {u1,v0}
	};
	return skybox;
}

void scene_model::display_skybox(std::map<std::string, GLuint>& shaders, scene_structure& scene) {
	glBindTexture(GL_TEXTURE_2D, texture_skybox);
	skybox.uniform.transform.scaling = 150.0f;
	skybox.uniform.transform.translation = scene.camera.camera_position() + vec3(0, 0, -50.0f);
	skybox.uniform.transform.rotation = rotation_from_axis_angle_mat3({ 1,0,1 }, 3.14f);
	draw(skybox, scene.camera, shaders["mesh_bf"], texture_skybox);
	glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}

void scene_model::update_balloons() {
	// Set forces & new fix position of the chimney
	const size_t N = particles.size();
	for (size_t k = 0; k < N; ++k) {
		strings[k].fix = house_boid.p + offset_chimney;
		particles[k].f = g + vec3(0, 5.f, 0);
	}

	// Integrate position and speed of particles through time

	for (size_t k = 0; k < N; ++k) {
		particle_structure& particle = particles[k];
		vec3& v = particle.v;
		vec3& p = particle.p;
		vec3& f = particle.f;

		if (!particle.drop) {
			// Afin d'�viter que le ballon reste sur le c�t� de la maison mais remonte
			if (abs(particles[k].p.x - strings[k].fix.x) > 0.8f)
				p = p - vec3((particles[k].p.x - strings[k].fix.x) * 2, 0, 0) / 200;
			if (abs(particles[k].p.z - strings[k].fix.z) > 0.5f)
				p = p - vec3(0, 0, (particles[k].p.z - strings[k].fix.z) * 2) / 200;
		}

		v = (1 - 0.9f * dt) * v + dt * f; // friction force + force
		p = p + dt * v;
		sub_particles[k].p = p - sub_vec;
	}

	for (size_t k = 0; k < N; ++k) {
		particle_structure& particle = particles[k];
		particle_structure& sub_particle = sub_particles[k];

		// La ficelle retient le ballon
		if (!particle.drop) {
			if (norm(particle.p - strings[k].fix) > particle.length_ficelle) {
				particle.v = -0.05f * norm(particle.v) * (particle.p - strings[k].fix);
				particle.p = particle.length_ficelle * (particle.p - strings[k].fix) / norm(particle.p - strings[k].fix) + strings[k].fix;
				sub_particles[k].p = particle.p - sub_vec;
			}
		}

		// Collisions entre les ballons
		for (size_t l = k + 1; l < N; ++l) {
			particle_structure& particle2 = particles[l];
			particle_structure& sub_particle2 = sub_particles[l];

			// Collision with the top of the balloon
			if ((norm(particle.p - particle2.p) <= particle.r + particle2.r)) {
				//std::cout << 'b' << std::endl;
				if (norm(particle.v - particle2.v) < 1) {
					particle.v = 0.5 * particle.v;
					particle2.v = 0.5 * particle2.v;
				}
				vec3 u = (particle.p - particle2.p) / norm(particle.p - particle2.p);
				float j = (2 * (particle.r * particle.r * particle.r) / (particle.r * particle.r * particle.r + particle2.r * particle2.r * particle2.r)) * dot(particle2.v - particle.v, u);

				particle.v = alpha_coll * particle.v + beta_coll * j * u;
				particle2.v = alpha_coll * particle2.v - beta_coll * j * u;

				// No divergence
				if (norm(particle.v) > 2.0f) {
					particle.v = { 0,0,0 };
				}
				if (norm(particle2.v) > 2.0f) {
					particle2.v = { 0,0,0 };
				}

				// No intracollision
				float d = particle.r + particle2.r - norm(particle.p - particle2.p);
				particle.p = particle.p + 0.5f * d * u;
				sub_particles[k].p = particle.p - sub_vec;
				particle2.p = particle2.p - 0.5f * d * u;
				sub_particles[l].p = particle2.p - sub_vec;

			}

			// Collision with the bottom of the balloon, represented by a second smaller particle
			if ((norm(particle.p - sub_particle2.p - 2 * sub_vec) <= particle.r + sub_particle2.r)) {
				if (norm(particle.v - particle2.v) < 1) {
					particle.v = 0.5 * particle.v;
					particle2.v = 0.5 * particle2.v;
				}
				vec3 u = (particle.p - sub_particle2.p - 2 * sub_vec) / norm(particle.p - sub_particle2.p - 2 * sub_vec);
				float j = (2 * (particle.r * particle.r * particle.r) / (particle.r * particle.r * particle.r + sub_particle2.r * sub_particle2.r * sub_particle2.r)) * dot(particle2.v - particle.v, u);

				particle.v = alpha_coll * particle.v + beta_coll * j * u;
				particle2.v = alpha_coll * particle2.v - beta_coll * j * u;

				// No divergence
				if (norm(particle.v) > 2.0f) {
					particle.v = { 0,0,0 };
				}
				if (norm(particle2.v) > 2.0f) {
					particle2.v = { 0,0,0 };
				}

				// No intra collision
				float d = particle.r + sub_particle2.r - norm(particle.p - sub_particle2.p - 2 * sub_vec);
				particle.p = particle.p + 0.5f * d * u;
				sub_particle.p = particle.p - sub_vec;
				particle2.p = particle2.p - 0.5f * d * u;
				sub_particle2.p = particle2.p - sub_vec;
			}
		}

		// String behavior
		string_structure& string = strings[k];
		string.life_span += dt;
		string.total_v = 0;
		std::vector<particle_structure> copy_parts = string.parts;
		vec3 plus(0.0f, -particle.r, 0.0f);
		string.parts[string.n_parts - 1].p = particle.p + 2.2f * plus;
		string.parts[0].p = string.fix;
		//const float L0 = 1.5f * particle.length_ficelle / string.n_parts; // Longueur � vide
		for (size_t ii = 1; ii < string.n_parts - 1; ii++) {
			vec3& v = string.parts[ii].v;
			vec3& p = string.parts[ii].p;

			// Explicit method to compute the force
			vec3 force = g + 300.0f * (copy_parts[ii + 1].p + copy_parts[ii - 1].p - 2 * copy_parts[ii].p);

			v = (1 - 8.0f * dt) * v + dt * force; // friction force + force
			if (norm(v) > 5.0f) {
				v = v / 10;
			}
			string.total_v += norm(v);
			p = p + dt * v;
		}
		// Behavior if the balloon is set free
		if (particle.drop) {
			string.parts[0].p = string.parts[0].p + vec3((particle.p.x - string.parts[0].p.x) / 30, (particle.p.y - particle.length_ficelle * 1.1f - string.parts[0].p.y), (particle.p.z - string.parts[0].p.z) / 30); // The length is fixed and the string follows the balloon
			particle.time_drop += dt;
		}

		// The string has only 2 points when it is stable
		if ((string.total_v < 0.005f) & (string.life_span > 2)) {
			string.parts.erase(string.parts.begin() + 1, string.parts.end() - 1);
			string.n_parts = 2;
		}
	}
}

void scene_model::update_remove() {
	for (int ii = 0; ii < particles.size(); ii++) {
		if (particles[ii].time_drop > 5) {
			particles.erase(particles.begin() + ii);
			sub_particles.erase(sub_particles.begin() + ii);
			strings.erase(strings.begin() + ii);
			break;
		}
	}
}

void scene_model::update_cylinder() {
	if (!motor) {
		// D�formation du tuyau lorsque la fum�e sort
		for (size_t k = 0; k < cylinder_shape.position.size() / N_sample_length; k++) {
			for (size_t h = 0; h < cylinder_shape.position.size() / N_sample_circumferential; h++) {
				float x = h / (cylinder_shape.position.size() / N_sample_circumferential);
				cylinder_shape.position[k * N_sample_length + h] = cylinder_shape.normal[k * N_sample_length] * (0.5f + abs(exp(-pow(0.5f - (x - log(modulo(time, T_smoke) + 1) * 6.f) - 2, 2) * 20)) * exp(x) * 0.1f) + vec3(0, 2 * x - 2.0f, 0);
			}
		}
		cylinder_visual.update_position(cylinder_shape.position);

		// Tuyau d'�chappement smoke
		if (abs(modulo(time, T_smoke) - 0.6f) < 0.05f) {
			particle_structure smoke = particle_structure(house_boid.p + vec3(-2.3f, -1.5f, 2.f));
			const float theta = rand_interval(0, 2 * 3.14f);
			const float rad = .2f;
			smoke.v = vec3(-.8f + rand_interval(0.0f, .2f), rad * std::sin(theta), rad * std::cos(theta));
			smoke.f = vec3(0, 0.5f, 0);
			smoke.time_drop = time;
			smokes.push_back(smoke);
		}
	}
	else {
		// Forme du tuyau en mode boost
		for (size_t k = 0; k < cylinder_shape.position.size() / N_sample_length; k++) {
			for (size_t h = 0; h < cylinder_shape.position.size() / N_sample_circumferential; h++) {
				float x = h / (cylinder_shape.position.size() / N_sample_circumferential);
				cylinder_shape.position[k * N_sample_length + h] = cylinder_shape.normal[k * N_sample_length] * (0.5f + abs(exp(-pow(0.5f - (x - log(modulo(.5f, T_smoke) + 1) * 6.f) - 2, 2) * 20)) * exp(x) * 0.1f) + vec3(0, 2 * x - 2.0f, 0);
			}
		}
		cylinder_visual.update_position(cylinder_shape.position);

		// Boosted smoke
		const float rad = rand_interval(0.1f, .8f);
		const float theta = rand_interval(0, 2 * 3.14f);
		particle_structure smoke = particle_structure(house_boid.p + vec3(-2.3f, -1.5f, 2.f));
		smoke.v = vec3(-3.0f + rand_interval(0.0f, 2.f), rad * std::sin(theta), rad * std::cos(theta));
		smoke.f = vec3(0, 0.f, 0);
		smoke.time_drop = time - 1.5f;
		smokes.push_back(smoke);
	}
}

void scene_model::update_sprites() {
	for (int i = 0; i < smokes.size(); i++) {
		if (time - smokes[i].time_drop > 2.f) {
			smokes.erase(smokes.begin() + i, smokes.begin() + i + 1);
		}
		break;
	};
	for (int i = 0; i < smokes.size(); i++) {
		if (time - smokes[i].time_drop > 1.5f) {
			smokes.erase(smokes.begin() + i, smokes.begin() + i + 1);
		}
		break;
	};
	for (int i = 0; i < smokes.size(); i++) {
		smokes[i].v = smokes[i].v + smokes[i].f * dt;
		smokes[i].p = smokes[i].p + smokes[i].v * dt;
	};
}

void scene_model::keyboard_input(scene_structure& scene, GLFWwindow* window, int key, int scancode, int action, int mods) {
	const bool key_up = glfwGetKey(window, GLFW_KEY_UP);
	const bool key_down = glfwGetKey(window, GLFW_KEY_DOWN);
	const bool key_right = glfwGetKey(window, GLFW_KEY_RIGHT);
	const bool key_left = glfwGetKey(window, GLFW_KEY_LEFT);
	// Pushing the house if arrow key pressed
	if (key_up) {
		motor = true;
		if (abs(boost_force.x) < 3.0)
			boost_force = boost_force + vec3(0.4f, 0, 0);
	}
	else if (key_down) {
		motor = true;
		if (abs(boost_force.x) < 3.0)
			boost_force = boost_force - vec3(0.4f, 0, 0);
	}
	else if (key_right) {
		motor = true;
		if (abs(boost_force.z) < 3.0)
			boost_force = boost_force + vec3(0, 0, 0.4f);
	}
	else if (key_left) {
		motor = true;
		if (abs(boost_force.z) < 3.0)
			boost_force = boost_force - vec3(0, 0, 0.4f);
	}
	// No boost_force on the house if no arrow key pressed
	else {
		motor = false;
		boost_force = vec3(0, 0, 0);
	}
}

void scene_model::update_house() {
	// Number of attached balloons
	int n_attached = 0;
	for (int i = 0; i < particles.size(); i++) {
		if (!particles[i].drop) n_attached = n_attached + 1;
	}

	// force applied on the house
	house_force = 10 * g + n_attached * vec3(0, 1.f * exp(-house_boid.p.y / 10.f), 0) - house_boid.v;
	house_boid.v = house_boid.v + (house_force + boost_force) * dt;
	if (house_boid.p.y <= 0) {
		if (house_boid.v.y < 0) {
			house_boid.v.y = 0;
		}
		house_boid.p.y = house_boid.v.y * dt;
	}
	else {
		house_boid.p = house_boid.p + house_boid.v * dt;
	}
	if (house_boid.p.y < 0) house_boid.p.y = 0;
};

void scene_model::setup_data(std::map<std::string, GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
	// to see better the scene, the camera is zoomed out since the beginning
	scene.camera.apply_scaling(5);

	texture_skybox = create_texture_gpu(image_load_png("scenes/sources/up/assets/skybox.png"));
	skybox = create_skybox();
	skybox.uniform.shading = { 1,0,0 };

	birds.push_back(mesh_load_file_obj("scenes/sources/up/assets/bird2.obj"));
	birds.push_back(mesh_load_file_obj("scenes/sources/up/assets/bird1.obj"));

	visual_bird = birds[0];
	visual_bird.shader = shaders["mesh"];
	visual_bird.uniform.shading.specular = 0.f;
	visual_bird.uniform.shading.diffuse = 0.8f;
	visual_bird.uniform.color = vec3(1, 1, 1);

	birds[0].fill_empty_fields();
	birds[1].fill_empty_fields();
	bird_shape_normals = birds[0].normal;

	house = mesh_load_file_obj("scenes/sources/up/assets/house4.obj");
	texture_house = create_texture_gpu(image_load_png("scenes/sources/up/assets/wood3.png"));
	house.shader = shaders["mesh"];
	house.uniform.color = vec3(1, 1, 1);
	house.uniform.transform.scaling = 0.025;
	house.uniform.shading.ambiant = 0.5f;
	house.uniform.shading.diffuse = 0.5f;
	house.uniform.shading.specular = 0.1f;
	house.uniform.shading.specular_exponent = 8;

	for (int i = 0; i < 20; i++) {
		boids.push_back(Boid(vec3(5 - 10 * (float)rand() / (RAND_MAX), 5 - 10 * (float)rand() / (RAND_MAX), 5 - 10 * (float)rand() / (RAND_MAX)) + vec3(0, 10, 0), vec3(0, 0, 0)));
	}

	rod = mesh_primitive_parallelepiped(vec3(0, 0, 0), vec3(0.1, 0, 0), vec3(0, 0.1, 0), vec3(0, 0, 2.5));
	rod.shader = shaders["mesh"];
	rod.uniform.color = vec3(0.5, 0.4, 0.3);
	rod.uniform.shading.ambiant = 0.7f;
	rod.uniform.shading.diffuse = 0.7f;
	rod.uniform.shading.specular = 0.9f;
	rod.uniform.shading.specular_exponent = 128;

	curtain1.origin = vec3(-1, 0, -3.5);
	curtain1.u1 = vec3(0, 0, 2);
	curtain1.u2 = vec3(0, -2, 0);
	curtain1.initialize();
	curtain1.cloth.shader = shaders["mesh"];

	curtain2.origin = vec3(-1, 0, 2.5);
	curtain2.u1 = vec3(0, 0, 2);
	curtain2.u2 = vec3(0, -2, 0);
	curtain2.initialize();
	curtain2.cloth.shader = shaders["mesh"];

	balloon = mesh_load_file_obj("scenes/sources/up/assets/balloon.obj");
	balloon.shader = shaders["mesh"];
	balloon.uniform.color = vec3(1, 1, 1);
	balloon.uniform.transform.scaling = 1.f;
	balloon.uniform.shading.ambiant = 0.5f;
	balloon.uniform.shading.specular = 0.1f;
	balloon.uniform.shading.specular_exponent = 100;

	shader_string = shaders["curve"];

	// Tuyaux
	cylinder_shape = mesh_primitive_cylinder(radius, { 0,-h / 2,0 }, { 0,h / 2,0 }, N_sample_circumferential, N_sample_length);
	cylinder_visual = cylinder_shape;
	cylinder_visual.shader = shaders["mesh"];
	cylinder_visual.uniform.transform.scaling = 0.73f;

	// Smoke
	sprite = mesh_primitive_quad();
	sprite.shader = shaders["mesh"];

	sprite.uniform.shading = { 1,0,0 };
	sprite.uniform.transform.scaling = 0.75f;
	sprite.texture_id = create_texture_gpu(image_load_png("scenes/sources/up/assets/smoke.png"));


	//mesh ground = mesh_primitive_quad(vec3(-100, 0, -100), vec3(100, 0, -100), vec3(100, 0, 100), vec3(-100, 0, 100));
	//ground.texture_uv = { {-100, -100}, { 100,-100 }, { 100,100 }, { -100,100 } };
	mesh ground = mesh_primitive_parallelepiped(vec3(-20, 0, -20), vec3(40, 0, 0), vec3(0, 0, 40), vec3(0, -0.5, 0));
	ground.texture_uv = { {-20,-20}, {-20,20}, {20,20}, {20,-20},
				  {-20,-20}, {-20,20}, {20,20}, {20,-20},
				  {-20,-20}, {-20,20}, {20,20}, {20,-20},
				  {-20,-20}, {-20,20}, {20,20}, {20,-20},
				  {-20,-20}, {-20,20}, {20,20}, {20,-20},
				  {-20,-20}, {-20,20}, {20,20}, {20,-20} };
	ground_draw = mesh_drawable(ground);
	ground_draw.shader = shaders["mesh"];
	ground_draw.uniform.transform.translation = vec3(0, -2.f, 0);
	ground_draw.uniform.shading.ambiant = 0.6;
	ground_draw.uniform.shading.diffuse = 0.5;
	ground_draw.uniform.shading.specular = 0.;
	ground_draw.texture_id = create_texture_gpu(image_load_png("scenes/sources/up/assets/grass.png"));

	gui.show_frame_worldspace = false;
	gui.show_frame_camera = false;

}

void scene_model::frame_draw(std::map<std::string, GLuint>& shaders, scene_structure& scene, gui_structure& gui)
{
	set_gui();
	timer.update();
	dt = (timer.t - time);
	time = timer.t;

	display_skybox(shaders, scene);

	glBindTexture(GL_TEXTURE_2D, ground_draw.texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	draw(ground_draw, scene.camera);
	glBindTexture(GL_TEXTURE_2D, scene.texture_white);

	if (show_boids) {
		update_bird_visual();
		for (Boid& boid : boids) {
			boid.follow(house_boid);
			boid.update_boid(boids);
			visual_bird.uniform.color = vec3(1, 1, 1);
			visual_bird.uniform.transform.translation = vec3(0, abs(std::sin(2 * time * 3.14f)) / 10, 0) + boid.p;
			visual_bird.uniform.transform.rotation = rotation_between_vector_mat3(vec3(0, 0, 1), boid.v) * rotation_from_axis_angle_mat3({ 1,0,0 }, std::_Pi / 6);
			draw(visual_bird, scene.camera);
		}
	}

	if (show_balloons) {
		update_remove();
		update_balloons();
		for (int i = 0; i < particles.size(); i++) {
			balloon.uniform.color = particles[i].c;
			balloon.uniform.transform.scaling = particles[i].scale / 3.5f;
			balloon.uniform.transform.translation = particles[i].p - vec3(0, 0.71f, 0);
			draw(balloon, scene.camera);

			const string_structure& string = strings[i];
			for (size_t ii = 0; ii < string.n_parts - 1; ii++) {
				std::vector<vec3> segment_string_part = { string.parts[ii].p,string.parts[ii + 1].p };
				string_part = segments_gpu(segment_string_part);
				string_part.uniform.color = { 0.2f,0.2f,0.2f };
				string_part.shader = shader_string;
				draw(string_part, scene.camera);
			}
		}
	}

	if (show_house) {
		update_house();
		house.uniform.transform.translation = house_boid.p + vec3(1.5, -2, 3);

		glBindTexture(GL_TEXTURE_2D, texture_house);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		draw(house, scene.camera, texture_house);
		glBindTexture(GL_TEXTURE_2D, scene.texture_white);

		glBindTexture(GL_TEXTURE_2D, texture_house);
		rod.uniform.transform.translation = house_boid.p + vec3(-1, 0, 2.5);
		draw(rod, scene.camera, texture_house);
		rod.uniform.transform.translation = house_boid.p + vec3(-1, 0, -4);
		draw(rod, scene.camera, texture_house);
		glBindTexture(GL_TEXTURE_2D, scene.texture_white);

		for (size_t ii = 0; ii < curtain1.N_cloth; ii++) {
			curtain1.positional_constraints[ii] = house_boid.p + vec3(-1, 0, -3.5) - curtain1.position[0] + curtain1.position[ii];
		}
		curtain1.positional_constraints[curtain1.N_cloth * curtain1.N_cloth - 1] = house_boid.p + vec3(-1, 0, -3.5) - curtain1.position[0] + curtain1.position[curtain1.N_cloth * curtain1.N_cloth - 1];
		curtain1.update_curtain();
		glBindTexture(GL_TEXTURE_2D, curtain1.cloth.texture_id);
		draw(curtain1.cloth, scene.camera, curtain1.cloth.shader, curtain1.cloth.texture_id);
		glBindTexture(GL_TEXTURE_2D, scene.texture_white);

		for (size_t ii = 0; ii < curtain2.N_cloth; ii++) {
			curtain2.positional_constraints[ii] = house_boid.p + vec3(-1, 0, 2.5) - curtain2.position[0] + curtain2.position[ii];
		}
		curtain2.positional_constraints[curtain2.N_cloth * (curtain2.N_cloth - 1)] = house_boid.p + vec3(-1, 0, 2.5) - curtain2.position[0] + curtain2.position[curtain2.N_cloth * (curtain2.N_cloth - 1)];
		curtain2.update_curtain();
		glBindTexture(GL_TEXTURE_2D, curtain2.cloth.texture_id);
		draw(curtain2.cloth, scene.camera, curtain2.cloth.shader, curtain2.cloth.texture_id);
		glBindTexture(GL_TEXTURE_2D, scene.texture_white);

		// Cylinder (pipes)
		update_cylinder();

		cylinder_visual.uniform.transform.scaling = 0.1f;
		cylinder_visual.uniform.transform.rotation = mat3(0, -1, 0, 1, 0, 0, 0, 0, 1);
		cylinder_visual.uniform.transform.translation = house_boid.p + vec3(-2.15f, -1.5f, 2.f);
		draw(cylinder_visual, scene.camera);

		cylinder_visual.uniform.transform.scaling = 0.1f;
		cylinder_visual.uniform.transform.rotation = mat3(0, -1, 0, 1, 0, 0, 0, 0, 1);
		cylinder_visual.uniform.transform.translation = house_boid.p + vec3(-2.4f, -1.5f, -.4f);
		draw(cylinder_visual, scene.camera);

		// Smoke
		update_sprites();
		glEnable(GL_BLEND);
		glDepthMask(false);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		for (int i = 0; i < smokes.size(); i++) {
			// Color of the boosted (or not) smoke
			if (abs(smokes[i].v.x) > .8f) {
				sprite.uniform.color = { 1.f,.8f - .6f * pow(pow(smokes[i].v.y,2) + pow(smokes[i].v.z,2), .5f),rand_interval(0.1f, .3f) };
			}
			else {
				sprite.uniform.color = { 0.6,0.6,0.6 };
			}

			sprite.uniform.transform.rotation = scene.camera.orientation;
			sprite.uniform.transform.translation = smokes[i].p;
			sprite.uniform.color_alpha = 0.5f;
			draw(sprite, scene.camera);

			sprite.uniform.transform.rotation = scene.camera.orientation;
			sprite.uniform.transform.translation = smokes[i].p + vec3(-0.25f, 0, -2.4f);
			sprite.uniform.color_alpha = 0.5f;
			draw(sprite, scene.camera);
		}
		glDepthMask(true);

	}

	glBindTexture(GL_TEXTURE_2D, scene.texture_white);
}

void scene_model::set_gui()
{
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f /*min value*/, 4.0f /*max value*/);
	bool const stop = ImGui::Button("Stop"); ImGui::SameLine();
	bool const start = ImGui::Button("Start");

	ImGui::Checkbox("Show House", &show_house);
	ImGui::Checkbox("Show Boids", &show_boids);
	ImGui::Checkbox("Show Balloons", &show_balloons);

	bool add = ImGui::Button("Add"); ImGui::SameLine();
	bool remove = ImGui::Button("Remove");

	if (add) {
		// Add one particle (represents the top of the balloon)
		//particle_structure part = particle_structure(house_boid.p + offset_chimney);
		particle_structure part = particle_structure(house_boid.p + offset_chimney + vec3(0, 0.5f, 0));
		particles.push_back(part);

		// Invisible, only for collision detection
		particle_structure new_sub_particle;
		new_sub_particle.r = part.r / 1.2f;
		// Initial position
		new_sub_particle.p = part.p - sub_vec;
		sub_particles.push_back(new_sub_particle);

		// Add the string
		string_structure string = string_structure(house_boid.p + offset_chimney);
		strings.push_back(string);
	}
	if (remove) {
		if (particles.size() > 0) {
			int i = 0;
			while (i < particles.size() & particles[i].drop) {
				i++;
			}
			if (!particles[i].drop) {
				particles[i].drop = true;
			}
		}
	}

	if (stop)  timer.stop();
	if (start) timer.start();


}

#endif