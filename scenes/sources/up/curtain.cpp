#include "curtain.hpp"

#ifdef UP
using namespace vcl;

static vec3 spring_force(const vec3& pi, const vec3& pj, float L0, float K)
{
	const vec3 p = pi - pj;
	float L = norm(p);
	const vec3 u = normalize(p);

	const vec3 F = -K * (L - L0) * u;
	return F;
}

Curtain::Curtain(vec3 _origin, vec3 _u1, vec3 _u2, std::map<int, vec3> _positional_constraints) {
	origin = _origin;
	u1 = _u1;
	u2 = _u2;
	positional_constraints = _positional_constraints;
};

Curtain::Curtain() {
	origin = vec3(0, 0, 0);
	u1 = vec3(1, 0, 0);
	u2 = vec3(0, -1, 0);
};

void Curtain::initialize() {

	L0 = norm(u1) / float(N_cloth - 1);
	wind = vec3(0, 0, 0);

	const mesh base_cloth = mesh_primitive_grid(N_cloth, N_cloth, origin, u1, u2);
	position = buffer2D_from_vector(base_cloth.position, N_cloth, N_cloth);

	speed.resize(position.dimension); speed.fill({ 0,0,0 });
	force.resize(position.dimension); force.fill({ 0,0,0 });

	connectivity = base_cloth.connectivity;
	normals = normal(position.data, connectivity);

	// Send data to GPU
	cloth.clear();
	cloth = mesh_drawable(base_cloth);
	cloth.uniform.shading.specular = 0.0f;
	cloth.uniform.shading.diffuse = 0.5f;
	cloth.uniform.shading.ambiant = 0.5f;

	texture_cloth = create_texture_gpu(image_load_png("scenes/sources/up/assets/curtain.png"));
	cloth.texture_id = texture_cloth;

	simulation_diverged = false;
	force_simulation = true;

	timer.update();
}

void Curtain::compute_forces()
{
	const size_t N = force.size();        // Total number of particles of the cloth Nu x Nv
	const int N_dim = force.dimension[0]; // Number of particles along one dimension (square dimension)

	// Gravity
	const vec3 g = { 0,-9.81f,0 };
	for (size_t k = 0; k < N; ++k)
		force[k] = m * g;

	// Drag
	for (size_t k = 0; k < N; ++k)
		force[k] = force[k] - mu * speed[k];

	// Springs
	const int N_neighbors = 2;
	for (int ku = 0; ku < N_dim; ++ku) {
		for (int kv = 0; kv < N_dim; ++kv) {
			vec3& f = force(ku, kv);

			// Loops over neighbors
			for (int du = -N_neighbors; du <= N_neighbors; ++du) {
				for (int dv = -N_neighbors; dv <= N_neighbors; ++dv) {
					if (du != 0 || dv != 0)
					{
						// Neighbors indices
						const int ku_n = ku + du;
						const int kv_n = kv + dv;
						const float alpha = float(std::sqrt(du * du + dv * dv)); // rest-length

						if (ku_n >= 0 && ku_n < N_dim && kv_n >= 0 && kv_n < N_dim)
							f += spring_force(position(ku, kv), position(ku_n, kv_n), alpha * L0, K / alpha);
					}
				}
			}
		}
	}

	// Wind
	const vec3 wind_u = normalize(wind);
	for (size_t k = 0; k < N; ++k)
	{
		const vec3& n = normals[k];
		const float wind_magnitude = dot(wind, n);
		const vec3 f = wind_magnitude * L0 * L0 * n;

		force[k] += f;
	}


}

void Curtain::detect_simulation_divergence()
{
	const size_t NN = position.size();
	for (size_t k = 0; simulation_diverged == false && k < NN; ++k)
	{
		const float f = norm(force[k]);
		const vec3& p = position[k];

		if (std::isnan(f)) // detect NaN in force
		{
			std::cout << "NaN detected in forces" << std::endl;
			simulation_diverged = true;
		}

		if (f > 1000.0f) // detect strong force magnitude
		{
			std::cout << " **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
			simulation_diverged = true;
		}

		if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
		{
			std::cout << "NaN detected in positions" << std::endl;
			simulation_diverged = true;
		}

		if (simulation_diverged == true)
		{
			std::cerr << " **** Simulation has diverged **** " << std::endl;
			std::cerr << " > Stop simulation iterations" << std::endl;
			timer.stop();
		}
	}

}

void Curtain::hard_constraints()
{
	for (const auto& constraints : positional_constraints)
		position[constraints.first] = constraints.second;
}

void Curtain::numerical_integration(float h)
{
	const size_t NN = position.size();

	for (size_t k = 0; k < NN; ++k)
	{
		vec3& p = position[k];
		vec3& v = speed[k];
		const vec3& f = force[k];

		v = v + h * f / m;
		p = p + h * v;
	}
}

void Curtain::update_curtain() {
	const float dt = timer.update();
	// Force constant simulation time step
	float h = 0.005;
	// Iterate over a fixed number of substeps between each frames
	const size_t number_of_substeps = 4;
	for (size_t k = 0; (!simulation_diverged || force_simulation) && k < number_of_substeps; ++k)
	{
		compute_forces();
		numerical_integration(h);
		hard_constraints();                      // Enforce hard positional constraints
		normal(position.data, connectivity, normals); // Update normals of the cloth
		detect_simulation_divergence();               // Check if the simulation seems to diverge
	}

	cloth.update_position(position.data);
	cloth.update_normal(normals.data);
}

#endif