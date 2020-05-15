#pragma once
#include "scenes/base/base.hpp"

#ifdef UP

class Boid {
private:
	// parameters
	float lambda_cohes = 0.2;
	float lambda_separ = 2.0;
	float lambda_align = 0.5;
	float lambda_ground = 1.;
	float delta_close = 2;
	float time_step = 0.005;
	float max_speed = 5.0;
	// the 3 forces defined by Reynolds
	vcl::vec3 f_cohes;
	vcl::vec3 f_separ;
	vcl::vec3 f_align;
	vcl::vec3 f_ground;
	// an id to identify the boid
	float id;

public:
	// the position and the velocity
	vcl::vec3 p;
	vcl::vec3 v;
	//constructors
	Boid(vcl::vec3 _p, vcl::vec3 _v);
	Boid();
	// boid's methods
	void update_forces(std::vector<Boid> boids);
	void update_boid(std::vector<Boid> boids);
	void follow(Boid lead, float lambda_lead = 1.5);
	void clip_v();
};

#endif