#include "boid.hpp"
#ifdef UP

using namespace vcl;

Boid::Boid() {
	p = vec3(0, 0, 0);
	v = vec3(0, 0, 0);
	f_cohes = vec3(0, 0, 0);
	f_separ = vec3(0, 0, 0);
	f_align = vec3(0, 0, 0);
	id = ((float)rand() / (RAND_MAX));
};

Boid::Boid(vcl::vec3 _p, vcl::vec3 _v) {
	p = _p;
	v = _v;
	f_cohes = vec3(0, 0, 0);
	f_separ = vec3(0, 0, 0);
	f_align = vec3(0, 0, 0);
	id = ((float)rand() / (RAND_MAX));
};

void Boid::update_forces(std::vector<Boid> boids) {
	int N = boids.size();
	vec3 cp = vec3(0, 0, 0);
	float n_close = 0.;
	vec3 cp_close = vec3(0, 0, 0);
	vec3 cv = vec3(0, 0, 0);
	float d = 0.;
	for (Boid boid : boids) {
		if (boid.id != id) {
			d = norm(boid.p - p);
			if (d < 0.1) {
				p += 5 - 10 * (float)rand() / (RAND_MAX);
				break;
			}
			cp += boid.p;
			if (d < 5 * delta_close) {
				if (d < delta_close) {
					cv += boid.v;
					cp_close += boid.p;
					n_close += 1;
				}
				else {
					cv += 0.25 * boid.v;
					cp_close += 0.25 * boid.p;
					n_close += 0.25;
				}
			}
		}
	}
	cp = 1 / N * cp;
	cp_close = (n_close != 0) ? 1 / n_close * cp_close : p;
	cv = (n_close != 0) ? 1 / n_close * cv : v;
	f_cohes = lambda_cohes * (cp - p);
	f_separ = lambda_separ * (p - cp_close);
	f_align = lambda_align * (cv - v);
	f_ground = lambda_ground * (10 / p.y) * vec3(0, 1, 0);
};

void Boid::update_boid(std::vector<Boid> boids) {
	update_forces(boids);
	v += time_step * (f_cohes + f_separ + f_align + f_ground);
	clip_v();
	p += time_step * v;
};

void Boid::clip_v() {
	if (norm(v) > max_speed) {
		v = max_speed / norm(v) * v;
	}
}

void Boid::follow(Boid lead, float lambda_lead) {
	vec3 f_follow(0, 0, 0);
	float d_carac = 6.f;
	float d = norm(lead.p - p);
	f_follow += lambda_lead * (log(d / d_carac) + (d / d_carac) * (d / d_carac)) * (lead.p - p) / d;
	f_follow += 0.5 * lambda_lead * norm(lead.v) * (lead.v - v);
	v += time_step * f_follow;
	clip_v();
	p += time_step * v;
}


#endif