#include "balloon.hpp"

#ifdef UP

using namespace vcl;
using namespace std;

float dt = 0.01f;

vec3 g(0.f, -0.5f, 0.f);
static const std::vector<vec3> color_lut = { {0.8,0.3,0.3},{0.3,0.7,0.2},{0.3,0.3,0.8},{0.7,0.7,0.2},{0.7,0.2,0.7},{0.2,0.7,0.5} };

particle_structure::particle_structure(vec3 _p) {
    scale = rand_interval(1.f,1.2f);
    r = 0.28f * scale;
    c = color_lut[int(rand_interval() * color_lut.size())];

    // Initial position
    p = _p;

    // Initial speed
    const float theta = rand_interval(0, 2 * 3.14f);
    v = vec3(0.8f * std::cos(theta), 0.8f, 0.8f * std::sin(theta));

    const float random_length = rand_interval(0, 0.5f);
    length_ficelle = 4.f + random_length;
}

particle_structure::particle_structure() {
    scale = rand_interval(1.f, 1.2f);
    r = 0.28f * scale;
    c = color_lut[int(rand_interval() * color_lut.size())];

    // Initial position
    p = vec3(0,0.5f,0);

    // Initial speed
    const float theta = rand_interval(0, 2 * 3.14f);
    v = 3*vec3(0.8f * std::cos(theta), 0.8f, 0.8f * std::sin(theta));
    const float random_length = rand_interval(0, 0.5f);
    length_ficelle = 4.f + random_length;
}

string_structure::string_structure() {
    const float r = rand_interval(0, 0.1f);
    const float theta = rand_interval(0, 3.14f);
    fix = vec3(r * std::cos(theta), 0.0f, r * std::sin(theta));
    for (size_t ii = 0; ii < n_parts; ii++) {
        particle_structure new_part;
        new_part.p = (n_parts - 1 - ii) * fix / (n_parts - 1);
        new_part.v = { 0,0,0 };
        parts.push_back(new_part);
    }
}

string_structure::string_structure(vec3 _p) {
    const float r = rand_interval(0, 0.1f);
    const float theta = rand_interval(0, 3.14f);
    fix = _p + vec3(r * std::cos(theta), 0.0f, r * std::sin(theta));;
    for (size_t ii = 0; ii < n_parts; ii++) {
        particle_structure new_part;
        new_part.p = fix + (n_parts - 1 - ii) * vec3(0,0.2f,0) / (n_parts - 1);
        new_part.v = { 0,0,0 };
        parts.push_back(new_part);
    }
}

#endif