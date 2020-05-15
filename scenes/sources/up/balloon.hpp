#pragma once
#include "scenes/base/base.hpp"

#ifdef UP

struct particle_structure {
    vcl::vec3 p; // Position
    vcl::vec3 v; // Speed
    vcl::vec3 f; // Forces

    vcl::vec3 c; // Color
    float r;     // Radius
    float scale;
    float length_ficelle; // Longueur de la ficelle

    bool drop = false;
    float time_drop = 0;

    particle_structure::particle_structure();
    particle_structure::particle_structure(vcl::vec3 _p);
};

struct string_structure {
    std::vector<particle_structure> parts;
    vcl::vec3 fix;
    int n_parts = 10;
    float life_span = 0;
    float total_v;

    string_structure::string_structure();
    string_structure::string_structure(vcl::vec3 _p);
};

#endif