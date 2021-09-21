#pragma once

#include <cmath>
#include <array>
#include <random>
#include "particle.hpp"

// N num particles
template <int N>
class Robot {
public:
    std::array<Particle, N> particles;

    Robot(double x, double y,
         double xvel, double yvel,
         double accelsigma) {
        
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> d{0, accelsigma};

        for (auto& p : particles) {
            p = Particle(x, y, xvel, yvel, d(gen), d(gen));
        }
    }

    void step(double dt) {
        for (auto& p : particles) {
            p.step(dt);
        }
    }
};