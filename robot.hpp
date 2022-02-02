#pragma once

#include <cmath>
#include <array>
#include <random>
#include "particle.hpp"
#include "ball.hpp"

// N num particles
template <int N>
class Robot {
public:
    std::array<Particle, N> particles;

    Robot(double x, double y,
         double xvel, double yvel,
         double vel_limit_mean, double vel_limit_sigma,
         double accel_limit_mean, double accel_limit_sigma,
         const std::array<Particle, N>& ball_particles) {
        
        std::random_device rd{};
        std::linear_congruential_engine<unsigned long, 1664525, 1013904223, static_cast<unsigned long>(pow(2, 32))> gen;
        gen.seed(0);
        std::normal_distribution<> vel_limit_distribution{vel_limit_mean, vel_limit_sigma};
        std::normal_distribution<> accel_limit_distribution{accel_limit_mean, accel_limit_sigma};

        for (size_t ball_idx = 0; ball_idx < N; ball_idx++) {
            const Particle& ball_particle = ball_particles.at(ball_idx);

            // projection = ball_vel.norm() . (robot - ball_pos) * ball_vel.normalized()
            double ball_to_robot_x = x - ball_particle.x;
            double ball_to_robot_y = y - ball_particle.y;
            double ball_vel_x_norm = ball_particle.xvel / sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);
            double ball_vel_y_norm = ball_particle.yvel / sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);

            double projected_dot = ball_vel_x_norm*ball_to_robot_x + ball_vel_y_norm*ball_to_robot_y;
            double projected_x = projected_dot * ball_vel_x_norm + ball_particle.x;
            double projected_y = projected_dot * ball_vel_y_norm + ball_particle.y;
            double dist_ball_to_projected = projected_dot;
            double ball_speed = sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);
            double time_ball_to_projected = dist_ball_to_projected / ball_speed; // can be negative

            double robot_to_projected_x = projected_x - x;
            double robot_to_projected_y = projected_y - y;

            double vel_limit = vel_limit_distribution(gen);
            double accel_limit = accel_limit_distribution(gen);

            particles.at(ball_idx) =
                Particle(x, y,
                        xvel, yvel,
                        vel_limit, accel_limit,
                        projected_x, projected_y,
                        time_ball_to_projected);
        }
    }

    void step(double t, double dt) {
        for (auto& p : particles) {
            p.step(t, dt);
        }
    }
};