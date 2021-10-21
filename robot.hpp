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
    std::array<std::array<Particle, N>, N> particles; // Which ball particle
                                                      // Then which robot particle

    Robot(double x, double y,
         double xvel, double yvel,
         double vel_limit_mean, double vel_limit_sigma,
         double accel_limit_mean, double accel_limit_sigma,
         const std::vector<Particle>& ball_particles) {
        
        std::random_device rd{};
        std::mt19937 gen{0};
        std::normal_distribution<> vel_dist{vel_limit_mean, vel_limit_sigma};
        std::normal_distribution<> accel_dist{accel_limit_mean, accel_limit_sigma};

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

            // Create a robot particle with some random limit
            for (size_t robot_particle_idx = 0; robot_particle_idx < N; robot_particle_idx++) {
                double vel_limit = vel_dist(gen);
                double accel_limit = accel_dist(gen);

                particles.at(ball_idx).at(robot_particle_idx) =
                    Particle(x, y,
                             xvel, yvel,
                             vel_limit, accel_limit,
                             projected_x, projected_y,
                             time_ball_to_projected);
            }
        }
    }

    void step(double t, double dt) {
        for (auto& ball_p_list : particles) {
            for (auto& p : ball_p_list) {
                p.step(t, dt);
            }
        }
    }
};