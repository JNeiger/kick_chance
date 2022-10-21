#pragma once

#include <cmath>
#include <array>
#include <random>
#include "particle.hpp"
#include "ball.hpp"

struct RobotInit {
    double x = 0, y = 0;
    double xvel = 0, yvel = 0;
    double vel_limit_mean = 0, vel_limit_stddev = 0;
    double accel_limit_mean = 0, accel_limit_stddev = 0;
};

// N num particles
template <int N>
class Robot {
public:
    std::array<Particle, N> particles;

    Robot(RobotInit init,
         const std::array<Particle, N>& ball_particles) {
        
        std::random_device rd{};
        std::linear_congruential_engine<unsigned long, 1664525, 1013904223, static_cast<unsigned long>(pow(2, 32))> gen;
        gen.seed(0);
        std::normal_distribution<> vel_limit_distribution{init.vel_limit_mean, init.vel_limit_stddev};
        std::normal_distribution<> accel_limit_distribution{init.accel_limit_mean, init.accel_limit_stddev};

        for (size_t ball_idx = 0; ball_idx < N; ball_idx++) {
            const Particle& ball_particle = ball_particles.at(ball_idx);

            // projection = ball_vel.norm() . (robot - ball_pos) * ball_vel.normalized()
            double ball_to_robot_x = init.x - ball_particle.x;
            double ball_to_robot_y = init.y - ball_particle.y;
            double ball_vel_x_norm = ball_particle.xvel / sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);
            double ball_vel_y_norm = ball_particle.yvel / sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);

            double projected_dot = ball_vel_x_norm*ball_to_robot_x + ball_vel_y_norm*ball_to_robot_y;
            double projected_x = projected_dot * ball_vel_x_norm + ball_particle.x;
            double projected_y = projected_dot * ball_vel_y_norm + ball_particle.y;
            double dist_ball_to_projected = projected_dot;
            double ball_speed = sqrt(ball_particle.xvel*ball_particle.xvel + ball_particle.yvel*ball_particle.yvel);
            double time_ball_to_projected = dist_ball_to_projected / ball_speed; // can be negative

            double vel_limit = vel_limit_distribution(gen);
            double accel_limit = accel_limit_distribution(gen);

            particles.at(ball_idx) =
                Particle(init.x, init.y,
                        init.xvel, init.yvel,
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