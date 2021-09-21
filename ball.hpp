#pragma once

#include <cmath>
#include <array>
#include <random>
#include "particle.hpp"
#include "robot.hpp"

// N num particles
// M num robots + every combination
template <int N, int M>
class Ball {
public:
    std::array<Particle, N> particles;
    std::array<std::array<std::array<bool, N>, M>, N> robot_particles_hit;

    Ball(double x, double y,
         double kickdir, double kickspeed,
         double dirsigma, double speedsigma) {
        
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<> d1{0, dirsigma};
        std::normal_distribution<> d2{0, dirsigma};

        for (auto& p : particles) {
            double speed = kickspeed + d1(gen);
            double theta = kickdir + d2(gen);
            p = Particle(x, y, speed*std::cos(theta), speed*std::sin(theta), 0, 0);
        }

        for (auto& mp : robot_particles_hit) {
            for (auto& r : mp) {
                for (auto& p : r) {
                    p = false;
                }
            }
        }
    }

    void check_collision(std::vector<Robot<N>> robots) {
        // For all my particles
        for (int i = 0; i < N; i++) {
            Particle& myP = particles[i];
            for (int r = 0; r < robots.size(); r++) {
                Robot<N>& robot = robots[r];
                for (int p = 0; p < robot.particles.size(); p++) {
                    Particle& theirP = robot.particles[p];
                    double deltaX = myP.x - theirP.x;
                    double deltaY = myP.y - theirP.y;
                    double dist = deltaX*deltaX + deltaY*deltaY;
                    double robotRadius = 0.01;
                    if (dist < robotRadius*robotRadius) {
                        robot_particles_hit[i][r][p] = true;
                    }
                }
            }
        }
    }

    void step(double dt) {
        for (auto& p : particles) {
            p.step(dt);
        }
    }

    double calcOut() {
        int perHit = 0;
        int totalNum = 0;

        for (auto& p : robot_particles_hit) {
            for (auto& r : p) {
                for (auto& h : r) {
                    if (h)
                        perHit++;
                    totalNum++;
                }
            }
        }

        return (double)perHit / (double)totalNum;
    }
};