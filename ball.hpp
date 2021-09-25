#pragma once

#include <algorithm>
#include <cmath>
#include <array>
#include <unordered_set>
#include <random>
#include "particle.hpp"
#include "robot.hpp"

// N num particles
// M num robots + every combination
template <int N, int M>
class Ball {
public:
    std::vector<Particle> particles;
    std::vector<bool> valid_pass;
    std::vector<std::vector<std::unordered_set<int>>> robot_particles_hit; // my particle, robot id, vs their particle

    Ball(double x, double y,
         double kickdir, double kickspeed,
         double dirsigma, double speedsigma) {
        
        std::random_device rd{};
        std::mt19937 gen{0};
        std::normal_distribution<> d1{0, dirsigma};
        std::normal_distribution<> d2{0, dirsigma};

        particles.reserve(N);
        for (int i = 0; i < N; i++) {
            double speed = kickspeed + d1(gen);
            double theta = kickdir + d2(gen);
            particles.emplace_back(x, y, speed*std::cos(theta), speed*std::sin(theta), 0, 0);
        }

        valid_pass.reserve(N);
        for (int i = 0; i < N; i++) {
            valid_pass.emplace_back(false);
        }

        robot_particles_hit.reserve(N);
        for (int i = 0; i < N; i++) {
            robot_particles_hit.push_back({});
            robot_particles_hit[i].reserve(M);
            for (int j = 0; j < M; j++) {
                robot_particles_hit[i].push_back({});
            }
        }
    }

    void check_line(double x1, double y1, double x2, double y2) {
        for (int i = 0; i < particles.size(); i++) {
            if (particles[i].x < std::max(x1, x2) &&
                particles[i].x > std::min(x1, x2) &&
                particles[i].y < std::max(y1, y2) &&
                particles[i].y > std::min(y1, y2)) {
                valid_pass[i] = true;
            }
        }
    }
    
    void check_collision(std::vector<Robot<N>> robots) {
        // For all my particles
        for (int i = 0; i < particles.size(); i++) {
            Particle& myP = particles[i];

            for (int r = 0; r < robots.size(); r++) {
                Robot<N>& robot = robots[r];
                for (int p = 0; p < robot.particles.size(); p++) {
                    Particle& theirP = robot.particles[p];
                    double deltaX = myP.x - theirP.x;
                    double deltaY = myP.y - theirP.y;
                    double dist = deltaX*deltaX + deltaY*deltaY;
                    double robotRadius = 0.01;
                    if (dist < robotRadius*robotRadius &&
                        robot_particles_hit[i][r].find(p) == robot_particles_hit[i][r].end()) {
                        robot_particles_hit[i][r].insert(p);
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
        double percent = 0.0;

        for (int i = 0; i < robot_particles_hit.size(); i++) {
            auto& p = robot_particles_hit[i];

            double numHitsForBallParticle = 0;
            for (auto& r : p) {
                numHitsForBallParticle += r.size();
            }

            for (auto& particle : p.front()) {
                for (int other = 1; other < robot_particles_hit[i].size(); other++) {
                    if (robot_particles_hit[i][other].find(particle) != robot_particles_hit[i][other].end()) {
                        numHitsForBallParticle--;
                    }
                }
            }
            percent += numHitsForBallParticle / N;
        }

        return percent / N;
    }
};