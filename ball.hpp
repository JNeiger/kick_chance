#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>
#include <array>
#include <unordered_set>
#include <random>
#include "particle.hpp"
#include "robot.hpp"

// N num particles
// M num robots
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
        double line_length_sq = (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1);
        
        double delta_21_x = x2 - x1;
        double delta_21_y = y2 - y1;
        for (int i = 0; i < particles.size(); i++) {
            if (valid_pass[i]) {
                continue;
            }
            auto& p = particles[i];
            double delta_p1_x = p.x - x1;
            double delta_p1_y = p.y - y1;
            double dot = delta_p1_x * delta_21_x + delta_p1_y * delta_21_y;
            double project_scale = dot / line_length_sq;
            project_scale = std::min(std::max(project_scale, 0.0), 1.0);

            double project_x = x1 + project_scale * delta_21_x;
            double project_y = y1 + project_scale * delta_21_y;

            double delta_pproject_x = p.x - project_x;
            double delta_pproject_y = p.y - project_y;

            // Within X of the end segment
            if (delta_pproject_x*delta_pproject_x + delta_pproject_y*delta_pproject_y < 0.01) {
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
        std::cout << "Calculating output" << std::endl;
        std::vector<std::vector<int>> setwiseAndRobotIDs;

        for (int numInSet = 2; numInSet <= M; numInSet++) {
            std::vector<int> itemsInSet;

            // Init to first
            for (int i = 0; i < numInSet; i++) {
                itemsInSet.push_back(i);
            }
            setwiseAndRobotIDs.push_back(itemsInSet);

            bool stillSetsToAdd = true;
            while (stillSetsToAdd) {
                int indexToInc = numInSet - 1;
                bool validInsert = false;
                while (!validInsert) {
                    // If we need to increment a bigger digit than avaliable
                    // we went too far
                    if (indexToInc < 0) {
                        stillSetsToAdd = false;
                        break;
                    }

                    itemsInSet.at(indexToInc)++;

                    // Make sure we can increment this digit
                    if (itemsInSet.at(indexToInc) >= M) {
                        indexToInc--;
                        continue;
                    }

                    // If digit is not at end, then we just incremented a higher digit
                    // Fix digits smaller so we don't repeat
                    for (int i = indexToInc + 1; i < numInSet; i++) {
                        itemsInSet.at(i) = itemsInSet.at(indexToInc) + i - indexToInc;
                    }

                    bool validSet = true;
                    for (auto& i : itemsInSet) {
                        if (i >= M)
                            validSet = false;
                    }

                    validInsert = validSet;
                    if (validSet)
                        setwiseAndRobotIDs.push_back(itemsInSet);
                }
            }
        }

        double overall_probability_hit = 0.0;
        for (auto& ball_particle_id : robot_particles_hit) {
            double ball_particle_probability_hit = 0.0;
            // P(hitting robot R) = % of robot R's particle hit
            for (auto& r : ball_particle_id) {
                ball_particle_probability_hit += (double)r.size() / N;
            }

            // P(hitting robot R1 and R2 etc) = P(hitting robot R1) * P(hitting robot R2)
            for (auto& set : setwiseAndRobotIDs) {
                double overall_p = 1;
                for (auto& id : set) {
                    overall_p *= (double)ball_particle_id[id].size() / N;
                }
                ball_particle_probability_hit -= overall_p;
            }

            // P(hit|particle)*P(particle)
            overall_probability_hit += ball_particle_probability_hit * (1.0 / N);
        }
        
        double overall_probability_success = 0.0;
        for (auto ball_particle_success : valid_pass) {
            if (ball_particle_success) {
                overall_probability_success += 1.0 * (1.0 / N);
            }
        }

        std::cout << overall_probability_hit << " hit " << overall_probability_success << "success" << std::endl;
        return overall_probability_success * (1 - overall_probability_hit);
    }
};