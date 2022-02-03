#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>
#include <array>
#include <unordered_set>
#include <random>
#include "particle.hpp"
#include "robot.hpp"

struct BallInit {
    double x = 0, y = 0;
    double angle = 0, speed = 0;
    double angle_stddev = 0, speed_stddev = 0;
};

// N num particles
// M num robots
template <int N, int M>
class Ball {
public:
    std::array<Particle, N> particles;
    std::array<bool, N> valid_pass{};
    std::array<std::array<bool, M>, N> robot_particles_hit{}; // my particle, robot id

    Ball(BallInit init) {
        
        std::random_device rd{};
        std::linear_congruential_engine<unsigned long, 1664525, 1013904223, static_cast<unsigned long>(pow(2, 32))> gen;
        gen.seed(0);
        std::normal_distribution<> direction_distibution{0, init.angle_stddev};
        std::normal_distribution<> speed_distribution{0, init.speed_stddev};

        for (int i = 0; i < N; i++) {
            double speed = init.speed + direction_distibution(gen);
            double theta = init.angle + speed_distribution(gen);
            particles[i] = Particle(init.x, init.y, speed*std::cos(theta), speed*std::sin(theta));
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
            // TODO Just use angle to figure out if it will intercept the line correctly
            if (delta_pproject_x*delta_pproject_x + delta_pproject_y*delta_pproject_y < 0.1) {
                valid_pass[i] = true;
            }
        }
    }
    
    void check_collision(std::vector<Robot<N>>& robots) {
        // For all my particles
        for (int i = 0; i < particles.size(); i++) {
            Particle& myP = particles[i];

            for (int r = 0; r < robots.size(); r++) {
                Robot<N>& robot = robots[r];
                Particle& theirP = robot.particles.at(i);

                if (robot_particles_hit[i][r] == true) {
                    continue;
                }

                double deltaX = myP.x - theirP.x;
                double deltaY = myP.y - theirP.y;
                double dist = deltaX*deltaX + deltaY*deltaY;
                double robotRadius = 0.05;
                if (dist < robotRadius*robotRadius &&
                    robot_particles_hit[i][r] == false) {
                    robot_particles_hit[i][r] = true;
                }
            }
        }
    }

    void step(double t, double dt) {
        for (auto& p : particles) {
            p.step(t, dt);
        }
    }

    double calcOut() {
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

        
        // Pre calculate num particles hit
        std::array<int, M> particles_hit_per_robot{};
        for (int robot_id = 0; robot_id < M; robot_id++) {
            for (int i = 0; i < N; i++) {
                if (robot_particles_hit[i][robot_id]) {
                    ++particles_hit_per_robot[robot_id];
                }
            }
        }

        // Get prob
        std::array<double, M> prob_hit_per_robot;
        for (int robot_id = 0; robot_id < M; robot_id++) {
            prob_hit_per_robot[robot_id] = (double)particles_hit_per_robot[robot_id] / N;
        }

        // Sum of individual probs
        double overall_probability_hit = 0.0;
        for (const auto& prob : prob_hit_per_robot) {
            overall_probability_hit += prob;
        }

        // Minus setwiseAnd
        for (auto& set : setwiseAndRobotIDs) {
            double overall_p = 1;
            for (auto& id : set) {
                overall_p *= prob_hit_per_robot[id];
            }
            overall_probability_hit -= overall_p;
        }

        double overall_probability_success = 0.0;
        for (auto ball_particle_success : valid_pass) {
            if (ball_particle_success) {
                overall_probability_success += (1.0 / N);
            }
        }

        return overall_probability_success * (1 - overall_probability_hit);
    }
};