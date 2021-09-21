#include <iostream>
#include <vector>
#include "ball.hpp"
#include "robot.hpp"

int main() {
    constexpr int numParticles = 1000;
    Ball<numParticles, 1> b(0, 0, 0, 1, 0.1, 0.01);

    std::vector<Robot<numParticles>> robots;
    robots.emplace_back(1, 0, 0, 0, 0.1);
    //robots.emplace_back(1, 4, 0, 0, 0.01);

    double t = 0;
    double dt = 0.001;
    
    while (t < 5) {
        b.check_collision(robots);

        b.step(dt);
        for (auto& r : robots) {
            r.step(dt);
        }
        t += dt;

        std::cout << t << " " << b.calcOut() << std::endl;
    }

    std::cout << b.calcOut() << std::endl;
    return 0;
}