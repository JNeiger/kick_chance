#include <iostream>
#include <vector>
#include "ball.hpp"
#include "robot.hpp"
#include <fstream>

int main() {
    constexpr int numParticles = 1000;
    std::ofstream f("out.csv");

    for (double x = -3; x < 3; x += 0.1) {
        for (double y = -3; y < 3; y += 0.1) {
            Ball<numParticles, 2> b(0, 0, 0, 1, 0.1, 0.1);

            std::vector<Robot<numParticles>> robots;
            robots.emplace_back(1, 0, 0, 0, 0.1);
            robots.emplace_back(1, 1, 0, 0, 0.1);

            double t = 0;
            double dt = 0.001;
            
            while (t < 2) {
                b.check_line(1.9, -1, 2.1, 1);
                b.check_collision(robots);

                b.step(dt);
                for (auto& r : robots) {
                    r.step(dt);
                }
                t += dt;
            }

            std::cout << x << " " << y << " : " << b.calcOut() << std::endl;
            f << x << "," << y << "," << b.calcOut() << std::endl;
        }
    }
    f.close();
    return 0;
}