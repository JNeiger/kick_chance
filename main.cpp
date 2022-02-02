#include <iostream>
#include <vector>
#include "ball.hpp"
#include "robot.hpp"
#include <fstream>
#include <thread>
#include <mutex>

class CSVOut {
public:
    CSVOut(std::string fileName) : f(fileName) {
        f << "x,y,z\n";
    }
    ~CSVOut() {
        f.flush();
        f.close();
    }

    void write(double x, double y, double p) {
        std::lock_guard<std::mutex> lock(fileMutex);
        std::cout << x << " " << y << " " << p << std::endl;
        f << x << "," << y << "," << p << "\n";
    }

private:
    std::ofstream f;
    std::mutex fileMutex;
};

//
// For each ball track
// have each robot have their own particle filter of accel/vel limits distributions
// they try to optimally intercept that specific track
//
// for each ball track
// count the number of interceptions
//  for each robot  - pairwise
//

void doFunc(double xlow, double xhigh, double xstep,
            double ylow, double yhigh, double ystep,
            CSVOut& csvout) {
    constexpr int numParticles = 90;
    for (double x = xlow; x <= xhigh; x += xstep) {
        for (double y = ylow; y <= yhigh; y += ystep) {
            double dx = 1.5 - x;
            double dy = 0 -  y;
            double angle = atan2(dy, dx);
            Ball<numParticles, 1> b(x, y,
                                    angle, 3,
                                    .1, 0.1);

            std::vector<Robot<numParticles>> robots;
            robots.emplace_back(0, 0,
                                0, -0.1,
                                1, 0.1,
                                1, 0.1,
                                b.particles);
            robots.emplace_back(0, 1,
                                0, 0.1,
                                1, 0.1,
                                1, 0.1,
                                b.particles);

            double t = 0;
            double dt = 0.01;

            while (t < 6) {
                b.check_line(1.5, -1, 1.5, 1);
                b.check_collision(robots);

                b.step(t, dt);
                for (auto& r : robots) {
                    r.step(t, dt);
                }
                t += dt;
            }

            csvout.write(x, y, b.calcOut());
        }
    }
}

int main() {
    CSVOut csvout("out.csv");
    int numThreads = 1;

    double xmin = -3;
    double xmax = 1.4;
    double xstep = 0.1;
    double ymin = -3;
    double ymax = 3;
    double ystep = 0.1;

    double xthread_step = xstep * std::round(std::round((xmax - xmin) / xstep) / numThreads);
    double xlow = xmin;
    double xhigh = xmin + xthread_step;
    std::vector<std::thread> threads;
    for (int i = 0; i < numThreads; i++) {
        threads.emplace_back(std::thread(doFunc, xlow, xhigh, xstep,
                                     ymin, ymax, ystep,
                                     std::ref(csvout)));
        xlow += xthread_step;
        xhigh += xthread_step;
    }
    for (auto& t : threads) {
        t.join();
    }
    return 0;
}
