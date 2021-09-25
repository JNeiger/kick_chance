#include <iostream>
#include <vector>
#include "ball.hpp"
#include "robot.hpp"
#include <fstream>
#include <thread>
#include <mutex>

class CSVOut {
public:
    CSVOut(std::string fileName) : f(fileName) {}
    ~CSVOut() {
        f.close();
    }

    void write(double x, double y, double p) {
        std::lock_guard<std::mutex> lock(fileMutex);
        std::cout << x << " " << y << std::endl;
        f << x << "," << y << "," << p << std::endl;
    }

private:
    std::ofstream f;
    std::mutex fileMutex;
};

void doFunc(double xlow, double xhigh, double xstep,
            double ylow, double yhigh, double ystep,
            CSVOut& csvout) {
    constexpr int numParticles = 1000;
    std::cout << "Starting run " << std::endl;
    for (double x = xlow; x < xhigh; x += xstep) {
        for (double y = ylow; y < yhigh; y += ystep) {
            Ball<numParticles, 2> b(x, y,
                                    0, 1,
                                    0.01, 0.01);

            std::vector<Robot<numParticles>> robots;
            robots.emplace_back(1, -1,
                                0.1, 0,
                                0.1);
            robots.emplace_back(1, 1,
                                0, 0.1,
                                0.1);

            double t = 0;
            double dt = 0.001;

            while (t < 4) {
                b.check_line(1.9, -1, 2.1, 1);
                b.check_collision(robots);

                b.step(dt);
                for (auto& r : robots) {
                    r.step(dt);
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

    double xmin = -0.5;
    double xmax = 1.5;
    double xstep = 0.1;
    double ymin = -1.5;
    double ymax = 1.5;
    double ystep = 0.01;

    double xthread_step = xstep * std::round(std::round((xmax - xmin) / xstep) / numThreads);
    double xlow = xmin;
    double xhigh = xmin + xthread_step;
    std::cout << xlow << " " << xhigh << std::endl;
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
