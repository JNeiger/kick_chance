#pragma once

class Particle {
public:
    double x, y;
    double xvel, yvel;
    double xaccel, yaccel;

    Particle() {}

    Particle(double x, double y,
             double xvel, double yvel,
             double xaccel, double yaccel)
     : x(x), y(y), xvel(xvel), yvel(yvel), xaccel(xaccel), yaccel(yaccel) {}

     void step(double dt) {
         x += dt*xvel;
         y += dt*yvel;
         xvel += dt*xaccel;
         yvel += dt*yaccel;
     }
};