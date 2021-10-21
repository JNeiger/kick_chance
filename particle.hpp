#pragma once

class Particle {
public:
    double x, y;
    double xvel, yvel;
    double vel_limit;
    double accel_limit;

    bool use_projected = false;
    double projected_x, projected_y;
    double projected_time_hit;

    Particle() {}

    Particle(double x, double y,
             double xvel, double yvel)
     : x(x), y(y),
       xvel(xvel), yvel(yvel) {}

    Particle(double x, double y,
             double xvel, double yvel,
             double vel_limit, double accel_limit,
             double projected_x, double projected_y,
             double projected_time_hit)
     : x(x), y(y),
       xvel(xvel), yvel(yvel),
       vel_limit(vel_limit), accel_limit(accel_limit), use_projected(true),
       projected_x(projected_x), projected_y(projected_y),
       projected_time_hit(projected_time_hit) {}

     void step(double t, double dt) {
         x += dt*xvel;
         y += dt*yvel;


        if (use_projected && t < projected_time_hit) {
            double delta_position_x = projected_x - x;
            double delta_position_y = projected_y - y;
            double time_to_projected = projected_time_hit - t;
            double average_vel_x = delta_position_x / time_to_projected;
            double average_vel_y = delta_position_y / time_to_projected;
            double delta_velocity_x = average_vel_x - xvel;
            double delta_velocity_y = average_vel_y - yvel;
            double delta_speed = sqrt(delta_velocity_x*delta_velocity_x + delta_velocity_y*delta_velocity_y);
            double time_to_reach_average = delta_speed / accel_limit;

            double delta_velocity_x_norm = delta_velocity_x / delta_speed;
            double delta_velocity_y_norm = delta_velocity_y / delta_speed;

            if (time_to_reach_average < time_to_projected) {
                xvel += dt*accel_limit*delta_velocity_x_norm;
                yvel += dt*accel_limit*delta_velocity_y_norm;
            } else {
                xvel -= dt*accel_limit*delta_velocity_x_norm;
                yvel -= dt*accel_limit*delta_velocity_y_norm;
            }

            if (xvel*xvel + yvel*yvel > vel_limit*vel_limit) {
                double cur_speed = sqrt(xvel*xvel + yvel*yvel);
                xvel = xvel/cur_speed*vel_limit;
                yvel = yvel/cur_speed*vel_limit;
            }
        }
     }
};