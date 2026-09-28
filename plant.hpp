#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

struct Plant {
    // add whatever state your model needs (velocity, motor-side angle, ...)
    // mimic how fast the can bus is actually going 
    double angle = 0.0;
    double velocity = 0.0;

    double K = 1.3; // gain of system (from step_test), mimics torque and efficiency 
    double T = 0.072; // time constant, mimics mechanical inertia
    double deadzone = 3.0; // in deg/s, mimics static friction threshold
    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    
    // map the actual physical behaviour of the motor
    // apply and account for physical conditions (lag, non-linearity, gain
    // static friction) that can alter the final position
    // calculate what the resultant encoder position should be 
    double step(double u_cmd, double dt) {
        // applying static friction threshold
        // if the command angle isn't greater than the non-linearity
        // the command angle isnt effective
        double u_effective = u_cmd;
        if (u_cmd > deadzone) {
            u_effective = u_cmd - deadzone;
        }
        else if (u_cmd < -deadzone) {
            u_effective = u_cmd + deadzone;
        }

        // using a first order lag model
        // update the internal velocity
        double acceleration = (K * u_effective - velocity) / T;
        velocity += acceleration * dt;

        // integrating internal velocity to find the raw position angle
        angle += velocity * dt;
        return std::round(angle / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() {
        angle = 0.0;
        velocity = 0.0;
    }
};