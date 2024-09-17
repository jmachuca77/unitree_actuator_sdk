#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <math.h>
#include <iostream>
#define PI 3.1415926

int main() {
    SerialPort  serial("/dev/ttyUSB0");
    MotorCmd    cmd;
    MotorData   data;

    float output_kp = 25;
    float output_kd = 0.6;
    float rotor_kp = 0;
    float rotor_kd = 0;
    float gear_ratio = queryGearRatio(MotorType::A1);

    rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
    rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;

    cmd.motorType = MotorType::A1;
    data.motorType = MotorType::A1;
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
    cmd.id    = 0;
    cmd.kp    = 0.0;
    cmd.kd    = 0.0;
    cmd.q     = 0.0;
    cmd.dq    = 0.0;
    cmd.tau   = 0.0;
    serial.sendRecv(&cmd, &data);

    // Step 1: Save the initial motor position
    float initial_position = data.q / gear_ratio * (180 / PI);
    std::cout << "Initial motor position set to " << initial_position << " degrees as the reference.\n";

    // Step 2: Ask the user for the desired position
    float desired_position;
    std::cout << "Enter the desired position in degrees relative to the initial position: ";
    std::cin >> desired_position;

    // Step 3: Calculate the target position
    float target_position = initial_position + desired_position;

    // Step 4: Move to the target position incrementally
    float current_position = initial_position;
    float step_size = 1.0; // Define the step size for increments
    float max_speed = 100.0; // Maximum speed limit (degrees per second)
    float min_speed = 5.0;  // Minimum speed limit to avoid stopping too early
    float ramp_distance = 10.0; // Distance within which to start slowing down

    while (fabs(target_position - current_position) > 0.1) { // Continue until close to the target
        // Determine the distance to the target
        float distance_to_target = target_position - current_position;

        // Adjust the speed based on the distance to the target
        float speed = max_speed;
        if (fabs(distance_to_target) < ramp_distance) {
            // Ramp-down speed
            speed = max_speed * (fabs(distance_to_target) / ramp_distance);
            if (speed < min_speed) speed = min_speed;
        }

        // Calculate the incremental step
        float increment = (distance_to_target > 0 ? step_size : -step_size);
        increment *= (speed / max_speed); // Scale step by speed

        // Update current position
        current_position += increment;

        // Convert the current position to rotor angle
        float rotor_angle_d = (current_position * (PI / 180)) * gear_ratio;

        // Send the command to the motor
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id    = 0;
        cmd.kp    = rotor_kp;
        cmd.kd    = rotor_kd;
        cmd.q     = rotor_angle_d;
        cmd.dq    = 0.0;
        cmd.tau   = 0.0;
        serial.sendRecv(&cmd, &data);

        std::cout << "Current Position: " << current_position << " degrees\n";
        usleep(1000); // Sleep for 20 ms between steps
    }

    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
    serial.sendRecv(&cmd, &data);
    std::cout << "Reached desired position.\n";

    return 0;
}
