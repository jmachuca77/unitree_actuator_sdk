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

    // Set motor to BRAKE mode before reading initial position
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

    // Ask the user for the desired speed
    float user_speed;
    std::cout << "Enter the desired speed in degrees per second: ";
    std::cin >> user_speed;

    // Step 3: Calculate the target position
    float target_position = initial_position + desired_position;

    // Step 4: Move to the target position incrementally with controlled speed
    float current_position = initial_position;
    float max_speed = user_speed; // Use the user-provided speed
    float min_speed = 5.0;  // Minimum speed limit to avoid stopping too early
    float ramp_distance = 10.0; // Distance within which to start slowing down
    float time_interval = 0.001; // Time interval between steps in seconds (20ms)

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

        // Calculate the incremental step based on the desired speed
        float increment = (speed * time_interval) * (distance_to_target > 0 ? 1 : -1);

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

        // Print current position relative to the initial position
        float relative_position = current_position - initial_position;
        std::cout << "Current Position Relative to Initial: " << relative_position << " degrees\n";
        usleep(time_interval * 1000000); // Sleep for the time interval in microseconds
    }

    std::cout << "Reached desired position.\n";
    usleep(2000000); // Sleep for the time interval in microseconds
    // Set motor to BRAKE mode after reaching the final position
    cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
    serial.sendRecv(&cmd, &data);

    return 0;
}
