// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <cstdlib>
#include <ctime>


double current_multisine = 0.0;
bool update_wave = true;
/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

double multisine(double time) {
    return (0.1 * 2 * M_PI * std::sin(2 * M_PI * 0.1 * time) +
           0.25 * 2 * M_PI * std::sin(2 * M_PI * 0.25 * time) +
           0.5 * 2 * M_PI * std::sin(2 * M_PI * 0.5 * time) +
           0.75 * 2 * M_PI * std::sin(2 * M_PI * 0.75 * time) +
           1 * 2 * M_PI * std::sin(2 * M_PI * 1 * time));    

}


double random_multisine(double time) {
    static double last_update_time = 0;
    static double multisine_period = 20.0;
    static double current_multisine_value = 0;
    static bool first_update = false;
    current_multisine_value = 0;

    static std::vector<std::function<double()>> waves;
    if (waves.empty()) {
        waves = {
            [&]() { return 0.25 * 2 * M_PI * std::sin(2 * M_PI * 0.25 * time); },
            [&]() { return 0.5 * 2 * M_PI * std::sin(2 * M_PI * 0.5 * time); },
            [&]() { return 0.75 * 2 * M_PI * std::sin(2 * M_PI * 0.75 * time); },
            [&]() { return 1 * 2 * M_PI * std::sin(2 * M_PI * 1 * time); }
        };
    }

    if (!first_update){
        std::random_shuffle(waves.begin(), waves.end());
        std::cout << "Time: " << time << " - First random multisine generated" << "Last update time:" << last_update_time << std::endl;

        for (int i = 0; i < 4; ++i) {
            current_multisine_value += waves[i]();
        }
        first_update = true;
    }
    else if (time - last_update_time >= multisine_period) // TODO fix if statement
    {

        std::random_shuffle(waves.begin(), waves.end());
        std::cout << "Time: " << time << " - Random multisine generated" << "Last update time:" << last_update_time << std::endl;
//        std::cout << "Waves: " << waves << std::endl;

        for (int i = 0; i < 4; ++i) {
            current_multisine_value += waves[i]();
        }

        last_update_time = time;
    } else {
//        std::cout << "Time: " << time << " - Old multisine used" << " Last update time:" << last_update_time << std::endl;
//        std::cout << "Waves: " << waves << std::endl;

        for (int i = 0; i < 4; ++i) {
            current_multisine_value += waves[i]();
        }
    }

//    std::cout << "Time: " << time << " Z value : " << current_multisine_value << std::endl;
    return current_multisine_value;
}


double rate_limit(double& last_value, double new_value, double max_rate) {
    double allowed_change = max_rate * 0.001;
    double change = new_value - last_value;
    if (std::abs(change) > allowed_change) {
        new_value = last_value + std::copysign(allowed_change, change);
    }
    last_value = new_value;
    return new_value;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    auto state = robot.readOnce();
    std::array<double, 7> q_goal = state.q;
	std::cout << "Intitial Joint State: " << std::endl;
    for (auto i : q_goal) {
        std::cout << i << std::endl;
    }

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);


    double running_time =300.0; // if executing mutlisine running_time must be factors of 20 to execute full cycles
    double time_max = 10.0;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
    double time = 0.0;
    double attentuation = 0.09;
    double last_velocity = 0.0;
    bool first_iteration = true;
    double max_velocity_change = 0.05;
    double period_time = 0.0;
    double counter = 0.0;
    std::cout << "Executing multisine" << std::endl;

//    std::cout << "New random multisine:" << counter << " " << time << std::endl;
//    current_multisine = random_multisine(time);

    robot.control([&](const franka::RobotState&, franka::Duration period) -> franka::CartesianVelocities {
        double cycle = std::floor(pow(1.0, (time - std::fmod(time, time_max)) / time_max));
        double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

		double v_z = v * -(attentuation * random_multisine(time));


		// Create the CartesianVelocities output
		franka::CartesianVelocities output = {{0.0, 0.0, v_z, 0.0, 0.0, 0.0}};

		time += period.toSec();

		if (time >= running_time) {
			std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
			return franka::MotionFinished(output);
		}
		return output;
   });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}