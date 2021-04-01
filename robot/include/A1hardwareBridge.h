//
// Created by zzhou387 on 3/29/21.
//
/*!
 * @file A1hardwareBridge.h
 * @brief Interface between robot code and A1 hardware through unitree_sdk
 *
 * This class will send and receive lcm msgs to lcm_server_low which should be running in another terminal
 */

#ifndef CHEETAH_SOFTWARE_A1HARDWAREBRIDGE_H
#define CHEETAH_SOFTWARE_A1HARDWAREBRIDGE_H

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include <unitree_legged_sdk/unitree_legged_sdk.h>

using namespace UNITREE_LEGGED_SDK;

/*
 * Interface between controller and A1 through @RobotInterface
 */
class A1hardwareBridge {
public:
    A1hardwareBridge()

};

#endif //CHEETAH_SOFTWARE_A1HARDWAREBRIDGE_H
