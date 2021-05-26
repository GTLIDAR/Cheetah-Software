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

#ifdef linux

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

#include <string>
#include <lcm-cpp.hpp>

#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "custom_cmd_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"
#include <unitree_legged_sdk/unitree_legged_sdk.h>

using namespace UNITREE_LEGGED_SDK;

/*
 * Interface between controller and A1 through @RobotInterface
 */
class A1hardwareBridge {
public:
    A1hardwareBridge(RobotController *robot_ctrl, uint8_t level, bool load_parameters_from_file)
            : statusTask(&taskManager, 0.5f),
              _interfaceLCM(getLcmUrl(255)),
              _visualizationLCM(getLcmUrl(255)),
              _highCmdLCM(getLcmUrl(255)),
              _lowLcm(level) {
        _controller = robot_ctrl;
        _userControlParameters = robot_ctrl->getUserControlParameters();
        _load_parameters_from_file = load_parameters_from_file;
    }
    ~A1hardwareBridge(){ delete _robotRunner;}
    void prefaultStack();
    void setupScheduler();
    void initError(const char* reason, bool printErrno = false);
    void initCommon();
    void initHardware();
    void run();
    void runUnitreeLCM();
    void getIMU();
    void handleInterfaceLCM();
    void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                          const gamepad_lcmt* msg);
    void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                                const std::string& chan,
                                const control_parameter_request_lcmt* msg);
    void handleHighCmd(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const custom_cmd_lcmt *msg);
    void LCMRecv();

    void publishVisualizationLCM();

protected:
    PeriodicTaskManager taskManager;
    PrintTaskStatus statusTask;
    GamepadCommand _gamepadCommand;
    VisualizationData _visualizationData;
    CheetahVisualization _mainCheetahVisualization;
    VectorNavData _vectorNavData;
    lcm::LCM _interfaceLCM;
    lcm::LCM _visualizationLCM;
    lcm::LCM _highCmdLCM;
    control_parameter_respones_lcmt _parameter_response_lcmt{};
    LowCmd _lowCmd{};
    LowState _lowState{};
    UNITREE_LEGGED_SDK::LCM _lowLcm;

    bool _firstRun = true;
    RobotRunner* _robotRunner = nullptr;
    RobotControlParameters _robotParams;
    u64 _iterations = 0;
    std::thread _interfaceLcmThread;
    volatile bool _interfaceLcmQuit = false;
    RobotController* _controller = nullptr;
    ControlParameters* _userControlParameters = nullptr;

    int _port{};
    bool _load_parameters_from_file;



};
#endif // END of #ifdef linux
#endif //CHEETAH_SOFTWARE_A1HARDWAREBRIDGE_H
