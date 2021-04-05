//
// Created by ziyi on 3/31/21.
//

#include "A1hardwareBridge.h"
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"
#include "Utilities/Utilities_print.h"

/*!
 * If an error occurs during initialization, before motors are enabled, print
 * error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void A1hardwareBridge::initError(const char *reason, bool printErrno) {
    printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

    if (printErrno) {
        printf("Error: %s\n", strerror(errno));
    }

    exit(-1);
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our
 * stack, this will make sure that we won't have a page fault when the stack
 * grows.  Also mlock's all pages associated with the current process, which
 * prevents the cheetah software from being swapped out.  If we do run out of
 * memory, the robot program will be killed by the OOM process killer (and
 * leaves a log) instead of just becoming unresponsive.
 */
void A1hardwareBridge::prefaultStack() {
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE];
    memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        initError(
                "mlockall failed.  This is likely because you didn't run robot as "
                "root.\n",
                true);
    }
}

/*!
 * All hardware initialization steps that are common between Cheetah 3，Mini Cheetah and A1
 */
void A1hardwareBridge::initCommon() {
    printf("[HardwareBridge] Init stack\n");
    prefaultStack();
    printf("[HardwareBridge] Init scheduler\n");
    setupScheduler();
    if (!_interfaceLCM.good()) {
        initError("_interfaceLCM failed to initialize\n", false);
    }

    printf("[HardwareBridge] Subscribe LCM\n");
    _interfaceLCM.subscribe("interface", &A1hardwareBridge::handleGamepadLCM, this);
    _interfaceLCM.subscribe("interface_request",
                            &A1hardwareBridge::handleControlParameter, this);

    printf("[HardwareBridge] Start interface LCM handler\n");
    _interfaceLcmThread = std::thread(&A1hardwareBridge::handleInterfaceLCM, this);
}

void A1hardwareBridge::handleInterfaceLCM() {
    while (!_interfaceLcmQuit) _interfaceLCM.handle();
}

/*!
 * LCM Handler for gamepad message
 */
void
A1hardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const gamepad_lcmt *msg) {
    (void)rbuf;
    (void)chan;
    _gamepadCommand.set(msg);
}

/*!
 * LCM Handler for control parameters
 */
void A1hardwareBridge::handleControlParameter(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                              const control_parameter_request_lcmt *msg) {
    (void)rbuf;
    (void)chan;
    if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) {
        // nothing to do!
        printf(
                "[HardwareBridge] Warning: the interface has run a ControlParameter "
                "iteration, but there is no new request!\n");
        // return;
    }

    // sanity check
    s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;
    if (nRequests != 1) {
        printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
               nRequests - 1);
    }

    switch (msg->requestKind) {
        case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME: {
            if(!_userControlParameters) {
                printf("[Warning] Got user param %s, but not using user parameters!\n",
                       (char*)msg->name);
            } else {
                std::string name((char*)msg->name);
                ControlParameter& param = _userControlParameters->collection.lookup(name);

                // type check
                if ((s8)param._kind != msg->parameterKind) {
                    throw std::runtime_error(
                            "type mismatch for parameter " + name + ", robot thinks it is " +
                            controlParameterValueKindToString(param._kind) +
                            " but received a command to set it to " +
                            controlParameterValueKindToString(
                                    (ControlParameterValueKind)msg->parameterKind));
                }

                // do the actual set
                ControlParameterValue v;
                memcpy(&v, msg->value, sizeof(v));
                param.set(v, (ControlParameterValueKind)msg->parameterKind);

                // respond:
                _parameter_response_lcmt.requestNumber =
                        msg->requestNumber;  // acknowledge that the set has happened
                _parameter_response_lcmt.parameterKind =
                        msg->parameterKind;  // just for debugging print statements
                memcpy(_parameter_response_lcmt.value, msg->value, 64);
                //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
                //for debugging print statements
                strcpy((char*)_parameter_response_lcmt.name,
                       name.c_str());  // just for debugging print statements
                _parameter_response_lcmt.requestKind = msg->requestKind;

                printf("[User Control Parameter] set %s to %s\n", name.c_str(),
                       controlParameterValueToString(
                               v, (ControlParameterValueKind)msg->parameterKind)
                               .c_str());
            }
        } break;

        case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: {
            std::string name((char*)msg->name);
            ControlParameter& param = _robotParams.collection.lookup(name);

            // type check
            if ((s8)param._kind != msg->parameterKind) {
                throw std::runtime_error(
                        "type mismatch for parameter " + name + ", robot thinks it is " +
                        controlParameterValueKindToString(param._kind) +
                        " but received a command to set it to " +
                        controlParameterValueKindToString(
                                (ControlParameterValueKind)msg->parameterKind));
            }

            // do the actual set
            ControlParameterValue v;
            memcpy(&v, msg->value, sizeof(v));
            param.set(v, (ControlParameterValueKind)msg->parameterKind);

            // respond:
            _parameter_response_lcmt.requestNumber =
                    msg->requestNumber;  // acknowledge that the set has happened
            _parameter_response_lcmt.parameterKind =
                    msg->parameterKind;  // just for debugging print statements
            memcpy(_parameter_response_lcmt.value, msg->value, 64);
            //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
            //for debugging print statements
            strcpy((char*)_parameter_response_lcmt.name,
                   name.c_str());  // just for debugging print statements
            _parameter_response_lcmt.requestKind = msg->requestKind;

            printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
                   controlParameterValueToString(
                           v, (ControlParameterValueKind)msg->parameterKind)
                           .c_str());

        } break;

        default: {
            throw std::runtime_error("parameter type unsupported");
        }
            break;
    }
    _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);
}

void A1hardwareBridge::publishVisualizationLCM() {

}

void A1hardwareBridge::LCMRecv()
{
    if(_lowLcm.lowCmdLCMHandler.isrunning){
        pthread_mutex_lock(&_lowLcm.lowCmdLCMHandler.countMut);
        _lowLcm.lowCmdLCMHandler.counter++;
        if(_lowLcm.lowCmdLCMHandler.counter > 10){
            printf("Error! LCM Time out.\n");
            exit(-1);              // can be commented out
        }
        pthread_mutex_unlock(&_lowLcm.lowCmdLCMHandler.countMut);
    }
    _lowLcm.Recv();
}

/*!
 * Configures the scheduler for real time priority
 */
void A1hardwareBridge::setupScheduler() {
    printf("[Init] Setup RT Scheduler...\n");
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
        initError("sched_setscheduler failed.\n", true);
    }
}

void A1hardwareBridge::initHardware() {
    _lowLcm.SubscribeState();
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
}

void A1hardwareBridge::run() {
    initCommon();
    initHardware();

    if(_load_parameters_from_file) {
        printf("[Hardware Bridge] Loading parameters from file...\n");

        try {
            _robotParams.initializeFromYamlFile(THIS_COM "config/a1-defaults.yaml");
        } catch(std::exception& e) {
            printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
            exit(1);
        }

        if(!_robotParams.isFullyInitialized()) {
            printf("Failed to initialize all robot parameters\n");
            exit(1);
        }

        printf("Loaded robot parameters\n");

        if(_userControlParameters) {
            try {
                _userControlParameters->initializeFromYamlFile(THIS_COM "config/a1-mit-ctrl-user-parameters.yaml");
            } catch(std::exception& e) {
                printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
                exit(1);
            }

            if(!_userControlParameters->isFullyInitialized()) {
                printf("Failed to initialize all user parameters\n");
                exit(1);
            }

            printf("Loaded user parameters\n");
        } else {
            printf("Did not load user parameters because there aren't any\n");
        }
    } else {
        printf("[Hardware Bridge] Loading parameters over LCM...\n");
        while (!_robotParams.isFullyInitialized()) {
            printf("[Hardware Bridge] Waiting for robot parameters...\n");
            usleep(1000000);
        }

        if(_userControlParameters) {
            while (!_userControlParameters->isFullyInitialized()) {
                printf("[Hardware Bridge] Waiting for user parameters...\n");
                usleep(1000000);
            }
        }
    }



    printf("[Hardware Bridge] Got all parameters, starting up!\n");

    _robotRunner =
            new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

    _robotRunner->driverCommand = &_gamepadCommand;
    _robotRunner->vectorNavData = &_vectorNavData;
    _robotRunner->a1Data = &_lowState;
    _robotRunner->a1Command = &_lowCmd;
    _robotRunner->robotType = RobotType::A1;
    _robotRunner->controlParameters = &_robotParams;
    _robotRunner->visualizationData = &_visualizationData;
    _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;

    _firstRun = false;

    // init control thread

    statusTask.start();

    // robot controller start; lcm msgs are published in RobotRunner
    _robotRunner->start();

    // receive the lcm msgs from lcm_server_low (can also be replaced by PeriodicMemberFunction; requires further evaluations)
    LoopFunc loop_lcm("LCM_Recv", 0.002, 3, boost::bind(&A1hardwareBridge::LCMRecv, this));

    // send command and receive data through unitree_sdk
    PeriodicMemberFunction<A1hardwareBridge> runUnitreeLCMTask(
            &taskManager, .002, "unitree-lcm",
            &A1hardwareBridge::runUnitreeLCM, this);
    runUnitreeLCMTask.start();

    // visualization start
    PeriodicMemberFunction<A1hardwareBridge> visualizationLCMTask(
            &taskManager, .0167, "lcm-vis",
            &A1hardwareBridge::publishVisualizationLCM, this);
    visualizationLCMTask.start();


    for (;;) {
        usleep(1000000);
        // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
    }



}

void A1hardwareBridge::runUnitreeLCM() {
    _lowLcm.Get(_lowState);
    _lowLcm.Send(_lowCmd);
    getIMU();
}

void A1hardwareBridge::getIMU() {
    _vectorNavData.accelerometer = Eigen::Vector3f(_lowState.imu.accelerometer);
    // TODO: double check the order for quat of _vectorNavData
    _vectorNavData.quat[0] = _lowState.imu.quaternion[1];
    _vectorNavData.quat[1] = _lowState.imu.quaternion[2];
    _vectorNavData.quat[2] = _lowState.imu.quaternion[3];
    _vectorNavData.quat[3] = _lowState.imu.quaternion[0];
    _vectorNavData.gyro = Eigen::Vector3f(_lowState.imu.gyroscope);
}


