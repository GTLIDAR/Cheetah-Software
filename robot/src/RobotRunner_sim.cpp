//
// Created by ziyi on 4/17/21.
//
/*!
 * @file RobotRunner_sim.cpp
 * @brief Common framework for running robot controllers.
 * This code is a common interface between control code and hardware/simulation
 * for mini cheetah, cheetah 3 and A1
 * for simulation only since the simulator for A1 is still using SPIBoard related data types
 */

#include <unistd.h>

#include "RobotRunner_sim.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/A1.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler.hpp"
#include "Utilities/Timer.h"
#include "Controllers/PositionVelocityEstimator.h"
//#include "rt/rt_interface_lcm.h"

RobotRunner_sim::RobotRunner_sim(RobotController* robot_ctrl,
                         PeriodicTaskManager* manager,
                         float period, std::string name):
        PeriodicTask(manager, period, name),
        _lcm(getLcmUrl(255)) {

    _robot_ctrl = robot_ctrl;
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner_sim::init() {
    printf("[RobotRunner_sim] initialize\n");

    // Build the appropriate Quadruped object
    if (robotType == RobotType::MINI_CHEETAH) {
        _quadruped = buildMiniCheetah<float>();
    } else if (robotType == RobotType::CHEETAH_3) {
        _quadruped = buildCheetah3<float>();
    } else if (robotType == RobotType::A1) {
        _quadruped = buildA1<float>();
    } else {
        assert(false);
    }

    // Initialize the model and robot data
    _model = _quadruped.buildModel();
    _jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);

    // Always initialize the leg controller and state entimator
    _legController = new LegController<float>(_quadruped);
    _stateEstimator = new StateEstimatorContainer<float>(
            cheaterState, vectorNavData, _legController->datas,
            &_stateEstimate, controlParameters);
    initializeStateEstimator(false);

    memset(&rc_control, 0, sizeof(rc_control_settings));
    // Initialize the DesiredStateCommand object
    _desiredStateCommand =
            new DesiredStateCommand<float>(driverCommand,
                                           &rc_control,
                                           HighlevelCmd,
                                           controlParameters,
                                           &_stateEstimate,
                                           controlParameters->controller_dt);

    // Controller initializations
    _robot_ctrl->_model = &_model;
    _robot_ctrl->_quadruped = &_quadruped;
    _robot_ctrl->_legController = _legController;
    _robot_ctrl->_stateEstimator = _stateEstimator;
    _robot_ctrl->_stateEstimate = &_stateEstimate;
    _robot_ctrl->_visualizationData= visualizationData;
    _robot_ctrl->_robotType = robotType;
    _robot_ctrl->_driverCommand = driverCommand;
    _robot_ctrl->_HighCmd = HighlevelCmd;
    _robot_ctrl->_controlParameters = controlParameters;
    _robot_ctrl->_desiredStateCommand = _desiredStateCommand;

    _robot_ctrl->initializeController();

}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner_sim::run() {
    // Run the state estimator step
    //_stateEstimator->run(cheetahMainVisualization);
    _stateEstimator->run();
    //cheetahMainVisualization->p = _stateEstimate.position;
    visualizationData->clear();

    // Update the data from the robot
    setupStep();

    static int count_ini(0);
    ++count_ini;
    if (count_ini < 10) {
        _legController->setEnabled(false);
    } else if (20 < count_ini && count_ini < 30) {
        _legController->setEnabled(false);
    } else if (40 < count_ini && count_ini < 50) {
        _legController->setEnabled(false);
    } else {
        _legController->setEnabled(true);

        if( (rc_control.mode == 0) && controlParameters->use_rc ) {
            if(count_ini%1000 ==0)   printf("ESTOP!\n");
            for (int leg = 0; leg < 4; leg++) {
                _legController->commands[leg].zero();
            }
            _robot_ctrl->Estop();
        } else {
            // Controller
            if(robotType == RobotType::MINI_CHEETAH || robotType == RobotType::CHEETAH_3) {
                if (!_jpos_initializer->IsInitialized(_legController)) {
                    Mat3<float> kpMat;
                    Mat3<float> kdMat;
                    // Update the jpos feedback gains
                    if (robotType == RobotType::MINI_CHEETAH) {
                        kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
                        kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
                    } else if (robotType == RobotType::A1) {
                        kpMat << 5, 0, 0, 0, 5, 0, 0, 0, 5;
                        kdMat << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
                    } else if (robotType == RobotType::CHEETAH_3) {
                        kpMat << 50, 0, 0, 0, 50, 0, 0, 0, 50;
                        kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
                    } else {
                        assert(false);
                    }

                    for (int leg = 0; leg < 4; leg++) {
                        _legController->commands[leg].kpJoint = kpMat;
                        _legController->commands[leg].kdJoint = kdMat;
                    }
                } else {
                    // Run Control
                    _robot_ctrl->runController();
                    cheetahMainVisualization->p = _stateEstimate.position;

                    // Update Visualization
                    _robot_ctrl->updateVisualization();
                    cheetahMainVisualization->p = _stateEstimate.position;
                }
            } else if (robotType == RobotType::A1){
                // Directly run Control
                _robot_ctrl->runController();
                cheetahMainVisualization->p = _stateEstimate.position;

                // Update Visualization
                _robot_ctrl->updateVisualization();
                cheetahMainVisualization->p = _stateEstimate.position;
            } else {
                assert(false);
            }
        }

    }



    // Visualization (will make this into a separate function later)
    for (int leg = 0; leg < 4; leg++) {
        for (int joint = 0; joint < 3; joint++) {
            cheetahMainVisualization->q[leg * 3 + joint] =
                    _legController->datas[leg].q[joint];
        }
    }
    cheetahMainVisualization->p.setZero();
    cheetahMainVisualization->p = _stateEstimate.position;
    cheetahMainVisualization->quat = _stateEstimate.orientation;

    // Sets the leg controller commands for the robot appropriate commands
    finalizeStep();
}

/*!
 * Before running user code, setup the leg control and estimators
 */
void RobotRunner_sim::setupStep() {
    // Update the leg data
    if (robotType == RobotType::MINI_CHEETAH || robotType == RobotType::A1) {
        _legController->updateData(spiData);
    } else if (robotType == RobotType::CHEETAH_3) {
        _legController->updateData(tiBoardData);
    } else {
        assert(false);
    }

    // Setup the leg controller for a new iteration
    _legController->zeroCommand();
    _legController->setEnabled(true);
    _legController->setMaxTorqueCheetah3(208.5);

    // state estimator
    // check transition to cheater mode:
    if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
        printf("[RobotRunner_sim] Transitioning to Cheater Mode...\n");
        initializeStateEstimator(true);
        // todo any configuration
        _cheaterModeEnabled = true;
    }

    // check transition from cheater mode:
    if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
        printf("[RobotRunner_sim] Transitioning from Cheater Mode...\n");
        initializeStateEstimator(false);
        // todo any configuration
        _cheaterModeEnabled = false;
    }

    get_rc_control_settings(&rc_control);

    // todo safety checks, sanity checks, etc...
}

/*!
 * After the user code, send leg commands, update state estimate, and publish debug data
 */
void RobotRunner_sim::finalizeStep() {
    if (robotType == RobotType::MINI_CHEETAH || robotType == RobotType::A1) {
        _legController->updateCommand(spiCommand);
    } else if (robotType == RobotType::CHEETAH_3) {
        _legController->updateCommand(tiBoardCommand);
    } else {
        assert(false);
    }
    _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
    _stateEstimate.setLcm(state_estimator_lcm);
    _lcm.publish("leg_control_command", &leg_control_command_lcm);
    _lcm.publish("leg_control_data", &leg_control_data_lcm);
    _lcm.publish("state_estimator", &state_estimator_lcm);
    // TODO: Add UNITREE_LEGGED_SDK::LCM publish (Done inside @A1hardwareBridge::runUnitreeLCM())
    _iterations++;
}

/*!
 * Reset the state estimator in the given mode.
 * @param cheaterMode
 */
void RobotRunner_sim::initializeStateEstimator(bool cheaterMode) {
    _stateEstimator->removeAllEstimators();
    _stateEstimator->addEstimator<ContactEstimator<float>>();
    Vec4<float> contactDefault;
    contactDefault << 0.5, 0.5, 0.5, 0.5;
    _stateEstimator->setContactPhase(contactDefault);
    if (cheaterMode) {
        _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
        _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();
    } else {
        _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
        _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
    }
}

RobotRunner_sim::~RobotRunner_sim() {
    delete _legController;
    delete _stateEstimator;
    delete _jpos_initializer;
}

void RobotRunner_sim::cleanup() {}

