/*! @file SimulationBridge.h
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <thread>
#include <lcm-cpp.hpp>

#include "ControlParameters/RobotParameters.h"
#include "RobotRunner_sim.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"

class SimulationBridge {
 public:
  explicit SimulationBridge(RobotType robot, RobotController* robot_ctrl) : 
    _robot(robot),
    _highCmdLCM(getLcmUrl(255)){
     _fakeTaskManager = new PeriodicTaskManager;
    _robotRunner = new RobotRunner_sim(robot_ctrl, _fakeTaskManager, 0, "robot-task");
    _userParams = robot_ctrl->getUserControlParameters();
 }
  void run();
  void handleControlParameters();
  void handleHighCmd(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const custom_cmd_lcmt *msg);
  void handleHighCmdLCM();
  void runRobotControl();
  ~SimulationBridge() {
    delete _fakeTaskManager;
    delete _robotRunner;
  }
  void run_sbus();

 private:
  PeriodicTaskManager taskManager;
  bool _firstControllerRun = true;
  PeriodicTaskManager* _fakeTaskManager = nullptr;
  RobotType _robot;
  HighCmdCustom _highlevelCommand;
  RobotRunner_sim* _robotRunner = nullptr;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  RobotControlParameters _robotParams;
  ControlParameters* _userParams = nullptr;
  u64 _iterations = 0;
  lcm::LCM _highCmdLCM;
  std::thread _highCmdLcmThread;
  volatile bool _interfaceLcmQuit = false;

  std::thread* sbus_thread;
};

#endif  // PROJECT_SIMULATIONDRIVER_H
