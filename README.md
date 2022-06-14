## Cheetah-Software for A1 (with new autonomous mode)
This repository contains the Robot and Simulation software project.  For a getting started guide, see the documentation folder.

The common folder contains the common library with dynamics and utilities
The resources folder will contain data files, like CAD of the robot used for the visualization
The robot folder will contain the robot program
The sim folder will contain the simulation program. It is the only program which depends on QT.
The third-party will contain *small* third party libraries that we have modified. This should just be libsoem for Cheetah 3, which Pat modified at one point.


## Configuration
The A1 hardware interface requires installation of `unitree_legged_sdk` and `aliengo_sdk` (not used but has to be installed for future development).
Make sure the following exist in your `~/.bashrc` file or export them in terminal. `melodic`, `gazebo-8`, `~/catkin_ws`, `amd64` and the paths to `unitree_legged_sdk` should be replaced in your own case.
* [unitree_legged_sdk](https://github.com/unitreerobotics) (checkout to v3.2)
* [aliengo_sdk](https://github.com/unitreerobotics) (master branch)
```
export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk
export ALIENGO_SDK_PATH=~/aliengo_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```


## Build
To build all code, follow the same procedures as original Cheetah Software:
```
git checkout mc_a1_nav
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

This build process builds the common library, robot code, and simulator. If you just change robot code, you can simply run `make -j4` again. If you change LCM types, you'll need to run `cmake ..; make -j4`. This automatically runs `make_types.sh`.

To test the common library, run `common/test-common`. To run the robot code, run `robot/robot`. To run the simulator, run `sim/sim`.

Part of this build process will automatically download the gtest software testing framework and sets it up. After it is done building, it will produce a `libbiomimetics.a` static library and an executable `test-common`.  Run the tests with `common/test-common`. This output should hopefully end with

```
[----------] Global test environment tear-down
[==========] 18 tests from 3 test cases ran. (0 ms total)
[  PASSED  ] 18 tests.
```
## Run simulation
The simulator and visualizer are running in the same Qt window, decided by either running the simulation or the hardware. 
This part only shows how to run the simulation. To run the simulator:
1. Open the control board
```
./sim/sim
```
* You will see a panel with three columns of parameters: `Simulator Control Parameters`, `Robot Control Parameters` and `User Control Parameters`. The `Robot Control Parameters` will only show up after you click the start button.
* On the left column, select `A1` and `Simulator`

2. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/A1JPos_Controller/a1jpos_ctrl a s // joint PD control
./user/MIT_Controller/mit_ctrl a s // MIT Controller for A1
```
* The arguments for the controller are:
```  
3: Cheetah 3, m: Mini Cheetah, a: A1
s: simulation, r: robot
```
* Once the controller starts, you will see the `Robot Control Parameters` in the middle column of the simulator. 
A1 will be lying on the ground with zero motor torques. The white one stands for the robot state from state estimator while the colored one is the 
  the ground truth computed by the simulator with noise added.
* By default, the robot is running in the Gamepad mode. Make sure `use_rc` and `auto_mode` are set to `0`. Change the control mode from `0` to `6` to let the robot stand up.
* To let A1 enter the locomotion mode, change the control mode from `6` to `4`.
* If you have a gamepad connected, you can use the left joystick to move the robot forward / backward / sideways. The right joystick can 
change the body height and yaw angle. The right analog trigger can control the stepping height with the maximum to be 0.15m.
* The other control modes are under development or reserved for other actions.

## Run A1 hardware
Currently, the controller code is only supported to be running on the user PC, which means an ethernet cable has to be 
connected between the onboard real-time PC on A1 and the user PC.
1. Boot A1 and let it stand up by itself (this part can't be avoided). Use the unitree controller to lie it down on the ground.
2. Setup the correct IP address and lcm for the user PC (replace `enx00e04c82a7f7` with your own port  first):
```
./scripts/ipconfig.sh
```
Do the ping test to verify the connection to the lower-level controller board:
```
ping 192.168.123.10
```
You should be able to get response from `192.168.123.10`.

3. Open the unitree LCM server under `unitree_legged_sdk`
```
cd /path/to/unitree_legged_sdk/build
sudo ./lcm_server_low
```

4. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/A1JPos_Controller/a1jpos_ctrl a s // joint PD control
./user/MIT_Controller/mit_ctrl a s // MIT Controller for A1
```
* The arguments for the controller are:
```  
3: Cheetah 3, m: Mini Cheetah, a: A1
s: simulation, r: robot
```
* If everything is initialized correctly, you will see the controller is waiting for parameters

5. Open the control panel
```
./sim/sim
```
* Press the `load` button on the right column and select the user control parameters for A1. Use `a1-mit-ctrl-user-parameters`
  for the MIT controller and `a1-jpos-user-parameters` for the joint PD controller.
* On the left column, select `A1` and `Robot`
* Click `Start` button to open the RobotInterface (make sure A1 is lying on the ground!! Press L2+B after A1 stands up using its default controller). This step will send all the necessary control parameters to the controller through LCM
* You should be able to see the state estimation result showing A1 lying on the ground. Otherwise, double check the 
  network setup.
* Similar to the simulation, you will see the `Robot Control Parameters` in the middle column of the simulator.
  However, you can only see the state estimation result now.
* By default, the robot is running in the Gamepad mode. Make sure `use_rc` and `auto_mode` are set to `0`. Change the control mode from `0` to `6` to let the robot stand up.
* To let A1 enter the locomotion mode, change the control mode from `6` to `4`.
* If you have a gamepad connected, you can use the left joystick to move the robot forward / backward / sideways. The right joystick can
  change the body height and yaw angle. The right analog trigger can control the stepping height with the maximum to be 0.15m.
* The other control modes are under development or reserved for other actions.

6. Autonomous mode:
* To enter the autonomous mode, change `auto_mode` from `0` to `1`.
* Before receiving any customized high-level command from the perception PC, change the control
  mode from `0` to `6` to let the robot stand up.
* Send customized high-level command from perception PC. So far the robot will switch between
  `4` locomotion mode and `3` balance stand mode depending on the commanded velocity.

7. Shut down the robot:
* Kill the `lcm_server_low`. The robot will go into braking mode and lie down by itself.
* Kill the `sim` and `controller` terminals.
* This is not recommended but if something really bad happens, press the `off` button on the emergency remote.
  Note that this will shut down the motor and the robot will fall hardly to the ground.
  Only use this when the robot is hung up.

## Dependencies:
- Qt 5.10 - https://www.qt.io/download-qt-installer
- LCM - https://lcm-proj.github.io/ (Please make it sure that you have a java to let lcm compile java-extension together)
- Eigen - http://eigen.tuxfamily.org
- `mesa-common-dev`
- `freeglut3-dev`
- `libblas-dev liblapack-dev`

To use Ipopt, use CMake Ipopt option. Ex) cmake -DIPOPT_OPTION=ON ..
