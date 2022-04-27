include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include
        $ENV{ALIENGO_SDK_PATH}/include)
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories($ENV{ALIENGO_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} libaliengo_comm.so)

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")

add_library(user_controller SHARED ${sources})
target_link_libraries(user_controller inih dynacore_param_handler JCQP osqp qpOASES Goldfarb_Optimizer ${EXTRA_LIBS})

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(mit_ctrl robot biomimetics)
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)
target_link_libraries(mit_ctrl VisionMPC)
