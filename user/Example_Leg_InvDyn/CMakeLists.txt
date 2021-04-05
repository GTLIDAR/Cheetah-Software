include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../lcm-types/cpp")
include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include
        $ENV{ALIENGO_SDK_PATH}/include)
link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
link_directories($ENV{ALIENGO_SDK_PATH}/lib)
string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} libaliengo_comm.so)

add_executable(leg_invdyn_ctrl main.cpp Leg_InvDyn_Controller.cpp)
target_link_libraries(leg_invdyn_ctrl robot)
