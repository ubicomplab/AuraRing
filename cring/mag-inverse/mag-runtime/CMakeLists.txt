
include_directories(../../jointAngle/jointAngle/jointAngle)
add_executable(runtime runtime.cpp comms.cpp ../file_utils.cpp ../hand_models.cpp ../mag_models.cpp ../utils.cpp ../../jointAngle/jointAngle/jointAngle/joint_ik.cpp)
target_link_libraries(runtime ${CERES_LIBRARIES})
if(WIN32)
    target_link_libraries(runtime wsock32 ws2_32)
endif()
target_link_libraries(runtime gflags)