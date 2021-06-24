add_executable(run_kitti_stereo run_kitti_stereo.cpp)
target_link_libraries(run_kitti_stereo slam ${THIRD_PARTY_LIBS})

add_executable(run_tum_rgbd run_tum_rgbd.cpp)
target_link_libraries(run_tum_rgbd slam ${THIRD_PARTY_LIBS})

