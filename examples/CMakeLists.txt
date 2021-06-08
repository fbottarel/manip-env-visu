message("Processing examples")

add_executable(gripper_render
               main.cpp
               )

target_link_libraries(gripper_render
                      ${LIB_TARGET_NAME}
                      ${urdfdom_LIBRARIES}
                      Eigen3::Eigen
                      ${VTK_LIBRARIES}
)