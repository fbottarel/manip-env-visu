set(EXE_TARGET_NAME gripper_render)

find_package(manip-env-visu REQUIRED)

set(${EXE_TARGET_NAME}_SRC
    main.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_SRC}
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      manip-env-visu::manip-env-visu
)