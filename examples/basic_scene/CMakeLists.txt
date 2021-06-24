set(EXE_TARGET_NAME basic_scene)

set(${EXE_TARGET_NAME}_SRC
    basic_scene.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_SRC}
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      manip-env-visu
)