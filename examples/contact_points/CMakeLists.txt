set(EXE_TARGET_NAME contact_points)

set(${EXE_TARGET_NAME}_SRC
    contact_points.cpp
)

add_executable(${EXE_TARGET_NAME}
               ${${EXE_TARGET_NAME}_SRC}
)

target_link_libraries(${EXE_TARGET_NAME}
                      PRIVATE
                      manip-env-visu
)