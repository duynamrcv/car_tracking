include_directories(generated_model)

file(GLOB_RECURSE SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

file(GLOB_RECURSE ACADOS_LIB_SRC 
    ${CMAKE_CURRENT_SOURCE_DIR}/generated_model/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/generated_model/*.h
)

add_library(path_tracking ${SRCS} ${ACADOS_LIB_SRC})

target_include_directories(path_tracking SYSTEM PUBLIC
    SYSTEM ${_VENDOR_INCLUDE_DIR_}
)

target_include_directories(path_tracking PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/generated_model
)

target_link_libraries(path_tracking PUBLIC
    ${_VENDOR_LIBRARIES_}
)
