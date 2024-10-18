file(GLOB_RECURSE SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(path_tracking ${SRCS})

target_include_directories(path_tracking SYSTEM PUBLIC
    SYSTEM ${_VENDOR_INCLUDE_DIR_}
)

target_include_directories(path_tracking  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/generated_model
    ${CMAKE_CURRENT_SOURCE_DIR}/include/generated_model/CarModel_model
)

target_link_libraries(path_tracking PUBLIC
    ${_VENDOR_LIBRARIES_}
)
