file(GLOB_RECURSE SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(path_preprocessing ${SRCS})

target_include_directories(path_preprocessing SYSTEM PUBLIC
    SYSTEM ${_VENDOR_INCLUDE_DIR_}
)

target_include_directories(path_preprocessing  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(path_preprocessing PUBLIC
    ${_VENDOR_LIBRARIES_}
)
