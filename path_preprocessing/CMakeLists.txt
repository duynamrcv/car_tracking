file(GLOB_RECURSE SRCS 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(lightweight_ogm ${SRCS})

target_include_directories(lightweight_ogm SYSTEM PUBLIC
    SYSTEM ${_VENDOR_INCLUDE_DIR_}
)

target_include_directories(lightweight_ogm  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lightweight_ogm PUBLIC
    ${_VENDOR_LIBRARIES_}
)
