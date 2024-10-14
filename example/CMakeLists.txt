cmake_minimum_required(VERSION 3.5)

add_executable(example main.cpp)

target_link_libraries(example
    path_preprocessing
)

target_include_directories(example PRIVATE
    ${CMAKE_SOURCE_DIR}
)
