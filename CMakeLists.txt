cmake_minimum_required(VERSION 3.0)

# 设置项目名称和语言
project(RRTMPC LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(rrt STATIC planning/rapidly_exploring_random_tree_star.cc)
target_compile_options(rrt PRIVATE "-O3")

add_library(path_opt STATIC planning/path_optimization.cc)
target_compile_options(path_opt PRIVATE "-O3")

add_library(mpc STATIC control/model_predictive_control.cc)
target_link_libraries(mpc PRIVATE qpOASES)
target_compile_options(mpc PRIVATE "-O3")

# 添加可执行文件
add_executable(test test/test.cc)
target_link_libraries(test
PRIVATE
  rrt
  mpc
  path_opt
)
target_compile_options(test PRIVATE "-O3")
