cmake_minimum_required(VERSION 3.0)

# 设置项目名称和语言
project(rrt LANGUAGES CXX)

# 查找Boost库
find_package(Boost REQUIRED COMPONENTS system filesystem)

# 添加可执行文件
add_executable(rrt algorithm/RRTStar.cc)

# 设置C++标准
set_target_properties(rrt PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# 添加头文件搜索路径
target_include_directories(rrt PRIVATE ${Boost_INCLUDE_DIRS})

# 添加链接库
target_link_libraries(rrt PRIVATE ${Boost_LIBRARIES})

target_compile_options(rrt PRIVATE "-O3")

# 添加可执行文件
add_executable(mpc algorithm/MPC.cc)

# 添加头文件搜索路径
target_include_directories(mpc PRIVATE ${Boost_INCLUDE_DIRS})

# 添加链接库
target_link_libraries(mpc PRIVATE ${Boost_LIBRARIES})

target_compile_options(mpc PRIVATE "-O3")

# 添加可执行文件
add_executable(test test/main.cc)

# 添加头文件搜索路径
target_include_directories(test PRIVATE ${Boost_INCLUDE_DIRS})

# 添加链接库
target_link_libraries(test PRIVATE ${Boost_LIBRARIES})

target_compile_options(test PRIVATE "-O3")