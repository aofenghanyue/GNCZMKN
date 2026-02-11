cmake_minimum_required(VERSION 3.16)
project(gnc_sim VERSION 1.0.0 LANGUAGES CXX)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 查找 Eigen3
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found, fetching from git...")
    include(FetchContent)
    FetchContent_Declare(
        Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
    )
    FetchContent_MakeAvailable(Eigen3)
    # Eigen 3.4.0 via FetchContent might not define Eigen3::Eigen alias if not installed
    if(NOT TARGET Eigen3::Eigen)
        if(TARGET eigen)
             add_library(Eigen3::Eigen ALIAS eigen)
        else()
             # If eigen target is not defined (e.g. header only), we might need to include directories manually
             # But usually Eigen defines 'eigen' target.
             message(STATUS "Eigen3 target 'eigen' not found after FetchContent.")
        endif()
    endif()
endif()

# 包含目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 收集源文件
file(GLOB_RECURSE SOURCES 
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# 创建可执行文件
add_executable(gnc_sim ${SOURCES})

# 链接 Eigen3
if(TARGET Eigen3::Eigen)
    target_link_libraries(gnc_sim PRIVATE Eigen3::Eigen)
else()
    # Fallback if target is missing but include dirs are set (rare with FetchContent)
    include_directories(${Eigen3_INCLUDE_DIRS})
endif()

# 编译选项
if(MSVC)
    target_compile_options(gnc_sim PRIVATE /W4 /utf-8)
else()
    target_compile_options(gnc_sim PRIVATE -Wall -Wextra -pedantic)
endif()

# 打印配置信息
message(STATUS "GNC Simulation Framework v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

# 测试可执行文件
add_executable(test_transform tests/test_transform.cpp)
target_link_libraries(test_transform PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(test_transform PRIVATE /W4 /utf-8)
else()
    target_compile_options(test_transform PRIVATE -Wall -Wextra -pedantic)
endif()

# 数学库测试
add_executable(test_math_headers tests/test_math_headers.cpp)
target_link_libraries(test_math_headers PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(test_math_headers PRIVATE /W4 /utf-8)
else()
    target_compile_options(test_math_headers PRIVATE -Wall -Wextra -pedantic)
endif()

# 性能测试
add_executable(benchmark_optimization tests/benchmark_optimization.cpp)
target_link_libraries(benchmark_optimization PRIVATE Eigen3::Eigen)
if(MSVC)
    target_compile_options(benchmark_optimization PRIVATE /W4 /utf-8)
else()
    target_compile_options(benchmark_optimization PRIVATE -Wall -Wextra -pedantic)
endif()
