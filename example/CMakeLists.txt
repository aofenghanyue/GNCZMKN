# Example 构建配置
# 在项目根目录 CMakeLists.txt 末尾添加以下行即可启用:
#   add_subdirectory(example)

add_executable(simple_gravity_turn simple_gravity_turn.cpp)
target_link_libraries(simple_gravity_turn PRIVATE Eigen3::Eigen)
target_include_directories(simple_gravity_turn PRIVATE ${CMAKE_SOURCE_DIR}/include)

if(MSVC)
    target_compile_options(simple_gravity_turn PRIVATE /W4 /utf-8)
else()
    target_compile_options(simple_gravity_turn PRIVATE -Wall -Wextra -pedantic)
endif()
