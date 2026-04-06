add_executable(example_gravity_turn
    02_gravity_turn/simple_gravity_turn.cpp
)

target_link_libraries(example_gravity_turn PRIVATE gnc_framework)

if(MSVC)
    target_compile_options(example_gravity_turn PRIVATE /W4 /utf-8)
else()
    target_compile_options(example_gravity_turn PRIVATE -Wall -Wextra -pedantic)
endif()

add_executable(example_cavh_3dof
    03_cavh_3dof/main.cpp
)

target_link_libraries(example_cavh_3dof PRIVATE gnc_framework)
target_include_directories(example_cavh_3dof PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/03_cavh_3dof
)

if(MSVC)
    target_compile_options(example_cavh_3dof PRIVATE /W4 /utf-8)
else()
    target_compile_options(example_cavh_3dof PRIVATE -Wall -Wextra -pedantic)
endif()
