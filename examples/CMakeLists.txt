add_executable(example_gravity_turn
    02_gravity_turn/simple_gravity_turn.cpp
)

target_link_libraries(example_gravity_turn PRIVATE gnc_framework)

if(MSVC)
    target_compile_options(example_gravity_turn PRIVATE /W4 /utf-8)
else()
    target_compile_options(example_gravity_turn PRIVATE -Wall -Wextra -pedantic)
endif()
