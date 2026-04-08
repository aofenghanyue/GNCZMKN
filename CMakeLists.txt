cmake_minimum_required(VERSION 3.16)
project(gnc_sim VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(framework)

set(USER_COMPONENTS_DIR "${CMAKE_SOURCE_DIR}/user/components")
file(GLOB_RECURSE USER_COMPONENT_FILES CONFIGURE_DEPENDS
    RELATIVE "${CMAKE_SOURCE_DIR}"
    "user/components/*.hpp"
)

set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_DIR}")

set(AUTO_REGISTER_FILE "${GENERATED_DIR}/user_components_register.hpp")
file(WRITE "${AUTO_REGISTER_FILE}" "#pragma once\n\n")

if(USER_COMPONENT_FILES)
    foreach(COMP_FILE IN LISTS USER_COMPONENT_FILES)
        file(APPEND "${AUTO_REGISTER_FILE}" "#include \"${COMP_FILE}\"\n")
    endforeach()
else()
    file(APPEND "${AUTO_REGISTER_FILE}" "\n")
endif()

list(LENGTH USER_COMPONENT_FILES USER_COMPONENT_COUNT)
message(STATUS "Found ${USER_COMPONENT_COUNT} user component(s) in ${USER_COMPONENTS_DIR}")

add_executable(gnc_sim src/runner.cpp)
target_link_libraries(gnc_sim PRIVATE gnc_framework)
target_include_directories(gnc_sim PRIVATE
    "${GENERATED_DIR}"
    "${CMAKE_SOURCE_DIR}"
    "${USER_COMPONENTS_DIR}"
)

if(MSVC)
    target_compile_options(gnc_sim PRIVATE /W4 /utf-8)
else()
    target_compile_options(gnc_sim PRIVATE -Wall -Wextra -pedantic)
endif()

option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES AND EXISTS "${CMAKE_SOURCE_DIR}/examples/CMakeLists.txt")
    add_subdirectory(examples)
endif()

option(BUILD_TESTS "Build test programs" OFF)
if(BUILD_TESTS)
    add_executable(test_transform tests/test_transform.cpp)
    target_link_libraries(test_transform PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_transform PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_transform PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_math_headers tests/test_math_headers.cpp)
    target_link_libraries(test_math_headers PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_math_headers PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_math_headers PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_integrators tests/test_integrators.cpp)
    target_link_libraries(test_integrators PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_integrators PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_integrators PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_cross_entity_access tests/test_cross_entity_access.cpp)
    target_link_libraries(test_cross_entity_access PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_cross_entity_access PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_cross_entity_access PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_dependency_validation tests/test_dependency_validation.cpp)
    target_link_libraries(test_dependency_validation PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_dependency_validation PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_dependency_validation PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_typed_state_providers tests/test_typed_state_providers.cpp)
    target_link_libraries(test_typed_state_providers PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_typed_state_providers PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_typed_state_providers PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_stop_conditions tests/test_stop_conditions.cpp)
    target_link_libraries(test_stop_conditions PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_stop_conditions PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_stop_conditions PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_dependency_preflight tests/test_dependency_preflight.cpp)
    target_link_libraries(test_dependency_preflight PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_dependency_preflight PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_dependency_preflight PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_dependency_injection_lifecycle tests/test_dependency_injection_lifecycle.cpp)
    target_link_libraries(test_dependency_injection_lifecycle PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_dependency_injection_lifecycle PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_dependency_injection_lifecycle PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_starter_dependency_preflight tests/test_starter_dependency_preflight.cpp)
    target_link_libraries(test_starter_dependency_preflight PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_starter_dependency_preflight PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_starter_dependency_preflight PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_declared_dependency_preflight tests/test_declared_dependency_preflight.cpp)
    target_link_libraries(test_declared_dependency_preflight PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_declared_dependency_preflight PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_declared_dependency_preflight PRIVATE -Wall -Wextra -pedantic)
    endif()

    add_executable(test_binding_diagnostics tests/test_binding_diagnostics.cpp)
    target_link_libraries(test_binding_diagnostics PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_binding_diagnostics PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_binding_diagnostics PRIVATE -Wall -Wextra -pedantic)
    endif()
endif()

message(STATUS "GNC Simulation Framework v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "User components dir: ${USER_COMPONENTS_DIR}")
message(STATUS "Auto-register file: ${AUTO_REGISTER_FILE}")
