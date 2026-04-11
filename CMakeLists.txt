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

set(GNC_ACTIVE_PROJECT "" CACHE STRING
    "Active project name under user/ with components/ and config/mission.json")
set(GNC_ACTIVE_PROJECT_FILE "${CMAKE_SOURCE_DIR}/user/active_project")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${GNC_ACTIVE_PROJECT_FILE}")
set(GNC_COMPONENT_DISCOVERY_MODE "starter only")

if(GNC_ACTIVE_PROJECT STREQUAL "" AND EXISTS "${GNC_ACTIVE_PROJECT_FILE}")
    configure_file("${GNC_ACTIVE_PROJECT_FILE}"
                   "${CMAKE_BINARY_DIR}/active_project.stamp"
                   COPYONLY)
    file(READ "${GNC_ACTIVE_PROJECT_FILE}" GNC_ACTIVE_PROJECT)
    string(STRIP "${GNC_ACTIVE_PROJECT}" GNC_ACTIVE_PROJECT)
endif()

set(USER_PROJECT_DIR "")
set(USER_COMPONENTS_DIR "")
set(GNC_DEFAULT_CONFIG_RELATIVE "user/config/missions/default.json")
set(USER_COMPONENT_FILES "")

if(NOT GNC_ACTIVE_PROJECT STREQUAL "")
    if(GNC_ACTIVE_PROJECT MATCHES "[/\\\\]")
        message(FATAL_ERROR
            "GNC active project must be a project name under user/, but got '${GNC_ACTIVE_PROJECT}'.")
    endif()

    set(USER_PROJECT_DIR "${CMAKE_SOURCE_DIR}/user/${GNC_ACTIVE_PROJECT}")
    if(NOT IS_DIRECTORY "${USER_PROJECT_DIR}")
        message(FATAL_ERROR
            "Active project '${GNC_ACTIVE_PROJECT}' does not exist under user/.")
    endif()

    set(GNC_COMPONENT_DISCOVERY_MODE "user project")
    set(USER_COMPONENTS_DIR "${USER_PROJECT_DIR}/components")
    if(IS_DIRECTORY "${USER_COMPONENTS_DIR}")
        file(GLOB_RECURSE USER_COMPONENT_FILES CONFIGURE_DEPENDS
            RELATIVE "${CMAKE_SOURCE_DIR}"
            "user/${GNC_ACTIVE_PROJECT}/components/*.hpp"
        )
    endif()

    if(EXISTS "${USER_PROJECT_DIR}/config/mission.json")
        set(GNC_DEFAULT_CONFIG_RELATIVE "user/${GNC_ACTIVE_PROJECT}/config/mission.json")
    else()
        message(STATUS
            "Active project '${GNC_ACTIVE_PROJECT}' has no config/mission.json; "
            "gnc_sim will keep the legacy default mission '${GNC_DEFAULT_CONFIG_RELATIVE}'.")
    endif()
endif()

set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_DIR}")

set(AUTO_REGISTER_FILE "${GENERATED_DIR}/auto_registered_components.hpp")
file(WRITE "${AUTO_REGISTER_FILE}" "#pragma once\n\n")

if(USER_COMPONENT_FILES)
    foreach(COMP_FILE IN LISTS USER_COMPONENT_FILES)
        file(APPEND "${AUTO_REGISTER_FILE}" "#include \"${COMP_FILE}\"\n")
    endforeach()
else()
    file(APPEND "${AUTO_REGISTER_FILE}" "\n")
endif()

set(ACTIVE_PROJECT_CONFIG_FILE "${GENERATED_DIR}/active_project_config.hpp")
file(WRITE "${ACTIVE_PROJECT_CONFIG_FILE}" "#pragma once\n\n")
file(APPEND "${ACTIVE_PROJECT_CONFIG_FILE}" "namespace gnc::build {\n")
file(APPEND "${ACTIVE_PROJECT_CONFIG_FILE}" "inline constexpr const char* kActiveProjectName = \"${GNC_ACTIVE_PROJECT}\";\n")
file(APPEND "${ACTIVE_PROJECT_CONFIG_FILE}" "inline constexpr const char* kDefaultConfigRelative = \"${GNC_DEFAULT_CONFIG_RELATIVE}\";\n")
file(APPEND "${ACTIVE_PROJECT_CONFIG_FILE}" "} // namespace gnc::build\n")

list(LENGTH USER_COMPONENT_FILES USER_COMPONENT_COUNT)
message(STATUS "Component discovery mode: ${GNC_COMPONENT_DISCOVERY_MODE}")
if(NOT GNC_ACTIVE_PROJECT STREQUAL "")
    message(STATUS "Active project: ${GNC_ACTIVE_PROJECT}")
endif()
message(STATUS "Auto-registered user project component headers: ${USER_COMPONENT_COUNT}")

add_executable(gnc_sim src/runner.cpp)
target_link_libraries(gnc_sim PRIVATE gnc_framework)
target_include_directories(gnc_sim PRIVATE
    "${GENERATED_DIR}"
    "${CMAKE_SOURCE_DIR}"
)
if(USER_COMPONENTS_DIR)
    target_include_directories(gnc_sim PRIVATE "${USER_COMPONENTS_DIR}")
endif()

if(MSVC)
    target_compile_options(gnc_sim PRIVATE /W4 /utf-8)
else()
    target_compile_options(gnc_sim PRIVATE -Wall -Wextra -pedantic)
endif()

option(BUILD_TESTS "Build test programs" OFF)
if(BUILD_TESTS)
    add_executable(test_coordinate_service tests/test_coordinate_service.cpp)
    target_link_libraries(test_coordinate_service PRIVATE gnc_framework)
    if(MSVC)
        target_compile_options(test_coordinate_service PRIVATE /W4 /utf-8)
    else()
        target_compile_options(test_coordinate_service PRIVATE -Wall -Wextra -pedantic)
    endif()

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
message(STATUS "User project components dir: ${USER_COMPONENTS_DIR}")
message(STATUS "Auto-register file: ${AUTO_REGISTER_FILE}")
