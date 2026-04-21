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
set(GNC_COMPONENT_DISCOVERY_MODE "explicit builtin bootstrap")

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

set(GNC_ATMOSPHERE_DATA_FILE
    "${CMAKE_SOURCE_DIR}/framework/data/atmosphere/ussa1976_si_merged.csv")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${GNC_ATMOSPHERE_DATA_FILE}")

function(generate_ussa1976_atmosphere_header input_file output_file)
    file(STRINGS "${input_file}" atmosphere_lines)

    set(altitude_values "")
    set(temperature_values "")
    set(pressure_values "")
    set(density_values "")
    set(sound_speed_values "")

    foreach(line IN LISTS atmosphere_lines)
        if(line STREQUAL "" OR line MATCHES "^[ \t]*#")
            continue()
        endif()

        string(REPLACE "," ";" fields "${line}")
        list(LENGTH fields field_count)
        if(NOT field_count EQUAL 5)
            message(FATAL_ERROR
                "Atmosphere data line must contain exactly 5 comma-separated fields: '${line}'")
        endif()

        list(GET fields 0 altitude_km)
        list(GET fields 1 temperature_k)
        list(GET fields 2 pressure_pa)
        list(GET fields 3 density_kgpm3)
        list(GET fields 4 sound_speed_mps)

        list(APPEND altitude_values "${altitude_km}")
        list(APPEND temperature_values "${temperature_k}")
        list(APPEND pressure_values "${pressure_pa}")
        list(APPEND density_values "${density_kgpm3}")
        list(APPEND sound_speed_values "${sound_speed_mps}")
    endforeach()

    list(LENGTH altitude_values atmosphere_table_size)
    if(atmosphere_table_size LESS 2)
        message(FATAL_ERROR "Atmosphere data must contain at least 2 rows.")
    endif()

    string(JOIN ", " altitude_joined ${altitude_values})
    string(JOIN ", " temperature_joined ${temperature_values})
    string(JOIN ", " pressure_joined ${pressure_values})
    string(JOIN ", " density_joined ${density_values})
    string(JOIN ", " sound_speed_joined ${sound_speed_values})

    set(header_content
"#pragma once

#include <array>
#include <cstddef>

namespace gnc::generated {

inline constexpr std::size_t kUssa1976TableSize = ${atmosphere_table_size};
inline constexpr std::array<double, kUssa1976TableSize> kUssa1976AltitudeKm = {${altitude_joined}};
inline constexpr std::array<double, kUssa1976TableSize> kUssa1976TemperatureK = {${temperature_joined}};
inline constexpr std::array<double, kUssa1976TableSize> kUssa1976PressurePa = {${pressure_joined}};
inline constexpr std::array<double, kUssa1976TableSize> kUssa1976DensityKgPerM3 = {${density_joined}};
inline constexpr std::array<double, kUssa1976TableSize> kUssa1976SpeedOfSoundMps = {${sound_speed_joined}};

} // namespace gnc::generated
")

    file(WRITE "${output_file}" "${header_content}")
endfunction()

set(GNC_ATMOSPHERE_HEADER_FILE "${GENERATED_DIR}/gnc_generated_ussa1976_atmosphere.hpp")
generate_ussa1976_atmosphere_header(
    "${GNC_ATMOSPHERE_DATA_FILE}"
    "${GNC_ATMOSPHERE_HEADER_FILE}")

set(AUTO_REGISTER_FILE "${GENERATED_DIR}/auto_registered_components.hpp")
file(WRITE "${AUTO_REGISTER_FILE}" "#pragma once\n\n")

set(AUTO_REGISTER_CALLS "")
if(USER_COMPONENT_FILES)
    set(COMP_INDEX 0)
    foreach(COMP_FILE IN LISTS USER_COMPONENT_FILES)
        file(APPEND "${AUTO_REGISTER_FILE}" "#define GNC_COMPONENT_REGISTRATION_FN gnc_register_project_component_${COMP_INDEX}\n")
        file(APPEND "${AUTO_REGISTER_FILE}" "#include \"${COMP_FILE}\"\n")
        file(APPEND "${AUTO_REGISTER_FILE}" "#undef GNC_COMPONENT_REGISTRATION_FN\n\n")
        string(APPEND AUTO_REGISTER_CALLS "    ::gnc_register_project_component_${COMP_INDEX}(factory);\n")
        math(EXPR COMP_INDEX "${COMP_INDEX} + 1")
    endforeach()
endif()

file(APPEND "${AUTO_REGISTER_FILE}" "#include \"gnc/core/component_factory.hpp\"\n\n")
file(APPEND "${AUTO_REGISTER_FILE}" "namespace gnc::build {\n")
file(APPEND "${AUTO_REGISTER_FILE}" "inline void registerAutoRegisteredProjectComponents(::gnc::core::ComponentFactory& factory) {\n")
file(APPEND "${AUTO_REGISTER_FILE}" "    (void)factory;\n")
if(AUTO_REGISTER_CALLS)
    file(APPEND "${AUTO_REGISTER_FILE}" "${AUTO_REGISTER_CALLS}")
endif()
file(APPEND "${AUTO_REGISTER_FILE}" "}\n")
file(APPEND "${AUTO_REGISTER_FILE}" "} // namespace gnc::build\n")

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
    enable_testing()

    function(add_gnc_test target source)
        add_executable(${target} ${source})
        target_link_libraries(${target} PRIVATE gnc_framework)
        target_include_directories(${target} PRIVATE
            "${CMAKE_SOURCE_DIR}"
            "${GENERATED_DIR}"
        )
        if(USER_COMPONENTS_DIR)
            target_include_directories(${target} PRIVATE "${USER_COMPONENTS_DIR}")
        endif()
        if(MSVC)
            target_compile_options(${target} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -pedantic)
        endif()
        add_test(NAME ${target} COMMAND ${target})
    endfunction()

    add_gnc_test(test_plugin_registry tests/test_plugin_registry.cpp)
    add_gnc_test(test_environment_plugin tests/test_environment_plugin.cpp)
    add_gnc_test(test_state_3dof_plugin tests/test_state_3dof_plugin.cpp)
    add_gnc_test(test_soviet_coord_plugin tests/test_soviet_coord_plugin.cpp)
    add_gnc_test(test_pluginized_build tests/test_pluginized_build.cpp)
    add_gnc_test(test_mission_contract tests/test_mission_contract.cpp)
    add_gnc_test(test_component_listing tests/test_component_listing.cpp)
    add_gnc_test(test_auto_data_logger tests/test_auto_data_logger.cpp)
endif()

message(STATUS "GNC Simulation Framework v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "User project components dir: ${USER_COMPONENTS_DIR}")
message(STATUS "Auto-register file: ${AUTO_REGISTER_FILE}")
