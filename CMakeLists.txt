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

set(GNC_ACTIVE_PROJECT_FROM_FILE "")
if(EXISTS "${GNC_ACTIVE_PROJECT_FILE}")
    configure_file("${GNC_ACTIVE_PROJECT_FILE}"
                   "${CMAKE_BINARY_DIR}/active_project.stamp"
                   COPYONLY)
    file(READ "${GNC_ACTIVE_PROJECT_FILE}" GNC_ACTIVE_PROJECT_FROM_FILE)
    string(STRIP "${GNC_ACTIVE_PROJECT_FROM_FILE}" GNC_ACTIVE_PROJECT_FROM_FILE)
endif()

if(NOT GNC_ACTIVE_PROJECT STREQUAL "" AND
   NOT GNC_ACTIVE_PROJECT_FROM_FILE STREQUAL "" AND
   NOT GNC_ACTIVE_PROJECT STREQUAL GNC_ACTIVE_PROJECT_FROM_FILE)
    message(WARNING
        "GNC_ACTIVE_PROJECT cache value '${GNC_ACTIVE_PROJECT}' overrides "
        "user/active_project value '${GNC_ACTIVE_PROJECT_FROM_FILE}'.")
endif()

if(GNC_ACTIVE_PROJECT STREQUAL "" AND NOT GNC_ACTIVE_PROJECT_FROM_FILE STREQUAL "")
    set(GNC_ACTIVE_PROJECT "${GNC_ACTIVE_PROJECT_FROM_FILE}")
endif()

set(USER_PROJECT_DIR "")
set(USER_COMPONENTS_DIR "")
set(GNC_DEFAULT_CONFIG_RELATIVE "")
set(USER_COMPONENT_FILES "")
set(USER_SIMFLOW_DIR "")
set(USER_SIMFLOW_FILES "")

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
        set(USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION "")
        foreach(USER_COMPONENT_FILE IN LISTS USER_COMPONENT_FILES)
            file(READ "${CMAKE_SOURCE_DIR}/${USER_COMPONENT_FILE}" USER_COMPONENT_HEADER_TEXT)
            # TODO: Replace this coarse string check with an explicit project
            # registration manifest so comments or string literals cannot pass
            # as component registration.
            string(FIND "${USER_COMPONENT_HEADER_TEXT}"
                        "GNC_REGISTER_COMPONENT_TYPE"
                        USER_COMPONENT_REGISTRATION_POS)
            if(USER_COMPONENT_REGISTRATION_POS EQUAL -1)
                list(APPEND USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION
                    "${USER_COMPONENT_FILE}")
            endif()
        endforeach()
        if(USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION)
            string(JOIN ", "
                USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION_TEXT
                ${USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION})
            message(FATAL_ERROR
                "Headers under user/${GNC_ACTIVE_PROJECT}/components/ must be "
                "registrable component headers containing GNC_REGISTER_COMPONENT_TYPE. "
                "Move shared project interfaces or utilities to interfaces/, common/, "
                "or include/. Offending files: "
                "${USER_COMPONENT_HEADERS_WITHOUT_REGISTRATION_TEXT}")
        endif()
    endif()

    set(USER_SIMFLOW_DIR "${USER_PROJECT_DIR}/simflows")
    if(IS_DIRECTORY "${USER_SIMFLOW_DIR}")
        file(GLOB_RECURSE USER_SIMFLOW_FILES CONFIGURE_DEPENDS
            RELATIVE "${CMAKE_SOURCE_DIR}"
            "user/${GNC_ACTIVE_PROJECT}/simflows/*.hpp"
        )
        set(USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION "")
        foreach(USER_SIMFLOW_FILE IN LISTS USER_SIMFLOW_FILES)
            file(READ "${CMAKE_SOURCE_DIR}/${USER_SIMFLOW_FILE}" USER_SIMFLOW_HEADER_TEXT)
            string(FIND "${USER_SIMFLOW_HEADER_TEXT}"
                        "GNC_REGISTER_SIMFLOW_MATERIALIZER"
                        USER_SIMFLOW_REGISTRATION_POS)
            if(USER_SIMFLOW_REGISTRATION_POS EQUAL -1)
                list(APPEND USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION
                    "${USER_SIMFLOW_FILE}")
            endif()
        endforeach()
        if(USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION)
            string(JOIN ", "
                USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION_TEXT
                ${USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION})
            message(FATAL_ERROR
                "Headers under user/${GNC_ACTIVE_PROJECT}/simflows/ must be "
                "registrable simflow materializer headers containing "
                "GNC_REGISTER_SIMFLOW_MATERIALIZER. Move shared helpers to "
                "common/ or include/. Offending files: "
                "${USER_SIMFLOW_HEADERS_WITHOUT_REGISTRATION_TEXT}")
        endif()
    endif()

    if(NOT EXISTS "${USER_PROJECT_DIR}/config/mission.json")
        message(FATAL_ERROR
            "Active project '${GNC_ACTIVE_PROJECT}' must provide "
            "config/mission.json. Create user/${GNC_ACTIVE_PROJECT}/config/mission.json "
            "or run gnc_sim with --config <path> from a build without an active project.")
    endif()
    set(GNC_DEFAULT_CONFIG_RELATIVE "user/${GNC_ACTIVE_PROJECT}/config/mission.json")
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

function(generate_component_registration_header output_file function_name symbol_prefix)
    set(component_files ${ARGN})

    file(WRITE "${output_file}" "#pragma once\n\n")

    set(auto_register_calls "")
    if(component_files)
        set(comp_index 0)
        foreach(comp_file IN LISTS component_files)
            file(APPEND "${output_file}" "#define GNC_COMPONENT_REGISTRATION_FN ${symbol_prefix}_${comp_index}\n")
            file(APPEND "${output_file}" "#include \"${comp_file}\"\n")
            file(APPEND "${output_file}" "#undef GNC_COMPONENT_REGISTRATION_FN\n\n")
            string(APPEND auto_register_calls "    ::${symbol_prefix}_${comp_index}(factory);\n")
            math(EXPR comp_index "${comp_index} + 1")
        endforeach()
    endif()

    file(APPEND "${output_file}" "#include \"gnc/core/component_factory.hpp\"\n\n")
    file(APPEND "${output_file}" "namespace gnc::build {\n")
    file(APPEND "${output_file}" "inline void ${function_name}(::gnc::core::ComponentFactory& factory) {\n")
    file(APPEND "${output_file}" "    (void)factory;\n")
    if(auto_register_calls)
        file(APPEND "${output_file}" "${auto_register_calls}")
    endif()
    file(APPEND "${output_file}" "}\n")
    file(APPEND "${output_file}" "} // namespace gnc::build\n")
endfunction()

function(generate_simflow_registration_header output_file function_name symbol_prefix)
    set(simflow_files ${ARGN})

    file(WRITE "${output_file}" "#pragma once\n\n")

    set(auto_register_calls "")
    if(simflow_files)
        set(simflow_index 0)
        foreach(simflow_file IN LISTS simflow_files)
            file(APPEND "${output_file}" "#define GNC_SIMFLOW_MATERIALIZER_REGISTRATION_FN ${symbol_prefix}_${simflow_index}\n")
            file(APPEND "${output_file}" "#include \"${simflow_file}\"\n")
            file(APPEND "${output_file}" "#undef GNC_SIMFLOW_MATERIALIZER_REGISTRATION_FN\n\n")
            string(APPEND auto_register_calls "    ::${symbol_prefix}_${simflow_index}(registry);\n")
            math(EXPR simflow_index "${simflow_index} + 1")
        endforeach()
    endif()

    file(APPEND "${output_file}" "#include \"gnc/simflow/simflow_materializer_registry.hpp\"\n\n")
    file(APPEND "${output_file}" "namespace gnc::build {\n")
    file(APPEND "${output_file}" "inline void ${function_name}(::gnc::simflow::SimFlowMaterializerRegistry& registry) {\n")
    file(APPEND "${output_file}" "    (void)registry;\n")
    if(auto_register_calls)
        file(APPEND "${output_file}" "${auto_register_calls}")
    endif()
    file(APPEND "${output_file}" "}\n")
    file(APPEND "${output_file}" "} // namespace gnc::build\n")
endfunction()

set(AUTO_REGISTER_FILE "${GENERATED_DIR}/auto_registered_components.hpp")
generate_component_registration_header(
    "${AUTO_REGISTER_FILE}"
    "registerAutoRegisteredProjectComponents"
    "gnc_register_project_component"
    ${USER_COMPONENT_FILES})

set(AUTO_REGISTER_SIMFLOW_FILE "${GENERATED_DIR}/auto_registered_simflow_materializers.hpp")
generate_simflow_registration_header(
    "${AUTO_REGISTER_SIMFLOW_FILE}"
    "registerAutoRegisteredSimFlowMaterializers"
    "gnc_register_project_simflow_materializer"
    ${USER_SIMFLOW_FILES})

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

add_executable(gnc_sim src/runner.cpp src/runner_component_registration.cpp)
if(MINGW)
    set_source_files_properties(src/runner_component_registration.cpp
        PROPERTIES COMPILE_OPTIONS "-Wa,-mbig-obj")
endif()
target_link_libraries(gnc_sim PRIVATE gnc_framework)
target_include_directories(gnc_sim PRIVATE
    "${GENERATED_DIR}"
    "${CMAKE_SOURCE_DIR}"
)
if(USER_COMPONENTS_DIR)
    target_include_directories(gnc_sim PRIVATE
        "${USER_PROJECT_DIR}"
        "${USER_PROJECT_DIR}/include"
        "${USER_PROJECT_DIR}/interfaces"
        "${USER_PROJECT_DIR}/common"
        "${USER_COMPONENTS_DIR}"
    )
endif()
if(USER_SIMFLOW_DIR)
    target_include_directories(gnc_sim PRIVATE
        "${USER_SIMFLOW_DIR}"
    )
endif()

if(MSVC)
    target_compile_options(gnc_sim PRIVATE /W4 /utf-8)
else()
    target_compile_options(gnc_sim PRIVATE -Wall -Wextra -pedantic)
endif()

option(BUILD_TESTS "Build test programs" OFF)
if(BUILD_TESTS)
    enable_testing()

    set(TEST_PROJECT_COMPONENT_FILES "")
    file(GLOB_RECURSE TEST_PROJECT_COMPONENT_FILES CONFIGURE_DEPENDS
        RELATIVE "${CMAKE_SOURCE_DIR}"
        "tests/project_components/*.hpp"
    )

    set(TEST_AUTO_REGISTER_FILE "${GENERATED_DIR}/auto_registered_test_components.hpp")
    generate_component_registration_header(
        "${TEST_AUTO_REGISTER_FILE}"
        "registerAutoRegisteredTestComponents"
        "gnc_register_test_component"
        ${TEST_PROJECT_COMPONENT_FILES})

    set(TEST_PROJECT_SIMFLOW_FILES "")
    file(GLOB_RECURSE TEST_PROJECT_SIMFLOW_FILES CONFIGURE_DEPENDS
        RELATIVE "${CMAKE_SOURCE_DIR}"
        "tests/project_simflows/*.hpp"
    )

    set(TEST_AUTO_REGISTER_SIMFLOW_FILE
        "${GENERATED_DIR}/auto_registered_test_simflow_materializers.hpp")
    generate_simflow_registration_header(
        "${TEST_AUTO_REGISTER_SIMFLOW_FILE}"
        "registerAutoRegisteredTestSimFlowMaterializers"
        "gnc_register_test_simflow_materializer"
        ${TEST_PROJECT_SIMFLOW_FILES})

    if(MINGW)
        set_source_files_properties(tests/test_support_registration.cpp
            PROPERTIES COMPILE_OPTIONS "-Wa,-mbig-obj")
    endif()

    function(add_gnc_test target source)
        add_executable(${target} ${source} tests/test_support_registration.cpp)
        target_link_libraries(${target} PRIVATE gnc_framework)
        target_include_directories(${target} PRIVATE
            "${CMAKE_SOURCE_DIR}"
            "${GENERATED_DIR}"
        )
        if(MSVC)
            target_compile_options(${target} PRIVATE /W4 /utf-8)
        else()
            target_compile_options(${target} PRIVATE -Wall -Wextra -pedantic)
        endif()
        add_test(NAME ${target} COMMAND ${target})
    endfunction()

    add_gnc_test(test_builtin_bootstrap tests/test_builtin_bootstrap.cpp)
    add_gnc_test(test_coordinate_tree_builder tests/test_coordinate_tree_builder.cpp)
    add_gnc_test(test_environment_packages tests/test_environment_packages.cpp)
    add_gnc_test(test_form_slices tests/test_form_slices.cpp)
    add_gnc_test(test_coordinate_tree_service tests/test_coordinate_tree_service.cpp)
    add_gnc_test(test_mission_contract tests/test_mission_contract.cpp)
    add_gnc_test(test_perturbation_contract tests/test_perturbation_contract.cpp)
    add_gnc_test(test_component_listing tests/test_component_listing.cpp)
    add_gnc_test(test_simflow_materializer tests/test_simflow_materializer.cpp)
    add_gnc_test(test_simflow_runner tests/test_simflow_runner.cpp)
    add_gnc_test(test_architecture_guards tests/test_architecture_guards.cpp)
    add_gnc_test(test_auto_data_logger tests/test_auto_data_logger.cpp)
    add_gnc_test(test_publish_semantics tests/test_publish_semantics.cpp)
    add_gnc_test(test_strict_config tests/test_strict_config.cpp)
    add_gnc_test(test_config_preprocessor tests/test_config_preprocessor.cpp)
    add_gnc_test(test_parser_facade tests/test_parser_facade.cpp)
    add_gnc_test(test_form_extension_contract tests/test_form_extension_contract.cpp)
    add_gnc_test(test_local_spherical_6dof_form tests/test_local_spherical_6dof_form.cpp)
    add_gnc_test(test_ideal_3dof_mission tests/test_ideal_3dof_mission.cpp)
    add_gnc_test(test_ideal_cartesian_3dof_mission tests/test_ideal_cartesian_3dof_mission.cpp)
    add_gnc_test(test_ideal_cartesian_6dof_mission tests/test_ideal_cartesian_6dof_mission.cpp)
    add_gnc_test(test_ideal_6dof_components tests/test_ideal_6dof_components.cpp)
    add_gnc_test(test_ideal_6dof_process tests/test_ideal_6dof_process.cpp)
    add_gnc_test(test_ideal_6dof_mission tests/test_ideal_6dof_mission.cpp)

    list(LENGTH TEST_PROJECT_COMPONENT_FILES TEST_PROJECT_COMPONENT_COUNT)
    message(STATUS
        "Auto-registered stable test project component headers: ${TEST_PROJECT_COMPONENT_COUNT}")
endif()

message(STATUS "GNC Simulation Framework v${PROJECT_VERSION}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "User project components dir: ${USER_COMPONENTS_DIR}")
message(STATUS "Auto-register file: ${AUTO_REGISTER_FILE}")
