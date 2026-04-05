add_library(gnc_framework INTERFACE)

target_include_directories(gnc_framework INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(gnc_framework INTERFACE Eigen3::Eigen)
target_compile_features(gnc_framework INTERFACE cxx_std_17)
