cmake_minimum_required(VERSION 3.20)

project(planegcs_native LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(PLANEGCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../planegcs" CACHE PATH
    "Directory containing GCS.cpp, Geo.cpp, Constraints.cpp, SubSystem.cpp and qp_eq.cpp")
set(PLANEGCS_EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/eigen" CACHE PATH
    "Directory containing the Eigen headers (Eigen/Dense)")

if(NOT EXISTS "${PLANEGCS_SOURCE_DIR}/GCS.cpp")
    message(FATAL_ERROR "PlaneGCS sources were not found in '${PLANEGCS_SOURCE_DIR}'.")
endif()

if(NOT EXISTS "${PLANEGCS_EIGEN_DIR}/Eigen/Dense")
    message(FATAL_ERROR "Eigen was not found in '${PLANEGCS_EIGEN_DIR}'. Run native/build.ps1 or download Eigen 3.4.0 manually.")
endif()

set(PGCS_OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/out")

add_library(planegcs_native SHARED
    src/planegcs_native.cpp
    "${PLANEGCS_SOURCE_DIR}/GCS.cpp"
    "${PLANEGCS_SOURCE_DIR}/Geo.cpp"
    "${PLANEGCS_SOURCE_DIR}/Constraints.cpp"
    "${PLANEGCS_SOURCE_DIR}/SubSystem.cpp"
    "${PLANEGCS_SOURCE_DIR}/qp_eq.cpp"
)

target_include_directories(planegcs_native PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${PLANEGCS_SOURCE_DIR}"
    "${PLANEGCS_SOURCE_DIR}/headers"
    "${PLANEGCS_EIGEN_DIR}"
)

target_compile_definitions(planegcs_native PRIVATE PLANEGCS_NATIVE_EXPORTS)

option(PLANEGCS_NATIVE_VERBOSE "Write PlaneGCS diagnostic messages to stderr" OFF)

if(PLANEGCS_NATIVE_VERBOSE)
    target_compile_definitions(planegcs_native PRIVATE PLANEGCS_NATIVE_VERBOSE)
endif()

if(MSVC)
    target_compile_options(planegcs_native PRIVATE /W3 /EHsc /utf-8 /bigobj
        "/FI${CMAKE_CURRENT_SOURCE_DIR}/include/planegcs_prelude.h")
    target_compile_definitions(planegcs_native PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
else()
    target_compile_options(planegcs_native PRIVATE -Wall -Wextra)
endif()

set_target_properties(planegcs_native PROPERTIES
    OUTPUT_NAME "planegcs_native"
    RUNTIME_OUTPUT_DIRECTORY "${PGCS_OUT_DIR}/$<CONFIG>"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PGCS_OUT_DIR}/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PGCS_OUT_DIR}/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PGCS_OUT_DIR}/RelWithDebInfo"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PGCS_OUT_DIR}/MinSizeRel"
    ARCHIVE_OUTPUT_DIRECTORY "${PGCS_OUT_DIR}/$<CONFIG>"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${PGCS_OUT_DIR}/Debug"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PGCS_OUT_DIR}/Release"
    ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PGCS_OUT_DIR}/RelWithDebInfo"
    ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL "${PGCS_OUT_DIR}/MinSizeRel"
)

option(PLANEGCS_BUILD_SMOKE "Build the native smoke test" ON)

if(PLANEGCS_BUILD_SMOKE)
    add_executable(planegcs_native_smoke smoke/smoke_test.cpp)
    target_link_libraries(planegcs_native_smoke PRIVATE planegcs_native)
    target_include_directories(planegcs_native_smoke PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

    set_target_properties(planegcs_native_smoke PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PGCS_OUT_DIR}/$<CONFIG>"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PGCS_OUT_DIR}/Debug"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PGCS_OUT_DIR}/Release"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PGCS_OUT_DIR}/RelWithDebInfo"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PGCS_OUT_DIR}/MinSizeRel"
    )
endif()
