cmake_minimum_required(VERSION 3.7.1)

# This is your project name
set(TARGET_NAME simplecontroller)

# This is the name of the driver according to SteamVR
set(DRIVER_NAME "driver_${TARGET_NAME}")

project(${TARGET_NAME})
set(OPENVR_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

include(ExternalProject)

ExternalProject_Add(
    libjoycon_proj
    GIT_REPOSITORY  https://github.com/Inokinoki/libjoycon
    GIT_TAG         main
    BUILD_COMMAND   make
    INSTALL_COMMAND ""
)

ExternalProject_Get_property(libjoycon_proj SOURCE_DIR)
set(libjoycon_SOURCE_DIR ${SOURCE_DIR})

ExternalProject_Get_property(libjoycon_proj BINARY_DIR)
set(libjoycon_BUILD_DIR ${BINARY_DIR})


# If not set, determine the platform architecture
if (NOT PLATFORM)
    if (CMAKE_SIZEOF_VOID_P MATCHES 8)
        set(PLATFORM 64)
    else ()
        set(PLATFORM 32)
    endif ()
endif ()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")

# OpenVR compatibility checking
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_definitions(-DLINUX -DPOSIX)
    set(ARCH_TARGET linux64)

    if (${PLATFORM} MATCHES 32)
        message(WARNING "OpenVR x86 binaries not provided on GNU/Linux.")
    endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(CMAKE_MACOSX_RPATH 0)
    add_definitions(-DOSX -DPOSIX)
    set(ARCH_TARGET osx32)

elseif (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_definitions(-D_WIN32)
    set(ARCH_TARGET win${PLATFORM})

    # Binaries path for thirdparties are not generics so we try to guess their suffixes.
    set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})
endif ()

add_library(libjoycon STATIC IMPORTED)

set_target_properties(libjoycon PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_property(TARGET libjoycon PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/src/libjoycon.so)
add_dependencies(libjoycon libjoycon_proj)

find_library(libjoycon_LIBRARY NAMES joycon PATHS ${libjoycon_BUILD_DIR}/src)

find_library(OPENVR_LIBRARIES
        NAMES
        openvr_api
        PATHS
        ${OPENVR_LIB_DIR}/lib/openvr
        ${OPENVR_LIB_DIR}/bin/openvr
        PATH_SUFFIXES
        osx${PLATFORM}
        linux${PLATFORM}
        win${PLATFORM}
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
        HINTS "${CMAKE_SOURCE_DIR}/lib/"
        )

set(OPENVR_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/headers/")

add_library(util_driverlog STATIC
    utils/driverlog/driverlog.cpp
    # ... other source files for util_driverlog
)

set_target_properties(util_driverlog PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Add the include directory for util_driverlog:
target_include_directories(util_driverlog PUBLIC  # Or PRIVATE if only used by util_driverlog
    ${CMAKE_SOURCE_DIR}/headers # Or the correct path if different
    ${libjoycon_SOURCE_DIR}/include
    ${libjoycon_SOURCE_DIR}/examples
)

add_subdirectory(utils/vrmath)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}/bin/${ARCH_TARGET}>)

file(GLOB_RECURSE DRIVER_SOURCES "src/*.cpp" "src/*.h")

# Set rpath to the binary's directory (relative path)
set(CMAKE_BUILD_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(${DRIVER_NAME} SHARED ${DRIVER_SOURCES})


message(STATUS "Lib: ${libjoycon_LIBRARY}")

find_package(PkgConfig REQUIRED)

if(PkgConfig_FOUND)
    pkg_check_modules(HIDAPI_HIDRAW REQUIRED hidapi-hidraw) # Correct package name
endif()

pkg_check_modules(HIDAPI_HIDRAW REQUIRED hidapi-hidraw)

if(HIDAPI_HIDRAW_FOUND)
    message(STATUS "HIDAPI (hidraw) found via pkg-config: ${HIDAPI_HIDRAW_INCLUDE_DIRS}") # Correct variable
    target_include_directories(${DRIVER_NAME} PUBLIC ${HIDAPI_HIDRAW_INCLUDE_DIRS}) # Correct variable
    target_link_libraries(${DRIVER_NAME} PUBLIC ${HIDAPI_HIDRAW_LIBRARIES}) # Correct variable
else()
    message(FATAL_ERROR "HIDAPI (hidraw) not found. Please install libhidapi-dev (or the equivalent for your distribution).")
endif()


message(STATUS "Lib:  ${HIDAPI_LIBRARY_DIRS}")

add_dependencies(${DRIVER_NAME} libjoycon)

# This is so we can build directly to "<binary_dir>/<target_name>/<platform>/<arch>/<driver_name>.<dll/so>"
set_target_properties(${DRIVER_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${ARCH_TARGET}>)

target_link_libraries(${DRIVER_NAME} PRIVATE ${OPENVR_LIBRARIES} util_driverlog util_vrmath ${libjoycon_BUILD_DIR}/src/libjoycon.so)
target_include_directories(${DRIVER_NAME} PRIVATE ${OPENVR_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/utils/driverlog ${CMAKE_SOURCE_DIR}/utils/vrmath ${CMAKE_SOURCE_DIR}/src libjoycon)



# Set the output name of the driver to "driver_simplecontroller" instead of "libdriver_simplecontroller"
set_target_properties(${DRIVER_NAME} PROPERTIES PREFIX "")
add_custom_command(
    TARGET ${DRIVER_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    ${libjoycon_BUILD_DIR}/src/libjoycon.so
    $<TARGET_FILE_DIR:${DRIVER_NAME}>/${TARGET_NAME}/bin/${ARCH_TARGET}/lib/libjoycon.so
)

add_custom_command(
    TARGET ${DRIVER_NAME}
    PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}
    $<TARGET_FILE_DIR:${DRIVER_NAME}>/${TARGET_NAME}
)

add_custom_command(
    TARGET ${DRIVER_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:${DRIVER_NAME}>
    $<TARGET_FILE_DIR:${DRIVER_NAME}>/${TARGET_NAME}/bin/${ARCH_TARGET}/$<TARGET_FILE_NAME:${DRIVER_NAME}>
)
