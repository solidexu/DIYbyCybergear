cmake_minimum_required(VERSION 3.0.6)
project(planning)

MACRO( ADD_MODULE_SRC MODULE_VAR)
    SET(ENV{${MODULE_VAR}} "$ENV{${MODULE_VAR}}${ARGN};")
ENDMACRO( ADD_MODULE_SRC MODULE_VAR)

option(ENABLE_TESTS "Enable tests" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
if(CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE STREQUAL "Debug")) 
  add_definitions(-DDEBUG)
endif()

if (${ENABLE_TESTS})
    
endif()

set(ENV{PLANNING_INCLUDE_DIRS})
set(ENV{PLANNING_SRCS})
set(ENV{PLANNING_TEST_SRCS})
set(ENV{PLANNING_DEPEND_LIBS})
add_subdirectory(common)
add_subdirectory(dijkstra)
add_subdirectory(a_star)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} $ENV{PLANNING_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/common/include)

# 打印 PLANNING_INCLUDE_DIRS
message(STATUS "PLANNING_INCLUDE_DIRS: $ENV{PLANNING_INCLUDE_DIRS}")
# 打印 PLANNING_SRCS
message(STATUS "PLANNING_SRCS: $ENV{PLANNING_SRCS}")
# 打印ENABLE_TESTS当前状态
message(STATUS "ENABLE_TESTS: ${ENABLE_TESTS}")
# 打印PLANNING_TEST_SRCS当前状态
message(STATUS "PLANNING_TEST_SRCS: $ENV{PLANNING_TEST_SRCS}")
if (${ENABLE_TESTS})
    add_definitions(-DENABLE_TESTING)
    find_package(GTest REQUIRED)

    add_definitions(-DENABLE_VISUAL_DEBUG)
    message(STATUS "ENABLE_VISUAL_DEBUG")
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage -lgcov")
    foreach(test_file $ENV{PLANNING_TEST_SRCS})
        STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${test_file})
        message(STATUS "test_file: ${FILE_NAME}")
        add_executable(${FILE_NAME} ${test_file} $ENV{PLANNING_SRCS})
        target_link_libraries(${FILE_NAME}  $ENV{PLANNING_DEPEND_LIBS} ${GTEST_BOTH_LIBRARIES} ${OpenCV_LIBS})
        set_target_properties(${FILE_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
    endforeach()
endif()