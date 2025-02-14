cmake_minimum_required(VERSION 3.0.6)
# 1. 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 2. 输出当前工程的路径
# 使用正则表达式获取${CMAKE_CURRENT_SOURCE_DIR}最后一级目录名，即工程名
string(REGEX REPLACE ".*/(.*)$" "\\1" SUB_PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR})
message("SUB_PROJECT_NAME: ${SUB_PROJECT_NAME}")
# 输出当前cmakelists.txt的路径
message("   CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")



# 3. 添加头文件路径
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/common/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# 打印头文件路径
message("   include_directories:" ${PROJECT_SOURCE_DIR}/common/include)

# 4. 生成动态链接库dijkstra.so和静态链接库dijkstra.a
add_library(${SUB_PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/${SUB_PROJECT_NAME}.cpp  )
add_library(${SUB_PROJECT_NAME}_static STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/${SUB_PROJECT_NAME}.cpp )
set_target_properties(${SUB_PROJECT_NAME}_static PROPERTIES OUTPUT_NAME ${SUB_PROJECT_NAME})

# 5. 将test文件夹下的所有cpp文件放到MOTION_CONTROL_TEST_SRCS全局变量中
file(GLOB TEST_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/test/[^.]*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/[^.]*.cc)
message("   TEST_SRCS:" ${TEST_SRCS})
# 将include文件夹下的所有源文件都放到MOTION_CONTROL_INCLUDE_DIRS全局变量中
file(GLOB INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/[^.]*.h ${CMAKE_CURRENT_SOURCE_DIR}/include/[^.]*.hpp)
# 将src文件夹下的所有源文件都放到MOTION_CONTROL_SRCS全局变量中
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/[^.]*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/[^.]*.cc)

# 6. install
# 将头文件安装到/usr/local/include/${SUB_PROJECT_NAME}目录下
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/ DESTINATION /usr/local/include/${SUB_PROJECT_NAME})
# 将动态链接库安装到/usr/local/lib目录下
install(TARGETS ${SUB_PROJECT_NAME} DESTINATION /usr/local/lib)
# 将静态链接库安装到/usr/local/lib目录下
install(TARGETS ${SUB_PROJECT_NAME}_static DESTINATION /usr/local/lib)

ADD_MODULE_SRC(MOTION_CONTROL_TEST_SRCS ${TEST_SRCS})
ADD_MODULE_SRC(MOTION_CONTROL_SRCS ${SRCS})
ADD_MODULE_SRC(MOTION_CONTROL_INCLUDE_DIRS ${INCLUDE_DIRS})
ADD_MODULE_SRC(MOTION_CONTROL_DEPEND_LIBS ${SUB_PROJECT_NAME})