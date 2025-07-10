cmake_minimum_required(VERSION 3.5)
project(hobot_arm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ai_msgs REQUIRED) 
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)  # 添加这一行

# 确保WiringPi库存在
if(NOT WIRINGPI_LIBRARIES)
  message(FATAL_ERROR "WiringPi library not found")
endif()

include_directories(include
${PROJECT_SOURCE_DIR}
${nlohmann_json_INCLUDE_DIRS}  # 添加这一行
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/arm_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "ai_msgs"
  "sensor_msgs"
)

# 链接WiringPi库
target_link_libraries(${PROJECT_NAME} ${nlohmann_json_LIBRARIES})  # 添加这一行
target_link_libraries(${PROJECT_NAME} ${WIRINGPI_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
