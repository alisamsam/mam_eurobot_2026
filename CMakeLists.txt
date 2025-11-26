cmake_minimum_required(VERSION 3.8)
project(mam_eurobot_2026)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dépendances
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# C++ (si tu as des headers)
include_directories(include)

# Exécutable C++
add_executable(cpp_test src/cpp_test.cpp)
ament_target_dependencies(cpp_test rclcpp geometry_msgs)
install(TARGETS cpp_test DESTINATION lib/${PROJECT_NAME})

# Launch / worlds / models
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Index ament + package.xml
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME} DESTINATION share/ament_index/resource_index/packages)

# Python : module + exécutables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/py_test.py
  ${PROJECT_NAME}/key_ctrl.py
  ${PROJECT_NAME}/half_spin.py
  ${PROJECT_NAME}/open_cv_cam.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
