cmake_minimum_required(VERSION 3.8)
project(touchless_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- Install Python package ---
# If you’re using 'scripts/' as a plain folder, NOT a Python package, do NOT use this.
# Instead, use install(PROGRAMS ...) below.
ament_python_install_package(scripts)

# --- Install Python executables (nodes) ---
install(PROGRAMS
  scripts/touchlessNav.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/map_process.py
    scripts/handclick.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch and any other resources ---
install(
  DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# --- Optional: lint testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
