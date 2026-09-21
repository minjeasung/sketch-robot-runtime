cmake_minimum_required(VERSION 3.5)
project(rbpodo_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the importable Python module so other code can do
#   from rbpodo_bringup.admittance_reset import AdmittanceResetNode
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Thin entrypoint scripts (use the installed Python modules above).
install(
  PROGRAMS
    scripts/admittance_reset_node.py
    scripts/ft_tare_node.py
    scripts/variable_impedance_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # This package consistently uses double-quoted Python strings.  Keep the
  # remaining ROS lint rules while avoiding a repository-wide quote rewrite.
  set(ament_cmake_flake8_CONFIG_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_ft_tare_client
    test/test_ft_tare_client.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}
  )
endif()

ament_package()
