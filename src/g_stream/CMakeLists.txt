cmake_minimum_required(VERSION 3.8)
project(g_stream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


ament_python_install_package(${PROJECT_NAME})

set(NODES
  g_stream/stream_node.py
)

set(APP_FILES
  g_stream/bfid.py
  g_stream/minimal_pipe.py
)

install(
  FILES
    ${APP_FILES}
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_param_load tests/test_dynamic_params.py)
endif()

if(DEFINED INCLUDE_CPACK)
  include(${INCLUDE_CPACK})
endif()

ament_package()
