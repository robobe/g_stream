cmake_minimum_required(VERSION 3.8)
project(g_stream_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(PLUGIN
  g_stream_control/rqt_plugin.py
)


install(PROGRAMS
    ${PLUGIN}
DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  resource
DESTINATION share/${PROJECT_NAME}
)


if(DEFINED INCLUDE_CPACK)
  include(${INCLUDE_CPACK})
endif()

ament_package()
