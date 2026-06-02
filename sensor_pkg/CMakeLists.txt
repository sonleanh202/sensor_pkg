cmake_minimum_required(VERSION 3.8)
project(sensor_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

add_library(sensor_core
  src/nodes/generic_modbus_sensor_node.cpp
  src/drivers/modbus_rtu_client.cpp
)

target_include_directories(sensor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

target_link_directories(sensor_core PUBLIC
  ${MODBUS_LIBRARY_DIRS}
)

target_link_libraries(sensor_core
  ${MODBUS_LIBRARIES}
)

ament_target_dependencies(sensor_core
  rclcpp
  interfaces
)

function(add_sensor_executable target_name)
  add_executable(${target_name}
    src/nodes/${target_name}.cpp
  )

  target_link_libraries(${target_name}
    sensor_core
  )

  ament_target_dependencies(${target_name}
    rclcpp
    interfaces
  )

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_sensor_executable(co_sensor_node)
add_sensor_executable(co2_sensor_node)
add_sensor_executable(ch4_sensor_node)
add_sensor_executable(o2_sensor_node)
add_sensor_executable(smoke_fire_alarm_node)
add_sensor_executable(humidity_sensor_node)

add_executable(h2s_sensor_node
  src/nodes/h2s_sensor_node.cpp
  src/drivers/sen0467_uart_rs485_client.cpp
)

target_include_directories(h2s_sensor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(h2s_sensor_node
  rclcpp
  interfaces
)

install(TARGETS h2s_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sensor_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()