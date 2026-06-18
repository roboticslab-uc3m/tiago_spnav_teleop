cmake_minimum_required(VERSION 3.16)

project(tiago_spnav_teleop LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ros2_control_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)

generate_parameter_library(${PROJECT_NAME}_parameters
                           src/${PROJECT_NAME}_parameters.yaml)

add_library(${PROJECT_NAME} src/tiago_spnav_teleop.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
                           include
                           ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC
                      ${PROJECT_NAME}_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      kdl_parser::kdl_parser
                      urdf::urdf
                      ${orocos_kdl_LIBRARIES})

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}_plugin.xml)

install(TARGETS ${PROJECT_NAME}
                ${PROJECT_NAME}_parameters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/init_spnav.py
                 scripts/stop_spnav.py
        DESTINATION bin)

install(FILES tiago_spnav_teleop_plugin.xml
        DESTINATION share/${PROJECT_NAME})

ament_package()
