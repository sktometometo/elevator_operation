cmake_minimum_required(VERSION 3.0.2)
project(elevator_operation)

find_package(catkin REQUIRED genmsg actionlib_msgs)

add_service_files(DIRECTORY srv FILES LookAtTarget.srv)

add_action_files(DIRECTORY action FILES MoveElevator.action)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package()
