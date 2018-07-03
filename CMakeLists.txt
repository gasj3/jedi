cmake_minimum_required(VERSION 2.8.3)
project(jedi)
find_package(catkin REQUIRED COMPONENTS
message_generation
)

add_message_files(
  FILES
  PersonDistance.msg
)
generate_messages(
  DEPENDENCIES
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
)



foreach(dir launch Media models scripts worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS scripts/tuck_arm.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
