cmake_minimum_required(VERSION 2.8.3)
project(jedi)
find_package(catkin REQUIRED COMPONENTS)
catkin_package()

foreach(dir launch Media models scripts worlds)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(PROGRAMS scripts/tuck_arm.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
