cmake_minimum_required(VERSION 2.8.3)
project(sl_sensor_rqt_guis_common)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )

install(DIRECTORY rqt_perspectives/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqt_perspectives
    )
