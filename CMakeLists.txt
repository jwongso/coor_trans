# Copyrght: jsaputra@riseup.net
# The reproduction, distribution and utilization of this file as
# well as the communication of its contents to others without express
# authorization is prohibited. Offenders will be held liable for the
# payment of damages and can be prosecuted. All rights reserved
# particularly in the event of the grant of a patent, utility model
# or design.

cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)
project(radar_lib_coordinate_transformation)

find_package(athena_cmake_modules REQUIRED COMPONENTS radar)

athena_internal_dependencies(
  PUBLIC
    radar_interface
    vfc
)

athena_add_library(${PROJECT_NAME}
  LINK_LIBRARIES
    radar_interface
    vfc
)

athena_add_unit_tests(
  LINK_LIBRARIES
    ${PROJECT_NAME}
)

athena_install_export()
