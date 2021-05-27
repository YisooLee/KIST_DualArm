project(SOEM
    DESCRIPTION "Simple Open EtherCAT Master"
    VERSION 1.4.0
    LANGUAGES CXX)
set(SOURCES main.cpp)
add_executable(main ${SOURCES})

find_package (Eigen3 3.3 REQUIRED)

TARGET_LINK_LIBRARIES(main m)
install(TARGETS main DESTINATION bin)

include_directories(
include
)

add_library(dualarm_lib
  get_home.cpp
)

target_link_libraries(main soem
dualarm_lib
)
