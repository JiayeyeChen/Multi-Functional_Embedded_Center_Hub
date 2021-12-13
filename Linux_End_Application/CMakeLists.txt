cmake_minimum_required(VERSION 3.0.2)
project(Multi-Functional_Embedded_Center)

find_package(Boost REQUIRED COMPONENTS system thread regex)

include_directories(
  Inc
  ${Boost_INCLUDE_DIR}
)

set(SRC
  main.cpp
  mfec_communication.cpp
  crc32_mpeg.cpp
  ak10_9_calibration.cpp
)

set(LIBS
  ${Boost_LIBRARIES}
)

add_executable(MFEC ${SRC})
target_link_libraries(MFEC ${LIBS})