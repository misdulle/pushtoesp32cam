cmake_minimum_required(VERSION 3.5)
project(indi-pushto C CXX)

find_library(INDI_DRIVER_LIB indidriver)
find_path(INDI_INCLUDE_DIR inditelescope.h PATH_SUFFIXES libindi indilib indi)

if(NOT INDI_DRIVER_LIB OR NOT INDI_INCLUDE_DIR)
    message(FATAL_ERROR "libindi development files not found (need libindidriver and inditelescope.h): sudo apt install libindi-dev")
endif()

add_executable(indi_push_to_circle indi_push_to_circle.cpp)
target_include_directories(indi_push_to_circle PRIVATE ${INDI_INCLUDE_DIR})
target_compile_features(indi_push_to_circle PRIVATE cxx_std_17)
target_compile_options(indi_push_to_circle PRIVATE -Wall -Wextra)

find_package(Threads REQUIRED)
target_link_libraries(indi_push_to_circle PRIVATE ${INDI_DRIVER_LIB} Threads::Threads m)

install(TARGETS indi_push_to_circle RUNTIME DESTINATION bin)
