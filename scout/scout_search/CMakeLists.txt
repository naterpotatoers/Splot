cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(scout_search)

add_executable(scout_search
    scout_search.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(scout_search
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(scout_search PRIVATE -Wall -Wextra)
else()
    add_compile_options(scout_search PRIVATE -WX -W2)
endif()
