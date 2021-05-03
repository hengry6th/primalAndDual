cmake_minimum_required(VERSION 3.1)
project(PrimalDual)



file(GLOB SRCFILES
        "./*.cpp"
        "./*.h"
        )
add_executable(PrimalDual ${SRCFILES})