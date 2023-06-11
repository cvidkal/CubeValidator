cmake_minimum_required(VERSION 3.6)
file (STRINGS "version.txt" LIB_VERSION)
project(template VERSION ${LIB_VERSION})

add_definitions("-DLOG_MODULE_NAME=${CMAKE_PROJECT_NAME}")

set(CMAKE_CXX_STANDARD 14)

if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    conan_basic_setup()
else()
    message(WARNING "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif()

if(CONAN_HOST_ARCH STREQUAL "armv7")
    message("CONAN HOST ARMV7: use -mfpu=neon")
    add_definitions("-mfpu=neon")
endif()

add_executable(test-template
tests/test.cpp
)

target_include_directories(test-template PUBLIC include)

target_link_libraries(test-template
${CONAN_LIBS}
pthread
stdc++fs
)

add_executable(test-multi-board
tests/test-multi-board.cpp
)


target_include_directories(test-multi-board PUBLIC include)
target_link_libraries(test-multi-board
${CONAN_LIBS}
pthread
stdc++fs
)
