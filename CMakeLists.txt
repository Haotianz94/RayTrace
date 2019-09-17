CMAKE_MINIMUM_REQUIRED( VERSION 2.8.7 )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O3 -fopenmp")
PROJECT(raytrace)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${raytrace_SOURCE_DIR}/bin)

SET(PROJECT_HEADER
        ${PROJECT_SOURCE_DIR}/include/canvas.h
        ${PROJECT_SOURCE_DIR}/include/foundation.h
        ${PROJECT_SOURCE_DIR}/include/object.h
        ${PROJECT_SOURCE_DIR}/include/kdtree.h)

SET(PROJECT_SRC
        ${PROJECT_SOURCE_DIR}/src/canvas.cpp
        ${PROJECT_SOURCE_DIR}/src/kdtree.cpp
        ${PROJECT_SOURCE_DIR}/src/object.cpp
        ${PROJECT_SOURCE_DIR}/src/main.cpp)

INCLUDE_DIRECTORIES(include)

ADD_EXECUTABLE(raytrace ${PROJECT_HEADER} ${PROJECT_SRC})

# opencv
FIND_PACKAGE(OpenCV REQUIRED)
TARGET_LINK_LIBRARIES(raytrace ${OpenCV_LIBS})