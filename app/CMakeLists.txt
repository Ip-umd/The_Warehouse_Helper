find_package(OpenCV REQUIRED)
add_executable(shell-app main.cpp NodeParam.cpp ObstacleMap.cpp AStar.cpp)
target_link_libraries(shell-app ${OpenCV_LIBS})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
