set(THIS LCNCollisionsUT)
set(CMAKE_CXX_STANDARD 20)

find_package(GTest REQUIRED)

add_executable(${THIS}
    src/UT01_Shapes.cpp
    src/UT02_DetectCollision.cpp)

target_link_libraries(${THIS} PRIVATE
    GTest::gtest
    GTest::gtest_main
    LCNCollisionsLib)