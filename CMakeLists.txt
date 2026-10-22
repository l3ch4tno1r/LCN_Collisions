set(THIS LCNCollisionLib)

add_library(${THIS} INTERFACE)

target_link_libraries(${THIS}
INTERFACE
    LCNMathLib
    UtilitiesLib)

target_include_directories(
    ${THIS}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)