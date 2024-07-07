set(THIS LCNCollisionsLib)

add_library(${THIS} INTERFACE)

target_include_directories(
    ${THIS}
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(${THIS} INTERFACE ${LCN_MATH_DIR})