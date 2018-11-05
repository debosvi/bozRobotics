
set(NODELET_NAME MyNodeletClass)

## Declare a C++ library
add_library(${NODELET_NAME} SHARED
    example_pkg/MyNodeletClass.cpp
)

target_link_libraries(${NODELET_NAME}
    ${catkin_LIBRARIES}
#     ${CUSTOM_QT_LIBRARIES}
)
