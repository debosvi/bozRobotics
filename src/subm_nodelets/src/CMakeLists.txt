
###############################################################################
set(NODELET_NAME RudderNodelet)

## Declare a C++ library
add_library(${NODELET_NAME} SHARED
    equipments/RudderNodelet.cpp
)

target_link_libraries(${NODELET_NAME}
    ${catkin_LIBRARIES}
#     ${CUSTOM_QT_LIBRARIES}
)
add_dependencies(${NODELET_NAME} subm_nodelets_gencpp)
