
set(${SUBM_LIB_NAME}_MOC_HEADERS
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QRosNode.hpp
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QRosNodeSingleton.hpp
)

CUSTOM_QT_WRAP_CPP(${SUBM_LIB_NAME}_MOC_SRC_H
    ${${SUBM_LIB_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_library(${SUBM_LIB_NAME} SHARED
    # MOCs 
    ${${SUBM_LIB_NAME}_MOC_SRC_H}

    # QT
    qt/QRosNode.cpp
    qt/QRosNodeSingleton.cpp
    
)

target_link_libraries(${SUBM_LIB_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
)
