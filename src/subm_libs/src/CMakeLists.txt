
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qt)

set(${SUBM_LIB_NAME}_MOC_HEADERS
    ## public classes
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QRosNode.hpp
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QRosNodeSingleton.hpp
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QSignalHandler.hpp
    ${INCLUDE_ROOT_PATH}/subm_libs/qt/QStateMachine.hpp
    
    ## private classes
    ${CMAKE_CURRENT_SOURCE_DIR}/qt/priv/QSignalHandlerPrivate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qt/priv/QStateMachinePrivate.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/qt/priv/QStringTransition.hpp
)

CUSTOM_QT_WRAP_CPP(${SUBM_LIB_NAME}_MOC_SRC_H
    ${${SUBM_LIB_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_library(${SUBM_LIB_NAME} SHARED
    # MOCs 
    ${${SUBM_LIB_NAME}_MOC_SRC_H}

    # private QT
    qt/priv/QSignalHandlerPrivate.cpp
    qt/priv/QStateMachinePrivate.cpp

    # public QT
    qt/QRosNode.cpp
    qt/QRosNodeSingleton.cpp
    qt/QSignalHandler.cpp
    qt/QStateMachine.cpp
    
)

target_link_libraries(${SUBM_LIB_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
)
