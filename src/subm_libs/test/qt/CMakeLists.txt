
###############################################################################
set(SUBS_NAME testQRosSubscriber)

set(${SUBS_NAME}_MOC_HEADERS
    testQRosSubscriber.hpp
)

CUSTOM_QT_WRAP_CPP(${SUBS_NAME}_MOC_SRC_H
    ${${SUBS_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${SUBS_NAME}
    # MOCs 
    ${${SUBS_NAME}_MOC_SRC_H}

    # QT
    testQRosSubscriber.cpp
)

target_link_libraries(${SUBS_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)


###############################################################################
set(PUBL_NAME testQRosPublisher)

set(${PUBL_NAME}_MOC_HEADERS
    testQRosPublisher.hpp
)

CUSTOM_QT_WRAP_CPP(${PUBL_NAME}_MOC_SRC_H
    ${${PUBL_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${PUBL_NAME}
    # MOCs 
    ${${PUBL_NAME}_MOC_SRC_H}

    # QT
    testQRosPublisher.cpp
)

target_link_libraries(${PUBL_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)


###############################################################################
set(QSUB_NAME testQRosNode)

set(${QSUB_NAME}_MOC_HEADERS
    testQRosSubscriber.hpp
)

CUSTOM_QT_WRAP_CPP(${QSUB_NAME}_MOC_SRC_H
    ${${QSUB_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${QSUB_NAME}
    # MOCs 
    ${${QSUB_NAME}_MOC_SRC_H}

    # QT
    testQRosNode.cpp
)

target_link_libraries(${QSUB_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)


###############################################################################
set(QSUBSING_NAME testQRosNodeSingleton)

set(${QSUBSING_NAME}_MOC_HEADERS
    testQRosSubscriber.hpp
)

CUSTOM_QT_WRAP_CPP(${QSUBSING_NAME}_MOC_SRC_H
    ${${QSUBSING_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${QSUBSING_NAME}
    # MOCs 
    ${${QSUBSING_NAME}_MOC_SRC_H}

    # QT
    testQRosNodeSingleton.cpp
)

target_link_libraries(${QSUBSING_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)


###############################################################################
set(QSH_NAME testQSignalHandler)

set(${QSH_NAME}_MOC_HEADERS
#     testQRosSubscriber.hpp
)

CUSTOM_QT_WRAP_CPP(${QSH_NAME}_MOC_SRC_H
    ${${QSH_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${QSH_NAME}
    # MOCs 
#     ${${QSH_NAME}_MOC_SRC_H}

    # QT
    testQSignalHandler.cpp
)

target_link_libraries(${QSH_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)


###############################################################################
set(QSTM_NAME testQStateMachine)

set(${QSTM_NAME}_MOC_HEADERS
#     testQRosSubscriber.hpp
)

CUSTOM_QT_WRAP_CPP(${QSTM_NAME}_MOC_SRC_H
    ${${QSTM_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${QSTM_NAME}
    # MOCs 
#     ${${QSTM_NAME}_MOC_SRC_H}

    # QT
    testQStateMachine.cpp
)

target_link_libraries(${QSTM_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)
