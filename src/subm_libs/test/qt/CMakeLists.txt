
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
