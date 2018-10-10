
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
set(SUBS_NAME testQRosPublisher)

set(${SUBS_NAME}_MOC_HEADERS
    testQRosPublisher.hpp
)

CUSTOM_QT_WRAP_CPP(${SUBS_NAME}_MOC_SRC_H
    ${${SUBS_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${SUBS_NAME}
    # MOCs 
    ${${SUBS_NAME}_MOC_SRC_H}

    # QT
    testQRosPublisher.cpp
)

target_link_libraries(${SUBS_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
    ${SUBM_LIB_NAME}
)
