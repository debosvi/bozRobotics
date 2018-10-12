
set(${SUBM_RUDDER_APP_NAME}_MOC_HEADERS
    test.h    
)

CUSTOM_QT_WRAP_CPP(${SUBM_RUDDER_APP_NAME}_MOC_SRC_H
    ${${SUBM_RUDDER_APP_NAME}_MOC_HEADERS}
)

## Declare a C++ library
add_executable(${SUBM_RUDDER_APP_NAME}
    # MOCs 
    ${${SUBM_RUDDER_APP_NAME}_MOC_SRC_H}

    # QT
    test.cpp    
    main.cpp    
)

target_link_libraries(${SUBM_RUDDER_APP_NAME}
    ${catkin_LIBRARIES}
    ${CUSTOM_QT_LIBRARIES}
)
