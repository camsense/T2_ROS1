INCLUDE_DIRECTORIES(.)

SET(LIDAR_SRC hchead.cpp hclidar.cpp lidar.cpp ReadParsePackage.cpp HC_serial.cpp HcSDK.cpp fltline.h)
ADD_LIBRARY(lidar STATIC ${LIDAR_SRC})
IF (WIN32)
ELSE()
    TARGET_LINK_LIBRARIES(lidar pthread)
ENDIF()
