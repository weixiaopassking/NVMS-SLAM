
add_library(${PROJECT_NAME}.lio
        loam_tbb/lidarodom.cpp
        loam_tbb_ic/lio_utils.cpp
        loam_tbb_ic/lidarFactor.cpp
        loam_tbb_ic/lidarodom.cpp               #       在上面基础上，增加了地图管理LRU cache(least recently used cache)
        loam_tbb_ic_normal/lidarodom.cpp        #       增加发向量
        )

target_link_libraries(${PROJECT_NAME}.lio
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

install(TARGETS  ${PROJECT_NAME}.lio
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
