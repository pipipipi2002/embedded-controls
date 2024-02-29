cmake_minimum_required(VERSION 3.12)

project(embedded-controls-lib-bno08x-shtp C CXX ASM )

#### BNO08X LIBARY ####
set(BNO08X_LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src )
set(BNO08X_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc )
set(BNO08X_LIB_SOURCE_FILES 
    ${BNO08X_LIB_SOURCE_DIR}/sh2_SensorValue.c
    ${BNO08X_LIB_SOURCE_DIR}/sh2_util.c
    ${BNO08X_LIB_SOURCE_DIR}/sh2.c
    ${BNO08X_LIB_SOURCE_DIR}/shtp.c
    ${BNO08X_LIB_SOURCE_DIR}/bno08x.c
    )

add_library(${PROJECT_NAME} INTERFACE )
target_sources(${PROJECT_NAME} INTERFACE 
    ${BNO08X_LIB_SOURCE_FILES}
    )
target_include_directories(${PROJECT_NAME} INTERFACE 
    ${BNO08X_LIB_INCLUDE_DIR} 
    )