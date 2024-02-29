cmake_minimum_required(VERSION 3.12)

project(embedded-controls-lib-bno08x-ll C CXX ASM )

#### BNO08X LIBARY ####
set(BNO08X_LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
set(BNO08X_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR} )
set(BNO08X_LIB_SOURCE_FILES 
    ${BNO08X_LIB_SOURCE_DIR}/BNO08X.c
    )

add_library(${PROJECT_NAME} INTERFACE )
target_sources(${PROJECT_NAME} INTERFACE 
    ${BNO08X_LIB_SOURCE_FILES}
    )
target_include_directories(${PROJECT_NAME} INTERFACE 
    ${BNO08X_LIB_INCLUDE_DIR} 
    )