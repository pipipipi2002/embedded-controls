cmake_minimum_required(VERSION 3.12)

project(embedded-controls-lib C CXX ASM )

add_subdirectory(BNO08x-shtp)

#### PRINTF LIBARY ####
set(PRINTF_LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/printf )
set(PRINTF_LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/printf )
set(PRINTF_LIB_SOURCE_FILES ${PRINTF_LIB_SOURCE_DIR}/printf.c )

add_library(${PROJECT_NAME} INTERFACE )
target_sources(${PROJECT_NAME} INTERFACE 
    ${PRINTF_LIB_SOURCE_FILES}
    )
target_include_directories(${PROJECT_NAME} INTERFACE 
    ${PRINTF_LIB_INCLUDE_DIR} 
    )