cmake_minimum_required(VERSION 3.7)
project(rvo_test)
enable_language(ASM)

#from github.com/zsummer

# plat : WIN32 APPLE UNIX    (UNIX contain UNIX like)

# CMAKE_CXX_COMPILER_ID:  GNU Intel Clang AppleClang MSVC 
# example IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang") ENDIF()

# jump compiler works check
# if(WIN32)
#    set(CMAKE_C_COMPILER_WORKS TRUE)
#    set(CMAKE_CXX_COMPILER_WORKS TRUE)
# endif(WIN32)

# jump this project build when msvc 
# set_target_properties(${PROJECT_NAME} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

# set this project setup build when msvc 
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# show msvc folder
#  SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON) 

# use C++ 14
# set(CMAKE_CXX_FLAGS -std=c++14) 

# 
# CMAKE_SOURCE_DIR   cmake root dir 
# CMAKE_CURRENT_SOURCE_DIR current cmakelist.txt dir  
# EXECUTABLE_OUTPUT_PATH can set it change bin out dir
# CMAKE_MODULE_PATH can set it change module dir 
# PROJECT_NAME cur project name 

# include 
# include_directories  
# link_directories 
# link_libraries 

# 
# execute_process

option(USE_RVO_2_0 "" ON)




#分组添加到IDE
macro(GROUP_SRC_BY_DIR FILES)
    foreach(PATH_NAME ${${FILES}})
        string(REGEX REPLACE ${CMAKE_CURRENT_SOURCE_DIR}/\(.*\) \\1 RELATIVE_PATH ${PATH_NAME})
        string(REGEX MATCH "(.+)/.*" HAS_GROUP ${RELATIVE_PATH})
        if(HAS_GROUP)
                string(REGEX REPLACE "(.+)/.*" \\1 GROUP_NAME ${RELATIVE_PATH})
                string(REPLACE "/" "\\" GROUP_NAME ${GROUP_NAME})
                message(${PATH_NAME} " into " ${GROUP_NAME})
                source_group(${GROUP_NAME} FILES ${PATH_NAME})
        else()
                source_group("" FILES ${PATH_NAME})
        endif()
    endforeach(PATH_NAME)
endmacro(GROUP_SRC_BY_DIR)


FILE(GLOB_RECURSE GLFW_SOURCE ${CMAKE_SOURCE_DIR}/glfw/*.h ${CMAKE_SOURCE_DIR}/glfw/*.hpp ${CMAKE_SOURCE_DIR}/glfw/*.c ${CMAKE_SOURCE_DIR}/glfw/*.cpp ${CMAKE_SOURCE_DIR}/glfw/*.S)
FILE(GLOB_RECURSE COMMON_SOURCE ${CMAKE_SOURCE_DIR}/common/*.h ${CMAKE_SOURCE_DIR}/common/*.hpp ${CMAKE_SOURCE_DIR}/common/*.c ${CMAKE_SOURCE_DIR}/common/*.cpp ${CMAKE_SOURCE_DIR}/common/*.S)
FILE(GLOB_RECURSE RVO_1_SOURCE ${CMAKE_SOURCE_DIR}/rvo_1_0/*.h ${CMAKE_SOURCE_DIR}/rvo_1_0/*.hpp ${CMAKE_SOURCE_DIR}/rvo_1_0/*.c ${CMAKE_SOURCE_DIR}/rvo_1_0/*.cpp ${CMAKE_SOURCE_DIR}/rvo_1_0/*.S)
FILE(GLOB_RECURSE RVO_2_SOURCE ${CMAKE_SOURCE_DIR}/rvo_2_0/*.h ${CMAKE_SOURCE_DIR}/rvo_2_0/*.hpp ${CMAKE_SOURCE_DIR}/rvo_2_0/*.c ${CMAKE_SOURCE_DIR}/rvo_2_0/*.cpp ${CMAKE_SOURCE_DIR}/rvo_2_0/*.S)

if(USE_RVO_2_0)
    message("USE_RVO_2_0")
    set(SOURCES ${GLFW_SOURCE} ${COMMON_SOURCE} ${RVO_2_SOURCE})
    ADD_DEFINITIONS(-DUSE_RVO_2_0)
else()
    message("USE_RVO_1_0")
    set(SOURCES ${GLFW_SOURCE} ${COMMON_SOURCE} ${RVO_1_SOURCE})
    ADD_DEFINITIONS(-DUSE_RVO_1_0)
endif()

GROUP_SRC_BY_DIR(SOURCES)


FOREACH (FILE_NAME ${SOURCES})
    MESSAGE(${FILE_NAME})
endforeach()

set(INC_SRC ${SOURCES} )
FOREACH(FILE_NAME ${INC_SRC})
        STRING( REGEX REPLACE "[^/\\\\]+$" " " DIR_NAME ${FILE_NAME} )
        LIST(APPEND DIR_NAME_LIST ${DIR_NAME})
ENDFOREACH()
LIST(REMOVE_DUPLICATES DIR_NAME_LIST)
FOREACH(DIR_NAME ${DIR_NAME_LIST})
        MESSAGE("auto include " ${DIR_NAME} )
        include_directories(${DIR_NAME})
ENDFOREACH()

#通用设置部分 包括启用分组 设置 启动项目  
if(WIN32)
    ADD_DEFINITIONS(-DWIN32 -W3)
    SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON) 
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
else()
    ADD_DEFINITIONS(-Wall -g -ggdb -O0 )
    set(CMAKE_CXX_FLAGS -std=c++14)
    link_libraries(pthread m c)
endif()

#输出
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#自定义部分 
ADD_DEFINITIONS(-DGLFW_USE_CONFIG_H -D_GLFW_WIN32)


if(USE_RVO_2_0)
    FILE(GLOB_RECURSE TEST_CPP ${CMAKE_SOURCE_DIR}/tests_2_0/*.cpp)
    FOREACH (FILE_NAME ${TEST_CPP})
        STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" TEST_NAME ${FILE_NAME})
        ADD_EXECUTABLE(${TEST_NAME}_2_0 ${FILE_NAME} ${GLFW_SOURCE} ${COMMON_SOURCE} ${RVO_2_SOURCE})
        IF (ENABLE_GCOV AND NOT WIN32 AND NOT APPLE)     
            SET_TARGET_PROPERTIES(${TEST_NAME} 
                PROPERTIES 
                COMPILE_FLAGS "-ftest-coverage -fprofile-arcs" 
                LINK_FLAGS "-lgcov --coverage" 
            ) 
        ENDIF()
        MESSAGE("TEST " ${TEST_NAME} )
    ENDFOREACH()
else()

    FILE(GLOB_RECURSE TEST_CPP ${CMAKE_SOURCE_DIR}/tests_1_0/*.cpp)
    FOREACH (FILE_NAME ${TEST_CPP})
        STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" TEST_NAME ${FILE_NAME})
        ADD_EXECUTABLE(${TEST_NAME}_1_0 ${FILE_NAME} ${GLFW_SOURCE} ${COMMON_SOURCE} ${RVO_1_SOURCE})
        IF (ENABLE_GCOV AND NOT WIN32 AND NOT APPLE)     
            SET_TARGET_PROPERTIES(${TEST_NAME} 
                PROPERTIES 
                COMPILE_FLAGS "-ftest-coverage -fprofile-arcs" 
                LINK_FLAGS "-lgcov --coverage" 
            ) 
        ENDIF()
        MESSAGE("TEST " ${TEST_NAME} )
    ENDFOREACH()

endif()
