cmake_minimum_required(VERSION 3.12.4)

if(NOT ESP_PLATFORM)
  project(TouchLib HOMEPAGE_URL https://github.com/mmMicky/TouchLib)
endif()

set(TOUCH_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

if(ESP_PLATFORM)
  
file(GLOB_RECURSE SOURCES ${TOUCH_ROOT_DIR}/src/*.cpp
                          ${TOUCH_ROOT_DIR}/src/*.tpp )

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS "src/."
                        PRIV_REQUIRES )

endif()
