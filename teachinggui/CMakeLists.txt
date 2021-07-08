cmake_minimum_required(VERSION 2.8.3)
project(teachinggui)

add_definitions(-std=c++11 -fPIC -Wall -Wextra)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets)


file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

set(QT_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/qnode.cpp

    src/main_window.hpp
    src/qnode.hpp
    )

cs_add_executable(teachinggui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(teachinggui
#    ${PROJECT_NAME}
    Qt5::Core
    Qt5::Widgets
)

cs_install()
cs_export()