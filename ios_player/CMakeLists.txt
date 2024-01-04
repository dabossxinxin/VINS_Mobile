cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
project(ios_player LANGUAGES CXX OBJC)

set(CODE_SIGN_IDENTITY "iPhone Developer")      # /usr/bin/env xcrun security find-identity -v -p codesigning
set(DEVELOPMENT_TEAM "${IOS_DEVELOPMENT_TEAM}") # XCode: Build Settings > Signing > Development Team

set(TARGETED_DEVICE_FAMILY "1") # 1: iphone; 2: ipad; 1,2: both

# Generate .xcodeproject for ios_player
set(EXECUTABLE_NAME ios_player)
set(BUNDLE_NAME "VINS-Mobile")
set(BUNDLE_IDENTIFIER "${APP_IDENTIFIER_PREFIX}.player")
set(BUNDLE_VERSION ${PROJECT_VERSION})
set(LAUNCH_STORYBOARD LaunchScreen)
set(MAIN_STORYBOARD Main)

set(TARGETSDK iPhoneOS16.0.sdk)
set(CMAKE_OSX_SYSROOT /Applications/Xcode.app/Contents/Developer/Platforms/iPhoneOS.platform/Developer/SDKs/${TARGETSDK})
message(STATUS "CMAKE_OSX_SYSROOT: ${CMAKE_OSX_SYSROOT}")

macro(add_framework fwname appname)
    find_library(FRAMEWORK_${fwname}
        NAMES ${fwname}
        PATHS ${CMAKE_OSX_SYSROOT}/System/Library
        PATH_SUFFIXES Frameworks
        NO_DEFAULT_PATH
    )
    if(${FRAMEWORK_${fwname}} STREQUAL FRAMEWORK_${fwname}-NOTFOUND)
        message(ERROR "Framework ${fwname} not found")
    else()
        target_link_libraries(${appname} PRIVATE "${FRAMEWORK_${fwname}}")
        message(STATUS "Framework ${fwname} found at ${FRAMEWORK_${fwname}}")
    endif()
endmacro(add_framework)

set(INFO_PLIST_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist)

set(VINS_IOS_PLAYER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.m
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AppDelegate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AppDelegate.m
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ViewController.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ViewController.mm
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CameraUtils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CameraUtils.m
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LoginController.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/LoginController.mm
)

set(MACOSX_BUNDLE_ICON_FILE icon.png)

set(VINS_IOS_PLAYER_STORYBOARDS
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/${LAUNCH_STORYBOARD}.storyboard
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/${MAIN_STORYBOARD}.storyboard
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/icon.png
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/brief_k10L6.bin
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/brief_pattern.yml
)

set(VINS_IOS_PLAYER_CONFIGS
    ${CMAKE_CURRENT_SOURCE_DIR}/configs)

set(VINS_IOS_PLAYER_SCN
    ${CMAKE_CURRENT_SOURCE_DIR}/scn)

set(VINS_IOS_PLAYER_RESOURCES
    ${VINS_IOS_PLAYER_STORYBOARDS}
    ${VINS_IOS_PLAYER_CONFIGS}
    ${VINS_IOS_PLAYER_SCN}
)

add_executable(${EXECUTABLE_NAME}
    MACOSX_BUNDLE
    ${VINS_IOS_PLAYER_SOURCES}
    ${VINS_IOS_PLAYER_STORYBOARDS}
    ${VINS_IOS_PLAYER_CONFIGS}
    ${VINS_IOS_PLAYER_SCN}
)

add_framework(CoreData ${EXECUTABLE_NAME})
add_framework(QuartzCore ${EXECUTABLE_NAME})
add_framework(CoreMotion ${EXECUTABLE_NAME})
add_framework(CoreVideo ${EXECUTABLE_NAME})
add_framework(CoreImage ${EXECUTABLE_NAME})
add_framework(CoreMedia ${EXECUTABLE_NAME})
add_framework(CoreGraphics ${EXECUTABLE_NAME})
add_framework(ImageCaptureCore ${EXECUTABLE_NAME})
add_framework(ImageIO ${EXECUTABLE_NAME})
add_framework(UIKit ${EXECUTABLE_NAME})
add_framework(AVFoundation ${EXECUTABLE_NAME})
add_framework(Accelerate ${EXECUTABLE_NAME})

target_link_libraries(${EXECUTABLE_NAME}
    PRIVATE
        ${DBoW}
        ${vins_estimator}
        ${OpenCV_LIBRARY}
        Boost::container
        RabbitMQ
        Json
        Ceres::ceres
)

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST ${INFO_PLIST_TEMPLATE}
    RESOURCE "${VINS_IOS_PLAYER_RESOURCES}"
)

macro (set_xcode_property TARGET XCODE_PROPERTY XCODE_VALUE)
set_xcode_property(${EXECUTABLE_NAME} CODE_SIGN_IDENTITY ${CODE_SIGN_IDENTITY} "All")
set_xcode_property(${EXECUTABLE_NAME} DEVELOPMENT_TEAM ${DEVELOPMENT_TEAM} "All")
set_xcode_property(${EXECUTABLE_NAME} TARGETED_DEVICE_FAMILY ${TARGETED_DEVICE_FAMILY} "All")
set_xcode_property(${EXECUTABLE_NAME} IPHONEOS_DEPLOYMENT_TARGET ${IOS_DEPLOYMENT_TARGET} "All")
set_xcode_property(${EXECUTABLE_NAME} SWIFT_VERSION "5.0" "All")
set_xcode_property(${EXECUTABLE_NAME} DEBUG_INFORMATION_FORMAT "dwarf-with-dsym" "All")
set_xcode_property(${EXECUTABLE_NAME} LD_RUNPATH_SEARCH_PATHS "@executable_path/Frameworks" "All")
endmacro (set_xcode_property)

if(ENABLE_BITCODE)
    macro (set_xcode_property TARGET XCODE_PROPERTY XCODE_VALUE)
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "YES" "All")
    endmacro (set_xcode_property)
else()
    macro (set_xcode_property TARGET XCODE_PROPERTY XCODE_VALUE)
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "NO" "All")
    endmacro (set_xcode_property)
endif()
if(ENABLE_ARC)
    macro (set_xcode_property TARGET XCODE_PROPERTY XCODE_VALUE)
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "YES" "All")
    endmacro (set_xcode_property)
else()
    macro (set_xcode_property TARGET XCODE_PROPERTY XCODE_VALUE)
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "NO" "All")
    endmacro (set_xcode_property)
endif()
