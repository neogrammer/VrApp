cmake_minimum_required(VERSION 3.31.1)
set(PROJECT_NAME VrApp)
project("${PROJECT_NAME}" LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Additional Directories for find_package() to search within.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
set(ANDROID_STL c++_shared CACHE STRING "STL type for Android" FORCE)# For FetchContent_Declare() and FetchContent_MakeAvailable()
include(FetchContent)

FetchContent_Declare(
        oboe
        GIT_REPOSITORY https://github.com/google/oboe.git
        GIT_TAG        1.10.0
)

FetchContent_MakeAvailable(oboe)





# openxr_loader - From github.com/KhronosGroup
set(BUILD_API_LAYERS ON CACHE INTERNAL "Use OpenXR layers")
set(BUILD_TESTS OFF CACHE INTERNAL "Build tests")

FetchContent_Declare(
        OpenXR
        EXCLUDE_FROM_ALL
        DOWNLOAD_EXTRACT_TIMESTAMP
        URL_HASH MD5=f52248ef83da9134bec2b2d8e0970677
        URL https://github.com/KhronosGroup/OpenXR-SDK-Source/archive/refs/tags/release-1.1.49.tar.gz
        SOURCE_DIR
        openxr
)

FetchContent_MakeAvailable(OpenXR)




# Files
set(SOURCES
        "app/src/main/cpp/vrcore/vr_instance.cpp"
        "app/src/main/cpp/app_main.cpp"
        "../Common/GraphicsAPI.cpp"
        "../Common/GraphicsAPI_OpenGL_ES.cpp"
        "../Common/OpenXRDebugUtils.cpp"
        "app/src/main/cpp/audio_engine.cpp"
)
set(HEADERS
        "app/src/main/cpp/vrcore/vr_instance.hpp"
        "../Common/DebugOutput.h"
        "../Common/GraphicsAPI.h"
        "../Common/GraphicsAPI_OpenGL_ES.h"
        "../Common/HelperFunctions.h"
        "../Common/OpenXRDebugUtils.h"
        "../Common/OpenXRHelper.h"
        "app/src/main/cpp/audio_engine.h"
)

set(ES_GLSL_SHADERS "../Shaders/VertexShader_GLES.glsl"
        "../Shaders/PixelShader_GLES.glsl"
)


add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} XrApiLayer_core_validation)

target_include_directories(
        ${PROJECT_NAME}
        PRIVATE
        # In this repo
        ../Common/
        # From OpenXR repo
        "${openxr_SOURCE_DIR}/src/common"
        "${openxr_SOURCE_DIR}/external/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/app/src/main/cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/app/src/main/cpp/vrcore"
        "${openxr_SOURCE_DIR}/include"
        "${openxr_SOURCE_DIR}/src/ext/include"
)

# export ANativeActivity_onCreate for java to call.
set_property(
        TARGET ${PROJECT_NAME}
        APPEND_STRING
        PROPERTY LINK_FLAGS " -u ANativeActivity_onCreate"
)

# add native_app_glue to connect JVM and c++ code allowing us to use android_main()
include(AndroidNdkModules)
android_ndk_import_module_native_app_glue()


target_link_libraries(${PROJECT_NAME} android native_app_glue openxr_loader oboe)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-cast-calling-convention)

include(../thirdparty/glwrapper/CMakeLists.txt)
if(TARGET tutorial_glwrapper)
    target_link_libraries(${PROJECT_NAME} tutorial_glwrapper)
    target_compile_definitions(
            ${PROJECT_NAME} PUBLIC XR_TUTORIAL_USE_OPENGL_ES
    )
endif()

#OpenGL ES GLSL
set(SHADER_DEST "${CMAKE_CURRENT_SOURCE_DIR}/app/src/main/assets/shaders")
foreach(FILE ${ES_GLSL_SHADERS})
    get_filename_component(FILE_WE ${FILE} NAME_WE)
    add_custom_command(
            OUTPUT "${SHADER_DEST}/${FILE_WE}.glsl"
            COMMAND
            ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/${FILE}"
            "${SHADER_DEST}/${FILE_WE}.glsl"
            COMMENT "GLSL ${FILE}"
            MAIN_DEPENDENCY "${FILE}"
            DEPEND "${FILE}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            VERBATIM
    )
    # Make our project depend on these files
    target_sources(${PROJECT_NAME} PRIVATE "${SHADER_DEST}/${FILE_WE}.glsl")
endforeach(FILE)