
find_library(OpenGLES_LIBRARY NAMES GLESv3 GLESv2 GLESv1_CM)
find_path(OpenGLES_INCLUDE_DIR NAMES GLES3/gl3.h GLES2/gl2.h GLES/gl.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenGLES DEFAULT_MSG OpenGLES_LIBRARY OpenGLES_INCLUDE_DIR)

if(OpenGLES_FOUND)
    set(OpenGLES_LIBRARIES ${OpenGLES_LIBRARY})
    set(OpenGLES_INCLUDE_DIRS ${OpenGLES_INCLUDE_DIR})
    if(NOT TARGET OpenGLES::OpenGLES)
        add_library(OpenGLES::OpenGLES UNKNOWN IMPORTED)
        set_target_properties(OpenGLES::OpenGLES PROPERTIES
            IMPORTED_LOCATION "${OpenGLES_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${OpenGLES_INCLUDE_DIR}"
        )
    endif()
endif()
