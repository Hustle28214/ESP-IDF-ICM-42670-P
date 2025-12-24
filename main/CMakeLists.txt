set(COMPONENT_SRCS
    "main.c"
)

set(COMPONENT_INCLUDES
    "."
    "src"
    "Examples"
)

foreach(module src Examples)
    file(GLOB MODULE_SRCS "${module}/*.c")
    list(APPEND COMPONENT_SRCS ${MODULE_SRCS})
endforeach()

idf_component_register(SRCS ${COMPONENT_SRCS}
                      INCLUDE_DIRS ${COMPONENT_INCLUDES})

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DESP_LOG_FLOAT=1")