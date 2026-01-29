file(GLOB CPP_SOURCES "*.cpp")

idf_component_register(SRCS ${CPP_SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES arduino Simple_FOC app_update)