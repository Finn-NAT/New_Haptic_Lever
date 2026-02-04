file(GLOB CPP_SOURCES
    "*.c"
    "*.cpp"
    "Motor_Haptic_Function/*.c"
    "Motor_Haptic_Function/*.cpp"
    "Motor_Haptic_Azipod_Function/*.c"
    "Motor_Haptic_Azipod_Function/*.cpp"
)

idf_component_register(SRCS ${CPP_SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES arduino Simple_FOC app_update)