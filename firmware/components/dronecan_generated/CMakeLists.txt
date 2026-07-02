file(GLOB DRONECAN_GENERATED_SRCS "src/*.c")

idf_component_register(
    SRCS
        ${DRONECAN_GENERATED_SRCS}
    INCLUDE_DIRS
        "include"
    REQUIRES
        canard
)

