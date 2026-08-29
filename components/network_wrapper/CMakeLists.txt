# Set source files and include directories
set(srcs)
set(include_dirs "include")

# Conditionally add source files
if(CONFIG_SIMPLE_NETWORK_WRAPPER)
    list(APPEND srcs
        "network_wrapper.c")
endif()

# Register the component
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       PRIV_REQUIRES esp_event ethernet_qemu wifi_sta)
