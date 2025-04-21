idf_component_register(SRCS "tuning_http_server_v3.c" "wifi_handler.c" "stepper_motor_encoder.c" "stepper_motor_ws_v2.c"
                       INCLUDE_DIRS "include"
                       EMBED_FILES ../frontend/index.html)

# set(COMPONENT_REQUIRES )
# set(COMPONENT_PRIV_REQUIRES )
                       
# set(COMPONENT_SRCS "tuning_http_server_v3.c" "wifi_handler.c" "stepper_motor_encoder.c" "stepper_motor_ws_v2.c")
# set(COMPONENT_ADD_INCLUDEDIRS "include")

# register_component()
# spiffs_create_partition_image(www "${CMAKE_CURRENT_SOURCE_DIR}/../frontend" FLASH_IN_PROJECT)
