idf_build_get_property(project_dir PROJECT_DIR)

idf_component_register(SRCS "time.c" "influxDB.c" "relay_logic.c" "wifi.c" "uart.c" "main.c"
                    INCLUDE_DIRS "")