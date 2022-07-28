idf_component_register(SRCS     "mpu6050/GY521.c"
                                "main.c" 
                                "max30102/max30102.c" 
                                "vs1053/vs1053.c"                           
                    INCLUDE_DIRS "." "mpu6050" "vs1053" "max30102")