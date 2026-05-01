set(srcs            ./app_main.cpp
                    ./frame_cap_pipeline.cpp
                    ./boundary_monitor_app.cpp
                    ../cv_test/c_port/intrusion_detector.c)

set(include_dirs    ./
                    ../cv_test/c_port)

set(requires who_spiflash_fatfs
             who_task
             who_frame_cap
             who_frame_lcd_disp
             who_lcd
             hand_detect)

idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})




