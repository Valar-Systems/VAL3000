
set(srcs 
        "."
        "source/TMC2208Stepper.cpp"
        "source/TMC2209Stepper.cpp"
        "source/CHOPCONF.cpp"
        "source/COOLCONF.cpp"
        "source/PWMCONF.cpp"
        "source/DRV_STATUS.cpp"
        "source/GCONF.cpp"
        "source/IHOLD_IRUN.cpp"
        "source/TMCStepper.cpp")
        
idf_component_register(SRCS  ${srcs}
                  INCLUDE_DIRS "."
                  REQUIRES driver esp_timer)