CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
 
PROJECT (wizard_stick)
 
# Startup code is written by ST in assembly, so without this statement there are errors.
ENABLE_LANGUAGE (ASM-ATT)
 
INCLUDE_DIRECTORIES("Src/")
INCLUDE_DIRECTORIES("Inc/")
 
# Resonator used in this project. Stm32F4-DISCO uses 8MHz crystal. I left this definition here
# in the CMakeLists.txt rather than the toolchain file, because it's project dependent, not
# "platform" dependent, where by platform I mean STM32F4.
ADD_DEFINITIONS (-DHSE_VALUE=8000000)
# All the sources goes here. Adding headers isn't obligatory, but since QtCreator treats CMakeLists.txt as
# its "project configuration" it simply makes header files appear in the source tree pane.
ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf
 "Src/main.c"
 "Inc/main.h"
 "Inc/stm32f1xx_hal_conf.h"
 "Src/stm32f1xx_hal_msp.c"
 "Src/stm32f1xx_it.c"
 "Inc/stm32f1xx_it.h"
 "Src/system_stm32f1xx.c"
 "Src/MPU6050/mpu6050.c"
 "Src/MPU6050/I2C.c"
 "Src/MPU6050/inv_mpu.c"
 "Src/MPU6050/inv_mpu_dmp_motion_driver.c"
 "Inc/MPU6050/mpu6050.h"
 "Inc/MPU6050/I2C.h"
 "Inc/MPU6050/inv_mpu.h"
 "Inc/MPU6050/inv_mpu_dmp_motion_driver.h"
 "Inc/MPU6050/dmpKey.h"
 "Inc/MPU6050/dmpmap.h"


)
 
# Workaround : splitting C and C++ code helps QtCreator parse header files correctly. Without it, QtCreator
# sometimes treats C++ as C and vice versa. EDIT : this comment was written when in the ADD_EXECUTABLE C++
# files were present.
add_library ("stm" STATIC
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_cortex.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio_ex.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_gpio.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc_ex.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal_rcc.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_cortex.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_gpio.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_rcc.h"
 "${CUBE_ROOT}/Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_ll_system.h"
 "${STARTUP_CODE}"
)
 
# This links both pieces together.
TARGET_LINK_LIBRARIES (${CMAKE_PROJECT_NAME}.elf -Wl,--whole-archive stm -Wl,--no-whole-archive)
 
FIND_PROGRAM (OPENOCD openocd)
ADD_CUSTOM_TARGET("upload" DEPENDS ${CMAKE_PROJECT_NAME}.elf COMMAND ${OPENOCD} -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c 'program ${CMAKE_PROJECT_NAME}.elf verify reset exit')
