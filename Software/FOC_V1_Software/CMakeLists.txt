cmake_minimum_required(VERSION 3.16)

# Host GCC cannot build this Cortex-M3 firmware. Point CMake at the ARM
# toolchain before the project() call when the user did not pass one.
if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/gcc/arm-none-eabi.cmake" CACHE FILEPATH "" FORCE)
endif()

project(foc_gb6010 C ASM)

set(TARGET foc_gb6010)

add_executable(${TARGET}
  gcc/startup_stm32f103xb.s
  Core/Src/main.c
  Core/Src/controll.c
  Core/Src/AS5600.c
  Core/Src/flash.c
  Core/Src/gpio.c
  Core/Src/adc.c
  Core/Src/can.c
  Core/Src/dma.c
  Core/Src/i2c.c
  Core/Src/tim.c
  Core/Src/usart.c
  Core/Src/stm32f1xx_it.c
  Core/Src/stm32f1xx_hal_msp.c
  Core/Src/system_stm32f1xx.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
)

target_include_directories(${TARGET} PRIVATE
  Core/Inc
  Drivers/STM32F1xx_HAL_Driver/Inc
  Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32F1xx/Include
  Drivers/CMSIS/Include
  IQMath-master
)

target_compile_definitions(${TARGET} PRIVATE
  USE_HAL_DRIVER
  STM32F103xB
)

target_compile_options(${TARGET} PRIVATE
  -Wall
  -ffunction-sections
  -fdata-sections
  -fno-common
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
)

target_link_options(${TARGET} PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/gcc/STM32F103C8Tx_FLASH.ld
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET}.map
  --specs=nano.specs
  --specs=nosys.specs
)

target_link_libraries(${TARGET} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/IQMath-master/gcc-cm3/IQmathLib-cm3.a
  m
)

set_target_properties(${TARGET} PROPERTIES
  SUFFIX ".elf"
  LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/gcc/STM32F103C8Tx_FLASH.ld"
)

add_custom_command(TARGET ${TARGET} POST_BUILD
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET}>
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET}> ${CMAKE_BINARY_DIR}/${TARGET}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET}> ${CMAKE_BINARY_DIR}/${TARGET}.bin
  VERBATIM
)
