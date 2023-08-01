cmake_minimum_required(VERSION 3.5)

# что-бы не проверялись компиляторы соборкой тестового проекта
set(CMAKE_ASM_COMPILER_WORKS TRUE)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

project(stm32f0_test LANGUAGES  CXX C ASM)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RELATIVE_PRJ_DIR ".")
set(RELATIVE_SCRIPTS_DIR "./scripts")
get_filename_component(PRJ_DIR ${RELATIVE_PRJ_DIR}  ABSOLUTE)
get_filename_component(SCRIPTS_DIR ${RELATIVE_SCRIPTS_DIR}  ABSOLUTE)





set(SRC_DIR "${PRJ_DIR}/src")

include_directories(${PRJ_DIR}/src)
include_directories(${PRJ_DIR}/src/system)
include_directories(${PRJ_DIR}/src/hal)

file (GLOB_RECURSE H_FILES ${SRC_DIR}/*.h)
file (GLOB_RECURSE CPP_FILES ${SRC_DIR}/*.cc)
file (GLOB_RECURSE C_FILES ${SRC_DIR}/*.c)


# Create bin directory
set(BIN_DIR "${PRJ_DIR}/bin")
if(NOT EXISTS ${BIN_DIR})
    file(MAKE_DIRECTORY ${BIN_DIR})
endif()

set(TOOLCHAIN_PATH /home/drd/opt/arm-gcc)
set(PER_LIB_PATH /home/drd/prj/tmp/stm32/stm32f103_base/stm32f0/STM32F0xx_StdPeriph_Lib_V1.6.0/Libraries)


# check toolcain path
if ("${TOOLCHAIN_PATH}" STREQUAL "")
  message(WARNING "variable TOOLCHAIN_PATH not set")
else()
  message("TOOLCHAIN_PATH: ${TOOLCHAIN_PATH}")
endif()

#AS = $(BINPATH)/$(PREFIX)gcc -x assembler-with-cpp
#SET(CMAKE_ASM_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-gcc" )
#SET(CMAKE_C_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-gcc" )
#SET(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}/arm-none-eabi-c++" )
#SET(CMAKE_C_AR "${TOOLCHAIN_PATH}/arm-none-eabi-ar" )
#SET(CMAKE_C_LINKER "${TOOLCHAIN_PATH}/arm-none-eabi-ld" )
SET(CMAKE_OBJCOPY "${TOOLCHAIN_PATH}/arm-none-eabi-objcopy" )
SET(CMAKE_OBJDUMP "${TOOLCHAIN_PATH}/arm-none-eabi-objdump" )
SET(CMAKE_SIZE "${TOOLCHAIN_PATH}/arm-none-eabi-size" )

SET(CMAKE_C_AR "${TOOLCHAIN_PATH}/bin/arm-none-eabi-ar" )
SET(CMAKE_C_LINKER "${TOOLCHAIN_PATH}/bin/arm-none-eabi-ld" )
SET(CMAKE_OBJCOPY "${TOOLCHAIN_PATH}/bin/arm-none-eabi-objcopy" )
SET(CMAKE_OBJDUMP "${TOOLCHAIN_PATH}/bin/arm-none-eabi-objdump" )
SET(CMAKE_SIZE "${TOOLCHAIN_PATH}/bin/arm-none-eabi-size" )


#####################################################
#/* #define STM32F10X_LD */     /*!< STM32F10X_LD: STM32 Low density devices */
# /* #define STM32F10X_LD_VL */  /*!< STM32F10X_LD_VL: STM32 Low density Value Line devices */
# #define STM32F10X_MD      /*!< STM32F10X_MD: STM32 Medium density devices */
# /* #define STM32F10X_MD_VL */  /*!< STM32F10X_MD_VL: STM32 Medium density Value Line devices */
# /* #define STM32F10X_HD */     /*!< STM32F10X_HD: STM32 High density devices */
# /* #define STM32F10X_HD_VL */  /*!< STM32F10X_HD_VL: STM32 High density value line devices */
# /* #define STM32F10X_XL */     /*!< STM32F10X_XL: STM32 XL-density devices */
# /* #define STM32F10X_CL */     /*!< STM32F10X_CL: STM32 Connectivity line devices */
#/*  Tip: To avoid modifying this file each time you need to switch between these
#       devices, you can define the device in your toolchain compiler preprocessor.

#- Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#  where the Flash memory density ranges between 16 and 32 Kbytes.
#- Low-density value line devices are STM32F100xx microcontrollers where the Flash
#  memory density ranges between 16 and 32 Kbytes.
#- Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#  where the Flash memory density ranges between 64 and 128 Kbytes.
#- Medium-density value line devices are STM32F100xx microcontrollers where the
#  Flash memory density ranges between 64 and 128 Kbytes.
#- High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#  the Flash memory density ranges between 256 and 512 Kbytes.
#- High-density value line devices are STM32F100xx microcontrollers where the
#  Flash memory density ranges between 256 and 512 Kbytes.
#- XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#  the Flash memory density ranges between 512 and 1024 Kbytes.
#- Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
# */
#####################################################

#set(TARGET_DEVICE STM32F0XX_SM)
set(TARGET_DEVICE STM32F031)

set (CPU -mcpu=cortex-m0)
set (CPU_RAM 4000)
set (CPU_FLASH 16000)

#-specs=nosys.specs -specs=nano.specs -u _printf_float
include_directories("${TOOLCHAIN_PATH}/arm-none-eabi/include")
include_directories("${PER_LIB_PATH}/CMSIS/Device/ST/STM32F0xx/Include")
include_directories("${PER_LIB_PATH}/CMSIS/Include")
include_directories("${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/inc")

#set ( CMAKE_C_FLAGS "-g3 -O0 -Wall -mthumb -mcpu=cortex-m3  -fdata-sections -ffunction-sections -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D ${TARGET_DEVICE}")
set ( CMAKE_C_FLAGS "-g3 -O0 -Wall -mthumb ${CPU} -fdata-sections -ffunction-sections -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D ${TARGET_DEVICE}")
#set ( CMAKE_CXX_FLAGS "-fno-exceptions  -g3 -O0 -Wall -mthumb -mcpu=cortex-m3 -fdata-sections -ffunction-sections -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D ${TARGET_DEVICE}")
set ( CMAKE_CXX_FLAGS "-fno-exceptions  -g3 -O0 -Wall -mthumb ${CPU} -fdata-sections -ffunction-sections -D STM32F10X_MD -D USE_STDPERIPH_DRIVER -D ${TARGET_DEVICE}")

#LDFLAGS
#set(LD_FILE ${SRC_DIR}/system/stm32f103c8tx_flash.ld)
#set(LD_FILE /home/drd/prj/tmp/stm32/stm32f103_base/stm32f0/STM32F0xx_StdPeriph_Lib_V1.6.0/Projects/STM32F0xx_StdPeriph_Templates/TrueSTUDIO/STM32F031/STM32F031C6_FLASH.ld)
set(LD_FILE ${SRC_DIR}/STM32F050K4_FLASH.ld)
#set ( CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -specs=nosys.specs -u _printf_float -Wl,--gc-sections" )
set ( CMAKE_EXE_LINKER_FLAGS "-specs=nano.specs -Wl,--gc-sections" )
set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T ${LD_FILE}" )

#ASMFLAGS
set ( CMAKE_ASM_FLAGS "-x assembler-with-cpp ${CMAKE_C_FLAGS}")


# StdLib
set(StdLib )
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/inc/stm32f0xx_rcc.h)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_rcc.c)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_gpio.c)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/inc/stm32f0xx_spi.h)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_spi.c)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/inc/stm32f0xx_misc.h)
list(APPEND StdLib ${PER_LIB_PATH}/STM32F0xx_StdPeriph_Driver/src/stm32f0xx_misc.c)

# startup file
#set(STARTUP_FILE ${SRC_DIR}/system/startup_stm32f10x_md.s)
set(STARTUP_FILE ${PER_LIB_PATH}/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc_ride7/startup_stm32f031.s)
add_executable ( ${PROJECT_NAME} ${H_FILES} ${C_FILES} ${CPP_FILES} ${STARTUP_FILE} ${StdLib})
#add_executable ( ${PROJECT_NAME}.elf ${H_FILES} ${C_FILES} ${CPP_FILES} )
#target_link_libraries(${PROJECT_NAME}    -lc )

# Create project.bin and copy it to bin directory
#add_custom_command(TARGET ${CMAKE_PROJECT_NAME}.elf
#    POST_BUILD
#    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
#    COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
#    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
#    COMMENT "Copy to ${BIN_DIR}:------------"
#    COMMAND cp ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin ${BIN_DIR}
#    COMMAND cp ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex ${BIN_DIR}
#    DEPENDS ${CMAKE_PROJECT_NAME}
#    )

# Create project.bin and copy it to bin directory

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMENT "Create binary from: ${CMAKE_BUILD_TYPE}"
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME} ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} --radix 10 --totals  ${PROJECT_NAME}
    COMMAND ${SCRIPTS_DIR}/fw_size.py --size ${CMAKE_SIZE} --file-name ${PROJECT_NAME} --flash-size ${CPU_FLASH} --ram-size ${CPU_RAM}
    COMMAND cp ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin ${BIN_DIR}
    COMMAND cp ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex ${BIN_DIR}
    DEPENDS ${PROJECT_NAME}
    )


set (CLEAN_FILES
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    ${BIN_DIR}/${CMAKE_PROJECT_NAME}.bin
    ${BIN_DIR}/${CMAKE_PROJECT_NAME}.hex
)

set_property(DIRECTORY PROPERTY
    ADDITIONAL_MAKE_CLEAN_FILES ${CLEAN_FILES}
)


