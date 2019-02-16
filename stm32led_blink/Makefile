PROJECT = led_blink
DRIVERS_PATH = /media/sdv/Tank/stm32_dev/STM32Cube_FW_F1_V1.7.0

COMPILER = arm-none-eabi
CC = ${COMPILER}-gcc
CXX = ${COMPILER}-g++
AR = ${COMPILER}-ar
AS = ${COMPILER}-as
LD = ${COMPILER}-ld 
OBJCOPY = ${COMPILER}-objcopy
RANLIB = ${COMPILER}-ranlib 
SIZE = ${COMPILER}-size

CFLAGS = -Wall
CFLAGS+= -TSTM32F103XB_FLASH.ld
CFLAGS += -mcpu=cortex-m3
CFLAGS += -mlittle-endian -mthumb
CFLAGS += -mfloat-abi=soft --specs=nosys.specs
CFLAGS += -ffreestanding

CPPFLAGS = -DSTM32F103xB
CPPFLAGS += -Iinclude
CPPFLAGS += -I${addprefix ${DRIVERS_PATH}, "/Drivers/STM32F1xx_HAL_Driver/Inc"}
CPPFLAGS += -I${addprefix ${DRIVERS_PATH}, "/Drivers/CMSIS/Include"}
CPPFLAGS += -I${addprefix ${DRIVERS_PATH}, "/Drivers/CMSIS/Device/ST/STM32F1xx/Include"}

LIB_NAME = libstm32cubef1.a
LDFLAGS = -L.
LDFLAGS += -lstm32cubef1

LIB_SRCS = ${wildcard ${DRIVERS_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/*.c}
LIB_SRCS := ${filter-out ${DRIVERS_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_timebase_tim_template.c, ${LIB_SRCS}}
LIB_SRCS := ${filter-out ${DRIVERS_PATH}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_timebase_rtc_alarm_template.c, ${LIB_SRCS}}
LIB_OBJS = ${LIB_SRCS:%.c=%.o}

SRCS = ${wildcard src/*.cpp}
SRCS += ${wildcard src/*.c}
SRCS += ${wildcard src/*.s}

CLEAN_FILES = ${PROJECT}.elf ${PROJECT}.hex ${PROJECT}.bin *.o ${LIB_NAME}

all: lib ${PROJECT}.elf

lib: ${LIB_OBJS}
	@echo "LIB_SRCS: " ${LIB_SRCS}
	@echo "LIB_OBJS: " ${LIB_OBJS}
	${AR} cr ${LIB_NAME} ${LIB_OBJS}

${PROJECT}.elf: ${SRCS}
	${CXX} ${CFLAGS} ${CPPFLAGS} ${SRCS} ${LDFLAGS} -o $@
	${OBJCOPY} -O ihex ${PROJECT}.elf ${PROJECT}.hex
	${OBJCOPY} -O binary ${PROJECT}.elf ${PROJECT}.bin 
	${SIZE} ${PROJECT}.elf 

%.o: %.c 
	${CC} -c ${CFLAGS} ${CPPFLAGS} $< -o $@

flash: all
	../stlink/build/Release/st-flash --reset write ${PROJECT}.bin 0x08000000
	
cleanlib:
	rm -rf ${LIB_OBJS}

clean:
	rm -rf ${CLEAN_FILES}

cleanall: cleanlib clean 

.PHONY: clean cleanlib cleanall