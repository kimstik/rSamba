################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../device_config/ \
../Device_Startup/ \
../drivers/ \
../sam_ba/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../Device_Startup/startup_saml22.c \
../Device_Startup/system_saml22.c \
../drivers/cdc_enumerate.c \
../drivers/uart_driver.c \
../main.c \
../sam_ba/sam_ba_monitor.c \
../sam_ba/usart_sam_ba.c \
../sam_ba/usb_sam_ba.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
Device_Startup/startup_saml22.o \
Device_Startup/system_saml22.o \
drivers/cdc_enumerate.o \
drivers/uart_driver.o \
main.o \
sam_ba/sam_ba_monitor.o \
sam_ba/usart_sam_ba.o \
sam_ba/usb_sam_ba.o

OBJS_AS_ARGS +=  \
Device_Startup/startup_saml22.o \
Device_Startup/system_saml22.o \
drivers/cdc_enumerate.o \
drivers/uart_driver.o \
main.o \
sam_ba/sam_ba_monitor.o \
sam_ba/usart_sam_ba.o \
sam_ba/usb_sam_ba.o

C_DEPS +=  \
Device_Startup/startup_saml22.d \
Device_Startup/system_saml22.d \
drivers/cdc_enumerate.d \
drivers/uart_driver.d \
main.d \
sam_ba/sam_ba_monitor.d \
sam_ba/usart_sam_ba.d \
sam_ba/usb_sam_ba.d

C_DEPS_AS_ARGS +=  \
Device_Startup/startup_saml22.d \
Device_Startup/system_saml22.d \
drivers/cdc_enumerate.d \
drivers/uart_driver.d \
main.d \
sam_ba/sam_ba_monitor.d \
sam_ba/usart_sam_ba.d \
sam_ba/usb_sam_ba.d

OUTPUT_FILE_PATH +=SAM-BA_MONITOR_ROMLESS.elf

OUTPUT_FILE_PATH_AS_ARGS +=SAM-BA_MONITOR_ROMLESS.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+=  \
../Device_Startup/saml22n18a_flash.ld \
../Device_Startup/saml22n18a_sram.ld


# AVR32/GNU C Compiler

















Device_Startup/%.o: ../Device_Startup/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAML22N18A__ -DDEBUG  -I"../device_config" -I"../drivers" -I"../sam_ba" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\ARM\CMSIS\5.0.1\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAML22_DFP\1.1.62\include"  -Og -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m0plus -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

drivers/%.o: ../drivers/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAML22N18A__ -DDEBUG  -I"../device_config" -I"../drivers" -I"../sam_ba" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\ARM\CMSIS\5.0.1\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAML22_DFP\1.1.62\include"  -Og -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m0plus -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAML22N18A__ -DDEBUG  -I"../device_config" -I"../drivers" -I"../sam_ba" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\ARM\CMSIS\5.0.1\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAML22_DFP\1.1.62\include"  -Og -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m0plus -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

sam_ba/%.o: ../sam_ba/%.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.2.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAML22N18A__ -DDEBUG  -I"../device_config" -I"../drivers" -I"../sam_ba" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\ARM\CMSIS\5.0.1\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAML22_DFP\1.1.62\include"  -Og -ffunction-sections -mlong-calls -g3 -Wall -mcpu=cortex-m0plus -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.2.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mthumb -Wl,-Map="SAM-BA_MONITOR_ROMLESS.map" --specs=nano.specs -Wl,--start-group -lm  -Wl,--end-group -L"..\\Device_Startup"  -Wl,--gc-sections -mcpu=cortex-m0plus -Wl,--defsym,STACK_SIZE=0x280 -Tsaml22n18a_flash.ld  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O binary "SAM-BA_MONITOR_ROMLESS.elf" "SAM-BA_MONITOR_ROMLESS.bin"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "SAM-BA_MONITOR_ROMLESS.elf" "SAM-BA_MONITOR_ROMLESS.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "SAM-BA_MONITOR_ROMLESS.elf" "SAM-BA_MONITOR_ROMLESS.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objdump.exe" -h -S "SAM-BA_MONITOR_ROMLESS.elf" > "SAM-BA_MONITOR_ROMLESS.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "SAM-BA_MONITOR_ROMLESS.elf" "SAM-BA_MONITOR_ROMLESS.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-size.exe" "SAM-BA_MONITOR_ROMLESS.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "SAM-BA_MONITOR_ROMLESS.elf" "SAM-BA_MONITOR_ROMLESS.a" "SAM-BA_MONITOR_ROMLESS.hex" "SAM-BA_MONITOR_ROMLESS.bin" "SAM-BA_MONITOR_ROMLESS.lss" "SAM-BA_MONITOR_ROMLESS.eep" "SAM-BA_MONITOR_ROMLESS.map" "SAM-BA_MONITOR_ROMLESS.srec"
	