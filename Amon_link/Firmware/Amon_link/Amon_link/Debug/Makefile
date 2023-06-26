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
../inc \
../src


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../src/LL_Fcn.c \
../src/main.c \
../src/nRF24L01.c \
../src/USART.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
src/LL_Fcn.o \
src/main.o \
src/nRF24L01.o \
src/USART.o

OBJS_AS_ARGS +=  \
src/LL_Fcn.o \
src/main.o \
src/nRF24L01.o \
src/USART.o

C_DEPS +=  \
src/LL_Fcn.d \
src/main.d \
src/nRF24L01.d \
src/USART.d

C_DEPS_AS_ARGS +=  \
src/LL_Fcn.d \
src/main.d \
src/nRF24L01.d \
src/USART.d

OUTPUT_FILE_PATH +=Amon_link.elf

OUTPUT_FILE_PATH_AS_ARGS +=Amon_link.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
src/LL_Fcn.o: ../src/LL_Fcn.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega328P  -mdfp="C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega328P__ -DDEBUG  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

src/main.o: ../src/main.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega328P  -mdfp="C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega328P__ -DDEBUG  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

src/nRF24L01.o: ../src/nRF24L01.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega328P  -mdfp="C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega328P__ -DDEBUG  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

src/USART.o: ../src/USART.c
	@echo Building file: $<
	@echo Invoking: XC8 C Compiler : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE)  -mcpu=ATmega328P  -mdfp="C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -c -x c -funsigned-char -funsigned-bitfields -mext=cci -D__ATmega328P__ -DDEBUG  -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
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
	@echo Invoking:  XC8 C Linker : 2.36
	$(QUOTE)C:\Program Files\Microchip\xc8\v2.36\bin\xc8-cc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mcpu=ATmega328P  -mdfp="C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.7.374\xc8"  -Wl,-Map="Amon_link.map" -funsigned-char -funsigned-bitfields -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -Og -ffunction-sections -fdata-sections -fpack-struct -fshort-enums --memorysummary,memoryfile.xml  
	@echo Finished building target: $@
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Amon_link.elf" "Amon_link.hex"
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "Amon_link.elf" "Amon_link.eep" || exit 0
	"C:\Program Files\Microchip\xc8\v2.36\bin\avr-objdump.exe" -h -S "Amon_link.elf" > "Amon_link.lss"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Amon_link.elf" "Amon_link.hex" "Amon_link.eep" "Amon_link.map" "Amon_link.usersignatures" "Amon_link.a"  "Amon_link.lss"
	