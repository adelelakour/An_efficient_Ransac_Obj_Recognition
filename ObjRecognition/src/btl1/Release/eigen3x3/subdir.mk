################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../eigen3x3/dsyevc3.cpp \
../eigen3x3/dsyevh3.cpp \
../eigen3x3/dsyevq3.cpp \
../eigen3x3/dsytrd3.cpp 

OBJS += \
./eigen3x3/dsyevc3.o \
./eigen3x3/dsyevh3.o \
./eigen3x3/dsyevq3.o \
./eigen3x3/dsytrd3.o 

CPP_DEPS += \
./eigen3x3/dsyevc3.d \
./eigen3x3/dsyevh3.d \
./eigen3x3/dsyevq3.d \
./eigen3x3/dsytrd3.d 


# Each subdirectory must supply rules for building sources it contributes
eigen3x3/%.o: ../eigen3x3/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


