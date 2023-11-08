/*
 * DebugHandler.c
 *
 * Created: 05.04.2017 08:29:36
 *  Author: M43734
 */ 

#include <stdint.h>
#include <stdio.h>

typedef enum
{
	H_NMI,
	H_HARDFAULT,
	H_MEMMANAGE,
	H_BUSFAULT,
	H_USAGEFAULT,
	H_MAX
} Err_Handler;

const char* errorToString[] =
{
	"NMI_Handler",
	"Hardfault_Handler",
	"MemManage_Handler",
	"BusFault_Handler",
	"UsageFault_Handler",
	"ERROR"
};

static Err_Handler errCode = H_MAX;

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
void Fault_HandlerC(uint32_t* hardfault_args)
{
	volatile uint32_t stacked_r0;
	volatile uint32_t stacked_r1;
	volatile uint32_t stacked_r2;
	volatile uint32_t stacked_r3;
	volatile uint32_t stacked_r12;
	volatile uint32_t stacked_lr;
	volatile uint32_t stacked_pc;
	volatile uint32_t stacked_psr;
	volatile uint32_t _CFSR;
	volatile uint32_t _HFSR;
	volatile uint32_t _DFSR;
	volatile uint32_t _AFSR;
	volatile uint32_t _BFAR;
	volatile uint32_t _MMAR;
	
	stacked_r0 =  hardfault_args[0];
	stacked_r1 =  hardfault_args[1];
	stacked_r2 =  hardfault_args[2];
	stacked_r3 =  hardfault_args[3];
	stacked_r12 = hardfault_args[4];
	stacked_lr =  hardfault_args[5];
	stacked_pc =  hardfault_args[6];
	stacked_psr = hardfault_args[7];
	// Configurable Fault Status Register
	// Consists of MMSR, BFSR and UFSR
	_CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
	// Hard Fault Status Register
	_HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
	// Debug Fault Status Register
	_DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
	// Auxiliary Fault Status Register
	_AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
	// Read the Fault Address Registers. These may not contain valid values.
	// Check BFARVALID/MMARVALID to see if they are valid values
	// MemManage Fault Address Register
	_MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
	// Bus Fault Address Register
	_BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
	printf("%s\r\n", errorToString[errCode]);
	__asm("BKPT #0\n") ; // Break into the debugger
	while(1);
}
#pragma GCC diagnostic pop

__attribute__((naked)) void Fault_HandlerA(void)
{
	__asm(
		".syntax unified\n"
		"MOVS R0, #4 \n"
		"MOV R1, LR \n"
		"TST R0, R1 \n"
		"BEQ _MSP \n"
		"MRS R0, PSP \n"
		"B Fault_HandlerC \n"
		"_MSP: \n"
		"MRS R0, MSP \n"
		"B Fault_HandlerC \n"
		".syntax divided \n"
	);
}

__attribute__((naked)) void MemManage_Handler(void)
{
	errCode = H_MEMMANAGE;
	__asm("b Fault_HandlerA");
}

__attribute__((naked)) void NMI_Handler(void)
{
	errCode = H_NMI;
	__asm("b Fault_HandlerA");
}

__attribute__((naked)) void HardFault_Handler(void)
{
	errCode = H_HARDFAULT;
	__asm("b Fault_HandlerA");
}

__attribute__((naked)) void BusFault_Handler(void)
{
	errCode = H_BUSFAULT;
	__asm("b Fault_HandlerA");
}

__attribute__((naked)) void UsageFault_Handler(void)
{
	errCode = H_USAGEFAULT;
	__asm("b Fault_HandlerA");
}
