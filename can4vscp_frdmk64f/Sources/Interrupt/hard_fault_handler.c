/*
 * hard_fault_handler.c
 *
 * Use this to debug hard faults while debugging
 *
 *
 *
 *
 */

#include "main.h"

// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file hardfault.s
void hard_fault_handler_c (unsigned int * hardfault_args)
{

	/* These are volatile to try and prevent the compiler/linker optimizing them
	away as the variables never actually get used.*/

  volatile unsigned int stacked_r0;
  volatile unsigned int stacked_r1;
  volatile unsigned int stacked_r2;
  volatile unsigned int stacked_r3;
  volatile unsigned int stacked_r12;
  volatile unsigned int stacked_lr;
  volatile unsigned int stacked_pc;
  volatile unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  PRINTF ("\n\n[Hard fault handler - all numbers in hex]\r\n");
  PRINTF ("R0 = %x\r\n", stacked_r0);
  PRINTF ("R1 = %x\r\n", stacked_r1);
  PRINTF ("R2 = %x\r\n", stacked_r2);
  PRINTF ("R3 = %x\r\n", stacked_r3);
  PRINTF ("R12 = %x\r\n", stacked_r12);
  PRINTF ("LR [R14] = %x  subroutine call return address\r\n", stacked_lr);
  PRINTF ("PC [R15] = %x  program counter\r\n", stacked_pc);
  PRINTF ("PSR = %x\r\n", stacked_psr);
  PRINTF ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  PRINTF ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  PRINTF ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  PRINTF ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
  PRINTF ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  //PRINTF ("SCB_SHCSR = %x\n", SCB->SHCSR);

  while (1);
}


void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[ 0 ];
	r1 = pulFaultStackAddress[ 1 ];
	r2 = pulFaultStackAddress[ 2 ];
	r3 = pulFaultStackAddress[ 3 ];

	r12 = pulFaultStackAddress[ 4 ];
	lr = pulFaultStackAddress[ 5 ];
	pc = pulFaultStackAddress[ 6 ];
	psr = pulFaultStackAddress[ 7 ];

	PRINTF ("R0 = %x\r\n", r0);
	PRINTF ("R1 = %x\r\n", r1);
	PRINTF ("R2 = %x\r\n", r2);
	PRINTF ("R3 = %x\r\n", r3);

	PRINTF ("R12 = %x\r\n", r12);
	PRINTF ("lr = %x\r\n", lr);
	PRINTF ("pc = %x\r\n", pc);
	PRINTF ("psr = %x\r\n", psr);

	/* When the following line is hit, the variables contain the register values. */
	for( ;; );
}
