/*
 * hardfault.s
 *
 *  Created on: Jul 29, 2015
 *      Author: Angus
 *
 *
 *      Use this to debug hard faults while debugging
 */


/*

.syntax unified
.cpu cortex-m4
.thumb

.global UsageFault_Handler
.extern hard_fault_handler_c

UsageFault_Handler:
  TST LR, #4			//AND test: TST Rn <op2>
  ITE EQ				//If - then - else?
  MRSEQ R0, MSP			//Read special register, (MSP) = Main stack pointer
  MRSNE R0, PSP
  B hard_fault_handler_c

 */



