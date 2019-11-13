;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V5.00
; * @date     08. March 2016
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     IntDefaultHandler            ; GPIO Port A
        DCD     IntDefaultHandler            ; GPIO Port B
        DCD     IntDefaultHandler            ; GPIO Port C
        DCD     IntDefaultHandler            ; GPIO Port D
        DCD     IntDefaultHandler            ; GPIO Port E
        DCD     IntDefaultHandler            ; UART0 Rx and Tx
        DCD     IntDefaultHandler            ; UART1 Rx and Tx
        DCD     IntDefaultHandler            ; SSI0 Rx and Tx
        DCD     IntDefaultHandler            ; I2C0 Master and Slave
        DCD     IntDefaultHandler            ; PWM Fault
        DCD     IntDefaultHandler            ; PWM Generator 0
        DCD     IntDefaultHandler            ; PWM Generator 1
        DCD     IntDefaultHandler            ; PWM Generator 2
        DCD     IntDefaultHandler            ; Quadrature Encoder 0
        DCD     IntDefaultHandler            ; ADC Sequence 0
        DCD     IntDefaultHandler            ; ADC Sequence 1
        DCD     IntDefaultHandler            ; ADC Sequence 2
        DCD     IntDefaultHandler            ; ADC Sequence 3
        DCD     IntDefaultHandler            ; Watchdog timer
        DCD     IntDefaultHandler            ; Timer 0 subtimer A
        DCD     IntDefaultHandler            ; Timer 0 subtimer B
        DCD     IntDefaultHandler            ; Timer 1 subtimer A
        DCD     IntDefaultHandler            ; Timer 1 subtimer B
        DCD     IntDefaultHandler            ; Timer 2 subtimer A
        DCD     IntDefaultHandler            ; Timer 2 subtimer B
        DCD     IntDefaultHandler            ; Analog Comparator 0
        DCD     IntDefaultHandler            ; Analog Comparator 1
        DCD     IntDefaultHandler            ; Analog Comparator 2
        DCD     IntDefaultHandler            ; System Control (PLL, OSC, BO)
        DCD     IntDefaultHandler            ; FLASH Control
        DCD     IntDefaultHandler            ; GPIO Port F
        DCD     IntDefaultHandler            ; GPIO Port G
        DCD     IntDefaultHandler            ; GPIO Port H
        DCD     IntDefaultHandler            ; UART2 Rx and Tx
        DCD     IntDefaultHandler            ; SSI1 Rx and Tx
        DCD     IntDefaultHandler            ; Timer 3 subtimer A
        DCD     IntDefaultHandler            ; Timer 3 subtimer B
        DCD     IntDefaultHandler            ; I2C1 Master and Slave
        DCD     IntDefaultHandler            ; Quadrature Encoder 1
        DCD     IntDefaultHandler            ; CAN0
        DCD     IntDefaultHandler            ; CAN1
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; Hibernate
        DCD     IntDefaultHandler            ; USB0
        DCD     IntDefaultHandler            ; PWM Generator 3
        DCD     IntDefaultHandler            ; uDMA Software Transfer
        DCD     IntDefaultHandler            ; uDMA Error
        DCD     IntDefaultHandler            ; ADC1 Sequence 0
        DCD     IntDefaultHandler            ; ADC1 Sequence 1
        DCD     IntDefaultHandler            ; ADC1 Sequence 2
        DCD     IntDefaultHandler            ; ADC1 Sequence 3
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; GPIO Port J
        DCD     IntDefaultHandler            ; GPIO Port K
        DCD     IntDefaultHandler            ; GPIO Port L
        DCD     IntDefaultHandler            ; SSI2 Rx and Tx
        DCD     IntDefaultHandler            ; SSI3 Rx and Tx
        DCD     IntDefaultHandler            ; UART3 Rx and Tx
        DCD     IntDefaultHandler            ; UART4 Rx and Tx
        DCD     IntDefaultHandler            ; UART5 Rx and Tx
        DCD     IntDefaultHandler            ; UART6 Rx and Tx
        DCD     IntDefaultHandler            ; UART7 Rx and Tx
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; I2C2 Master and Slave
        DCD     IntDefaultHandler            ; I2C3 Master and Slave
        DCD     IntDefaultHandler            ; Timer 4 subtimer A
        DCD     IntDefaultHandler            ; Timer 4 subtimer B
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; Timer 5 subtimer A
        DCD     IntDefaultHandler            ; Timer 5 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 0 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 0 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 1 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 1 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 2 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 2 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 3 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 3 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 4 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 4 subtimer B
        DCD     IntDefaultHandler            ; Wide Timer 5 subtimer A
        DCD     IntDefaultHandler            ; Wide Timer 5 subtimer B
        DCD     IntDefaultHandler            ; FPU
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; I2C4 Master and Slave
        DCD     IntDefaultHandler            ; I2C5 Master and Slave
        DCD     IntDefaultHandler            ; GPIO Port M
        DCD     IntDefaultHandler            ; GPIO Port N
        DCD     IntDefaultHandler            ; Quadrature Encoder 2
        DCD     0                            ; Reserved
        DCD     0                            ; Reserved
        DCD     IntDefaultHandler            ; GPIO Port P (Summary or P0)
        DCD     IntDefaultHandler            ; GPIO Port P1
        DCD     IntDefaultHandler            ; GPIO Port P2
        DCD     IntDefaultHandler            ; GPIO Port P3
        DCD     IntDefaultHandler            ; GPIO Port P4
        DCD     IntDefaultHandler            ; GPIO Port P5
        DCD     IntDefaultHandler            ; GPIO Port P6
        DCD     IntDefaultHandler            ; GPIO Port P7
        DCD     IntDefaultHandler            ; GPIO Port Q (Summary or Q0)
        DCD     IntDefaultHandler            ; GPIO Port Q1
        DCD     IntDefaultHandler            ; GPIO Port Q2
        DCD     IntDefaultHandler            ; GPIO Port Q3
        DCD     IntDefaultHandler            ; GPIO Port Q4
        DCD     IntDefaultHandler            ; GPIO Port Q5
        DCD     IntDefaultHandler            ; GPIO Port Q6
        DCD     IntDefaultHandler            ; GPIO Port Q7
        DCD     IntDefaultHandler            ; GPIO Port R
        DCD     IntDefaultHandler            ; GPIO Port S
        DCD     IntDefaultHandler            ; PWM 1 Generator 0
        DCD     IntDefaultHandler            ; PWM 1 Generator 1
        DCD     IntDefaultHandler            ; PWM 1 Generator 2
        DCD     IntDefaultHandler            ; PWM 1 Generator 3
        DCD     IntDefaultHandler            ; PWM 1 Fault
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B HardFault_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK IntDefaultHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IntDefaultHandler
        B IntDefaultHandler

        END
