//Funções do periférico ADC
//Autor: Yuri Andreiko
//Data: 02/07/19

#ifndef UART_FUNCS_H
#define	UART_FUNCS_H

void UART_init();

//
//UART Send byte -> definido com função "No Blocking" //TALVEZ ALTERAR****
//Parâmetros:
//      *pui8buffer: ponteiro de vetor de bytes
//      ui32Count: tamanho do vetor de bytes
//
void UART_send_byte(const uint8_t *pui8Buffer, uint32_t ui32Count);

extern uint32_t SystemCoreClock;        //Clock do uC, definido em sysctl.h

uint8_t UART_char_available();

int32_t UART_get_byte();


#endif
