/*******************************************************************************
*lab3_sistemas_embarcados
*
*Alunos
*1. Rodrigo Knelsen Friesen
*2. Yuri Andreiko
*
*Equipe: S11_G09
*
*Data: 13/11/2019
*
*******************************************************************************/

#include <stdint.h>
#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include "UART_funcs.h"  //UART functions
#include "PWM_funcs.h"  //PWM functions 
#include "QEI_funcs.h"  //QEI functions 
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"


#define MSGQUEUE_OBJECTS      5 //quantidade de mensagens na fila

osThreadId_t UART_thread_id, PWM_thread_id;

osMessageQueueId_t Set_Point_msg;  

void UART_thread(void *arg)
{
  uint32_t valor;
  int32_t firstDigit = 0, secondDigit = 0, intermediaryDigit = 0;

  while(1)
  {
    while(UART_char_available())                                // 2 char sequencia
    {                                                           // converter 2 char em 1 uint16_t
      intermediaryDigit = UART_get_byte();
      if(UART_char_available())                                 //Means there's a two digit number on FIFO
      {
        firstDigit = intermediaryDigit;
        firstDigit -= 0x30;
        secondDigit = UART_get_byte();
        secondDigit -= 0x30;
        valor = (firstDigit * 10) + secondDigit;                //Calculates the angle final value based on the delivered FIFO values        
      }
      else                                                      //Means there's just one number on FIFO
      {
        intermediaryDigit -= 0x30;
        valor = intermediaryDigit;                              //Just passes the only FIFO value to angle
      }
      
       osMessageQueuePut(Set_Point_msg, &valor, 0, NULL);
    }
  }
}

void PWM_thread(void *arg)
{
  uint8_t duty_cycle;
  osStatus_t status;
  while(1) {
    status = osMessageQueueGet(Set_Point_msg, &duty_cycle, NULL, osWaitForever);  // wait for message
    if (status == osOK) {
      
        PWM_set_duty(duty_cycle);
    }
  } 
}

uint32_t qeiVelocidade = 0;

void main(void){
  SystemInit();
//  UART_init();
//  PWM_init();
  QEI_init();

//  osKernelInitialize();
//  
//  UART_thread_id = osThreadNew(UART_thread, NULL, NULL);
//  PWM_thread_id = osThreadNew(PWM_thread, NULL, NULL);
//  
//  Set_Point_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint32_t), NULL);
//
//  if(osKernelGetState() == osKernelReady)
//    osKernelStart();


  
  while(1)
  {
      qeiVelocidade = (uint32_t)QEIVelocityGet(QEI0_BASE)/18;
      SysCtlDelay (10000);
  }
} // main
