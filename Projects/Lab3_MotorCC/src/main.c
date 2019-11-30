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
#include "driverlib/gpio.h"
#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include "UART_funcs.h"  //UART functions
#include "PWM_funcs.h"  //PWM functions 
#include "QEI_funcs.h"  //QEI functions 
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"


#define MSGQUEUE_OBJECTS      5 //quantidade de mensagens na fila

typedef struct{
  uint32_t RPM;
  uint8_t sentido;
}dados_motor;
  

osThreadId_t UART_thread_id, PWM_thread_id, QEI_thread_id, Control_thread_id;

osMessageQueueId_t SetPoint_msg;  
osMessageQueueId_t RPM_msg;
osMessageQueueId_t LeituraReal_msg;
osMessageQueueId_t DutyCycle_msg;

void UART_thread(void *arg)
{
  uint8_t str[12] = "LeituraReal:";
  uint8_t str2[2] = "\r\n";
  osStatus_t status;
  uint32_t RPM_to_send;
  uint8_t c = 0;
  uint32_t divisor;

  dados_motor set_point;
  uint8_t RPM_ASCII[5];
  uint16_t cont = 1;
  uint32_t intermediaryDigit = 0;

  while(1)
  {
    while(UART_char_available())                                
    {
      intermediaryDigit = UART_get_byte();
      if(intermediaryDigit != '\n')                                 
      {
        if( (intermediaryDigit) >= '0' && (intermediaryDigit) <= '9')
        {
          intermediaryDigit -= 0x30;
          
          set_point.RPM *= cont;
          set_point.RPM += intermediaryDigit;
          cont *= 10;
        }
        else
          set_point.sentido = intermediaryDigit;
      }
      else                                                      
      {
        osMessageQueuePut(SetPoint_msg, &set_point, 0, NULL);
        set_point.RPM = 0;
        cont = 1;
      }  
    }
    
    status = osMessageQueueGet(LeituraReal_msg , &RPM_to_send, NULL, 1000);  // wait for message
    if (status == osOK) {
      
        divisor = 10000;
        for(c = 0; c < 5; c++)
        {
          RPM_ASCII[c] = (RPM_to_send/divisor);
          RPM_to_send %= divisor;
          divisor /= 10;
        }
        for(c = 0; c < 5; c++)
        {
          RPM_ASCII[c] += 0x30;
        }
        //UART_send_byte(str, 12);
        UART_send_byte(RPM_ASCII, 5);
        UART_send_byte(str2,2);
    }
  }
}

void PWM_thread(void *arg)
{
  uint16_t duty_cycle;
  osStatus_t status;
  while(1) {
    status = osMessageQueueGet(DutyCycle_msg, &duty_cycle, NULL, 1000);  // wait for message
    if (status == osOK) {
      
        PWM_set_duty(duty_cycle);
    }
  } 
}

void QEI_thread(void *arg)
{
  uint32_t tick;
  uint32_t RPS;
  uint32_t RPM;
  while(1) 
  {  
      tick = osKernelGetTickCount();
      RPS = (uint32_t)QEIVelocityGet(QEI0_BASE);
      RPM = (uint32_t)RPS*60/(18*4);
      osMessageQueuePut(RPM_msg, &RPM, 0, NULL);
      
      osDelayUntil(tick + 1000);
  } 
}

void Control_thread(void *arg)
{
  dados_motor parametros;
  osStatus_t status;
  uint32_t RPM;
  uint32_t contador = 0;
  while(1) 
  {  
    //Ler Setpoint
    status = osMessageQueueGet(SetPoint_msg, &parametros, NULL, 1000);  // wait for message
    if (status == osOK) 
    {
      if(parametros.sentido == 'A')
      {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //freio
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);     
        osDelay(1000);     
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //gira
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);
      }
      else if (parametros.sentido == 'H')
      {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //freio
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);     
        osDelay(1000); 
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
      }
      
    //Escrever Duty
    //TODO: falta transformar parametros.RPM em Duty
    osMessageQueuePut(DutyCycle_msg, &parametros.RPM, 0, NULL);
    }

    //ler QEI
    status = osMessageQueueGet(RPM_msg, &RPM, NULL, 1000);  // wait for message
    if (status == osOK) 
    {
      contador++;
      //Escrever LeituraReal
      osMessageQueuePut(LeituraReal_msg, &RPM, 0, NULL);
      
      //Aplica controle
      if(RPM < parametros.RPM - 300 && contador == 10) //LeituraReal < SetPoint
      {
        contador = 0;
        //somar constante no RPM
      }
    }
  } 
}

void main(void){
  SystemInit();
  UART_init();
  PWM_init();
  QEI_init();

  osKernelInitialize();
  
  UART_thread_id = osThreadNew(UART_thread, NULL, NULL);
  PWM_thread_id = osThreadNew(PWM_thread, NULL, NULL);
  QEI_thread_id = osThreadNew(QEI_thread, NULL, NULL);
  Control_thread_id = osThreadNew(Control_thread, NULL, NULL);
  
  SetPoint_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(dados_motor), NULL);
  RPM_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint32_t), NULL);
  LeituraReal_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint32_t), NULL);
  DutyCycle_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(uint32_t), NULL);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1)
  {
    
  }
} // main
