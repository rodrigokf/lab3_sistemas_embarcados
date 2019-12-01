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

#define kp  0.1
#define ki  0.03
#define kd  0

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
  osStatus_t status;
  uint32_t RPM_to_send;
  uint8_t c = 0;
  uint32_t divisor;

  dados_motor set_point;
  uint32_t set_point_anterior;
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
        {
          set_point.RPM = set_point_anterior;
          set_point.sentido = intermediaryDigit;
        }
      }
      else                                                      
      {
        osMessageQueuePut(SetPoint_msg, &set_point, 0, NULL);
        set_point_anterior = set_point.RPM;
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
        UART_send_byte("LeituraReal: ", 13);
        UART_send_byte(RPM_ASCII, 5);
        UART_send_byte("\n\r",2);
    }
  }
}

void PWM_thread(void *arg)
{
  uint32_t duty_cycle;
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
  uint8_t sentido_anterior;

  float pv_speed = 0;
  float set_speed = 0;
  float e_speed = 0; //error of speed = set_speed - pv_speed
  float e_speed_pre = 0;  //last error of speed
  float e_speed_sum = 0;  //sum error of speed
  float pwm_pulse = 0;     //this value is 0~255
  
  while(1) 
  {  
    //Ler Setpoint
    status = osMessageQueueGet(SetPoint_msg, &parametros, NULL, 500);  // wait for message
    if (status == osOK) 
    {
      if(parametros.sentido == 'A' && sentido_anterior == 'H')
      {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //freio
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);     
        osDelay(500);     
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //gira
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0);
        
        sentido_anterior = 'A';
      }
      else if (parametros.sentido == 'H' && sentido_anterior == 'A')
      {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);    //freio
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);     
        osDelay(500); 
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0);
        sentido_anterior = 'H';
      }
      
    //Escrever Duty
    //TODO: falta transformar parametros.RPM em Duty
    set_speed = parametros.RPM;
      
    e_speed = set_speed - pv_speed;
    pwm_pulse = (float)e_speed*kp + (float)e_speed_sum*ki + (float)(e_speed - e_speed_pre)*kd;
    e_speed_pre = e_speed;  //save last (previous) error
    e_speed_sum += e_speed; //sum of error
    
    if (e_speed_sum >10000) 
      e_speed_sum = 10000;
    if (e_speed_sum <-10000) 
      e_speed_sum = -10000;
    
    if(pwm_pulse > 1000)
      pwm_pulse = 1000;
    if(pwm_pulse < 0)
      pwm_pulse = 0;
    
    osMessageQueuePut(DutyCycle_msg, &pwm_pulse, 0, NULL);
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
  DutyCycle_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(float), NULL);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1)
  {
    
  }
} // main
