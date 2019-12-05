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

#define kp  0.7
#define ki  0.005
#define kd  0.0

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
  dados_motor data_to_send;
  uint8_t c = 0;
  uint32_t divisor;

  dados_motor set_point;
  uint32_t set_point_anterior;
  uint8_t RPM_ASCII[6];
  uint8_t RPM_Setpoint[6] = {"00000"};
  uint16_t cont = 1;
  uint16_t vector_index = 0;
  uint32_t intermediaryDigit = 0;
  
  set_point_anterior = set_point.RPM = 0000;
  set_point_anterior = set_point.sentido = 'H';

   set_point.sentido = 'A';

  while(1)
  {
    while(UART_char_available())                                
    {      
      intermediaryDigit = UART_get_byte();
        
      if(intermediaryDigit != '\n')                                 
      {
        if( (intermediaryDigit) >= '0' && (intermediaryDigit) <= '9')
        {
                            
          if(cont == 1)
          {
            set_point.RPM = 0;
            for(vector_index = 0; vector_index < 6; vector_index++)
              RPM_Setpoint[vector_index] = 0;
            
            vector_index = 0;
          }
          
          RPM_Setpoint[vector_index] = intermediaryDigit;
          vector_index++;
          
          intermediaryDigit -= 0x30;
          
          set_point.RPM *= cont;
          set_point.RPM += intermediaryDigit;
          cont = 10;
        }
        else if( intermediaryDigit == 'A' || intermediaryDigit == 'H')
        {
          set_point.RPM = set_point_anterior;
          set_point.sentido = intermediaryDigit;
        }
      }
      else                                                      
      {  
        set_point_anterior = set_point.RPM;
        cont = 1;
      }  
    }
    osMessageQueuePut(SetPoint_msg, &set_point, 0, NULL);
    
    status = osMessageQueueGet(LeituraReal_msg , &data_to_send, NULL, 100);  // wait for message
    if (status == osOK) {
      
        divisor = 10000;
        for(c = 0; c < 5; c++)
        {
          RPM_ASCII[c] = (data_to_send.RPM/divisor);
          data_to_send.RPM %= divisor;
          divisor /= 10;
        }
        for(c = 0; c < 5; c++)
        {
          RPM_ASCII[c] += 0x30;
        }
        
        UART_send_byte("SetPoint: ", 10);
        osDelay(1);
        UART_send_byte(RPM_Setpoint, vector_index);
        osDelay(1);
        UART_send_byte("\r",1);
        osDelay(1);
        UART_send_byte("Sentido: ", 9);
        osDelay(1);
        UART_send_byte(&set_point.sentido, 1);      
        osDelay(1);
        UART_send_byte("\r",1);
        osDelay(1);
        UART_send_byte("RPM Real: ", 10);
        osDelay(1);
        UART_send_byte(RPM_ASCII, 5);
        osDelay(1);
        UART_send_byte("\r",1);
        UART_send_byte("Sentido Real: ", 14);
        osDelay(1);
        UART_send_byte(&data_to_send.sentido, 1);
        osDelay(1);
        UART_send_byte("\r",1);
        osDelay(500);
    }
  }
}

void PWM_thread(void *arg)
{
  float duty_cycle;
  osStatus_t status;
  while(1) {
    status = osMessageQueueGet(DutyCycle_msg, &duty_cycle, NULL, 1);  // wait for message
    if (status == osOK) {
      
        PWM_set_duty(duty_cycle);
    }
  } 
}

void QEI_thread(void *arg)
{
  dados_motor leitura_qei;
  uint32_t tick;
  uint32_t RPS;
  while(1) 
  {  
      tick = osKernelGetTickCount();
      RPS = (uint32_t)QEIVelocityGet(QEI0_BASE);
      leitura_qei.RPM = (uint32_t)RPS*60/(18*4);
      
      if(QEIDirectionGet(QEI0_BASE) == 1)
        leitura_qei.sentido = 'A';
      
      else if(QEIDirectionGet(QEI0_BASE) == -1) 
        leitura_qei.sentido = 'H';
      
      else
         leitura_qei.sentido = 'N';
         
      osMessageQueuePut(RPM_msg, &leitura_qei, 0, NULL);
      
      osDelayUntil(tick + 10);
  } 
}

void Control_thread(void *arg)
{  
  dados_motor parametros;
  osStatus_t status;
  dados_motor dados_qei;
  uint8_t sentido_anterior = 'H';
  //uint32_t RPM;
  int32_t pv_speed = 0;           //process variable
  int32_t set_speed = 0;          //set point
  int32_t e_speed = 0;            //error of speed = set_speed - pv_speed
  int32_t e_speed_pre = 0;        //last error of speed
  int32_t e_speed_sum = 0;        //sum error of speed
  float pwm_pulse = 0;          //this value is 0~1000
  
  while(1) 
  {  
    
    //Ler Setpoint
    status = osMessageQueueGet(SetPoint_msg, &parametros, NULL, NULL);  // se tiver nova msg, atualiza os valores
    if (status == osOK) 
    {
      if(parametros.sentido == 'A' && sentido_anterior  == 'H')
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
    }
            //ler QEI
    status = osMessageQueueGet(RPM_msg, &dados_qei, NULL, osWaitForever);  // wait for message
    if (status == osOK) 
    {
      //envia os dados reais
      osMessageQueuePut(LeituraReal_msg, &dados_qei, 0, NULL);

    }

    set_speed = (int32_t)(parametros.RPM * 1.0);
    
    
    pv_speed = dados_qei.RPM;  
    e_speed = set_speed - pv_speed;
    
    pwm_pulse = (float)e_speed*kp; 
    pwm_pulse +=(float)e_speed_sum*ki;
    pwm_pulse +=(float)(e_speed - e_speed_pre)*kd;
    
    e_speed_pre = e_speed;  //save last (previous) error
    e_speed_sum += e_speed; //sum of error
    
    if (e_speed_sum >10000000) 
      e_speed_sum = 10000000;
    if (e_speed_sum <-10000000) 
      e_speed_sum = -10000000;
   
    
    pwm_pulse /= 100;
    
    if(pwm_pulse > 99)
      pwm_pulse = 99;
    if(pwm_pulse < 0.0)
      pwm_pulse = 0.0;
    
    //PWM_set_duty(pwm_pulse);
    osMessageQueuePut(DutyCycle_msg, &pwm_pulse, 0, NULL);
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
  RPM_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(dados_motor), NULL);
  LeituraReal_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(dados_motor), NULL);
  DutyCycle_msg = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(float), NULL);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1)
  {
    
  }
} // main
