#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/qei.h"
#include "driverlib/sysctl.h"
#include "QEI_funcs.h"

void QEI_init()
{
      //QEI1 = PC5 e PC6
      // Enable QEI Peripherals
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
      SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
      
      //Set Pins to be PHA0 and PHB0
      GPIOPinConfigure(GPIO_PL1_PHA0);
      GPIOPinConfigure(GPIO_PL2_PHB0);
      
      //Set GPIO pins for QEI. PhA0 -> PC5, PhB0 ->PC6. I believe this sets the pull up and makes them inputs
      GPIOPinTypeQEI(GPIO_PORTL_BASE, GPIO_PIN_1 |  GPIO_PIN_2);
      
      //DISable peripheral and int before configuration
      QEIDisable(QEI0_BASE);
      QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
      
      // Configure quadrature encoder, use an arbitrary top limit of 1000
      QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET 	| QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1);
      
      // Enable the quadrature encoder.
      QEIEnable(QEI0_BASE);
      
      //Set position to a middle value so we can see if things are working
      QEIPositionSet(QEI0_BASE, 0);
      
      QEIVelocityDisable(QEI0_BASE);
      
      QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, 120000000);
      
      QEIVelocityEnable(QEI0_BASE);
}