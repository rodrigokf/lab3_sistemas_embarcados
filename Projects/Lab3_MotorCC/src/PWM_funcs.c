#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "PWM_funcs.h"

void PWM_init()
{
  //Set GPIO IN3(PB2) e IN4(PB3)
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2 | GPIO_PIN_3);
  
    //
    // Set the PWM clock to the system clock.
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //
    // The PWM peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // For this example PWM0 is used with PortF Pin1.  The actual port
    // and pins used may be different on your part, consult the data sheet for
    // more information.  GPIO port B needs to be enabled so these pins can be
    // used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIO pin muxing to select PWM functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PF2_M0PWM2);

    //
    // Configure the GPIO pad for PWM function on pin PF1.  Consult
    // the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the PWM0 to count up/down without synchronization.
    // Note: Enabling the dead-band generator automatically couples the 2
    // outputs from the PWM block so we don't use the PWM synchronization.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 1kHz) * 250MHz = 250000 cycles.  Note that
    // the maximum period you can set is 2^16 - 1.
    // TODO: modify this calculation to use the clock frequency that you are
    // using.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 250000);

    //
    // Set PWM0 PD0 to a duty cycle of 25%.  You set the duty cycle as a
    // function of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or 16000 clock cycles (64000 / 4).
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     (uint32_t)(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)*0.5));

    //
    // Enables the counter for a PWM generator block.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
  
}

void PWM_set_duty(uint16_t duty)
{
  
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     (uint32_t)(PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1)*(float)duty/100));
}