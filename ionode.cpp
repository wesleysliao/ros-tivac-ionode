/*********************************************************************
 *
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  rosserial_tivac chatter tutorial
 *
 *  On this demo your TivaC Launchpad will publish a string over the
 * topic "/chatter".
 *
 * Full guide: http://wiki.ros.org/rosserial_tivac/Tutorials
 *
 *********************************************************************/


#include <stdbool.h>
#include <stdint.h>
#include <string>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/pin_map.h>
  #include <driverlib/uart.h>

  #include <inc/hw_memmap.h>
  #include <inc/hw_types.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/UInt32.h>

void LCDclear()
{
  UARTCharPut(UART1_BASE, 0xFE);
  UARTCharPut(UART1_BASE, 0x01);
  return;
}

void LCDprint(std::string str){
  for (uint32_t i =0; i<str.length(); i++ )
  {
    UARTCharPut(UART1_BASE, str.at(i));
  }
}

void doutputs_handler(const std_msgs::UInt32& doutputs_msg)
{
    UARTCharPut(UART1_BASE, doutputs_msg.data);
}

// ROS nodehandle
ros::NodeHandle nh;

std_msgs::UInt32 dinputs_msg;
ros::Publisher dinputs("tivac_serial/dinputs", &dinputs_msg);

std_msgs::UInt32 doutputs_msg;
ros::Subscriber<std_msgs::UInt32> doutputs("tivac_serial/doutputs", &doutputs_handler);

int main(void)
{
  // TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  GPIOPinConfigure(GPIO_PB0_U1RX);
  GPIOPinConfigure(GPIO_PB1_U1TX);
  
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  LCDclear();
  LCDprint(" Tiva C IO Node  initializing...");
  
  // ROS nodehandle initialization and topic registration
  nh.initNode();
  nh.advertise(dinputs);
  nh.subscribe(doutputs);
  
  LCDprint(" Tiva C IO Node  waiting for ROS");

  bool nh_prev_state = false;
  while (1)
  {
    // If subscribed, enable RGB driver
    if (nh.connected() && !nh_prev_state)
    {
      LCDprint(" Tiva C IO Node  ROS connected  ");
      nh_prev_state = true;
    }
    if (!nh.connected() && nh_prev_state)
    {
      LCDprint(" Tiva C IO Node  waiting for ROS");
      nh_prev_state = false;
    }
    // Publish message to be transmitted.
    // str_msg.data = hello;
    // chatter.publish(&str_msg);

    // Handle all communications and callbacks.
    nh.spinOnce();

    // Delay for a bit.
    nh.getHardware()->delay(100);
  }
}