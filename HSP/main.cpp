/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#include "mbed.h"
#include "MAX14720.h"
#include "MAX30101.h"
#include "MAX30205.h"
#include "LIS2DH.h"
#include "USBSerial.h"
#include "RpcServer.h"
#include "StringInOut.h"
#include "Peripherals.h"
#include "BMP280.h"
#include "MAX30001.h"
#include "DataLoggingService.h"
#include "MAX30101_helper.h"
#include "S25FS512.h"
#include "QuadSpiInterface.h"
#include "PushButton.h"
#include "BLE.h"
#include "HspBLE.h"
#include "USBSerial.h"
#include "Streaming.h"

/// define the HVOUT Boost Voltage default for the MAX14720 PMIC
#define HVOUT_VOLTAGE 4500 // set to 4500 mV

/// define all I2C addresses
#define MAX30205_I2C_SLAVE_ADDR_TOP (0x92)
#define MAX30205_I2C_SLAVE_ADDR_BOTTOM (0x90)
#define MAX14720_I2C_SLAVE_ADDR (0x54)
#define BMP280_I2C_SLAVE_ADDR (0xEC)
#define MAX30101_I2C_SLAVE_ADDR (0xAE)
#define LIS2DH_I2C_SLAVE_ADDR (0x32)

///
/// wire Interfaces
///
/// Define with Maxim VID and a Maxim assigned PID, set to version 0x0001 and non-blocking
USBSerial usbSerial(0x0b6a, 0x0100, 0x0001, false);
/// I2C Master 1
I2C i2c1(I2C1_SDA, I2C1_SCL); // used by MAX30205 (1), MAX30205 (2), BMP280
/// I2C Master 2
I2C i2c2(I2C2_SDA, I2C2_SCL); // used by MAX14720, MAX30101, LIS2DH
/// SPI Master 0 with SPI0_SS for use with MAX30001
SPI spi(SPI0_MOSI, SPI0_MISO, SPI0_SCK, SPI0_SS); // used by MAX30001
/// SPI Master 1
QuadSpiInterface quadSpiInterface(SPI1_MOSI, SPI1_MISO, SPI1_SCK,
                                  SPI1_SS); // used by S25FS512

///
/// Devices
///
/// Pressure Sensor
BMP280 bmp280(&i2c1, BMP280_I2C_SLAVE_ADDR);
/// Top Local Temperature Sensor
MAX30205 MAX30205_top(&i2c1, MAX30205_I2C_SLAVE_ADDR_TOP);
/// Bottom Local Temperature Sensor
MAX30205 MAX30205_bottom(&i2c1, MAX30205_I2C_SLAVE_ADDR_BOTTOM);
/// Accelerometer
LIS2DH lis2dh(&i2c2, LIS2DH_I2C_SLAVE_ADDR);
InterruptIn lis2dh_Interrupt(P4_7);
/// PMIC
MAX14720 max14720(&i2c2, MAX14720_I2C_SLAVE_ADDR);
/// Optical Oximeter
MAX30101 max30101(&i2c2, MAX30101_I2C_SLAVE_ADDR);
InterruptIn max30101_Interrupt(P4_0);
/// External Flash
S25FS512 s25fs512(&quadSpiInterface);
/// ECG device
MAX30001 max30001(&spi);
InterruptIn max30001_InterruptB(P3_6);
InterruptIn max30001_Interrupt2B(P4_5);
/// PWM used as fclk for the MAX30001
PwmOut pwmout(P1_7);
/// HSP platform LED
HspLed hspLed(LED_RED);
/// Packet TimeStamp Timer, set for 1uS
Timer timestampTimer;
/// HSP Platform push button
PushButton pushButton(SW1);

/// BLE instance
static BLE ble;

/// HSP BluetoothLE specific functions
HspBLE hspBLE(&ble);
USBSerial *usbSerialPtr;

int main() {
  // hold results for returning funtcoins
  int result;
  // local input state of the RPC
  int inputState;
  // RPC request buffer
  char request[128];
  // RPC reply buffer
  char reply[128];

  // display start banner
  printf("Maxim Integrated mbed hSensor 3.0.0 10/14/16\n");
  fflush(stdout);

  // initialize HVOUT on the MAX14720 PMIC
  printf("Init MAX14720...\n");
  fflush(stdout);
  result = max14720.init();
  if (result == MAX14720_ERROR)
    printf("Error initializing MAX14720");
  max14720.boostEn = MAX14720::BOOST_ENABLED;
  max14720.boostSetVoltage(HVOUT_VOLTAGE);

  // turn on red led
  printf("Init HSPLED...\n");
  fflush(stdout);
  hspLed.on();

  // set NVIC priorities for GPIO to prevent priority inversion
  printf("Init NVIC Priorities...\n");
  fflush(stdout);
  NVIC_SetPriority(GPIO_P0_IRQn, 5);
  NVIC_SetPriority(GPIO_P1_IRQn, 5);
  NVIC_SetPriority(GPIO_P2_IRQn, 5);
  NVIC_SetPriority(GPIO_P3_IRQn, 5);
  NVIC_SetPriority(GPIO_P4_IRQn, 5);
  NVIC_SetPriority(GPIO_P5_IRQn, 5);
  NVIC_SetPriority(GPIO_P6_IRQn, 5);
  // used by the MAX30001
  NVIC_SetPriority(SPI1_IRQn, 0);

  // Be able to statically reference these devices anywhere in the application
  Peripherals::setS25FS512(&s25fs512);
  Peripherals::setMAX30205_top(&MAX30205_top);
  Peripherals::setMAX30205_bottom(&MAX30205_bottom);
  Peripherals::setBMP280(&bmp280);
  Peripherals::setLIS2DH(&lis2dh);
  //Peripherals::setUSBSerial(&usbSerial);
  Peripherals::setTimestampTimer(&timestampTimer);
  Peripherals::setHspLed(&hspLed);
  Peripherals::setMAX30101(&max30101);
  Peripherals::setI2c1(&i2c1);
  Peripherals::setI2c2(&i2c2);
  Peripherals::setPushButton(&pushButton);
  Peripherals::setBLE(&ble);
  Peripherals::setMAX14720(&max14720);
  Peripherals::setMAX30001(&max30001);
  Peripherals::setHspBLE(&hspBLE);
  usbSerialPtr = &usbSerial;

  // init the S25FS256 external flash device
  printf("Init S25FS512...\n");
  fflush(stdout);
  s25fs512.init();

  // init the BMP280
  printf("Init BMP280...\n");
  fflush(stdout);
  bmp280.init(BMP280::OVERSAMPLING_X2_P, BMP280::OVERSAMPLING_X1_T,
              BMP280::FILT_OFF, BMP280::NORMAL_MODE, BMP280::T_62_5);

  // Initialize BLE base layer
  printf("Init HSPBLE...\n");
  fflush(stdout);
  hspBLE.init();

  // start blinking led1
  printf("Init HSPLED Blink...\n");
  fflush(stdout);
  hspLed.blink(1000);

  // MAX30101 initialize interrupt
  printf("Init MAX30101 callbacks, interrupt...\n");
  fflush(stdout);
  max30101.onInterrupt(&MAX30101_OnInterrupt);
  max30101.onDataAvailable(&StreamPacketUint32);
  max30101_Interrupt.fall(&MAX30101::MidIntHandler);

  //
  // MAX30001
  //
  printf("Init MAX30001 callbacks, interrupts...\n");
  fflush(stdout);
  max30001_InterruptB.disable_irq();
  max30001_Interrupt2B.disable_irq();
  max30001_InterruptB.mode(PullUp);
  max30001_InterruptB.fall(&MAX30001::Mid_IntB_Handler);
  max30001_Interrupt2B.mode(PullUp);
  max30001_Interrupt2B.fall(&MAX30001::Mid_Int2B_Handler);
  max30001_InterruptB.enable_irq();
  max30001_Interrupt2B.enable_irq();
  max30001.AllowInterrupts(1);
  // Configuring the FCLK for the ECG, set to 32.768KHZ
  printf("Init MAX30001 PWM...\n");
  fflush(stdout);
  //pwmout.period_us(31);      
  //pwmout.write(0.5);          // 0-1 is 0-100%, 0.5 = 50% duty cycle.
  max30001.FCLK_MaximOnly();    // mbed does not provide the resolution necessary, so for now we have a specific solution...
  max30001.sw_rst(); // Do a software reset of the MAX30001
  max30001.INT_assignment(MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_enint_loc,      en_eovf_loc,   en_fstint_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_dcloffint_loc,  en_bint_loc,   en_bovf_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_NO_INT,  //  en_bover_loc,      en_bundr_loc,  en_bcgmon_loc,
                                     MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,   MAX30001::MAX30001_NO_INT,  //  en_pint_loc,       en_povf_loc,   en_pedge_loc,
                                     MAX30001::MAX30001_INT_2B,   MAX30001::MAX30001_INT_B,    MAX30001::MAX30001_NO_INT,  //  en_lonint_loc,     en_rrint_loc,  en_samp_loc,
                                     MAX30001::MAX30001_INT_ODNR, MAX30001::MAX30001_INT_ODNR);                            //  intb_Type,         int2b_Type)
  max30001.onDataAvailable(&StreamPacketUint32);

  // initialize the LIS2DH accelerometer and interrupts
  printf("Init LIS2DH interrupt...\n");
  fflush(stdout);
  lis2dh.init();
  lis2dh_Interrupt.fall(&LIS2DHIntHandler);
  lis2dh_Interrupt.mode(PullUp);
  // initialize the RPC server
  printf("Init RPC Server...\n");
  fflush(stdout);
  RPC_init();
  // initialize the logging service
  printf("Init LoggingService...\n");
  fflush(stdout);
  LoggingService_Init();

  // start main loop
  printf("Start main loop...\n");
  fflush(stdout);
  while (1) {
    // get a RPC string if one is available
    inputState = getLine(request, sizeof(request));
    // if a string has been captured, process string
    if (inputState == GETLINE_DONE) {
      // process the RPC string
      RPC_call(request, reply);
      // output the reply
      putStr(reply);
    }
    // process any logging or streaming requests
    LoggingService_ServiceRoutine();
    // determine if we are doing a large USB transfer of flash datalog sensor data 
    //   skip updating the ble and sleeping, this conditional branch increases download
    //   rates by about %450 (time to download 32M flash from 90 minutes to about 20 minutes)
    if (RPC_IsTransferingFlashPages() == false) { 
      // allow for ble processing
      ble.waitForEvent();
    }
  }
}
