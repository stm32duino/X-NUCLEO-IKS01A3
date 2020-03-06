/**
 ******************************************************************************
 * @file    X_NUCLEO_IKS01A3_LSM6DSOX_MLC.ino
 * @author  SRA
 * @version V1.1.0
 * @date    March 2020
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-IKS01A3
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
//NOTE: This example isn't compatible with Arduino Uno.
//NOTE: For this example you need the STEVAL-MKI197V1 board connected to the DIL24 connector of the X-NUCLEO-IKS01A3

// Includes
#include "LSM6DSOXSensor.h"
#include "lsm6dsox_activity_recognition_for_mobile.h"

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define INT_1 A5

//Interrupts.
volatile int mems_event = 0;

// Components
LSM6DSOXSensor *AccGyr;

// MLC
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

void INT1Event_cb();

void setup() {
  uint8_t mlc_out[8];
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  
  // Initialize I2C bus.
  DEV_I2C.begin();

  
  AccGyr = new LSM6DSOXSensor (&DEV_I2C, LSM6DSOX_I2C_ADD_L);
  AccGyr->Enable_X();
  AccGyr->Enable_G();

  /* Feed the program to Machine Learning Core */
  /* Activity Recognition Default program */  
  ProgramPointer = (ucf_line_t *)lsm6dsox_activity_recognition_for_mobile;
  TotalNumberOfLine = sizeof(lsm6dsox_activity_recognition_for_mobile) / sizeof(ucf_line_t);
  SerialPort.println("Activity Recognition for LSM6DSOX MLC");
  SerialPort.print("UCF Number Line=");
  SerialPort.println(TotalNumberOfLine);

  for (LineCounter=0; LineCounter<TotalNumberOfLine; LineCounter++) {
    if(AccGyr->Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
      SerialPort.print("Error loading the Program to LSM6DSOX at line: ");
      SerialPort.println(LineCounter);
      while(1) {
        // Led blinking.
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
      }
    }
  }

  SerialPort.println("Program loaded inside the LSM6DSOX MLC");

  //Interrupts.
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  /* We need to wait for a time window before having the first MLC status */
  delay(3000);

  AccGyr->Get_MLC_Output(mlc_out);
  switch(mlc_out[0]) {
    case 0:
      SerialPort.println("Activity: Stationary");
      break;
    case 1:
      SerialPort.println("Activity: Walking");
      break;
    case 4:
      SerialPort.println("Activity: Jogging");
      break;
    case 8:
      SerialPort.println("Activity: Biking");
      break;
    case 12:
      SerialPort.println("Activity: Driving");
      break;
    default:
      SerialPort.println("Activity: Unknown");
      break;
  }
}

void loop() {
  if (mems_event) {
    mems_event=0;
    LSM6DSOX_MLC_Status_t status;
    AccGyr->Get_MLC_Status(&status);
    if (status.is_mlc1) {
      uint8_t mlc_out[8];
      AccGyr->Get_MLC_Output(mlc_out);
      switch(mlc_out[0]) {
        case 0:
          SerialPort.println("Activity: Stationary");
          break;
        case 1:
          SerialPort.println("Activity: Walking");
          break;
        case 4:
          SerialPort.println("Activity: Jogging");
          break;
        case 8:
          SerialPort.println("Activity: Biking");
          break;
        case 12:
          SerialPort.println("Activity: Driving");
          break;
        default:
          SerialPort.println("Activity: Unknown");
          break;
      }	  
    }
  }
}

void INT1Event_cb() {
  mems_event = 1;
}
