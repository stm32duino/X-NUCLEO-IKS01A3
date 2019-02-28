/**
 ******************************************************************************
 * @file    X_NUCLEO_IKS01A3_STTS751_TemperatureLimit.ino
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Arduino test application for the STMicrolectronics
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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
//NOTE: this example isn't compatible with Arduino Uno


#include <STTS751Sensor.h>

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

#define INT_1 A4

//Interrupts.
volatile int mems_event = 0;

uint8_t high = 0, low = 0;
uint32_t previous_tick;
float temperature = 0;

STTS751Sensor *Temp;

void INT1Event_cb()
{
  mems_event = 1;
}

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();
  
  //Interrupts.
  attachInterrupt(INT_1, INT1Event_cb, FALLING);

  Temp = new STTS751Sensor(&DEV_I2C);
  Temp->Enable();
  Temp->SetOutputDataRate(4.0f);
  Temp->SetLowTemperatureThreshold(22.0f);
  Temp->SetHighTemperatureThreshold(28.0f);
  Temp->SetEventPin(1);
  Temp->GetTemperatureLimitStatus(NULL, NULL, NULL);

  previous_tick=millis();
}

void loop() {
  if (mems_event)
  {
    mems_event=0;
    uint8_t highTemp = 0, lowTemp = 0;
    Temp->GetTemperatureLimitStatus(&highTemp, &lowTemp, NULL);
    if (highTemp){
      high = 1;
      low = 0;
    }
    if (lowTemp){
      low = 1;
      high = 0;
    }
    
    Temp->GetTemperature(&temperature);
    // Led blinking.
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);   
  }
  uint32_t current_tick = millis();
  if ((current_tick - previous_tick) >= 2000){
    if (!high && !low){
      Temp->GetTemperature(&temperature);
    }
    SerialPort.print("Temp[C]: ");
    SerialPort.print(temperature, 2);
    if (high){
      SerialPort.println(" High temperature detected!(>28C) ");
      high = 0;
    } else if (low) {
      SerialPort.println(" Low temperature detected!(<22C) ");
      low = 0;
    } else {
      SerialPort.println();
    }
    previous_tick = millis();
  }
}
