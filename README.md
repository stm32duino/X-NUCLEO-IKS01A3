# X-NUCLEO-IKS01A3

The X-NUCLEO-IKS01A3 is a motion MEMS and environmental sensor expansion board for the STM32 Nucleo.
It is equipped with Arduino UNO R3 connector layout, and is designed around the LSM6DSO 3D accelerometer and 3D gyroscope, 
the LIS2DW12 3D accelerometer, the LIS2MDL 3D magnetometer, the HTS221 humidity and temperature sensor, the LPS22HH 
pressure and temperature sensor and the STTS751 temperature sensor.
The X-NUCLEO-IKS01A3 interfaces with the STM32 microcontroller or the Arduino boards via the I²C pin.

# Examples

There are several examples with the X-NUCLEO-IKS01A3 library.
* X_NUCLEO_IKS01A3_HelloWorld: This application provides a simple example of usage of the X-NUCLEO-IKS01A3 
Expansion Board. It shows how to display on a hyperterminal the values of all on-board MEMS and environmental sensors.
* X_NUCLEO_IKS01A3_LIS2DW12_6DOrientation: This application shows how to use X-NUCLEO-IKS01A3 LIS2DW12 accelerometer 
to find out the 6D orientation and display data on a hyperterminal.
* X_NUCLEO_IKS01A3_LIS2DW12_WakeUpDetection: This application shows how to detect the wake-up event using the 
X-NUCLEO-IKS01A3 LIS2DW12 accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_6DOrientation: This application shows how to use X-NUCLEO-IKS01A3 LSM6DSO accelerometer 
to find out the 6D orientation and display data on a hyperterminal.
* X_NUCLEO_IKS01A3_LSM6DSO_FreeFallDetection: This application shows how to detect the free fall event using the 
X-NUCLEO-IKS01A3 LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_MultiEvent: This application shows how to detect free fall, tap, double tap, tilt, wake up,
6D Orientation and step events using the X-NUCLEO-IKS01A3 LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_Pedometer: This application shows how to use X-NUCLEO-IKS01A3 LSM6DSO accelerometer 
to count steps.
* X_NUCLEO_IKS01A3_LSM6DSO_SingleTap: This application shows how to detect the single tap event using the 
X-NUCLEO-IKS01A3 LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_DoubleTap: This application shows how to detect the double tap event using the 
X-NUCLEO-IKS01A3 LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_TiltDetection: This application shows how to detect the tilt event using the X-NUCLEO-IKS01A3 
LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_LSM6DSO_WakeUpDetection: This application shows how to detect the wake-up event using the 
X-NUCLEO-IKS01A3 LSM6DSO accelerometer.
* X_NUCLEO_IKS01A3_STTS751_TemeperatureLimit: This application shows how to detect low temperature and high temperature 
events using the X-NUCLEO-IKS01A3 STTS751 temperature sensor.

# Dependencies

The X-NUCLEO-IKS01A3 library requires the following STM32duino libraries:

* STM32duino LSM6DSO: https://github.com/stm32duino/LSM6DSO
* STM32duino LIS2DW12: https://github.com/stm32duino/LIS2DW12
* STM32duino LIS2MDL: https://github.com/stm32duino/LIS2MDL
* STM32duino HTS221: https://github.com/stm32duino/HTS221
* STM32duino LPS22HH: https://github.com/stm32duino/LPS22HH
* STM32duino STTS751: https://github.com/stm32duino/STTS751

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-IKS01A3

The X-NUCLEO-IKS01A3 datasheet is available at  
https://www.st.com/content/st_com/en/products/ecosystems/stm32-open-development-environment/stm32-nucleo-expansion-boards/stm32-ode-sense-hw/x-nucleo-iks01a3.html
