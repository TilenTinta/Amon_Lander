Amon Lander - Flight controller

Flight controler (MCU only version). It is 4-layer PCB with:<br />
- Grounged mounting holes
- Two XT60 power connectors
- Voltage measuring: board power and EDF battery voltage
- Input protection: reverse polarity protection - diode
- Max. input voltage: 12V (tested)
- Two buck conterters: 3V3 (MCU, sensors...) and 5V (servos) - both are capable of ~5A
- Board can be turned on when battery is connected or with switch wired to terminal 
- Sensors: Gyroscope (MPU6050), Temperature/Pressure/Humidity (BME280), TOF (TOF400C-VL53L1X) connector, Compas (HMC5883L)
- Radios: 2x NRF24L01+PA/LNA (RX-TX)
- GPS: GY-NEO6MV2 connector [same connector used for drone calibration - process described in firmware]
- On board RGB LED for status indication
- PWM: 4x servo, 1x EDF (can be configured differently)
- Additional external connectors for SPI and I2C devices
- MCU: STM32F405RGT6-LQFP64
- External flash for saving telemetry data
- SD card holder (additional data storing device)
- 10-pin JTAG programming connector
- USB OTG with surge protection

*PCB works but it will be replaced/upgraded to a newer version.

![alt text](https://github.com/TilenTinta/Amon_Lander/blob/main/Pictures/PCB/PCB_FB1.PNG)

