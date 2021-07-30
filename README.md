# Rodan
The 34.5 mm x 34.5 mm pcb is designed to mount into a blue transparent Hammond 1551-Q box. The STM32WB5MMG module with integrated pcb antenna is lower left. Programming is via USB (or tag connect) and the Arduino IDE using Thomas Roell's excellent [STM32WB system layer](https://github.com/GrumpyOldPizza/ArduinoCore-stm32wb) and Arduino wrapper.

The board can be powered via USB and there is a switch-enabled 300 mA MCP1812 LDO for the purpose The board can also be powered by the AEM10941 HVOUT (3V3, 80 mA) when there is an AM-5412CAR solar cell source and 100 mAH LiPo battery attached. The idea is to program the board via USB (my preference) and then deploy outside with just solar cell and battery with the sensors collecting and storing data on the 8 MByte  MX25R6435FZAI QSPI flash for ~six months or so.

The sensors include the VEML6040 RGBW light sensor, LPS22HB barometer, HDC2010 humidity and temperature sensor, BMA400 accelerometer for motion detection, and the CCS811 for eCO2 and TVOC estimation. There is also an LC709204 battery fuel gauge for battery voltage and percent remaining charge monitoring, etc..  

![Rodan](https://user-images.githubusercontent.com/6698410/127719506-94157222-1675-4999-b114-421aa7fc567d.jpg)

![Rodanplus](https://user-images.githubusercontent.com/6698410/127720557-4324a0b5-17c7-46a0-829b-7c1a68d71a95.jpg)

See [Hackaday](https://hackaday.io/project/19649-stm32l4-sensor-tile/log/195996-same-idea-different-implementation) for more.
Pcb [design](https://oshpark.com/shared_projects/23LSLZPF) is open-source and available on OSH Park shared space.
